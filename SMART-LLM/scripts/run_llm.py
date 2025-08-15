import copy
import glob
import json
import os
import argparse
from pathlib import Path
from datetime import datetime
import random
import subprocess
import time
import re
from openai import OpenAI

import ai2thor.controller

import sys
sys.path.append(".")

import resources.actions as actions
import resources.robots as robots

# --- Compact, shared prompt snippets (English) ---

COMPACT_DECOMP = r"""
# === HUMAN-ASSIST (Decomposition, brief) ===
# Mark HUMAN only when ALL active robots lack the required skill,
# the object is too heavy, occluded/unreachable, or you need >2 concurrent actions (cap=2).
# For wash/clean/rinse subtasks, the plan MUST include:
# Put item into Sink -> SwitchOn Faucet -> CleanObject -> SwitchOff Faucet.
# Footer exactly:
# [HUMAN_ASSIST_CANDIDATES]
# - <reason> -> <1-2 steps>
# or: [HUMAN_ASSIST_CANDIDATES] none
"""


COMPACT_ALLOC = r"""
# === ALLOCATION POLICY (brief) ===
# Max 2 robots can act concurrently.
# Prefer robots whose skills match the subtask exactly.
# Use HUMAN only for: missing skill across all active robots, object too heavy,
# occluded/unreachable, or >2 concurrent needs.
# Prefer robots that have BOTH PickupObject and PutObject for container placement.
# If the task mentions wash/clean/rinse, prefer a robot with CleanObject; still MUST implement the strict sequence.
"""


COMPACT_CODE = r"""
# === CODE CONSTRAINTS (concise) ===
# Return only valid Python. Convert any markers to comments.
# Allowed atomic actions (NO DropHandObject):
# GoToObject, PickupObject, PutObject, OpenObject, CloseObject,
# SliceObject, SwitchOn, SwitchOff, BreakObject, CleanObject, ThrowObject, PushObject, PullObject.

# Concurrency cap: at most 2 robots act concurrently.
# Always call atomic actions with ONE robot (never a list).
ACTIVE_ROBOTS = robots[:min(2, len(robots))]
def pick_robot(candidates=ACTIVE_ROBOTS, prefer_skill=None):
    for r in candidates:
        if prefer_skill and prefer_skill in set(r.get("skills", [])): return r
    return candidates[0]

# Carrying / handoff:
# Prefer the same robot to continue operating on the same object.
# If switching robots: PutObject to a stable surface (CounterTop/Sink), then the next robot PickupObject.

# HUMAN(reason, steps): only when required. Operator whitelist: mv/open/close/put/slice/wash/break/throw/done

# === STRICT WASHING SEQUENCE (HARD REQUIREMENT) ===
# If the task text contains 'wash', 'clean', or 'rinse' (case-insensitive),
# you MUST implement this exact order for the target item ITEM (e.g., 'Lettuce') and MUST NOT reorder:
# 1) r = pick_robot(prefer_skill="PutObject")
# 2) GoToObject(r, "Sink")
# 3) PutObject(r, "ITEM", "Sink")      # item is inside sink before faucet on
# 4) SwitchOn(r, "Faucet")
# 5) CleanObject(r, "ITEM")            # do not skip; if robots lack skill, use HUMAN fallback
# 6) time.sleep(3)
# 7) SwitchOff(r, "Faucet")            # never switch off before CleanObject
# 8) PickupObject(r, "ITEM")
# Then continue (e.g., GoToObject(r, "CounterTop"); PutObject(r, "ITEM", "CounterTop")).
# NEVER call SwitchOn/SwitchOff on 'Faucet' outside this sequence in washing subtasks.
"""



def LM(prompt, gpt_version, max_tokens=128, temperature=0, stop=None, logprobs=1, frequency_penalty=0):
    
    if "gpt" not in gpt_version:
        response = client.chat.completions.create(model=gpt_version, 
                                            prompt=prompt, 
                                            max_tokens=max_tokens, 
                                            temperature=temperature, 
                                            stop=stop, 
                                            logprobs=logprobs, 
                                            frequency_penalty = frequency_penalty)
        
        text = response.choices[0].message.content.strip()
        return response, text
    
    else:
        response = client.chat.completions.create(model=gpt_version, 
                                            messages=prompt, 
                                            max_tokens=max_tokens, 
                                            temperature=temperature, 
                                            frequency_penalty = frequency_penalty)
        
        text = response.choices[0].message.content.strip()
        return response, text

def set_api_key(openai_api_key_file):
    key = Path(openai_api_key_file + '.txt').read_text().strip()
    os.environ["OPENAI_API_KEY"] = key


def get_ai2_thor_objects_compact(floor_plan_id, limit=60, dedup=True):
    controller = ai2thor.controller.Controller(scene="FloorPlan"+str(floor_plan_id), physics_enabled=False)
    objs = controller.last_event.metadata["objects"]
    seen = set()
    out = []
    for o in objs:
        name = o.get("objectType") or o.get("name")
        mass = o.get("mass", 1.0)
        if not name:
            continue
        if dedup and name in seen:
            continue
        seen.add(name)
        out.append({"name": name, "mass": mass})
        if len(out) >= limit:
            break
    controller.stop()
    return out

def to_compact_json(obj) -> str:
    return json.dumps(obj, ensure_ascii=False, separators=(",", ":"))

def truncate_text(s: str, max_chars: int) -> str:
    return (s[:max_chars] + "\n# [TRUNCATED FOR TOKEN BUDGET]\n") if len(s) > max_chars else s

# Function returns object list with name and properties.
def convert_to_dict_objprop(objs, obj_mass):
    objs_dict = []
    for i, obj in enumerate(objs):
        obj_dict = {'name': obj , 'mass' : obj_mass[i]}
        # obj_dict = {'name': obj , 'mass' : 1.0}
        objs_dict.append(obj_dict)
    return objs_dict

def get_ai2_thor_objects(floor_plan_id):
    # connector to ai2thor to get object list
    controller = ai2thor.controller.Controller(scene="FloorPlan"+str(floor_plan_id), physics_enabled=False)
    obj = list([obj["objectType"] for obj in controller.last_event.metadata["objects"]])
    obj_mass = list([obj["mass"] for obj in controller.last_event.metadata["objects"]])
    controller.stop()
    obj = convert_to_dict_objprop(obj, obj_mass)
    return obj

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--floor-plan", type=int, required=True)
    parser.add_argument("--openai-api-key-file", type=str, default="api_key")
    parser.add_argument("--gpt-version", type=str, default="gpt-4", 
                        choices=['gpt-3.5-turbo', 'gpt-4', 'gpt-3.5-turbo-16k'])
    
    parser.add_argument("--prompt-decompse-set", type=str, default="train_task_decompose", 
                        choices=['train_task_decompose'])
    
    parser.add_argument("--prompt-allocation-set", type=str, default="train_task_allocation", 
                        choices=['train_task_allocation'])
    
    parser.add_argument("--test-set", type=str, default="final_test", 
                        choices=['final_test'])
    
    parser.add_argument("--log-results", type=bool, default=True)
    
    args = parser.parse_args()

    set_api_key(args.openai_api_key_file)
    client = OpenAI()
    
    if not os.path.isdir(f"./logs/"):
        os.makedirs(f"./logs/")
        
    # read the tasks        
    test_tasks = []
    robots_test_tasks = []  
    gt_test_tasks = []    
    trans_cnt_tasks = []
    max_trans_cnt_tasks = []  
    with open (f"./data/{args.test_set}/FloorPlan{args.floor_plan}.json", "r") as f:
        for line in f.readlines():
            test_tasks.append(list(json.loads(line).values())[0])
            robots_test_tasks.append(list(json.loads(line).values())[1])
            gt_test_tasks.append(list(json.loads(line).values())[2])
            trans_cnt_tasks.append(list(json.loads(line).values())[3])
            max_trans_cnt_tasks.append(list(json.loads(line).values())[4])
                    
    print(f"\n----Test set tasks----\n{test_tasks}\nTotal: {len(test_tasks)} tasks\n")
    # prepare list of robots for the tasks
    available_robots = []
    for robots_list in robots_test_tasks:
        task_robots = []
        for i, r_id in enumerate(robots_list):
            rob = copy.deepcopy(robots.robots[r_id-1])  
            rob['name'] = 'robot' + str(i+1)
            if 'mass_capacity' not in rob and 'mass' in rob:
                rob['mass_capacity'] = rob['mass']
            task_robots.append(rob)
        available_robots.append(task_robots)
        
    
    ######## Train Task Decomposition ########
        
    # prepare train decompostion demonstration for ai2thor samples
    prompt = f"from skills import " + actions.ai2thor_actions
    prompt += f"\nimport time"
    prompt += f"\nimport threading"
    # objects_ai = f"\n\nobjects = {get_ai2_thor_objects(args.floor_plan)}"
    # prompt += objects_ai
    objects_list_small = get_ai2_thor_objects_compact(args.floor_plan, limit=60, dedup=True)
    objects_ai = "\n\nobjects = " + to_compact_json(objects_list_small)
    prompt += objects_ai
    
    # read input train prompts
    decompose_prompt_file = open(os.getcwd() + "/data/pythonic_plans/" + args.prompt_decompse_set + ".py", "r")
    decompose_prompt = decompose_prompt_file.read()
    decompose_prompt_file.close()
    
    prompt += "\n\n" + decompose_prompt

    # >>> ADD: Human-assist rules for decomposition (compact)
    prompt += COMPACT_DECOMP


    print ("Generating Decompsed Plans...")
    
    decomposed_plan = []
    for task in test_tasks:
        curr_prompt =  f"{prompt}\n\n# Task Description: {task}"
        
        if "gpt" not in args.gpt_version:
            # older gpt versions
            _, text = LM(curr_prompt, args.gpt_version, max_tokens=1000, stop=["def"], frequency_penalty=0.15)
        else:            
            messages = [{"role": "user", "content": curr_prompt}]
            _, text = LM(messages,args.gpt_version, max_tokens=1300, frequency_penalty=0.0)

        delay = 30 if "gpt-4" in args.gpt_version else 1
        time.sleep(delay)

        decomposed_plan.append(text)
        
    print ("Generating Allocation Solution...")

    ######## Train Task Allocation - SOLUTION ########
    prompt = f"from skills import " + actions.ai2thor_actions
    prompt += f"\nimport time"
    prompt += f"\nimport threading"
    
    prompt_file = os.getcwd() + "/data/pythonic_plans/" + args.prompt_allocation_set + "_solution.py"
    allocated_prompt_file = open(prompt_file, "r")
    allocated_prompt = allocated_prompt_file.read()
    allocated_prompt_file.close()
    
    prompt += "\n\n" + allocated_prompt + "\n\n"

    # >>> ADD: Allocation policy (compact)
    prompt += COMPACT_ALLOC


    allocated_plan = []
    for i, plan in enumerate(decomposed_plan):
        no_robot  = len(available_robots[i])
        curr_prompt = prompt + plan
        curr_prompt += f"\n# TASK ALLOCATION"
        curr_prompt += f"\n# Scenario: There are {no_robot} robots available, The task should be performed using the minimum number of robots necessary. Robots should be assigned to subtasks that match its skills and mass capacity. Using your reasoning come up with a solution to satisfy all contraints."
        curr_prompt += f"\n\nrobots = {available_robots[i]}"
        curr_prompt += f"\n{objects_ai}"
        curr_prompt += f"\n\n# IMPORTANT: The AI should ensure that the robots assigned to the tasks have all the necessary skills to perform the tasks. IMPORTANT: Determine whether the subtasks must be performed sequentially or in parallel, or a combination of both and allocate robots based on availablitiy. "
        curr_prompt += f"\n# SOLUTION  \n"

        if "gpt" not in args.gpt_version:
            # older versions of GPT
            _, text = LM(curr_prompt, args.gpt_version, max_tokens=1000, stop=["def"], frequency_penalty=0.65)
        
        elif "gpt-3.5" in args.gpt_version:
            # gpt 3.5 and its variants
            messages = [{"role": "user", "content": curr_prompt}]
            _, text = LM(messages, args.gpt_version, max_tokens=1500, frequency_penalty=0.35)
        
        else:          
            # gpt 4.0
            messages = [{"role": "system", "content": "You are a Robot Task Allocation Expert. Determine whether the subtasks must be performed sequentially or in parallel, or a combination of both based on your reasoning. In the case of Task Allocation based on Robot Skills alone - First check if robot teams are required. Then Ensure that robot skills or robot team skills match the required skills for the subtask when allocating. Make sure that condition is met. In the case of Task Allocation based on Mass alone - First check if robot teams are required. Then Ensure that robot mass capacity or robot team combined mass capacity is greater than or equal to the mass for the object when allocating. Make sure that condition is met. In both the Task Task Allocation based on Mass alone and Task Allocation based on Skill alone, if there are multiple options for allocation, pick the best available option by reasoning to the best of your ability."},{"role": "system", "content": "You are a Robot Task Allocation Expert"},{"role": "user", "content": curr_prompt}]
            _, text = LM(messages, args.gpt_version, max_tokens=400, frequency_penalty=0.69)

        delay = 30 if "gpt-4" in args.gpt_version else 1
        time.sleep(delay)

        allocated_plan.append(text)
    
    print ("Generating Allocated Code...")
    
    ######## Train Task Allocation - CODE Solution ########

    prompt = f"from skills import " + actions.ai2thor_actions
    prompt += f"\nimport time"
    prompt += f"\nimport threading"
    prompt += objects_ai
    
    code_plan = []

    prompt_file1 = os.getcwd() + "/data/pythonic_plans/" + args.prompt_allocation_set + "_code.py"
    code_prompt_file = open(prompt_file1, "r")
    code_prompt = code_prompt_file.read()
    code_prompt_file.close()
    
    prompt += "\n\n" + code_prompt + "\n\n"

    
    # --- compact code policy (one block only)
    prompt += COMPACT_CODE



    for i, (plan, solution) in enumerate(zip(decomposed_plan,allocated_plan)):
        curr_prompt = prompt + plan
        curr_prompt += f"\n# TASK ALLOCATION"
        curr_prompt += f"\n\nrobots = {available_robots[i]}"
        curr_prompt += solution
        curr_prompt += f"\n# CODE Solution  \n"
        
        if "gpt" not in args.gpt_version:
            # older versions of GPT
            _, text = LM(curr_prompt, args.gpt_version, max_tokens=1000, stop=["def"], frequency_penalty=0.30)
        else:            
            # using variants of gpt 4 or 3.5
            messages = [{"role": "system", "content": "You are a Robot Task Allocation Expert"},{"role": "user", "content": curr_prompt}]
            _, text = LM(messages, args.gpt_version, max_tokens=1400, frequency_penalty=0.4)

        delay = 30 if "gpt-4" in args.gpt_version else 1
        time.sleep(delay)
        code_plan.append(text)
    
    # save generated plan
    exec_folders = []
    if args.log_results:
        line = {}
        now = datetime.now() # current date and time
        date_time = now.strftime("%m-%d-%Y-%H-%M-%S")
        
        for idx, task in enumerate(test_tasks):
            task_name = "{fxn}".format(fxn = '_'.join(task.split(' ')))
            task_name = task_name.replace('\n','')
            folder_name = f"{task_name}_plans_{date_time}"
            exec_folders.append(folder_name)
            
            os.mkdir("./logs/"+folder_name)
     
            with open(f"./logs/{folder_name}/log.txt", 'w') as f:
                f.write(task)
                f.write(f"\n\nGPT Version: {args.gpt_version}")
                f.write(f"\n\nFloor Plan: {args.floor_plan}")
                f.write(f"\n{objects_ai}")
                f.write(f"\nrobots = {available_robots[idx]}")
                f.write(f"\nground_truth = {gt_test_tasks[idx]}")
                f.write(f"\ntrans = {trans_cnt_tasks[idx]}")
                f.write(f"\nmax_trans = {max_trans_cnt_tasks[idx]}")

            with open(f"./logs/{folder_name}/decomposed_plan.py", 'w') as d:
                d.write(decomposed_plan[idx])
                
            with open(f"./logs/{folder_name}/allocated_plan.py", 'w') as a:
                a.write(allocated_plan[idx])
                
            with open(f"./logs/{folder_name}/code_plan.py", 'w') as x:
                x.write(code_plan[idx])
            