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
# === HUMAN-ASSIST (Decomposition, strict; English-only comments) ===
# Mark HUMAN only when: (a) ALL active robots lack the required skill,
# or (b) there is a hard block (too heavy / occluded / unreachable),
# or (c) >2 concurrent actors would be required (global cap = 2).
# Do NOT mark HUMAN for: GoToObject, PickupObject, PutObject, SwitchOn, SwitchOff,
# unless it's a hard block as defined above.

# Output no natural-language multi-step text inline; use the footer only.
# Footer must be exactly one of:
# [HUMAN_ASSIST_CANDIDATES]
# - <reason> -> <1-2 steps>
# or:
# [HUMAN_ASSIST_CANDIDATES] none

# === REQUIRED SUBTASKS (DO NOT SKIP) ===
# - Keep every verb from the task as a subtask: wash/clean, slice, throw, put/place,
#   open/close, switch on/off.
# - If skills are missing, KEEP the subtask and mark HUMAN in the footer; never remove or comment it out.
# - 'place/put X on/into Y' implies an explicit PutObject(X, Y); navigation alone never satisfies 'place'.
# - Precondition: Put/Place requires X to be in hand; include a Pickup step if needed.
"""

COMPACT_ALLOC = r"""
# === ALLOCATION POLICY (strict; English-only comments) ===
# Concurrency cap: at most 2 robots act at the same time.
# Prefer robots whose skills exactly match each subtask; consider mass constraints where relevant.
# HUMAN is allowed only when: all active robots lack the skill, hard block (too heavy / occluded / unreachable),
# or >2 concurrent actors would be required. Never assign HUMAN for trivial cases.
# For placement into receptacles, prefer robots that have BOTH PickupObject and PutObject.

# === SAME-OBJECT CONTINUITY (MANDATORY) ===
# If multiple subtasks operate on the SAME physical object (e.g., wash X then place X on Y),
# allocate the SAME robot to the entire chain whenever possible.
# Only reassign if you will add an explicit handoff in code:
#   PutObject onto a stable surface (CounterTop or Sink) -> next robot PickupObject from there.
"""


COMPACT_CODE = r"""
# === CODE CONSTRAINTS (concise; English-only comments) ===
# Output valid Python only. Convert markers to comments, e.g.:
#   # [HUMAN_ASSIST_CANDIDATES]
#   # none
#
# Allowed atomic actions (runtime-supported; NO DropHandObject, NO PushObject, NO PullObject):
# GoToObject, PickupObject, PutObject, OpenObject, CloseObject,
# SliceObject, SwitchOn, SwitchOff, BreakObject, CleanObject, ThrowObject.
#
# Concurrency cap: ≤2. Each atomic call uses ONE robot (even if runtime accepts a list).

ACTIVE_ROBOTS = robots[:min(2, len(robots))]
def pick_robot(candidates=ACTIVE_ROBOTS, prefer_skill=None):
    for r in candidates:
        if prefer_skill and prefer_skill in set(r.get("skills", [])):
            return r
    return candidates[0]

# Carrying / handoff rules:
# - Prefer the same robot to continue operations on the same object.
# - If you must switch, do an explicit handoff:
#   PutObject onto CounterTop/Sink, then the next robot PickupObject from there.
# - Never assume implicit transfer.

# === STRICT HUMAN USAGE (REINFORCED) ===
# - HUMAN is allowed ONLY for the cleaning action itself when robots lack CleanObject,
#   or for hard blocks (too heavy / occluded / unreachable).
# - NEVER use HUMAN for: GoToObject, PickupObject, PutObject, SwitchOn, SwitchOff.
# HUMAN(reason, steps) whitelist: mv / open / close / put / slice / wash / break / throw / done

# === WASHING PATTERN (NON-NEGOTIABLE TEMPLATE) ===
# Always implement washing like this (replace 'Lettuce' with the actual target):
#   r = pick_robot(prefer_skill="PutObject")   # same robot continues the chain
#   GoToObject(r, "Sink")
#   PutObject(r, "Lettuce", "Sink")
#   SwitchOn(r, "Faucet")
#   # If robots have CleanObject:
#   #   CleanObject(r, "Lettuce")
#   # else:
#   #   HUMAN('Wash the Lettuce', ['wash Lettuce','done'])   # exactly once
#   time.sleep(2)
#   SwitchOff(r, "Faucet")
#   PickupObject(r, "Lettuce")
# After a successful (or human) clean, NEVER call CleanObject on the same target again.

# === POST-HUMAN WASH GUARANTEE (HARD MANDATE) ===
# If you used HUMAN('wash ...'), you MUST still emit:
#   SwitchOff(r, "Faucet")
#   PickupObject(r, "Lettuce")
# Do NOT end the washing subtask before the two lines above are emitted.
# Do NOT assume the human hands the object back.

# === DO-NOT-SKIP GUARANTEE (MANDATORY) ===
# Do NOT skip, omit, or comment out required subtasks. If a required skill is missing,
# insert HUMAN(...) once and continue. Never replace a subtask with a comment.

# === HONOR VALID HUMAN CANDIDATES (MANDATORY) ===
# Honor any [HUMAN_ASSIST_CANDIDATES] that satisfy STRICT HUMAN USAGE by inserting exactly one HUMAN(...)
# at the correct step, then proceed.

# === DO-NOT-SWITCH OWNER (MANDATORY) ===
# For the same object across subtasks, keep the SAME robot. Only switch with an explicit handoff:
#   PutObject(..., "CounterTop"/"Sink") -> next robot PickupObject(...).

# === PLACE MEANS PUT (MANDATORY) ===
# Implement 'place/put X on/into Y' as:
#   (1) ensure holding barrier for X
#   (2) GoToObject(r, "Y")
#   (3) PutObject(r, "X", "Y")
# Navigation alone never satisfies 'place'.

# === PRE-PUT HOLDING BARRIER (MANDATORY) ===
# Before ANY PutObject(r, "<Obj>", "<Rec>"), ensure r is actually holding <Obj>:
#   aid = int(r['name'].replace('robot','')) - 1
#   for _ in range(30):   # up to ~3s
#       if _held_object_id(aid): break
#       time.sleep(0.1)
# If still empty: GoToObject(r, "<Obj>"); PickupObject(r, "<Obj>"); then PutObject.

# === ROBOT HANDLE ONLY (HARD RULE) ===
# All helpers accept robot HANDLE(s) (dict). Never pass 'robot1'/'robot2' strings.
# Always resolve a handle first, e.g.:
#   r = pick_robot(prefer_skill="PickupObject")
#   # or by name: r = next(rr for rr in robots if rr.get("name")=="robot1")
# Then call:
#   GoToObject(r, "Lettuce"); PickupObject(r, "Lettuce")
#   GoToObject(r, "Sink"); PutObject(r, "Lettuce", "Sink")
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
    with open(f"./data/{args.test_set}/FloorPlan{args.floor_plan}.json", "r") as f:
        for raw in f:
            line = raw.strip()
            if not line:
                continue
            data = json.loads(line)

            test_tasks.append(data.get("task", ""))

            robots_key = "robot list" if "robot list" in data else "robot_list" 
            robots_test_tasks.append(data.get(robots_key, []))

            gt_test_tasks.append(data.get("object_states", []))

            trans_val = int(data.get("trans", 0) or 0)
            max_trans_val = int(data.get("max_trans", trans_val) or trans_val)
            trans_cnt_tasks.append(trans_val)
            max_trans_cnt_tasks.append(max_trans_val)
                    
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
    for i, task in enumerate(test_tasks):
        curr_prompt = (
            f"{prompt}"
            f"\n\nrobots = {available_robots[i]}"                    
            f"\n# Task Description: {task}"
            f"\n# STRICT: Do NOT print any natural-language 'HUMAN: ...' multi-step lists." 
            f"\n# Use ONLY the footer syntax below with 1-2 steps from the whitelist."      
            f"\n# [HUMAN_ASSIST_CANDIDATES] or [HUMAN_ASSIST_CANDIDATES] none"
        )
        
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
            