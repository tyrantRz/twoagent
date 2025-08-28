from resources.Human_assist import start_human_console, human_console_loop
from ai2thor.controller import Controller
import sys, os
import threading

CONTROLLER_LOCK = threading.RLock()
BASE_DIR = os.path.dirname(os.path.abspath(sys.argv[0]))

class LockedController:
    _COUNTABLE = {
        "PickupObject", "PutObject", "ToggleObjectOn", "ToggleObjectOff",
        "OpenObject", "CloseObject", "SliceObject", "ThrowObject",
        "BreakObject", "CleanObject", "DropHandObject"
    }

    def __init__(self, ctrl, lock):
        self._c = ctrl
        self._lock = lock

    def step(self, *args, **kwargs):
        action_name = None
        if args and isinstance(args[0], dict):
            action_name = args[0].get("action")
        action_name = action_name or kwargs.get("action")

        _count  = kwargs.pop("_count", None)   
        _source = kwargs.pop("_source", None)  

        with self._lock:
            ev = self._c.step(*args, **kwargs)

        if action_name in self._COUNTABLE and (_count is None or _count is True):
            ok = False
            try:
                if hasattr(ev, "events"):
                    ok = any(e.metadata.get("lastActionSuccess", False) for e in ev.events)
                else:
                    ok = bool(ev.metadata.get("lastActionSuccess", False))
            except Exception:
                ok = False

            global total_exec, success_exec
            total_exec += 1
            if ok:
                success_exec += 1

            try:
                if hasattr(ev, "events"):
                    err = "|".join(
                        [e.metadata.get("errorMessage", "") for e in ev.events
                         if not e.metadata.get("lastActionSuccess", False)]
                    )
                else:
                    err = ev.metadata.get("errorMessage", "")
            except Exception:
                err = ""
            print(f"[COUNT] action={action_name} src={_source or 'auto'} ok={ok} "
                  f"total={total_exec} success={success_exec} err={err}")

        return ev

    def __getattr__(self, name):
        return getattr(self._c, name)

    
def boot_env(floor_no: int, agent_count: int, want_topcam: bool = True,
             w: int = 1000, h: int = 1000,
             server_timeout: int = 300, launch_timeout: int = 300):
    def _new():
        return Controller(width=w, height=h,
                          server_timeout=server_timeout, launch_timeout=launch_timeout)

    ctrl = _new()
    ctrl.reset(f"FloorPlan{floor_no}_physics")
    ctrl.step(action="Initialize", agentCount=agent_count)

    has_top = False
    if want_topcam:
        try:
            props = ctrl.step(action="GetMapViewCameraProperties").metadata["actionReturn"]
            ctrl.step(action="AddThirdPartyCamera", **props)
            has_top = True
        except Exception as e:
            print("[WARN] Top-view camera failed, restarting controller WITHOUT it:", e)
            try:
                ctrl.stop()
            except Exception:
                pass
            ctrl = _new()
            ctrl.reset(f"FloorPlan{floor_no}")
            ctrl.step(action="Initialize", agentCount=agent_count)
            has_top = False

    try:
        lock = CONTROLLER_LOCK  
    except NameError:
        lock = threading.RLock()
    locked_ctrl = LockedController(ctrl, lock)
    return locked_ctrl, has_top

# --- helpers: accept single robot or a list of robots ---
def _as_list(x):
    return x if isinstance(x, list) else [x]

def _choose_robot(robots, prefer_skill: str | None = None):
    rlist = _as_list(robots)
    if prefer_skill:
        for r in rlist:
            if prefer_skill in set(r.get("skills", [])):
                return r
    return rlist[0]

def _agent_id_for(robots, prefer_skill: str | None = None):
    r = _choose_robot(robots, prefer_skill)
    name = r.get("name", "robot1")
    agent_id = int(name.replace("robot", "")) - 1
    return agent_id, r


def _require_skill_or_human(robot_or_robots, skill: str, obj_regex: str, human_steps: list[str]) -> bool:
    """
    Return True indicates that the robot has the skill and the original logic should be executed.
    Returning False indicates that the robot does not possess the skill. The reason has been printed 
    and a prompt for a human to execute in the console has been displayed. The caller should `return` to end this action.
    """
    rlist = _as_list(robot_or_robots)
    for r in rlist:
        if skill in set(r.get("skills", [])):
            return True

    names = ", ".join(r.get("name", "?") for r in rlist)
    reason = f"Active robots [{names}] lack the skill: {skill} (target: {obj_regex})."
    print("\n[NEED HUMAN] Reason:", reason)
    print("[HOW TO HELP]:")
    for s in human_steps:
        print("  -", s)

    steps_text = "\n".join(f"- {s}" for s in human_steps)
    human_console_loop(c, prompt=reason + "\n" + steps_text)
    return False

total_exec = 0
success_exec = 0

# c = Controller( height=1000, width=1000)
# c.reset("FloorPlan" + str(floor_no)) 
no_robot = len(robots)

no_robot = min(2, no_robot)  

# initialize n agents into the scene
# multi_agent_event = c.step(dict(action='Initialize', agentMode="default", snapGrid=False, gridSize=0.5, rotateStepDegrees=20, visibilityDistance=100, fieldOfView=90, agentCount=no_robot))

# add a top view camera
c, has_top = boot_env(floor_no=floor_no, agent_count=no_robot, want_topcam=True)
# === HUMAN helper API: explain WHY + WHAT TO DO, then block until 'done' ===

def HUMAN(reason: str, steps):
    """
    Pause execution, explain why a human is needed and what to do.
    'steps' can be a list/tuple of console commands (mv/open/close/put/done).
    Blocks until the operator types 'done'.
    """
    if isinstance(steps, (list, tuple)):
        pretty = "\n".join([f"  - {s}" for s in steps])
    else:
        pretty = f"  - {steps}"
    prompt = (
        "【Need Human assists】\n"
        f"Reason：{reason}\n"
        "Please after finishing steps input done：\n"
        f"{pretty}\n"
    )
    human_console_loop(c, prompt=prompt)
# === end HUMAN helper API ===

# start_human_console(c, no_robot)

#try:
#    event = c.step(action="GetMapViewCameraProperties")
#    event = c.step(action="AddThirdPartyCamera", **event.metadata["actionReturn"])
#except Exception as e:
#    print("[WARN] Skip top-viewer camera, error: ", e)



# get reachabel positions
reachable_positions_ = c.step(action="GetReachablePositions").metadata["actionReturn"]
reachable_positions = positions_tuple = [(p["x"], p["y"], p["z"]) for p in reachable_positions_]

# randomize postions of the agents
for i in range (no_robot):
    init_pos = random.choice(reachable_positions_)
    c.step(dict(action="Teleport", position=init_pos, agentId=i))
    
objs = list([obj["objectId"] for obj in c.last_event.metadata["objects"]])
# print (objs)
    
# x = c.step(dict(action="RemoveFromScene", objectId='Lettuce|+01.11|+00.83|-01.43'))
#c.step({"action":"InitialRandomSpawn", "excludedReceptacles":["Microwave", "Pan", "Chair", "Plate", "Fridge", "Cabinet", "Drawer", "GarbageCan"]})
# c.step({"action":"InitialRandomSpawn", "excludedReceptacles":["Cabinet", "Drawer", "GarbageCan"]})

action_queue = []

task_over = False

recp_id = None

for i in range (no_robot):
    multi_agent_event = c.step(action="LookDown", degrees=35, agentId=i)
    # c.step(action="LookUp", degrees=30, 'agent_id':i)

def exec_actions():
    global total_exec, success_exec
    # delete if current output already exist
    cur_path = os.path.join(BASE_DIR, "agent_*")
    for x in glob(cur_path, recursive = True):
        shutil.rmtree (x, ignore_errors=True)
    shutil.rmtree(os.path.join(BASE_DIR, "top_view"), ignore_errors=True)
    
    # create new folders to save the images from the agents
    for i in range(no_robot):
        # folder_name = "agent_" + str(i+1)
        # folder_path = os.path.dirname(__file__) + "/" + folder_name
        # if not os.path.exists(folder_path):
            # os.makedirs(folder_path)
        folder_path = os.path.join(BASE_DIR, f"agent_{i+1}")
        os.makedirs(folder_path, exist_ok=True)
    
    # create folder to store the top view images
    # folder_name = "top_view"
    # folder_path = os.path.dirname(__file__) + "/" + folder_name
    # if not os.path.exists(folder_path):
        # os.makedirs(folder_path)
    os.makedirs(os.path.join(BASE_DIR, "top_view"), exist_ok=True)

    img_counter = 0
    
    while (not task_over) or (len(action_queue) > 0):
        if len(action_queue) > 0:
            try:
                act = action_queue[0]
                if act['action'] == 'ObjectNavExpertAction':
                    multi_agent_event = c.step(dict(action=act['action'], position=act['position'], agentId=act['agent_id']))
                    next_action = multi_agent_event.metadata['actionReturn']

                    if next_action != None:
                        multi_agent_event = c.step(action=next_action, agentId=act['agent_id'], forceAction=True)
                
                elif act['action'] == 'MoveAhead':
                    multi_agent_event = c.step(action="MoveAhead", agentId=act['agent_id'])
                    
                elif act['action'] == 'MoveBack':
                    multi_agent_event = c.step(action="MoveBack", agentId=act['agent_id'])
                        
                elif act['action'] == 'RotateLeft':
                    multi_agent_event = c.step(action="RotateLeft", degrees=act['degrees'], agentId=act['agent_id'])
                    
                elif act['action'] == 'RotateRight':
                    multi_agent_event = c.step(action="RotateRight", degrees=act['degrees'], agentId=act['agent_id'])
                    
                elif act['action'] == 'PickupObject':
                    total_exec += 1
                    multi_agent_event = c.step(action="PickupObject", objectId=act['objectId'], agentId=act['agent_id'], forceAction=True, _count=False, _source='robot')
                    if multi_agent_event.metadata['errorMessage'] != "":
                        print (multi_agent_event.metadata['errorMessage'])
                    else:
                        success_exec += 1
 
                elif act['action'] == 'PutObject':
                    total_exec += 1
                    kwargs = dict(
                        action="PutObject",
                        objectId=act['objectId'],
                        agentId=act['agent_id'],
                        forceAction=True,
                        _count=False, _source='robot'
                    )
                    if act.get('receptacleObjectId'):
                        kwargs['receptacleObjectId'] = act['receptacleObjectId']
                    multi_agent_event = c.step(**kwargs
                    )

                    if multi_agent_event.metadata['errorMessage'] != "":
                        print(multi_agent_event.metadata['errorMessage'])
                    else:
                        success_exec += 1
 
                elif act['action'] == 'ToggleObjectOn':
                    total_exec += 1
                    multi_agent_event = c.step(action="ToggleObjectOn", objectId=act['objectId'], agentId=act['agent_id'], forceAction=True, _count=False, _source='robot')
                    if multi_agent_event.metadata['errorMessage'] != "":
                        print (multi_agent_event.metadata['errorMessage'])
                    else:
                        success_exec += 1
                
                elif act['action'] == 'ToggleObjectOff':
                    total_exec += 1
                    multi_agent_event = c.step(action="ToggleObjectOff", objectId=act['objectId'], agentId=act['agent_id'], forceAction=True, _count=False, _source='robot')
                    if multi_agent_event.metadata['errorMessage'] != "":
                        print (multi_agent_event.metadata['errorMessage'])
                    else:
                        success_exec += 1
                    
                elif act['action'] == 'OpenObject':
                    total_exec += 1
                    multi_agent_event = c.step(action="OpenObject", objectId=act['objectId'], agentId=act['agent_id'], forceAction=True, _count=False, _source='robot')
                    if multi_agent_event.metadata['errorMessage'] != "":
                        print (multi_agent_event.metadata['errorMessage'])
                    else:
                        success_exec += 1
 
                    
                elif act['action'] == 'CloseObject':
                    total_exec += 1
                    multi_agent_event = c.step(action="CloseObject", objectId=act['objectId'], agentId=act['agent_id'], forceAction=True, _count=False, _source='robot')
                    if multi_agent_event.metadata['errorMessage'] != "":
                        print (multi_agent_event.metadata['errorMessage'])
                    else:
                        success_exec += 1
                        
                elif act['action'] == 'SliceObject':
                    total_exec += 1
                    multi_agent_event = c.step(action="SliceObject", objectId=act['objectId'], agentId=act['agent_id'], forceAction=True, _count=False, _source='robot')
                    if multi_agent_event.metadata['errorMessage'] != "":
                        print (multi_agent_event.metadata['errorMessage'])
                    else:
                        success_exec += 1
                        
                elif act['action'] == 'ThrowObject':
                    total_exec += 1
                    multi_agent_event = c.step(action="ThrowObject", moveMagnitude=7, agentId=act['agent_id'], forceAction=True, _count=False, _source='robot')
                    if multi_agent_event.metadata['errorMessage'] != "":
                        print (multi_agent_event.metadata['errorMessage'])
                    else:
                        success_exec += 1
                        
                elif act['action'] == 'BreakObject':
                    total_exec += 1
                    multi_agent_event = c.step(action="BreakObject", objectId=act['objectId'], agentId=act['agent_id'], forceAction=True, _count=False, _source='robot')
                    if multi_agent_event.metadata['errorMessage'] != "":
                        print (multi_agent_event.metadata['errorMessage'])
                    else:
                        success_exec += 1

                elif act['action'] == 'CleanObject':
                    total_exec += 1
                    multi_agent_event = c.step(
                        action="CleanObject",
                        objectId=act['objectId'],
                        agentId=act['agent_id'],
                        forceAction=True, _count=False, _source='robot'
                    )
                    if multi_agent_event.metadata['errorMessage'] != "":
                        print(multi_agent_event.metadata['errorMessage'])
                    else:
                        success_exec += 1
 
                elif act['action'] == 'DropHandObject':
                    total_exec += 1
                    multi_agent_event = c.step(
                        action="DropHandObject",
                        agentId=act['agent_id'],
                        forceAction=True,
                        _count=False, _source='robot'
                    )
                    if multi_agent_event.metadata['errorMessage'] != "":
                        print(multi_agent_event.metadata['errorMessage'])
                    else:
                        success_exec += 1

                elif act['action'] == 'Done':
                    multi_agent_event = c.step(action="Done")
                    
                for i,e in enumerate(multi_agent_event.events):
                    # cv2.imshow('agent%s' % i, e.cv2img)
                    # f_name = os.path.dirname(__file__) + "/agent_" + str(i+1) + "/img_" + str(img_counter).zfill(5) + ".png"
                    # cv2.imwrite(f_name, e.cv2img)
                    f_name = os.path.join(BASE_DIR, f"agent_{i+1}", f"img_{str(img_counter).zfill(5)}.png")
                    cv2.imwrite(f_name, e.cv2img)

                if has_top and c.last_event.events and c.last_event.events[0].third_party_camera_frames:
                    # top_view_rgb = cv2.cvtColor(c.last_event.events[0].third_party_camera_frames[-1], cv2.COLOR_BGR2RGB)
                    # cv2.imshow('Top View', top_view_rgb)
                    # f_name = os.path.dirname(__file__) + "/top_view/img_" + str(img_counter).zfill(5) + ".png"
                    # cv2.imwrite(f_name, top_view_rgb)
                    top_view_rgb = cv2.cvtColor(c.last_event.events[0].third_party_camera_frames[-1], cv2.COLOR_BGR2RGB)
                    f_name = os.path.join(BASE_DIR, "top_view", f"img_{str(img_counter).zfill(5)}.png")
                    cv2.imwrite(f_name, top_view_rgb)

                # if cv2.waitKey(25) & 0xFF == ord('q'):
                #    break
                
                img_counter += 1 

            except Exception as e:
                print (e)
                
            finally:
                if len(action_queue) > 0:
                    action_queue.pop(0)

        else:
            time.sleep(0.01)

       
actions_thread = threading.Thread(target=exec_actions, daemon=True)
actions_thread.start()

def GoToObject(robots, dest_obj):
    global recp_id
    
    # check if robots is a list
    
    if not isinstance(robots, list):
        # convert robot to a list
        robots = [robots]
    no_agents = len (robots)
    # robots distance to the goal 
    dist_goals = [10.0] * len(robots)
    prev_dist_goals = [10.0] * len(robots)
    count_since_update = [0] * len(robots)
    clost_node_location = [0] * len(robots)
    
    # list of objects in the scene and their centers
    objs = list([obj["objectId"] for obj in c.last_event.metadata["objects"]])
    objs_center = list([obj["axisAlignedBoundingBox"]["center"] for obj in c.last_event.metadata["objects"]])
    if "|" in dest_obj:
        # obj alredy given
        dest_obj_id = dest_obj
        pos_arr = dest_obj_id.split("|")
        dest_obj_center = {'x': float(pos_arr[1]), 'y': float(pos_arr[2]), 'z': float(pos_arr[3])}
    else:
        for idx, obj in enumerate(objs):
            
            match = re.match(dest_obj, obj)
            if match is not None:
                dest_obj_id = obj
                dest_obj_center = objs_center[idx]
                if dest_obj_center != {'x': 0.0, 'y': 0.0, 'z': 0.0}:
                    break # find the first instance
        
    print ("Going to ", dest_obj_id, dest_obj_center)
        
    dest_obj_pos = [dest_obj_center['x'], dest_obj_center['y'], dest_obj_center['z']] 
    
    # closest reachable position for each robot
    # all robots cannot reach the same spot 
    # differt close points needs to be found for each robot
    crp = closest_node(dest_obj_pos, reachable_positions, no_agents, clost_node_location)
    
    goal_thresh = 0.25
    # at least one robot is far away from the goal
    
    while all(d > goal_thresh for d in dist_goals):
        for ia, robot in enumerate(robots):
            robot_name = robot['name']
            agent_id = int(robot_name[-1]) - 1
            
            # get the pose of robot        
            metadata = c.last_event.events[agent_id].metadata
            location = {
                "x": metadata["agent"]["position"]["x"],
                "y": metadata["agent"]["position"]["y"],
                "z": metadata["agent"]["position"]["z"],
                "rotation": metadata["agent"]["rotation"]["y"],
                "horizon": metadata["agent"]["cameraHorizon"]}
            
            prev_dist_goals[ia] = dist_goals[ia] # store the previous distance to goal
            dist_goals[ia] = distance_pts([location['x'], location['y'], location['z']], crp[ia])
            
            dist_del = abs(dist_goals[ia] - prev_dist_goals[ia])
            # print (ia, "Dist to Goal: ", dist_goals[ia], dist_del, clost_node_location[ia])
            if dist_del < 0.2:
                # robot did not move 
                count_since_update[ia] += 1
            else:
                # robot moving 
                count_since_update[ia] = 0
                
            if count_since_update[ia] < 8:
                action_queue.append({'action':'ObjectNavExpertAction', 'position':dict(x=crp[ia][0], y=crp[ia][1], z=crp[ia][2]), 'agent_id':agent_id})
            else:    
                #updating goal
                clost_node_location[ia] += 1
                count_since_update[ia] = 0
                crp = closest_node(dest_obj_pos, reachable_positions, no_agents, clost_node_location)
    
            time.sleep(0.5)

    # align the robot once goal is reached
    # compute angle between robot heading and object
    metadata = c.last_event.events[agent_id].metadata
    robot_location = {
        "x": metadata["agent"]["position"]["x"],
        "y": metadata["agent"]["position"]["y"],
        "z": metadata["agent"]["position"]["z"],
        "rotation": metadata["agent"]["rotation"]["y"],
        "horizon": metadata["agent"]["cameraHorizon"]}
    
    robot_object_vec = [dest_obj_pos[0] -robot_location['x'], dest_obj_pos[2]-robot_location['z']]
    y_axis = [0, 1]
    unit_y = y_axis / np.linalg.norm(y_axis)
    unit_vector = robot_object_vec / np.linalg.norm(robot_object_vec)
    
    angle = math.atan2(np.linalg.det([unit_vector,unit_y]),np.dot(unit_vector,unit_y))
    angle = 360*angle/(2*np.pi)
    angle = (angle + 360) % 360
    rot_angle = angle - robot_location['rotation']
    
    if rot_angle > 0:
        action_queue.append({'action':'RotateRight', 'degrees':abs(rot_angle), 'agent_id':agent_id})
    else:
        action_queue.append({'action':'RotateLeft', 'degrees':abs(rot_angle), 'agent_id':agent_id})
        
    print ("Reached: ", dest_obj)
    if dest_obj == "Cabinet" or dest_obj == "Fridge" or dest_obj == "CounterTop":
        recp_id = dest_obj_id
    
def _wait_until_holding(agent_id: int, obj_regex: str, timeout: float = 8.0):
    """轮询直到 agent 手里拿到匹配 obj_regex 的物体，或超时。"""
    import time, re
    t0 = time.time()
    while time.time() - t0 < timeout:
        held = _held_object_id(agent_id)
        if held and re.match(obj_regex, held):
            return True
        time.sleep(0.05)
    return False

def _safe_interactable_poses(object_id: str, agent_id: int):
    poses = []
    try:
        ret = c.step(action="GetInteractablePoses",
                     objectId=object_id, agentId=agent_id).metadata.get("actionReturn")
    except Exception:
        ret = None

    if not ret:
        return poses

    for p in ret:
        if isinstance(p, dict):
            pos = p.get('position') or {}
            rot = p.get('rotation') or {}
            poses.append({
                'pos': {'x': pos.get('x'), 'y': pos.get('y'), 'z': pos.get('z')},
                'rot_y': (rot.get('y') if isinstance(rot, dict) else None)
            })
        elif isinstance(p, (list, tuple)):
            # [x,y,z] or [x,y,z,rotY]
            if len(p) >= 3 and all(isinstance(v, (int, float)) for v in p[:3]):
                poses.append({
                    'pos': {'x': p[0], 'y': p[1], 'z': p[2]},
                    'rot_y': (p[3] if len(p) >= 4 and isinstance(p[3], (int, float)) else None)
                })

    poses = [q for q in poses if q['pos'].get('x') is not None and q['pos'].get('z') is not None]
    return poses


def PickupObject(robots, pick_obj):
    if not isinstance(robots, list):
        robots = [robots]


    for robot in robots:
        agent_id = int(robot['name'].replace("robot", "")) - 1

        held0 = _held_object_id(agent_id)
        if held0:
            print(f"[PK/SKIP] agent={agent_id+1} already holding {held0}; skip pickup({pick_obj}).")
            continue

        if any(a.get('action') == 'PickupObject' and a.get('agent_id') == agent_id for a in action_queue):
            print(f"[PK/SKIP] agent={agent_id+1} already has a pending PickupObject; skip duplicate.")
            continue

        target = None
        for o in c.last_event.metadata["objects"]:
            if re.match(pick_obj, o["objectId"]):
                target = o; break
        if not target:
            print(f"[PK/ERR] no object matched by regex: {pick_obj}")
            continue

        pick_obj_id = target["objectId"]
        parent_list = target.get("parentReceptacles") or []
        parent_id = parent_list[0] if parent_list else None
        print(f"[PK/CTX] agent={agent_id+1} held_now=None regex='{pick_obj}'")
        print(f"[PK/TGT] id={pick_obj_id} type={target.get('objectType')} "
              f"visible={target.get('visible', False)} "
              f"dist={target.get('distance', None)} "
              f"pickupable={target.get('pickupable', None)} "
              f"parent={parent_id}")

        if parent_id:
            print(f"[PK/POSE] will use {parent_id} poses ONLY for navigation context; PickupObject target stays {pick_obj_id}")

        print(f"[PK/DO ] enqueue PickupObject agent={agent_id+1} objectId={pick_obj_id}")
        action_queue.append({'action': 'PickupObject', 'objectId': pick_obj_id, 'agent_id': agent_id})

        vis = target.get('visible', False)
        dist = target.get('distance', 9e9)
        if (not vis) or (dist is None or dist > 1.5):
            reason = []
            if not vis: reason.append("not_visible")
            if dist is None or dist > 1.5: reason.append(f"distance>{dist:.2f}" if dist else "distance>1.5")
            print(f"[PK/HINT] target conditions likely bad: {'|'.join(reason)} (you may need nav/rotate/clear blockers)")

        if not _wait_until_holding(agent_id, pick_obj, timeout=8.0):
            print(f"[PK/WARN] agent={agent_id+1} still not holding '{pick_obj}' after wait; subsequent Put may abort.")


    
def _held_object_id(agent_id):
    inv = c.last_event.events[agent_id].metadata.get("inventoryObjects", [])
    return inv[0]["objectId"] if inv else None


def PutObject(robot, put_obj, recp):

    req_agent = int(robot['name'].replace("robot", "")) - 1

    holder_id, held_oid = None, None
    for aid in range(no_robot):
        h = _held_object_id(aid)
        if h and re.match(put_obj, h):
            holder_id, held_oid = aid, h
            break
    if holder_id is None:
        if _wait_until_holding(req_agent, put_obj, timeout=8.0):
            holder_id, held_oid = req_agent, _held_object_id(req_agent)
        else:
            for aid in range(no_robot):
                h = _held_object_id(aid)
                if h and re.match(put_obj, h):
                    holder_id, held_oid = aid, h
                    break

    try:
        meta = c.last_event.events[req_agent].metadata
        pos = meta['agent']['position']; rot = meta['agent']['rotation']['y']; hor = meta['agent']['cameraHorizon']
    except Exception:
        pos = {'x': 0, 'y': 0, 'z': 0}; rot = 0; hor = 0
    print(f"[PUT/CTX] req_agent={req_agent+1} holder_agent={None if holder_id is None else holder_id+1} "
          f"held_oid={held_oid} recp_regex='{recp}' (put_obj_regex='{put_obj}') "
          f"req_pos=({pos['x']:.2f},{pos['y']:.2f},{pos['z']:.2f}) rot={rot:.1f} hor={hor:.1f}")

    if holder_id is None:
        print("[PUT/ABORT] no agent holds the target object after waiting; skip PutObject.")
        return

    recp_obj, best_d = None, float('inf')
    for o in c.last_event.metadata["objects"]:
        oid = o["objectId"]
        if re.match(recp, oid):
            d = o.get("distance", float('inf'))
            if d < best_d:
                best_d, recp_obj = d, o
    if not recp_obj:
        print(f"[PUT/ABORT] no receptacle matched by '{recp}'.")
        return

    print(f"[PUT/TGT] recp={recp_obj['objectId']} visible={recp_obj.get('visible', False)} "
          f"dist={recp_obj.get('distance', None)} openable={recp_obj.get('openable', False)} "
          f"isOpen={recp_obj.get('isOpen', None)} receptacle={recp_obj.get('receptacle', None)}")

    poses = _safe_interactable_poses(recp_obj['objectId'], holder_id)
    print(f"[PUT/POSE] {len(poses)} interactable poses for {recp_obj['objectId']}")

    if poses:
        hpos = c.last_event.events[holder_id].metadata['agent']['position']
        def d2(p):
            dx = p['pos']['x'] - hpos['x']; dz = p['pos']['z'] - hpos['z']; return dx*dx + dz*dz
        best_pose = min(poses, key=d2)
        print(f"[PUT/NAV] via interactable pose -> {best_pose['pos']} rot={best_pose['rot_y']}")
        action_queue.append({'action': 'ObjectNavExpertAction', 'position': best_pose['pos'], 'agent_id': holder_id})
        if best_pose['rot_y'] is not None:
            cur_rot = c.last_event.events[holder_id].metadata["agent"]["rotation"]["y"]
            delta = (best_pose['rot_y'] - cur_rot + 540.0) % 360.0 - 180.0
            if abs(delta) > 1.0:
                action_queue.append({'action': 'RotateRight' if delta > 0 else 'RotateLeft',
                                     'degrees': abs(delta), 'agent_id': holder_id})
    else:
        need_nav = (not recp_obj.get('visible', False)) or (
            isinstance(recp_obj.get('distance', None), (int, float)) and recp_obj['distance'] > 1.5
        )
        if need_nav:
            print(f"[PUT/NAV] navigating -> {recp_obj['objectId']} "
                  f"(visible={recp_obj.get('visible', False)} dist={recp_obj.get('distance')})")
            GoToObject({'name': f'robot{holder_id+1}', 'skills': []}, recp_obj['objectId'])

    print(f"[PUT/DO ] enqueue PutObject agent={holder_id+1} objectId={held_oid} -> recp={recp_obj['objectId']}")
    action_queue.append({
        'action': 'PutObject',
        'objectId': held_oid,
        'receptacleObjectId': recp_obj['objectId'],
        'agent_id': holder_id
    })

    t0 = time.time(); last = None
    while time.time() - t0 < 12.0:
        m = c.last_event.events[holder_id].metadata
        la = m.get('lastAction')
        if la == 'PutObject':
            break
        if la != last:
            print(f"[PUT/WAI] agent={holder_id+1} lastAction={la}")
            last = la
        time.sleep(0.05)

    m = c.last_event.events[holder_id].metadata
    held_after = _held_object_id(holder_id)
    if m.get('lastAction') == 'PutObject':
        print(f"[PUT/RES] agent={holder_id+1} ok={m.get('lastActionSuccess')} "
              f"err='{m.get('errorMessage','')}' post_held={held_after}")
        if not m.get('lastActionSuccess'):
            suspects = []
            if not recp_obj.get('visible', False): suspects.append('recp_not_visible')
            d = recp_obj.get('distance', None)
            if isinstance(d, (int, float)) and d > 1.5: suspects.append(f'recp_distance>{d:.2f}')
            if recp_obj.get('openable') and not recp_obj.get('isOpen', True): suspects.append('recp_closed')
            if recp_obj.get('receptacle') is False: suspects.append('not_a_receptacle')
            print(f"[PUT/SUSPECT] agent={holder_id+1} reason={'|'.join(suspects) or 'unknown'}")
    else:
        print(f"[PUT/WAIT] timeout; lastAction={m.get('lastAction')} (Put may still execute from queue later)")


         
def SwitchOn(robot, sw_obj):
    print ("Switching On: ", sw_obj)
    robot_name = robot['name']
    agent_id = int(robot_name[-1]) - 1
    objs = list(set([obj["objectId"] for obj in c.last_event.metadata["objects"]]))
    
    # turn on all stove burner
    if sw_obj == "StoveKnob":
        for obj in objs:
            match = re.match(sw_obj, obj)
            if match is not None:
                sw_obj_id = obj
                GoToObject(robot, sw_obj_id)
                # time.sleep(1)
                action_queue.append({'action':'ToggleObjectOn', 'objectId':sw_obj_id, 'agent_id':agent_id})
                time.sleep(0.1)
    
    # all objects apart from Stove Burner
    else:
        for obj in objs:
            match = re.match(sw_obj, obj)
            if match is not None:
                sw_obj_id = obj
                break # find the first instance
        GoToObject(robot, sw_obj_id)
        time.sleep(1)
        action_queue.append({'action':'ToggleObjectOn', 'objectId':sw_obj_id, 'agent_id':agent_id})
        time.sleep(1)            
        
def SwitchOff(robot, sw_obj):
    print ("Switching Off: ", sw_obj)
    robot_name = robot['name']
    agent_id = int(robot_name[-1]) - 1
    objs = list(set([obj["objectId"] for obj in c.last_event.metadata["objects"]]))
    
    # turn on all stove burner
    if sw_obj == "StoveKnob":
        for obj in objs:
            match = re.match(sw_obj, obj)
            if match is not None:
                sw_obj_id = obj
                action_queue.append({'action':'ToggleObjectOff', 'objectId':sw_obj_id, 'agent_id':agent_id})
                time.sleep(0.1)
    
    # all objects apart from Stove Burner
    else:
        for obj in objs:
            match = re.match(sw_obj, obj)
            if match is not None:
                sw_obj_id = obj
                break # find the first instance
        GoToObject(robot, sw_obj_id)
        time.sleep(1)
        action_queue.append({'action':'ToggleObjectOff', 'objectId':sw_obj_id, 'agent_id':agent_id})
        time.sleep(1)      
    
def OpenObject(robot, sw_obj):
    if not _require_skill_or_human(robot, "OpenObject", sw_obj, [f"open {sw_obj}", "done"]):
        return
    print("Opening: ", sw_obj)
    agent_id, robot = _agent_id_for(robot, "OpenObject")
    objs = list(set([obj["objectId"] for obj in c.last_event.metadata["objects"]]))
    
    for obj in objs:
        match = re.match(sw_obj, obj)
        if match is not None:
            sw_obj_id = obj
            break # find the first instance
        
    global recp_id
    if recp_id is not None:
        sw_obj_id = recp_id
    
    GoToObject(robot, sw_obj_id)
    time.sleep(1)
    action_queue.append({'action':'OpenObject', 'objectId':sw_obj_id, 'agent_id':agent_id})
    time.sleep(1)
    
def CloseObject(robot, sw_obj):
    if not _require_skill_or_human(robot, "CloseObject", sw_obj, [f"close {sw_obj}", "done"]):
        return
    print("Closing: ", sw_obj)   
    agent_id, robot = _agent_id_for(robot, "CloseObject")
    objs = list(set([obj["objectId"] for obj in c.last_event.metadata["objects"]]))
    
    for obj in objs:
        match = re.match(sw_obj, obj)
        if match is not None:
            sw_obj_id = obj
            break # find the first instance
        
    global recp_id
    if recp_id is not None:
        sw_obj_id = recp_id
        
    GoToObject(robot, sw_obj_id)
    time.sleep(1)
    
    action_queue.append({'action':'CloseObject', 'objectId':sw_obj_id, 'agent_id':agent_id}) 
    
    if recp_id is not None:
        recp_id = None
    time.sleep(1)
    
def BreakObject(robot, sw_obj):
    if not _require_skill_or_human(robot, "BreakObject", sw_obj, [f"break {sw_obj}", "done"]):
        return
    print ("Breaking: ", sw_obj)
    agent_id, robot = _agent_id_for(robot, "BreakObject")
    objs = list(set([obj["objectId"] for obj in c.last_event.metadata["objects"]]))
    
    for obj in objs:
        match = re.match(sw_obj, obj)
        if match is not None:
            sw_obj_id = obj
            break # find the first instance
    GoToObject(robot, sw_obj_id)
    time.sleep(1)
    action_queue.append({'action':'BreakObject', 'objectId':sw_obj_id, 'agent_id':agent_id}) 
    time.sleep(1)
    
def SliceObject(robot, sw_obj):
    if not _require_skill_or_human(robot, "SliceObject", sw_obj, [f"slice {sw_obj}", "done"]):
        return
    print ("Slicing: ", sw_obj)
    agent_id, robot = _agent_id_for(robot, "SliceObject")
    objs = list(set([obj["objectId"] for obj in c.last_event.metadata["objects"]]))
    
    for obj in objs:
        match = re.match(sw_obj, obj)
        if match is not None:
            sw_obj_id = obj
            break # find the first instance
    GoToObject(robot, sw_obj_id)
    time.sleep(1)
    action_queue.append({'action':'SliceObject', 'objectId':sw_obj_id, 'agent_id':agent_id})      
    time.sleep(1)
    
def CleanObject(robot, sw_obj):
    if not _require_skill_or_human(robot, "CleanObject", sw_obj, [f"wash {sw_obj}", "done"]):
        return
    print ("Cleaning: ", sw_obj)
    agent_id, robot = _agent_id_for(robot, "CleanObject")
    objs = list(set([obj["objectId"] for obj in c.last_event.metadata["objects"]]))

    for obj in objs:
        match = re.match(sw_obj, obj)
        if match is not None:
            sw_obj_id = obj
            break # find the first instance
    GoToObject(robot, sw_obj_id)
    time.sleep(1)
    action_queue.append({'action':'CleanObject', 'objectId':sw_obj_id, 'agent_id':agent_id}) 
    time.sleep(1)
    
def ThrowObject(robot, sw_obj):
    if not _require_skill_or_human(robot, "ThrowObject", sw_obj, [f"throw {sw_obj}", "done"]):
        return
    print ("Throwing: ", sw_obj)
    agent_id, robot = _agent_id_for(robot, "ThrowObject")
    objs = list(set([obj["objectId"] for obj in c.last_event.metadata["objects"]]))

    for obj in objs:
        match = re.match(sw_obj, obj)
        if match is not None:
            sw_obj_id = obj
            break # find the first instance
    
    action_queue.append({'action':'ThrowObject', 'objectId':sw_obj_id, 'agent_id':agent_id}) 
    time.sleep(1)