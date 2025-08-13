import re
from typing import Optional, Dict, Any
import threading
import time
import sys, atexit
_saw_closed_once = False

def _quiet_closed_error(e) -> bool:
    s = str(e).lower()
    return ("write to closed file" in s
            or "broken pipe" in s
            or "bad file descriptor" in s)

class _DedupStream:
    def __init__(self, stream, max_repeat=1):
        self._s = stream
        self._last = None
        self._rep = 0
        self._max = max_repeat
    def write(self, data):
        try:
            line = (data or "").strip()
            if "write to closed file" in line.lower():
                return
            if line == self._last:
                self._rep += 1
                if self._rep <= self._max:
                    self._s.write(data)
            else:
                self._last = line
                self._rep = 1
                self._s.write(data)
        except Exception:
            pass
    def flush(self):
        try:
            self._s.flush()
        except Exception:
            pass
    def __getattr__(self, name):
        return getattr(self._s, name)

sys.stdout = _DedupStream(sys.stdout, max_repeat=1)
sys.stderr = _DedupStream(sys.stderr, max_repeat=1)

@atexit.register
def _quiet_exit():
    try:
        sys.stdout.flush(); sys.stderr.flush()
    except Exception:
        pass

HELP = """
[HUMAN HELP] Available command:
  mv <ObjRegex> near robotN         # Move the object to the side of robotN(N=1/2)
  mv <ObjRegex> to <x> <y> <z>      # Move the object to the specified position
  open <ObjRegex>                   # Open the container(e.g. Fridge, Cabinet )
  close <ObjRegex>                  # Close the container
  put <ObjRegex> into <RecRegex>    # Put the object into container(if fail then back to TeleportObject on the container)
  slice <ObjRegex> [with robotN]    # Slice the object using robotN (default robot1)
  done                              # End processing and continue the task
"""

def _find_obj_id_by_regex(controller, pattern: str) -> Optional[str]:
    for o in controller.last_event.metadata["objects"]:
        if re.match(pattern, o["objectId"]):
            return o["objectId"]
    return None

def _agent_pos(controller, agent_id: int) -> Dict[str, float]:
    a = controller.last_event.events[agent_id].metadata["agent"]["position"]
    return dict(x=a["x"], y=a["y"], z=a["z"])

def _offset(pos: Dict[str, float], dx=0.35, dy=0.0, dz=0.0) -> Dict[str, float]:
    return dict(x=pos["x"] + dx, y=pos["y"] + dy, z=pos["z"] + dz)

def _center_of(controller, obj_id: str) -> Optional[Dict[str, float]]:
    for o in controller.last_event.metadata["objects"]:
        if o["objectId"] == obj_id:
            c = o["axisAlignedBoundingBox"]["center"]
            return dict(x=c["x"], y=c["y"], z=c["z"])
    return None

def _find_agent_holding(controller, agent_id: int, obj_type: str) -> bool:
    
    try:
        inv = controller.last_event.events[agent_id].metadata.get("inventoryObjects", [])
        if any(obj_type.lower() in (o.get("objectType","") or "").lower() for o in inv):
            return True
    except Exception:
        pass
    try:
        inv2 = controller.last_event.events[agent_id].metadata["agent"].get("inventoryObjects", [])
        if any(obj_type.lower() in (o.get("objectType","") or "").lower() for o in inv2):
            return True
    except Exception:
        pass
    return False

def _teleport_near(controller, agent_id: int, obj_id: str, dx=0.25, dy=0.0, dz=0.0) -> bool:
    pos = _agent_pos(controller, agent_id)
    target = _offset(pos, dx=dx, dy=dy, dz=dz)
    ev = controller.step(action="TeleportObject", objectId=obj_id, position=target, forceAction=True)
    return ev.metadata.get("lastActionSuccess", False)

def human_console_loop(controller, prompt: str = "") -> None:
    """
    Blocking console loop: Read a command -> Invoke one instance of the AI2-THOR action.
    Return condition: User inputs 'done'
    """
    if prompt:
        print(f"\n[HUMAN] need assist: {prompt}")
    print(HELP)

    while True:
        cmd = input("[HUMAN] > ").strip()
        if not cmd:
            continue

        try:
            toks = cmd.split()
            op = toks[0].lower()

            if op == "done":
                print("[HUMAN] task continue")
                return

            elif op == "mv":
                # mv <ObjRegex> near robotN | to x y z
                if len(toks) < 3:
                    print("[HUMAN] grammar:mv <ObjRegex> near robotN | to x y z")
                    continue
                obj_pat = toks[1]
                obj_id = _find_obj_id_by_regex(controller, obj_pat)
                if not obj_id:
                    print("[HUMAN] unfind object:", obj_pat); continue
                if toks[2] == "near":
                    robotN = toks[3]  # robot1/robot2
                    agent_id = int(robotN.replace("robot","")) - 1
                    pos = _agent_pos(controller, agent_id)
                    target = _offset(pos)
                elif toks[2] == "to":
                    x, y, z = float(toks[3]), float(toks[4]), float(toks[5])
                    target = dict(x=x, y=y, z=z)
                else:
                    print("[HUMAN] wrong grammar"); continue
                ev = controller.step(action="TeleportObject", objectId=obj_id, position=target, forceAction=True)
                print("[HUMAN] TeleportObject:", ev.metadata["lastActionSuccess"])

            elif op == "open":
                obj_id = _find_obj_id_by_regex(controller, toks[1])
                ev = controller.step(action="OpenObject", objectId=obj_id, forceAction=True)
                print("[HUMAN] Open:", ev.metadata["lastActionSuccess"])

            elif op == "close":
                obj_id = _find_obj_id_by_regex(controller, toks[1])
                ev = controller.step(action="CloseObject", objectId=obj_id, forceAction=True)
                print("[HUMAN] Close:", ev.metadata["lastActionSuccess"])

            elif op == "put":
                # put <ObjRegex> into <RecRegex>
                if len(toks) < 4 or toks[2] != "into":
                    print("[HUMAN] grammar: put <ObjRegex> into <RecRegex>")
                    continue
                obj_id = _find_obj_id_by_regex(controller, toks[1])
                rec_id = _find_obj_id_by_regex(controller, toks[3])
                if not obj_id or not rec_id:
                    print("[HUMAN] object/container unfound"); continue

                # Direct Put may require the agent to hold it; if unsuccessful, it will revert to being transported above the container.
                ev = controller.step(action="PutObject", objectId=obj_id, receptacleObjectId=rec_id, forceAction=True)
                ok = ev.metadata["lastActionSuccess"]
                if not ok:
                    c = _center_of(controller, rec_id)
                    if c:
                        above = dict(x=c["x"], y=c["y"] + 0.2, z=c["z"])
                        ev = controller.step(action="TeleportObject", objectId=obj_id, position=above, forceAction=True)
                        ok = ev.metadata["lastActionSuccess"]
                print("[HUMAN] Put/Teleport fallback:", ok)
            
            elif op == "slice":
                if len(toks) < 2:
                    print("[HUMAN] usage: slice <ObjRegex> [with robotN]")
                    continue

                obj_pat = toks[1]
                agent_id = 0
                if len(toks) >= 4 and toks[2].lower() == "with":
                    try:
                        agent_id = int(toks[3].replace("robot","")) - 1
                    except Exception:
                        print("[HUMAN] bad agent spec, fallback to robot1")
                        agent_id = 0

                obj_id = _find_obj_id_by_regex(controller, obj_pat)
                if not obj_id:
                    print("[HUMAN] object not found:", obj_pat); 
                    continue

                ev = controller.step(action="SliceObject", objectId=obj_id, agentId=agent_id, forceAction=True)
                if ev.metadata.get("lastActionSuccess", False):
                    print("[HUMAN] SliceObject:", True)
                    continue

                if not _find_agent_holding(controller, agent_id, "Knife"):
                    knife_id = _find_obj_id_by_regex(controller, r"Knife\|.*") or _find_obj_id_by_regex(controller, r"Knife")
                    if not knife_id:
                        print("[HUMAN] Knife not found. Tip: mv Knife near robot{} then retry.".format(agent_id+1))
                        continue
                    ev = controller.step(action="PickupObject", objectId=knife_id, agentId=agent_id, forceAction=True)
                    if not ev.metadata.get("lastActionSuccess", False):
                        _teleport_near(controller, agent_id, knife_id, dx=0.0)
                        ev = controller.step(action="PickupObject", objectId=knife_id, agentId=agent_id, forceAction=True)
                        if not ev.metadata.get("lastActionSuccess", False):
                            print("[HUMAN] Pickup Knife failed. Tip: mv Knife near robot{} then retry 'slice ...'".format(agent_id+1))
                            continue

                _teleport_near(controller, agent_id, obj_id, dx=0.3)

                ev = controller.step(action="SliceObject", objectId=obj_id, agentId=agent_id, forceAction=True)
                print("[HUMAN] SliceObject:", ev.metadata.get("lastActionSuccess", False))
            else:
                print("[HUMAN] unsurported command:", op)

        except Exception as e:
            global _saw_closed_once
            if _quiet_closed_error(e):
                if not _saw_closed_once:
                    _saw_closed_once = True
                    try:
                        print("[HUMAN] stream closed; suppress further writes")
                    except Exception:
                        pass
                return
            try:
                print("[HUMAN] Command execution failed:", e)
            except Exception:
                pass

def start_human_console(controller, n_agents: int = 2, prompt: str = ""):
    """
    A launcher that remains compatible with existing calls:
    Starts a human control console thread in the background, without blocking the main process.
    """
    t = threading.Thread(
        target=human_console_loop,
        args=(controller, prompt),
        daemon=True
    )
    t.start()
    time.sleep(0.1)
    return t
