"""Pick-and-place demo for the arm controller, talking only over the bus like the task node will.

    mjpython launch.py dev_viewer          # or vr, any profile with the controller and the supervisor
    python -m controls.demo                # sort every cube the robot may lift
    python -m controls.demo --tag 12       # one cube
    python -m controls.demo --home         # just go home

Uses `objects` for poses and task/objects.py for colors and the too-heavy flag. Heavy cubes are skipped, like the
task node will hand them to the human.

Grasp checks (the task node will reuse them):
- after closing, the gripper must stop on the cube (measured opening above GRIPPER_EMPTY, it reads about 18 on a
  3 cm cube and 0 when it closes on nothing)
- while carrying, every 0.1 s: gripper still above GRIPPER_EMPTY and, if the cube is visible, the cube within
  LOST_DISTANCE of the tool; otherwise the arm is stopped
- after release, the cube must be inside the bin
A failed attempt opens the gripper, waits for the cube to settle, looks again and retries (up to --attempts).
"""
import argparse
import itertools
import math
import time

import numpy as np

from interfaces.bus import Publisher, Subscriber
from interfaces.messages import ArmGoal, Topic, ToolTarget
from interfaces.transforms import quat_to_matrix
from task.objects import load_objects, target_bin

GRIPPER_OPEN = 60.0
GRIPPER_CLOSED = 0.0
GRIPPER_EMPTY = 6.0
LOST_DISTANCE = 0.05
MAX_TILT_DEG = 15.0
BIN_HALF = 0.055
PAN_AXIS = (0.0388, 0.0)
_ids = itertools.count(1)


class Lost(Exception):
    pass


def cube_grasp(pos, quat):
    """Roll that lines the jaws up with the cube faces, from the full orientation, so a cube that landed on
    another face works too. Returns (roll, tilt_deg); tilt is how far the cube's most vertical axis is off vertical."""
    R = quat_to_matrix(quat)
    up = int(np.argmax(np.abs(R[2])))
    tilt = math.degrees(math.acos(min(1.0, abs(R[2, up]))))
    side = R[:, (up + 1) % 3]
    cube_yaw = math.atan2(side[1], side[0])
    arm_yaw = math.atan2(pos[1] - PAN_AXIS[1], pos[0] - PAN_AXIS[0])
    r = arm_yaw - cube_yaw
    return (r + math.pi / 4) % (math.pi / 2) - math.pi / 4, tilt


class Arm:
    def __init__(self, timeout=30.0):
        self.pub = Publisher()
        self.sub = Subscriber([Topic.ARM_STATUS, Topic.OBJECTS, Topic.ARM_STATE])
        self.timeout = timeout
        time.sleep(0.5)

    def objects(self):
        deadline = time.time() + 3
        while time.time() < deadline:
            msg = self.sub.latest(Topic.OBJECTS, max_age=0.5)
            if msg is not None:
                return {o.tag_id: o for o in msg.objects}
            time.sleep(0.05)
        raise RuntimeError("no objects on the bus, is the launcher running?")

    def state(self):
        return self.sub.latest(Topic.ARM_STATE, max_age=0.5)

    def gripper(self):
        st = self.state()
        return st.joints["gripper"] if st is not None else None

    def stop(self):
        self.pub.publish(Topic.ARM_GOAL, ArmGoal(f"demo-{next(_ids)}", "stop", source="task"))

    def run(self, kind, target=None, gripper=None, speed=0.10, label="", monitor=None):
        """Send a goal and wait. monitor() is called every 0.1 s while it runs; if it returns a reason, the arm is
        stopped and Lost(reason) raised. Returns True on success."""
        goal = ArmGoal(f"demo-{next(_ids)}", kind, target=target, gripper=gripper, max_speed=speed, source="task")
        self.sub.drain()
        self.pub.publish(Topic.ARM_GOAL, goal)
        deadline = time.time() + self.timeout
        next_check = time.time() + 0.1
        while time.time() < deadline:
            r = self.sub.recv(timeout_ms=20)
            if monitor is not None and time.time() >= next_check:
                next_check += 0.1
                reason = monitor()
                if reason:
                    self.stop()
                    raise Lost(reason)
            if r is None or r[0] != Topic.ARM_STATUS or r[1].goal_id != goal.goal_id:
                continue
            st = r[1]
            if st.status == "accepted" and st.reason:
                print(f"  {label}: {st.reason}")
            if st.status in ("succeeded", "failed", "rejected", "preempted"):
                if st.status != "succeeded":
                    print(f"  {label}: {st.status} {st.reason}")
                return st.status == "succeeded"
        print(f"  {label}: timed out")
        return False

    def pose(self, pos, pitch=-math.pi / 2, roll=0.0, gripper=None, speed=0.10, label="move", monitor=None):
        return self.run("pose", ToolTarget([float(v) for v in pos], pitch, roll), gripper, speed, label, monitor)


def holding_monitor(arm, tag):
    def check():
        g = arm.gripper()
        if g is not None and g < GRIPPER_EMPTY:
            return f"gripper closed to {g:.0f}, nothing in it"
        obj = arm.objects().get(tag)
        st = arm.state()
        if obj is not None and st is not None and st.tool is not None:
            d = float(np.linalg.norm(np.asarray(obj.pose.pos) - np.asarray(st.tool.pos)))
            if d > LOST_DISTANCE:
                return f"cube {d * 100:.0f} cm away from the gripper"
        return None
    return check


def in_bin(obj, bin_obj):
    return np.max(np.abs(np.asarray(obj.pose.pos[:2]) - np.asarray(bin_obj.pose.pos[:2]))) < BIN_HALF


def attempt(arm, cube, a, bin_id):
    objs = arm.objects()
    obj, bin_obj = objs[cube.tag_id], objs[bin_id]
    p = np.asarray(obj.pose.pos, float)
    roll, tilt = cube_grasp(p, obj.pose.quat)
    if tilt > MAX_TILT_DEG:
        raise Lost(f"cube is tilted {tilt:.0f} deg, cannot grasp from the top")
    grasp = p + [0, 0, a.grasp_offset]
    above = grasp + [0, 0, a.approach]
    print(f"  cube at {np.round(p, 3).tolist()}, roll {math.degrees(roll):.0f} deg")
    hold = holding_monitor(arm, cube.tag_id)
    if not arm.pose(above, roll=roll, gripper=GRIPPER_OPEN, speed=a.speed, label="above cube"):
        return False
    if not arm.pose(grasp, roll=roll, speed=a.speed / 2, label="descend"):
        return False
    arm.run("gripper", gripper=GRIPPER_CLOSED, label="close")
    time.sleep(0.1)
    g = arm.gripper()
    if g is None or g < GRIPPER_EMPTY:
        raise Lost(f"missed, gripper closed to {g:.0f}")
    arm.pose(above, roll=roll, speed=a.speed / 2, label="lift", monitor=hold)
    if not arm.pose(np.asarray(bin_obj.pose.pos) + [0, 0, a.drop_height], speed=a.speed, label="over bin",
                    monitor=hold):
        return False
    arm.run("gripper", gripper=GRIPPER_OPEN, label="release")
    time.sleep(0.5)
    final = arm.objects()[cube.tag_id]
    if not in_bin(final, bin_obj):
        raise Lost(f"released but the cube is at {np.round(final.pose.pos, 3).tolist()}, not in the bin")
    print(f"  in the bin at {np.round(final.pose.pos, 3).tolist()}")
    return True


def pick_and_place(arm, cube, bin_id, a):
    for n in range(1, a.attempts + 1):
        try:
            if attempt(arm, cube, a, bin_id):
                return True
            return False
        except Lost as e:
            print(f"  attempt {n} failed: {e}")
            arm.run("gripper", gripper=GRIPPER_OPEN, label="open")
            time.sleep(1.0)
            objs = arm.objects()
            if cube.tag_id in objs and in_bin(objs[cube.tag_id], objs[bin_id]):
                print("  it fell into the bin anyway")
                return True
            if "tilted" in str(e):
                print("  leaving it for the human")
                return False
    print(f"  giving up after {a.attempts} attempts")
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", type=int, action="append")
    ap.add_argument("--home", action="store_true")
    ap.add_argument("--speed", type=float, default=0.12)
    ap.add_argument("--grasp-offset", type=float, default=0.0, help="tool z relative to the cube center")
    ap.add_argument("--approach", type=float, default=0.06)
    ap.add_argument("--drop-height", type=float, default=0.10, help="tool z above the bin floor")
    ap.add_argument("--attempts", type=int, default=3)
    a = ap.parse_args()

    arm = Arm()
    if a.home:
        arm.run("home", label="home")
        return
    cubes, bins = load_objects()
    tags = a.tag or sorted(cubes)
    done = 0
    for tag in tags:
        cube = cubes[tag]
        b = target_bin(cube, bins)
        print(f"tag {tag} ({cube.color}) -> {b.name}")
        if not cube.robot_feasible:
            print(f"  skipped, {cube.reason} (for the human)")
            continue
        objs = arm.objects()
        if tag not in objs or b.tag_id not in objs:
            print("  not visible")
            continue
        if in_bin(objs[tag], objs[b.tag_id]):
            print("  already in the bin")
            continue
        if pick_and_place(arm, cube, b.tag_id, a):
            done += 1
    arm.run("home", label="home")
    print(f"done, {done} cube(s) sorted")


if __name__ == "__main__":
    main()
