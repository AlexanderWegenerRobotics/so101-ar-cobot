import argparse
import time

import numpy as np

from controls.kinematics import load_kinematics
from interfaces.bus import Publisher, Subscriber
from interfaces.messages import ArmGoal, Topic, ToolTarget

TERMINAL_STATUSES = ("succeeded", "failed", "rejected", "preempted")
CONNECT_DELAY = 0.3


def parse_args():
    parser = argparse.ArgumentParser(description="Send one goal to the arm controller on arm/command")
    parser.add_argument("--timeout", type=float, default=20.0)
    kinds = parser.add_subparsers(dest="kind", required=True)
    pose = kinds.add_parser("pose", help="move the tool to x y z in meters, base frame")
    for axis in ("x", "y", "z"):
        pose.add_argument(axis, type=float)
    pose.add_argument("--pitch", type=float, default=-90.0, help="tool pitch in degrees, -90 points down")
    pose.add_argument("--roll", type=float, default=0.0, help="wrist roll in degrees")
    pose.add_argument("--speed", type=float, default=0.10, help="peak tool speed in m/s")
    joints = kinds.add_parser("joints", help="move to joint angles in degrees")
    joints.add_argument("angles", type=float, nargs=5, metavar="ANGLE")
    kinds.add_parser("home", help="go to the home pose")
    kinds.add_parser("stop", help="stop and hold")
    return parser.parse_args()


def build_goal(args):
    goal_id = f"cli-{int(time.time() * 1000)}"
    if args.kind == "pose":
        target = ToolTarget([args.x, args.y, args.z], np.radians(args.pitch), np.radians(args.roll))
        return ArmGoal(goal_id, "pose", target, max_speed=args.speed)
    if args.kind == "joints":
        names = [joint.name for joint in load_kinematics().joints]
        return ArmGoal(goal_id, "joints", joints=dict(zip(names, args.angles)))
    return ArmGoal(goal_id, args.kind)


def main():
    args = parse_args()
    goal = build_goal(args)
    statuses = Subscriber(Topic.ARM_STATUS)
    publisher = Publisher()
    time.sleep(CONNECT_DELAY)
    publisher.publish(Topic.ARM_COMMAND, goal)
    print(f"sent {goal.kind} goal {goal.goal_id}")
    deadline = time.time() + args.timeout
    while time.time() < deadline:
        received = statuses.recv(timeout_ms=200)
        if received is None or received[1].goal_id != goal.goal_id:
            continue
        msg = received[1]
        print(f"{msg.status} {msg.reason}".strip())
        if msg.status in TERMINAL_STATUSES:
            break
    else:
        print("no final status before the timeout, is cobot-launch controls running?")
    publisher.close()
    statuses.close()


if __name__ == "__main__":
    main()
