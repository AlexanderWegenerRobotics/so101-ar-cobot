import argparse
import time

from controls.grasp import plan_pick_and_place, unreachable_steps
from controls.kinematics import load_kinematics
from hardware import load_robot_config
from interfaces.bus import Publisher, Subscriber
from interfaces.messages import ArmGoal, Topic
from task.objects import load_objects, target_bin

TERMINAL_STATUSES = ("succeeded", "failed", "rejected", "preempted")
CONNECT_DELAY = 0.3
POLL_PERIOD = 0.01
TIMEOUT = 60.0


class GoalRunner:
    def __init__(self):
        self.publisher = Publisher()
        self.subscriber = Subscriber([Topic.OBJECTS, Topic.ARM_STATE, Topic.ARM_STATUS])
        self.latest = {}
        self.statuses = {}
        self.goal_count = 0
        time.sleep(CONNECT_DELAY)

    def pump(self):
        time.sleep(POLL_PERIOD)
        for topic, msg in self.subscriber.drain():
            self.latest[topic] = msg
            if topic == Topic.ARM_STATUS:
                self.statuses[msg.goal_id] = msg

    def wait_until_ready(self):
        deadline = time.time() + TIMEOUT
        while not {Topic.OBJECTS, Topic.ARM_STATE} <= self.latest.keys():
            if time.time() > deadline:
                raise TimeoutError("no objects or arm state on the bus, is cobot-launch pickplace running?")
            self.pump()

    def send(self, kind, **fields):
        self.goal_count += 1
        goal = ArmGoal(f"pick-{self.goal_count}", kind, **fields)
        self.publisher.publish(Topic.ARM_COMMAND, goal)
        return goal

    def run(self, kind, **fields):
        goal = self.send(kind, **fields)
        deadline = time.time() + TIMEOUT
        while time.time() < deadline:
            self.pump()
            msg = self.statuses.get(goal.goal_id)
            if msg is not None and msg.status in TERMINAL_STATUSES:
                if msg.status != "succeeded":
                    raise RuntimeError(f"{kind} goal {msg.status}: {msg.reason}")
                return
        raise TimeoutError(f"{kind} goal did not finish")


def parse_args():
    parser = argparse.ArgumentParser(description="Pick cubes and place them in the bin of their color")
    parser.add_argument("--cubes", type=int, nargs="+", help="tag ids to move, default: all the robot can lift")
    parser.add_argument("--dry-run", action="store_true", help="plan and check the paths without moving the arm")
    return parser.parse_args()


def main():
    args = parse_args()
    robot_cfg = load_robot_config()
    params, angular_speed = robot_cfg["grasp"], robot_cfg["controller"]["max_angular_speed"]
    kinematics = load_kinematics()
    cubes, bins = load_objects()
    ready = dict(zip([joint.name for joint in kinematics.joints], params["ready_deg"]))

    runner = GoalRunner()
    runner.wait_until_ready()
    objects = {obj.tag_id: obj for obj in runner.latest[Topic.OBJECTS].objects}
    tags = args.cubes or [tag for tag, cube in cubes.items() if cube.robot_feasible]
    try:
        if not args.dry_run:
            runner.run("joints", joints=ready)
            time.sleep(POLL_PERIOD * 10)
            runner.pump()
        start = runner.latest[Topic.ARM_STATE].tool
        for tag in tags:
            bin_tag = target_bin(cubes[tag], bins).tag_id
            if tag not in objects or bin_tag not in objects:
                print(f"cube {tag}: skipped, cube or bin not detected")
                continue
            steps = plan_pick_and_place(kinematics, objects[tag].pose, objects[bin_tag].pose.pos, params)
            problems = unreachable_steps(kinematics, start, steps, params["speed"], angular_speed)
            if problems:
                print(f"cube {tag}: skipped, unreachable steps {problems}")
                continue
            print(f"cube {tag} -> {objects[bin_tag].name}")
            for step in steps:
                if not args.dry_run:
                    runner.run("pose", target=step.target, gripper=step.gripper, max_speed=params["speed"])
                    time.sleep(step.dwell)
            start = steps[-1].target
        if not args.dry_run:
            runner.run("joints", joints=ready)
    except (RuntimeError, TimeoutError) as error:
        runner.send("stop")
        raise SystemExit(f"aborted, the arm was told to stop: {error}")


if __name__ == "__main__":
    main()
