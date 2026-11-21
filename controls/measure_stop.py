import argparse
import time

import numpy as np

from interfaces.bus import Publisher, Subscriber
from interfaces.messages import ArmGoal, SafetyState, Topic, ToolTarget

READY_JOINTS = {"shoulder_pan": -0.3, "shoulder_lift": -22.0, "elbow_flex": 25.2, "wrist_flex": 86.8,
                "wrist_roll": 0.0}
START = [0.20, -0.12, 0.06]
END = [0.20, 0.12, 0.06]
APPROACH_SPEED = 0.10
TRIGGER_FRACTION = 0.9
HALT_SPEED = 0.005
SAFETY_PERIOD = 1 / 30
POLL_PERIOD = 0.002
CONNECT_DELAY = 0.3
COLLECT_TIME = 1.5
SETTLE_TIME = 0.3
TIMEOUT = 30.0
TERMINAL_STATUSES = ("succeeded", "failed", "rejected", "preempted")


def stop_metrics(times, positions, statuses, stop_time):
    speed = np.concatenate([[0.0], np.linalg.norm(np.diff(positions, axis=0), axis=1) / np.diff(times)])
    last_before = np.flatnonzero(times <= stop_time)[-1]
    after = np.arange(last_before + 1, len(times))
    acknowledged = after[[statuses[i] == "stopped" for i in after]]
    halted = after[speed[after] < HALT_SPEED]
    if len(acknowledged) == 0 or len(halted) == 0:
        raise RuntimeError("the arm did not report stopped and come to rest, collect for longer")
    return {"speed": speed[last_before],
            "latency": times[acknowledged[0]] - stop_time,
            "distance": np.linalg.norm(positions[halted[0]] - positions[last_before])}


class Rig:
    def __init__(self):
        self.publisher = Publisher()
        self.subscriber = Subscriber([Topic.ARM_STATE, Topic.ARM_STATUS])
        self.stop = False
        self.goal_count = 0
        time.sleep(CONNECT_DELAY)
        self.publish_safety()

    def publish_safety(self):
        self.publisher.publish(Topic.SAFETY, SafetyState(speed_scale=1.0, stop=self.stop))
        self.last_safety = time.time()

    def pump(self):
        if time.time() - self.last_safety >= SAFETY_PERIOD:
            self.publish_safety()
        time.sleep(POLL_PERIOD)
        return self.subscriber.drain()

    def send(self, kind, **fields):
        self.goal_count += 1
        goal = ArmGoal(f"measure-{self.goal_count}", kind, **fields)
        self.publisher.publish(Topic.ARM_COMMAND, goal)
        return goal

    def run_goal(self, kind, **fields):
        goal = self.send(kind, **fields)
        deadline = time.time() + TIMEOUT
        while time.time() < deadline:
            for topic, msg in self.pump():
                if topic == Topic.ARM_STATUS and msg.goal_id == goal.goal_id and msg.status in TERMINAL_STATUSES:
                    if msg.status != "succeeded":
                        raise RuntimeError(f"{kind} goal {msg.status}: {msg.reason}")
                    return
        raise TimeoutError(f"{kind} goal did not finish")


def measure_stop(rig, speed):
    rig.run_goal("pose", target=ToolTarget(START), max_speed=APPROACH_SPEED)
    rig.send("pose", target=ToolTarget(END), max_speed=speed)
    samples = []
    stop_time = None
    deadline = time.time() + TIMEOUT
    while time.time() < deadline:
        for topic, msg in rig.pump():
            if topic == Topic.ARM_STATE:
                samples.append((msg.stamp, np.array(msg.tool.pos), msg.status))
        if stop_time is None and len(samples) >= 2:
            (t0, p0, _), (t1, p1, _) = samples[-2:]
            if np.linalg.norm(p1 - p0) / (t1 - t0) >= TRIGGER_FRACTION * speed:
                stop_time = time.time()
                rig.stop = True
                rig.publish_safety()
        if stop_time is not None and time.time() - stop_time > COLLECT_TIME:
            break
    else:
        raise TimeoutError("the tool never reached 90% of the requested speed, max_joint_speed in robot.yaml may cap it")
    rig.send("stop")
    rig.stop = False
    time.sleep(SETTLE_TIME)
    times, positions, statuses = zip(*samples)
    return stop_metrics(np.array(times), np.array(positions), statuses, stop_time)


def main():
    parser = argparse.ArgumentParser(description="Measure arm stop latency and stopping distance")
    parser.add_argument("--speeds", type=float, nargs="+", default=[0.05, 0.075, 0.10], help="peak tool speeds in m/s")
    parser.add_argument("--trials", type=int, default=5)
    args = parser.parse_args()

    rig = Rig()
    rig.run_goal("joints", joints=READY_JOINTS)
    print("speed m/s (goal / at stop)    reaction latency ms (mean / max)    stopping distance mm (mean / max)")
    for speed in args.speeds:
        results = [measure_stop(rig, speed) for _ in range(args.trials)]
        latency = 1000 * np.array([r["latency"] for r in results])
        distance = 1000 * np.array([r["distance"] for r in results])
        at_stop = np.mean([r["speed"] for r in results])
        print(f"{speed:9.3f} / {at_stop:<8.3f}    {latency.mean():18.0f} / {latency.max():<6.0f}    "
              f"{distance.mean():20.1f} / {distance.max():.1f}")
    rig.run_goal("home")


if __name__ == "__main__":
    main()
