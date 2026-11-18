import numpy as np
import pytest

from controls.arm_controller import ArmController
from controls.kinematics import load_kinematics
from interfaces.messages import ArmGoal, SafetyState, ToolTarget

RATE = 30.0
CFG = {"max_joint_speed": 60.0, "max_angular_speed": 1.0, "watchdog_timeout": 0.2, "goal_tolerance": 1.0,
       "goal_timeout": 20.0}
NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
IDENTITY_MAP = {name: {"sign": 1, "offset_deg": 0.0} for name in NAMES}
HOME_Q = [0.0, -1.49, 0.0, 0.0, 0.0]
START = {"shoulder_pan": 0.0, "shoulder_lift": -22.9, "elbow_flex": 68.8, "wrist_flex": 44.1,
         "wrist_roll": 0.0, "gripper": 50.0}
SAFE = SafetyState(speed_scale=1.0)


class Bench:
    def __init__(self, joint_map=IDENTITY_MAP):
        self.controller = ArmController(load_kinematics(), joint_map, HOME_Q, CFG)
        self.joints = dict(START)
        self.now = 0.0
        self.messages = []
        self.blocked = False

    def send(self, goal):
        self.messages += self.controller.on_goal(goal, self.joints, self.now)

    def run(self, seconds, safety=SAFE, stream=None):
        for _ in range(round(seconds * RATE)):
            self.now += 1 / RATE
            if safety is not None:
                self.controller.on_safety(safety, self.now)
            if stream is not None:
                self.send(stream)
            action, messages = self.controller.update(self.joints, self.now)
            if not self.blocked:
                self.joints = {name: action[name] for name in action}
            self.messages += messages

    def states(self, topic="arm/status"):
        return [(msg.goal_id, msg.status) for t, msg in self.messages if t == topic]

    def last_arm_state(self):
        return [msg for t, msg in self.messages if t == "arm/state"][-1]

    def travelled(self):
        return max(abs(self.joints[n] - START[n]) for n in NAMES)


def start_tool(bench):
    return bench.last_arm_state().tool


def side_goal(bench, goal_id="g1", dy=0.05):
    bench.run(0.1)
    tool = start_tool(bench)
    return ArmGoal(goal_id, "pose", ToolTarget([tool.pos[0], tool.pos[1] + dy, tool.pos[2]], tool.pitch, tool.roll))


def test_pose_goal_reaches_target_and_succeeds():
    bench = Bench()
    goal = side_goal(bench)
    bench.send(goal)
    bench.run(5.0)
    tool = bench.last_arm_state().tool
    assert np.allclose(tool.pos, goal.target.pos, atol=2e-3)
    assert ("g1", "succeeded") in bench.states()
    assert bench.last_arm_state().status == "idle"


def test_starts_stopped_until_safety_arrives():
    bench = Bench()
    bench.run(0.5, safety=None)
    assert bench.last_arm_state().status == "stopped"
    assert bench.travelled() == 0.0


def test_speed_scale_slows_the_arm():
    full, half = Bench(), Bench()
    for bench, safety in ((full, SAFE), (half, SafetyState(speed_scale=0.5))):
        bench.send(side_goal(bench))
        bench.run(0.5, safety=safety)
    assert half.travelled() < 0.7 * full.travelled()


def test_safety_stop_holds_then_resumes():
    bench = Bench()
    bench.send(side_goal(bench))
    bench.run(0.4)
    bench.run(1.0, safety=SafetyState(speed_scale=1.0, stop=True))
    held = dict(bench.joints)
    bench.run(1.0, safety=SafetyState(speed_scale=1.0, stop=True))
    assert bench.joints == held
    assert ("g1", "succeeded") not in bench.states()
    bench.run(5.0)
    assert ("g1", "succeeded") in bench.states()


def test_watchdog_stops_when_safety_messages_end():
    bench = Bench()
    bench.send(side_goal(bench))
    bench.run(0.4)
    bench.run(0.5, safety=None)
    held = dict(bench.joints)
    bench.run(1.0, safety=None)
    assert bench.joints == held
    assert ("g1", "succeeded") not in bench.states()
    assert bench.last_arm_state().status == "stopped"


def test_stream_holds_when_commands_stop():
    bench = Bench()
    bench.run(0.1)
    tool = start_tool(bench)
    stream = ArmGoal("s1", "stream", ToolTarget([tool.pos[0], tool.pos[1] + 0.02, tool.pos[2]], tool.pitch, tool.roll))
    bench.run(1.0, stream=stream)
    assert bench.last_arm_state().status == "moving"
    bench.run(0.5)
    held = dict(bench.joints)
    bench.run(1.0)
    assert bench.joints == held
    assert bench.last_arm_state().status == "holding"


def test_stop_goal_preempts_and_holds():
    bench = Bench()
    bench.send(side_goal(bench))
    bench.run(0.4)
    bench.send(ArmGoal("stop1", "stop"))
    held = dict(bench.joints)
    bench.run(2.0)
    assert bench.joints == held
    assert ("g1", "preempted") in bench.states()
    assert ("stop1", "succeeded") in bench.states()


def test_stop_goal_works_without_safety_messages():
    bench = Bench()
    bench.send(ArmGoal("stop1", "stop"))
    assert bench.states() == [("stop1", "succeeded")]


def test_unreachable_goal_is_rejected_without_disturbing_running_goal():
    bench = Bench()
    bench.send(side_goal(bench))
    bench.send(ArmGoal("far", "pose", ToolTarget([0.8, 0.0, 0.2])))
    assert ("far", "rejected") in bench.states()
    bench.run(5.0)
    assert ("g1", "succeeded") in bench.states()


def test_blocked_arm_makes_the_goal_fail_with_timeout():
    bench = Bench()
    bench.send(side_goal(bench))
    bench.blocked = True
    bench.run(CFG["goal_timeout"] + 2.0)
    assert ("g1", "failed") in bench.states()


def test_home_goal_moves_to_home():
    bench = Bench()
    bench.send(ArmGoal("home", "home"))
    bench.run(6.0)
    assert ("home", "succeeded") in bench.states()
    assert bench.joints["shoulder_lift"] == pytest.approx(np.degrees(HOME_Q[1]), abs=1.0)


def test_joint_map_conversion_round_trips():
    joint_map = {name: {"sign": -1 if i % 2 else 1, "offset_deg": 7.0 * i} for i, name in enumerate(NAMES)}
    controller = Bench(joint_map).controller
    q = np.array([0.1, -0.5, 0.9, 0.3, -0.2])
    assert np.allclose(controller.to_radians(controller.to_degrees(q)), q)
