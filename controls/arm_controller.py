import numpy as np

from controls.trajectory import LinePath
from interfaces.messages import ArmPlan, ArmState, GoalStatus, Topic, ToolTarget

MAX_DT = 0.1
KINDS = ("pose", "stream", "joints", "home")


def status(goal, state, reason=""):
    return Topic.ARM_STATUS, GoalStatus(goal.goal_id, state, reason)


class ArmController:
    def __init__(self, kinematics, joint_map, home_q, cfg):
        self.kin = kinematics
        self.names = [joint.name for joint in kinematics.joints]
        self.limits = np.array([joint.limits for joint in kinematics.joints])
        self.joint_map = joint_map
        self.home_q = np.asarray(home_q, float)
        self.cfg = cfg
        self.goal = None
        self.path = None
        self.path_time = 0.0
        self.joint_target = None
        self.stream_target = None
        self.stream_time = -np.inf
        self.last_q = None
        self.gripper = None
        self.hold = None
        self.safety = None
        self.safety_time = -np.inf
        self.safety_hold = None
        self.last_time = None

    def to_radians(self, joints):
        degrees = [self.joint_map[n]["sign"] * joints[n] - self.joint_map[n]["offset_deg"] for n in self.names]
        return np.radians(degrees)

    def to_degrees(self, q):
        return {n: self.joint_map[n]["sign"] * (d + self.joint_map[n]["offset_deg"])
                for n, d in zip(self.names, np.degrees(q))}

    def tool_target(self, q):
        pose = self.kin.forward(q)
        return ToolTarget(pose.pos.tolist(), float(self.kin.pitch(pose)), float(q[-1]))

    def on_safety(self, safety, now):
        self.safety = safety
        self.safety_time = now

    def on_goal(self, goal, joints, now):
        if goal.kind == "stop":
            return self._stop(goal, joints)
        q = self.to_radians(joints)
        path, joint_target, messages = None, None, []
        if goal.kind == "pose":
            if self.kin.inverse(goal.target, q) is None:
                return [status(goal, "rejected", "unreachable")]
            path = LinePath(self.tool_target(q), goal.target, goal.max_speed, self.cfg["max_angular_speed"])
            messages.append((Topic.ARM_PLAN, ArmPlan(goal.goal_id, path.waypoints())))
        elif goal.kind in ("joints", "home"):
            joint_target = self.home_q if goal.kind == "home" else self.to_radians({**joints, **goal.joints})
            if not np.all((joint_target >= self.limits[:, 0]) & (joint_target <= self.limits[:, 1])):
                return [status(goal, "rejected", "unreachable")]
        elif goal.kind != "stream":
            return [status(goal, "rejected", "unknown_kind")]

        if goal.gripper is not None:
            self.gripper = goal.gripper
        if goal.kind == "stream":
            self.stream_target, self.stream_time = goal.target, now
            if self.goal is not None and self.goal.kind == "stream":
                self.goal = goal
                return []
        if self.goal is not None:
            messages.append(status(self.goal, "preempted"))
        self.goal, self.path, self.joint_target = goal, path, joint_target
        self.path_time, self.last_q, self.hold = 0.0, q, None
        messages.append(status(goal, "accepted"))
        return messages

    def update(self, joints, now):
        dt = 0.0 if self.last_time is None else min(now - self.last_time, MAX_DT)
        self.last_time = now
        if self.gripper is None:
            self.gripper = joints["gripper"]
        if self._safety_stopped(now):
            if self.safety_hold is None:
                self.safety_hold = dict(joints)
            action, state, scale, messages = self.safety_hold, "stopped", 0.0, []
        else:
            self.safety_hold = None
            scale = min(self.safety.speed_scale, 1.0)
            action, state, messages = self._follow(joints, now, dt, scale)
        messages.append((Topic.ARM_STATE, self._arm_state(joints, state, scale)))
        return action, messages

    def _safety_stopped(self, now):
        return (self.safety is None or now - self.safety_time > self.cfg["watchdog_timeout"]
                or self.safety.stop or self.safety.speed_scale <= 0.0)

    def _stop(self, goal, joints):
        messages = [] if self.goal is None else [status(self.goal, "preempted")]
        self._clear_goal()
        self.hold = dict(joints)
        return messages + [status(goal, "succeeded")]

    def _clear_goal(self):
        self.goal = self.path = self.joint_target = None

    def _hold_action(self, joints):
        if self.hold is None:
            self.hold = dict(joints)
        return self.hold

    def _follow(self, joints, now, dt, scale):
        goal = self.goal
        if goal is None:
            return self._hold_action(joints), "idle", []
        if goal.kind == "stream" and now - self.stream_time > self.cfg["watchdog_timeout"]:
            return self._hold_action(joints), "holding", []

        if goal.kind == "pose":
            self.path_time += dt * scale
            desired_q = self.kin.inverse(self.path.at(self.path_time), self.last_q)
        elif goal.kind == "stream":
            desired_q = self.kin.inverse(self.stream_target, self.last_q)
        else:
            desired_q = self.joint_target
        if desired_q is None:
            if goal.kind == "stream":
                return self._hold_action(joints), "holding", []
            failed = status(goal, "failed", "unreachable")
            self._clear_goal()
            return self._hold_action(joints), "idle", [failed]

        self.last_q = desired_q
        desired = self.to_degrees(desired_q)
        action = self._limit_step(joints, desired, dt * scale)
        self.hold = None
        path_done = self.path is None or self.path_time >= self.path.duration
        error = max(abs(desired[n] - joints[n]) for n in self.names)
        if goal.kind != "stream" and path_done and error < self.cfg["goal_tolerance"]:
            done = status(goal, "succeeded")
            self._clear_goal()
            self.hold = action
            return action, "idle", [done]
        return action, "moving", []

    def _limit_step(self, joints, desired, dt):
        max_step = self.cfg["max_joint_speed"] * dt
        action = {n: joints[n] + float(np.clip(desired[n] - joints[n], -max_step, max_step)) for n in self.names}
        action["gripper"] = self.gripper
        return action

    def _arm_state(self, joints, state, scale):
        goal_id = None if self.goal is None else self.goal.goal_id
        return ArmState(joints=dict(joints), status=state, goal_id=goal_id,
                        tool=self.tool_target(self.to_radians(joints)), speed_scale=scale)
