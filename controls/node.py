"""Arm controller: the only node that moves the arm.

Listens to arm/command (from the supervisor) and safety, plans and executes, publishes arm/state, arm/plan and
arm/status. Runs the robot in position mode through make_robot(), so the same code drives the sim and the real
SO-101. See controls/README.md.
"""
import math
import time

import numpy as np

from controls.kinematics import ARM_JOINTS, GRIPPER, SO101Kinematics
from controls.planner import Planner, PlannerConfig, Unreachable
from controls.trajectory import Segment, Trajectory
from interfaces.messages import ArmGoal, ArmPlan, ArmState, GoalStatus, SafetyState, Topic, ToolTarget
from interfaces.node import Node


def profile(segments, dt=1 / 60):
    """Peak joint speed and acceleration of a planned motion, by stepping it like the controller does."""
    tr = Trajectory(segments)
    qs = [segments[0].q[0]]
    while not tr.done:
        qs.append(tr.advance(dt))
    qs = np.array(qs)
    if len(qs) < 3:
        return np.zeros(qs.shape[1]), np.zeros(qs.shape[1])
    v = np.diff(qs, axis=0) / dt
    return np.abs(v).max(axis=0), np.abs(np.diff(v, axis=0) / dt).max(axis=0)


class ArmController(Node):
    name = "controls"
    rate_hz = 60.0
    subscribe = [Topic.ARM_COMMAND, Topic.SAFETY]

    def setup(self):
        from hardware import load_robot_config, make_robot
        p = self.params
        self.cfg = load_robot_config()
        self.robot = make_robot(p.get("backend"), cfg=self.cfg)
        self.robot.connect()
        self.kin = SO101Kinematics(self.cfg.get("sim", {}).get("joint_map", {}))
        self.planner = Planner(self.kin, PlannerConfig(
            safe_z=p.get("safe_z", 0.08), min_z=p.get("min_z", 0.0),
            pitch_tolerance=math.radians(p.get("pitch_tolerance_deg", 25))))
        self.home_q = np.array(self.cfg["home_qpos"][:5], float)
        self.safety_timeout = p.get("safety_timeout", 0.2)
        self.stream_timeout = p.get("stream_timeout", 0.2)
        self.require_safety = p.get("require_safety", True)
        self.gripper_speed = p.get("gripper_speed", 150.0)
        self.gripper_settle = p.get("gripper_settle", 0.4)
        self.stream_turn_rate = p.get("stream_turn_rate", 2.0)
        self.state_every = max(1, int(round(self.rate_hz / p.get("state_rate_hz", 30.0))))
        self.blocked_error = p.get("blocked_error", 0.2)
        self.blocked_time = p.get("blocked_time", 0.5)
        self.blocked_since = None

        obs = self._read()
        self.q_cmd = self.kin.robot_to_q(obs)
        self.grip_cmd = obs[GRIPPER]
        self.grip_target = self.grip_cmd
        self.goal = None
        self.traj = None
        self.phase = "idle"
        self.status = "idle"
        self.grip_wait = 0.0
        self.stream = None
        self.safety = None
        self.safety_time = None
        self.speed_scale = 0.0
        self.tick = 0
        self.last = time.perf_counter()
        self.stop_reason = None
        self.log.info(f"arm controller up, tool at {np.round(self.kin.fk(self.q_cmd)[0], 3).tolist()}")

    # ----- io

    def _read(self):
        obs = self.robot.get_observation()
        return {k.removesuffix(".pos"): v for k, v in obs.items() if k.endswith(".pos")}

    def _send(self):
        action = {f"{k}.pos": v for k, v in self.kin.q_to_robot(self.q_cmd).items()}
        action[f"{GRIPPER}.pos"] = float(self.grip_cmd)
        self.robot.send_action(action)

    def _status(self, goal, status, reason=""):
        if goal is None:
            return
        self.publish(Topic.ARM_STATUS, GoalStatus(goal.goal_id, status, reason))
        if status in ("failed", "rejected"):
            self.log.warning(f"goal {goal.goal_id} ({goal.kind}) {status}: {reason}")
        else:
            self.log.info(f"goal {goal.goal_id} ({goal.kind}) {status}" + (f": {reason}" if reason else ""))

    # ----- goals

    def _finish(self, status, reason=""):
        self._status(self.goal, status, reason)
        self.goal, self.traj, self.stream = None, None, None
        self.phase = "idle"

    def _accept(self, goal):
        if goal.kind == "stream" and self.goal is not None and self.goal.kind == "stream" \
                and self.goal.goal_id == goal.goal_id:
            self.stream = (goal, time.perf_counter())
            if goal.gripper is not None:
                self.grip_target = float(goal.gripper)
            return
        if self.goal is not None:
            self._finish("preempted", f"by {goal.goal_id}")
        if goal.kind == "stop":
            self.grip_target = self.grip_cmd
            self._status(goal, "succeeded")
            return
        if goal.kind not in ("pose", "stream", "joints", "home", "gripper"):
            self._status(goal, "rejected", f"unknown kind {goal.kind!r}")
            return
        t0 = time.perf_counter()
        try:
            traj, plan, note = self._plan(goal)
        except Unreachable as e:
            self._status(goal, "rejected", f"unreachable: {e}")
            return
        except (TypeError, KeyError, ValueError) as e:
            self._status(goal, "rejected", f"bad goal: {e}")
            return
        self.goal, self.traj = goal, traj
        self._log_plan(goal, traj, time.perf_counter() - t0)
        self.announced = False
        self.phase = "stream" if goal.kind == "stream" else "move"
        if goal.kind == "stream":
            self.stream = (goal, time.perf_counter())
            if goal.gripper is not None:
                self.grip_target = float(goal.gripper)
        self._status(goal, "accepted", note)
        if plan:
            self.publish(Topic.ARM_PLAN, ArmPlan(goal.goal_id, plan))

    def _plan(self, goal):
        v = max(float(goal.max_speed or 0.10), 1e-3)
        note = ""
        if goal.kind == "stream":
            return None, None, note
        if goal.kind == "gripper":
            if goal.gripper is None:
                raise ValueError("gripper goal without gripper value")
            return Trajectory([]), None, note
        if goal.kind == "home":
            segs = [self.planner.joint(self.q_cmd, self.home_q, v, "home")] if not np.allclose(self.q_cmd, self.home_q) else []
        elif goal.kind == "joints":
            q = self.q_cmd.copy()
            for i, j in enumerate(ARM_JOINTS):
                if j in goal.joints:
                    q[i] = self.kin.joints.to_q(j, goal.joints[j])
            q = np.clip(q, self.kin.limits[:, 0], self.kin.limits[:, 1])
            segs = [self.planner.joint(self.q_cmd, q, v, "joints")]
        else:
            t = goal.target
            segs, pitch = self.planner.plan_pose(self.q_cmd, (t.pos, t.pitch, t.roll), v)
            if abs(pitch - t.pitch) > 1e-6:
                note = f"pitch relaxed to {math.degrees(pitch):.0f} deg"
        return Trajectory(segs), self.planner.waypoints(segs) if segs else None, note

    def _log_plan(self, goal, traj, plan_time):
        if traj is None:
            self.log.debug(f"goal {goal.goal_id} stream target {goal.target}")
            return
        target = f" target {np.round(goal.target.pos, 3).tolist()} pitch {math.degrees(goal.target.pitch):.0f} " \
                 f"roll {math.degrees(goal.target.roll):.0f}" if goal.target is not None else ""
        segs = ", ".join(f"{s.kind} {s.duration:.2f}s/{len(s.q)}pts" for s in traj.segments)
        self.log.debug(f"goal {goal.goal_id} {goal.kind}{target} v {goal.max_speed} planned in "
                       f"{plan_time * 1000:.0f} ms: [{segs}] total {traj.duration:.2f} s")
        if traj.segments:
            vel, acc = profile(traj.segments)
            self.log.debug(f"  q0 {np.round(traj.segments[0].q[0], 3).tolist()} q1 {np.round(traj.end(), 3).tolist()} "
                           f"peak joint speed {np.round(vel, 2).tolist()} rad/s, peak accel {np.round(acc, 1).tolist()} rad/s2")

    # ----- safety

    def _update_safety(self, now):
        scale, stop, reason = 1.0, False, None
        if self.safety is None or now - self.safety_time > self.safety_timeout:
            if self.require_safety:
                scale, stop, reason = 0.0, True, "no safety heartbeat"
        else:
            scale = float(np.clip(self.safety.speed_scale, 0.0, 1.0))
            if self.safety.stop:
                scale, stop, reason = 0.0, True, f"safety stop ({self.safety.reason or self.safety.zone})"
        if stop and self.stop_reason is None:
            self.log.warning(f"stopping: {reason}")
        elif not stop and self.stop_reason is not None:
            self.log.info("safety clear, resuming")
        self.stop_reason = reason if stop else None
        self.speed_scale = scale

    # ----- loop

    def step(self):
        now = time.perf_counter()
        dt = min(now - self.last, 0.1)
        self.last = now
        for topic, msg in self.sub.drain():
            if topic == Topic.SAFETY and isinstance(msg, SafetyState):
                self.safety, self.safety_time = msg, now
            elif topic == Topic.ARM_COMMAND and isinstance(msg, ArmGoal):
                self._accept(msg)
        self._update_safety(now)

        if self.phase == "move":
            if not self.announced:
                self.announced = True
                self._status(self.goal, "running")
            q = self.traj.advance(dt * self.speed_scale)
            if q is not None:
                self.q_cmd = q
            if self.traj.done:
                if self.goal.gripper is not None:
                    self.grip_target = float(self.goal.gripper)
                    self.phase = "gripper"
                    self.grip_wait = self.gripper_settle
                else:
                    self._finish("succeeded")
        elif self.phase == "stream":
            self._step_stream(now, dt)

        if self.speed_scale > 0:
            step = self.gripper_speed * dt * self.speed_scale
            self.grip_cmd += float(np.clip(self.grip_target - self.grip_cmd, -step, step))
        if self.phase == "gripper" and abs(self.grip_cmd - self.grip_target) < 1e-3:
            self.grip_wait -= dt * self.speed_scale
            if self.grip_wait <= 0:
                self._finish("succeeded")

        if self.stop_reason is not None:
            self.status = "stopped"
        elif self.phase in ("move", "gripper") or (self.phase == "stream" and self.stream_moving):
            self.status = "moving"
        elif self.phase == "stream":
            self.status = "holding"
        else:
            self.status = "idle"
        self._send()
        self.tick += 1
        if self.tick % self.state_every == 0:
            self._publish_state()

    stream_moving = False

    def _step_stream(self, now, dt):
        goal, t_recv = self.stream
        self.stream_moving = False
        if now - t_recv > self.stream_timeout or self.speed_scale <= 0:
            return
        t = goal.target
        p, pitch, roll = self.kin.fk(self.q_cmd)
        target = np.asarray(t.pos, float)
        d = target - p
        n = float(np.linalg.norm(d))
        max_step = max(float(goal.max_speed or 0.10), 1e-3) * self.speed_scale * dt
        if n > max_step:
            d *= max_step / n
        turn = self.stream_turn_rate * self.speed_scale * dt
        dp = float(np.clip(math.remainder(t.pitch - pitch, 2 * math.pi), -turn, turn))
        dr = float(np.clip(math.remainder(t.roll - roll, 2 * math.pi), -turn, turn))
        if n < 5e-4 and abs(dp) < 1e-3 and abs(dr) < 1e-3:
            return
        r = self.kin._ik(p + d, pitch + dp, roll + dr, self.q_cmd, iters=8)
        if r.pos_error < 0.003 and np.max(np.abs(r.q - self.q_cmd)) < 0.3:
            self.q_cmd = r.q
            self.stream_moving = True

    def _check_blocked(self, obs):
        """Collision check: the servos are position controlled, so a large tracking error that does not go away
        means something is in the way. Fail the goal and hold where the arm actually is instead of pushing."""
        q_meas = self.kin.robot_to_q(obs)
        err = float(np.max(np.abs(self.q_cmd - q_meas)))
        now = time.perf_counter()
        if err < self.blocked_error or self.phase not in ("move", "stream"):
            self.blocked_since = None
            return
        if self.blocked_since is None:
            self.blocked_since = now
        elif now - self.blocked_since > self.blocked_time:
            self.blocked_since = None
            self.q_cmd = q_meas
            if self.goal is not None:
                self._finish("failed", f"blocked (tracking error {err:.2f} rad)")
            else:
                self.phase = "idle"

    def _publish_state(self):
        obs = self._read()
        self._check_blocked(obs)
        if self.tick % (3 * self.state_every) == 0 and self.status != "idle":
            q_meas = self.kin.robot_to_q(obs)
            self.log.debug(f"{self.status} cmd {np.round(self.q_cmd, 3).tolist()} "
                           f"err {np.round(self.q_cmd - q_meas, 3).tolist()} grip {self.grip_cmd:.0f}/{obs[GRIPPER]:.0f} "
                           f"scale {self.speed_scale:.2f}")
        p, pitch, roll = self.kin.fk(self.kin.robot_to_q(obs))
        self.publish(Topic.ARM_STATE, ArmState(
            joints=obs, status=self.status, goal_id=self.goal.goal_id if self.goal else None,
            tool=ToolTarget([round(float(v), 4) for v in p], round(pitch, 4), round(roll, 4)),
            speed_scale=self.speed_scale))

    def teardown(self):
        if getattr(self, "robot", None) is not None and self.robot.is_connected:
            self.robot.disconnect()
