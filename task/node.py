"""Task node: shares the sorting between robot and human, runs the robot's part, tells the headset the plan.

- Tracks every cube from `objects`: pending, robot_active, human_active (lifted by the human), done (in its bin).
- Plans with task/allocator.py (who does which cube, in which order) and replans on every event: a cube done, the
  human taking a cube, a cube moved, a robot failure, an operator command, a mode or alpha change.
- Executes the robot's next cube as a step machine over arm/goal (approach, descend, close, check grasp, carry to a
  free spot in the bin, release, check). Grasp is monitored while carrying; failures retry up to max_failures,
  then the cube goes to the human with reason robot_failed.
- Publishes task/state at 5 Hz and on change: owner, order, why, robot feasibility, status per cube.

Operator commands (operator/command): start, pause, set_mode (auto, paused, teleop), assign (tag_id, owner robot /
human / null to clear), select_object (tag_id: robot does it next), cancel (clear all operator assignments),
set_alpha (value), replan. See task/README.md.
"""
import math
import time

import numpy as np

from interfaces.messages import (ArmGoal, ObjectTask, OperatorCommand, TaskState, TeleopTarget, Topic, ToolTarget)
from interfaces.node import Node
from interfaces.transforms import quat_to_matrix
from task.allocator import HUMAN, ROBOT, Agent, Allocator, AllocatorConfig, Job
from task.trials import TrialLog
from task.objects import load_objects, target_bin

GRIPPER_OPEN = 60.0
GRIPPER_CLOSED = 0.0


def cube_grasp(pos, quat, pan_axis):
    """Roll that lines the jaws up with the cube faces, and how far the cube is tilted (deg)."""
    R = quat_to_matrix(quat)
    up = int(np.argmax(np.abs(R[2])))
    tilt = math.degrees(math.acos(min(1.0, abs(R[2, up]))))
    side = R[:, (up + 1) % 3]
    r = math.atan2(pos[1] - pan_axis[1], pos[0] - pan_axis[0]) - math.atan2(side[1], side[0])
    return (r + math.pi / 4) % (math.pi / 2) - math.pi / 4, tilt


class CubeTrack:
    def __init__(self, cube, bin_):
        self.cube = cube
        self.bin = bin_
        self.status = "pending"
        self.pos = None
        self.quat = None
        self.planned_pos = None
        self.failures = 0
        self.lock = None
        self.robot_first = False
        self.reach = None
        self.reach_pos = None
        self.in_bin_raw = False
        self.in_bin_since = 0.0
        self.last_move = 0.0


class RobotJob:
    STEPS = ["approach", "descend", "close", "check_grasp", "carry", "release", "check_place"]

    def __init__(self, tag, start_time, estimate):
        self.tag = tag
        self.step = "approach"
        self.goal_id = None
        self.goal_status = None
        self.step_started = time.time()
        self.started = start_time
        self.estimate = estimate
        self.roll = 0.0
        self.grasp_pos = None
        self.drop_pos = None
        self.goal_reason = ""

    @property
    def holding(self):
        return self.step in ("check_grasp", "carry", "release")


class TaskManager(Node):
    name = "task"
    rate_hz = 20.0
    subscribe = [Topic.OBJECTS, Topic.ARM_STATUS, Topic.ARM_STATE, Topic.OPERATOR_COMMAND, Topic.OPERATOR_TELEOP,
                 Topic.HAND, Topic.SAFETY]

    def setup(self):
        from controls.kinematics import SO101Kinematics
        from controls.planner import Planner
        from hardware import load_robot_config
        p = self.params
        cfg = load_robot_config()
        self.kin = SO101Kinematics(cfg.get("sim", {}).get("joint_map", {}))
        self.planner = Planner(self.kin)
        self.home_q = np.array(cfg["home_qpos"][:5], float)
        self.pan_axis = self.kin.pan_axis[:2].copy()
        self.allocator = Allocator(AllocatorConfig(alpha=p.get("alpha", 0.5),
                                                   w_interference=p.get("w_interference", 3.0),
                                                   robot_pan_axis=tuple(self.pan_axis)))
        self.speed = p.get("speed", 0.12)
        self.approach = p.get("approach", 0.06)
        self.drop_z = p.get("drop_z", 0.09)
        self.slot = p.get("bin_slot", 0.025)
        self.max_failures = p.get("max_failures", 3)
        self.max_tilt = p.get("max_tilt_deg", 15.0)
        self.gripper_empty = p.get("gripper_empty", 6.0)
        self.lost_distance = p.get("lost_distance", 0.05)
        self.moved_threshold = p.get("moved_threshold", 0.02)
        self.settle_time = p.get("settle_time", 0.3)
        self.human_rest = np.array(p.get("human_rest", [0.55, 0.0, 0.12]))
        self.mode = "auto" if p.get("autostart", False) else "paused"
        self.min_reach, self.max_reach = self._grasp_ring()
        cubes, bins = load_objects()
        self.bins = bins
        self.tracks = {t: CubeTrack(c, target_bin(c, bins)) for t, c in cubes.items()}
        self.bin_pos = {}
        self.objects = {}
        self.arm_state = None
        self.hand = {}
        self.plan = None
        self.job = None
        self.dirty = "start"
        self.message = "press Start" if self.mode == "paused" else ""
        self.last_publish = 0.0
        self.changed = True
        self._ids = 0
        self.teleop_id = None
        self.reset_goal = None
        self.reset_world_at = None
        self.human_next = None
        self.at_home = True
        self.home_goal = None
        self.idle_since = None
        self.idle_home_delay = p.get("idle_home_delay", 1.0)
        self.trials = TrialLog(self.log, self.allocator.cfg.alpha)
        self.trial_pending = True
        self.log.info(f"task manager up, mode {self.mode}, alpha {self.allocator.cfg.alpha}")

    @staticmethod
    def _grasp_ring():
        """Quick radial bounds (from the pan axis) before the exact IK check, from config/workspace.json."""
        try:
            from hardware.workspace import load_or_compute
            z = next(z for z in load_or_compute()["zones"] if z["name"] == "grasp")
            return z["min_reach"] - 0.02, z["max_reach"] + 0.03
        except Exception:
            return 0.05, 0.40

    # ----- bus helpers

    def _goal(self, kind, target=None, gripper=None, speed=None):
        self._ids += 1
        gid = f"task-{self._ids}"
        self.publish(Topic.ARM_GOAL, ArmGoal(gid, kind, target=target, gripper=gripper,
                                             max_speed=speed or self.speed, source="task"))
        return gid

    def _say(self, text, level="info"):
        self.message = text
        getattr(self.log, level)(text)
        self.changed = True

    # ----- perception

    def _update_objects(self, msg):
        for o in msg.objects:
            self.objects[o.tag_id] = o
            if o.tag_id in self.bins:
                self.bin_pos[o.tag_id] = np.asarray(o.pose.pos, float)
        for tag, tr in self.tracks.items():
            o = self.objects.get(tag)
            if o is None:
                continue
            new_pos = np.asarray(o.pose.pos, float)
            if tr.pos is None or np.linalg.norm(new_pos - tr.pos) > 0.002:
                tr.last_move = time.time()
            tr.pos, tr.quat = new_pos, o.pose.quat
            robot_has_it = self.job is not None and self.job.tag == tag and self.job.holding
            now = time.time()
            raw = self._in_bin(tag, tr.bin.tag_id) and tr.pos[2] < 0.06 and not robot_has_it
            if raw != tr.in_bin_raw:
                tr.in_bin_raw, tr.in_bin_since = raw, now
            settled = now - tr.in_bin_since >= self.settle_time
            in_bin = raw if settled else tr.status == "done"
            if in_bin and not robot_has_it:
                if tr.status != "done":
                    who = "human" if tr.status == "human_active" else "robot" if \
                        (self.job is not None and self.job.tag == tag) or getattr(self, "last_robot_tag", None) == tag \
                        else "human"
                    tr.status = "done"
                    self.trials.done(tag, who)
                    self._event(f"tag {tag} in {tr.bin.name} ({who})")
                continue
            if tr.status == "done":
                tr.status = "pending"
                self._event(f"tag {tag} left its bin")
            lifted = tr.pos[2] > 0.035
            over_bin = self._in_bin(tag, tr.bin.tag_id)
            still = now - tr.last_move >= self.settle_time
            if lifted and not robot_has_it and tr.status != "human_active" and not over_bin:
                if self.job is not None and self.job.tag == tag:
                    self._abort_job(f"the human took tag {tag}")
                tr.status = "human_active"
                suggested = self.human_next
                self.trials.human_pick(tag, suggested)
                self._event(f"human picked up tag {tag}" + ("" if suggested in (None, tag) else
                                                             f" (suggested was {suggested})"))
            elif not lifted and tr.status == "human_active" and not over_bin and still:
                tr.status = "pending"
                self._event(f"human put tag {tag} down outside its bin")
            if tr.planned_pos is not None and tr.status == "pending" and not robot_has_it \
                    and np.linalg.norm(tr.pos[:2] - tr.planned_pos[:2]) > self.moved_threshold:
                tr.planned_pos = tr.pos.copy()
                self._event(f"tag {tag} moved")

    def _in_bin(self, tag, bin_tag):
        o, b = self.objects.get(tag), self.bin_pos.get(bin_tag)
        if o is None or b is None:
            return False
        return np.max(np.abs(np.asarray(o.pose.pos[:2]) - b[:2])) < 0.055

    def _event(self, why):
        self.log.info(why)
        self.dirty = why
        self.changed = True

    def _hand_pos(self):
        best = None
        for side, (msg, t) in self.hand.items():
            if time.time() - t < 0.5 and msg.tracked and msg.joints:
                p = np.asarray(msg.joints[0], float)
                if best is None or side == "right":
                    best = p
        return best

    # ----- robot feasibility

    def _robot_ok(self, tr):
        c = tr.cube
        if not c.robot_feasible:
            return False, c.reason
        if tr.failures >= self.max_failures:
            return False, "robot_failed"
        if tr.pos is None:
            return False, "not_visible"
        roll, tilt = cube_grasp(tr.pos, tr.quat, self.pan_axis)
        if tilt > self.max_tilt:
            return False, "tilted"
        if tr.status != "pending" and tr.reach is not None:
            return (True, "") if tr.reach else (False, "out_of_reach")
        if tr.reach_pos is None or np.linalg.norm(tr.reach_pos - tr.pos) > 0.005:
            tr.reach_pos = tr.pos.copy()
            rho = float(np.linalg.norm(tr.pos[:2] - self.pan_axis))
            if not self.min_reach <= rho <= self.max_reach:
                tr.reach = False
                return False, "out_of_reach"
            ok = self.planner.feasible_pitch(tr.pos, -math.pi / 2, roll, self.home_q) is not None and \
                self.planner.feasible_pitch(tr.pos + [0, 0, self.approach], -math.pi / 2, roll, self.home_q) is not None
            tr.reach = ok
        return (True, "") if tr.reach else (False, "out_of_reach")

    # ----- planning

    def _replan(self, why):
        jobs = []
        reasons = {}
        for tag, tr in self.tracks.items():
            ok, reason = self._robot_ok(tr)
            reasons[tag] = (ok, reason)
            if tr.status in ("done", "human_active") or tr.pos is None:
                continue
            if self.job is not None and self.job.tag == tag:
                continue
            b = self.bin_pos.get(tr.bin.tag_id)
            if b is None:
                continue
            jobs.append(Job(tag, tuple(tr.pos), tuple(b), robot_ok=ok, robot_reason=reason,
                            lock=tr.lock, robot_first=tr.robot_first))
            tr.planned_pos = tr.pos.copy()
        robot = self._robot_agent()
        human = self._human_agent()
        self.reasons = reasons
        if not jobs and robot.busy_tag is None and human.busy_tag is None:
            self.plan = None
            if self.mode == "auto" and self.message != "all cubes sorted":
                self._say("all cubes sorted")
                self.trials.finish()
            return
        try:
            self.plan_time = time.time()
            self.plan = self.allocator.plan(jobs, robot, human)
        except ValueError as e:
            self._say(f"no plan: {e}", "warning")
            self.plan = None
            return
        p = self.plan
        waits = "".join(f" wait {b - a:.0f}s@{a:.0f}" for a, b in p.waits)
        self.log.info(f"plan ({why}): robot {p.sequence(ROBOT)} human {p.sequence(HUMAN)}{waits} "
                      f"makespan {p.makespan:.1f} s human {p.human_time:.1f} s interference {p.interference:.1f} "
                      f"alpha {self.allocator.cfg.alpha} cap {p.weight} [{p.solve_ms:.0f} ms, {p.states} states]")
        self.changed = True

    def _robot_agent(self):
        tool = self.arm_state.tool.pos if self.arm_state is not None and self.arm_state.tool else [0.1, 0, 0.3]
        a = Agent(pos=tuple(tool))
        if self.job is not None:
            tr = self.tracks[self.job.tag]
            b = self.bin_pos[tr.bin.tag_id]
            al = self.allocator
            if self.job.holding:
                rem = al.robot_travel(tool, b) + al.cfg.robot_place_time
            else:
                rem = al.robot_job_time(tool, Job(tr.cube.tag_id, tuple(tr.pos), tuple(b)))
            a.busy_tag, a.busy_bin, a.busy_remaining, a.available = self.job.tag, tuple(b), rem, True
        return a

    def _human_agent(self):
        hand = self._hand_pos()
        pos = hand if hand is not None else self.human_rest
        a = Agent(pos=tuple(pos))
        for tag, tr in self.tracks.items():
            if tr.status == "human_active" and tr.bin.tag_id in self.bin_pos:
                b = self.bin_pos[tr.bin.tag_id]
                a.busy_tag, a.busy_bin = tag, tuple(b)
                a.busy_remaining = float(np.linalg.norm(b - tr.pos)) / self.allocator.cfg.human_speed + \
                    self.allocator.cfg.human_place_time
                a.pos = tuple(tr.pos)
                break
        return a

    # ----- robot execution

    def _go_home_when_idle(self):
        """Nothing left for the robot (all its cubes done, or the plan gives it none): drive home once, so it is
        out of the way and starts the next cube from the same pose."""
        robot_left = self.plan is not None and any(
            self.tracks[t].status == "pending" for t in self.plan.sequence(ROBOT))
        now = time.time()
        if robot_left or self.at_home or self.home_goal is not None:
            self.idle_since = None
            return
        if self.idle_since is None:
            self.idle_since = now
        elif now - self.idle_since > self.idle_home_delay:
            self.home_goal = self._goal("home", gripper=GRIPPER_OPEN, speed=0.10)
            self.log.info("robot has nothing to do, going home")
            self.idle_since = None

    def _start_next_job(self):
        if self.plan is None:
            return
        now = time.time()
        seq = [t for t in self.plan.sequence(ROBOT) if self.tracks[t].status == "pending"
               and now - self.tracks[t].last_move >= self.settle_time
               and not self._in_bin(t, self.tracks[t].bin.tag_id)]
        if not seq:
            return
        tag = seq[0]
        tr = self.tracks[tag]
        a = self.plan.assignments[tag]
        self.job = RobotJob(tag, time.time(), a.end - a.start)
        self.at_home = False
        self.home_goal = None
        tr.status = "robot_active"
        tr.robot_first = False
        self.log.info(f"robot starts tag {tag} (estimate {self.job.estimate:.1f} s)")
        self._begin_step("approach")
        self.changed = True

    def _begin_step(self, step):
        j = self.job
        tr = self.tracks[j.tag]
        j.step, j.step_started, j.goal_status = step, time.time(), None
        if step == "approach":
            j.roll, _ = cube_grasp(tr.pos, tr.quat, self.pan_axis)
            j.grasp_pos = tr.pos.copy()
            j.goal_id = self._goal("pose", ToolTarget((j.grasp_pos + [0, 0, self.approach]).tolist(), -math.pi / 2,
                                                      j.roll), GRIPPER_OPEN)
        elif step == "descend":
            j.goal_id = self._goal("pose", ToolTarget(j.grasp_pos.tolist(), -math.pi / 2, j.roll),
                                   speed=self.speed / 2)
        elif step == "close":
            j.goal_id = self._goal("gripper", gripper=GRIPPER_CLOSED)
        elif step == "check_grasp":
            j.goal_id = None
        elif step == "carry":
            j.drop_pos = self._drop_spot(tr.bin.tag_id)
            j.goal_id = self._goal("pose", ToolTarget(j.drop_pos.tolist(), -math.pi / 2, 0.0))
        elif step == "release":
            j.goal_id = self._goal("gripper", gripper=GRIPPER_OPEN)
        elif step == "check_place":
            j.goal_id = None
        self.changed = True

    def _drop_spot(self, bin_tag):
        b = self.bin_pos[bin_tag]
        s = self.slot
        spots = [b + [dx, dy, 0] for dx in (-s, s) for dy in (-s, s)]
        inside = [tr.pos for t, tr in self.tracks.items() if tr.pos is not None and self._in_bin(t, bin_tag)]

        def clearance(p):
            return min([np.linalg.norm(p[:2] - q[:2]) for q in inside] + [1.0])
        spot = max(spots, key=lambda p: (clearance(p), -np.linalg.norm(p[:2] - self.pan_axis)))
        return np.array([spot[0], spot[1], self.drop_z])

    def _step_job(self):
        j = self.job
        tr = self.tracks[j.tag]
        st = j.goal_status
        if st in ("failed", "rejected"):
            self._fail_job(f"{j.step} {st}: {j.goal_reason}", count="blocked" not in (j.goal_reason or ""))
            return
        if j.step in ("approach", "descend") and tr.pos is not None \
                and np.linalg.norm(tr.pos[:2] - j.grasp_pos[:2]) > self.moved_threshold:
            self._abort_job(f"tag {j.tag} moved while the robot reached for it")
            return
        if j.step == "carry":
            reason = self._holding_problem(tr)
            if reason:
                self._goal("stop")
                self._fail_job(f"lost the cube while carrying: {reason}")
                return
        if j.step == "check_grasp":
            if time.time() - j.step_started < 0.15:
                return
            g = self._gripper()
            if g is None or g < self.gripper_empty:
                self._fail_job(f"missed the grasp (gripper at {g:.0f})" if g is not None else "no gripper reading")
                return
            self._begin_step("carry")
            return
        if j.step == "check_place":
            if time.time() - j.step_started < 0.5:
                return
            if self._in_bin(j.tag, tr.bin.tag_id):
                tr.status = "done"
                took = time.time() - j.started
                self.log.info(f"robot finished tag {j.tag} in {took:.1f} s (estimate {j.estimate:.1f} s)")
                self.last_robot_tag = j.tag
                self.job = None
                self._event(f"robot placed tag {tr.cube.tag_id}")
            else:
                self._fail_job("released but the cube is not in the bin")
            return
        if st == "succeeded":
            nxt = RobotJob.STEPS[RobotJob.STEPS.index(j.step) + 1]
            self._begin_step(nxt)

    def _holding_problem(self, tr):
        g = self._gripper()
        if g is not None and g < self.gripper_empty:
            return f"gripper closed to {g:.0f}"
        if self.arm_state is not None and self.arm_state.tool is not None and tr.pos is not None:
            d = float(np.linalg.norm(tr.pos - np.asarray(self.arm_state.tool.pos)))
            if d > self.lost_distance:
                return f"cube {d * 100:.0f} cm from the gripper"
        return None

    def _gripper(self):
        return self.arm_state.joints.get("gripper") if self.arm_state is not None else None

    def _fail_job(self, reason, count=True):
        j = self.job
        self.last_robot_tag = j.tag
        tr = self.tracks[j.tag]
        tr.failures += 1 if count else 0
        self.trials.count("robot_failures")
        tr.status = "pending"
        self._goal("gripper", gripper=GRIPPER_OPEN)
        self.job = None
        self.retry_after = time.time() + 1.0
        left = self.max_failures - tr.failures
        self._say(f"robot failed tag {j.tag}: {reason}" + (f", {left} tries left" if left > 0 else
                                                            ", handing it to the human"), "warning")
        self.dirty = f"robot failed tag {j.tag}"

    def _abort_job(self, reason):
        j = self.job
        if j is None:
            return
        self._goal("stop")
        tr = self.tracks[j.tag]
        if tr.status == "robot_active":
            tr.status = "pending"
        if j.holding:
            self._goal("gripper", gripper=GRIPPER_OPEN)
        self.job = None
        self._say(f"robot stopped tag {j.tag}: {reason}")
        self.dirty = reason

    # ----- operator

    def _command(self, cmd):
        k = cmd.kind
        if k in ("start", "pause") or (k == "set_mode" and cmd.mode):
            mode = {"start": "auto", "pause": "paused"}.get(k, cmd.mode)
            self._set_mode(mode)
        elif k in ("assign", "select_object"):
            tr = self.tracks.get(cmd.tag_id)
            if tr is None:
                self._say(f"unknown tag {cmd.tag_id}", "warning")
                return
            owner = ROBOT if k == "select_object" else cmd.owner
            if owner == ROBOT:
                ok, reason = self._robot_ok(tr)
                if not ok:
                    self._say(f"the robot cannot do tag {cmd.tag_id}: {reason.replace('_', ' ')}")
                    return
            if tr.status not in ("pending", "robot_active"):
                self._say(f"tag {cmd.tag_id} is {tr.status.replace('_', ' ')}, nothing to assign")
                return
            tr.lock = owner if owner in (ROBOT, HUMAN) else None
            if k == "select_object":
                for other in self.tracks.values():
                    other.robot_first = False
            tr.robot_first = k == "select_object"
            self.trials.count("operator_assignments")
            job = self.job
            if owner == HUMAN and job is not None and job.tag == cmd.tag_id and not job.holding:
                self._abort_job(f"operator gave tag {cmd.tag_id} to the human")
            if k == "select_object" and job is not None and job.tag != cmd.tag_id and not job.holding:
                self._abort_job(f"operator wants tag {cmd.tag_id} first")
            if k == "select_object":
                when = "after the cube in its gripper" if job is not None and job.holding and job.tag != cmd.tag_id \
                    else "now"
                self._say(f"robot does tag {cmd.tag_id} next ({when})")
            else:
                self._say(f"operator: tag {cmd.tag_id} -> {owner or 'planner'}")
            self.dirty = "operator assignment"
        elif k == "cancel":
            for tr in self.tracks.values():
                tr.lock, tr.robot_first = None, False
            self._say("operator assignments cleared")
            self.dirty = "operator cancel"
        elif k == "set_alpha" and cmd.value is not None:
            self.allocator.cfg.alpha = float(np.clip(cmd.value, 0.0, 1.0))
            self._say(f"alpha {self.allocator.cfg.alpha:.2f}")
            self.dirty = "alpha changed"
            self._log_front()
        elif k == "replan":
            self.dirty = "operator replan"
        elif k == "reset_scene":
            self._start_reset()

    def _start_reset(self):
        """Next trial: stop, drive the arm home smoothly (gripper opens there), then let the sim put the cubes back,
        then wait paused for Go. The arm is never teleported, and every trial starts from the same state."""
        if self.mode == "resetting":
            return
        self.trials.abort()
        if self.job is not None:
            self._goal("stop")
            self.job = None
        self.mode = "resetting"
        self.reset_goal = self._goal("home", gripper=GRIPPER_OPEN, speed=0.10)
        self._say("resetting: arm going home")

    def _reset_arm_done(self, status, reason):
        if status != "succeeded":
            self.log.warning(f"arm did not reach home ({status} {reason}), resetting anyway")
        self.reset_goal = None
        self.publish(Topic.OPERATOR_COMMAND, OperatorCommand(kind="reset_world"))
        self.reset_world_at = time.time()

    def _finish_reset(self):
        self.reset_world_at = None
        for tr in self.tracks.values():
            tr.status, tr.failures, tr.lock, tr.robot_first = "pending", 0, None, False
            tr.reach, tr.reach_pos, tr.planned_pos = None, None, None
            tr.in_bin_raw, tr.in_bin_since = False, 0.0
        self.last_robot_tag = None
        self.mode = "paused"
        self.trials.next_trial()
        self.trial_pending = True
        self._say(f"trial {self.trials.trial} ready, press Go")
        self.dirty = "scene reset"

    def _set_mode(self, mode):
        if mode == self.mode or self.mode == "resetting":
            return
        old, self.mode = self.mode, mode
        if mode != "auto" and self.job is not None:
            self._goal("stop")
        if old == "teleop" and self.job is not None:
            self.job = None
        if mode == "auto" and self.job is not None:
            self._begin_step(self._resume_step())
        self.teleop_id = None
        self._say(f"mode {mode}")
        self.dirty = f"mode {mode}"
        if mode == "auto":
            self._log_front()

    def _resume_step(self):
        j = self.job
        if j.step in ("approach", "descend"):
            return "approach"
        if j.step in ("close", "check_grasp"):
            return "close"
        if j.step == "carry":
            return "carry"
        return j.step

    def _log_front(self):
        if not hasattr(self, "reasons"):
            return
        try:
            jobs = [Job(t, tuple(tr.pos), tuple(self.bin_pos[tr.bin.tag_id]), *self.reasons.get(t, (True, "")),
                        lock=tr.lock) for t, tr in self.tracks.items()
                    if tr.status == "pending" and tr.pos is not None and tr.bin.tag_id in self.bin_pos]
            if not jobs:
                return
            p = self.allocator.plan(jobs, self._robot_agent(), self._human_agent(), front=True)
            seen = set()
            for f in p.front:
                key = (f["human_jobs"], f["robot_jobs"], f["makespan"])
                if key in seen:
                    continue
                seen.add(key)
                self.log.info(f"  front: human does <= {f['human_cap']}: {f['human_jobs']} human / "
                              f"{f['robot_jobs']} robot cubes, done in {f['makespan']:.1f} s, human {f['human_time']:.1f} s"
                              f", interference {f['interference']:.1f}" + ("  <- chosen" if f["chosen"] else ""))
        except Exception as e:
            self.log.warning(f"front: {e}")

    def _teleop(self, msg):
        if self.mode != "teleop" or not msg.active:
            return
        if self.teleop_id is None:
            self._ids += 1
            self.teleop_id = f"teleop-{self._ids}"
        self.publish(Topic.ARM_GOAL, ArmGoal(self.teleop_id, "stream", target=msg.target, gripper=msg.gripper,
                                             max_speed=0.15, source="operator"))

    # ----- loop

    def step(self):
        for topic, msg in self.sub.drain():
            if topic == Topic.OBJECTS:
                self._update_objects(msg)
            elif topic == Topic.ARM_STATE:
                self.arm_state = msg
            elif topic == Topic.SAFETY:
                self.trials.safety(msg, robot_busy=self.job is not None)
            elif topic == Topic.ARM_STATUS:
                if msg.goal_id == self.reset_goal and msg.status in ("succeeded", "failed", "rejected", "preempted"):
                    self._reset_arm_done(msg.status, msg.reason)
                    self.at_home = msg.status == "succeeded"
                if msg.goal_id == self.home_goal and msg.status in ("succeeded", "failed", "rejected", "preempted"):
                    self.home_goal = None
                    self.at_home = msg.status == "succeeded"
                if self.job is not None and msg.goal_id == self.job.goal_id:
                    self.job.goal_status = msg.status
                    self.job.goal_reason = msg.reason
                    if msg.status in ("failed", "rejected"):
                        self.log.warning(f"goal {msg.goal_id} {msg.status}: {msg.reason}")
            elif topic == Topic.OPERATOR_COMMAND and isinstance(msg, OperatorCommand):
                self._command(msg)
            elif topic == Topic.OPERATOR_TELEOP and isinstance(msg, TeleopTarget):
                self._teleop(msg)
            elif topic == Topic.HAND:
                first = msg.side not in self.hand
                self.hand[msg.side] = (msg, time.time())
                if first and msg.tracked:
                    self.dirty = f"{msg.side} hand appeared"
        if not self.bin_pos:
            return
        if self.reset_world_at is not None and time.time() - self.reset_world_at > 1.0:
            self._finish_reset()
        if self.dirty:
            why, self.dirty = self.dirty, None
            self._replan(why)
        if self.mode == "auto" and self.trial_pending and self.plan is not None:
            self.trial_pending = False
            self.trials.start(self.allocator.cfg.alpha, self.plan.makespan)
        if self.mode == "auto":
            if self.job is not None:
                self._step_job()
            elif time.time() >= getattr(self, "retry_after", 0.0):
                self._start_next_job()
            if self.job is None:
                self._go_home_when_idle()
        now = time.time()
        if self.changed or now - self.last_publish > 0.2:
            self._publish_state()
            self.changed = False
            self.last_publish = now

    def _publish_state(self):
        objs = []
        human_next = None
        reasons = getattr(self, "reasons", {})
        for tag, tr in sorted(self.tracks.items()):
            ok, rreason = reasons.get(tag, (tr.cube.robot_feasible, tr.cube.reason))
            a = self.plan.assignments.get(tag) if self.plan is not None else None
            if self.job is not None and self.job.tag == tag:
                owner, order, why = ROBOT, 0, "in progress"
            elif tr.status == "human_active":
                owner, order, why = HUMAN, 0, "in progress"
            elif tr.status == "done":
                owner, order, why = "", 0, ""
            elif a is not None:
                busy = self.job is not None if a.owner == ROBOT else \
                    any(t.status == "human_active" for t in self.tracks.values())
                owner, order, why = a.owner, max(a.order - (1 if busy else 0), 0), a.why
            else:
                owner, order, why = "", 0, ""
            age = time.time() - getattr(self, "plan_time", time.time())
            objs.append(ObjectTask(tag, owner, tr.bin.tag_id, tr.status, why, order, ok, rreason, tr.lock,
                                   round(max(a.start - age, 0.0), 1) if a else 0.0,
                                   round(max(a.end - age, 0.0), 1) if a else 0.0))
            if owner == HUMAN and tr.status == "pending" and (human_next is None or order < human_next[1]):
                human_next = (tag, order)
        route = []
        active = next((t for t, tr in self.tracks.items() if tr.status == "human_active"), None)
        if active is not None:
            human_next = None
            tr = self.tracks[active]
            if tr.bin.tag_id in self.bin_pos:
                _, carry = self.allocator.human_route(tr.pos, Job(active, tuple(tr.pos), tuple(self.bin_pos[tr.bin.tag_id])))
                route = carry
        elif human_next is not None:
            tr = self.tracks[human_next[0]]
            hand = self._hand_pos()
            start = hand if hand is not None else self.human_rest
            if tr.pos is not None and tr.bin.tag_id in self.bin_pos:
                reach, carry = self.allocator.human_route(
                    start, Job(human_next[0], tuple(tr.pos), tuple(self.bin_pos[tr.bin.tag_id])))
                route = list(reach) + list(carry[1:])
        self.human_next = human_next[0] if human_next else None
        plan = {}
        if self.plan is not None:
            p = self.plan
            plan = {"makespan": round(p.makespan, 1), "human_time": round(p.human_time, 1),
                    "robot_time": round(p.robot_time, 1), "interference": round(float(p.interference), 2),
                    "alpha": self.allocator.cfg.alpha, "human_cap": p.weight, "solve_ms": round(p.solve_ms),
                    "human_waits": len(p.waits)}
        self.publish(Topic.TASK_STATE, TaskState(
            mode=self.mode, objects=objs, active_tag_id=self.job.tag if self.job else None,
            human_next_tag_id=human_next[0] if human_next else None,
            robot_step=self.job.step if self.job else "", plan=plan, message=self.message,
            human_route=[[round(float(p[0]), 3), round(float(p[1]), 3)] for p in route], trial=self.trials.trial))
