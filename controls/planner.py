"""Turns tool pose goals into joint paths: lift, travel, descend.

Vertical lift and descent are straight lines (IK on every sample). The travel between them is interpolated in
cylindrical coordinates around the pan axis (radius, angle, height), so it arcs around the base instead of cutting
through the unreachable area next to it. When the arm starts from a pose that is not a tool pose over the table
(home, upright), the first move is in joint space to the point above the target.
"""
import math
from dataclasses import dataclass

import numpy as np

from controls.kinematics import wrap
from controls.trajectory import Segment, min_jerk_accel_duration, min_jerk_duration


class Unreachable(Exception):
    pass


@dataclass
class PlannerConfig:
    safe_z: float = 0.08
    min_z: float = 0.0
    sample_step: float = 0.004
    joint_speed: float = 1.5
    joint_speed_max: float = 3.0
    min_segment_time: float = 0.25
    cartesian_min_radius: float = 0.09
    direct_distance: float = 0.03
    pitch_tolerance: float = math.radians(25)
    pitch_step: float = math.radians(5)
    max_sample_jump: float = 0.12
    ik_tol_pos: float = 4e-4
    ik_tol_angle: float = math.radians(0.4)
    tool_acc: float = 0.5
    joint_acc: float = 4.0
    blend_samples: int = 6
    min_vertical: float = 0.02


class Planner:
    def __init__(self, kin, cfg=None):
        self.kin = kin
        self.cfg = cfg or PlannerConfig()

    def cyl(self, p):
        a = self.kin.pan_axis
        d = np.asarray(p[:2]) - a[:2]
        return np.array([math.hypot(*d), math.atan2(d[1], d[0]), p[2]])

    def from_cyl(self, c):
        a = self.kin.pan_axis
        return np.array([a[0] + c[0] * math.cos(c[1]), a[1] + c[0] * math.sin(c[1]), c[2]])

    def _duration(self, q, length, v, joint_move=False):
        """Cartesian moves are limited by the tool speed v. Joint moves are limited by the joint speed (scaled
        with v, so a slow goal is slow everywhere) and let the tool go up to 2.5 v, because their tool path is
        not a straight line and its speed is uneven. Both also respect an acceleration limit (scaled with v), so
        a short hop does not get a very short, snappy duration."""
        dq = float(np.max(np.sum(np.abs(np.diff(q, axis=0)), axis=0))) if len(q) > 1 else 0.0
        k = v / 0.10
        joint_v = min(self.cfg.joint_speed * k, self.cfg.joint_speed_max)
        tool_v = 2.5 * v if joint_move else v
        return max(min_jerk_duration(length, tool_v), min_jerk_duration(dq, joint_v),
                   min_jerk_accel_duration(length, self.cfg.tool_acc * k),
                   min_jerk_accel_duration(dq, self.cfg.joint_acc * k),
                   self.cfg.min_segment_time)

    @staticmethod
    def _is_joint(seg):
        return getattr(seg, "joint_space", False)

    def blend(self, segments, v):
        """One continuous motion through all segments of a goal: no stop at the corners. The joint samples are
        concatenated, corners are rounded with a moving average (window shrinks to 0 at both ends, so start and
        target stay exact and straight parts stay straight), and the path parameter is spaced by the time each
        step needs at the speed limits, so one min-jerk profile over the whole path respects them everywhere."""
        if len(segments) <= 1 and (not segments or segments[0].knots is not None):
            return segments
        q = np.concatenate([segments[0].q] + [s.q[1:] for s in segments[1:]])
        tool_factor = np.concatenate([np.full(len(s.q) - 1, 2.5 if "joint" in s.kind or s.kind in ("travel", "home")
                                              and s.knots is None and self._is_joint(s) else 1.0)
                                      for s in segments])
        h = self.cfg.blend_samples
        for _ in range(2):
            n = len(q)
            out = q.copy()
            for i in range(1, n - 1):
                k = min(h, i, n - 1 - i)
                out[i] = q[i - k:i + k + 1].mean(axis=0)
            q = out
        pts = np.array([self.kin.fk(x)[0] for x in q])
        if np.min(pts[:, 2]) < self.cfg.min_z - 0.002:
            return segments
        k = v / 0.10
        dp = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        dq = np.max(np.abs(np.diff(q, axis=0)), axis=1)
        cost = np.maximum(dp / (v * tool_factor), dq / min(self.cfg.joint_speed * k, self.cfg.joint_speed_max)) + 1e-9
        knots = np.concatenate([[0.0], np.cumsum(cost)])
        length = float(dp.sum())
        joint_len = float(np.max(np.sum(np.abs(np.diff(q, axis=0)), axis=0)))
        duration = max(1.875 * knots[-1], min_jerk_accel_duration(length, self.cfg.tool_acc * k),
                       min_jerk_accel_duration(joint_len, self.cfg.joint_acc * k), self.cfg.min_segment_time)
        return [Segment(q, duration, "+".join(s.kind for s in segments), knots / knots[-1])]

    def _path_length(self, q):
        pts = np.array([self.kin.fk(x)[0] for x in q[:: max(1, len(q) // 20)]] + [self.kin.fk(q[-1])[0]])
        return float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1))), pts

    def _ik(self, pos, pitch, roll, q, track=False):
        """track=True: follow a path from the previous sample, no reseeding (a reseed can jump to another branch)."""
        if track:
            return self.kin._ik(pos, pitch, roll, q, iters=100, tol_pos=self.cfg.ik_tol_pos,
                                tol_angle=self.cfg.ik_tol_angle)
        return self.kin.ik(pos, pitch, roll, q, tol_pos=self.cfg.ik_tol_pos, tol_angle=self.cfg.ik_tol_angle)

    def feasible_pitch(self, pos, pitch, roll, q):
        """Steepest pitch at or after `pitch` (tilting toward horizontal) that has an IK solution, 1 deg resolution.
        Coarse 5 deg steps tracked from q first (fast), then refined; reseeded IK only if tracking finds nothing."""
        direction = 1.0 if pitch < 0 else -1.0
        coarse = self.cfg.pitch_step
        n = int(round(self.cfg.pitch_tolerance / coarse))
        for track in (True, False):
            for i in range(n + 1):
                p = pitch + direction * i * coarse
                if self._ik(pos, p, roll, q, track).ok:
                    for j in range(1, int(round(math.degrees(coarse)))):
                        fine = p - direction * j * math.radians(1)
                        if i == 0 or not self._ik(pos, fine, roll, q, track).ok:
                            return p - direction * (j - 1) * math.radians(1)
                    return p - direction * (int(round(math.degrees(coarse))) - 1) * math.radians(1)
        return None

    def cartesian(self, q_start, pose_a, pose_b, v, kind="cartesian"):
        """pose = (pos, pitch, roll), both ends reachable. Interpolates position in cylindrical coordinates and
        pitch/roll linearly, solves IK on each sample. If the middle of the path is out of reach, the pitch gets a
        smooth extra tilt (sin bump, 0 at both ends) instead of per-sample jumps."""
        ca, cb = self.cyl(pose_a[0]), self.cyl(pose_b[0])
        dphi = wrap(cb[1] - ca[1])
        rho = 0.5 * (ca[0] + cb[0])
        length = math.sqrt((cb[0] - ca[0]) ** 2 + (rho * dphi) ** 2 + (cb[2] - ca[2]) ** 2)
        n = max(2, int(math.ceil(length / self.cfg.sample_step)) + 1)
        direction = 1.0 if pose_b[1] < 0 else -1.0
        bumps = [0.0] + [i * self.cfg.pitch_step for i in range(1, int(round(self.cfg.pitch_tolerance / self.cfg.pitch_step)) + 1)]
        error = None
        for bump in bumps:
            try:
                qs = self._solve_path(q_start, ca, dphi, cb, pose_a, pose_b, n, direction * bump)
                break
            except Unreachable as e:
                error = error or e
        else:
            raise error
        return Segment(qs, self._duration(qs, length, v), kind)

    def _solve_path(self, q_start, ca, dphi, cb, pose_a, pose_b, n, bump):
        qs, q = [np.array(q_start, float)], np.array(q_start, float)
        for s in np.linspace(0.0, 1.0, n)[1:]:
            c = np.array([ca[0] + s * (cb[0] - ca[0]), ca[1] + s * dphi, ca[2] + s * (cb[2] - ca[2])])
            pitch = pose_a[1] + s * wrap(pose_b[1] - pose_a[1]) + bump * math.sin(math.pi * s)
            roll = pose_a[2] + s * wrap(pose_b[2] - pose_a[2])
            r = self._ik(self.from_cyl(c), pitch, roll, q, track=True)
            if not r.ok:
                raise Unreachable(f"no IK at {np.round(self.from_cyl(c), 3).tolist()} "
                                  f"(error {r.pos_error * 1000:.0f} mm, {math.degrees(r.angle_error):.0f} deg)")
            if np.max(np.abs(r.q - q)) > self.cfg.max_sample_jump:
                raise Unreachable("the path needs a configuration flip")
            q = r.q
            qs.append(q)
        return np.array(qs)

    def joint(self, q_start, q_end, v, kind="joint"):
        q_start, q_end = np.asarray(q_start, float), np.asarray(q_end, float)
        n = max(2, int(math.ceil(np.max(np.abs(q_end - q_start)) / 0.02)) + 1)
        qs = np.linspace(q_start, q_end, n)
        length, pts = self._path_length(qs)
        if np.min(pts[:, 2]) < self.cfg.min_z + 0.01 and np.min(pts[:, 2]) < min(pts[0, 2], pts[-1, 2]) - 0.005:
            raise Unreachable("joint move would sweep through the table")
        seg = Segment(qs, self._duration(qs, length, v, joint_move=True), kind)
        seg.joint_space = True
        return seg

    def plan_pose(self, q_start, target, v):
        """target = (pos, pitch, roll). Returns (segments, pitch actually used at the target).

        The SO-101 has 5 joints, so a tool pointing straight down is only reachable on part of the table. Every
        path corner (lift top, point above the target, target) gets the steepest reachable pitch at or after the
        requested one, up to pitch_tolerance. Within a segment the pitch changes linearly between its corners."""
        cfg = self.cfg
        pos = np.asarray(target[0], float)
        roll = target[2]
        if pos[2] < cfg.min_z:
            raise Unreachable(f"target below the table ({pos[2]:.3f} m)")
        pitch_t = self.feasible_pitch(pos, target[1], roll, q_start)
        if pitch_t is None:
            r = self.kin.ik(pos, target[1], roll, q_start)
            raise Unreachable(f"no IK for the target (error {r.pos_error * 1000:.0f} mm, "
                              f"{math.degrees(r.angle_error):.0f} deg)")
        goal = (pos, pitch_t, roll)
        p0, pitch0, roll0 = self.kin.fk(q_start)
        start = (p0, pitch0, roll0)
        c0 = self.cyl(p0)
        tool_like = c0[0] >= cfg.cartesian_min_radius and p0[2] < 0.25 and pitch0 < -0.3
        if np.linalg.norm(p0[:2] - pos[:2]) < cfg.direct_distance and tool_like:
            return self.blend([self.move(q_start, start, goal, v, "direct")], v), pitch_t

        safe = max(cfg.safe_z, pos[2])
        if safe - pos[2] < cfg.min_vertical:
            safe = pos[2]
        above_pos = np.array([pos[0], pos[1], safe])
        pitch_a = self.feasible_pitch(above_pos, pitch_t, roll, q_start)
        if pitch_a is None:
            raise Unreachable(f"no IK above the target at {np.round(above_pos, 3).tolist()}")
        above = (above_pos, pitch_a, roll)
        segments = []
        q = np.array(q_start, float)
        if tool_like:
            lift_z = max(safe, p0[2])
            lifted = start
            if lift_z - p0[2] >= cfg.min_vertical:
                top = np.array([p0[0], p0[1], lift_z])
                pitch_l = self.feasible_pitch(top, pitch0, roll0, q)
                if pitch_l is None:
                    raise Unreachable(f"cannot lift to {lift_z:.2f} m here")
                lifted = (top, pitch_l, roll0)
                segments.append(self.move(q, start, lifted, v, "lift"))
                q = segments[-1].q[-1]
            segments.append(self.move(q, lifted, above, v, "travel"))
        else:
            segments.append(self._joint_to(q, above, v))
        q = segments[-1].q[-1]
        if safe - pos[2] > 0.002:
            segments.append(self.move(q, above, goal, v, "descend"))
        return self.blend(segments, v), pitch_t

    def move(self, q, pose_a, pose_b, v, kind):
        """Straight tool path if one exists on the current IK branch, else a joint move to pose_b (smooth too, but
        the tool path bends a little)."""
        try:
            return self.cartesian(q, pose_a, pose_b, v, kind)
        except Unreachable:
            seg = self._joint_to(q, pose_b, v)
            seg.kind = kind + "_joint"
            return seg

    def _joint_to(self, q, pose, v):
        r = self._ik(pose[0], pose[1], pose[2], q)
        if not r.ok:
            raise Unreachable(f"no IK at {np.round(pose[0], 3).tolist()}")
        return self.joint(q, r.q, v, "travel")

    def waypoints(self, segments):
        pts = []
        for seg in segments:
            idx = sorted(set(np.linspace(0, len(seg.q) - 1, min(len(seg.q), 12)).astype(int).tolist()))
            for i in idx:
                p = self.kin.fk(seg.q[i])[0]
                if not pts or np.linalg.norm(p - pts[-1]) > 0.003:
                    pts.append(p)
        return [[round(float(v), 4) for v in p] for p in pts]
