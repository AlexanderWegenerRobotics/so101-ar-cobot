"""Who does which cube, and in which order: robot and human share the sorting work.

The human is several times faster than the SO-101 (about 3 s against 15 s per cube), so a plain weighted sum of
"seconds until done" and "seconds of human work" only moves work to the robot near one end of the weight. Instead:

1. Pareto front. For every human budget K (the human does at most K new cubes), solve exactly

       min  makespan + w_interference * interference + w_tie * human_time

   Each K gives one plan; together they trace the trade-off between finishing fast and sparing the human.
   (A weighted sum makespan + w * human_time would only find the corners of that front, because the robot is so
   much slower that every plan in between lies on a straight line between two corners.)
2. alpha in [0, 1] picks the point on the front by human workload: the robot takes about alpha of the cubes it
   could do, so the human budget is K = forced + round((1 - alpha) * doable), where forced are the cubes only the
   human may do (heavy, out of reach, locked to the human) and doable the rest. alpha = 0 is the fastest plan,
   alpha = 1 the plan with the least human work. A weighted score (linear or log) over the front does not work
   here: the front is almost a straight line (every cube moved to the robot costs about 16 s of total time and
   saves the human about 3 s), so any weighting jumps from one end to the other.

Interference: for a robot job r and a human job h, conflict[r, h] in [0, 1] is the expected slowdown the safety
supervisor would impose while both run: its speed ramp (1 at slow_distance, 0 at stop_distance) applied to the
distance between the human hand path (around the base, cube -> bin) and the robot (tool path and arm). Running the
two jobs at the same time costs w_interference * conflict * overlap seconds.

Solver: exact search over the joint schedule with memoization (dynamic programming over states). Whichever agent
is free first picks its next job, or stops for good; the human may also wait until the robot finishes its current
job (costs time, avoids interference). A state is

    (jobs left, free agent, where each agent is, the other agent's current job, its remaining time)

with the remaining time rounded to `time_step` for the memo key, so equal situations reached through different
orders are solved once. 8 open jobs: about 18k states, well under 100 ms per weight in Python.
"""
import math
from dataclasses import dataclass, field
from functools import lru_cache

import numpy as np

ROBOT, HUMAN = "robot", "human"


@dataclass
class Job:
    tag_id: int
    pos: tuple
    bin_pos: tuple
    robot_ok: bool = True
    robot_reason: str = ""
    lock: str | None = None
    robot_first: bool = False


@dataclass
class Agent:
    pos: tuple
    busy_tag: int | None = None
    busy_bin: tuple | None = None
    busy_remaining: float = 0.0
    available: bool = True


@dataclass
class AllocatorConfig:
    alpha: float = 0.5
    w_interference: float = 3.0
    w_tie: float = 0.05
    robot_speed: float = 0.12
    robot_pick_time: float = 1.8
    robot_place_time: float = 0.8
    robot_pan_axis: tuple = (0.0388, 0.0)
    human_speed: float = 0.45
    human_pick_time: float = 0.8
    human_place_time: float = 0.5
    slow_distance: float = 0.25
    stop_distance: float = 0.10
    keep_out_radius: float = 0.14
    wait_min_conflict: float = 0.15
    time_step: float = 0.25


@dataclass
class Assignment:
    tag_id: int
    owner: str
    order: int
    start: float
    end: float
    why: str = ""


@dataclass
class Plan:
    assignments: dict = field(default_factory=dict)
    makespan: float = 0.0
    robot_time: float = 0.0
    human_time: float = 0.0
    interference: float = 0.0
    cost: float = 0.0
    weight: float = 0.0
    states: int = 0
    solve_ms: float = 0.0
    front: list = field(default_factory=list)
    waits: list = field(default_factory=list)

    def sequence(self, owner):
        return [a.tag_id for a in sorted(self.assignments.values(), key=lambda a: a.order) if a.owner == owner]


def detour(a, b, center, radius, arc_step=0.35):
    """2D path from a to b that does not enter the circle (center, radius) around the robot base: straight if the
    segment stays outside, otherwise in to the circle, along the shorter arc, and out again."""
    a, b, c = (np.asarray(v[:2], float) for v in (a, b, center))
    ab = b - a
    t = float(np.clip(np.dot(c - a, ab) / max(np.dot(ab, ab), 1e-12), 0.0, 1.0))
    if np.linalg.norm(a + t * ab - c) >= radius:
        return [a, b]
    r = radius * 1.05
    ta = math.atan2(*(a - c)[::-1])
    tb = math.atan2(*(b - c)[::-1])
    dt = (tb - ta + math.pi) % (2 * math.pi) - math.pi
    n = max(2, int(abs(dt) / arc_step) + 2)
    arc = [c + r * np.array([math.cos(ta + s * dt), math.sin(ta + s * dt)]) for s in np.linspace(0, 1, n)]
    pts = [a] if np.linalg.norm(a - c) > r else []
    pts += arc
    if np.linalg.norm(b - c) > r:
        pts.append(b)
    else:
        pts[-1] = b
    if np.linalg.norm(a - c) <= r:
        pts[0] = a
    return pts


def path_length(points):
    return float(sum(np.linalg.norm(np.asarray(q) - np.asarray(p)) for p, q in zip(points[:-1], points[1:])))


def _cyl(p, axis):
    d = np.asarray(p[:2], float) - axis
    return math.hypot(*d), math.atan2(d[1], d[0]), p[2] if len(p) > 2 else 0.0


class Allocator:
    def __init__(self, cfg=None):
        self.cfg = cfg or AllocatorConfig()

    # ----- time and conflict models

    def robot_travel(self, a, b):
        """Tool path length like the controller plans it: lift, arc around the pan axis, descend."""
        ax = np.asarray(self.cfg.robot_pan_axis)
        ra, pa, _ = _cyl(a, ax)
        rb, pb, _ = _cyl(b, ax)
        dphi = abs((pb - pa + math.pi) % (2 * math.pi) - math.pi)
        horiz = math.hypot(rb - ra, 0.5 * (ra + rb) * dphi)
        length = horiz + 0.12
        return 1.875 * length / self.cfg.robot_speed

    def robot_job_time(self, start, job):
        c = self.cfg
        return self.robot_travel(start, job.pos) + c.robot_pick_time + self.robot_travel(job.pos, job.bin_pos) \
            + c.robot_place_time

    def human_route(self, start, job):
        """Hand path (2D) start -> cube -> bin, around the keep-out circle at the robot base."""
        c, r = self.cfg.robot_pan_axis, self.cfg.keep_out_radius
        reach = detour(start, job.pos, c, r)
        carry = detour(job.pos, job.bin_pos, c, r)
        return reach, carry

    def human_job_time(self, start, job):
        c = self.cfg
        reach, carry = self.human_route(start, job)
        return path_length(reach) / c.human_speed + c.human_pick_time + path_length(carry) / c.human_speed \
            + c.human_place_time

    def conflict(self, r_job, h_job):
        """Expected slowdown of the robot while both jobs run, in [0, 1]: for every point of the human hand path
        (around the base, cube -> bin) the supervisor's speed ramp is applied to its distance from the robot
        (tool path cube -> bin as the controller arcs it, plus points along the arm back to the pan axis), and the
        result is averaged. 1 means the arm would stand still the whole time."""
        c = self.cfg
        axis = np.asarray(c.robot_pan_axis, float)
        ra, pa, _ = _cyl(r_job.pos, axis)
        rb, pb, _ = _cyl(r_job.bin_pos, axis)
        dphi = (pb - pa + math.pi) % (2 * math.pi) - math.pi
        robot_pts = []
        for s in np.linspace(0, 1, 10):
            r, ph = ra + s * (rb - ra), pa + s * dphi
            tool = axis + r * np.array([math.cos(ph), math.sin(ph)])
            robot_pts += [axis + f * (tool - axis) for f in (0.35, 0.7, 1.0)]
        robot_pts = np.array(robot_pts)
        carry = detour(h_job.pos, h_job.bin_pos, c.robot_pan_axis, c.keep_out_radius)
        human_pts = [np.asarray(h_job.pos[:2], float)] * 3
        for p, q in zip(carry[:-1], carry[1:]):
            human_pts += list(np.linspace(p, q, 6))
        human_pts = np.array(human_pts)
        d = np.min(np.linalg.norm(human_pts[:, None, :] - robot_pts[None, :, :], axis=2), axis=1)
        slow = np.clip((c.slow_distance - d) / (c.slow_distance - c.stop_distance), 0.0, 1.0)
        return float(np.mean(slow))

    # ----- solver

    def plan(self, jobs, robot, human, alpha=None, front=False):
        """Fastest plan within the human budget set by alpha. front=True also solves every other budget and stores
        the whole Pareto front in plan.front (for the log and plots, about 8x the time)."""
        import time
        t0 = time.perf_counter()
        alpha = self.cfg.alpha if alpha is None else alpha
        jobs = list(jobs)
        forced = sum(1 for j in jobs if not j.robot_ok or j.lock == HUMAN or not robot.available)
        doable = len(jobs) - forced
        cap = forced + int(round((1 - alpha) * doable))
        chosen = None
        for k in range(cap, len(jobs) + 1):
            try:
                chosen = self.solve(jobs, robot, human, self.cfg.w_tie, k)
                break
            except ValueError:
                continue
        if chosen is None:
            raise ValueError("no feasible plan")
        states = chosen.states
        if front:
            for k in range(forced, len(jobs) + 1):
                if k == chosen.weight:
                    p = chosen
                else:
                    try:
                        p = self.solve(jobs, robot, human, self.cfg.w_tie, k)
                    except ValueError:
                        continue
                    states += p.states
                chosen.front.append({"human_cap": k, "makespan": round(p.makespan, 2),
                                     "human_time": round(p.human_time, 2),
                                     "interference": round(float(p.interference), 2),
                                     "robot_jobs": len(p.sequence(ROBOT)), "human_jobs": len(p.sequence(HUMAN)),
                                     "chosen": p is chosen})
        chosen.states = states
        chosen.solve_ms = (time.perf_counter() - t0) * 1000
        return chosen

    def solve(self, jobs, robot, human, w_human, human_cap=None):
        """Exact min of makespan + w_human * human_time + w_interference * interference, with the human doing at
        most human_cap new jobs."""
        import time
        t0 = time.perf_counter()
        c = self.cfg
        jobs = list(jobs)
        n = len(jobs)
        idx = {j.tag_id: i for i, j in enumerate(jobs)}
        R_BUSY, H_BUSY, FREE, DONE = n, n + 1, -2, -1
        r_start = robot.busy_bin if robot.busy_tag is not None else robot.pos
        h_start = human.busy_bin if human.busy_tag is not None else human.pos
        place = lambda pid, start: start if pid < 0 else jobs[pid].bin_pos
        can_r = [j.robot_ok and j.lock != HUMAN for j in jobs]
        can_h = [j.lock != ROBOT for j in jobs]
        dur = {}
        for pid in range(-1, n):
            for k, j in enumerate(jobs):
                dur[ROBOT, pid, k] = self.robot_job_time(place(pid, r_start), j) if can_r[k] else math.inf
                dur[HUMAN, pid, k] = self.human_job_time(place(pid, h_start), j) if can_h[k] else math.inf
        ext = jobs + [self._busy_job(robot), self._busy_job(human)]
        conflict = np.zeros((n + 2, n + 2))
        for a in range(n + 2):
            for b in range(n + 2):
                if ext[a] is not None and ext[b] is not None:
                    conflict[a, b] = self.conflict(ext[a], ext[b])
        first = {k for k, j in enumerate(jobs) if j.robot_first and can_r[k]}
        w_m, alpha, w_i, step = 1.0, w_human, c.w_interference, c.time_step
        other_of = {ROBOT: HUMAN, HUMAN: ROBOT}

        def moves(state):
            """All options of the free agent: (cost of the interval until the next decision, next state, info)."""
            mask, free, r_pos, h_pos, other_job, bucket, r_done, h_done, hb = state
            o = bucket * step
            other = other_of[free]
            busy_other = other_job >= 0
            if mask == 0:
                yield o * (w_m + (alpha if other == HUMAN and busy_other else 0.0)), None, None
                return
            done_self = r_done if free == ROBOT else h_done
            options_k = []
            if not done_self:
                pos = r_pos if free == ROBOT else h_pos
                for k in range(n):
                    if not mask >> k & 1:
                        continue
                    if free == ROBOT and first and k not in first and any(mask >> f & 1 for f in first):
                        continue
                    d = dur[free, pos, k]
                    if not math.isfinite(d) or (free == HUMAN and hb <= 0):
                        continue
                    nhb = hb - 1 if free == HUMAN else hb
                    options_k.append(k)
                    nmask = mask & ~(1 << k)
                    nr, nh = (k, h_pos) if free == ROBOT else (r_pos, k)
                    info = ("job", k, d)
                    if other_job == FREE or (busy_other and o <= d):
                        both = o if busy_other else 0.0
                        pair = ((k, other_job) if free == ROBOT else (other_job, k)) if busy_other else None
                        cost = both * (w_m + alpha + (w_i * conflict[pair] if pair else 0.0))
                        nxt = (nmask, other, nr, nh, k, int(round((d - both) / step)), r_done, h_done, nhb)
                    elif busy_other:
                        pair = (k, other_job) if free == ROBOT else (other_job, k)
                        cost = d * (w_m + alpha + w_i * conflict[pair])
                        nxt = (nmask, free, nr, nh, other_job, int(round((o - d) / step)), r_done, h_done, nhb)
                    else:
                        cost = d * (w_m + (alpha if free == HUMAN else 0.0))
                        nxt = (nmask, free, nr, nh, DONE, 0, r_done, h_done, nhb)
                    yield cost, nxt, info
            if free == HUMAN and not done_self and busy_other and o > 0 and hb > 0 and options_k \
                    and min(conflict[other_job, k] for k in options_k) >= c.wait_min_conflict:
                yield o * w_m, (mask, ROBOT, r_pos, h_pos, FREE, 0, r_done, h_done, hb), ("wait", o)
            rd, hd = (True, h_done) if free == ROBOT else (r_done, True)
            if other_job == DONE:
                return
            if busy_other:
                cost = o * (w_m + (alpha if other == HUMAN else 0.0))
            else:
                cost = 0.0
            yield cost, (mask, other, r_pos, h_pos, DONE, 0, rd, hd, hb), ("stop",)

        states = [0]

        @lru_cache(maxsize=None)
        def solve(state):
            states[0] += 1
            best = (math.inf, None)
            for cost, nxt, info in moves(state):
                total = cost + (solve(nxt)[0] if nxt is not None else 0.0)
                if total < best[0]:
                    best = (total, (nxt, info))
            return best

        r_busy = robot.busy_tag is not None
        h_busy = human.busy_tag is not None
        r_rem = robot.busy_remaining if r_busy else 0.0
        h_rem = human.busy_remaining if h_busy else 0.0
        r_done0, h_done0 = not robot.available, not human.available
        pre = 0.0
        if r_rem <= h_rem:
            free, wait, other_rem = ROBOT, r_rem, h_rem - r_rem
            other_job = H_BUSY if h_busy and other_rem > 0 else (DONE if h_done0 else FREE)
        else:
            free, wait, other_rem = HUMAN, h_rem, r_rem - h_rem
            other_job = R_BUSY if r_busy and other_rem > 0 else (DONE if r_done0 else FREE)
        if wait > 0:
            both = r_busy and h_busy
            pre = wait * (w_m + (alpha if h_busy else 0.0) + (w_i * conflict[R_BUSY, H_BUSY] if both else 0.0))
        if (r_done0 if free == ROBOT else h_done0):
            free, other_job = other_of[free], DONE
        root = ((1 << n) - 1, free, -1, -1, other_job, int(round(other_rem / step)), r_done0, h_done0,
                n if human_cap is None else human_cap)
        cost, _ = solve(root)
        if not math.isfinite(cost):
            raise ValueError("no feasible plan: some job nobody may do")

        plan = Plan(cost=cost + pre, weight=human_cap if human_cap is not None else -1)
        order = {ROBOT: 0, HUMAN: 0}
        if r_busy:
            plan.assignments[robot.busy_tag] = Assignment(robot.busy_tag, ROBOT, 0, 0.0, r_rem, "in progress")
            order[ROBOT] = 1
        if h_busy:
            plan.assignments[human.busy_tag] = Assignment(human.busy_tag, HUMAN, 0, 0.0, h_rem, "in progress")
            order[HUMAN] = 1
        plan.robot_time, plan.human_time = r_rem, h_rem
        now, state = wait, root
        while state is not None:
            _, choice = solve(state)
            if choice is None:
                break
            nxt, info = choice
            if info is None:
                break
            mask, free_, r_pos, h_pos, oj, bucket, _, _, _ = state
            if info[0] == "job":
                k, d = info[1], info[2]
                plan.assignments[jobs[k].tag_id] = Assignment(jobs[k].tag_id, free_, order[free_], now, now + d)
                order[free_] += 1
                if free_ == ROBOT:
                    plan.robot_time += d
                else:
                    plan.human_time += d
                if oj >= 0:
                    pair = (k, oj) if free_ == ROBOT else (oj, k)
                    plan.interference += conflict[pair] * min(d, bucket * step)
                if oj == FREE or (oj >= 0 and bucket * step <= d):
                    now += bucket * step if oj >= 0 else 0.0
                elif oj >= 0:
                    now += d
                else:
                    now += d
            elif info[0] == "wait":
                plan.waits.append((now, now + info[1]))
                now += info[1]
            else:
                now += bucket * step if oj >= 0 else 0.0
            state = nxt
        if h_busy and r_busy:
            plan.interference += conflict[R_BUSY, H_BUSY] * min(r_rem, h_rem)
        plan.makespan = max([a.end for a in plan.assignments.values()] + [0.0])
        plan.states = states[0]
        plan.solve_ms = (time.perf_counter() - t0) * 1000
        for job in jobs:
            a = plan.assignments.get(job.tag_id)
            if a is None or a.why:
                continue
            if a.owner == HUMAN:
                a.why = job.robot_reason if not job.robot_ok else "operator" if job.lock == HUMAN else \
                    "robot unavailable" if not robot.available else "planner"
            else:
                a.why = "operator" if job.lock == ROBOT or job.robot_first else "planner"
        return plan

    @staticmethod
    def _busy_job(agent):
        if agent.busy_tag is None:
            return None
        return Job(agent.busy_tag, agent.pos, agent.busy_bin)
