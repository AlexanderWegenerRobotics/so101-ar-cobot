"""Run the real SafetySupervisor on random hand motion with tracking dropouts, and check every decision against the
rule the UPPAAL model assumes. If the code and the model ever disagree, this fails.

No bus and no sim world: the node is driven directly, with a fake clock, a fake subscriber and the real forward
kinematics. Each run moves one hand toward and away from the arm at up to hand_speed while the arm moves, drops
tracking at random (single frames, flicker, long gaps, the whole hand stream going silent), lets arm/state go
stale now and then, and sends arm goals.

Checks every tick:
  rule      zone, speed_scale and stop follow from the reported worst-case clearance d exactly as in the model,
            including the hysteresis (d < S stop, still stopped and d < S + hysteresis stop, d < S + slow_buffer
            slow with scale (d - S) / slow_buffer, otherwise clear; stale arm/state stop; no hand yet clear)
  update    d == clearance(arm now, hand at its last real detection) - hand_speed * time since that detection,
            i.e. only tracked messages reset the age (flicker cannot keep old data alive)
  bound     d <= true clearance (the model's invariant: the supervisor never thinks the hand is farther than it is)
  raw       raw_distance == clearance(arm now, hand at its last real detection), without the age term
  goals     every arm/goal is forwarded to arm/command unchanged (the controller holds on stop)

    python supervisor/uppaal/check_conformance.py                 200 runs of 30 s
    python supervisor/uppaal/check_conformance.py --runs 20 --seed 1
    python supervisor/uppaal/check_conformance.py --mutate        break the age rule on purpose, the check must fail
"""
import argparse
import sys
from pathlib import Path
from unittest import mock

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import supervisor.node as sup  # noqa: E402
from interfaces.messages import HAND_JOINTS, ArmGoal, ArmState, HandState, Topic  # noqa: E402

TOL_D = 1e-4
TOL_SCALE = 1e-3
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
HOME = {"shoulder_pan": 0.0, "shoulder_lift": -85.0, "elbow_flex": 85.0, "wrist_flex": 60.0, "wrist_roll": 0.0}


class FakeSub:
    def __init__(self):
        self.queue = []

    def drain(self):
        msgs, self.queue = self.queue, []
        return msgs


class Clock:
    t = 1000.0

    def time(self):
        return self.t


def make_node(clock):
    node = sup.SafetySupervisor(bus_cfg=None)
    node.sub = FakeSub()
    node.out = []
    node.publish = lambda topic, msg: node.out.append((topic, msg))
    with mock.patch.object(sup.time, "time", clock.time):
        node.setup()
    return node


def clearance(node, joints, hand):
    pts = node.arm_points(joints)
    return float(np.linalg.norm(pts[:, None, :] - hand[None, :, :], axis=2).min()) - node.radii


def run(node, clock, rng, seconds, mutate, seen):
    S, slow, hyst, vh = node.S, node.slow, node.hysteresis, node.hand_speed
    dt = 1.0 / node.rate_hz
    node.hands, node.arm_state, node.arm_time, node.stopped = {}, None, None, False
    joints = dict(HOME)
    arm_dir = {j: rng.uniform(-1, 1) for j in JOINTS}
    start = node.arm_points(joints)[-1]
    offset = rng.normal(size=3)
    center = start + offset / np.linalg.norm(offset) * rng.uniform(0.4, 0.7)
    cloud_offsets = rng.normal(0.0, 0.02, size=(len(HAND_JOINTS), 3))
    approach, phase_left = True, 0
    last_seen, arm_sent = None, None
    mode, mode_left, arm_gap = "ok", 0, 0
    exp_stopped = False
    errors = []

    for k in range(int(seconds / dt)):
        clock.t += dt
        for j in JOINTS:
            joints[j] = float(np.clip(joints[j] + arm_dir[j] * 20.0 * dt, HOME[j] - 30, HOME[j] + 30))
            if abs(joints[j] - HOME[j]) >= 30:
                arm_dir[j] = -arm_dir[j]
        if arm_gap <= 0 and rng.random() < 0.005:
            arm_gap = int(rng.integers(3, 15))
        if arm_gap > 0:
            arm_gap -= 1
        else:
            node.sub.queue.append((Topic.ARM_STATE, ArmState(joints=dict(joints), stamp=clock.t)))
            arm_sent = (dict(joints), clock.t)

        tip = node.arm_points(joints)[-1]
        to_tip = tip - center
        toward = to_tip / max(np.linalg.norm(to_tip), 1e-9)
        if phase_left <= 0:
            approach, phase_left = not approach, int(rng.integers(15, 90))
        phase_left -= 1
        direction = (toward if approach else -toward) + rng.normal(scale=0.5, size=3)
        center = center + direction / max(np.linalg.norm(direction), 1e-9) * vh * dt * rng.uniform(0.0, 1.0)
        cloud = center + cloud_offsets

        if mode_left <= 0:
            mode = rng.choice(["ok", "ok", "ok", "lost", "flicker", "silent"])
            mode_left = int(rng.integers(1, 40))
        mode_left -= 1
        if mode == "ok" or (mode == "flicker" and rng.random() < 0.5):
            msg = HandState(side="right", tracked=True, joints=cloud.tolist(), stamp=clock.t)
            last_seen = (cloud.copy(), clock.t)
        elif mode == "silent":
            msg = None
        else:
            msg = HandState(side="right", tracked=False, joints=[], stamp=clock.t)
        if msg is not None:
            node.sub.queue.append((Topic.HAND, msg))
        goals = []
        if rng.random() < 0.1:
            goals = [ArmGoal(f"g{k}", "pose")]
            node.sub.queue += [(Topic.ARM_GOAL, g) for g in goals]

        node.out = []
        with mock.patch.object(sup.time, "time", clock.time):
            if mutate and msg is not None and not msg.tracked and "right" in node.hands:
                node.hands["right"] = (node.hands["right"][0], clock.t)
            node.step()

        safety = [m for t, m in node.out if t == Topic.SAFETY][-1]
        forwarded = [m for t, m in node.out if t == Topic.ARM_COMMAND]
        d = safety.min_distance
        where = f"t={k * dt:6.2f}s"
        seen[safety.zone] += 1

        if forwarded != goals:
            errors.append(f"{where} goals: sent {len(goals)}, forwarded {len(forwarded)}")
        if safety.stop != (safety.zone == "stop"):
            errors.append(f"{where} rule: zone {safety.zone} but stop = {safety.stop}")

        if arm_sent is None or clock.t - arm_sent[1] > node.arm_max_age:
            seen["arm stale"] += 1
            exp_stopped = True
            if safety.zone != "stop" or safety.speed_scale != 0.0:
                errors.append(f"{where} rule: arm/state stale but zone {safety.zone}")
            continue
        if last_seen is None:
            exp_stopped = False
            if safety.zone != "clear" or safety.speed_scale != 1.0:
                errors.append(f"{where} rule: no hand seen yet but zone {safety.zone}")
            continue
        if last_seen[1] < clock.t:
            seen["tracking lost"] += 1

        arm_joints = arm_sent[0]
        d_model = clearance(node, arm_joints, last_seen[0]) - vh * (clock.t - last_seen[1])
        d_true = clearance(node, arm_joints, cloud)
        if d is None or abs(d - d_model) > TOL_D:
            errors.append(f"{where} update: code d = {d}, model d = {d_model:.4f} ({mode})")
            continue
        raw_model = clearance(node, arm_joints, last_seen[0])
        if safety.raw_distance is None or abs(safety.raw_distance - raw_model) > TOL_D:
            errors.append(f"{where} raw: code raw = {safety.raw_distance}, model raw = {raw_model:.4f} ({mode})")
        if d > d_true + TOL_D:
            errors.append(f"{where} bound: d = {d:.4f} > true clearance {d_true:.4f} ({mode})")

        if d < S or (exp_stopped and d < S + hyst):
            want, scale = "stop", 0.0
        elif d < slow:
            want, scale = "slow", (d - S) / (slow - S)
        else:
            want, scale = "clear", 1.0
        on_edge = min(abs(d - S), abs(d - S - hyst), abs(d - slow)) < TOL_D
        exp_stopped = want == "stop" if not on_edge else safety.zone == "stop"
        if not on_edge and (safety.zone != want or abs(safety.speed_scale - scale) > TOL_SCALE):
            errors.append(f"{where} rule: d = {d:.4f}, S = {S:.4f}, expected {want} {scale:.3f}, "
                          f"got {safety.zone} {safety.speed_scale:.3f}")
    return errors


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=int, default=200)
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--mutate", action="store_true")
    args = ap.parse_args()

    clock = Clock()
    node = make_node(clock)
    rng = np.random.default_rng(args.seed)
    seen = {"clear": 0, "slow": 0, "stop": 0, "tracking lost": 0, "arm stale": 0}
    ticks, failed, first = 0, 0, []
    for r in range(args.runs):
        errors = run(node, clock, rng, args.seconds, args.mutate, seen)
        ticks += int(args.seconds * node.rate_hz)
        if errors:
            failed += 1
            first += [f"run {r}: {e}" for e in errors[:3]]
    print(f"S = {node.S:.3f} m, {args.runs} runs, {ticks} ticks, {failed} runs with mismatches")
    print("ticks: " + ", ".join(f"{k} {v}" for k, v in seen.items()))
    for line in first[:10]:
        print("  " + line)
    print("code matches the model rule" if not failed else "MISMATCH between code and model")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
