"""Safety supervisor: speed and separation monitoring between the operator's hands and the arm.

Every tick: arm link points from arm/state (forward kinematics on the MuJoCo model, points along every moving link
from the shoulder out; the fixed base is not counted) and the joints of every hand at its last real detection.
For each hand the worst-case clearance is

    d = distance(arm now, hand when last seen) - link_radius - finger_radius - hand_speed * age

where age is the time since that hand was last tracked. While tracking works age is about 0; when tracking drops
out the hand could be anywhere in a sphere that grows at hand_speed, so the arm keeps going if the hand was far and
stops if it was close. A lost hand is never forgotten, only a new tracked detection resets age.

The smallest d over all hands picks the zone, with the separation distance S from config/supervisor.yaml:

    d > S + slow_buffer         clear   speed_scale 1
    S < d < S + slow_buffer     slow    speed_scale falls linearly to 0 at S
    d < S                       stop    stop = true, the controller holds; released above S + hysteresis
    no hand seen yet            clear
    arm/state older than arm_max_age, or none yet: stop

min_distance in safety is that worst-case d, used for the zones. raw_distance leaves out the hand_speed * age
term, i.e. it is the distance to where each hand was last seen; that is what the trial log records.

Goals from arm/goal go to arm/command unchanged (the controller applies the scale and the stop). Zone changes are
logged with the distance, which is the data for the safety study.
"""
import time
from pathlib import Path

import mujoco
import numpy as np
import yaml

from interfaces.messages import ArmGoal, ArmState, HandState, SafetyState, Topic
from interfaces.node import Node

CONFIG = Path(__file__).resolve().parents[1] / "config" / "supervisor.yaml"
ARM_BODIES = ["upper_arm", "lower_arm", "wrist", "gripper", "moving_jaw_so101_v1"]


def load_config(params=None):
    cfg = yaml.safe_load(CONFIG.read_text())
    cfg.update(params or {})
    return cfg


def safe_distance(c):
    t_react = c["latency"] + 1.0 / c["rate_hz"]
    t_stop = c["stop_time"]
    return (c["hand_speed"] * (t_react + t_stop)
            + c["arm_speed"] * t_react
            + c["arm_speed"] * t_stop / 2
            + c["margin"])


class SafetySupervisor(Node):
    name = "supervisor"
    rate_hz = 30.0
    subscribe = [Topic.ARM_GOAL, Topic.ARM_STATE, Topic.HAND]

    def setup(self):
        from controls.kinematics import ARM_JOINTS, SO101Kinematics
        from hardware import load_robot_config
        cfg = load_robot_config()
        self.kin = SO101Kinematics(cfg.get("sim", {}).get("joint_map", {}))
        self.arm_joints = ARM_JOINTS
        self.bodies = [self.kin.model.body(b).id for b in ARM_BODIES]
        self.parents = [self.kin.model.body_parentid[b] for b in self.bodies]
        self.cfg = c = load_config(self.params)
        self.rate_hz = float(c["rate_hz"])
        self.S = safe_distance(c)
        self.slow = self.S + c["slow_buffer"]
        self.hysteresis = c["hysteresis"]
        self.radii = c["link_radius"] + c["finger_radius"]
        self.hand_speed = c["hand_speed"]
        self.arm_max_age = c["arm_max_age"]
        self.enabled = c["enabled"]
        self.arm_state = None
        self.arm_time = None
        self.hands = {}
        self.zone = "clear"
        self.stopped = False
        self.zone_since = time.time()
        self.log.info(f"safety supervisor: separation distance S = {self.S:.3f} m, slow below {self.slow:.3f} m"
                      + ("" if self.enabled else " (DISABLED, monitoring only)"))

    def arm_points(self, joints):
        q = self.kin.robot_to_q(joints)
        d = self.kin.data
        d.qpos[self.kin.qadr] = q
        mujoco.mj_kinematics(self.kin.model, d)
        pts = []
        for b, parent in zip(self.bodies, self.parents):
            a, c = d.xpos[parent], d.xpos[b]
            pts += [a + s * (c - a) for s in np.linspace(0, 1, 5)]
        g, tip = d.xpos[self.kin.model.body("gripper").id], d.site_xpos[self.kin.site]
        pts += [g + s * (tip - g) for s in np.linspace(0, 1, 4)]
        return np.array(pts)

    def worst_case(self, now):
        arm = self.arm_points(self.arm_state.joints)
        best, raw = None, None
        for joints, seen in self.hands.values():
            gap = float(np.linalg.norm(arm[:, None, :] - joints[None, :, :], axis=2).min()) - self.radii
            d = gap - self.hand_speed * (now - seen)
            if best is None or d < best[0]:
                best = (d, now - seen)
            raw = gap if raw is None else min(raw, gap)
        return best[0], best[1], raw

    def step(self):
        now = time.time()
        for topic, msg in self.sub.drain():
            if topic == Topic.ARM_GOAL and isinstance(msg, ArmGoal):
                self.publish(Topic.ARM_COMMAND, msg)
            elif topic == Topic.ARM_STATE and isinstance(msg, ArmState):
                self.arm_state, self.arm_time = msg, now
            elif topic == Topic.HAND and isinstance(msg, HandState) and msg.tracked and msg.joints:
                self.hands[msg.side] = (np.asarray(msg.joints, float), now)

        dist, age, raw = None, 0.0, None
        if self.arm_state is None or now - self.arm_time > self.arm_max_age:
            scale, zone, reason = 0.0, "stop", "no arm state"
        elif not self.hands:
            scale, zone, reason = 1.0, "clear", ""
        else:
            dist, age, raw = self.worst_case(now)
            reason = f"hand {dist:.2f} m from the arm" + (f", not seen for {age:.1f} s" if age > 2.0 / self.rate_hz else "")
            if dist < self.S or (self.stopped and dist < self.S + self.hysteresis):
                scale, zone = 0.0, "stop"
            elif dist < self.slow:
                scale, zone = (dist - self.S) / (self.slow - self.S), "slow"
            else:
                scale, zone = 1.0, "clear"
        self.stopped = zone == "stop"
        if zone != self.zone:
            held = now - self.zone_since
            self.log.info(f"zone {self.zone} -> {zone} (hand-arm {dist if dist is not None else float('nan'):.3f} m, "
                          f"{held:.1f} s in {self.zone})")
            self.zone, self.zone_since = zone, now
        if not self.enabled:
            scale, stop = 1.0, False
        else:
            stop = zone == "stop"
        self.publish(Topic.SAFETY, SafetyState(
            speed_scale=round(float(np.clip(scale, 0.0, 1.0)), 3), stop=stop, zone=zone, reason=reason,
            min_distance=None if dist is None else round(dist, 4),
            raw_distance=None if raw is None else round(raw, 4)))
