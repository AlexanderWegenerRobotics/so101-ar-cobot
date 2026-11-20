"""Safety supervisor: speed and separation monitoring between the operator's hands and the arm.

Every tick: arm link points from arm/state (forward kinematics on the MuJoCo model, points along every moving link
from the shoulder out; the fixed base is not counted) and
all tracked hand joints (both hands, from `hand`). The minimum distance d (between points, minus link and finger
radii) picks the zone:

    d > slow_distance           clear   speed_scale 1
    stop_distance < d < slow     slow    speed_scale falls linearly to 0 at stop_distance
    d < stop_distance            stop    stop = true, the controller holds; released above stop + hysteresis

Goals from arm/goal go to arm/command unchanged (the controller applies the scale and the stop). Zone changes are
logged with the distance, which is the data for the safety study.
"""
import time

import mujoco
import numpy as np

from interfaces.messages import ArmGoal, ArmState, HandState, SafetyState, Topic
from interfaces.node import Node

ARM_BODIES = ["upper_arm", "lower_arm", "wrist", "gripper", "moving_jaw_so101_v1"]


class SafetySupervisor(Node):
    name = "supervisor"
    rate_hz = 30.0
    subscribe = [Topic.ARM_GOAL, Topic.ARM_STATE, Topic.HAND]

    def setup(self):
        from controls.kinematics import ARM_JOINTS, SO101Kinematics
        from hardware import load_robot_config
        p = self.params
        cfg = load_robot_config()
        self.kin = SO101Kinematics(cfg.get("sim", {}).get("joint_map", {}))
        self.arm_joints = ARM_JOINTS
        self.bodies = [self.kin.model.body(b).id for b in ARM_BODIES]
        self.parents = [self.kin.model.body_parentid[b] for b in self.bodies]
        self.slow = p.get("slow_distance", 0.25)
        self.stop_d = p.get("stop_distance", 0.10)
        self.hysteresis = p.get("hysteresis", 0.03)
        self.link_radius = p.get("link_radius", 0.03)
        self.finger_radius = p.get("finger_radius", 0.01)
        self.hand_timeout = p.get("hand_timeout", 0.3)
        self.enabled = p.get("enabled", True)
        self.arm_state = None
        self.hands = {}
        self.zone = "clear"
        self.stopped = False
        self.zone_since = time.time()
        self.closest = None
        self.log.info(f"safety supervisor: slow below {self.slow:.2f} m, stop below {self.stop_d:.2f} m"
                      + ("" if self.enabled else " (DISABLED, monitoring only)"))

    def arm_points(self):
        joints = self.arm_state.joints
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

    def hand_points(self, now):
        pts = [np.asarray(m.joints, float) for m, t in self.hands.values()
               if now - t < self.hand_timeout and m.tracked and m.joints]
        return np.concatenate(pts) if pts else None

    def step(self):
        now = time.time()
        for topic, msg in self.sub.drain():
            if topic == Topic.ARM_GOAL and isinstance(msg, ArmGoal):
                self.publish(Topic.ARM_COMMAND, msg)
            elif topic == Topic.ARM_STATE and isinstance(msg, ArmState):
                self.arm_state = msg
            elif topic == Topic.HAND and isinstance(msg, HandState):
                self.hands[msg.side] = (msg, now)
        dist = None
        if self.arm_state is not None:
            hands = self.hand_points(now)
            if hands is not None:
                arm = self.arm_points()
                dmat = np.linalg.norm(arm[:, None, :] - hands[None, :, :], axis=2)
                dist = float(dmat.min()) - self.link_radius - self.finger_radius
        if dist is None:
            scale, zone = 1.0, "clear"
        elif dist < self.stop_d or (self.stopped and dist < self.stop_d + self.hysteresis):
            scale, zone = 0.0, "stop"
        elif dist < self.slow:
            scale = (dist - self.stop_d) / (self.slow - self.stop_d)
            zone = "slow"
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
            speed_scale=round(float(np.clip(scale, 0.0, 1.0)), 3), stop=stop, zone=zone,
            reason="" if dist is None else f"hand {dist:.2f} m from the arm",
            min_distance=None if dist is None else round(dist, 4)))
