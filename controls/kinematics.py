import math
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MODEL_XML = ROOT / "sim" / "model" / "so101.xml"

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
GRIPPER = "gripper"
TOOL_SITE = "gripperframe"


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass
class IKResult:
    q: np.ndarray
    ok: bool
    pos_error: float
    angle_error: float
    iterations: int


class JointMap:
    def __init__(self, model, joint_map):
        self.model = model
        self.map = joint_map or {}

    def _m(self, name):
        return self.map.get(name, {})

    def to_robot(self, name, q):
        mp = self._m(name)
        if mp.get("mode") == "range":
            lo, hi = self.model.jnt_range[self.model.joint(name).id]
            frac = (q - lo) / (hi - lo)
            frac = frac if mp.get("sign", 1) > 0 else 1 - frac
            return float(np.clip(frac * 100.0, 0.0, 100.0))
        return mp.get("sign", 1) * (math.degrees(q) + mp.get("offset_deg", 0.0))

    def to_q(self, name, value):
        mp = self._m(name)
        if mp.get("mode") == "range":
            lo, hi = self.model.jnt_range[self.model.joint(name).id]
            frac = value / 100.0
            frac = frac if mp.get("sign", 1) > 0 else 1 - frac
            return lo + frac * (hi - lo)
        return math.radians(mp.get("sign", 1) * value - mp.get("offset_deg", 0.0))


class SO101Kinematics:
    def __init__(self, joint_map=None, model_xml=MODEL_XML):
        self.model = mujoco.MjModel.from_xml_path(str(model_xml))
        self.data = mujoco.MjData(self.model)
        self.site = self.model.site(TOOL_SITE).id
        self.qadr = np.array([self.model.jnt_qposadr[self.model.joint(j).id] for j in ARM_JOINTS])
        self.limits = np.array([self.model.jnt_range[self.model.joint(j).id] for j in ARM_JOINTS])
        self.joints = JointMap(self.model, joint_map)
        mujoco.mj_forward(self.model, self.data)
        self.pan_axis = self.data.xanchor[self.model.joint("shoulder_pan").id].copy()
        q = np.zeros(5)
        q[0] = 0.3
        self.pan_sign = 1.0 if self._site(q)[0][1] > 0 else -1.0

    def pan_for(self, p):
        """Pan angle that points the arm at p (the SO-101 pan joint turns clockwise for positive angles)."""
        return self.pan_sign * self.arm_yaw(p)

    def robot_to_q(self, robot_joints):
        return np.array([self.joints.to_q(j, robot_joints[j]) for j in ARM_JOINTS])

    def q_to_robot(self, q):
        return {j: float(self.joints.to_robot(j, v)) for j, v in zip(ARM_JOINTS, q)}

    def gripper_to_q(self, value):
        return self.joints.to_q(GRIPPER, value)

    def _site(self, q):
        self.data.qpos[self.qadr] = q
        mujoco.mj_kinematics(self.model, self.data)
        return self.data.site_xpos[self.site].copy(), self.data.site_xmat[self.site].reshape(3, 3).copy()

    def arm_yaw(self, p):
        return math.atan2(p[1] - self.pan_axis[1], p[0] - self.pan_axis[0])

    def tool_angles(self, p, R):
        a = R[:, 0]
        pitch = math.asin(float(np.clip(a[2], -1.0, 1.0)))
        yaw = self.arm_yaw(p)
        h = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
        y = R[:, 1] - a * float(a @ R[:, 1])
        y /= np.linalg.norm(y)
        h = h - a * float(a @ h)
        n = np.linalg.norm(h)
        if n < 1e-6:
            return pitch, 0.0
        h /= n
        roll = math.atan2(float(np.cross(h, y) @ a), float(h @ y))
        return pitch, roll

    def fk(self, q):
        p, R = self._site(q)
        pitch, roll = self.tool_angles(p, R)
        return p, pitch, roll

    def task(self, q):
        p, pitch, roll = self.fk(q)
        return np.array([p[0], p[1], p[2], pitch, roll])

    def ik(self, pos, pitch, roll, q0, **kw):
        """Damped least squares from q0; if that fails, retry from seeds with the pan already pointing at the target."""
        best = self._ik(pos, pitch, roll, q0, **kw)
        if best.ok:
            return best
        yaw = float(np.clip(self.pan_for(pos), *self.limits[0]))
        for lift, elbow, wrist in ((0.0, 0.0, 0.0), (0.3, 0.6, 1.2), (-0.5, 1.2, 0.8), (0.8, -0.3, 1.0)):
            seed = np.array([yaw, lift, elbow, wrist, q0[4]])
            r = self._ik(pos, pitch, roll, seed, **kw)
            if r.ok:
                return r
            if r.pos_error + 0.05 * r.angle_error < best.pos_error + 0.05 * best.angle_error:
                best = r
        return best

    def _ik(self, pos, pitch, roll, q0, iters=60, damping=0.02, angle_weight=0.05,
           tol_pos=0.001, tol_angle=math.radians(1.0)):
        q = np.clip(np.array(q0, float), self.limits[:, 0], self.limits[:, 1])
        W = np.diag([1, 1, 1, angle_weight, angle_weight])
        target = np.array([*pos, pitch, roll])
        eps = 1e-5
        for it in range(iters):
            x = self.task(q)
            e = target - x
            e[3] = wrap(e[3])
            e[4] = wrap(e[4])
            pe, ae = float(np.linalg.norm(e[:3])), float(max(abs(e[3]), abs(e[4])))
            if pe < tol_pos and ae < tol_angle:
                return IKResult(q, True, pe, ae, it)
            J = np.zeros((5, 5))
            for k in range(5):
                dq = np.zeros(5)
                dq[k] = eps
                d = self.task(q + dq) - x
                d[3], d[4] = wrap(d[3]), wrap(d[4])
                J[:, k] = d / eps
            Jw, ew = W @ J, W @ e
            step = Jw.T @ np.linalg.solve(Jw @ Jw.T + damping ** 2 * np.eye(5), ew)
            n = np.linalg.norm(step)
            if n > 0.3:
                step *= 0.3 / n
            q = np.clip(q + step, self.limits[:, 0], self.limits[:, 1])
        x = self.task(q)
        e = target - x
        e[3], e[4] = wrap(e[3]), wrap(e[4])
        pe, ae = float(np.linalg.norm(e[:3])), float(max(abs(e[3]), abs(e[4])))
        return IKResult(q, pe < 3 * tol_pos and ae < 3 * tol_angle, pe, ae, iters)
