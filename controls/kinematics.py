from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from interfaces.transforms import Pose

ROOT = Path(__file__).resolve().parents[1]
KINEMATICS_CONFIG = ROOT / "config" / "kinematics.yaml"

MAX_ITERATIONS = 200
TOLERANCE = 1e-6
DAMPING = 0.05
JACOBIAN_STEP = 1e-6


@dataclass
class Joint:
    name: str
    frame: Pose
    axis: np.ndarray
    limits: np.ndarray


def rotation_about(axis, angle):
    return Pose([0.0, 0.0, 0.0], np.concatenate([[np.cos(angle / 2)], np.sin(angle / 2) * axis]))


def wrap_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


class ArmKinematics:
    def __init__(self, joints, tool):
        self.joints = joints
        self.tool = tool

    def forward(self, q):
        pose = Pose()
        for joint, angle in zip(self.joints, q):
            pose = pose * joint.frame * rotation_about(joint.axis, angle)
        return pose * self.tool

    def pitch(self, pose):
        approach = pose.rotation[:, 0]
        heading = pose.pos[:2] - self.joints[0].frame.pos[:2]
        heading = heading / np.linalg.norm(heading)
        return np.arctan2(approach[2], approach[:2] @ heading)

    def inverse(self, target, seed):
        limits = np.array([joint.limits for joint in self.joints])
        if not limits[-1, 0] <= target.roll <= limits[-1, 1]:
            return None
        q = np.array(seed[:4], float)
        for _ in range(MAX_ITERATIONS):
            error = self._error(q, target)
            if np.linalg.norm(error) < TOLERANCE:
                return np.append(q, target.roll)
            jacobian = np.column_stack(
                [(self._error(q + JACOBIAN_STEP * unit, target) - error) / JACOBIAN_STEP for unit in np.eye(4)])
            step = np.linalg.solve(jacobian.T @ jacobian + DAMPING ** 2 * np.eye(4), jacobian.T @ error)
            q = np.clip(q - step, limits[:4, 0], limits[:4, 1])
        return None

    def _error(self, q, target):
        pose = self.forward(np.append(q, target.roll))
        return np.append(pose.pos - target.pos, wrap_angle(self.pitch(pose) - target.pitch))


def load_kinematics(path=KINEMATICS_CONFIG):
    with open(path) as f:
        cfg = yaml.safe_load(f)
    joints = [Joint(j["name"], Pose(j["pos"], j["quat"]), np.array(j["axis"], float), np.array(j["limits"], float))
              for j in cfg["joints"]]
    return ArmKinematics(joints, Pose(cfg["tool"]["pos"], cfg["tool"]["quat"]))
