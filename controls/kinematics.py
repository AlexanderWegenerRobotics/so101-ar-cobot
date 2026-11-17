from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from interfaces.transforms import Pose

ROOT = Path(__file__).resolve().parents[1]
KINEMATICS_CONFIG = ROOT / "config" / "kinematics.yaml"


@dataclass
class Joint:
    name: str
    frame: Pose
    axis: np.ndarray


def rotation_about(axis, angle):
    return Pose([0.0, 0.0, 0.0], np.concatenate([[np.cos(angle / 2)], np.sin(angle / 2) * axis]))


class ArmKinematics:
    def __init__(self, joints, tool):
        self.joints = joints
        self.tool = tool

    def forward(self, q):
        pose = Pose()
        for joint, angle in zip(self.joints, q):
            pose = pose * joint.frame * rotation_about(joint.axis, angle)
        return pose * self.tool


def load_kinematics(path=KINEMATICS_CONFIG):
    with open(path) as f:
        cfg = yaml.safe_load(f)
    joints = [Joint(j["name"], Pose(j["pos"], j["quat"]), np.array(j["axis"], float)) for j in cfg["joints"]]
    return ArmKinematics(joints, Pose(cfg["tool"]["pos"], cfg["tool"]["quat"]))
