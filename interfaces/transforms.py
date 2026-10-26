from dataclasses import dataclass, field

import numpy as np


def quat_normalize(q):
    q = np.asarray(q, float)
    return q / np.linalg.norm(q)


def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_to_matrix(q):
    w, x, y, z = quat_normalize(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat(m):
    m = np.asarray(m, float)
    tr = np.trace(m)
    if tr > 0:
        s = 2 * np.sqrt(tr + 1)
        q = [0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2 * np.sqrt(1 + m[0, 0] - m[1, 1] - m[2, 2])
        q = [(m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s]
    elif m[1, 1] > m[2, 2]:
        s = 2 * np.sqrt(1 + m[1, 1] - m[0, 0] - m[2, 2])
        q = [(m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s]
    else:
        s = 2 * np.sqrt(1 + m[2, 2] - m[0, 0] - m[1, 1])
        q = [(m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s]
    q = quat_normalize(q)
    return q if q[0] >= 0 else -q


@dataclass
class Pose:
    pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    quat: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0]))

    def __post_init__(self):
        self.pos = np.asarray(self.pos, float).reshape(3)
        self.quat = quat_normalize(self.quat)

    @classmethod
    def identity(cls):
        return cls()

    @classmethod
    def from_matrix(cls, T):
        T = np.asarray(T, float)
        return cls(T[:3, 3], matrix_to_quat(T[:3, :3]))

    @classmethod
    def from_rotation(cls, pos, R):
        return cls(pos, matrix_to_quat(R))

    @property
    def rotation(self):
        return quat_to_matrix(self.quat)

    def matrix(self):
        T = np.eye(4)
        T[:3, :3] = self.rotation
        T[:3, 3] = self.pos
        return T

    def inverse(self):
        qi = quat_conj(self.quat)
        return Pose(-quat_to_matrix(qi) @ self.pos, qi)

    def __mul__(self, other):
        if isinstance(other, Pose):
            return Pose(self.pos + self.rotation @ other.pos, quat_mul(self.quat, other.quat))
        return self.transform_point(other)

    def transform_point(self, p):
        p = np.asarray(p, float)
        return p @ self.rotation.T + self.pos

    def yaw(self):
        R = self.rotation
        return float(np.arctan2(R[1, 0], R[0, 0]))

    def to_dict(self):
        return {"pos": self.pos.tolist(), "quat": self.quat.tolist()}

    @classmethod
    def from_dict(cls, d):
        return cls(d["pos"], d["quat"])


def pose_error(a, b):
    dq = quat_mul(quat_conj(a.quat), b.quat)
    angle = 2 * np.arccos(min(1.0, abs(dq[0])))
    dyaw = (b.yaw() - a.yaw() + np.pi) % (2 * np.pi) - np.pi
    return {"pos": float(np.linalg.norm(b.pos - a.pos)), "angle": float(angle), "yaw": float(dyaw)}
