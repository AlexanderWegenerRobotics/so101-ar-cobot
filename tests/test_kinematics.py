import numpy as np
import pytest

from controls.kinematics import load_kinematics
from interfaces.messages import ToolTarget

READY_SEED = [0.0, -0.4, 1.2, 0.77]
STRAIGHT_DOWN = -np.pi / 2
CUBE_CENTER_HEIGHT = 0.015


@pytest.fixture(scope="module")
def kinematics():
    return load_kinematics()


def test_inverse_reaches_pose_from_forward(kinematics):
    rng = np.random.default_rng(0)
    for _ in range(20):
        q = rng.uniform([-1.0, -0.8, 0.3, 0.2, -1.5], [1.0, -0.2, 1.3, 1.2, 1.5])
        pose = kinematics.forward(q)
        target = ToolTarget(pose.pos.tolist(), kinematics.pitch(pose), q[4])
        solution = kinematics.inverse(target, READY_SEED)
        assert solution is not None
        solved = kinematics.forward(solution)
        assert np.allclose(solved.pos, pose.pos, atol=1e-4)
        assert kinematics.pitch(solved) == pytest.approx(target.pitch, abs=1e-3)


def test_straight_down_grasp_over_cube_area(kinematics):
    for radius in np.linspace(0.17, 0.26, 4):
        for angle in np.radians(np.linspace(-65, 65, 7)):
            target = ToolTarget([radius * np.cos(angle), radius * np.sin(angle), CUBE_CENTER_HEIGHT], STRAIGHT_DOWN)
            assert kinematics.inverse(target, READY_SEED) is not None


def test_unreachable_target_returns_none(kinematics):
    assert kinematics.inverse(ToolTarget([0.8, 0.0, 0.2], STRAIGHT_DOWN), READY_SEED) is None
