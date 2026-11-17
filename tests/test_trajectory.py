import numpy as np
import pytest

from controls.trajectory import LinePath
from interfaces.messages import ToolTarget

START = ToolTarget([0.20, 0.00, 0.05])
END = ToolTarget([0.20, 0.10, 0.05], roll=0.4)
MAX_SPEED = 0.10
MAX_ANGULAR_SPEED = 1.0


def test_path_starts_and_ends_at_targets():
    path = LinePath(START, END, MAX_SPEED, MAX_ANGULAR_SPEED)
    assert np.allclose(path.at(0.0).pos, START.pos)
    assert np.allclose(path.at(path.duration).pos, END.pos)
    assert path.at(path.duration).roll == pytest.approx(END.roll)


def test_peak_speed_matches_max_speed():
    path = LinePath(START, END, MAX_SPEED, MAX_ANGULAR_SPEED)
    times = np.linspace(0.0, path.duration, 1000)
    positions = np.array([path.at(t).pos for t in times])
    speeds = np.linalg.norm(np.diff(positions, axis=0), axis=1) / np.diff(times)
    assert speeds.max() == pytest.approx(MAX_SPEED, rel=0.02)


def test_rotation_only_move_is_limited_by_angular_speed():
    end = ToolTarget(START.pos, START.pitch, START.roll + 1.5)
    path = LinePath(START, end, MAX_SPEED, MAX_ANGULAR_SPEED)
    assert path.duration == pytest.approx(1.875 * 1.5 / MAX_ANGULAR_SPEED)


def test_zero_length_move_has_minimum_duration():
    assert LinePath(START, START, MAX_SPEED, MAX_ANGULAR_SPEED).duration == pytest.approx(0.5)
