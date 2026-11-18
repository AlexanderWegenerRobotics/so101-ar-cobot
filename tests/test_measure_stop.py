import numpy as np
import pytest

from controls.measure_stop import stop_metrics

RATE = 30.0
CRUISE_SPEED = 0.10
STOP_SENT = 1.01
ARM_HALTS = 1.05


def recorded_stop():
    times = np.arange(0.0, 2.0, 1 / RATE)
    positions = np.zeros((len(times), 3))
    positions[:, 0] = CRUISE_SPEED * np.minimum(times, ARM_HALTS)
    statuses = ["stopped" if t >= ARM_HALTS else "moving" for t in times]
    return times, positions, statuses


def test_stop_metrics_report_latency_distance_and_speed():
    times, positions, statuses = recorded_stop()
    metrics = stop_metrics(times, positions, statuses, STOP_SENT)
    assert metrics["speed"] == pytest.approx(CRUISE_SPEED, rel=1e-3)
    assert metrics["latency"] == pytest.approx(0.0567, abs=1e-3)
    assert metrics["distance"] == pytest.approx(0.005, abs=1e-4)


def test_stop_metrics_raise_if_the_arm_never_stops():
    times, positions, _ = recorded_stop()
    with pytest.raises(RuntimeError):
        stop_metrics(times, positions, ["moving"] * len(times), STOP_SENT)
