"""Minimum jerk segments over densely sampled joint paths.

A motion is a list of segments. Each segment holds joint samples q[0..N] spaced evenly in the path parameter s
and a duration T. Progress tau goes 0..1 and is advanced by dt * speed_scale / T, so scaling the speed (safety,
operator) time-warps the motion along the same path, and a scale of 0 holds the arm in place.
"""
import math

import numpy as np


def min_jerk(tau):
    tau = min(max(tau, 0.0), 1.0)
    return tau ** 3 * (10 - 15 * tau + 6 * tau ** 2)


def min_jerk_duration(length, v_max, t_min=0.0):
    """Peak speed of a min-jerk profile is 1.875 * L / T."""
    return max(1.875 * length / v_max if v_max > 0 else 0.0, t_min)


def min_jerk_accel_duration(length, a_max):
    """Peak acceleration of a min-jerk profile is 5.774 * L / T^2."""
    return math.sqrt(5.774 * length / a_max) if a_max > 0 and length > 0 else 0.0


class Segment:
    """Joint samples q[0..N] along a path. knots[i] in [0, 1] is where sample i sits on the path parameter
    (default: evenly spaced). The parameter follows a min-jerk profile over the duration."""

    def __init__(self, q, duration, kind, knots=None):
        self.q = np.asarray(q, float)
        self.duration = max(duration, 1e-3)
        self.kind = kind
        self.knots = None if knots is None else np.asarray(knots, float)

    def at(self, tau):
        s = min_jerk(tau)
        if self.knots is None:
            s *= len(self.q) - 1
            i = min(int(s), len(self.q) - 2)
            f = s - i
        else:
            i = int(np.clip(np.searchsorted(self.knots, s, side="right") - 1, 0, len(self.q) - 2))
            span = self.knots[i + 1] - self.knots[i]
            f = 0.0 if span <= 0 else min(max((s - self.knots[i]) / span, 0.0), 1.0)
        return (1 - f) * self.q[i] + f * self.q[i + 1]


class Trajectory:
    def __init__(self, segments):
        self.segments = [s for s in segments if len(s.q) >= 2]
        self.index = 0
        self.tau = 0.0

    @property
    def done(self):
        return self.index >= len(self.segments)

    @property
    def duration(self):
        return sum(s.duration for s in self.segments)

    def advance(self, dt):
        """Advance by dt seconds of nominal time (already multiplied by the speed scale); returns joint command."""
        while not self.done:
            seg = self.segments[self.index]
            self.tau += dt / seg.duration
            if self.tau < 1.0:
                return seg.at(self.tau)
            dt = (self.tau - 1.0) * seg.duration
            self.tau = 0.0
            self.index += 1
        return self.segments[-1].q[-1].copy() if self.segments else None

    def end(self):
        return self.segments[-1].q[-1].copy() if self.segments else None
