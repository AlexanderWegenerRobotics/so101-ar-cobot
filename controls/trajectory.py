import numpy as np

from interfaces.messages import ToolTarget

MIN_DURATION = 0.5
PEAK_SPEED_FACTOR = 1.875


def min_jerk(s):
    s = np.clip(s, 0.0, 1.0)
    return 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5


class LinePath:
    def __init__(self, start, end, max_speed, max_angular_speed):
        self.start = start
        self.end = end
        distance = np.linalg.norm(np.subtract(end.pos, start.pos))
        rotation = max(abs(end.pitch - start.pitch), abs(end.roll - start.roll))
        slowest = max(distance / max_speed, rotation / max_angular_speed)
        self.duration = max(MIN_DURATION, PEAK_SPEED_FACTOR * slowest)

    def at(self, t):
        s = min_jerk(t / self.duration)
        start_pos, end_pos = np.asarray(self.start.pos), np.asarray(self.end.pos)
        return ToolTarget((start_pos + s * (end_pos - start_pos)).tolist(),
                          self.start.pitch + s * (self.end.pitch - self.start.pitch),
                          self.start.roll + s * (self.end.roll - self.start.roll))

    def waypoints(self, count=20):
        return [self.at(t).pos for t in np.linspace(0.0, self.duration, count)]
