import numpy as np

from interfaces.messages import HAND_JOINTS, HandState, Topic
from interfaces.node import Node
from interfaces.transforms import Pose

FINGERS = {
    "index": (0.024, 1.00),
    "middle": (0.004, 1.06),
    "ring": (-0.015, 0.98),
    "little": (-0.032, 0.82),
}
FINGER_X = {"metacarpal": 0.030, "proximal": 0.090, "intermediate": 0.132, "distal": 0.158, "tip": 0.180}
THUMB = {"metacarpal": (0.020, 0.030), "proximal": (0.045, 0.055), "distal": (0.068, 0.072), "tip": (0.088, 0.082)}


def flat_hand(side):
    mirror = 1.0 if side == "right" else -1.0
    pts = {"wrist": (0.0, 0.0), "palm": (0.050, 0.0)}
    for seg, (x, y) in THUMB.items():
        pts[f"thumb_{seg}"] = (x, y)
    for finger, (y, scale) in FINGERS.items():
        for seg, x in FINGER_X.items():
            base = FINGER_X["metacarpal"]
            pts[f"{finger}_{seg}"] = (x if seg == "metacarpal" else base + (x - base) * scale, y * (0.8 if seg == "metacarpal" else 1.0))
    return np.array([[pts[j][0], mirror * pts[j][1], 0.0] for j in HAND_JOINTS])


def min_jerk(s):
    s = np.clip(s, 0.0, 1.0)
    return 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5


def hand_state(side, wrist_pose, source, pinch=0.0):
    joints = wrist_pose * flat_hand(side)
    idx_p, idx_t = HAND_JOINTS.index("index_proximal"), HAND_JOINTS.index("index_tip")
    ray = joints[idx_t] - joints[idx_p]
    return HandState(side=side, tracked=True, joints=joints.tolist(), pinch=pinch,
                     ray_origin=joints[idx_t].tolist(), ray_dir=(ray / np.linalg.norm(ray)).tolist(), source=source)


class ScriptedReach:
    def __init__(self, targets, rest, hover=0.05, t_reach=1.5, t_hold=2.0, t_back=1.5, t_rest=3.0):
        self.targets = [np.asarray(t, float) for t in targets]
        self.rest = np.asarray(rest, float)
        self.hover = hover
        self.phases = [t_rest, t_reach, t_hold, t_back]
        self.cycle = sum(self.phases)

    def wrist_pos(self, t):
        k = int(t // self.cycle) % len(self.targets)
        target = self.targets[k] + [0.06, 0.0, self.hover]
        tc = t % self.cycle
        t_rest, t_reach, t_hold, _ = self.phases
        if tc < t_rest:
            return self.rest
        if tc < t_rest + t_reach:
            return self.rest + min_jerk((tc - t_rest) / t_reach) * (target - self.rest)
        if tc < t_rest + t_reach + t_hold:
            return target
        return target + min_jerk((tc - t_rest - t_reach - t_hold) / self.phases[3]) * (self.rest - target)


class SimHand(Node):
    name = "sim_hand"
    rate_hz = 30.0

    def setup(self):
        from hardware import get_sim_world
        self.world = get_sim_world()
        self.mode = self.params.get("source", "scripted")
        self.side = self.params.get("side", "right")
        self.yaw = np.radians(self.params.get("yaw_deg", 180.0))
        if self.mode == "scripted":
            targets = self.params.get("targets")
            if targets is None:
                targets = [c["init_pos"] for c in self.world.manifest["cubes"]]
            self.script = ScriptedReach(targets, self.params.get("rest", [0.55, 0.12, 0.12]),
                                        **self.params.get("timing", {}))
        elif self.mode == "bus":
            from interfaces.bus import Subscriber
            self.sub = Subscriber(Topic.HAND, self.bus_cfg)
        self.t0 = self.world.sim_time()

    def step(self):
        if self.mode == "scripted":
            t = self.world.sim_time() - self.t0
            wrist = Pose(self.script.wrist_pos(t), [np.cos(self.yaw / 2), 0, 0, np.sin(self.yaw / 2)])
            msg = hand_state(self.side, wrist, source="sim_scripted")
            self.world.set_hand(self.side, np.asarray(msg.joints))
            self.publish(Topic.HAND, msg)
        else:
            for _, msg in self.sub.drain():
                if msg.source.startswith("sim"):
                    continue
                self.world.set_hand(msg.side, np.asarray(msg.joints) if msg.tracked and msg.joints else None)
