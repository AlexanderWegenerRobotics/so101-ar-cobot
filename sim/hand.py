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


J = {name: i for i, name in enumerate(HAND_JOINTS)}


def pinch_frame(joints):
    thumb, index = joints[J["thumb_tip"]], joints[J["index_tip"]]
    wrist = joints[J["wrist"]]
    knuckle_i, knuckle_l = joints[J["index_metacarpal"]], joints[J["little_metacarpal"]]
    forward = (knuckle_i + knuckle_l) / 2 - wrist
    forward /= np.linalg.norm(forward)
    side = knuckle_i - knuckle_l
    normal = np.cross(forward, side)
    normal /= np.linalg.norm(normal)
    side = np.cross(normal, forward)
    return Pose.from_rotation((thumb + index) / 2, np.column_stack([forward, side, normal]))


class Grasper:
    def __init__(self, world, side, grab_pinch=0.8, release_pinch=0.5, radius=0.05, max_speed=2.0):
        self.world = world
        self.side = side
        self.grab_pinch = grab_pinch
        self.release_pinch = release_pinch
        self.radius = radius
        self.max_speed = max_speed
        self.offset = None
        self.last = None
        self.velocity = np.zeros(3)
        self.cubes = [c["name"] for c in world.manifest["cubes"]]

    def update(self, msg):
        if not msg.tracked or not msg.joints:
            self.drop()
            return
        frame = pinch_frame(np.asarray(msg.joints, float))
        now = msg.stamp
        if self.last is not None and now > self.last[1]:
            v = (frame.pos - self.last[0]) / (now - self.last[1])
            self.velocity = 0.6 * self.velocity + 0.4 * v
        self.last = (frame.pos, now)
        if self.offset is None:
            if msg.pinch >= self.grab_pinch:
                self.try_grab(frame)
        elif msg.pinch <= self.release_pinch:
            self.drop()
        else:
            self.world.move_grasp(self.side, frame * self.offset)

    def try_grab(self, frame):
        best, best_d = None, self.radius
        for name in self.cubes:
            if any(self.world.held(s) == name for s in ("left", "right")):
                continue
            d = np.linalg.norm(self.world.body_pose_base(name).pos - frame.pos)
            if d < best_d:
                best, best_d = name, d
        if best is None:
            return
        self.offset = frame.inverse() * self.world.body_pose_base(best)
        self.world.grasp(self.side, best, frame * self.offset)

    def drop(self):
        if self.offset is None:
            return
        speed = np.linalg.norm(self.velocity)
        v = self.velocity if speed <= self.max_speed else self.velocity * self.max_speed / speed
        self.world.release(self.side, v)
        self.offset = None


class HumanAgent:
    """Scripted stand-in for the operator: follows the human part of task/state (next cube -> its bin), but with
    probability `deviate` picks another open cube instead, to exercise replanning. Moves the hand with min-jerk
    segments at `speed` and grasps through the same Grasper the Quest hands use (pinch close to the cube)."""

    def __init__(self, world, side, yaw, rest, speed=0.85, deviate=0.25, pause=0.5, seed=0, carry_z=0.11,
                 pinch_z=0.055):
        self.world = world
        self.side = side
        self.rot = Pose([0, 0, 0], [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)])
        offset = pinch_frame(flat_hand(side)).pos
        self.offset = self.rot.rotation @ offset
        self.rest = np.asarray(rest, float)
        self.speed = speed
        self.deviate = deviate
        self.pause = pause
        self.carry_z = carry_z
        self.pinch_z = pinch_z
        self.rng = np.random.default_rng(seed)
        self.pos = self.rest.copy()
        self.path = []
        self.pinch = 0.0
        self.busy_until = 0.0
        self.target = None
        self.log = None

    def wrist(self):
        return Pose(self.pos - self.offset, self.rot.quat)

    def _around(self, a, b, z):
        """Hand path at height z from a to b around the robot base, like the planner suggests it."""
        from task.allocator import AllocatorConfig, detour
        c = AllocatorConfig()
        return [np.array([p[0], p[1], z]) for p in detour(a, b, c.robot_pan_axis, c.keep_out_radius)[1:]]

    def _go(self, points, t):
        pts = [self.pos.copy()] + [np.asarray(p, float) for p in points]
        segs = []
        for a, b in zip(pts[:-1], pts[1:]):
            dur = max(0.3, 1.875 * np.linalg.norm(b - a) / self.speed)
            segs.append((t, dur, a, b))
            t += dur
        return segs, t

    def choose(self, task, objects):
        open_tags = [o.tag_id for o in task.objects if o.status == "pending" and o.owner != "robot"
                     and o.tag_id in objects]
        nxt = task.human_next_tag_id
        wait = next((o.est_start for o in task.objects if o.tag_id == nxt), 0.0) if nxt is not None else 0.0
        if nxt is not None and wait > 1.0:
            return None, False
        if nxt is not None and self.rng.random() >= self.deviate:
            return nxt, False
        others = [t for t in open_tags if t != nxt]
        if others and self.rng.random() < self.deviate * 2:
            robot_tags = [o.tag_id for o in task.objects if o.status == "pending" and o.owner == "robot"
                          and o.tag_id in objects]
            pool = others + robot_tags
            return int(self.rng.choice(pool)), True
        return nxt, False

    def reset(self):
        self.path = []
        self.pinch = 0.0
        self.pos = self.rest.copy()
        self.phase = "idle"
        self.busy_until = 0.0

    def step(self, t, task, objects, bins):
        if t < getattr(self, "last_t", -1.0) - 0.01:
            self.reset()
        self.last_t = t
        if self.path:
            start, dur, a, b = self.path[0]
            s = min_jerk((t - start) / dur)
            self.pos = a + s * (b - a)
            if t >= start + dur:
                self.path.pop(0)
                if not self.path and self.phase == "to_cube":
                    self.pinch = 1.0
                    self.phase = "grab"
                    self.busy_until = t + 0.3
                elif not self.path and self.phase == "to_bin":
                    self.phase = "settle"
                    self.busy_until = t + 0.25
                elif not self.path and self.phase == "home":
                    self.phase = "idle"
                    self.busy_until = t + self.pause
            return
        if t < self.busy_until:
            return
        phase = getattr(self, "phase", "idle")
        if phase == "grab":
            bin_pos = bins.get(self.target_bin)
            if bin_pos is None or self.world.held(self.side) is None:
                self.pinch = 0.0
                self.phase = "home"
                self.path, _ = self._go([self.pos + [0, 0, 0.05], self.rest], t)
                return
            spot = np.asarray(bin_pos[:2], float) + self.rng.uniform(-0.025, 0.025, 2)
            above = np.array([spot[0], spot[1], self.carry_z])
            lift = np.array([self.pos[0], self.pos[1], self.carry_z])
            self.path, _ = self._go([lift] + self._around(lift, above, self.carry_z) + [[spot[0], spot[1], 0.085]], t)
            self.phase = "to_bin"
        elif phase == "settle":
            self.pinch = 0.0
            self.phase = "release"
            self.busy_until = t + 0.3
        elif phase == "release":
            self.phase = "home"
            up = self.pos + [0, 0, 0.05]
            self.path, _ = self._go([up] + self._around(up, self.rest, up[2])[:-1] + [self.rest], t)
        elif phase == "idle":
            if task is None or task.mode != "auto":
                return
            tag, deviated = self.choose(task, objects)
            if tag is None or tag not in objects:
                return
            o = next(x for x in task.objects if x.tag_id == tag)
            cube = np.asarray(objects[tag], float)
            self.target, self.target_bin = tag, o.target_bin
            if self.log:
                self.log.info(f"sim human goes for tag {tag}" + (" (not the suggested one)" if deviated else ""))
            up = np.array([self.pos[0], self.pos[1], self.carry_z])
            self.path, _ = self._go([up] + self._around(up, cube, self.carry_z) +
                                    [[cube[0], cube[1], cube[2] + self.pinch_z - 0.015]], t)
            self.phase = "to_cube"


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
            grasp = self.params.get("grasp", {})
            self.graspers = {side: Grasper(self.world, side, **grasp) for side in ("left", "right")}
        elif self.mode == "agent":
            from interfaces.bus import Subscriber
            self.sub = Subscriber([Topic.TASK_STATE, Topic.OBJECTS], self.bus_cfg)
            a = self.params.get("agent", {})
            self.agent = HumanAgent(self.world, self.side, self.yaw, self.params.get("rest", [0.55, 0.12, 0.12]), **a)
            self.agent.log = self.log
            self.grasper = Grasper(self.world, self.side, **self.params.get("grasp", {}))
            self.task = None
            self.objects = {}
            self.bins = {}
        elif self.mode == "off":
            for side in ("left", "right"):
                self.world.set_hand(side, None)
        self.t0 = self.world.sim_time()

    def step(self):
        if self.mode == "off":
            return
        if self.mode == "agent":
            for topic, msg in self.sub.drain():
                if topic == Topic.TASK_STATE:
                    self.task = msg
                elif topic == Topic.OBJECTS:
                    for o in msg.objects:
                        self.objects[o.tag_id] = list(o.pose.pos)
                        if o.kind == "bin" or o.name.startswith("bin"):
                            self.bins[o.tag_id] = list(o.pose.pos)
            t = self.world.sim_time() - self.t0
            self.agent.step(t, self.task, self.objects, self.bins)
            msg = hand_state(self.side, self.agent.wrist(), source="sim_agent", pinch=self.agent.pinch)
            self.world.set_hand(self.side, np.asarray(msg.joints))
            self.grasper.update(msg)
            self.publish(Topic.HAND, msg)
            return
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
                if msg.side in self.graspers:
                    self.graspers[msg.side].update(msg)
