import threading
import time

import mujoco
import numpy as np

from interfaces.transforms import Pose
from sim.scene_builder import build, home_data, load_config, write_manifest

CV_FROM_MJ_CAMERA = np.diag([1.0, -1.0, -1.0])
BASE_SITE = "baseframe"


class SimWorld:
    def __init__(self, scene_cfg=None, robot_cfg=None, realtime=True, viewer=False):
        self.spec, self.model, self.manifest = build(scene_cfg, robot_cfg)
        write_manifest(self.manifest)
        self.data = home_data(self.model)
        self.lock = threading.RLock()
        self.realtime = realtime
        self.use_viewer = viewer
        self._viewer = None
        self._thread = None
        self._running = False
        self._renderers = {}
        self._grasps = {}

    @property
    def is_running(self):
        return self._running

    def start(self):
        if self._running:
            return
        self._running = True
        if self.use_viewer:
            import mujoco.viewer
            self._viewer = mujoco.viewer.launch_passive(self.model, self.data)
        if self.realtime:
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None

    def step(self, n=1):
        with self.lock:
            for _ in range(n):
                self._apply_grasps()
                mujoco.mj_step(self.model, self.data)

    def advance(self, seconds):
        self.step(max(1, int(round(seconds / self.model.opt.timestep))))

    def _loop(self):
        dt = self.model.opt.timestep
        next_t = time.perf_counter()
        last_sync = 0.0
        while self._running:
            with self.lock:
                self._apply_grasps()
                mujoco.mj_step(self.model, self.data)
                if self._viewer is not None and next_t - last_sync > 1 / 60:
                    self._viewer.sync()
                    last_sync = next_t
            next_t += dt
            delay = next_t - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            elif delay < -0.1:
                next_t = time.perf_counter()

    def reset(self):
        with self.lock:
            self._grasps.clear()
            mujoco.mj_resetDataKeyframe(self.model, self.data, self.model.key("home").id)
            mujoco.mj_forward(self.model, self.data)

    def render(self, camera=None, width=None, height=None):
        cam = self.manifest["camera"]
        camera = camera or cam["name"]
        width = width or cam["resolution"][0]
        height = height or cam["resolution"][1]
        key = (threading.get_ident(), width, height)
        if key not in self._renderers:
            self._renderers[key] = mujoco.Renderer(self.model, height, width)
        renderer = self._renderers[key]
        with self.lock:
            renderer.update_scene(self.data, camera=camera)
        return renderer.render()

    def body_pose(self, name):
        with self.lock:
            body = self.model.body(name).id
            return Pose(self.data.xpos[body].copy(), self.data.xquat[body].copy())

    def site_pose(self, name):
        with self.lock:
            site = self.model.site(name).id
            return Pose.from_rotation(self.data.site_xpos[site].copy(), self.data.site_xmat[site].reshape(3, 3).copy())

    def base_from_mujoco(self):
        return self.site_pose(BASE_SITE).inverse()

    def body_pose_base(self, name):
        return self.base_from_mujoco() * self.body_pose(name)

    def camera_pose_base(self, name=None):
        name = name or self.manifest["camera"]["name"]
        with self.lock:
            cam = self.model.camera(name).id
            pos = self.data.cam_xpos[cam].copy()
            R = self.data.cam_xmat[cam].reshape(3, 3) @ CV_FROM_MJ_CAMERA
        return self.base_from_mujoco() * Pose.from_rotation(pos, R)

    def set_hand(self, side, joints_base):
        hands = self.manifest.get("hands")
        if not hands or side not in hands["sides"]:
            return
        T = self.base_from_mujoco().inverse()
        with self.lock:
            for i, joint in enumerate(hands["joints"]):
                mocap = self.model.body_mocapid[self.model.body(f"hand_{side}_{joint}").id]
                self.data.mocap_pos[mocap] = hands["park"] if joints_base is None else T * joints_base[i]

    def grasp(self, side, body_name, target_base):
        body = self.model.body(body_name)
        joint = body.jntadr[0]
        with self.lock:
            self._grasps[side] = {
                "body": body_name,
                "qpos": int(self.model.jnt_qposadr[joint]),
                "dof": int(self.model.jnt_dofadr[joint]),
                "target": self.base_from_mujoco().inverse() * target_base,
            }

    def move_grasp(self, side, target_base):
        with self.lock:
            g = self._grasps.get(side)
            if g is not None:
                g["target"] = self.base_from_mujoco().inverse() * target_base

    def release(self, side, velocity_base=None):
        with self.lock:
            g = self._grasps.pop(side, None)
            if g is None:
                return None
            if velocity_base is not None:
                R = self.base_from_mujoco().inverse().rotation
                self.data.qvel[g["dof"]:g["dof"] + 3] = R @ np.asarray(velocity_base, float)
            return g["body"]

    def held(self, side):
        g = self._grasps.get(side)
        return g["body"] if g else None

    def _apply_grasps(self):
        for g in self._grasps.values():
            a, d, T = g["qpos"], g["dof"], g["target"]
            self.data.qpos[a:a + 3] = T.pos
            self.data.qpos[a + 3:a + 7] = T.quat
            self.data.qvel[d:d + 6] = 0.0

    def object_poses(self):
        names = [c["name"] for c in self.manifest["cubes"]] + [b["name"] for b in self.manifest["bins"]]
        return {n: self.body_pose_base(n) for n in names}

    def sim_time(self):
        return float(self.data.time)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int)
    args = parser.parse_args()
    cfg = load_config()
    if args.seed is not None:
        cfg["seed"] = args.seed
    world = SimWorld(cfg, viewer=True)
    world.start()
    try:
        while world._viewer is not None and world._viewer.is_running():
            time.sleep(0.1)
    finally:
        world.stop()
