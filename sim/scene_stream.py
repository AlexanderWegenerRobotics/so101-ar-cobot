import time

from interfaces.messages import SceneInfo, ScenePoses, Topic
from interfaces.node import Node
from sim.scene_export import dynamic_bodies, pose_array, scene_info


class SceneStreamer(Node):
    name = "scene_streamer"
    rate_hz = 60.0

    def setup(self):
        from hardware import get_sim_world
        self.rate_hz = self.params.get("rate_hz", self.rate_hz)
        self.info_period = self.params.get("info_period", 1.0)
        self.world = get_sim_world()
        info = scene_info(self.world)
        self.info = SceneInfo(version=info["version"], asset_version=info["asset_version"],
                              bodies=info["bodies"], textures=info["textures"])
        dyn = dynamic_bodies(self.world, info["bodies"])
        self.names = [n for n, _ in dyn]
        self.ids = [i for _, i in dyn]
        self.last_info = 0.0
        self.log.info(f"scene {self.info.version}, assets {self.info.asset_version}, "
                      f"{len(self.info.bodies)} bodies, {len(self.names)} streamed")

    def step(self):
        t = time.time()
        if t - self.last_info >= self.info_period:
            self.info.stamp = t
            self.publish(Topic.SIM_SCENE_INFO, self.info)
            self.last_info = t
        poses, sim_time = pose_array(self.world, self.ids)
        self.publish(Topic.SIM_SCENE, ScenePoses(version=self.info.version, names=self.names,
                                                 poses=poses, sim_time=sim_time))
