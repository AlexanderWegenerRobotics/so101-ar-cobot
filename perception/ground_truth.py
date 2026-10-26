import time

from interfaces.objects import ObjectState


class GroundTruthTracker:
    source = "ground_truth"

    def __init__(self, world):
        self.world = world
        m = world.manifest
        self.registry = {m["anchor"]["tag_id"]: ("anchor", "anchor")}
        self.registry.update({b["tag_id"]: (b["name"], "bin") for b in m["bins"]})
        self.registry.update({c["tag_id"]: (c["name"], "cube") for c in m["cubes"]})

    def get_objects(self, kinds=None):
        stamp = time.time()
        out = {}
        for tag_id, (name, kind) in self.registry.items():
            if kinds is not None and kind not in kinds:
                continue
            out[tag_id] = ObjectState(tag_id, name, kind, self.world.body_pose_base(name), stamp, self.source)
        return out

    def get_object(self, tag_id):
        name, kind = self.registry[tag_id]
        return ObjectState(tag_id, name, kind, self.world.body_pose_base(name), time.time(), self.source)

    def camera_pose(self):
        return self.world.camera_pose_base()
