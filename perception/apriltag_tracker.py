import argparse
import time
from dataclasses import dataclass

import cv2
import numpy as np

from hardware import camera_intrinsics
from interfaces import ObjectState, Pose, pose_error
from perception.apriltag import camera_matrix, detect_tags, make_detector, tag_pose_camera


@dataclass
class TagSpec:
    name: str
    kind: str
    tag_size: float
    center_height: float
    tag_height: float


def make_spec(entry, kind, center_height):
    return TagSpec(entry["name"], kind, entry["tag_size"], center_height, center_height + entry["tag_offset"][2])


def build_registry(manifest):
    registry = {c["tag_id"]: make_spec(c, "cube", c["size"] / 2) for c in manifest["cubes"]}
    registry.update({b["tag_id"]: make_spec(b, "bin", 0.0) for b in manifest["bins"]})
    return registry


def point_at_height(origin, point, height):
    scale = (height - origin[2]) / (point[2] - origin[2])
    return origin + scale * (point - origin)


class AprilTagTracker:
    source = "apriltag"

    def __init__(self, camera, T_base_camera, registry, family):
        self.camera = camera
        self.T_base_camera = T_base_camera
        self.registry = registry
        self.detector = make_detector(family)
        i = camera_intrinsics(camera)
        self.K = np.array([[i["fx"], 0, i["cx"]], [0, i["fy"], i["cy"]], [0, 0, 1]])
        self.dist = np.array(i["coeffs"])
        self.last_seen = {}

    def get_objects(self, kinds=None):
        self._update()
        return {tag_id: obj for tag_id, obj in self.last_seen.items() if kinds is None or obj.kind in kinds}

    def get_object(self, tag_id):
        self._update()
        return self.last_seen[tag_id]

    def camera_pose(self):
        return self.T_base_camera

    def _update(self):
        gray = cv2.cvtColor(self.camera.read_latest(), cv2.COLOR_RGB2GRAY)
        stamp = time.time()
        for tag_id, corners in detect_tags(self.detector, gray).items():
            spec = self.registry.get(tag_id)
            if spec is None:
                continue
            pose = self._object_pose(corners, spec)
            if pose is not None:
                self.last_seen[tag_id] = ObjectState(tag_id, spec.name, spec.kind, pose, stamp, self.source)

    def _object_pose(self, corners, spec):
        tag_in_camera = tag_pose_camera(corners, spec.tag_size, self.K, self.dist)
        if tag_in_camera is None:
            return None
        tag_in_base = self.T_base_camera * tag_in_camera
        x, y, _ = point_at_height(self.T_base_camera.pos, tag_in_base.pos, spec.tag_height)
        yaw = tag_in_base.yaw()
        return Pose([x, y, spec.center_height], [np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)])


def sim_check():
    from hardware import get_sim_world, make_camera
    from perception.ground_truth import GroundTruthTracker
    from perception.qr_extrinsics import calibrate_camera

    world = get_sim_world()
    manifest = world.manifest
    with make_camera("sim") as camera:
        tracker = AprilTagTracker(camera, calibrate_camera(camera), build_registry(manifest), manifest["tag_family"])
        truth = GroundTruthTracker(world).get_objects()
        detected = tracker.get_objects()
    for tag_id, obj in detected.items():
        error = pose_error(truth[tag_id].pose, obj.pose)
        print(f"tag {tag_id:3d}  position error {error['pos'] * 1000:5.2f} mm  yaw error {np.degrees(error['yaw']):7.2f} deg")
    print(f"{len(detected)} of {len(truth)} objects detected")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", action="store_true")
    args = parser.parse_args()
    if args.sim:
        sim_check()


if __name__ == "__main__":
    main()
