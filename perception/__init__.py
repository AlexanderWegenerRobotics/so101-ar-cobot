import json
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
PERCEPTION_CONFIG = ROOT / "config" / "perception.yaml"


def load_perception_config(path=PERCEPTION_CONFIG):
    with open(path) as f:
        return yaml.safe_load(f)


def make_apriltag_tracker(cfg):
    from hardware import make_camera
    from perception.apriltag_tracker import AprilTagTracker, build_registry
    from perception.qr_extrinsics import calibrate_camera

    manifest = json.loads((ROOT / cfg["manifest"]).read_text())
    camera = make_camera()
    camera.connect()
    return AprilTagTracker(camera, calibrate_camera(camera), build_registry(manifest), manifest["tag_family"])


def make_object_tracker(source=None, cfg=None, world=None):
    cfg = cfg or load_perception_config()
    source = source or cfg["source"]
    if source == "ground_truth":
        from hardware import get_sim_world
        from perception.ground_truth import GroundTruthTracker
        return GroundTruthTracker(world or get_sim_world())
    if source == "apriltag":
        return make_apriltag_tracker(cfg)
    raise ValueError(f"unknown source {source!r}, use 'ground_truth' or 'apriltag'")
