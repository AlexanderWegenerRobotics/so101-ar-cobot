from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
PERCEPTION_CONFIG = ROOT / "config" / "perception.yaml"


def load_perception_config(path=PERCEPTION_CONFIG):
    with open(path) as f:
        return yaml.safe_load(f)


def make_object_tracker(source=None, cfg=None, world=None):
    cfg = cfg or load_perception_config()
    source = source or cfg["source"]
    if source == "ground_truth":
        from hardware import get_sim_world
        from perception.ground_truth import GroundTruthTracker
        return GroundTruthTracker(world or get_sim_world())
    if source == "apriltag":
        raise NotImplementedError("apriltag tracker is not written yet, use source: ground_truth")
    raise ValueError(f"unknown source {source!r}, use 'ground_truth' or 'apriltag'")
