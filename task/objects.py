import json
from dataclasses import dataclass
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MANIFEST = ROOT / "sim" / "generated" / "scene_manifest.json"
ROBOT_CONFIG = ROOT / "config" / "robot.yaml"


@dataclass
class Cube:
    tag_id: int
    name: str
    type: str
    color: str
    size: float
    mass: float
    robot_feasible: bool
    reason: str


@dataclass
class Bin:
    tag_id: int
    name: str
    color: str


def check_feasible(size, mass, robot):
    reasons = []
    if mass > robot["max_payload"]:
        reasons.append("too_heavy")
    if size > robot["max_grasp_width"]:
        reasons.append("too_large")
    return not reasons, ",".join(reasons)


def load_objects(manifest_path=DEFAULT_MANIFEST, robot_path=ROBOT_CONFIG):
    manifest = json.loads(Path(manifest_path).read_text())
    robot = yaml.safe_load(Path(robot_path).read_text())
    cubes = {}
    for c in manifest["cubes"]:
        ok, reason = check_feasible(c["size"], c["mass"], robot)
        cubes[c["tag_id"]] = Cube(c["tag_id"], c["name"], c["type"], c["color"], c["size"], c["mass"], ok, reason)
    bins = {b["tag_id"]: Bin(b["tag_id"], b["name"], b["color"]) for b in manifest["bins"]}
    return cubes, bins


def target_bin(cube, bins):
    return next(b for b in bins.values() if b.color == cube.color)


if __name__ == "__main__":
    cubes, bins = load_objects()
    for cube in cubes.values():
        owner = "robot" if cube.robot_feasible else f"human ({cube.reason})"
        print(f"tag {cube.tag_id:3d}  {cube.type:8s} {cube.color:5s} -> {target_bin(cube, bins).name:9s} {owner}")
