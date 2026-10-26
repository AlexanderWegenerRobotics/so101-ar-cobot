from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
ROBOT_CONFIG = ROOT / "config" / "robot.yaml"


def load_robot_config(path=ROBOT_CONFIG):
    with open(path) as f:
        return yaml.safe_load(f)


def _calibration_dir(cfg):
    path = Path(cfg["calibration_dir"])
    return path if path.is_absolute() else ROOT / path


def make_robot(backend=None, cfg=None, world=None):
    cfg = cfg or load_robot_config()
    backend = backend or cfg["backend"]
    common = dict(
        id=cfg["id"],
        calibration_dir=_calibration_dir(cfg),
        disable_torque_on_disconnect=cfg.get("disable_torque_on_disconnect", True),
        max_relative_target=cfg.get("max_relative_target"),
        use_degrees=cfg.get("use_degrees", True),
    )
    if backend == "real":
        from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
        return SO101Follower(SO101FollowerConfig(port=cfg["port"], **common))
    if backend == "sim":
        from hardware.sim_so101 import SimSO101Follower, SimSO101FollowerConfig
        from sim.world import SimWorld
        world = world or SimWorld(robot_cfg=cfg, **cfg.get("sim", {}).get("world", {}))
        config = SimSO101FollowerConfig(joint_map=cfg.get("sim", {}).get("joint_map", {}), **common)
        return SimSO101Follower(config, world)
    raise ValueError(f"unknown backend {backend!r}, use 'sim' or 'real'")
