from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
ROBOT_CONFIG = ROOT / "config" / "robot.yaml"
CAMERA_CONFIG = ROOT / "config" / "camera.yaml"

_shared_world = None


def load_robot_config(path=ROBOT_CONFIG):
    with open(path) as f:
        return yaml.safe_load(f)


def load_camera_config(path=CAMERA_CONFIG):
    with open(path) as f:
        return yaml.safe_load(f)


def get_sim_world(robot_cfg=None, **world_kwargs):
    global _shared_world
    if _shared_world is None:
        from sim.world import SimWorld
        robot_cfg = robot_cfg or load_robot_config()
        kwargs = {**robot_cfg.get("sim", {}).get("world", {}), **world_kwargs}
        _shared_world = SimWorld(robot_cfg=robot_cfg, **kwargs)
    return _shared_world


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
        world = world or get_sim_world(cfg)
        config = SimSO101FollowerConfig(joint_map=cfg.get("sim", {}).get("joint_map", {}), **common)
        return SimSO101Follower(config, world)
    raise ValueError(f"unknown backend {backend!r}, use 'sim' or 'real'")


def make_camera(backend=None, cfg=None, world=None):
    cfg = cfg or load_camera_config()
    backend = backend or cfg["backend"]
    common = dict(
        serial_number_or_name=str(cfg["serial_number_or_name"]),
        fps=cfg["fps"], width=cfg["width"], height=cfg["height"],
        color_mode=cfg.get("color_mode", "rgb"), use_rgb=cfg.get("use_rgb", True),
        use_depth=cfg.get("use_depth", False), rotation=cfg.get("rotation", 0),
        warmup_s=cfg.get("warmup_s", 1), exposure=cfg.get("exposure"),
        gain=cfg.get("gain"), white_balance=cfg.get("white_balance"),
    )
    if backend == "real":
        from lerobot.cameras.realsense import RealSenseCamera, RealSenseCameraConfig
        return RealSenseCamera(RealSenseCameraConfig(**common))
    if backend == "sim":
        from hardware.sim_realsense import SimRealSenseCamera, SimRealSenseCameraConfig
        return SimRealSenseCamera(SimRealSenseCameraConfig(**common), world or get_sim_world())
    raise ValueError(f"unknown backend {backend!r}, use 'sim' or 'real'")


def camera_intrinsics(camera, depth=False):
    if hasattr(camera, "intrinsics"):
        return camera.intrinsics(depth)
    import pyrealsense2 as rs
    stream = rs.stream.depth if depth else rs.stream.color
    i = camera.rs_profile.get_stream(stream).as_video_stream_profile().get_intrinsics()
    return {"width": i.width, "height": i.height, "fx": i.fx, "fy": i.fy,
            "cx": i.ppx, "cy": i.ppy, "coeffs": list(i.coeffs)}
