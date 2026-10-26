import logging
from dataclasses import dataclass, field
from functools import cached_property
from pathlib import Path

from hardware.sim_bus import (
    DeviceAlreadyConnectedError,
    DeviceNotConnectedError,
    Motor,
    MotorNormMode,
    SimMotorsBus,
    load_calibration,
)

logger = logging.getLogger(__name__)

MOTORS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


@dataclass
class SimSO101FollowerConfig:
    id: str | None = None
    calibration_dir: Path | None = None
    port: str = "sim"
    disable_torque_on_disconnect: bool = True
    max_relative_target: float | dict[str, float] | None = None
    cameras: dict = field(default_factory=dict)
    use_degrees: bool = True
    position_p_coefficient: int = 16
    position_i_coefficient: int = 0
    position_d_coefficient: int = 32
    num_read_retries: int = 2
    joint_map: dict = field(default_factory=dict)


def ensure_safe_goal_position(goal_present_pos, max_relative_target):
    if isinstance(max_relative_target, (int, float)):
        cap = dict.fromkeys(goal_present_pos, float(max_relative_target))
    else:
        cap = max_relative_target
    safe = {}
    for key, (goal, present) in goal_present_pos.items():
        diff = max(-cap[key], min(cap[key], goal - present))
        safe[key] = present + diff
        if abs(safe[key] - goal) > 1e-4:
            logger.warning(f"{key}: goal {goal:.2f} clipped to {safe[key]:.2f}")
    return safe


class SimSO101Follower:
    config_class = SimSO101FollowerConfig
    name = "so_follower"
    supports_position_hold = True

    def __init__(self, config, world):
        self.config = config
        self.world = world
        self.robot_type = self.name
        self.id = config.id
        self.calibration_dir = Path(config.calibration_dir) if config.calibration_dir else None
        self.calibration_fpath = self.calibration_dir / f"{self.id}.json" if self.calibration_dir else None
        body_mode = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100
        motors = {m: Motor(i + 1, "sts3215", body_mode) for i, m in enumerate(MOTORS)}
        motors["gripper"] = Motor(6, "sts3215", MotorNormMode.RANGE_0_100)
        self.bus = SimMotorsBus(world, motors, joint_map=config.joint_map)
        self.calibration = {}
        if self.calibration_fpath is not None and self.calibration_fpath.is_file():
            self.calibration = load_calibration(self.calibration_fpath)
            logger.info(f"sim uses real calibration file {self.calibration_fpath}")
        self.bus.calibration = self.calibration
        self.cameras = {}

    def __str__(self):
        return f"{self.id} {self.__class__.__name__}"

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.disconnect()

    @cached_property
    def observation_features(self):
        return {f"{m}.pos": float for m in self.bus.motors}

    @cached_property
    def action_features(self):
        return {f"{m}.pos": float for m in self.bus.motors}

    @property
    def is_connected(self):
        return self.bus.is_connected

    @property
    def is_calibrated(self):
        return self.bus.is_calibrated

    def connect(self, calibrate=True):
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} is already connected.")
        self.bus.connect()
        if not self.is_calibrated and calibrate:
            self.calibrate()
        self.configure()
        logger.info(f"{self} connected.")

    def calibrate(self):
        if self.calibration:
            self.bus.write_calibration(self.calibration)
            return
        self.calibration = self.bus.synthetic_calibration()
        self.bus.write_calibration(self.calibration)
        logger.info("no calibration file found, sim uses a calibration derived from the MJCF joint ranges")

    def configure(self):
        with self.bus.torque_disabled():
            self.bus.configure_motors()
            for motor in self.bus.motors:
                self.bus.set_operating_mode("position", motor)
                self.bus.write("P_Coefficient", motor, self.config.position_p_coefficient)
                self.bus.write("I_Coefficient", motor, self.config.position_i_coefficient)
                self.bus.write("D_Coefficient", motor, self.config.position_d_coefficient)
                if motor == "gripper":
                    self.bus.write("Max_Torque_Limit", motor, 500)
                    self.bus.write("Protection_Current", motor, 250)
                    self.bus.write("Overload_Torque", motor, 25)

    def setup_motors(self):
        self.bus.setup_motors()

    def get_observation(self):
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self.__class__.__name__} is not connected. Run `.connect()` first.")
        obs = self.bus.sync_read("Present_Position", num_retry=self.config.num_read_retries)
        return {f"{m}.pos": v for m, v in obs.items()}

    def send_action(self, action):
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self.__class__.__name__} is not connected. Run `.connect()` first.")
        goal = {k.removesuffix(".pos"): v for k, v in action.items() if k.endswith(".pos")}
        if self.config.max_relative_target is not None:
            present = self.bus.sync_read("Present_Position", num_retry=self.config.num_read_retries)
            goal = ensure_safe_goal_position({k: (g, present[k]) for k, g in goal.items()},
                                             self.config.max_relative_target)
        self.bus.sync_write("Goal_Position", goal)
        return {f"{m}.pos": v for m, v in goal.items()}

    def disconnect(self):
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self.__class__.__name__} is not connected. Run `.connect()` first.")
        self.bus.disconnect(self.config.disable_torque_on_disconnect)
        logger.info(f"{self} disconnected.")
