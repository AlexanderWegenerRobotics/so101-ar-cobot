import json
import math
from contextlib import contextmanager
from dataclasses import asdict, dataclass
from enum import Enum

import numpy as np

try:
    from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
except ImportError:
    class DeviceNotConnectedError(ConnectionError):
        pass

    class DeviceAlreadyConnectedError(ConnectionError):
        pass

RESOLUTION = 4096
MAX_RES = RESOLUTION - 1
HALF_TURN = MAX_RES // 2
DEFAULT_P_COEFFICIENT = 16


class MotorNormMode(str, Enum):
    RANGE_0_100 = "range_0_100"
    RANGE_M100_100 = "range_m100_100"
    DEGREES = "degrees"


@dataclass
class Motor:
    id: int
    model: str
    norm_mode: MotorNormMode


@dataclass
class MotorCalibration:
    id: int
    drive_mode: int
    homing_offset: int
    range_min: int
    range_max: int


def load_calibration(path):
    raw = json.loads(path.read_text())
    return {name: MotorCalibration(**vals) for name, vals in raw.items()}


def save_calibration(calibration, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({k: asdict(v) for k, v in calibration.items()}, indent=4))


class SimMotorsBus:
    normalized_data = frozenset({"goal_position", "present_position"})
    stored_registers = {
        "P_Coefficient": DEFAULT_P_COEFFICIENT, "I_Coefficient": 0, "D_Coefficient": 32,
        "Max_Torque_Limit": 1000, "Protection_Current": 500, "Overload_Torque": 100,
        "Operating_Mode": 0, "Lock": 0, "Return_Delay_Time": 0, "Acceleration": 0,
        "Maximum_Acceleration": 254,
    }

    def __init__(self, world, motors, calibration=None, joint_map=None):
        self.world = world
        self.model = world.model
        self.data = world.data
        self.motors = motors
        self.calibration = calibration or {}
        self.joint_map = joint_map or {}
        self._connected = False
        self._torque = {m: True for m in motors}
        self._regs = {m: dict(self.stored_registers) for m in motors}
        self._jnt = {m: self.model.joint(m) for m in motors}
        self._act = {m: self.model.actuator(m).id for m in motors}
        self._kp0 = {m: float(self.model.actuator_gainprm[a, 0]) for m, a in self._act.items()}
        self._kv0 = {m: float(-self.model.actuator_biasprm[a, 2]) for m, a in self._act.items()}
        self._force0 = {m: self.model.actuator_forcerange[a].copy() for m, a in self._act.items()}

    def __str__(self):
        return f"SimMotorsBus({list(self.motors)})"

    @property
    def is_connected(self):
        return self._connected

    def connect(self, handshake=True):
        if self._connected:
            raise DeviceAlreadyConnectedError(f"{self} is already connected.")
        self._connected = True
        if not self.world.is_running:
            self.world.start()

    def disconnect(self, disable_torque=True):
        if not self._connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        if disable_torque:
            self.disable_torque()
        self._connected = False

    def _check(self):
        if not self._connected:
            raise DeviceNotConnectedError(f"{self} is not connected. Run `.connect()` first.")

    def _names(self, motors):
        if motors is None:
            return list(self.motors)
        if isinstance(motors, (str, int)):
            motors = [motors]
        by_id = {m.id: n for n, m in self.motors.items()}
        return [by_id[m] if isinstance(m, int) else m for m in motors]

    def _mapping(self, motor):
        return self.joint_map.get(motor, {})

    def _q_range(self, motor):
        return self.model.jnt_range[self._jnt[motor].id]

    def _q_to_raw(self, motor, q):
        mp = self._mapping(motor)
        sign = mp.get("sign", 1)
        if mp.get("mode", "angle") == "range":
            lo, hi = self._q_range(motor)
            cal = self.calibration[motor]
            frac = (q - lo) / (hi - lo)
            frac = frac if sign > 0 else 1 - frac
            return cal.range_min + frac * (cal.range_max - cal.range_min)
        offset = math.radians(mp.get("offset_deg", 0.0))
        return HALF_TURN + sign * (q + offset) * MAX_RES / (2 * math.pi)

    def _raw_to_q(self, motor, raw):
        mp = self._mapping(motor)
        sign = mp.get("sign", 1)
        if mp.get("mode", "angle") == "range":
            lo, hi = self._q_range(motor)
            cal = self.calibration[motor]
            frac = (raw - cal.range_min) / (cal.range_max - cal.range_min)
            frac = frac if sign > 0 else 1 - frac
            return lo + frac * (hi - lo)
        offset = math.radians(mp.get("offset_deg", 0.0))
        return sign * (raw - HALF_TURN) * 2 * math.pi / MAX_RES - offset

    def _normalize(self, motor, raw):
        cal = self.calibration[motor]
        lo, hi = cal.range_min, cal.range_max
        mode = self.motors[motor].norm_mode
        bounded = min(hi, max(lo, raw))
        if mode is MotorNormMode.RANGE_M100_100:
            return ((bounded - lo) / (hi - lo)) * 200 - 100
        if mode is MotorNormMode.RANGE_0_100:
            return ((bounded - lo) / (hi - lo)) * 100
        return (raw - (lo + hi) / 2) * 360 / MAX_RES

    def _unnormalize(self, motor, value):
        cal = self.calibration[motor]
        lo, hi = cal.range_min, cal.range_max
        mode = self.motors[motor].norm_mode
        if mode is MotorNormMode.RANGE_M100_100:
            return int(((min(100.0, max(-100.0, value)) + 100) / 200) * (hi - lo) + lo)
        if mode is MotorNormMode.RANGE_0_100:
            return int((min(100.0, max(0.0, value)) / 100) * (hi - lo) + lo)
        return int(value * MAX_RES / 360 + (lo + hi) / 2)

    def _read_raw(self, data_name, motor):
        j, a = self._jnt[motor], self._act[motor]
        if data_name == "Present_Position":
            return int(round(self._q_to_raw(motor, self.data.qpos[j.qposadr[0]])))
        if data_name == "Goal_Position":
            return int(round(self._q_to_raw(motor, self.data.ctrl[a])))
        if data_name == "Present_Velocity":
            sign = self._mapping(motor).get("sign", 1)
            return int(round(sign * self.data.qvel[j.dofadr[0]] * MAX_RES / (2 * math.pi)))
        if data_name == "Present_Load":
            limit = max(abs(self._force0[motor][1]), 1e-9)
            return int(round(1000 * self.data.actuator_force[a] / limit))
        if data_name == "Torque_Enable":
            return int(self._torque[motor])
        if data_name in self._regs[motor]:
            return self._regs[motor][data_name]
        raise KeyError(f"register {data_name!r} is not simulated")

    def _write_raw(self, data_name, motor, raw):
        a = self._act[motor]
        if data_name == "Goal_Position":
            lo, hi = self.model.actuator_ctrlrange[a]
            self.data.ctrl[a] = float(np.clip(self._raw_to_q(motor, raw), lo, hi))
        elif data_name == "Torque_Enable":
            self._set_torque(motor, bool(raw))
        elif data_name in self._regs[motor]:
            self._regs[motor][data_name] = raw
            self._apply_gains(motor)
        else:
            raise KeyError(f"register {data_name!r} is not simulated")

    def _apply_gains(self, motor):
        a = self._act[motor]
        if not self._torque[motor]:
            return
        scale = self._regs[motor]["P_Coefficient"] / DEFAULT_P_COEFFICIENT
        kp = self._kp0[motor] * scale
        self.model.actuator_gainprm[a, 0] = kp
        self.model.actuator_biasprm[a, 1] = -kp
        self.model.actuator_biasprm[a, 2] = -self._kv0[motor]
        self.model.actuator_forcerange[a] = self._force0[motor] * self._regs[motor]["Max_Torque_Limit"] / 1000

    def _set_torque(self, motor, on):
        a, j = self._act[motor], self._jnt[motor]
        if on and not self._torque[motor]:
            self.data.ctrl[a] = self.data.qpos[j.qposadr[0]]
        self._torque[motor] = on
        if on:
            self._apply_gains(motor)
        else:
            self.model.actuator_gainprm[a, 0] = 0
            self.model.actuator_biasprm[a, 1] = 0
            self.model.actuator_biasprm[a, 2] = 0

    def read(self, data_name, motor, *, normalize=True, num_retry=0):
        return self.sync_read(data_name, motor, normalize=normalize, num_retry=num_retry)[self._names(motor)[0]]

    def write(self, data_name, motor, value, *, normalize=True, num_retry=0):
        self.sync_write(data_name, {self._names(motor)[0]: value}, normalize=normalize, num_retry=num_retry)

    def sync_read(self, data_name, motors=None, *, normalize=True, num_retry=0):
        self._check()
        out = {}
        with self.world.lock:
            for m in self._names(motors):
                raw = self._read_raw(data_name, m)
                out[m] = self._normalize(m, raw) if normalize and data_name.lower() in self.normalized_data else raw
        return out

    def sync_write(self, data_name, values, *, normalize=True, num_retry=0):
        self._check()
        with self.world.lock:
            for m, v in values.items():
                raw = self._unnormalize(m, v) if normalize and data_name.lower() in self.normalized_data else v
                self._write_raw(data_name, m, raw)

    def enable_torque(self, motors=None, num_retry=0):
        with self.world.lock:
            for m in self._names(motors):
                self._set_torque(m, True)

    def disable_torque(self, motors=None, num_retry=0):
        with self.world.lock:
            for m in self._names(motors):
                self._set_torque(m, False)

    @contextmanager
    def torque_disabled(self, motors=None):
        self.disable_torque(motors)
        try:
            yield
        finally:
            self.enable_torque(motors)

    @property
    def is_calibrated(self):
        return bool(self.calibration) and set(self.calibration) == set(self.motors)

    def read_calibration(self):
        return dict(self.calibration)

    def write_calibration(self, calibration_dict, cache=True):
        if cache:
            self.calibration = dict(calibration_dict)

    def reset_calibration(self, motors=None):
        pass

    def configure_motors(self, *args, **kwargs):
        pass

    def set_operating_mode(self, mode, motors=None):
        for m in self._names(motors):
            self._regs[m]["Operating_Mode"] = 0 if mode == "position" else self._regs[m]["Operating_Mode"]

    def setup_motors(self, motors=None):
        pass

    def ping(self, motor, num_retry=0):
        return 3215 if self._names(motor)[0] in self.motors else None

    def synthetic_calibration(self):
        cal = {}
        for name, motor in self.motors.items():
            if self._mapping(name).get("full_turn"):
                lo, hi = 0, MAX_RES
            else:
                q_lo, q_hi = self._q_range(name)
                saved = self.calibration
                self.calibration = {}
                ends = sorted(self._q_to_raw_angle(name, q) for q in (q_lo, q_hi))
                self.calibration = saved
                lo, hi = int(math.floor(ends[0])), int(math.ceil(ends[1]))
            cal[name] = MotorCalibration(id=motor.id, drive_mode=0, homing_offset=0, range_min=lo, range_max=hi)
        return cal

    def _q_to_raw_angle(self, motor, q):
        mp = self._mapping(motor)
        offset = math.radians(mp.get("offset_deg", 0.0))
        return HALF_TURN + mp.get("sign", 1) * (q + offset) * MAX_RES / (2 * math.pi)
