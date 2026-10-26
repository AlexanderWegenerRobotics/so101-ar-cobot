import dataclasses
import time
import types
import typing
from dataclasses import dataclass, field

import numpy as np

from interfaces.objects import ObjectState
from interfaces.transforms import Pose

HAND_JOINTS = [
    "palm", "wrist",
    "thumb_metacarpal", "thumb_proximal", "thumb_distal", "thumb_tip",
    "index_metacarpal", "index_proximal", "index_intermediate", "index_distal", "index_tip",
    "middle_metacarpal", "middle_proximal", "middle_intermediate", "middle_distal", "middle_tip",
    "ring_metacarpal", "ring_proximal", "ring_intermediate", "ring_distal", "ring_tip",
    "little_metacarpal", "little_proximal", "little_intermediate", "little_distal", "little_tip",
]


class Topic:
    OBJECTS = "objects"
    HAND = "hand"
    ARM_GOAL = "arm/goal"
    ARM_COMMAND = "arm/command"
    ARM_STATUS = "arm/status"
    ARM_STATE = "arm/state"
    ARM_PLAN = "arm/plan"
    SAFETY = "safety"
    TASK_STATE = "task/state"
    OPERATOR_COMMAND = "operator/command"
    OPERATOR_TELEOP = "operator/teleop"


def now():
    return time.time()


@dataclass
class ToolTarget:
    pos: list
    pitch: float = -np.pi / 2
    roll: float = 0.0


@dataclass
class ArmGoal:
    goal_id: str
    kind: str
    target: ToolTarget | None = None
    gripper: float | None = None
    joints: dict | None = None
    max_speed: float = 0.10
    source: str = "task"
    stamp: float = field(default_factory=now)


@dataclass
class GoalStatus:
    goal_id: str
    status: str
    reason: str = ""
    stamp: float = field(default_factory=now)


@dataclass
class ArmState:
    joints: dict
    status: str = "idle"
    goal_id: str | None = None
    tool: ToolTarget | None = None
    speed_scale: float = 1.0
    stamp: float = field(default_factory=now)


@dataclass
class ArmPlan:
    goal_id: str
    waypoints: list
    stamp: float = field(default_factory=now)


@dataclass
class SafetyState:
    speed_scale: float
    stop: bool = False
    zone: str = "clear"
    reason: str = ""
    min_distance: float | None = None
    stamp: float = field(default_factory=now)


@dataclass
class HandState:
    side: str
    tracked: bool
    joints: list = field(default_factory=list)
    pinch: float = 0.0
    ray_origin: list | None = None
    ray_dir: list | None = None
    source: str = "quest"
    stamp: float = field(default_factory=now)


@dataclass
class ObjectArray:
    objects: list
    source: str
    stamp: float = field(default_factory=now)


@dataclass
class ObjectTask:
    tag_id: int
    owner: str
    target_bin: int | None = None
    status: str = "pending"
    reason: str = ""


@dataclass
class TaskState:
    mode: str
    objects: list
    active_tag_id: int | None = None
    stamp: float = field(default_factory=now)


@dataclass
class OperatorCommand:
    kind: str
    mode: str | None = None
    tag_id: int | None = None
    bin_tag_id: int | None = None
    stamp: float = field(default_factory=now)


@dataclass
class TeleopTarget:
    target: ToolTarget
    gripper: float | None = None
    active: bool = True
    stamp: float = field(default_factory=now)


MESSAGE_TYPES = {cls.__name__: cls for cls in [
    ToolTarget, ArmGoal, GoalStatus, ArmState, ArmPlan, SafetyState, HandState,
    ObjectArray, ObjectTask, TaskState, OperatorCommand, TeleopTarget, ObjectState, Pose,
]}


def to_jsonable(obj):
    if isinstance(obj, Pose):
        return obj.to_dict()
    if isinstance(obj, ObjectState):
        return obj.to_dict()
    if dataclasses.is_dataclass(obj):
        return {f.name: to_jsonable(getattr(obj, f.name)) for f in dataclasses.fields(obj)}
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, (np.floating, np.integer)):
        return obj.item()
    if isinstance(obj, dict):
        return {k: to_jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [to_jsonable(v) for v in obj]
    return obj


def encode(msg):
    d = to_jsonable(msg)
    d["type"] = type(msg).__name__
    return d


def _decode_value(hint, value):
    if value is None:
        return None
    origin = typing.get_origin(hint)
    if origin in (typing.Union, types.UnionType):
        for arg in typing.get_args(hint):
            if arg is type(None):
                continue
            return _decode_value(arg, value)
    if hint is Pose:
        return Pose.from_dict(value)
    if hint is ObjectState:
        return ObjectState.from_dict(value)
    if dataclasses.is_dataclass(hint):
        return decode_as(hint, value)
    return value


def decode_as(cls, d):
    hints = typing.get_type_hints(cls)
    kwargs = {f.name: _decode_value(hints[f.name], d[f.name]) for f in dataclasses.fields(cls) if f.name in d}
    msg = cls(**kwargs)
    if cls is ObjectArray:
        msg.objects = [o if isinstance(o, ObjectState) else ObjectState.from_dict(o) for o in msg.objects]
    if cls is TaskState:
        msg.objects = [o if isinstance(o, ObjectTask) else decode_as(ObjectTask, o) for o in msg.objects]
    return msg


def decode(d):
    return decode_as(MESSAGE_TYPES[d["type"]], d)
