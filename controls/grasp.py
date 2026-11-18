from dataclasses import dataclass

import numpy as np

from controls.trajectory import LinePath
from interfaces.messages import ToolTarget

STRAIGHT_DOWN = -np.pi / 2
IK_SEED = [0.0, -0.4, 1.2, 0.77]
PATH_SAMPLES = 15


@dataclass
class Step:
    name: str
    target: ToolTarget
    gripper: float
    dwell: float = 0.0


def wrap_quarter_turn(angle):
    return (angle + np.pi / 4) % (np.pi / 2) - np.pi / 4


def seed_towards(kinematics, position):
    pan_x, pan_y = kinematics.joints[0].frame.pos[:2]
    return [-np.arctan2(position[1] - pan_y, position[0] - pan_x), *IK_SEED[1:]]


def grasp_roll(kinematics, position, cube_yaw):
    target = ToolTarget(list(position), STRAIGHT_DOWN, 0.0)
    q = kinematics.inverse(target, seed_towards(kinematics, position))
    if q is None:
        return 0.0
    jaw_axis = kinematics.forward(q).rotation[:, 1]
    return wrap_quarter_turn(cube_yaw - np.arctan2(jaw_axis[1], jaw_axis[0]))


def plan_pick_and_place(kinematics, cube_pose, bin_position, params):
    x, y, z = cube_pose.pos
    roll = grasp_roll(kinematics, cube_pose.pos, cube_pose.yaw())
    transit = np.radians(params["transit_pitch_deg"])
    above_cube = [x, y, params["hover_height"]]
    over_bin = [bin_position[0], bin_position[1], params["release_height"]]
    in_front = [params["via_x"], 0.0, params["release_height"]]
    opened, closed, dwell = params["gripper_open"], params["gripper_closed"], params["dwell"]
    return [
        Step("hover over cube", ToolTarget(above_cube, transit, roll), opened),
        Step("descend", ToolTarget([x, y, z], STRAIGHT_DOWN, roll), opened),
        Step("grasp", ToolTarget([x, y, z], STRAIGHT_DOWN, roll), closed, dwell),
        Step("lift", ToolTarget(above_cube, transit, roll), closed),
        Step("pass in front of base", ToolTarget(in_front, transit, 0.0), closed),
        Step("move over bin", ToolTarget(over_bin, transit, 0.0), closed),
        Step("release", ToolTarget(over_bin, transit, 0.0), opened, dwell),
        Step("return in front of base", ToolTarget(in_front, transit, 0.0), opened),
    ]


def unreachable_steps(kinematics, start, steps, max_speed, max_angular_speed):
    problems = []
    seed = seed_towards(kinematics, start.pos)
    for step in steps:
        path = LinePath(start, step.target, max_speed, max_angular_speed)
        for t in np.linspace(0.0, path.duration, PATH_SAMPLES):
            q = kinematics.inverse(path.at(t), seed)
            if q is None:
                problems.append(step.name)
                break
            seed = q[:4]
        start = step.target
    return problems
