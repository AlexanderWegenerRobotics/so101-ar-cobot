import numpy as np

from controls.grasp import STRAIGHT_DOWN, grasp_roll, plan_pick_and_place, seed_towards, unreachable_steps, wrap_quarter_turn
from controls.kinematics import load_kinematics
from interfaces.messages import ToolTarget
from interfaces.transforms import Pose

PARAMS = {"hover_height": 0.05, "release_height": 0.08, "via_x": 0.18, "transit_pitch_deg": -80.0,
          "gripper_open": 80.0, "gripper_closed": 0.0, "dwell": 0.6}
SPEED, ANGULAR_SPEED = 0.10, 1.0
CUBE_HEIGHT = 0.015
BINS = [(0.0, 0.24), (0.0, -0.24)]
SCENE_SEED_0 = [(0.12017, -0.25359, 0.4331449, 0.9013243), (0.15258, 0.08758, 0.5242524, -0.8515629),
                (0.28446, 0.02684, 0.8084855, -0.5885161), (0.27202, 0.12180, 0.8903499, 0.4552769),
                (0.25798, -0.06643, 0.4117894, -0.9112791), (0.21865, -0.00703, 0.3402500, 0.9403350),
                (0.14547, 0.17795, 0.6545862, -0.7559873), (0.22461, 0.17125, 0.6831007, -0.7303242)]


def cube_pose(x, y, qw=1.0, qz=0.0):
    return Pose([x, y, CUBE_HEIGHT], [qw, 0.0, 0.0, qz])


def test_roll_aligns_jaws_with_cube_faces():
    kinematics = load_kinematics()
    for x, y in [(0.22, 0.0), (0.15, 0.18), (0.12, -0.25), (0.27, 0.12)]:
        for yaw in np.radians(np.arange(-180, 180, 23)):
            roll = grasp_roll(kinematics, [x, y, CUBE_HEIGHT], yaw)
            q = kinematics.inverse(ToolTarget([x, y, CUBE_HEIGHT], STRAIGHT_DOWN, roll), seed_towards(kinematics, [x, y]))
            jaw_axis = kinematics.forward(q).rotation[:, 1]
            misalignment = wrap_quarter_turn(np.arctan2(jaw_axis[1], jaw_axis[0]) - yaw)
            assert abs(misalignment) < np.radians(3.0)


def test_every_cube_in_the_default_scene_can_reach_both_bins():
    kinematics = load_kinematics()
    ready = ToolTarget([0.20, 0.0, 0.07], STRAIGHT_DOWN, 0.0)
    in_front = ToolTarget([PARAMS["via_x"], 0.0, PARAMS["release_height"]], np.radians(PARAMS["transit_pitch_deg"]), 0.0)
    for x, y, qw, qz in SCENE_SEED_0:
        for bin_position in BINS:
            steps = plan_pick_and_place(kinematics, cube_pose(x, y, qw, qz), bin_position, PARAMS)
            for start in (ready, in_front):
                assert unreachable_steps(kinematics, start, steps, SPEED, ANGULAR_SPEED) == []


def test_plan_ends_in_front_of_the_base_with_the_gripper_open():
    kinematics = load_kinematics()
    steps = plan_pick_and_place(kinematics, cube_pose(0.22, 0.0), BINS[0], PARAMS)
    assert [step.gripper for step in steps] == [80, 80, 0, 0, 0, 0, 80, 80]
    assert steps[-1].target.pos[:2] == [PARAMS["via_x"], 0.0]


def test_unreachable_cube_is_reported():
    kinematics = load_kinematics()
    ready = ToolTarget([0.20, 0.0, 0.07], STRAIGHT_DOWN, 0.0)
    steps = plan_pick_and_place(kinematics, cube_pose(0.5, 0.0), BINS[0], PARAMS)
    assert unreachable_steps(kinematics, ready, steps, SPEED, ANGULAR_SPEED) != []
