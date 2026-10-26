# so101-ar-cobot

Shared-workspace cobot with an SO-101 arm and Meta Quest AR: the arm sorts tagged objects next to a human, with a runtime safety supervisor, gesture goals, teleop takeover, and a MuJoCo digital twin.

Course project for EC545 Cyber-Physical Systems, Boston University, Fall 2026.

## Setup

Python 3.12 is recommended (LeRobot needs it for the real hardware).

```
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Only needed when working with the real arm or camera:

```
pip install "lerobot[feetech,intelrealsense]"
```

## Quickstart

Run everything from the repo root and always use `python -m ...`, otherwise the imports between folders fail.

```
python -m sim.scene_builder --view
mjpython -m hardware.demo
python -m task.objects
```

The first command opens the generated scene in the MuJoCo viewer. The second runs a small motion demo through the robot API with the viewer open (on macOS the live viewer needs `mjpython`, which comes with mujoco). The third prints which cube goes to which bin and whether the robot or the human handles it.

## Repo layout

| Folder | What it is |
|---|---|
| `config/` | All settings: scene, robot, camera, calibration files |
| `sim/` | MuJoCo digital twin: scene generator, physics world, SO-101 model |
| `hardware/` | Device interface: same API for sim and real arm and camera |
| `perception/` | AprilTag detection, object poses |
| `controls/` | IK, trajectories, Cartesian commands |
| `supervisor/` | Runtime safety monitor between task/teleop and controls |
| `task/` | Object registry, task allocation, mode switching |
| `interfaces/` | Message definitions shared between modules |
| `quest_app/` | Unity app for the Meta Quest |

More detail in `interfaces/README.md` (frames and message types), `sim/README.md` and `hardware/README.md`.

## Sim or real

The backend is set only in the config:

- `config/robot.yaml` → `backend: sim` or `backend: real`
- `config/camera.yaml` → `backend: sim` or `backend: real`
- `config/perception.yaml` → `source: ground_truth` (sim only) or `source: apriltag`

Code never changes between the two.

## Team rules

1. Get devices only through `make_robot()` and `make_camera()` from `hardware`. Never import `sim` or `lerobot` in controls, perception or supervisor code.
2. Don't use `robot.world` or any sim ground truth inside the control loop. It does not exist on the real system. Ground truth is only for evaluation and debugging.
3. Settings go in `config/`, not hardcoded in the code.
4. Units: meters, radians in the sim internals; the robot API uses degrees for joints and 0 to 100 for the gripper, same as LeRobot.
5. Everything that leaves a module is in the `base` frame (robot base, x forward, z up). See `interfaces/README.md`.
