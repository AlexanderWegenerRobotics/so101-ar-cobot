# hardware

Device interface for the SO-101 arm and the RealSense D435i. Every device has a real backend (LeRobot) and a sim backend with the same API. Which one you get is decided by `backend` in the config, nothing else.

Mirrors LeRobot v0.6.2 (`SO101Follower`, `RealSenseCamera`).

## Robot

```python
from hardware import make_robot

robot = make_robot()
robot.connect()
obs = robot.get_observation()
robot.send_action({"shoulder_pan.pos": 30.0, "gripper.pos": 80.0})
robot.disconnect()
```

Joints: `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`, `gripper`. Keys are `<joint>.pos`. Joints are in degrees, the gripper goes from 0 (closed) to 100 (open).

Robot methods: `connect(calibrate=True)`, `disconnect()`, `is_connected`, `is_calibrated`, `calibrate()`, `configure()`, `setup_motors()`, `get_observation()`, `send_action()` (returns the action actually sent), `observation_features`, `action_features`, and `with robot:`.

Motor bus (`robot.bus`): `read`, `write`, `sync_read`, `sync_write` (with `normalize=` and `num_retry=`), `enable_torque`, `disable_torque`, `torque_disabled()`, calibration methods.

Registers in the sim:

- `Present_Position`, `Goal_Position`, `Torque_Enable`: behave like the real servo. Torque off makes the arm go limp and fall under gravity.
- `P_Coefficient` scales the sim stiffness, `Max_Torque_Limit` scales the sim force limit.
- `Present_Velocity`, `Present_Load`: rough approximations only.
- Unknown registers raise a `KeyError`.

Safety: `max_relative_target` in `config/robot.yaml` caps how far one command can move a joint, same logic as LeRobot.

Demo:

```
mjpython -m hardware.demo
python -m hardware.demo --backend real
```

## Camera

```python
from hardware import make_camera, camera_intrinsics

cam = make_camera()
cam.connect()
frame = cam.read_latest()
K = camera_intrinsics(cam)
cam.disconnect()
```

Methods: `connect(warmup=True)`, `disconnect()`, `is_connected`, `find_cameras()`, `read()`, `async_read()`, `read_latest()`, `read_depth()`, `async_read_depth()`, `read_latest_depth()`.

- Color: uint8, shape (H, W, 3), RGB by default.
- Depth: uint16 in millimeters, shape (H, W, 1), 0 where there is no valid reading.
- `camera_intrinsics(cam)` gives fx, fy, cx, cy and distortion coeffs for both backends. Use `depth=True` for the depth stream.

Depth runs at the same resolution as color (LeRobot does it that way), and the D435i depth sensor only goes up to 1280x720. So depth on means 1280x720 or lower for both streams. Recommended: 1920x1080 with depth off for tag detection.

If robot and camera both run in sim, they share one simulated world automatically.

## Config

`config/robot.yaml`:

- `backend`, `id`, `port`, `calibration_dir`, `use_degrees`, `max_relative_target`, `disable_torque_on_disconnect`
- `max_payload`, `max_grasp_width`: used by the task allocation (estimates, measure on the real arm)
- `sim.world`: `realtime`, `viewer`
- `sim.joint_map`: direction (`sign`) and `offset_deg` per joint between MuJoCo and the real arm

`config/camera.yaml`: same fields as LeRobot's `RealSenseCameraConfig` plus `backend`.

## Calibration

Calibration files live in `config/calibration/<id>.json`, not in LeRobot's default cache folder. The real arm writes it there on first calibration. The sim loads the same file, so the same physical pose gives the same numbers in sim and real. Without a file the sim builds a calibration from the joint limits of the MuJoCo model.

## Check sim against real (to do once the arm is here)

The MuJoCo zero pose looks like LeRobot's calibration pose (upper arm vertical, forearm horizontal), but the joint directions are not verified yet.

1. Calibrate the real arm (`python -m hardware.demo --backend real` asks for it on first connect).
2. Put the arm in the calibration pose: every joint should read about 0 degrees.
3. Move each joint by hand in the positive direction and check that the sim moves the same way with the same values.
4. Fix `sign` or `offset_deg` in `sim.joint_map` wherever it doesn't match.

## Real hardware notes

- Find the arm port with `lerobot-find-port` and the camera serial with `lerobot-find-cameras realsense`.
- pyrealsense2 has historically been hard to install on Apple Silicon. If it fails, run the camera from the Windows PC.
