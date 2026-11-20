# controls

The arm controller: the only node that moves the arm. It takes `arm/command` goals from the supervisor, plans
them, and streams joint positions to the robot through `make_robot()`, so the same code drives the MuJoCo sim and
the real SO-101 (STS3215 servos in position mode).

```
mjpython launch.py dev_viewer        # sim + viewer + controller + pass-through supervisor
python -m controls.demo              # sort every cube the robot may lift into its bin
python -m controls.demo --tag 12     # one cube
python -m controls.demo --home       # go home
```

## Files

| File | What |
|---|---|
| `kinematics.py` | FK and IK on the MuJoCo arm model (`sim/model/so101.xml`), tool frame = `gripperframe` site. Robot units (degrees, gripper 0 to 100) to radians via `robot.yaml` `sim.joint_map`. |
| `planner.py` | Tool pose goal to joint path: lift, travel, descend. |
| `trajectory.py` | Minimum jerk timing over a sampled joint path, time-warped by the speed scale. |
| `node.py` | `ArmController` node (60 Hz). |
| `demo.py` | Pick-and-place over the bus, a stand-in for the task node. |

## How a pose goal runs

1. **IK** (damped least squares, numeric Jacobian, joint limits clamped). The tool has 5 DoF: position, `pitch`
   (angle of the tool axis from horizontal, -90 deg = straight down) and `roll` about the tool axis. Yaw is not
   free, it follows the pan joint. `roll = 0` means the jaws close along the horizontal line perpendicular to the
   arm direction.
2. **Pitch relaxation.** Straight down is only reachable on part of the table (the green zone of the workspace
   overlay). Every corner of the path (lift top, point above the target, target) gets the steepest reachable pitch
   at or after the requested one, at 1 deg resolution, up to 25 deg. Within a segment the pitch changes linearly
   between its corners; if the middle of a segment is out of reach it gets an extra smooth tilt (a sine bump that
   is 0 at both ends). `arm/status` `accepted` carries `reason: "pitch relaxed to -82 deg"`.
3. **Path.** From the current tool pose: vertical lift to `safe_z` (8 cm), travel at that height, vertical descent
   onto the target. The travel is interpolated in cylindrical coordinates around the pan axis, so it arcs around the
   base instead of cutting through the area the arm cannot reach. Path samples (4 mm apart) are solved by tracking
   IK from the previous sample (no reseeding, so no jumps to another IK branch; more than 0.12 rad between samples
   counts as a jump). If a straight tool path is not possible on the current branch, that segment becomes a joint
   space move (kind `..._joint` in the log). From home (arm upright) the first move is a joint move to the point
   above the target. Moves shorter than 3 cm sideways go straight. Everything is solved when the goal arrives
   (6 to 300 ms), so an unreachable goal is `rejected` with `unreachable: ...` before anything moves.
4. **Blending and timing.** The segments of one goal (lift, travel, descend) run as one continuous motion, no stop
   at the corners. Their joint samples are concatenated and the corners rounded with a moving average (6 samples,
   about 2 cm; the window shrinks to 0 at the start and the target, so both stay exact). Vertical moves shorter than
   2 cm are folded into the travel instead of becoming a tiny separate hop. The path parameter is spaced by the
   time each step needs at the speed limits (tool `max_speed`, 2.5x that on joint space parts, joint speed
   1.5 rad/s at 0.1 m/s), and one minimum jerk profile `s = 10t^3 - 15t^4 + 6t^5` runs over the whole path with
   `T = max(1.875 * path time, acceleration limit, 0.25 s)`. Acceleration limits: tool 0.5 m/s^2, joints
   4 rad/s^2, both scaled with `max_speed / 0.1`, using peak min-jerk acceleration `5.774 L / T^2`.
5. **Speed scale.** Progress advances by `dt * speed_scale / T`. The supervisor's `speed_scale` slows the arm along
   the same path, 0 holds it in place, and it continues when the scale comes back.
6. **Gripper** moves after the arm arrives (150 %/s), then 0.4 s settle, then `succeeded`.

## Goal kinds

| `kind` | Behaviour |
|---|---|
| `pose` | Plan and execute as above, then set `gripper` if given. |
| `gripper` | Only move the gripper to `gripper` (0 closed, 100 open). |
| `joints` | Joint space min jerk move to `joints` (degrees, missing joints keep their value). |
| `home` | Joint move to `home_qpos` from `robot.yaml`. |
| `stream` | Teleop: follow `target` directly, tool speed limited to `max_speed`, pitch/roll at 2 rad/s, small IK step per tick. Holds if no update for 0.2 s. Keep the same `goal_id` while streaming. |
| `stop` | Preempt the current goal and hold. |

A new goal preempts the running one (`preempted`). Status sequence: `accepted` (or `rejected`), `running`,
`succeeded` / `preempted`.

## Safety

- Commands come only from `arm/command` (the supervisor), never from `arm/goal`.
- No `safety` message for 0.2 s or `stop: true`: the arm holds (`ArmState.status = stopped`) and resumes when the
  heartbeat and a clear state come back. The goal is not dropped.
- `supervisor/passthrough.py` is a placeholder that forwards `arm/goal` to `arm/command` and publishes
  `speed_scale: 1`. The real supervisor replaces it in `config/launch.yaml`.

## Parameters (`config/launch.yaml`)

`safe_z` 0.08, `min_z` 0.0, `pitch_tolerance_deg` 25, `safety_timeout` 0.2, `stream_timeout` 0.2,
`require_safety` true, `gripper_speed` 150, `gripper_settle` 0.4, `state_rate_hz` 30.

## Logs

`launch.py` writes everything to `logs/<date>_<time>_<profile>.log` (the terminal shows INFO, the file DEBUG too).
The controller logs every plan (segments, duration, start/end joints, peak joint speed and acceleration from stepping the plan at 60 Hz) and, every 0.1 s while
moving, the commanded joints, the tracking error (command minus measured, rad) and the gripper.

```
grep planned logs/*.log            # one line per goal
grep "moving cmd" logs/*.log       # tracking
```

## Blocked detection

The servos are position controlled, so if something is in the way the arm stops while the command keeps going.
When the tracking error (command minus measured) stays above 0.2 rad for 0.5 s, the goal fails with
`blocked (tracking error X rad)` and the command is set to the measured position, so the arm stops pushing.
Parameters `blocked_error`, `blocked_time`. Contact detection, not a replacement for the supervisor's distance
zones.

## Known limits

- Preempting a moving goal restarts from rest at the current command (a short jerk). Fine for now.
- No collision checking beyond "not below the table". Bins and other cubes are cleared by the 8 cm travel height.
