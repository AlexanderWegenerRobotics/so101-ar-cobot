# interfaces

Shared definitions every module uses: coordinate frames, the pose type and the object state message.

## Frames

| Frame | Where | Axes | Used by |
|---|---|---|---|
| `base` | Center of the robot base bottom (`baseframe` site in the MJCF) | x forward toward the workspace, y left, z up | Everything. This is the world frame. |
| `mujoco` | MuJoCo world origin | z up | Only inside `sim/`, never leaves it |
| `camera` | D435i color optical frame | OpenCV: x right, y down, z forward (viewing direction) | Perception |
| `quest` | Headset tracking origin | Unity: left-handed, y up | Only inside `quest_app/` and the Unity bridge |

Rules:

- Every pose that leaves a module is in `base`.
- Meters for positions, quaternions as (w, x, y, z), right-handed.
- Transforms are written `T_a_b` = pose of frame b expressed in frame a, so a point goes from b to a with `p_a = T_a_b * p_b`, and chains read left to right: `T_base_tag = T_base_camera * T_camera_tag`.
- In the current scene `base` and `mujoco` happen to be the same, but code never assumes that. The sim computes `T_base_mujoco` from the `baseframe` site.
- The conversion between `base` (right-handed, z up) and Unity (left-handed, y up) happens in exactly one place, the Unity bridge.

## Object pose

The pose of an object is its geometric center, not the tag pose. Detection gives the tag pose; the manifest stores `tag_offset` (tag center relative to the object center), and perception applies it. So ground truth and detection report the same thing.

- Cube: center of the cube, orientation of the cube body.
- Bin: center of the bin floor bottom, at table height.
- Anchor: center of the tag, on the table surface.

## Types

```python
from interfaces import Pose, ObjectState, pose_error

T_base_camera = Pose(pos=[0.18, 0.0, 0.70], quat=[0.0, 0.7071, -0.7071, 0.0])
p_base = T_base_camera * p_camera
T_camera_base = T_base_camera.inverse()
err = pose_error(ground_truth.pose, detected.pose)
```

`ObjectState`: `tag_id`, `name`, `kind` (`cube`, `bin`, `anchor`), `pose`, `stamp` (wall clock seconds), `source` (`ground_truth` or `apriltag`), `frame` (always `base`). `to_dict()` / `from_dict()` give a JSON-ready version for the Unity bridge.

Object properties like color, mass or "too heavy" are not part of `ObjectState`. They come from the object registry (`task/objects.py`), looked up by tag ID.

## Message bus

All modules talk only through messages on a ZeroMQ bus, never by calling each other. Python nodes run as threads in one process (`launch.py`) because in sim they share one MuJoCo world. Unity connects to the same bus over the network (NetMQ).

- Broker: publishers connect to port 5555, subscribers to port 5556 (`config/bus.yaml`). The launcher starts it.
- Wire format: ZeroMQ multipart `[topic, json]`. The JSON has a `type` field with the message class name. Field names are the same as in `interfaces/messages.py`.
- All poses in `base`. Positions in meters, angles in radians, quaternions (w, x, y, z), joints in degrees, gripper 0 (closed) to 100 (open), `stamp` in wall clock seconds.

```python
from interfaces.bus import Publisher, Subscriber
from interfaces.messages import Topic, ArmGoal, ToolTarget

pub = Publisher()
pub.publish(Topic.ARM_GOAL, ArmGoal("g1", "pose", ToolTarget([0.20, 0.05, 0.03])))

sub = Subscriber([Topic.OBJECTS, Topic.SAFETY])
safety = sub.latest(Topic.SAFETY, max_age=0.2)
```

Debug: `cobot-echo` prints everything on the bus, `cobot-echo objects hand` only those topics.

## Data flow

```
Quest app ──hand, operator/command, operator/teleop──▶ task ──arm/goal──▶ supervisor ──arm/command──▶ controls ──▶ robot
                                                         ▲                    │                          │
perception ──objects──▶ everyone                         └────arm/status──────┼──────────────────────────┤
                                                                              └──safety──▶ controls, Unity
controls ──arm/state, arm/plan──▶ everyone        task ──task/state──▶ Unity
```

## Topics

| Topic | Message | From → To | Rate |
|---|---|---|---|
| `objects` | `ObjectArray` | perception → all | 30 Hz |
| `hand` | `HandState` | Quest app (or sim hand) → all | 30 to 60 Hz |
| `arm/goal` | `ArmGoal` | task → supervisor | on event, 30 Hz when streaming |
| `arm/command` | `ArmGoal` | supervisor → controls | same as `arm/goal` |
| `arm/status` | `GoalStatus` | controls → task, supervisor | on change |
| `arm/state` | `ArmState` | controls → all | 30 Hz |
| `arm/plan` | `ArmPlan` | controls → Unity | once per goal |
| `safety` | `SafetyState` | supervisor → controls, Unity | 30 Hz |
| `task/state` | `TaskState` | task → Unity | 5 Hz and on change |
| `operator/command` | `OperatorCommand` | Quest app → task | on event |
| `operator/teleop` | `TeleopTarget` | Quest app → task | 30 Hz while teleop |
| `sim/scene_info` | `SceneInfo` | sim → Quest WorldRenderer only | 1 Hz |
| `sim/scene` | `ScenePoses` | sim → Quest WorldRenderer only | 60 Hz |

## Messages

**`ToolTarget`**: the 5-DoF tool pose the SO-101 can actually reach. `pos` (x, y, z of the gripper frame), `pitch` (tool angle from horizontal, -pi/2 = pointing straight down, the default for grasping from the top), `roll` (rotation about the tool axis, use the cube yaw for grasping). Yaw is not a field: it is always `atan2(y, x)`.

**`ArmGoal`**: `goal_id`, `kind`, `target`, `gripper`, `joints`, `max_speed` (m/s), `source` (`task` or `teleop`).

| `kind` | Meaning | Uses |
|---|---|---|
| `pose` | Move to `target`, controls plans the path (minimum jerk) | `target`, `gripper` |
| `stream` | Follow `target` directly, no planning (teleop) | `target`, `gripper`, sent at 30 Hz |
| `joints` | Move to joint angles | `joints`, `gripper` |
| `home` | Go to the home pose | |
| `stop` | Stop now and hold position | |

**`GoalStatus`**: `goal_id`, `status` = `accepted`, `running`, `succeeded`, `failed`, `rejected`, `preempted`, plus `reason` (for example `unreachable`, `too_heavy`, `safety_stop`).

**`ArmState`**: `joints` (dict, degrees, gripper 0 to 100), `status` (`idle`, `moving`, `holding`, `stopped`), `goal_id`, `tool` (current `ToolTarget` from forward kinematics), `speed_scale` (what controls is applying right now).

**`ArmPlan`**: `goal_id`, `waypoints` (list of tool positions). For the AR path overlay.

**`SafetyState`**: `speed_scale` 0 to 1, `stop`, `zone` (`clear`, `slow`, `stop`), `reason`, `min_distance` (hand to arm, meters).

**`HandState`**: `side`, `tracked`, `joints` (26 positions in OpenXR order, see `HAND_JOINTS`), `pinch` 0 to 1, `ray_origin` and `ray_dir` (pointing ray from the index finger), `source`. When `tracked` is false, `joints` can be empty.

**`ObjectArray`**: `objects` (list of `ObjectState`), `source`.

**`TaskState`**: `mode` (`auto`, `teleop`, `paused`), `active_tag_id`, `objects` = list of `ObjectTask` with `tag_id`, `owner` (`robot` or `human`), `target_bin`, `status` (`pending`, `active`, `done`, `failed`), `reason`.

**`OperatorCommand`**: `kind` = `set_mode` (uses `mode`), `select_object` (pointing gesture: `tag_id`, optional `bin_tag_id`), `cancel`.

**`TeleopTarget`**: `target` (`ToolTarget`), `gripper`, `active`. The task node turns it into `stream` goals while in teleop mode.

## Ground truth topics (sim only)

`sim/*` is the simulated reality. In VR mode the Quest draws the virtual world from it in place of passthrough. Nothing else may use it: algorithms and the AR layer only see what they would also see on real hardware (`objects`, `arm/state`, ...). `Subscriber` raises `GroundTruthAccessError` for `sim/*` topics unless `allow_ground_truth=True`, which only `cobot-echo` and test scripts pass.

Whether `objects` itself comes from ground truth or from AprilTag detection is set in `config/perception.yaml`. Start with `ground_truth` while setting up AR, so a misplaced overlay is never a perception problem. The renderer and the AR layer don't care which one it is.

**`SceneInfo`**: `version` (changes when the scene changes, e.g. a new seed), `asset_version` (hash of the meshes, must match `Assets/Resources/SimScene/asset_version.txt` in the Quest build), `frame` (`base`), `textures` (name → base64 PNG), `bodies`:

```
{"name": "cube_10", "kind": "cube", "dynamic": true,
 "pose": [x, y, z, qw, qx, qy, qz],            only for static bodies
 "geoms": [{"name": ..., "type": "box|sphere|capsule|cylinder|ellipsoid|mesh",
            "size": [...], "pos": [...], "quat": [...], "rgba": [...],
            "mesh": "upper_arm_so101_v1",      only for type mesh
            "texture": "cube_10_tex"}]}        only for textured geoms
```

`kind` is `world` (table, plate), `arm`, `cube`, `bin`, `marker`, `hand_left`, `hand_right`. Geom `pos`/`quat` are relative to the body, `size` uses MuJoCo conventions (half sizes for box, radius and half length for cylinder and capsule along the geom z axis).

**`ScenePoses`**: `version` (same as `SceneInfo.version`, rebuild when it changes), `names`, `poses` (one `[x, y, z, qw, qx, qy, qz]` per name, base frame), `sim_time`. Only dynamic bodies are listed.

**Assets**: `python -m sim.scene_export` writes the meshes as OBJ (decimated to about 4000 faces each) into `quest_app/QuestTeleop/Assets/Resources/SimScene/Meshes/`. Vertices are in the geom frame with base axes. Rerun it when the model changes and rebuild the Quest app.

**Conversion to Unity** (the one place it happens, in the WorldRenderer):

- Position: `unity = (-y, z, x)`
- Quaternion (w, x, y, z) → Unity `Quaternion(y, -z, -x, w)`
- Box size (hx, hy, hz) → `localScale = (2hy, 2hz, 2hx)` on a Unity cube. Cylinder and capsule axis is z in MuJoCo and y in Unity, so `localScale = (2r, h, 2r)` with h the half length.
- Meshes: Unity's OBJ importer mirrors x, so the mesh child gets the fixed `localRotation = Quaternion(-0.5, 0.5, 0.5, 0.5)`. If the arm looks mirrored in the headset, this is the first thing to check.
- Textures sit on the geom +z face (Unity +y), image up = geom +y, image right = geom +x.

## Safety rules

These are the rules the verification part can state formally.

1. Every motion command passes through the supervisor. Controls only listens to `arm/command`, never to `arm/goal`.
2. Controls multiplies its speed by the latest `safety.speed_scale` and stops when `safety.stop` is true.
3. Watchdog: if controls gets no `safety` message for 0.2 s, it stops (fail-safe).
4. In `stream` mode, if no new command arrives for 0.2 s, controls holds position.
5. `stop` goals preempt everything and are never delayed.

## Stubs for working alone

| You work on | Run | You get |
|---|---|---|
| Unity / Quest | `cobot-launch dev` | objects, arm joints, a scripted hand |
| Supervisor | `cobot-launch dev` | hand and arm state to compute distances |
| Controls | `cobot-launch dev` plus your node | objects to target. Publish `arm/command` yourself from a test script while there is no supervisor |
| Task | `cobot-launch dev` | objects; fake `arm/status` from a test script |

The `hardware.state_node:ArmStateStub` publishes joint angles only. It is a stand-in until controls publishes `arm/state` itself, so remove it from the profile once controls runs, or both will publish.
