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
