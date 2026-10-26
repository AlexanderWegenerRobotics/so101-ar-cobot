# sim

MuJoCo digital twin of the workspace: SO-101 arm on a table, two bins, AprilTag cubes and a RealSense D435i above the table.

## Files

| File | What it does |
|---|---|
| `model/` | SO-101 MJCF from MuJoCo Menagerie (`robotstudio_so101`), Apache 2.0 |
| `scene_builder.py` | Builds the scene from `config/scene.yaml` and writes the manifest |
| `tags.py` | Generates the AprilTag textures (tag36h11) |
| `world.py` | `SimWorld`: runs physics in real time in a background thread, rendering, ground truth poses |
| `generated/` | Output of the scene builder (gitignored) |

## Commands

```
python -m sim.scene_builder --view
python -m sim.scene_builder --seed 3 --render cam.png
mjpython -m sim.world
```

`--view` opens the scene (paused physics viewer). `--render` saves one frame of the workspace camera. `sim.world` runs the live world with physics, the same thing the robot and camera APIs use.

## Scene

- World frame = robot base, z up, table top at z = 0.
- Anchor tag ID 0, fixed on the table, used later to calibrate the Quest and the camera frames.
- Bins: blue (tag 1) and red (tag 2), fixed positions. The tag sits on a flat tab next to each bin so cubes inside don't cover it.
- Cubes: tag IDs from 10 upwards, tag on the top face, color on the sides and as a border around the tag. Random positions in the reachable area in front of the arm, repeatable through `seed`.
- Arm starts in the `home` keyframe (upright, out of the camera view), defined by `home_qpos` in `config/robot.yaml`.

## Cubes and heavy objects

`cubes` in `config/scene.yaml`:

- `count`: total number of cubes
- `color_ratio`: split between blue and red, turned into exact counts (8 at 50/50 gives 4 and 4)
- `types`: `standard` and `heavy`, each with `ratio`, `size` and `mass`

Heavy cubes look exactly like standard ones. Only their tag ID tells the robot that they are too heavy, so the operator cannot see it. In the sim they really weigh 1.5 kg, so the arm can't lift them in teleop either.

Around 16 cubes fit in the spawn area. If you ask for more, the builder stops with an error saying how many it placed.

## Manifest

Every build writes `generated/scene_manifest.json`. It lists for every tag ID the object name, type, color, size, mass and initial pose, plus the camera intrinsics. `task/objects.py` reads it to decide what the robot is allowed to pick. On the real setup this file gets written by hand once for the printed cubes.

## Camera

D435i color stream: 1920x1080, 30 fps, 42 degree vertical FOV (69 x 42), mounted 70 cm above the table looking straight down. A second camera `workspace_cam_depth` simulates the depth sensor (87 x 58 degree FOV, max 1280x720, about 1.5 cm offset from the color sensor). Depth and color are not aligned, same as on the real camera.

## Tag detection notes

In tests on rendered frames all tags are detected, with position errors around 1 to 2 mm. Two things matter for perception:

- Use corner refinement (`CORNER_REFINE_APRILTAG` in OpenCV). Without it the error is around 1 to 2 cm.
- Tilt estimates for the flat tags (anchor, bins) are noisy. Better to assume they lie flat on the table.
