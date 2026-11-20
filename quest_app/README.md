# quest_app

Unity 6.6 project for the Meta Quest 3S (`CobotQuest/`), Meta XR SDK only (`com.meta.xr.sdk.all` 207.0.1 on OpenXR 1.19), no XR Interaction Toolkit. Talks to the Python side through the Quest bridge (`interfaces/quest_bridge.py`), a TCP connection with one JSON message per line.

## VR world (sim)

The Quest draws the MuJoCo scene in the robot base frame and follows the sim live. Physics only runs in MuJoCo: your tracked hands go to the sim hand, the sim pushes the cubes, and the new poses come back.

| Script | What it does |
|---|---|
| `CobotBridge` | TCP client to the Mac, reconnects on its own. `Subscribe(topic, handler)` and `Publish(topic, msg)`, handlers run on the main thread |
| `BaseFrame` | The only place where robot base coordinates (right-handed, z up) become Unity coordinates (left-handed, y up) and back |
| `WorldAnchor` | The robot base frame in the Unity scene. Everything from the sim is placed under it. For now it is put in front of you on start and with the A button (R in the editor); later the QR calibration sets it |
| `WorldRenderer` | Builds the scene from `sim/scene_info` (meshes, boxes, tag textures) and moves the bodies with `sim/scene` |
| `HandStreamer` | Sends both tracked hands as `hand` (26 OpenXR joints from `OVRSkeleton`, pinch, pointing ray), in the base frame |
| `WristMenu` | Buttons 7.5 cm above the back of your left hand, tilted halfway toward your face: turn the back of your hand toward you to show them. They move with the hand, not with your head, and freeze in place as soon as your right index finger comes within 16 cm, so covering the left hand while pressing does not move them. A button triggers when your fingertip touches its surface. Buttons in the `settings` group sit in a second row that the `Settings` button opens |
| `CobotWristActions` | What the buttons do. Main row: `Table` (place the table in front of you), `Reset` (next trial: arm home, cubes back, wait for Go), `Go/Stop` (label shows the state), `Settings`. Settings popup (closes after 6 s or after a press): `Reach` (workspace overlay on/off), `Tasks` (task overlay on/off), `Fast` / `Even` / `Robot max` (robot/human split, the selected one is bright) |
| `ButtonIcons` | Draws the round button images (colored disc, white ring, white icon) |
| `HandJoints` | Small helper to read joint positions from Meta's hand tracking |
| `WorkspaceOverlay` | Draws `arm/workspace` flat on the table: green = the arm can grasp there, orange = it can only reach there |
| `TaskOverlay` + `TaskTag` | Draws the task plan from `task/state`: a ring on the table around every open cube, a card above it (owner, order, hint), the suggested path for your next cube, and a summary line above the robot. Hold a card to assign |
| `StatusLabel` | Small text in front of the camera while not connected or not live |

### Setup

1. Unity loads the Newtonsoft JSON package from `Packages/manifest.json` on its own.
2. The scene needs the Camera Rig, Passthrough and Hand Tracking building blocks (Meta > Tools > Building Blocks).
3. Menu **Cobot > Add VR World To Scene**. It adds the objects, wires them and turns on internet permission for Android.
4. Select `Cobot` and set **Host** in `CobotBridge` to the Mac's IP address (System Settings > Wi-Fi > Details). Mac and Quest must be on the same network, and the macOS firewall must allow incoming connections for Python.
5. On the Mac: `python -m sim.scene_export` (only after model changes) and `cobot-launch vr`.
6. Build and Run.

The table appears in front of you at about desk height. Press A to move it in front of you again. Your hands push the cubes in MuJoCo. Turn on `Show Sim Hands` on `WorldRenderer` to see the spheres MuJoCo uses for your hands.

### Wrist menu

Raise your left hand and look at it, the buttons appear next to the wrist. Touch one with the tip of your right index finger. A button fires once per touch, then you have to pull the finger back.

To add a button: add an entry to `Buttons` on the `WristMenu` component (id, label, icon, color). Icons are drawn in `ButtonIcons.cs` (`table`, `reset`, `zones`, `run`, `mix`, `tasks`); leave `icon` empty and the label is written inside the circle instead. Then either wire its `On Press` event in the inspector, or handle its id in `CobotWristActions.OnPressed`. From code, `WristMenu.AddButton(id, label, color)` works too.

### Task overlay

Every open cube has a ring on the table and a card floating above it, turned toward you.

| Card | Meaning |
|---|---|
| blue `ROBOT · NOW` (pulsing) | the robot is working on it |
| blue `ROBOT · NEXT` | the robot does this one next (you chose it) |
| blue `ROBOT · 2` | planned for the robot, in this order |
| orange `YOU · NEXT` (pulsing) | what you should do now; `YOU · NEXT IN 6s` = wait for the robot first |
| orange `YOU · 2` | planned for you; the robot could do it, the plan just prefers you |
| red `YOU · 1`, `robot can't: too heavy` | only you can do it (too heavy, out of reach, tilted, robot failed) |
| `· set by you` | you assigned it |

The second line on each card says what holding it does. Touch a card with your index fingertip and hold: a bar
fills along the bottom, and when it is full (about half a second) the assignment is sent:

- a cube the robot can do: the robot does it next (`ROBOT · NEXT`), the rest of the plan is recomputed around it.
  If the robot is only reaching for another cube it switches right away; if it holds one it places that first.
- the `ROBOT · NEXT` cube: you take it yourself.
- a red cube: the card flashes `ROBOT CAN'T` and the reason.

An orange line on the table shows the suggested path to your next cube and on to its bin, always around the robot
base. The line above the robot shows the trial, how many cubes each of you has, the estimated time until done, the
mix and the last event. If you take a different cube than suggested, the plan adapts.

The task node starts paused in the `quest` and `vr` profiles: press `Go` on the wrist to start. When the robot has
no cube left in the plan it drives home and waits there.

### Meshes

`python -m sim.scene_export` writes the arm and plate meshes to `Assets/Resources/SimScene/Meshes/*.bytes` (decimated, about 45k triangles in total) plus `asset_version.txt`. If the sim's meshes don't match the build, the status label says so. Boxes, spheres, colors and textures come over the bus, so a new seed needs no rebuild.

If the arm looks inside out (dark, you see the back faces), toggle `Flip Mesh Winding` on `WorldRenderer`.

## Rules

- Only `WorldRenderer` subscribes to `sim/*`. Everything else (AR overlays, pointing, UI) uses `objects`, `arm/state`, `safety` and so on, which also exist on the real robot.
- Every pose sent to Python goes through `WorldAnchor.ToBase`.
