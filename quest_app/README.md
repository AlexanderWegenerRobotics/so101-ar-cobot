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
| `WristMenu` | One button above the back of your left hand (turn the back of your hand toward you): `Table` places the table in front of you. Stays with your hand, freezes while your right finger is near. Other buttons can still be added with `AddButton` |
| `CobotWristActions` | Wires the wrist `Table` button |
| `MissionControl` + `UiKit` | Floating panel next to the table: trial, run state, elapsed time, progress, time left and layout, cubes done by robot and you, who-does-what selector (Fast / Even / Robot max), live safety zone with hand distance, last event, and buttons Go/Stop, Reset trial, New layout. Grab the bar above it with a pinch to move it |
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

### Mission control

A panel stands next to the table on your left, at about eye height, turned toward you. It belongs to the table: if
you place the table somewhere else, the panel moves with it. To move it, pinch the grey bar above it and drag; it
turns to face you while you hold it and stays where you let go.

- top: run state (`RUNNING`, `PAUSED`, `RESETTING`, `DONE`, `OFFLINE`), trial number, elapsed time
- progress bar: cubes sorted, estimated time left, layout
- `ROBOT` / `YOU`: cubes each of you sorted in this trial, and how many are still planned for each
- `WHO DOES WHAT`: `FAST` (finish as fast as possible, you do most), `EVEN`, `ROBOT MAX` (you only do what the
  robot can't); the selected one is lit
- safety line: `CLEAR`, `SLOWING`, `STOPPED` with the live hand-to-arm distance and the stops in this trial
- last event from the task node
- `GO` / `STOP`; `RESET TRIAL`: arm home, the same cube layout again, then `GO` (repeatable trials);
  `NEW LAYOUT`: arm home, a new random layout (its number shows under the progress bar and in `trials.csv`)

Press a button by touching its surface with your index fingertip.

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
base. If you take a different cube than suggested, the plan adapts.

The task node starts paused in the `quest` and `vr` profiles: press `GO` on the mission control panel to start. When the robot has
no cube left in the plan it drives home and waits there.

### Meshes

`python -m sim.scene_export` writes the arm and plate meshes to `Assets/Resources/SimScene/Meshes/*.bytes` (decimated, about 45k triangles in total) plus `asset_version.txt`. If the sim's meshes don't match the build, the status label says so. Boxes, spheres, colors and textures come over the bus, so a new seed needs no rebuild.

If the arm looks inside out (dark, you see the back faces), toggle `Flip Mesh Winding` on `WorldRenderer`.

## Rules

- Only `WorldRenderer` subscribes to `sim/*`. Everything else (AR overlays, pointing, UI) uses `objects`, `arm/state`, `safety` and so on, which also exist on the real robot.
- Every pose sent to Python goes through `WorldAnchor.ToBase`.
