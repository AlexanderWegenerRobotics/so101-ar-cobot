# task

Shares the sorting work between robot and human, runs the robot's part, and tells the headset who should do what.

```
mjpython launch.py dev_viewer     # sim + controller + task node + a sim human that does its part of the plan
mjpython launch.py vr             # with the Quest: the task node waits for Go on the wrist menu
```

| File | What |
|---|---|
| `allocator.py` | Who does which cube and in which order (exact DP over the joint schedule). |
| `node.py` | `TaskManager`: tracks cubes, replans on events, executes the robot's jobs, publishes `task/state`. |
| `objects.py` | Cube and bin registry (color, mass, too heavy) from the scene manifest. |
| `trials.py` | One row per trial in `logs/trials.csv`. |

## Allocation

Every open cube is a job: from where it lies to its bin. Robot or human can do it, with three exceptions that make it
a human-only job (shown in the headset as "robot can't: ..."):

- `too_heavy` (registry, payload limit in `robot.yaml`),
- `out_of_reach` (exact IK check with the controller's planner, top-down grasp with up to 25 deg tilt),
- `tilted` (cube not lying flat), `robot_failed` (3 failed attempts), or the operator locked it to the human.

**Time models.** Robot: tool path like the controller plans it (lift, arc around the pan axis, descend) at
0.12 m/s with min-jerk timing, plus pick and place overheads (calibrated against logged runs, within about 15 %).
Human: straight hand path at 0.45 m/s plus about 1.3 s for grasp and release.

**Human paths around the arm.** The human never crosses the robot base: every hand path (hand -> cube -> bin) that
would pass within `keep_out_radius` (14 cm) of the pan axis goes around that circle instead. The detour counts in
the human's time, the sim human walks it, and the headset draws the suggested path on the table.

**Interference.** For a robot job and a human job, `conflict` in [0, 1] is the expected slowdown the safety
supervisor would impose while both run: its speed ramp (1 at 25 cm, 0 at 10 cm) applied to the distance between
the human's detoured hand path and the robot's sweep (tool path cube -> bin plus points along the arm back to the
base), averaged along the hand path. Running both at the same time costs `w_interference (3) x conflict x overlap`
seconds.

**Waiting.** Besides picking a cube, the human may wait until the robot finishes its current cube (only offered when
every cube the human could take next conflicts with it). The headset then shows `you: NEXT in 6 s`.

**Solver.** Exact search over the joint schedule with memoization (DP). Whichever agent is free first picks its next
cube or stops. State = (cubes left, free agent, where each agent is, the other agent's current job and remaining
time rounded to 0.25 s, the human's remaining budget). Cost = makespan + interference + 0.05 x human time. Checked
against brute force on random 5-cube instances (identical). 8 cubes: about 13k states, 30 to 60 ms; replans after
the first are 1 to 10 ms.

**Trade-off (alpha).** The human is about 5x faster than the SO-101, so a weighted sum of "time until done" and
"human work" only flips between the two extremes. Instead the node solves for the fastest plan in which the human
does at most K cubes, with K = forced + round((1 - alpha) x doable). Solving all K gives the Pareto front (logged
when the run starts or alpha changes):

```
front: human does <= 2: 2 human / 6 robot cubes, done in 102 s, human 5.5 s
front: human does <= 5: 5 human / 3 robot cubes, done in  50 s, human 13.4 s
front: human does <= 8: 8 human / 0 robot cubes, done in  19 s, human 19.5 s
```

- `alpha = 0`: fastest plan (the robot only takes what it can do without slowing things down)
- `alpha = 1`: least human work (human only gets the human-only cubes)
- `alpha = 0.5`: the robot takes about half of what it can do

## Replanning

The plan is a suggestion. The node replans whenever reality changes: a cube lands in a bin, the human lifts a cube
(any cube, also one planned for the robot), the human puts a cube down outside its bin, a cube moves more than 2 cm,
the robot fails, an operator command, a mode or alpha change, a hand appears, a scene reset. The robot's current
job stays locked; if the human grabs the cube the robot is reaching for, the robot stops and replans.

## Trials and reset

`Reset` on the wrist (`reset_scene`) starts a new trial: the robot stops, drives home with a normal min-jerk move
(gripper opens there), then the sim puts the cubes back (`reset_world` to `sim.control`), then the node waits
paused: `trial N ready, press Go`. A trial runs from Go until all cubes are sorted. Each trial is one row in
`logs/trials.csv`:

`date, trial, completed, alpha, duration_s, planned_makespan_s, robot_cubes, human_cubes, human_deviations
(took another cube than suggested), operator_assignments, robot_failures, min_hand_arm_m, slow_s, stop_s, stops`

Safety numbers (minimum hand-arm distance, time in the slow and stop zones, number of stops) only count while the
robot is working on a cube. A reset in the middle writes the row with `completed = 0`.

## Robot execution

Step machine over `arm/goal`: `approach` (6 cm above, gripper open) -> `descend` -> `close` -> `check_grasp`
(gripper must stop on the cube, above 6) -> `carry` (lift and travel to a free spot in the bin as one goal, grasp
monitored every tick: gripper reading and cube distance from the tool) -> `release` -> `check_place` (cube inside
the bin). Bins have 4 drop spots (2x2, 2.5 cm off center); the robot picks the one farthest from cubes already
there. A failure opens the gripper, waits 1 s and replans; after 3 failures the cube goes to the human. A move that
fails as `blocked` (arm hit something, see controls) does not count as an attempt.

## task/state

`TaskState`: `mode`, `active_tag_id` (robot's cube), `human_next_tag_id`, `robot_step`, `message`, `plan`
(`makespan`, `human_time`, `robot_time`, `interference`, `alpha`, `human_cap`, `solve_ms`), and per cube
`ObjectTask`: `owner` (robot / human / empty when done), `order` (0 = next for that agent), `status` (`pending`,
`robot_active`, `human_active`, `done`), `reason` (why this owner: `planner`, `operator`, `too_heavy`,
`out_of_reach`, ...), `robot_ok` + `robot_reason`, `locked`, `est_start`, `est_end` (s from now).

## Operator commands

| `kind` | Fields | Effect |
|---|---|---|
| `start`, `pause` | | auto mode on / off (pause stops the arm, resume restarts the current step) |
| `set_mode` | `mode` | `auto`, `paused`, `teleop` (forwards `operator/teleop` as stream goals) |
| `assign` | `tag_id`, `owner` | lock a cube to `robot` or `human` (null clears), whole plan is recomputed |
| `select_object` | `tag_id` | robot does this cube next: locked to the robot and forced as its next pick in the DP (whenever the robot is free and this cube is open, it is the only job the robot may take), the rest of the plan is optimized around it. If the robot is only reaching for another cube it switches now; if it holds one it places that first. Only one cube can be "next" |
| `cancel` | | clear all operator locks |
| `set_alpha` | `value` | move along the trade-off, whole plan is recomputed |
| `replan` | | replan now |
| `reset_scene` | | next trial: arm home (smooth), then cubes reset (sim), then paused until Go |

Assigning a human-only cube to the robot is refused with the reason in `message`.

## Parameters (`config/launch.yaml`)

`autostart` (false for the headset profiles), `alpha` 0.5, `w_interference` 3.0, `speed` 0.12, `approach` 0.06,
`drop_z` 0.09, `bin_slot` 0.025, `max_failures` 3, `max_tilt_deg` 15, `gripper_empty` 6, `lost_distance` 0.05,
`moved_threshold` 0.02, `human_rest` [0.55, 0, 0.12] (where the human is assumed when no hand is tracked).

## Log

Each replan: `plan (<why>): robot [..] human [..] makespan .. human .. interference .. alpha .. cap .. [ms, states]`.
Each robot job: `robot starts tag N (estimate X s)` and `robot finished tag N in Y s (estimate X s)`, to check the
time model.
