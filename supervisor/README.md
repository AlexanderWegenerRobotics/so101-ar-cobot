# supervisor

`supervisor.node:SafetySupervisor` sits between the task node and the controller: `arm/goal` -> `arm/command`, plus
`safety` at 30 Hz. Speed and separation monitoring between your hands and the arm.

- Arm: points along every moving link (shoulder out to the jaw), from `arm/state` joints and forward kinematics on
  the MuJoCo model. The fixed base is not counted.
- Hands: all 26 joints of every tracked hand (`hand`, Quest or sim), dropped after 0.3 s without an update.
- Distance: minimum point distance minus 3 cm link radius and 1 cm finger radius.

| Distance | Zone | Arm |
|---|---|---|
| > 25 cm | `clear` | full speed |
| 10 to 25 cm | `slow` | speed scale falls linearly from 1 to 0 |
| < 10 cm | `stop` | holds (goal kept); released above 13 cm |

The controller applies the scale by time-warping the motion along the same path, so a slowed arm never leaves its
planned path, and stops when no `safety` message arrives for 0.2 s. Zone changes are logged with the distance and
the time spent in the previous zone. The task planner uses the same ramp to predict interference (`task/README.md`).

Parameters: `slow_distance` 0.25, `stop_distance` 0.10, `hysteresis` 0.03, `link_radius` 0.03, `finger_radius`
0.01, `hand_timeout` 0.3, `enabled` true (false: only monitor and log, never slow the arm, for baseline trials).

`passthrough.py` is the old placeholder (no checks).
