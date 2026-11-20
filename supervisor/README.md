# supervisor

`supervisor.node:SafetySupervisor` sits between the task node and the controller: `arm/goal` -> `arm/command`, plus
`safety` at 30 Hz. Speed and separation monitoring between your hands and the arm. Settings are in
`config/supervisor.yaml`.

- Arm: points along every moving link (shoulder out to the jaw), from `arm/state` joints and forward kinematics on
  the MuJoCo model. The fixed base is not counted.
- Hands: all 26 joints of every hand (`hand`, Quest or sim) at its last tracked detection, and how long ago that was
  (`age`). Only a tracked message resets `age`; a hand that stops being tracked is never forgotten.
- Distance: minimum point distance minus 3 cm link radius and 1 cm finger radius, minus `hand_speed * age`. While
  tracking works `age` is about 0. During a dropout the hand could be anywhere in a sphere that grows at
  `hand_speed`, so the arm keeps going if the hand was far and stops quickly if it was close.

| Distance | Zone | Arm |
|---|---|---|
| > S + 15 cm | `clear` | full speed |
| S to S + 15 cm | `slow` | speed scale falls linearly from 1 to 0 |
| < S | `stop` | holds (goal kept); released above S + 3 cm |
| no hand seen yet | `clear` | full speed |
| `arm/state` older than 0.2 s | `stop` | holds |

The controller applies the scale by time-warping the motion along the same path, so a slowed arm never leaves its
planned path, and stops when no `safety` message arrives for 0.2 s. Zone changes are logged with the distance and
the time spent in the previous zone. `reason` says how far the hand is and, during a dropout, for how long it has
not been seen. `min_distance` is the worst-case distance above, not the raw one.

`enabled: false` only monitors and logs, never slows the arm (baseline trials). `passthrough.py` is the old
placeholder (no checks).

## Separation distance S

ISO/TS 15066 speed and separation monitoring:

```
S = v_h * (T_r + T_s) + v_r * T_r + v_r * T_s / 2 + C        T_r = latency + 1 / rate_hz
```

`v_h` hand speed, `v_r` arm speed, `T_r` reaction time, `T_s` stopping time, `C` margin. With the current config
S = 0.233 m; the node logs it at startup. `latency`, `arm_speed` and `stop_time` are estimates until we measure them
on the real arm. With the ISO 13855 hand speed of 2.0 m/s S is about 0.5 m.

The task planner predicts interference with its own copy of the ramp (`slow_distance` 0.25, `stop_distance` 0.10 in
`task/allocator.py`), which no longer matches S.

## UPPAAL

| File | What it is |
|---|---|
| `uppaal/safe_hands_v1.xml` | Fixed 0.2 s timeout on stale hand data. Shows a hand losing tracking while close can touch the arm before it stops |
| `uppaal/safe_hands_v2.xml` | The current rule, verifies the stop |
| `uppaal/gen_model.py` | Writes the model's constants from `config/supervisor.yaml` and runs verifyta |
| `uppaal/check_conformance.py` | Runs the real node and checks every decision against the model's rule |

The model is 1D and discrete (11.1 ms steps, 5.6 mm units). Every step the hand can move toward the arm, away or
stay, and the detection can be there or missing, so it covers every motion and every dropout or flicker pattern.
Rounding is always on the safe side, the arm always moves toward the hand, slow mode counts as full speed and every
detection may read `tracking_error` too far.

| Query | Result |
|---|---|
| `A[] not deadlock` | satisfied |
| `A[] (d == 0 imply mode == STP)`, hand touches arm only when the arm is fully stopped | satisfied |
| `E<> d == 0`, the hand can reach the arm | satisfied |
| `E<> (stopped_once and mode == MOV)`, the arm can resume after a stop | satisfied |
| `E<> (age >= 15 and mode == MOV and zone == CLEAR)`, no stop during a short dropout when the hand is far | satisfied |

With a 10 cm stop distance, or with `C` 0.05 (S = 0.208 m), the safety query fails: one detection that reads a bit
too far releases a stop and the arm stops too late. That's why `margin` is 0.075.

Keeping the model and the node in sync, run both after changing the yaml or `node.py`:

```
python supervisor/uppaal/gen_model.py --verify
python supervisor/uppaal/check_conformance.py
```

`gen_model.py` reads the yaml, takes S from `safe_distance` in `node.py`, converts everything into model units
(times and `tracking_error` rounded up, S, `slow_buffer` and `hysteresis` rounded down) and writes the constants into
`safe_hands_v2.xml`, then runs verifyta. `--check` only compares and exits 1 if the model is out of date. Don't edit
the model's constants by hand.

`check_conformance.py` drives the real `SafetySupervisor` with a fake clock and bus: 200 runs of 30 s with a hand
moving toward and away from the moving arm at up to `hand_speed`, tracking that drops out, flickers or goes silent,
and `arm/state` going stale now and then. Every tick it checks that zone, speed scale and stop follow the model's
thresholds and hysteresis, that the distance matches the model's update (only a tracked message resets `age`), that
it is never more than the true distance, and that goals are forwarded unchanged. Exit 1 on any mismatch. `--mutate`
breaks the age rule on purpose to show the check catches it. Takes about 3 minutes.
