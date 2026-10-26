import argparse
import math
import time

from hardware import load_robot_config, make_robot


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--backend", choices=["sim", "real"])
    parser.add_argument("--amplitude", type=float, default=15.0)
    parser.add_argument("--seconds", type=float, default=6.0)
    parser.add_argument("--no-viewer", action="store_true")
    args = parser.parse_args()

    cfg = load_robot_config()
    cfg["sim"]["world"]["viewer"] = not args.no_viewer
    cfg["max_relative_target"] = cfg.get("max_relative_target") or 5.0
    robot = make_robot(args.backend, cfg)

    with robot:
        start = robot.get_observation()
        print({k: round(v, 1) for k, v in start.items()})
        t0 = time.perf_counter()
        while (t := time.perf_counter() - t0) < args.seconds:
            action = dict(start)
            action["shoulder_pan.pos"] = start["shoulder_pan.pos"] + args.amplitude * math.sin(2 * math.pi * t / 3)
            robot.send_action(action)
            time.sleep(1 / 30)
        for _ in range(60):
            robot.send_action(start)
            time.sleep(1 / 30)
        print({k: round(v, 1) for k, v in robot.get_observation().items()})


if __name__ == "__main__":
    main()
