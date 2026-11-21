"""Write the constants of the UPPAAL model from config/supervisor.yaml, so the model and the supervisor use the same numbers.

S comes from supervisor.node.safe_distance, i.e. from the code itself. Every value is rounded the way that makes the
supervisor's job harder (times and error up, thresholds and hysteresis down), so a model that passes means the config passes.

    python supervisor/uppaal/gen_model.py            rewrite the constants in safe_hands_v2.xml
    python supervisor/uppaal/gen_model.py --check    only compare, exit 1 if the model is out of date
    python supervisor/uppaal/gen_model.py --verify   rewrite, then run verifyta on all queries
"""
import argparse
import math
import re
import shutil
import subprocess
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from supervisor.node import CONFIG, safe_distance  # noqa: E402

MODEL = Path(__file__).resolve().parent / "safe_hands_v2.xml"
STEPS_PER_TICK = 3
VERIFYTA = [r"C:\Program Files\UPPAAL-5.0.0\app\bin\verifyta.exe", "verifyta"]
EPS = 1e-6


def up(x):
    return math.ceil(x - EPS)


def down(x):
    return math.floor(x + EPS)


def model_constants(cfg):
    dt = 1.0 / (cfg["rate_hz"] * STEPS_PER_TICK)
    unit = min(cfg["hand_speed"], cfg["arm_speed"]) * dt
    va = up(cfg["arm_speed"] * dt / unit)
    c = {
        "PERIOD": STEPS_PER_TICK,
        "LAT": up(cfg["latency"] / dt),
        "T_STOP": up(cfg["stop_time"] / dt),
        "VH": up(cfg["hand_speed"] * dt / unit),
        "VA": va,
        "C_ERR": up(cfg.get("tracking_error", cfg["margin"]) / unit),
        "S": down(safe_distance(cfg) / unit),
        "BUF": down(cfg["slow_buffer"] / unit),
        "HYST": down(cfg["hysteresis"] / unit),
    }
    c["BRAKE_MOVE"] = min(c["T_STOP"], up(cfg["arm_speed"] * cfg["stop_time"] / 2 / (va * unit)))
    if not c["PERIOD"] <= c["LAT"] < 2 * c["PERIOD"]:
        sys.exit(f"latency {cfg['latency']} s is {c['LAT']} steps; the model needs {c['PERIOD']} <= LAT < {2 * c['PERIOD']}")
    return c, dt, unit


def read_constants(text):
    return {name: int(v) for name, v in re.findall(r"const int (\w+)\s*=\s*(-?\d+)\s*;", text)}


def write_constants(text, c):
    for name, v in c.items():
        text, n = re.subn(rf"(const int {name}\s*=\s*)-?\d+(\s*;)", rf"\g<1>{v}\g<2>", text)
        if n != 1:
            sys.exit(f"{MODEL.name}: expected one `const int {name}`, found {n}")
    return text


def verify(path):
    exe = next((e for e in VERIFYTA if Path(e).exists() or shutil.which(e)), None)
    if exe is None:
        sys.exit("verifyta not found")
    out = subprocess.run([exe, "-q", str(path)], capture_output=True, text=True)
    results = re.findall(r"Formula is (NOT )?satisfied", out.stdout + out.stderr)
    formulas = re.findall(r"<formula>(.*?)</formula>", path.read_text())
    if not results:
        sys.exit(out.stdout + out.stderr)
    for f, r in zip(formulas, results):
        print(f"{'FAIL' if r else 'ok  '}  {f.replace('&lt;', '<').replace('&gt;', '>').replace('&amp;', '&')}")
    return all(not r for r in results)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--verify", action="store_true")
    args = ap.parse_args()

    cfg = yaml.safe_load(CONFIG.read_text())
    c, dt, unit = model_constants(cfg)
    print(f"step {dt * 1000:.2f} ms, unit {unit * 1000:.2f} mm, S = {safe_distance(cfg):.3f} m -> {c['S']} units")

    text = MODEL.read_text()
    have = read_constants(text)
    diff = {k: (have.get(k), v) for k, v in c.items() if have.get(k) != v}
    for k, (old, new) in diff.items():
        print(f"  {k}: model {old}, config says {new}")

    if args.check:
        print("model matches config" if not diff else "model is out of date, run gen_model.py")
        sys.exit(1 if diff else 0)
    if diff:
        MODEL.write_text(write_constants(text, c), newline="\n")
        print(f"updated {MODEL.name}")
    else:
        print("model already matches config")
    if args.verify:
        sys.exit(0 if verify(MODEL) else 1)


if __name__ == "__main__":
    main()
