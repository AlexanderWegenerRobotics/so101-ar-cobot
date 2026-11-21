"""One row per trial in logs/trials.csv, for comparing runs in the safety study.

A trial starts with the first Go after a reset (or after launch) and ends when all cubes are sorted. A reset in the
middle writes the row with `completed = 0`.
"""
import csv
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CSV = ROOT / "logs" / "trials.csv"
FIELDS = ["date", "trial", "completed", "layout", "alpha", "duration_s", "planned_makespan_s", "robot_cubes", "human_cubes",
          "human_deviations", "operator_assignments", "robot_failures", "min_hand_arm_m", "slow_s", "stop_s",
          "stops"]


class TrialLog:
    def __init__(self, log, alpha, path=CSV):
        self.log = log
        self.path = path
        self.trial = 1
        self.alpha = alpha
        self.t0 = None
        self.stats = None
        self.last_safety = None
        self.last_row = None
        self.layout = None
        self.ran = False

    def start(self, alpha, planned):
        if self.t0 is not None:
            return
        self.t0 = time.time()
        self.ran = True
        self.alpha = alpha
        self.stats = {"robot_cubes": 0, "human_cubes": 0, "human_deviations": 0, "operator_assignments": 0,
                      "robot_failures": 0, "min_hand_arm_m": None, "slow_s": 0.0, "stop_s": 0.0, "stops": 0,
                      "planned_makespan_s": round(planned, 1) if planned is not None else ""}
        self.last_safety = None
        self.log.info(f"trial {self.trial} started (alpha {alpha})")

    @property
    def running(self):
        return self.t0 is not None

    def count(self, key, n=1):
        if self.running:
            self.stats[key] += n

    def human_pick(self, tag, suggested):
        if self.running and suggested is not None and suggested != tag:
            self.stats["human_deviations"] += 1

    def done(self, tag, who):
        if self.running:
            self.stats["robot_cubes" if who == "robot" else "human_cubes"] += 1

    def safety(self, msg, robot_busy=True):
        """Slow and stop time count only while the robot is working on a cube."""
        if not self.running:
            return
        now = time.time()
        if self.last_safety is not None and robot_busy:
            dt = min(now - self.last_safety[1], 0.5)
            zone = self.last_safety[0].zone
            if zone == "slow":
                self.stats["slow_s"] += dt
            elif zone == "stop":
                self.stats["stop_s"] += dt
            if msg.zone == "stop" and zone != "stop":
                self.stats["stops"] += 1
        if msg.raw_distance is not None and robot_busy:
            m = self.stats["min_hand_arm_m"]
            self.stats["min_hand_arm_m"] = msg.raw_distance if m is None else min(m, msg.raw_distance)
        self.last_safety = (msg, now)

    def finish(self, completed=True):
        if not self.running:
            return
        row = {"date": time.strftime("%Y-%m-%d %H:%M:%S"), "trial": self.trial, "completed": int(completed),
               "layout": self.layout if self.layout is not None else "default",
               "alpha": self.alpha, "duration_s": round(time.time() - self.t0, 1), **self.stats}
        row["slow_s"], row["stop_s"] = round(row["slow_s"], 1), round(row["stop_s"], 1)
        self.t0 = None
        self.last_row = row
        self.log.info("trial " + ", ".join(f"{k} {v}" for k, v in row.items() if k != "date"))
        try:
            self.path.parent.mkdir(exist_ok=True)
            if self.path.exists():
                with open(self.path) as f:
                    header = f.readline().strip().split(",")
                if header != FIELDS:
                    self.path.rename(self.path.with_name(f"trials_{time.strftime('%Y%m%d_%H%M%S')}_old.csv"))
            new = not self.path.exists()
            with open(self.path, "a", newline="") as f:
                w = csv.DictWriter(f, fieldnames=FIELDS)
                if new:
                    w.writeheader()
                w.writerow(row)
        except OSError as e:
            self.log.warning(f"could not write {self.path}: {e}")

    def snapshot(self):
        """Live numbers for the headset: the running trial, or the last finished one."""
        if self.running:
            st = self.stats
            return {"running": True, "layout": self.layout, "elapsed": round(time.time() - self.t0, 1), "robot_cubes": st["robot_cubes"],
                    "human_cubes": st["human_cubes"], "stops": st["stops"], "stop_s": round(st["stop_s"], 1),
                    "slow_s": round(st["slow_s"], 1), "min_hand_arm_m": st["min_hand_arm_m"]}
        if self.last_row is not None:
            r = self.last_row
            return {"running": False, "layout": self.layout, "elapsed": r["duration_s"], "robot_cubes": r["robot_cubes"],
                    "human_cubes": r["human_cubes"], "stops": r["stops"], "stop_s": r["stop_s"],
                    "slow_s": r["slow_s"], "min_hand_arm_m": r["min_hand_arm_m"], "completed": r["completed"]}
        return {"running": False, "layout": self.layout, "elapsed": 0.0}

    def abort(self):
        self.finish(completed=False)

    def next_trial(self):
        """Only count a new trial if the previous one actually ran."""
        if self.ran:
            self.trial += 1
            self.ran = False
