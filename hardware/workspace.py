import argparse
import base64
import hashlib
import json
import math
import time
from pathlib import Path

import cv2
import mujoco
import numpy as np

from interfaces.messages import Topic, WorkspaceMap
from interfaces.node import Node

ROOT = Path(__file__).resolve().parents[1]
MODEL_XML = ROOT / "sim" / "model" / "so101.xml"
CACHE = ROOT / "config" / "workspace.json"

TOOL_SITE = "gripperframe"
PAN_JOINT = "shoulder_pan"
SAMPLED_JOINTS = ["shoulder_lift", "elbow_flex", "wrist_flex"]
ARM_ROOT = "base"

ZONES = [
    {"name": "reach", "label": "arm can reach (low over the table)",
     "z": [0.0, 0.10], "max_tilt_deg": 180.0, "rgba": [255, 170, 40, 70], "edge": [255, 170, 40, 230]},
    {"name": "grasp", "label": "arm can grasp a cube from the top",
     "z": [0.005, 0.03], "max_tilt_deg": 20.0, "rgba": [60, 200, 90, 95], "edge": [60, 220, 100, 255]},
]

GRID = {"x": [-0.5, 0.6], "y": [-0.55, 0.55], "res": 0.002}
SAMPLES = 64
TABLE_Z = 0.0
MIN_LINK_Z = -0.004


def model_hash():
    h = hashlib.sha1(MODEL_XML.read_bytes())
    h.update(json.dumps([ZONES, GRID, SAMPLES]).encode())
    return h.hexdigest()[:10]


def arm_bodies(model):
    root = model.body(ARM_ROOT).id
    return [b for b in range(model.nbody) if model.body_rootid[b] == model.body_rootid[root]]


def self_collision(model, data, arm):
    for i in range(data.ncon):
        c = data.contact[i]
        if model.geom_bodyid[c.geom1] in arm and model.geom_bodyid[c.geom2] in arm and c.dist < -0.002:
            return True
    return False


def sample_tool_positions(samples=SAMPLES):
    model = mujoco.MjModel.from_xml_path(str(MODEL_XML))
    data = mujoco.MjData(model)
    site = model.site(TOOL_SITE).id
    arm = set(arm_bodies(model))
    arm_list = sorted(arm)
    qadr = [model.jnt_qposadr[model.joint(j).id] for j in SAMPLED_JOINTS]
    grids = [np.linspace(*model.jnt_range[model.joint(j).id], samples) for j in SAMPLED_JOINTS]
    pan = model.joint(PAN_JOINT)
    pan_range = model.jnt_range[pan.id]
    mujoco.mj_forward(model, data)
    pan_axis = data.xanchor[pan.id].copy()

    out = []
    zmin = min(z["z"][0] for z in ZONES)
    zmax = max(z["z"][1] for z in ZONES)
    for a in grids[0]:
        for b in grids[1]:
            for c in grids[2]:
                data.qpos[:] = 0
                data.qpos[qadr[0]], data.qpos[qadr[1]], data.qpos[qadr[2]] = a, b, c
                mujoco.mj_kinematics(model, data)
                p = data.site_xpos[site]
                if not (zmin <= p[2] - TABLE_Z <= zmax):
                    continue
                mujoco.mj_forward(model, data)
                if self_collision(model, data, arm) or np.min(data.xpos[arm_list[1:], 2]) < MIN_LINK_Z:
                    continue
                approach = data.site_xmat[site].reshape(3, 3)[:, 0]
                tilt = math.degrees(math.acos(np.clip(-approach[2], -1.0, 1.0)))
                out.append((p[0] - pan_axis[0], p[2] - TABLE_Z, tilt))
    return np.array(out), pan_axis, pan_range


def rasterize(samples, pan_axis, pan_range, zone):
    sel = samples[(samples[:, 1] >= zone["z"][0]) & (samples[:, 1] <= zone["z"][1]) & (samples[:, 2] <= zone["max_tilt_deg"])]
    res = GRID["res"]
    bins = set(np.round(sel[:, 0] / res).astype(int).tolist())
    xs = np.arange(GRID["x"][0], GRID["x"][1], res) + res / 2
    ys = np.arange(GRID["y"][0], GRID["y"][1], res) + res / 2
    X, Y = np.meshgrid(xs, ys)
    dx, dy = X - pan_axis[0], Y - pan_axis[1]
    rho = np.hypot(dx, dy)
    k = np.round(rho / res).astype(int)
    fwd = np.isin(k, list(bins)) & (np.abs(np.arctan2(dy, dx)) <= pan_range[1])
    back = np.isin(-k, list(bins)) & (np.abs(np.arctan2(-dy, -dx)) <= pan_range[1])
    mask = (fwd | back).astype(np.uint8)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    radii = sel[:, 0]
    stats = {"min_reach": float(np.min(np.abs(radii))) if len(radii) else None,
             "max_reach": float(np.max(np.abs(radii))) if len(radii) else None}
    return mask, stats


def render(masks):
    h, w = masks[0][0].shape
    img = np.zeros((h, w, 4), np.uint8)
    for mask, zone in masks:
        img[mask > 0] = zone["rgba"]
    for mask, zone in masks:
        contours, _ = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
        cv2.drawContours(img, contours, -1, tuple(int(v) for v in zone["edge"]), 2)
    return img


def compute():
    t0 = time.time()
    samples, pan_axis, pan_range = sample_tool_positions()
    masks, legend = [], []
    for zone in ZONES:
        mask, stats = rasterize(samples, pan_axis, pan_range, zone)
        masks.append((mask, zone))
        legend.append({"name": zone["name"], "label": zone["label"], "rgba": zone["edge"], **stats,
                       "area_m2": float(mask.sum() * GRID["res"] ** 2)})
    img = render(masks)
    png = cv2.imencode(".png", cv2.cvtColor(np.flipud(img), cv2.COLOR_RGBA2BGRA))[1].tobytes()
    return {
        "version": model_hash(),
        "frame": "base",
        "bounds": [GRID["x"][0], GRID["x"][1], GRID["y"][0], GRID["y"][1]],
        "z": TABLE_Z + 0.001,
        "pan_axis": [float(pan_axis[0]), float(pan_axis[1])],
        "pan_range_deg": [math.degrees(pan_range[0]), math.degrees(pan_range[1])],
        "zones": legend,
        "image": base64.b64encode(png).decode(),
        "compute_seconds": round(time.time() - t0, 1),
    }


def load_or_compute(force=False):
    if not force and CACHE.exists():
        data = json.loads(CACHE.read_text())
        if data.get("version") == model_hash():
            return data
    data = compute()
    CACHE.write_text(json.dumps(data))
    return data


class WorkspacePublisher(Node):
    name = "workspace"
    rate_hz = 1.0

    def setup(self):
        data = load_or_compute()
        self.msg = WorkspaceMap(version=data["version"], bounds=data["bounds"], z=data["z"],
                                zones=data["zones"], image=data["image"])
        reach = {z["name"]: (z["min_reach"], z["max_reach"]) for z in data["zones"]}
        self.log.info(f"workspace {data['version']}: " + ", ".join(
            f"{k} {v[0]:.2f}-{v[1]:.2f} m from the pan axis" for k, v in reach.items() if v[0] is not None))

    def step(self):
        self.msg.stamp = time.time()
        self.publish(Topic.ARM_WORKSPACE, self.msg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--png", metavar="FILE")
    args = parser.parse_args()
    data = load_or_compute(args.force)
    for z in data["zones"]:
        print(f"{z['name']:6s} {z['min_reach']:.3f}-{z['max_reach']:.3f} m from the pan axis, {z['area_m2']:.3f} m2")
    if args.png:
        Path(args.png).write_bytes(base64.b64decode(data["image"]))
        print(f"wrote {args.png}")


if __name__ == "__main__":
    main()
