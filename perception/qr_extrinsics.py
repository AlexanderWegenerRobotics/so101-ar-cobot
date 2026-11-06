import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml

from interfaces.transforms import Pose

ROOT = Path(__file__).resolve().parents[1]
MARKERS_YAML = ROOT / "config" / "world_markers.yaml"
FINDER_CORNERS = [0, 1, 3]
SUBPIX = ((2, 2), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.01))


def load_markers(path=MARKERS_YAML):
    with open(path) as f:
        data = yaml.safe_load(f)
    return {m["payload"]: m for m in data["markers"] if "realsense" in m["use"]}


def detect(gray, markers):
    ok, texts, corners, _ = cv2.QRCodeDetectorAruco().detectAndDecodeMulti(gray)
    found = {}
    if not ok:
        return found
    for text, pts in zip(texts, corners):
        if text not in markers:
            continue
        refined = cv2.cornerSubPix(gray, pts.reshape(-1, 1, 2).astype(np.float32), *SUBPIX).reshape(-1, 2)
        found[text] = refined
    return found


def camera_pose(gray, K, dist=None, markers=None, min_markers=3):
    markers = markers or load_markers()
    found = detect(gray, markers)
    if len(found) < min_markers:
        return None, sorted(found)
    obj = np.concatenate([np.array(markers[t]["corners"], float)[FINDER_CORNERS] for t in found])
    img = np.concatenate([c[FINDER_CORNERS] for c in found.values()])
    ok, rvec, tvec = cv2.solvePnP(obj, img, np.asarray(K, float), dist)
    if not ok:
        return None, sorted(found)
    R_cw, _ = cv2.Rodrigues(rvec)
    R_wc = R_cw.T
    pos = (-R_wc @ tvec).ravel()
    return Pose.from_rotation(pos, R_wc), sorted(found)


def sim_check():
    from sim.scene_builder import build, render_camera, home_data

    _, model, man = build()
    w, h = man["camera"]["resolution"]
    k = man["camera"]["intrinsics"]
    K = [[k["fx"], 0, k["cx"]], [0, k["fy"], k["cy"]], [0, 0, 1]]
    gray = cv2.cvtColor(render_camera(model, man["camera"]["name"], w, h), cv2.COLOR_RGB2GRAY)
    est, found = camera_pose(gray, K)
    data = home_data(model)
    cid = model.camera(man["camera"]["name"]).id
    true_pos = data.cam_xpos[cid]
    true_R = data.cam_xmat[cid].reshape(3, 3) @ np.diag([1, -1, -1])
    print("markers found:", found)
    if est is None:
        print("calibration failed")
        return
    T = est.matrix()
    dR = T[:3, :3].T @ true_R
    angle = np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1)))
    print(f"position error {np.linalg.norm(T[:3, 3] - true_pos) * 1000:.2f} mm, rotation error {angle:.3f} deg")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", action="store_true")
    args = ap.parse_args()
    if args.sim:
        sim_check()


if __name__ == "__main__":
    main()
