import argparse

import cv2
import numpy as np

from interfaces.transforms import Pose

CV_FROM_MJ_CAMERA = np.diag([1.0, -1.0, -1.0])
TAG_CORNER_SIGNS = np.array([[-1, 1, 0], [1, 1, 0], [1, -1, 0], [-1, -1, 0]], dtype=float)


def make_detector(family):
    dictionary_id = getattr(cv2.aruco, f"DICT_APRILTAG_{family.removeprefix('tag')}")
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    return cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(dictionary_id), params)


def detect_tags(detector, gray):
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None:
        return {}
    return {int(tag_id): c.reshape(4, 2) for tag_id, c in zip(ids.ravel(), corners)}


def tag_pose_camera(corners, tag_size, K, dist=None):
    object_points = TAG_CORNER_SIGNS * tag_size / 2
    ok, rvec, tvec = cv2.solvePnP(object_points, corners, np.asarray(K, float), dist,
                                  flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    R, _ = cv2.Rodrigues(rvec)
    return Pose.from_rotation(tvec.ravel(), R)


def sim_check():
    from sim.scene_builder import build, home_data, render_camera

    _, model, man = build()
    w, h = man["camera"]["resolution"]
    k = man["camera"]["intrinsics"]
    K = [[k["fx"], 0, k["cx"]], [0, k["fy"], k["cy"]], [0, 0, 1]]
    gray = cv2.cvtColor(render_camera(model, man["camera"]["name"], w, h), cv2.COLOR_RGB2GRAY)
    detected = detect_tags(make_detector(man["tag_family"]), gray)

    data = home_data(model)
    cam = model.camera(man["camera"]["name"]).id
    R_cam = data.cam_xmat[cam].reshape(3, 3) @ CV_FROM_MJ_CAMERA
    tags = man["cubes"] + man["bins"]
    for tag in tags:
        if tag["tag_id"] not in detected:
            print(f"tag {tag['tag_id']:3d}  not detected")
            continue
        body = model.body(tag["name"]).id
        tag_world = data.xpos[body] + data.xmat[body].reshape(3, 3) @ tag["tag_offset"]
        true_pos = R_cam.T @ (tag_world - data.cam_xpos[cam])
        pose = tag_pose_camera(detected[tag["tag_id"]], tag["tag_size"], K)
        error_mm = (pose.pos - true_pos) * 1000
        print(f"tag {tag['tag_id']:3d}  error x {error_mm[0]:6.2f}  y {error_mm[1]:6.2f}  z {error_mm[2]:6.2f} mm")
    print(f"{len(detected)} of {len(tags)} tags detected")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", action="store_true")
    args = parser.parse_args()
    if args.sim:
        sim_check()


if __name__ == "__main__":
    main()
