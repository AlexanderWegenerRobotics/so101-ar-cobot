import argparse
import json
import math
from pathlib import Path

import mujoco
import numpy as np
import yaml

from sim.tags import black_edge, tag_face_image

ROOT = Path(__file__).resolve().parents[1]
MODEL_XML = ROOT / "sim" / "model" / "so101.xml"
DEFAULT_CONFIG = ROOT / "config" / "scene.yaml"
ROBOT_CONFIG = ROOT / "config" / "robot.yaml"
GENERATED = ROOT / "sim" / "generated"
STICKER_HALF_THICKNESS = 0.0002
HAND_PARK = [0.0, 0.0, -2.0]


def load_config(path=DEFAULT_CONFIG):
    with open(path) as f:
        return yaml.safe_load(f)


def yaw_quat(yaw):
    return [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]


def split_counts(total, ratios):
    names = list(ratios)
    weights = np.array([ratios[n] for n in names], dtype=float)
    weights /= weights.sum()
    raw = weights * total
    counts = np.floor(raw).astype(int)
    for i in np.argsort(-(raw - counts))[: total - counts.sum()]:
        counts[i] += 1
    return {n: int(c) for n, c in zip(names, counts)}


def camera_xyaxes(pos, look_at, up):
    forward = np.asarray(look_at, float) - np.asarray(pos, float)
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, up)
    right /= np.linalg.norm(right)
    cam_up = np.cross(right, forward)
    return np.concatenate([right, cam_up])


def camera_intrinsics(fovy_deg, width, height):
    f = 0.5 * height / math.tan(math.radians(fovy_deg) / 2)
    return {"fx": f, "fy": f, "cx": width / 2, "cy": height / 2}


class SceneBuilder:
    def __init__(self, cfg, robot_cfg):
        self.cfg = cfg
        self.robot_cfg = robot_cfg
        self.rng = np.random.default_rng(cfg.get("seed"))
        self.spec = mujoco.MjSpec.from_file(str(MODEL_XML))
        self.spec.modelname = "so101_workspace"
        self.world = self.spec.worldbody
        self.family = cfg["tags"]["family"]
        self.footprints = []
        self.cube_body_order = []
        self.manifest = {"tag_family": self.family, "frame": "robot_base"}

    def build(self):
        self._add_environment()
        self._add_camera()
        self._add_anchor()
        self._add_bins()
        self._add_cubes()
        self._add_hands()
        self._add_home_key()
        return self.spec

    def _add_hands(self):
        h = self.cfg.get("hands")
        if not h:
            return
        from interfaces.messages import HAND_JOINTS
        collide = 1 if h.get("collide", False) else 0
        self.manifest["hands"] = {"sides": h["sides"], "joints": HAND_JOINTS, "park": HAND_PARK}
        for side in h["sides"]:
            for joint in HAND_JOINTS:
                radius = h["palm_radius"] if joint == "palm" else h["tip_radius"] if joint.endswith("tip") else h["joint_radius"]
                body = self.world.add_body(name=f"hand_{side}_{joint}", pos=HAND_PARK, mocap=True)
                body.add_geom(name=f"hand_{side}_{joint}", type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[radius, 0, 0],
                              rgba=h["rgba"], contype=collide, conaffinity=collide, group=1)

    def _add_home_key(self):
        home = [float(v) for v in self.robot_cfg["home_qpos"]]
        qpos = list(home)
        for i in self.cube_body_order:
            cube = self.manifest["cubes"][i]
            qpos += cube["init_pos"] + yaw_quat(cube["init_yaw"])
        self.spec.add_key(name="home", qpos=qpos, ctrl=home)

    def _rgba(self, color):
        return self.cfg["colors"][color]

    def _add_tag_material(self, name, tag_id, margin_rgba=(1, 1, 1, 1), face_fraction=1.0):
        margin = tuple(int(255 * c) for c in margin_rgba[:3])
        img = tag_face_image(self.family, tag_id, face_fraction, margin)
        tex = self.spec.add_texture(name=f"{name}_tex", type=mujoco.mjtTexture.mjTEXTURE_2D,
                                    width=img.shape[1], height=img.shape[0], nchannel=3)
        tex.data = img.tobytes()
        self.spec.add_material(name=f"{name}_mat", textures=["", f"{name}_tex"])
        return f"{name}_mat"

    def _add_sticker(self, body, name, material, size, z):
        body.add_geom(name=f"{name}_tag", type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[size / 2, size / 2, STICKER_HALF_THICKNESS], pos=[0, 0, z],
                      material=material, contype=0, conaffinity=0, mass=0, group=1)

    def _add_environment(self):
        t = self.cfg["table"]
        self.spec.visual.headlight.diffuse = [0.6, 0.6, 0.6]
        self.spec.visual.headlight.ambient = [0.3, 0.3, 0.3]
        self.spec.add_texture(name="skybox", type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
                              builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
                              rgb1=[0.3, 0.5, 0.7], rgb2=[0, 0, 0], width=512, height=3072)
        self.spec.add_texture(name="floor_tex", type=mujoco.mjtTexture.mjTEXTURE_2D,
                              builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER, mark=mujoco.mjtMark.mjMARK_EDGE,
                              rgb1=[0.2, 0.3, 0.4], rgb2=[0.1, 0.2, 0.3], markrgb=[0.8, 0.8, 0.8],
                              width=300, height=300)
        self.spec.add_material(name="floor_mat", textures=["", "floor_tex"], texuniform=True,
                               texrepeat=[5, 5], reflectance=0.2)
        self.spec.add_material(name="table_mat", rgba=[0.82, 0.78, 0.72, 1])
        self.world.add_light(pos=[0, 0, 3], dir=[0, 0, -1], type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL)
        self.world.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[0, 0, 0.05],
                            pos=[0, 0, -t["height"]], material="floor_mat")
        self.world.add_geom(name="table", type=mujoco.mjtGeom.mjGEOM_BOX,
                            size=[t["size"][0] / 2, t["size"][1] / 2, t["height"] / 2],
                            pos=[t["center"][0], t["center"][1], -t["height"] / 2], material="table_mat")

    def _add_camera(self):
        c = self.cfg["camera"]
        cam = self.world.add_camera(name=c["name"], pos=c["pos"], fovy=c["fovy"])
        cam.alt.type = mujoco.mjtOrientation.mjORIENTATION_XYAXES
        cam.alt.xyaxes = camera_xyaxes(c["pos"], c["look_at"], c["up"])
        w, h = c["resolution"]
        self.spec.visual.global_.offwidth = max(self.spec.visual.global_.offwidth, w)
        self.spec.visual.global_.offheight = max(self.spec.visual.global_.offheight, h)
        self.manifest["camera"] = {"name": c["name"], "model": c.get("model"), "fps": c.get("fps"), "resolution": [w, h], "fovy_deg": c["fovy"],
                                   "intrinsics": camera_intrinsics(c["fovy"], w, h)}
        d = c.get("depth")
        if d:
            offset = np.asarray(d.get("offset", [0, 0, 0]), float)
            pos = np.asarray(c["pos"], float) + offset
            look_at = np.asarray(c["look_at"], float) + offset
            dcam = self.world.add_camera(name=d["name"], pos=pos.tolist(), fovy=d["fovy"])
            dcam.alt.type = mujoco.mjtOrientation.mjORIENTATION_XYAXES
            dcam.alt.xyaxes = camera_xyaxes(pos, look_at, c["up"])
            dw, dh = d["resolution"]
            self.spec.visual.global_.offwidth = max(self.spec.visual.global_.offwidth, dw)
            self.spec.visual.global_.offheight = max(self.spec.visual.global_.offheight, dh)
            self.spec.visual.map.znear = min(self.spec.visual.map.znear, 0.01)
            self.manifest["camera"]["depth"] = {"name": d["name"], "resolution": [dw, dh], "fovy_deg": d["fovy"],
                                                "offset": offset.tolist(), "range": d["range"],
                                                "intrinsics": camera_intrinsics(d["fovy"], dw, dh)}

    def _add_anchor(self):
        a = self.cfg["tags"]["anchor"]
        body = self.world.add_body(name="anchor", pos=[a["pos"][0], a["pos"][1], 0])
        mat = self._add_tag_material("anchor", a["id"])
        self._add_sticker(body, "anchor", mat, a["size"], STICKER_HALF_THICKNESS)
        self.footprints.append((np.array(a["pos"]), a["size"] * 0.75))
        self.manifest["anchor"] = {"tag_id": a["id"], "pos": [a["pos"][0], a["pos"][1], 0.0],
                                   "tag_size": black_edge(self.family, a["size"])}

    def _add_bins(self):
        b = self.cfg["bins"]
        ix, iy = b["inner_size"]
        h, w = b["height"], b["wall"]
        tab = b["tag_size"] * 1.2
        self.manifest["bins"] = []
        for item in b["items"]:
            yaw = math.radians(item.get("yaw_deg", 0))
            body = self.world.add_body(name=item["name"], pos=[*item["pos"], 0], quat=yaw_quat(yaw))
            rgba = self._rgba(item["color"])
            ox, oy = ix / 2 + w, iy / 2 + w
            parts = [
                ("floor", [ox, oy, w / 2], [0, 0, w / 2]),
                ("wall_px", [w / 2, oy, h / 2], [ix / 2 + w / 2, 0, h / 2]),
                ("wall_nx", [w / 2, oy, h / 2], [-ix / 2 - w / 2, 0, h / 2]),
                ("wall_py", [ix / 2, w / 2, h / 2], [0, iy / 2 + w / 2, h / 2]),
                ("wall_ny", [ix / 2, w / 2, h / 2], [0, -iy / 2 - w / 2, h / 2]),
            ]
            for part, size, pos in parts:
                body.add_geom(name=f"{item['name']}_{part}", type=mujoco.mjtGeom.mjGEOM_BOX,
                              size=size, pos=pos, rgba=rgba)
            tab_x = ox + tab / 2
            tab_body = body.add_body(name=f"{item['name']}_tab", pos=[tab_x, 0, 0])
            tab_body.add_geom(name=f"{item['name']}_tab_plate", type=mujoco.mjtGeom.mjGEOM_BOX,
                              size=[tab / 2, tab / 2, 0.001], pos=[0, 0, 0.001], rgba=rgba)
            mat = self._add_tag_material(item["name"], item["tag_id"])
            self._add_sticker(tab_body, item["name"], mat, b["tag_size"], 0.002 + STICKER_HALF_THICKNESS)
            center = np.array(item["pos"], float)
            self.footprints.append((center, math.hypot(ox, oy)))
            tab_world = center + tab_x * np.array([math.cos(yaw), math.sin(yaw)])
            self.footprints.append((tab_world, tab * 0.75))
            self.manifest["bins"].append({
                "name": item["name"], "color": item["color"], "tag_id": item["tag_id"],
                "pos": [*item["pos"], 0.0], "yaw": yaw, "inner_size": [ix, iy], "height": h,
                "tag_size": black_edge(self.family, b["tag_size"]),
                "tag_offset": [tab_x, 0.0, 0.002 + 2 * STICKER_HALF_THICKNESS],
            })

    def _sample_position(self, radius, placed, total):
        s = self.cfg["cubes"]["spawn"]
        r_min, r_max = s["radius"]
        a_min, a_max = np.radians(s["angle_deg"])
        for _ in range(s["max_tries"]):
            r = math.sqrt(self.rng.uniform(r_min ** 2, r_max ** 2))
            a = self.rng.uniform(a_min, a_max)
            p = np.array([r * math.cos(a), r * math.sin(a)])
            if all(np.linalg.norm(p - q) >= radius + rq + s["min_gap"] for q, rq in self.footprints):
                return p
        raise RuntimeError(f"placed {placed} of {total} cubes, lower count, min_gap or widen spawn region")

    def _add_cubes(self):
        c = self.cfg["cubes"]
        type_counts = split_counts(c["count"], {k: v["ratio"] for k, v in c["types"].items()})
        color_counts = split_counts(c["count"], c["color_ratio"])
        types = [t for t, n in type_counts.items() for _ in range(n)]
        colors = [k for k, n in color_counts.items() for _ in range(n)]
        self.rng.shuffle(types)
        self.rng.shuffle(colors)
        order = sorted(range(len(types)), key=lambda i: -c["types"][types[i]]["size"])
        face_fraction = self.cfg["tags"]["face_fraction"]
        self.manifest["cubes"] = [None] * len(types)
        for i in order:
            ctype, color = types[i], colors[i]
            props = c["types"][ctype]
            size, mass = props["size"], props["mass"]
            tag_id = c["first_tag_id"] + i
            name = f"cube_{tag_id}"
            radius = size / 2 * math.sqrt(2)
            xy = self._sample_position(radius, len(self.cube_body_order), len(types))
            yaw = self.rng.uniform(-math.pi, math.pi) if c["spawn"]["random_yaw"] else 0.0
            self.footprints.append((xy, radius))
            self.cube_body_order.append(i)
            body = self.world.add_body(name=name, pos=[xy[0], xy[1], size / 2], quat=yaw_quat(yaw))
            body.add_freejoint(name=f"{name}_joint")
            rgba = self._rgba(color)
            body.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_BOX, size=[size / 2] * 3, mass=mass,
                          rgba=rgba, friction=[1, 0.005, 0.0001], solref=[0.01, 1], condim=4)
            mat = self._add_tag_material(name, tag_id, rgba, face_fraction)
            self._add_sticker(body, name, mat, size, size / 2 + STICKER_HALF_THICKNESS)
            self.manifest["cubes"][i] = {
                "name": name, "tag_id": tag_id, "type": ctype, "color": color, "size": size,
                "mass": mass, "tag_size": black_edge(self.family, size * face_fraction),
                "tag_offset": [0.0, 0.0, size / 2 + 2 * STICKER_HALF_THICKNESS],
                "init_pos": [float(xy[0]), float(xy[1]), size / 2], "init_yaw": float(yaw),
            }


def build(cfg=None, robot_cfg=None):
    cfg = cfg or load_config()
    robot_cfg = robot_cfg or load_config(ROBOT_CONFIG)
    builder = SceneBuilder(cfg, robot_cfg)
    spec = builder.build()
    return spec, spec.compile(), builder.manifest


def write_manifest(manifest, path=GENERATED / "scene_manifest.json"):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(manifest, indent=2))
    return path


def home_data(model):
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    mujoco.mj_forward(model, data)
    return data


def render_camera(model, cam_name, width, height, settle_steps=200):
    data = home_data(model)
    for _ in range(settle_steps):
        mujoco.mj_step(model, data)
    with mujoco.Renderer(model, height, width) as renderer:
        renderer.update_scene(data, camera=cam_name)
        return renderer.render()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--seed", type=int)
    parser.add_argument("--view", action="store_true")
    parser.add_argument("--render", metavar="PNG")
    args = parser.parse_args()

    cfg = load_config(args.config)
    if args.seed is not None:
        cfg["seed"] = args.seed
    _, model, manifest = build(cfg)
    print(f"manifest: {write_manifest(manifest)}")

    if args.render:
        import cv2
        w, h = manifest["camera"]["resolution"]
        img = render_camera(model, manifest["camera"]["name"], w, h)
        cv2.imwrite(args.render, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        print(f"render: {args.render}")
    if args.view:
        import mujoco.viewer
        mujoco.viewer.launch(model, home_data(model))


if __name__ == "__main__":
    main()
