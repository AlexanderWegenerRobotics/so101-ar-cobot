import argparse
import base64
import hashlib
import json
from pathlib import Path

import cv2
import mujoco
import numpy as np

from interfaces.transforms import Pose

ROOT = Path(__file__).resolve().parents[1]
UNITY_DIR = ROOT / "quest_app" / "QuestTeleop" / "Assets" / "Resources" / "SimScene"
VISUAL_GROUPS = (0, 1, 2)
SKIP_GEOMS = {"floor"}
GEOM_TYPES = {int(k): v for k, v in {
    mujoco.mjtGeom.mjGEOM_BOX: "box",
    mujoco.mjtGeom.mjGEOM_SPHERE: "sphere",
    mujoco.mjtGeom.mjGEOM_CAPSULE: "capsule",
    mujoco.mjtGeom.mjGEOM_CYLINDER: "cylinder",
    mujoco.mjtGeom.mjGEOM_ELLIPSOID: "ellipsoid",
    mujoco.mjtGeom.mjGEOM_PLANE: "plane",
    mujoco.mjtGeom.mjGEOM_MESH: "mesh",
}.items()}
TEX_RGB = int(mujoco.mjtTextureRole.mjTEXROLE_RGB)


def _r(v, n=5):
    return [round(float(x), n) for x in v]


def body_kind(model, body_id, manifest):
    name = model.body(body_id).name
    if body_id == 0:
        return "world"
    if name.startswith("hand_"):
        return f"hand_{name.split('_')[1]}"
    if name.startswith("marker_"):
        return "marker"
    if any(c["name"] == name for c in manifest.get("cubes", [])):
        return "cube"
    if any(b["name"] == name for b in manifest.get("bins", [])):
        return "bin"
    root = model.body_rootid[body_id]
    if model.body(root).name == "base":
        return "arm"
    return "other"


def is_dynamic(model, body_id):
    return bool(model.body_weldid[body_id] != 0 or model.body_mocapid[body_id] >= 0)


def visual_geoms(model):
    out = {}
    for g in range(model.ngeom):
        if model.geom_group[g] not in VISUAL_GROUPS or model.geom(g).name in SKIP_GEOMS:
            continue
        if int(model.geom_type[g]) not in GEOM_TYPES:
            continue
        out.setdefault(int(model.geom_bodyid[g]), []).append(g)
    return out


def geom_texture(model, g):
    mat = model.geom_matid[g]
    if mat < 0:
        return None
    tex = model.mat_texid[mat][TEX_RGB]
    if tex < 0 or int(model.tex_type[tex]) != int(mujoco.mjtTexture.mjTEXTURE_2D):
        return None
    return int(tex)


def geom_rgba(model, g):
    mat = model.geom_matid[g]
    return _r(model.mat_rgba[mat] if mat >= 0 else model.geom_rgba[g], 4)


def texture_name(model, tex):
    return model.texture(tex).name or f"tex_{tex}"


def scene_bodies(world):
    model = world.model
    geoms = visual_geoms(model)
    T = world.base_from_mujoco()
    bodies = []
    for body_id, gids in sorted(geoms.items()):
        dynamic = is_dynamic(model, body_id)
        entry = {
            "name": model.body(body_id).name or "world",
            "kind": body_kind(model, body_id, world.manifest),
            "dynamic": dynamic,
            "geoms": [],
        }
        if not dynamic:
            with world.lock:
                p = T * Pose(world.data.xpos[body_id].copy(), world.data.xquat[body_id].copy())
            entry["pose"] = _r(p.pos) + _r(p.quat)
        for g in gids:
            gtype = GEOM_TYPES[int(model.geom_type[g])]
            geom = {
                "name": model.geom(g).name or f"geom_{g}",
                "type": gtype,
                "size": _r(model.geom_size[g]),
                "pos": _r(model.geom_pos[g]),
                "quat": _r(model.geom_quat[g]),
                "rgba": geom_rgba(model, g),
            }
            if gtype == "mesh":
                geom["mesh"] = model.mesh(model.geom_dataid[g]).name
            tex = geom_texture(model, g)
            if tex is not None:
                geom["texture"] = texture_name(model, tex)
            entry["geoms"].append(geom)
        bodies.append(entry)
    return bodies


def scene_version(bodies):
    return hashlib.sha1(json.dumps(bodies, sort_keys=True).encode()).hexdigest()[:10]


def referenced_assets(bodies):
    meshes, textures = set(), set()
    for b in bodies:
        for g in b["geoms"]:
            if "mesh" in g:
                meshes.add(g["mesh"])
            if "texture" in g:
                textures.add(g["texture"])
    return sorted(meshes), sorted(textures)


def asset_version(model, bodies):
    h = hashlib.sha1()
    meshes, _ = referenced_assets(bodies)
    for name in meshes:
        v, f = mesh_arrays(model, model.mesh(name).id)
        h.update(name.encode())
        h.update(np.round(v, 6).tobytes())
        h.update(f.tobytes())
    return h.hexdigest()[:10]


def texture_png(model, tex):
    w, h, n = model.tex_width[tex], model.tex_height[tex], model.tex_nchannel[tex]
    adr = model.tex_adr[tex]
    img = model.tex_data[adr:adr + w * h * n].reshape(h, w, n)
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR if n == 3 else cv2.COLOR_RGBA2BGRA)
    return cv2.imencode(".png", img)[1].tobytes()


def scene_info(world):
    bodies = scene_bodies(world)
    _, textures = referenced_assets(bodies)
    pngs = {name: base64.b64encode(texture_png(world.model, world.model.texture(name).id)).decode()
            for name in textures}
    return {"version": scene_version(bodies), "asset_version": asset_version(world.model, bodies),
            "frame": "base", "bodies": bodies, "textures": pngs}


def dynamic_bodies(world, bodies):
    model = world.model
    return [(b["name"], model.body(b["name"]).id if b["name"] != "world" else 0) for b in bodies if b["dynamic"]]


def pose_array(world, ids):
    T = world.base_from_mujoco()
    with world.lock:
        xpos = world.data.xpos[ids].copy()
        xquat = world.data.xquat[ids].copy()
        t = float(world.data.time)
    out = []
    for p, q in zip(xpos, xquat):
        pb = T * Pose(p, q)
        out.append(_r(pb.pos) + _r(pb.quat))
    return out, t


def mesh_arrays(model, mesh_id):
    va, vn = model.mesh_vertadr[mesh_id], model.mesh_vertnum[mesh_id]
    fa, fn = model.mesh_faceadr[mesh_id], model.mesh_facenum[mesh_id]
    return model.mesh_vert[va:va + vn].copy(), model.mesh_face[fa:fa + fn].copy()


def cluster(verts, faces, cell):
    keys = np.floor(verts / cell).astype(np.int64)
    _, idx, inv = np.unique(keys, axis=0, return_index=True, return_inverse=True)
    inv = inv.ravel()
    counts = np.bincount(inv)
    new_verts = np.zeros((len(counts), 3))
    np.add.at(new_verts, inv, verts)
    new_verts /= counts[:, None]
    f = inv[faces]
    keep = (f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])
    f = np.unique(np.sort(f[keep], axis=1), axis=0, return_index=True)[1]
    f = inv[faces][keep][np.sort(f)]
    return new_verts, f


def simplify(verts, faces, max_faces):
    if max_faces is None or len(faces) <= max_faces:
        return verts, faces
    for cell in (0.0005, 0.00075, 0.001, 0.0015, 0.002, 0.003, 0.004):
        v, f = cluster(verts, faces, cell)
        if len(f) <= max_faces:
            return v, f
    return v, f


def write_obj(path, verts, faces):
    lines = [f"v {x:.6f} {y:.6f} {z:.6f}" for x, y, z in verts]
    lines += [f"f {a + 1} {b + 1} {c + 1}" for a, b, c in faces]
    path.write_text("\n".join(lines) + "\n")


def export(world, out_dir=UNITY_DIR, max_faces=4000):
    model = world.model
    out_dir = Path(out_dir)
    (out_dir / "Meshes").mkdir(parents=True, exist_ok=True)
    info = scene_info(world)
    meshes, _ = referenced_assets(info["bodies"])
    faces_total = 0
    for name in meshes:
        v, f = simplify(*mesh_arrays(model, model.mesh(name).id), max_faces)
        faces_total += len(f)
        write_obj(out_dir / "Meshes" / f"{name}.obj", v, f)
    (out_dir / "asset_version.txt").write_text(info["asset_version"] + "\n")
    return info, len(meshes), faces_total


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default=str(UNITY_DIR))
    parser.add_argument("--max-faces", type=int, default=4000)
    args = parser.parse_args()
    from sim.world import SimWorld
    world = SimWorld(realtime=False)
    info, n_mesh, faces = export(world, args.out, args.max_faces)
    dyn = sum(b["dynamic"] for b in info["bodies"])
    print(f"assets {info['asset_version']}: {n_mesh} meshes ({faces} faces) -> {args.out}")
    print(f"scene {info['version']}: {len(info['bodies'])} bodies ({dyn} dynamic), {len(info['textures'])} textures sent over the bus")


if __name__ == "__main__":
    main()
