import argparse
import math
from pathlib import Path

import ezdxf
import numpy as np
import qrcode
import trimesh
import yaml
from reportlab.lib.pagesizes import letter
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas
from shapely.geometry import Polygon, box
from shapely.ops import unary_union

ROOT = Path(__file__).resolve().parents[2]
BASE_STL = ROOT / "sim" / "model" / "assets" / "base_so101_v2.stl"
PLATE_STL = ROOT / "sim" / "model" / "assets" / "baseplate.stl"
OUT = Path(__file__).resolve().parent / "out"
MARKERS_YAML = ROOT / "config" / "world_markers.yaml"
SCENE_YAML = ROOT / "config" / "scene.yaml"

BASE_MESH_POS = [-0.006365, 0.0, -0.0024]
BASE_MESH_QUAT = [1, 1, 1, 1]

SIDE_STICKER = 0.060
FRONT_STICKER = 0.050
GAP = 0.006
FRONT_GAP = 0.022
EDGE = 0.004
QR_BORDER_MODULES = 2

PLATE = {
    "thickness": 0.004,
    "cutout_clearance": 0.0003,
    "collar_width": 0.003,
    "collar_height": 0.012,
    "rear_slot_half_width": 0.033,
    "corner_radius": 0.006,
    "recess_depth": 0.0006,
    "recess_clearance": 0.0003,
    "mark_size": 0.005,
}

BED = [0.256, 0.256]
CUBE_HALF_DIAG = 0.0215


def base_footprint():
    m = trimesh.load(BASE_STL)
    q = np.array(BASE_MESH_QUAT, float)
    tf = trimesh.transformations.quaternion_matrix(q / np.linalg.norm(q))
    tf[:3, 3] = BASE_MESH_POS
    m.apply_transform(tf)
    z0 = m.bounds[0, 2]
    polys = []
    for dz in np.linspace(0.0003, PLATE["thickness"] + 0.001, 8):
        sec = m.section(plane_origin=[0, 0, z0 + dz], plane_normal=[0, 0, 1])
        if sec is None:
            continue
        path2d, _ = sec.to_2D(to_2D=np.eye(4))
        for p in path2d.polygons_full:
            polys.append(Polygon(p.exterior))
    return unary_union(polys).convex_hull


def layout(footprint):
    bx0, by0, bx1, by1 = footprint.bounds
    side_y = by1 + GAP + SIDE_STICKER / 2
    side_x = (bx0 + bx1) / 2 - 0.01
    front_x = bx1 + FRONT_GAP + FRONT_STICKER / 2
    markers = [
        {"name": "W1", "payload": "so101-W1", "pos": [round(side_x, 4), round(side_y, 4)], "yaw_deg": 0,
         "sticker": SIDE_STICKER},
        {"name": "W2", "payload": "so101-W2", "pos": [round(side_x, 4), round(-side_y, 4)], "yaw_deg": 0,
         "sticker": SIDE_STICKER},
        {"name": "W3", "payload": "so101-W3", "pos": [round(front_x, 4), 0.0], "yaw_deg": 90,
         "sticker": FRONT_STICKER},
    ]
    x0 = bx0 - PLATE["cutout_clearance"] - PLATE["collar_width"]
    x1 = front_x + FRONT_STICKER / 2 + EDGE
    y1 = side_y + SIDE_STICKER / 2 + EDGE
    return markers, (x0, x1, -y1, y1)


def load_spawn():
    with open(SCENE_YAML) as f:
        return yaml.safe_load(f)["cubes"]["spawn"]


def spawn_region(spawn):
    r0, r1 = spawn["radius"]
    a0, a1 = np.radians(spawn["angle_deg"])
    arc = np.linspace(a0, a1, 80)
    outer = [(r1 * math.cos(a), r1 * math.sin(a)) for a in arc]
    inner = [(r0 * math.cos(a), r0 * math.sin(a)) for a in arc[::-1]]
    return Polygon(outer + inner).buffer(CUBE_HALF_DIAG)


def plate_outline(bounds, cutout, spawn):
    x0, x1, y0, y1 = bounds
    poly = box(x0, y0, x1, y1).difference(spawn_region(spawn).buffer(0.003))
    if poly.geom_type != "Polygon":
        poly = max(poly.geoms, key=lambda g: g.area)
    r = PLATE["corner_radius"]
    poly = poly.buffer(-r, join_style=1).buffer(r, join_style=1)
    body = poly.union(collar_ring(cutout)).difference(cutout).difference(rear_slot(cutout))
    if body.geom_type != "Polygon":
        body = max(body.geoms, key=lambda g: g.area)
    return body


def rear_slot(cutout):
    hw = PLATE["rear_slot_half_width"]
    return box(cutout.bounds[0] - 0.02, -hw, cutout.bounds[0] + 0.004, hw)


def collar_ring(cutout):
    ring = cutout.buffer(PLATE["collar_width"], join_style=1).difference(cutout)
    return ring.difference(rear_slot(cutout))


def qr_matrix(payload):
    qr = qrcode.QRCode(error_correction=qrcode.constants.ERROR_CORRECT_M, border=0)
    qr.add_data(payload)
    qr.make(fit=True)
    return np.array(qr.get_matrix(), dtype=bool)


def code_size(mk):
    n = qr_matrix(mk["payload"]).shape[0]
    return mk["sticker"] * n / (n + 2 * QR_BORDER_MODULES)


def axes(mk):
    yaw = math.radians(mk["yaw_deg"])
    up = np.array([math.cos(yaw), math.sin(yaw)])
    right = np.array([math.sin(yaw), -math.cos(yaw)])
    return up, right


def code_corners(mk):
    c = np.array(mk["pos"], float)
    h = code_size(mk) / 2
    up, right = axes(mk)
    return [(c + h * up - h * right).tolist(), (c + h * up + h * right).tolist(),
            (c - h * up + h * right).tolist(), (c - h * up - h * right).tolist()]


def marker_square(mk, extra=0.0):
    s = mk["sticker"] / 2 + extra
    return box(mk["pos"][0] - s, mk["pos"][1] - s, mk["pos"][0] + s, mk["pos"][1] + s)


def orientation_mark(mk):
    c = np.array(mk["pos"], float)
    s = mk["sticker"] / 2 + PLATE["recess_clearance"]
    m = PLATE["mark_size"]
    up, right = axes(mk)
    base = c + (s - 0.0005) * up
    return Polygon([tuple(base - m * right), tuple(c + (s + m) * up), tuple(base + m * right)])


def check_layout(plate, markers, spawn):
    issues = []
    for mk in markers:
        sq = marker_square(mk)
        if not plate.buffer(-0.002).contains(sq):
            issues.append(f"{mk['name']} sticker is not fully on the plate")
    for i, a in enumerate(markers):
        for b in markers[i + 1:]:
            if marker_square(a).intersects(marker_square(b).buffer(0.003)):
                issues.append(f"{a['name']} overlaps {b['name']}")
    overlap = plate.intersection(spawn_region(spawn)).area
    if overlap > 1e-6:
        issues.append(f"plate overlaps cube spawn area by {overlap * 1e4:.1f} cm2")
    w = plate.bounds[2] - plate.bounds[0]
    h = plate.bounds[3] - plate.bounds[1]
    if not ((w <= BED[0] and h <= BED[1]) or (w <= BED[1] and h <= BED[0])):
        issues.append(f"plate {w * 1000:.0f} x {h * 1000:.0f} mm does not fit the bed {BED}")
    return issues


def write_stl(plate, cutout, markers, path):
    t, d = PLATE["thickness"], PLATE["recess_depth"]
    solid = trimesh.creation.extrude_polygon(plate, t)
    ring = collar_ring(cutout).intersection(plate.buffer(1e-6))
    ring_parts = [ring] if ring.geom_type == "Polygon" else [g for g in ring.geoms if g.area > 1e-8]
    collar = trimesh.util.concatenate([trimesh.creation.extrude_polygon(g, PLATE["collar_height"]) for g in ring_parts])
    solid = solid.union(collar, engine="manifold")
    cuts = unary_union([marker_square(mk, PLATE["recess_clearance"]) for mk in markers] +
                       [orientation_mark(mk) for mk in markers]).intersection(plate)
    parts = [cuts] if cuts.geom_type == "Polygon" else [g for g in cuts.geoms if g.area > 0]
    pocket = trimesh.util.concatenate([trimesh.creation.extrude_polygon(p, d + 0.001) for p in parts])
    pocket.apply_translation([0, 0, t - d])
    solid = solid.difference(pocket, engine="manifold")
    solid.apply_scale(1000.0)
    path.parent.mkdir(parents=True, exist_ok=True)
    solid.export(path)
    return solid


def write_dxf(plate, markers, path):
    doc = ezdxf.new("R2010", setup=True)
    doc.units = ezdxf.units.MM
    doc.layers.add("CUT", color=1)
    doc.layers.add("ENGRAVE", color=5)
    msp = doc.modelspace()

    def poly_mm(geom, layer):
        msp.add_lwpolyline([(x * 1000, y * 1000) for x, y in geom.exterior.coords], close=True,
                           dxfattribs={"layer": layer})

    poly_mm(plate, "CUT")
    for mk in markers:
        poly_mm(marker_square(mk), "ENGRAVE")
        poly_mm(orientation_mark(mk), "ENGRAVE")
    doc.saveas(path)


def draw_marker(c, mk, x0, y0):
    s = mk["sticker"] * 1000 * mm
    c.setLineWidth(0.3)
    c.setStrokeGray(0.6)
    c.rect(x0, y0, s, s, stroke=1, fill=0)
    mat = qr_matrix(mk["payload"])
    n = mat.shape[0]
    mod = s / (n + 2 * QR_BORDER_MODULES)
    c.setFillGray(0)
    for r in range(n):
        for col in range(n):
            if mat[r, col]:
                c.rect(x0 + (QR_BORDER_MODULES + col) * mod, y0 + s - (QR_BORDER_MODULES + r + 1) * mod,
                       mod, mod, stroke=0, fill=1)
    c.setFont("Helvetica", 8)
    c.drawString(x0, y0 + s + 4, f"{mk['name']}  '{mk['payload']}'  top edge to the triangle")
    c.drawString(x0, y0 - 11, f"{mk['sticker'] * 1000:.0f} mm, cut on the grey line")


def write_marker_pdf(markers, path):
    c = canvas.Canvas(str(path), pagesize=letter)
    w, h = letter
    c.setFont("Helvetica-Bold", 11)
    c.drawString(20 * mm, h - 15 * mm, "so101-ar-cobot base plate markers - print at 100% / actual size")
    c.setFont("Helvetica", 9)
    c.drawString(20 * mm, h - 21 * mm, "Check the 100 mm bar. The top edge of each code points to the triangle next to its recess.")
    c.setLineWidth(1)
    c.line(20 * mm, h - 28 * mm, 120 * mm, h - 28 * mm)
    for t in range(0, 101, 10):
        c.line((20 + t) * mm, h - 28 * mm, (20 + t) * mm, h - (30 if t % 50 else 32) * mm)
    c.drawString(122 * mm, h - 29 * mm, "100 mm")
    for mk, (sx, sy) in zip(markers, [(20, 170), (110, 170), (20, 90)]):
        draw_marker(c, mk, sx * mm, sy * mm)
    c.showPage()
    c.save()


def write_preview(plate, cutout, markers, spawn, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon as MplPoly, Rectangle, Wedge

    fig, ax = plt.subplots(figsize=(8, 8))
    r0, r1 = spawn["radius"]
    a0, a1 = spawn["angle_deg"]
    ax.add_patch(Wedge((0, 0), r1, a0, a1, width=r1 - r0, color="#f5d7a1", alpha=0.6, label="cube spawn area"))
    for y in (0.24, -0.24):
        ax.add_patch(Rectangle((-0.059, y - 0.059), 0.118, 0.118, fill=False, ec="#888", ls="--"))
    ax.add_patch(MplPoly(np.array(plate.exterior.coords), fc="#dde6f0", ec="k", lw=1.5, label="plate"))
    ax.add_patch(MplPoly(np.array(cutout.exterior.coords), fill=False, ec="r", lw=1.2, label="arm base"))
    for mk in markers:
        s = mk["sticker"]
        ax.add_patch(Rectangle((mk["pos"][0] - s / 2, mk["pos"][1] - s / 2), s, s, fc="#2a6", alpha=0.35, ec="#2a6"))
        ax.add_patch(MplPoly(np.array(orientation_mark(mk).exterior.coords), fc="k"))
        ax.plot(*mk["pos"], "k+")
        ax.text(mk["pos"][0], mk["pos"][1] + s / 2 + 0.006, f"{mk['name']} ({mk['pos'][0]:.3f}, {mk['pos'][1]:.3f})",
                ha="center", fontsize=8)
    ax.annotate("", (0.05, 0), (0, 0), arrowprops=dict(arrowstyle="->", color="r"))
    ax.annotate("", (0, 0.05), (0, 0), arrowprops=dict(arrowstyle="->", color="g"))
    ax.text(0.052, -0.008, "x", color="r")
    ax.text(-0.008, 0.052, "y", color="g")
    ax.annotate("rear free for the clamp", (-0.022, 0), (-0.14, -0.05), arrowprops=dict(arrowstyle="->"), fontsize=8)
    ax.set_xlim(-0.2, 0.35)
    ax.set_ylim(-0.32, 0.32)
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_xlabel("x [m] (world = robot base)")
    ax.set_ylabel("y [m]")
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=130)


def write_yaml(plate, markers, path):
    t = PLATE["thickness"]
    z = round(t - PLATE["recess_depth"], 4)
    x0, y0, x1, y1 = plate.bounds
    out = {
        "frame": "robot_base",
        "units": "m",
        "plate": {
            "thickness": t,
            "recess_depth": PLATE["recess_depth"],
            "bounds": [round(x0, 4), round(x1, 4), round(y0, 4), round(y1, 4)],
            "mesh": str(PLATE_STL.relative_to(ROOT)),
            "note": "base stands on the table inside the open cutout, table top stays at z = 0",
        },
        "qr": {
            "border_modules": QR_BORDER_MODULES,
            "corner_order": "top_left, top_right, bottom_right, bottom_left of the code",
            "yaw": "direction the top edge of the code points to, from +x, counterclockwise",
        },
        "markers": [],
    }
    for mk in markers:
        out["markers"].append({
            "name": mk["name"], "type": "qr", "payload": mk["payload"], "use": ["quest", "realsense"],
            "pos": [mk["pos"][0], mk["pos"][1], z], "yaw_deg": mk["yaw_deg"], "sticker_size": mk["sticker"],
            "code_size": round(code_size(mk), 5),
            "corners": [[round(cx, 5), round(cy, 5), z] for cx, cy in code_corners(mk)],
        })
    with open(path, "w") as f:
        f.write("# generated by hardware/baseplate/make_baseplate.py, edit the layout there and rerun\n")
        yaml.safe_dump(out, f, sort_keys=False, default_flow_style=None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=OUT)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    spawn = load_spawn()
    footprint = base_footprint()
    cutout = footprint.buffer(PLATE["cutout_clearance"], join_style=2)
    markers, bounds = layout(footprint)
    plate = plate_outline(bounds, cutout, spawn)

    for issue in check_layout(plate, markers, spawn):
        print("WARNING:", issue)

    solid = write_stl(plate, cutout, markers, PLATE_STL)
    write_dxf(plate, markers, args.out / "baseplate.dxf")
    write_marker_pdf(markers, args.out / "markers.pdf")
    write_preview(plate, cutout, markers, spawn, args.out / "baseplate_preview.png")
    write_yaml(plate, markers, MARKERS_YAML)

    b = plate.bounds
    print(f"plate {(b[2] - b[0]) * 1000:.0f} x {(b[3] - b[1]) * 1000:.0f} mm, watertight {solid.is_watertight}")
    for mk in markers:
        print(mk["name"], mk["pos"], f"code {code_size(mk) * 1000:.1f} mm")
    print("wrote", PLATE_STL, MARKERS_YAML, args.out)


if __name__ == "__main__":
    main()
