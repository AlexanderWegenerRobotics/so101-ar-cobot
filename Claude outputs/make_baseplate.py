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
from shapely.affinity import translate
from shapely.geometry import Point, Polygon, box
from shapely.ops import unary_union

from sim.tags import black_edge, tag_face_image

ROOT = Path(__file__).resolve().parents[2]
BASE_STL = ROOT / "sim" / "model" / "assets" / "base_so101_v2.stl"
OUT = Path(__file__).resolve().parent / "out"
MARKERS_YAML = ROOT / "config" / "world_markers.yaml"

BASE_MESH_POS = [-0.006365, 0.0, -0.0024]
BASE_MESH_QUAT = [1, 1, 1, 1]

PLATE = {
    "thickness": 0.003,
    "x": [-0.20, 0.07],
    "y": [-0.17, 0.17],
    "front_chamfer": 0.025,
    "cutout_clearance": 0.0004,
    "corner_radius": 0.008,
}

MARKERS = [
    {"name": "W1", "type": "qr", "payload": "so101-W1", "pos": [-0.14, 0.11], "sticker": 0.080},
    {"name": "W2", "type": "qr", "payload": "so101-W2", "pos": [-0.14, -0.11], "sticker": 0.080},
    {"name": "W3", "type": "qr", "payload": "so101-W3", "pos": [-0.015, -0.115], "sticker": 0.080},
    {"name": "anchor", "type": "apriltag", "family": "tag36h11", "id": 0, "pos": [-0.01, 0.115], "sticker": 0.060},
]

QR_BORDER_MODULES = 2

SPAWN = {"radius": [0.14, 0.30], "angle_deg": [-65, 65], "cube_half_diag": 0.0215}


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
    return unary_union(polys).convex_hull, float(z0)


def plate_outline():
    x0, x1 = PLATE["x"]
    y0, y1 = PLATE["y"]
    c = PLATE["front_chamfer"]
    poly = Polygon([(x0, y0), (x1 - c, y0), (x1, y0 + c), (x1, y1 - c), (x1 - c, y1), (x0, y1)])
    poly = poly.difference(spawn_region().buffer(0.003))
    if poly.geom_type != "Polygon":
        poly = max(poly.geoms, key=lambda g: g.area)
    r = PLATE["corner_radius"]
    return poly.buffer(-r, join_style=1).buffer(r, join_style=1)


def spawn_region():
    r0, r1 = SPAWN["radius"]
    a0, a1 = np.radians(SPAWN["angle_deg"])
    arc = np.linspace(a0, a1, 80)
    outer = [(r1 * math.cos(a), r1 * math.sin(a)) for a in arc]
    inner = [(r0 * math.cos(a), r0 * math.sin(a)) for a in arc[::-1]]
    return Polygon(outer + inner).buffer(SPAWN["cube_half_diag"])


def qr_matrix(payload):
    qr = qrcode.QRCode(error_correction=qrcode.constants.ERROR_CORRECT_M, border=0)
    qr.add_data(payload)
    qr.make(fit=True)
    return np.array(qr.get_matrix(), dtype=bool)


def marker_geometry(mk):
    if mk["type"] == "qr":
        mat = qr_matrix(mk["payload"])
        n = mat.shape[0]
        module = mk["sticker"] / (n + 2 * QR_BORDER_MODULES)
        return {"code_size": module * n, "module": module, "modules": n}
    return {"code_size": black_edge(mk["family"], mk["sticker"])}


def check_layout(plate, cutout):
    issues = []
    stickers = []
    for mk in MARKERS:
        s = mk["sticker"] / 2
        sq = box(mk["pos"][0] - s, mk["pos"][1] - s, mk["pos"][0] + s, mk["pos"][1] + s)
        if not plate.buffer(-0.004).contains(sq):
            issues.append(f"{mk['name']} sticker is too close to the plate edge")
        if sq.intersects(cutout.buffer(0.005)):
            issues.append(f"{mk['name']} sticker overlaps the arm base")
        for other_name, other in stickers:
            if sq.intersects(other.buffer(0.005)):
                issues.append(f"{mk['name']} overlaps {other_name}")
        stickers.append((mk["name"], sq))
    overlap = plate.intersection(spawn_region()).area
    if overlap > 1e-6:
        issues.append(f"plate overlaps cube spawn area by {overlap * 1e4:.1f} cm2")
    pts = np.array([mk["pos"] for mk in MARKERS if mk["type"] == "qr"])
    if len(pts) >= 3:
        a, b, c = pts[:3]
        u, v = b - a, c - a
        area = abs(u[0] * v[1] - u[1] * v[0]) / 2
        if area < 0.005:
            issues.append(f"QR markers are close to collinear (triangle area {area * 1e4:.0f} cm2)")
    return issues


def write_dxf(plate, cutout, path):
    doc = ezdxf.new("R2010", setup=True)
    doc.units = ezdxf.units.MM
    doc.layers.add("CUT", color=1)
    doc.layers.add("ENGRAVE", color=5)
    msp = doc.modelspace()

    def poly_mm(geom, layer):
        pts = [(x * 1000, y * 1000) for x, y in geom.exterior.coords]
        msp.add_lwpolyline(pts, close=True, dxfattribs={"layer": layer})

    poly_mm(plate, "CUT")
    poly_mm(cutout, "CUT")
    for mk in MARKERS:
        cx, cy = mk["pos"]
        s = mk["sticker"] / 2
        poly_mm(box(cx - s, cy - s, cx + s, cy + s), "ENGRAVE")
        tick = 0.006
        msp.add_line(((cx - tick) * 1000, cy * 1000), ((cx + tick) * 1000, cy * 1000), dxfattribs={"layer": "ENGRAVE"})
        msp.add_line((cx * 1000, (cy - tick) * 1000), (cx * 1000, (cy + tick) * 1000), dxfattribs={"layer": "ENGRAVE"})
        msp.add_text(mk["name"], height=5, dxfattribs={"layer": "ENGRAVE"}).set_placement(
            ((cx - s) * 1000, (cy - s - 0.008) * 1000))
    ax, ay = PLATE["x"][0] + 0.02, 0.0
    msp.add_line((ax * 1000, ay * 1000), ((ax + 0.04) * 1000, ay * 1000), dxfattribs={"layer": "ENGRAVE"})
    msp.add_line(((ax + 0.04) * 1000, ay * 1000), ((ax + 0.032) * 1000, (ay + 0.005) * 1000), dxfattribs={"layer": "ENGRAVE"})
    msp.add_line(((ax + 0.04) * 1000, ay * 1000), ((ax + 0.032) * 1000, (ay - 0.005) * 1000), dxfattribs={"layer": "ENGRAVE"})
    msp.add_text("+x", height=5, dxfattribs={"layer": "ENGRAVE"}).set_placement(((ax + 0.045) * 1000, (ay - 0.0025) * 1000))
    doc.saveas(path)


def draw_marker(c, mk, x0, y0):
    s = mk["sticker"] * 1000 * mm
    c.setLineWidth(0.3)
    c.setStrokeGray(0.6)
    c.rect(x0, y0, s, s, stroke=1, fill=0)
    if mk["type"] == "qr":
        mat = qr_matrix(mk["payload"])
        n = mat.shape[0]
        mod = s / (n + 2 * QR_BORDER_MODULES)
        c.setFillGray(0)
        for r in range(n):
            for col in range(n):
                if mat[r, col]:
                    c.rect(x0 + (QR_BORDER_MODULES + col) * mod, y0 + s - (QR_BORDER_MODULES + r + 1) * mod,
                           mod, mod, stroke=0, fill=1)
        label = f"{mk['name']}  QR '{mk['payload']}'  sticker {mk['sticker'] * 1000:.0f} mm"
    else:
        img = tag_face_image(mk["family"], mk["id"], 1.0)[:, :, 0]
        n = img.shape[0]
        px = s / n
        c.setFillGray(0)
        for r in range(n):
            row = img[r] < 128
            col = 0
            while col < n:
                if row[col]:
                    start = col
                    while col < n and row[col]:
                        col += 1
                    c.rect(x0 + start * px, y0 + s - (r + 1) * px, (col - start) * px, px, stroke=0, fill=1)
                else:
                    col += 1
        label = f"{mk['name']}  {mk['family']} id {mk['id']}  sticker {mk['sticker'] * 1000:.0f} mm"
    c.setFillGray(0)
    c.setFont("Helvetica", 8)
    c.drawString(x0, y0 - 11, label)


def write_marker_pdf(path):
    c = canvas.Canvas(str(path), pagesize=letter)
    w, h = letter
    c.setFont("Helvetica-Bold", 11)
    c.drawString(20 * mm, h - 15 * mm, "so101-ar-cobot base plate markers - print at 100% / actual size, no scaling")
    c.setFont("Helvetica", 9)
    c.drawString(20 * mm, h - 21 * mm, "Check the 100 mm bar with a ruler before cutting. Cut on the grey outline, align with the engraved squares.")
    c.setLineWidth(1)
    c.line(20 * mm, h - 28 * mm, 120 * mm, h - 28 * mm)
    for t in range(0, 101, 10):
        c.line((20 + t) * mm, h - 28 * mm, (20 + t) * mm, h - (30 if t % 50 else 32) * mm)
    c.drawString(122 * mm, h - 29 * mm, "100 mm")
    slots = [(20, 150), (110, 150), (20, 50), (110, 50)]
    for mk, (sx, sy) in zip(MARKERS, slots):
        draw_marker(c, mk, sx * mm, sy * mm)
    c.showPage()
    c.save()


def write_preview(plate, cutout, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon as MplPoly, Rectangle, Wedge

    fig, ax = plt.subplots(figsize=(8, 8))
    r0, r1 = SPAWN["radius"]
    a0, a1 = SPAWN["angle_deg"]
    ax.add_patch(Wedge((0, 0), r1, a0, a1, width=r1 - r0, color="#f5d7a1", alpha=0.6, label="cube spawn area"))
    for y in (0.24, -0.24):
        ax.add_patch(Rectangle((-0.059, y - 0.059), 0.118, 0.118, fill=False, ec="#888", ls="--"))
    ax.add_patch(MplPoly(np.array(plate.exterior.coords), fc="#dde6f0", ec="k", lw=1.5, label="plate (cut)"))
    ax.add_patch(MplPoly(np.array(cutout.exterior.coords), fc="white", ec="r", lw=1.2, label="base cutout"))
    for mk in MARKERS:
        s = mk["sticker"]
        col = "#2a6" if mk["type"] == "qr" else "#a33"
        ax.add_patch(Rectangle((mk["pos"][0] - s / 2, mk["pos"][1] - s / 2), s, s, fc=col, alpha=0.35, ec=col))
        ax.plot(*mk["pos"], "k+")
        ax.text(mk["pos"][0], mk["pos"][1] + s / 2 + 0.006, f"{mk['name']} ({mk['pos'][0]:.3f}, {mk['pos'][1]:.3f})",
                ha="center", fontsize=8)
    ax.annotate("", (0.05, 0), (0, 0), arrowprops=dict(arrowstyle="->", color="r"))
    ax.annotate("", (0, 0.05), (0, 0), arrowprops=dict(arrowstyle="->", color="g"))
    ax.text(0.052, -0.006, "x", color="r")
    ax.text(-0.008, 0.052, "y", color="g")
    ax.set_xlim(-0.25, 0.35)
    ax.set_ylim(-0.32, 0.32)
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_xlabel("x [m] (world = robot base)")
    ax.set_ylabel("y [m]")
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=130)


def write_yaml(path):
    t = PLATE["thickness"]
    out = {
        "frame": "robot_base",
        "units": "m",
        "plate": {
            "thickness": t,
            "x": PLATE["x"],
            "y": PLATE["y"],
            "top_z": t,
            "note": "base sits in a through-cutout on the table, so the table top stays at z = 0 and the plate top is at z = thickness",
        },
        "markers": [],
    }
    for mk in MARKERS:
        g = marker_geometry(mk)
        entry = {"name": mk["name"], "type": mk["type"], "pos": [round(mk["pos"][0], 4), round(mk["pos"][1], 4), t],
                 "sticker_size": mk["sticker"], "code_size": round(g["code_size"], 5)}
        if mk["type"] == "qr":
            entry["payload"] = mk["payload"]
            entry["use"] = "quest"
        else:
            entry["family"] = mk["family"]
            entry["id"] = mk["id"]
            entry["use"] = "realsense"
        out["markers"].append(entry)
    with open(path, "w") as f:
        f.write("# generated by hardware/baseplate/make_baseplate.py, edit MARKERS there and rerun\n")
        yaml.safe_dump(out, f, sort_keys=False)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=OUT)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    footprint, _ = base_footprint()
    cutout = footprint.buffer(PLATE["cutout_clearance"], join_style=2)
    plate = plate_outline()

    issues = check_layout(plate, cutout)
    for i in issues:
        print("WARNING:", i)

    write_dxf(plate, cutout, args.out / "baseplate.dxf")
    write_marker_pdf(args.out / "markers.pdf")
    write_preview(plate, cutout, args.out / "baseplate_preview.png")
    write_yaml(MARKERS_YAML)

    b = footprint.bounds
    print(f"base footprint x [{b[0]:.4f}, {b[2]:.4f}] y [{b[1]:.4f}, {b[3]:.4f}] m")
    print(f"plate {(PLATE['x'][1] - PLATE['x'][0]) * 1000:.0f} x {(PLATE['y'][1] - PLATE['y'][0]) * 1000:.0f} mm")
    for mk in MARKERS:
        print(mk["name"], mk["type"], mk["pos"], {k: round(v, 4) if isinstance(v, float) else v
                                                   for k, v in marker_geometry(mk).items()})
    print("wrote", args.out, MARKERS_YAML)


if __name__ == "__main__":
    main()
