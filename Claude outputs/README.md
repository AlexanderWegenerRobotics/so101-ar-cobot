# Base plate

Flat plate around the SO-101 base that carries the world markers. The arm base drops into a cutout, so the plate defines where the markers are relative to the robot base frame.

![preview](out/baseplate_preview.png)

## Files

| File | What it is |
|---|---|
| `make_baseplate.py` | Generates everything below from the base mesh and the `MARKERS` list |
| `out/baseplate.dxf` | Laser cut file in mm. Layer `CUT` = outline and base cutout, layer `ENGRAVE` = marker squares, crosshairs, labels, +x arrow |
| `out/markers.pdf` | Markers at 1:1 for printing on Letter |
| `out/baseplate_preview.png` | Top view with spawn area and bins |
| `../../config/world_markers.yaml` | Marker positions in the robot base frame, read by the Quest app, perception and sim |

```
python -m hardware.baseplate.make_baseplate
```

Needs `trimesh ezdxf qrcode reportlab shapely matplotlib`.

## Layout

- World frame = robot base, x towards the workspace, z up.
- W1, W2, W3: QR codes for the Quest (MRUK), payloads `so101-W1..3`, 80 mm stickers. They form an L-shape of about 22 x 13 cm, not collinear.
- anchor: AprilTag tag36h11 id 0, 60 mm sticker, for the RealSense. It sits at x = -0.01, inside the camera view.
- Plate 3 mm thick, 270 x 340 mm. The front is cut back so it stays out of the cube spawn area.
- The cutout is the convex hull of the base footprint plus 0.4 mm clearance. The base still stands on the table, so the table top stays at z = 0 and the markers are at z = 3 mm.

## Building it

1. Laser cut `baseplate.dxf` from 3 mm plywood or acrylic. Cut `CUT`, engrave `ENGRAVE`.
2. Print `markers.pdf` at 100 % and check the 100 mm bar. Matte paper, cut on the grey line, glue into the engraved squares.
3. Put the plate on the table, drop the arm base into the cutout, clamp the arm as usual and fix the plate with double-sided tape or a clamp at the back edge.

If the plate or a marker moves, the calibration is wrong. Change positions only in `MARKERS` and rerun the script.
