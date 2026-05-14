#!/usr/bin/env python3
"""Generate extruded text PLY meshes for the motion-constraints demo's
per-arm scenario labels and row-level descriptions.

This is a minimal port of `text_to_ply` from
viam-labs/example-visualizations-python's `scripts/generate_assets.py`.
Output: `assets/text__<heightMM>mm__<safeText>.ply` for every label
listed in LABELS / ROW_LABELS below.

Dependencies: matplotlib, shapely, trimesh, mapbox_earcut. The
easiest way to run is via the sibling Python module's venv:

    ~/viam/example-visualizations-python/.venv/bin/python \\
        scripts/generate_text_assets.py

Re-run after editing LABELS or ROW_LABELS. The PLY files are
committed alongside the source so customers don't need this tooling
to use the module.
"""

import math
import re
import sys
from pathlib import Path

import numpy as np
import trimesh
from matplotlib.font_manager import FontProperties
from matplotlib.textpath import TextPath
from shapely.geometry import Polygon

MM_PER_M = 1000.0

# Per-arm scenario labels (one per arm in the demo) plus row-level
# headings. The Go loader looks up assets by the same safe-filename
# rule used here: non-[A-Za-z0-9_-] characters become underscores.
ITEM_LABEL_HEIGHT_MM = 25.0
ROW_LABEL_HEIGHT_MM = 70.0
LABEL_DEPTH_MM = 2.0

LABELS = [
    "translation",
    "rotation",
    "translation + gripper",
    "rotation + gripper",
]

ROW_LABELS = [
    "End-Effector Control Frame",
]


def label_asset_filename(text: str, height_mm: float) -> str:
    safe = re.sub(r"[^A-Za-z0-9_-]", "_", text)
    return f"text__{int(round(height_mm))}mm__{safe}.ply"


def text_to_ply(text: str, height_mm: float, depth_mm: float = LABEL_DEPTH_MM) -> bytes:
    """Convert ``text`` to an extruded PLY mesh.

    Returns ASCII-PLY bytes in METERS so the RDK reader scales by 1000
    to mm correctly. Mesh is oriented with text upright (text-Y == world-Z)
    and front face at +Y so the default Viam viewer camera reads it
    straight on.
    """
    tp = TextPath((0, 0), text, size=100.0, prop=FontProperties(family="DejaVu Sans"))
    polys = tp.to_polygons()
    outers, holes = [], []
    for p in polys:
        if len(p) < 3:
            continue
        arr = np.asarray(p)
        sa = 0.5 * np.sum(arr[:-1, 0] * arr[1:, 1] - arr[1:, 0] * arr[:-1, 1])
        (outers if sa < 0 else holes).append(arr)

    if not outers:
        raise ValueError(f"text {text!r} produced no outer contours")

    shapes = []
    outer_polys = [Polygon(o.tolist()) for o in outers]
    for i, op in enumerate(outer_polys):
        my_holes = []
        for h in holes:
            hp = Polygon(h.tolist())
            if op.contains(hp.representative_point()):
                my_holes.append(h.tolist())
        shapes.append(Polygon(outers[i].tolist(), holes=my_holes))

    meshes = [trimesh.creation.extrude_polygon(s, depth_mm) for s in shapes]
    mesh = trimesh.util.concatenate(meshes)

    y_span = mesh.bounds[1][1] - mesh.bounds[0][1]
    if y_span <= 0:
        raise ValueError(f"text {text!r} has zero-height bounding box")
    mesh.apply_scale((height_mm / y_span) / MM_PER_M)

    # Stand the text up (Y-axis world-vertical) with the front face
    # at +Y so the default camera reads it left-to-right.
    mesh.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0]))
    mesh.apply_transform(trimesh.transformations.rotation_matrix(math.pi, [0, 0, 1]))

    center = (mesh.bounds[0] + mesh.bounds[1]) / 2
    mesh.apply_translation([-center[0], 0, -center[2]])

    return mesh.export(file_type="ply", encoding="ascii")


def main() -> int:
    out_dir = Path(__file__).resolve().parent.parent / "assets"
    out_dir.mkdir(exist_ok=True)

    pairs = [(t, ITEM_LABEL_HEIGHT_MM) for t in LABELS] + [
        (t, ROW_LABEL_HEIGHT_MM) for t in ROW_LABELS
    ]
    for text, height in pairs:
        path = out_dir / label_asset_filename(text, height)
        data = text_to_ply(text, height)
        path.write_bytes(data)
        print(f"wrote {path.name} ({len(data)} bytes)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
