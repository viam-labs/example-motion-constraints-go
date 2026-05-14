#!/usr/bin/env python3
"""Generate extruded text PLY meshes for the motion-constraints demo's
per-arm scenario labels.

Multi-line input: when a label contains '\\n', each line is rendered
as a separate text mesh and the meshes are stacked vertically (later
lines BELOW earlier lines) into a single PLY. Line spacing is 1.4x
the per-line height for readability.

This is a port of `text_to_ply` from
viam-labs/example-visualizations-python's `scripts/generate_assets.py`.
Output: `assets/text__<heightMM>mm__<safeText>.ply` for every label
listed in LABELS below.

Dependencies: matplotlib, shapely, trimesh, mapbox_earcut. Easiest to
run via the sibling Python module's venv:

    ~/viam/example-visualizations-python/.venv/bin/python \\
        scripts/generate_text_assets.py
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

ITEM_LABEL_HEIGHT_MM = 35.0
LABEL_DEPTH_MM = 2.0
LINE_SPACING = 1.4  # multiplier on per-line cap height for line gap

LABELS = [
    "Arm Only\nTranslation Only\nConstraint: None\nCollidables: Self Only",
    "Arm Only\nRotation Only\nConstraint: None\nCollidables: Self Only",
    "Arm + Gripper\nTranslation Only\nConstraint: None\nCollidables: Self + Tool",
    "Arm + Gripper\nRotation Only\nConstraint: None\nCollidables: Self + Tool",
    "Arm Only\nTranslation\nConstraint: Linear\nCollidables: Self Only",
    "Arm Only\nRotation\nConstraint: Linear\nCollidables: Self Only",
    "Arm + Gripper\nTranslation\nConstraint: Linear\nCollidables: Self + Tool",
    "Arm + Gripper\nRotation\nConstraint: Linear\nCollidables: Self + Tool",
]


def label_asset_filename(text: str, height_mm: float) -> str:
    safe = re.sub(r"[^A-Za-z0-9_-]", "_", text)
    return f"text__{int(round(height_mm))}mm__{safe}.ply"


def _line_to_mesh(line: str, height_mm: float, depth_mm: float):
    """Build a trimesh for a single line of text. Returns None for
    blank lines (so they leave vertical space without crashing the
    polygon assembly)."""
    line = line.rstrip()
    if not line:
        return None

    tp = TextPath((0, 0), line, size=100.0, prop=FontProperties(family="DejaVu Sans"))
    polys = tp.to_polygons()
    outers, holes = [], []
    for p in polys:
        if len(p) < 3:
            continue
        arr = np.asarray(p)
        sa = 0.5 * np.sum(arr[:-1, 0] * arr[1:, 1] - arr[1:, 0] * arr[:-1, 1])
        (outers if sa < 0 else holes).append(arr)

    if not outers:
        return None

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
        return None
    mesh.apply_scale((height_mm / y_span) / MM_PER_M)

    return mesh


def text_to_ply(text: str, height_mm: float, depth_mm: float = LABEL_DEPTH_MM) -> bytes:
    """Convert ``text`` (possibly multi-line with '\\n') to a PLY mesh.

    Vertices are in METERS so the RDK reader scales by 1000 to mm
    correctly. After all lines are stacked, the whole mesh is rotated
    "upright with front face at +Y" so the default Viam viewer camera
    reads it left-to-right.
    """
    lines = text.split("\n")
    line_meshes = []
    line_height_m = height_mm / MM_PER_M
    line_spacing_m = LINE_SPACING * line_height_m
    for i, line in enumerate(lines):
        m = _line_to_mesh(line, height_mm, depth_mm)
        if m is None:
            continue
        # Stack vertically: line 0 at y=0, line 1 below at -line_spacing, etc.
        m.apply_translation([0, -i * line_spacing_m, 0])
        line_meshes.append(m)

    if not line_meshes:
        raise ValueError(f"text {text!r} produced no renderable lines")

    mesh = trimesh.util.concatenate(line_meshes)

    # Center on (X, Y); the rotation below moves Y -> Z so the text
    # ends up standing upright. Re-center afterwards so the host
    # transform's pose is the LABEL CENTER not a corner.
    center = (mesh.bounds[0] + mesh.bounds[1]) / 2
    mesh.apply_translation([-center[0], -center[1], 0])

    # Stand the text up (text-Y -> world-Z) with front face at +Y so
    # the default Viam viewer camera reads it.
    mesh.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0]))
    mesh.apply_transform(trimesh.transformations.rotation_matrix(math.pi, [0, 0, 1]))

    center = (mesh.bounds[0] + mesh.bounds[1]) / 2
    mesh.apply_translation([-center[0], 0, -center[2]])

    return mesh.export(file_type="ply", encoding="ascii")


def main() -> int:
    out_dir = Path(__file__).resolve().parent.parent / "assets"
    out_dir.mkdir(exist_ok=True)

    for text in LABELS:
        path = out_dir / label_asset_filename(text, ITEM_LABEL_HEIGHT_MM)
        data = text_to_ply(text, ITEM_LABEL_HEIGHT_MM)
        path.write_bytes(data)
        print(f"wrote {path.name} ({len(data)} bytes) for: {text!r}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
