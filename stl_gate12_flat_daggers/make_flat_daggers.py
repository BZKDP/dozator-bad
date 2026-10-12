# -*- coding: utf-8 -*-
"""Build a print-ready plate of 20 Gate12 cardboard daggers laid flat.

The original foot geometry is reused without modification.  Each copy is
rotated onto its long side for better bed adhesion and stronger layer
orientation.  A one-layer rectangular brim is fused only around the outside
edge; it can be cut away with scissors or flush cutters after printing.
"""

from pathlib import Path
import math

import numpy as np
import trimesh


HERE = Path(__file__).resolve().parent
SOURCE = HERE.parent / "stl_gate12" / "foot_x4.stl"
OUTPUT = HERE / "gate12_daggers_flat_20pcs.stl"

SET_COUNT = 5
DAGGERS_PER_SET = 4
COUNT = SET_COUNT * DAGGERS_PER_SET

BRIM_HEIGHT = 0.28
BRIM_WIDTH = 3.0
BRIM_OVERLAP = 0.7
COL_GAP = 6.0
ROW_GAP = 6.0


def box(width: float, depth: float, height: float, x: float, y: float) -> trimesh.Trimesh:
    mesh = trimesh.creation.box(extents=[width, depth, height])
    mesh.apply_translation([x, y, height / 2])
    return mesh


def brim_frame(width: float, depth: float) -> list[trimesh.Trimesh]:
    """Four rails forming a removable printed brim around one dagger."""
    inner_w = max(1.0, width - 2 * BRIM_OVERLAP)
    inner_d = max(1.0, depth - 2 * BRIM_OVERLAP)
    outer_w = inner_w + 2 * BRIM_WIDTH
    outer_d = inner_d + 2 * BRIM_WIDTH

    return [
        box(outer_w, BRIM_WIDTH, BRIM_HEIGHT, 0, +(inner_d + BRIM_WIDTH) / 2),
        box(outer_w, BRIM_WIDTH, BRIM_HEIGHT, 0, -(inner_d + BRIM_WIDTH) / 2),
        box(BRIM_WIDTH, inner_d, BRIM_HEIGHT, +(inner_w + BRIM_WIDTH) / 2, 0),
        box(BRIM_WIDTH, inner_d, BRIM_HEIGHT, -(inner_w + BRIM_WIDTH) / 2, 0),
    ]


def laid_flat(source: trimesh.Trimesh) -> trimesh.Trimesh:
    result = source.copy()
    # Original long dagger direction is +Z.  Rotate it into the XY bed plane;
    # the smaller original X dimension becomes print height.
    result.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0]))
    result.apply_translation(-result.bounds[0])
    result.apply_translation([-result.extents[0] / 2, -result.extents[1] / 2, 0])
    return result


def main() -> None:
    source = trimesh.load_mesh(SOURCE, force="mesh")
    dagger = laid_flat(source)
    width, depth, _ = dagger.extents

    cell_w = width + 2 * (BRIM_WIDTH - BRIM_OVERLAP) + COL_GAP
    cell_d = depth + 2 * (BRIM_WIDTH - BRIM_OVERLAP) + ROW_GAP
    cols = SET_COUNT
    rows = DAGGERS_PER_SET

    meshes: list[trimesh.Trimesh] = []
    for col in range(cols):
        for row in range(rows):
            cx = (col - (cols - 1) / 2) * cell_w
            cy = (row - (rows - 1) / 2) * cell_d

            part = dagger.copy()
            part.apply_translation([cx, cy, 0])
            unit = [part]

            for rail in brim_frame(width, depth):
                rail.apply_translation([cx, cy, 0])
                unit.append(rail)

            # Make the brim and dagger one watertight printable body.  The
            # overlap is only 0.7 mm around the lowest edge, so the brim is
            # easy to snip away without changing the functional geometry.
            try:
                meshes.append(trimesh.boolean.union(unit, engine="manifold"))
            except BaseException:
                meshes.append(trimesh.util.concatenate(unit))

    plate = trimesh.util.concatenate(meshes)
    plate.export(OUTPUT)

    size = plate.extents
    print(f"Created: {OUTPUT}")
    print(f"Daggers: {COUNT} ({SET_COUNT} sets x {DAGGERS_PER_SET})")
    print(f"Plate: {size[0]:.1f} x {size[1]:.1f} x {size[2]:.1f} mm")
    print("Orientation: flat; supports: OFF; slicer brim: OFF")


if __name__ == "__main__":
    main()
