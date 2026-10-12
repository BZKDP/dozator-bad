# -*- coding: utf-8 -*-
"""V11-B — gravity-indexed four-pocket star gate for 70 x Ø47 cans.

The queue, not the servo, turns the rotor.  A SG90/MG90S briefly lifts a
gravity pawl with its stock double horn.  The pawl then falls into the next of
four index stops after 90 degrees.  In every indexed position the load path is
rotor -> index stop -> pawl -> frame -> common cassette lower-sheet edge clamp; the servo horn
has clearance and carries no holding load.

Coordinates in millimetres:
    X  lane width and can/rotor axis
    Y  can travel, positive towards the customer
    Z  vertical
"""
from __future__ import annotations

import csv
import hashlib
import json
import math
import zipfile
from pathlib import Path
from typing import Iterable

import numpy as np
import trimesh
from shapely import affinity
from shapely.geometry import LineString, Point, Polygon, box as sbox
from shapely.ops import unary_union


HERE = Path(__file__).resolve().parent
COMMON_ASSEMBLY_DIR = (
    HERE.parent / "00_ОБЩАЯ_КОМПОНОВКА" / "STL_КАССЕТА_СБОРКА"
)
PRINT_DIR = HERE / "STL_ПЕЧАТЬ"
ASSEMBLY_DIR = HERE / "STL_СБОРКА_НЕ_ПЕЧАТАТЬ"
PLATE_DIR = HERE / "PLATES_3MF"
for directory in (PRINT_DIR, ASSEMBLY_DIR, PLATE_DIR):
    directory.mkdir(parents=True, exist_ok=True)


P = {
    "revision": "V11-B-GRAVITY-STAR-R2",
    "lane_count": 5,
    "lane_clear": 75.0,
    "lane_pitch": 79.0,
    "divider_thickness": 4.0,
    "sheet_thickness": 4.0,
    "sheet_front_y": 0.0,
    "divider_front_y": 39.48236084,
    "sheet_reference_back_y": -551.03527832,
    "common_reference_lane_center_global_y": 197.5,
    "can_length": 70.0,
    "can_axis_x": 0.5,
    "can_diameter_nominal": 47.0,
    "can_diameter_envelope": 48.0,
    "lower_slope_deg": 15.0,
    "rotor_center_y": 56.0,
    # The first draft used the lower front vertex of the imported 4 mm sheet
    # as if it were the can-running face.  Raising the complete mechanism by
    # this measured delta makes the Ø48 queue tangent to the actual upper
    # face of cassette_lower_sheet.stl instead of intersecting it.
    "mechanism_z_shift": 4.20,
    "rotor_center_z": 33.80,
    "pocket_center_radius": 42.0,
    "pocket_cut_radius": 24.8,
    "star_outer_radius": 58.0,
    "rotor_axial_clear": 71.2,
    "rotor_end_spider_thickness": 0.9,
    "rotor_left_inner_x": -34.95,
    "rotor_right_inner_x": 36.1,
    "rotor_axis_bore_radius": 7.35,
    "stationary_bearing_radius": 7.0,
    "index_radius": 23.0,
    "index_lug_radius": 3.0,
    "index_axial_thickness": 1.65,
    "rotation_per_cycle_deg": 90.0,
    "pawl_lift_deg": 7.1,
    "pawl_pivot_y": 99.0,
    "pawl_pivot_z": 35.70,
    "pawl_axial_x0": -37.40,
    "pawl_axial_thickness": 1.55,
    "pawl_pivot_clearance": 0.35,
    "left_frame_inner_x": -38.25,
    "left_frame_outer_x": -39.45,
    # The right flat cheek starts 0.05 mm outside the 75 mm floor.  This
    # removes the legacy 0.10–0.20 mm fused overlap now that the frame is a
    # genuinely assemblable flat-pack instead of one boolean body.
    "right_frame_inner_x": 37.55,
    "right_frame_outer_x": 38.75,
    "module_envelope_width": 78.20,
    "servo_axis_x": -31.0,
    "servo_axis_y": 144.0,
    "servo_axis_z": 40.70,
    "servo_home_horn_deg": 210.0,
    "servo_unlock_horn_deg": 238.0,
    "attachment_screws": 2,
    "bed_x": 220.0,
    "bed_y": 220.0,
    "sheet_upper_front_z": 12.649819374084473,
}


# ---------------------------------------------------------------------------
# Mesh primitives
# ---------------------------------------------------------------------------
def clean(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    result = mesh.copy()
    result.remove_unreferenced_vertices()
    trimesh.repair.fix_normals(result)
    return result


def remove_tiny_components(mesh: trimesh.Trimesh, minimum_volume=1.0) -> trimesh.Trimesh:
    """Drop numerical boolean slivers while preserving every real printed body."""
    components = [part for part in clean(mesh).split() if abs(part.volume) >= minimum_volume]
    if not components:
        raise RuntimeError("Boolean operation left no printable component")
    return clean(trimesh.util.concatenate(components))


def fuse(meshes: Iterable[trimesh.Trimesh]) -> trimesh.Trimesh:
    items = [clean(m) for m in meshes if m is not None and len(m.faces)]
    if not items:
        raise ValueError("No meshes to fuse")
    if len(items) == 1:
        return items[0]
    result = trimesh.boolean.union(items, engine="manifold")
    if result is None:
        raise RuntimeError("Boolean union returned no mesh")
    return clean(result)


def cut(mesh: trimesh.Trimesh, cutters: Iterable[trimesh.Trimesh]) -> trimesh.Trimesh:
    tools = [clean(m) for m in cutters if m is not None and len(m.faces)]
    if not tools:
        return clean(mesh)
    result = trimesh.boolean.difference([clean(mesh), *tools], engine="manifold")
    if result is None:
        raise RuntimeError("Boolean difference returned no mesh")
    return clean(result)


def intersection_volume(a: trimesh.Trimesh, b: trimesh.Trimesh):
    if np.any(a.bounds[1] < b.bounds[0]) or np.any(b.bounds[1] < a.bounds[0]):
        return 0.0
    try:
        result = trimesh.boolean.intersection([clean(a), clean(b)], engine="manifold")
        if result is None:
            return 0.0
        return abs(float(result.volume))
    except BaseException:
        return None


def bx(size, center=(0.0, 0.0, 0.0)):
    mesh = trimesh.creation.box(extents=np.asarray(size, dtype=float))
    mesh.apply_translation(np.asarray(center, dtype=float))
    return mesh


def cylinder_x(radius, length, center=(0.0, 0.0, 0.0), sections=48):
    mesh = trimesh.creation.cylinder(radius=radius, height=length, sections=sections)
    mesh.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0]))
    mesh.apply_translation(np.asarray(center, dtype=float))
    return mesh


def cylinder_normal(radius, length, center, slope_deg, sections=40):
    mesh = trimesh.creation.cylinder(radius=radius, height=length, sections=sections)
    mesh.apply_transform(
        trimesh.transformations.rotation_matrix(math.radians(-slope_deg), [1, 0, 0])
    )
    mesh.apply_translation(np.asarray(center, dtype=float))
    return mesh


def extrude_yz(poly, thickness, x0=0.0):
    if poly.geom_type in ("MultiPolygon", "GeometryCollection"):
        pieces = [
            extrude_yz(item, thickness, x0)
            for item in poly.geoms
            if item.geom_type == "Polygon" and not item.is_empty and item.area > 1e-6
        ]
        if not pieces:
            raise ValueError("No polygonal area to extrude")
        return clean(trimesh.util.concatenate(pieces))
    mesh = trimesh.creation.extrude_polygon(poly, height=thickness)
    transform = np.array(
        [[0, 0, 1, x0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]],
        dtype=float,
    )
    mesh.apply_transform(transform)
    return clean(mesh)


def extrude_xy(poly, thickness, z0=0.0):
    mesh = trimesh.creation.extrude_polygon(poly, height=thickness)
    mesh.apply_translation([0.0, 0.0, z0])
    return clean(mesh)


def rotate(mesh, angle_deg, axis, point=(0.0, 0.0, 0.0)):
    result = mesh.copy()
    result.apply_transform(
        trimesh.transformations.rotation_matrix(math.radians(angle_deg), axis, point)
    )
    return clean(result)


def translate(mesh, xyz):
    result = mesh.copy()
    result.apply_translation(np.asarray(xyz, dtype=float))
    return result


def on_bed(mesh):
    result = clean(mesh)
    result.apply_translation([0, 0, -float(result.bounds[0, 2])])
    return result


def sector_annulus(r_inner, r_outer, angle0, angle1, center, steps=72):
    cy, cz = center
    angles = np.linspace(math.radians(angle0), math.radians(angle1), steps)
    outer = [(cy + r_outer * math.cos(a), cz + r_outer * math.sin(a)) for a in angles]
    inner = [
        (cy + r_inner * math.cos(a), cz + r_inner * math.sin(a))
        for a in angles[::-1]
    ]
    return Polygon(outer + inner)


def slope_slab(x0, x1, y0, y1, z0, z1, thickness):
    run = y1 - y0
    rise = z1 - z0
    length = math.hypot(run, rise)
    theta = math.atan2(rise, run)
    normal = np.array([0.0, -math.sin(theta), math.cos(theta)])
    center = np.array([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2])
    center -= normal * thickness / 2
    mesh = bx((x1 - x0, length, thickness), center)
    return rotate(mesh, math.degrees(theta), [1, 0, 0], center)


def cylinder_between(p0, p1, radius, sections=24):
    p0 = np.asarray(p0, dtype=float)
    p1 = np.asarray(p1, dtype=float)
    vector = p1 - p0
    length = float(np.linalg.norm(vector))
    mesh = trimesh.creation.cylinder(radius=radius, height=length, sections=sections)
    mesh.apply_transform(trimesh.geometry.align_vectors([0, 0, 1], vector / length))
    mesh.apply_translation((p0 + p1) / 2)
    return mesh


def export_mesh(mesh, path):
    mesh = clean(mesh)
    mesh.export(path)
    ext = mesh.extents
    return {
        "file": path.name,
        "extents_mm": [round(float(v), 2) for v in ext],
        "watertight": bool(mesh.is_watertight),
        "components": int(len(mesh.split(only_watertight=False))),
        "volume_cm3": round(abs(float(mesh.volume)) / 1000.0, 3),
        "min_z_mm": round(float(mesh.bounds[0, 2]), 4),
        "finite": bool(np.all(np.isfinite(mesh.vertices))),
    }


# ---------------------------------------------------------------------------
# Cassette, cans and bought-part references
# ---------------------------------------------------------------------------
def inlet_top(y):
    # Running face is the actual upper plane of the imported 4 mm sheet.
    return exact_sheet_upper(y)


def exact_sheet_upper(y):
    """Upper running plane measured from cassette_lower_sheet.stl."""
    return P["sheet_upper_front_z"] - y * math.tan(
        math.radians(P["lower_slope_deg"])
    )


def exact_sheet_lower(y):
    """Lower plane of the 4 mm sheet at the same local Y coordinate."""
    return exact_sheet_upper(y) - P["sheet_thickness"] / math.cos(
        math.radians(P["lower_slope_deg"])
    )


def local_feed_top(y):
    """Integrated 20° final ramp; about 1.5 mm lower than the cassette sheet at its nose."""
    if y <= -3.0:
        return inlet_top(y)
    return inlet_top(-3.0) - (y + 3.0) * math.tan(math.radians(20.0))


def output_top(y):
    return (
        -34.60
        + P["mechanism_z_shift"]
        - (y - P["rotor_center_y"]) * math.tan(math.radians(12.0))
    )


def can_mesh(center):
    x, y, z = center
    r = P["can_diameter_nominal"] / 2
    return fuse(
        [
            cylinder_x(r, 66.0, (x, y, z), sections=64),
            cylinder_x(r + 0.55, 2.0, (x - 34.0, y, z), sections=64),
            cylinder_x(r + 0.55, 2.0, (x + 34.0, y, z), sections=64),
            # Recessed end face stays inside the 70 mm rim-to-rim envelope.
            cylinder_x(r - 1.2, 1.0, (x + 34.0, y, z), sections=64),
        ]
    )


def _common_to_local(mesh, lane_center_global_y=None):
    if lane_center_global_y is None:
        lane_center_global_y = P["common_reference_lane_center_global_y"]
    result = mesh.copy()
    transform = np.array(
        [
            [0.0, 1.0, 0.0, -lane_center_global_y],
            [-1.0, 0.0, 0.0, P["divider_front_y"]],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    result.apply_transform(transform)
    return clean(result)


def _load_common(name):
    path = COMMON_ASSEMBLY_DIR / name
    if not path.exists():
        raise FileNotFoundError(f"Missing common assembly reference: {path}")
    mesh = trimesh.load_mesh(path, process=True)
    if not isinstance(mesh, trimesh.Trimesh):
        mesh = trimesh.util.concatenate(tuple(mesh.geometry.values()))
    return clean(mesh)


def cassette_reference():
    return _common_to_local(_load_common("cassette_lower_sheet.stl"))


def divider_references():
    base = _load_common("cassette_divider.stl")
    result = {}
    centres = (79.0, 158.0, 237.0, 316.0)
    for index, global_y in enumerate(centres, start=1):
        item = base.copy()
        item.apply_translation([0.0, global_y, 0.0])
        result[f"divider_{index}"] = _common_to_local(item)
    return result


def adjacent_divider_references():
    all_dividers = divider_references()
    return {
        "divider_left": all_dividers["divider_2"],
        "divider_right": all_dividers["divider_3"],
    }


def horn_outline(clearance=0.0):
    hub = Point(0, 0).buffer(3.7 + clearance, resolution=24)
    pieces = [hub]
    arm = 15.8 + clearance
    for sign in (-1, 1):
        pieces.append(
            Polygon(
                [
                    (sign * 3.0, 3.45 + clearance),
                    (sign * arm, 2.0 + clearance),
                    (sign * arm, -2.0 - clearance),
                    (sign * 3.0, -3.45 - clearance),
                ]
            )
        )
        pieces.append(Point(sign * arm, 0).buffer(2.0 + clearance, resolution=16))
    return unary_union(pieces)


def servo_reference_parts(horn_angle=None):
    if horn_angle is None:
        horn_angle = P["servo_home_horn_deg"]
    face_x = P["servo_axis_x"]
    ay = P["servo_axis_y"]
    az = P["servo_axis_z"]
    body_cx = face_x + 1.2 + 28.5 / 2
    body_cz = az - 5.4
    body = bx((28.5, 22.8, 12.2), (body_cx, ay, body_cz))
    flange = fuse(
        [
            bx((2.4, 24.0, 10.0), (face_x + 0.2, ay, body_cz)),
            cylinder_x(3.2, 2.4, (face_x + 0.2, ay - 14.5, body_cz), sections=28),
            cylinder_x(3.2, 2.4, (face_x + 0.2, ay + 14.5, body_cz), sections=28),
            bx((2.4, 8.0, 5.8), (face_x + 0.2, ay - 12.0, body_cz)),
            bx((2.4, 8.0, 5.8), (face_x + 0.2, ay + 12.0, body_cz)),
        ]
    )
    boss = cylinder_x(5.8, 2.2, (face_x - 1.1, ay, az), sections=40)
    shaft = cylinder_x(2.35, 5.5, (face_x - 4.0, ay, az), sections=32)
    poly = affinity.rotate(horn_outline(), horn_angle, origin=(0, 0))
    poly = affinity.translate(poly, ay, az)
    horn = extrude_yz(poly, 1.8, x0=face_x - 5.7)
    screw = fuse(
        [
            cylinder_x(1.0, 6.0, (face_x - 3.8, ay, az), sections=24),
            cylinder_x(2.15, 1.2, (face_x - 6.1, ay, az), sections=24),
        ]
    )
    return {
        "servo_body": fuse([body, flange, boss]),
        "servo_shaft": shaft,
        "servo_horn": horn,
        "servo_center_screw": screw,
    }


def servo_wire_reference():
    x0 = P["servo_axis_x"] + 29.5
    y0 = P["servo_axis_y"]
    z0 = P["servo_axis_z"] - 11.0
    result = {}
    for name, dy in (("brown", -2.0), ("red", 0.0), ("orange", 2.0)):
        result[name] = fuse(
            [
                cylinder_x(0.65, 31.0, (x0 + 15.5, y0 + dy, z0), sections=16),
                bx((8.0, 5.0, 7.0), (x0 + 35.0, y0 + dy, z0)),
            ]
        )
    return result


# ---------------------------------------------------------------------------
# B01A/B01B: support-free keyed four-pocket star rotor
# ---------------------------------------------------------------------------
def pocket_centres_home():
    cy = P["rotor_center_y"]
    cz = P["rotor_center_z"]
    radius = P["pocket_center_radius"]
    return [
        (cy + radius * math.cos(math.radians(a)), cz + radius * math.sin(math.radians(a)))
        for a in (0.0, 90.0, 180.0, 270.0)
    ]


def star_profile():
    center = (P["rotor_center_y"], P["rotor_center_z"])
    disc = Point(*center).buffer(P["star_outer_radius"], resolution=96)
    pockets = unary_union(
        [Point(y, z).buffer(P["pocket_cut_radius"], resolution=64) for y, z in pocket_centres_home()]
    )
    return disc.difference(pockets)


def rotor_spider_profile():
    # The four scallops are open to the outside, so this is still a light
    # four-lobed spider rather than a solid disc.  Keeping it as one valid
    # polygon also avoids zero-width contacts in the sliced top web.
    return star_profile()


def pocket_rail_positions():
    result = []
    rail_offset = P["pocket_cut_radius"] + 1.2
    for pocket_angle, (py, pz) in zip((0.0, 90.0, 180.0, 270.0), pocket_centres_home()):
        for relative in (90.0, 180.0, 270.0):
            a = math.radians(pocket_angle + relative)
            result.append((py + rail_offset * math.cos(a), pz + rail_offset * math.sin(a)))
    return result


def index_cross_profile():
    cy = P["rotor_center_y"]
    cz = P["rotor_center_z"]
    pieces = [Point(cy, cz).buffer(12.0, resolution=40)]
    for angle in (0.0, 90.0, 180.0, 270.0):
        a = math.radians(angle)
        end = (
            cy + P["index_radius"] * math.cos(a),
            cz + P["index_radius"] * math.sin(a),
        )
        pieces.append(LineString([(cy, cz), end]).buffer(P["index_lug_radius"], cap_style=1))
    return unary_union(pieces)


def rotor_key_centres():
    cy = P["rotor_center_y"]
    cz = P["rotor_center_z"]
    return [
        (
            cy + 9.5 * math.cos(math.radians(angle)),
            cz + 9.5 * math.sin(math.radians(angle)),
        )
        for angle in (0.0, 120.0, 240.0)
    ]


def make_rotor_body():
    wall = P["rotor_end_spider_thickness"]
    left_inner = P["rotor_left_inner_x"]
    right_inner = P["rotor_right_inner_x"]
    left_outer = left_inner - wall
    # Support-free Ender-3 architecture: the left datum spider lies directly
    # on the bed and the right end stays open.  Every longitudinal feature
    # grows vertically from this datum; no 46 x 76 mm top bridge remains.
    axial_x0 = left_inner - 0.10
    axial_x1 = right_inner
    axial_length = axial_x1 - axial_x0
    axial_center = (axial_x0 + axial_x1) / 2
    base_profile = unary_union(
        [
            rotor_spider_profile(),
            *[
                Point(y, z).buffer(1.75, resolution=24)
                for y, z in pocket_rail_positions()
            ],
        ]
    )
    parts = [
        extrude_yz(base_profile, wall, left_outer),
        cylinder_x(12.0, axial_length, (axial_center, P["rotor_center_y"], P["rotor_center_z"]), sections=64),
    ]

    # Four full-length radial blades are the separators that guarantee one
    # pocket per 90° index.  They print vertically with the rotor axis on Z.
    for angle in (45.0, 135.0, 225.0, 315.0):
        blade = sector_annulus(
            8.5,
            P["star_outer_radius"],
            angle - 4.0,
            angle + 4.0,
            (P["rotor_center_y"], P["rotor_center_z"]),
            steps=22,
        )
        parts.append(extrude_yz(blade, axial_length, axial_center - axial_length / 2))

    # Three longitudinal rails form each full-length pocket without a heavy drum.
    for y, z in pocket_rail_positions():
        parts.append(cylinder_x(1.6, axial_length, (axial_center, y, z), sections=24))

    rotor = fuse(parts)
    key_sockets = [
        cylinder_x(1.45, 5.4, (left_outer + 2.6, y, z), sections=32)
        for y, z in rotor_key_centres()
    ]
    bore = cylinder_x(
        P["rotor_axis_bore_radius"],
        axial_length + 5.0,
        (axial_center, P["rotor_center_y"], P["rotor_center_z"]),
        sections=64,
    )
    return cut(rotor, [bore, *key_sockets])


def make_rotor_indexer():
    left_outer = P["rotor_left_inner_x"] - P["rotor_end_spider_thickness"]
    x0 = left_outer - P["index_axial_thickness"]
    body = extrude_yz(index_cross_profile(), P["index_axial_thickness"], x0)
    pins = [
        cylinder_x(1.20, 5.0, (left_outer + 2.40, y, z), sections=32)
        for y, z in rotor_key_centres()
    ]
    indexer = fuse([body, *pins])
    bore = cylinder_x(
        P["rotor_axis_bore_radius"],
        P["index_axial_thickness"] + 6.0,
        (x0 + (P["index_axial_thickness"] + 6.0) / 2, P["rotor_center_y"], P["rotor_center_z"]),
        sections=64,
    )
    return cut(indexer, [bore])


def make_rotor():
    """Complete rotating assembly; two printed bodies joined by three keys."""
    return clean(trimesh.util.concatenate([make_rotor_body(), make_rotor_indexer()]))


# ---------------------------------------------------------------------------
# B02L/B02R: split frame and no-drill cassette lower-sheet edge clamp
# ---------------------------------------------------------------------------
def guide_profile():
    return sector_annulus(
        66.4,
        71.0,
        150.0,
        302.0,
        (P["rotor_center_y"], P["rotor_center_z"]),
        steps=90,
    )


def rotor_sweep_cutter(axial_length=90.0):
    return cylinder_x(
        P["star_outer_radius"] + 0.8,
        axial_length,
        (P["can_axis_x"], P["rotor_center_y"], P["rotor_center_z"]),
        sections=96,
    )


def indexed_can_sweep_cutter():
    """Keep-out for the actual rim envelope during one 90 degree index.

    The axial cutter stops just outside the 70 mm can so the two narrow side
    webs of the no-drill sheet clamp can join its upper and lower jaws without
    entering the can path.
    """
    return cylinder_x(
        P["pocket_center_radius"] + P["can_diameter_envelope"] / 2 + 0.40,
        P["can_length"] + 0.60,
        (P["can_axis_x"], P["rotor_center_y"], P["rotor_center_z"]),
        sections=112,
    )


def frame_side_profile(side):
    cy = P["rotor_center_y"]
    cz = P["rotor_center_z"]
    hub = Point(cy, cz).buffer(9.5, resolution=40)
    guide = guide_profile()
    spokes = []
    for angle in (165.0, 225.0, 275.0, 315.0, 45.0):
        a = math.radians(angle)
        endpoint = (cy + 68.0 * math.cos(a), cz + 68.0 * math.sin(a))
        spokes.append(LineString([(cy, cz), endpoint]).buffer(3.5, cap_style=1))
    output_anchor = (
        cy + 68.0 * math.cos(math.radians(275.0)),
        cz + 68.0 * math.sin(math.radians(275.0)),
    )
    output_rail = LineString(
        [output_anchor, (cy + 52.0, output_top(cy + 52.0) - 2.0), (cy + 126.0, output_top(cy + 126.0) - 2.0)]
    ).buffer(3.2)
    profile = unary_union([guide, hub, output_rail, *spokes])
    if side == "left":
        body_cz = P["servo_axis_z"] - 5.4
        ear1 = (P["servo_axis_y"] - 14.5, body_cz)
        ear2 = (P["servo_axis_y"] + 14.5, body_cz)
        pivot = (P["pawl_pivot_y"], P["pawl_pivot_z"])
        profile = unary_union(
            [
                profile,
                Point(*ear1).buffer(5.0, resolution=24),
                Point(*ear2).buffer(5.0, resolution=24),
                Point(*pivot).buffer(6.0, resolution=28),
                LineString([(cy + 38.0, cz + 42.0), ear1]).buffer(3.4),
                LineString([ear1, ear2]).buffer(3.4),
                LineString([(cy + 34.0, cz + 32.0), pivot]).buffer(3.4),
            ]
        )
    # Any cheek wider than the 75 mm clear lane exists only in front of the
    # exact divider end.  The rear guide is exported separately inside ±37.5.
    return profile.intersection(sbox(P["divider_front_y"] + 0.25, -160.0, 205.0, 190.0))


def rear_guide_profile():
    # Complete inner guide lives inside the 75 mm clear lane, including the
    # 39.48 mm zone where the real dividers project beyond the lower sheet.
    cy = P["rotor_center_y"]
    cz = P["rotor_center_z"]
    feed_connector = sector_annulus(58.9, 67.2, 190.0, 216.0, (cy, cz), steps=28)
    output_anchor = (
        cy + 67.0 * math.cos(math.radians(276.0)),
        cz + 67.0 * math.sin(math.radians(276.0)),
    )
    output_connector = LineString(
        [output_anchor, (cy + 8.0, output_top(cy + 8.0) - 1.2)]
    ).buffer(3.0)
    return unary_union([guide_profile(), feed_connector, output_connector])


def clamp_jaw(side):
    if side == "left":
        x0, x1 = -37.5, 0.0
    else:
        x0, x1 = 0.0, 37.5
    angle = P["lower_slope_deg"]
    # The clamp must wrap the imported sheet, never occupy it.  Its 2.0 mm
    # upper jaw clears the measured upper plane by 0.50 mm.  A 10 mm run-on
    # wedge lets the can climb onto that jaw instead of striking a square
    # step.  The lower jaw clears the measured lower plane by 0.60 mm.
    upper_thickness = 2.0
    upper_clearance = 0.50
    upper_top_offset = upper_thickness / math.cos(math.radians(angle)) + upper_clearance
    ramp_y0, ramp_y1 = -27.0, -16.8
    entry_ramp = extrude_yz(
        Polygon(
            [
                (ramp_y0, exact_sheet_upper(ramp_y0) + upper_clearance),
                (ramp_y1, exact_sheet_upper(ramp_y1) + upper_clearance),
                (ramp_y1, exact_sheet_upper(ramp_y1) + upper_top_offset),
                (ramp_y0, exact_sheet_upper(ramp_y0) + upper_clearance + 0.40),
            ]
        ),
        x1 - x0,
        x0,
    )
    upper_a = slope_slab(
        x0,
        x1,
        -17.0,
        0.0,
        exact_sheet_upper(-17.0) + upper_top_offset,
        exact_sheet_upper(0.0) + upper_top_offset,
        upper_thickness,
    )
    upper_b = slope_slab(
        x0,
        x1,
        0.0,
        13.0,
        exact_sheet_upper(0.0) + upper_top_offset,
        local_feed_top(13.0),
        upper_thickness,
    )
    lower = slope_slab(
        x0,
        x1,
        -17.0,
        2.5,
        exact_sheet_lower(-17.0) - 0.60,
        exact_sheet_lower(2.5) - 0.60,
        4.0,
    )
    # Join the two jaws immediately ahead of the exact front edge.  At this
    # Y the connector lies just outside the rotor keep-out; the old connector
    # at y=2.5 was cut in two after the sheet datum was corrected.
    y = 0.55
    lower_bridge_z = exact_sheet_lower(y) - 1.0
    upper_bridge_z = 13.0
    bridge_z = (lower_bridge_z + upper_bridge_z) / 2
    # Only the axial margin outside the 70 mm can joins the jaws.  The middle
    # is cleared by indexed_can_sweep_cutter below.
    if side == "left":
        bridge_x0, bridge_x1 = -37.5, -34.85
    else:
        bridge_x0, bridge_x1 = 35.85, 37.5
    bridge = bx(
        (bridge_x1 - bridge_x0, 0.90, upper_bridge_z - lower_bridge_z),
        ((bridge_x0 + bridge_x1) / 2, y, bridge_z),
    )
    screw_x = -30.0 if side == "left" else 30.0
    screw_y = -7.0
    surface_z = exact_sheet_lower(screw_y) - 0.60
    boss = cylinder_normal(5.0, 8.0, (screw_x, screw_y, surface_z - 4.8), angle, sections=36)
    jaw = fuse([entry_ramp, upper_a, upper_b, lower, bridge, boss])
    pilot = cylinder_normal(1.65, 14.0, (screw_x, screw_y, surface_z - 2.5), angle)
    return cut(jaw, [pilot, rotor_sweep_cutter(), indexed_can_sweep_cutter()])


def output_chute(side):
    x0, x1 = (-37.5, 0.0) if side == "left" else (0.0, 37.5)
    return slope_slab(x0, x1, 52.0, 175.0, output_top(52.0), output_top(175.0), 3.0)


def bearing_sleeve(side):
    cy = P["rotor_center_y"]
    cz = P["rotor_center_z"]
    if side == "left":
        x0, x1 = P["left_frame_outer_x"], -33.4
    else:
        x0, x1 = 32.6, P["right_frame_outer_x"]
    sleeve = cylinder_x(P["stationary_bearing_radius"], x1 - x0, ((x0 + x1) / 2, cy, cz), sections=56)
    pilot = cylinder_x(2.15, x1 - x0 + 2.0, ((x0 + x1) / 2, cy, cz), sections=40)
    if side == "left":
        head = cylinder_x(4.1, 1.1, (P["left_frame_outer_x"] + 0.50, cy, cz), sections=48)
        return cut(sleeve, [pilot, head])
    insert_pocket = cylinder_x(2.35, 5.0, (35.0, cy, cz), sections=40)
    return cut(sleeve, [pilot, insert_pocket])


def seam_features(side):
    sites = [
        (P["rotor_center_y"] + 52.0, output_top(P["rotor_center_y"] + 52.0) - 1.5),
        (P["rotor_center_y"] + 112.0, output_top(P["rotor_center_y"] + 112.0) - 1.5),
    ]
    if side == "left":
        pins = [cylinder_x(1.75, 6.0, (2.0, y, z), sections=28) for y, z in sites]
        return pins, []
    bosses = [cylinder_x(3.8, 7.8, (3.8, y, z), sections=32) for y, z in sites]
    sockets = [cylinder_x(2.05, 8.5, (3.8, y, z), sections=28) for y, z in sites]
    return bosses, sockets


def servo_cantilever_parts():
    face_x = P["servo_axis_x"]
    ay = P["servo_axis_y"]
    body_cz = P["servo_axis_z"] - 5.4
    ears = [(ay - 14.5, body_cz), (ay + 14.5, body_cz)]
    # Two independent mounting pads sit directly on axial columns.  The old
    # one-piece pad bridged 29 mm between the columns when B02L lay flat.
    pads = []
    for y, z in ears:
        pad = extrude_yz(Point(y, z).buffer(4.2, resolution=24), 2.0, x0=face_x - 0.1)
        pads.append(cut(pad, [cylinder_x(1.10, 5.0, (face_x, y, z), sections=24)]))
    beams = []
    for y, z in ears:
        beams.append(
            cylinder_x(
                2.8,
                face_x - P["left_frame_inner_x"] + 0.5,
                ((face_x + P["left_frame_inner_x"] - 0.5) / 2, y, z),
                sections=28,
            )
        )
    return [*pads, *beams]


def pawl_pivot_and_stops():
    py = P["pawl_pivot_y"]
    pz = P["pawl_pivot_z"]
    pin_x0 = P["left_frame_inner_x"] - 0.02
    pin_x1 = -36.68
    pin_center = (pin_x0 + pin_x1) / 2
    pin_length = pin_x1 - pin_x0
    # Printed pivot pin ends before the rotating B01 end spider.  B03 has a
    # +0.35 mm rotating bore and is captive once B02R is closed.
    pivot = cylinder_x(1.50, pin_length, (pin_center, py, pz), sections=36)
    # A tab 18 mm below the pivot meets these positive stops at 0°/-7.1°.
    lock_stop = cylinder_x(2.0, pin_length, (pin_center, py + 4.00, pz - 18.0), sections=30)
    lift_stop = cylinder_x(2.0, pin_length, (pin_center, py - 6.19, pz - 17.37), sections=30)
    # All supporting material stays on/outside the cheek; only the three short
    # pins enter the 1.17 mm mechanism gap.
    support_profile = unary_union(
        [
            Point(py, pz).buffer(5.2, resolution=28),
            Point(py + 4.00, pz - 18.0).buffer(4.2, resolution=24),
            Point(py - 6.19, pz - 17.37).buffer(4.2, resolution=24),
            LineString([(py, pz), (py + 4.00, pz - 18.0)]).buffer(2.8),
            LineString([(py, pz), (py - 6.19, pz - 17.37)]).buffer(2.8),
        ]
    )
    support_plate = extrude_yz(
        support_profile,
        P["left_frame_inner_x"] - P["left_frame_outer_x"],
        P["left_frame_outer_x"],
    )
    return [support_plate, pivot, lock_stop, lift_stop]


MAIN_GUIDE_PIN_SITES = (
    (56.0, -39.10 + P["mechanism_z_shift"]),
    (80.0, -34.77 + P["mechanism_z_shift"]),
)
FLOOR_GUIDE_PIN_SITES = {
    "outlet": (65.0, -38.00 + P["mechanism_z_shift"]),
}


def rear_guide_raw(side):
    """Lower/forward guide, trimmed against the actual cassette sheet.

    The imported sheet already supports and guides the incoming queue.  The
    old annular profile passed straight through that solid sheet and left an
    impossible overlapping upper arc.  After the exact boolean trim only the
    largest lower/forward component is retained; it hands the can into the
    rotor and out toward the chute without duplicating the sheet.
    """
    if side == "left":
        rear_x0, rear_x1 = -37.40, -35.80
    else:
        rear_x0, rear_x1 = 35.80, 37.40
    rear_fin = extrude_yz(rear_guide_profile(), rear_x1 - rear_x0, rear_x0)
    trimmed = cut(rear_fin, [rotor_sweep_cutter(), cassette_reference()])
    components = list(clean(trimmed).split(only_watertight=False))
    if not components:
        raise RuntimeError("Exact cassette trim removed the whole rear guide")
    return clean(max(components, key=lambda part: abs(float(part.volume))))


def frame_main_locator_pins(side):
    """Two short axial pins; the M4 rotor axle later captures the joint."""
    if side == "left":
        x0, x1 = -38.60, -36.10
    else:
        x0, x1 = 36.10, 38.60
    return [
        cylinder_x(1.15, x1 - x0, ((x0 + x1) / 2, y, z), sections=28)
        for y, z in MAIN_GUIDE_PIN_SITES
    ]


def make_frame_main(side):
    """Outboard cheek, bearing and purchased-part interfaces; prints face-down."""
    if side == "left":
        cheek_x0 = P["left_frame_outer_x"]
        cheek_thickness = P["left_frame_inner_x"] - P["left_frame_outer_x"]
    else:
        cheek_x0 = P["right_frame_inner_x"]
        cheek_thickness = P["right_frame_outer_x"] - P["right_frame_inner_x"]
    cheek = extrude_yz(frame_side_profile(side), cheek_thickness, cheek_x0)
    parts = [cheek, bearing_sleeve(side), *frame_main_locator_pins(side)]
    if side == "left":
        parts.extend(servo_cantilever_parts())
        parts.extend(pawl_pivot_and_stops())
    return fuse(parts)


def make_rear_guide(side):
    """Flat inner guide with main-cheek sockets and one outlet locator pin."""
    guide = rear_guide_raw(side)
    guide_mid_x = -36.60 if side == "left" else 36.60
    main_pin_sockets = [
        cylinder_x(1.40, 2.50, (guide_mid_x, y, z), sections=30)
        for y, z in MAIN_GUIDE_PIN_SITES
    ]
    guide = cut(guide, main_pin_sockets)

    if side == "left":
        floor_x0, floor_x1 = -36.30, -31.00
    else:
        floor_x0, floor_x1 = 31.00, 36.30
    floor_pins = [
        cylinder_x(
            1.35,
            floor_x1 - floor_x0,
            ((floor_x0 + floor_x1) / 2, y, z),
            sections=30,
        )
        for y, z in FLOOR_GUIDE_PIN_SITES.values()
    ]
    return fuse([guide, *floor_pins])


def guide_clearance_cutters(side, clearance=0.22):
    """Axis-expanded guide envelope used to leave a practical FDM slip fit."""
    raw = rear_guide_raw(side)
    offsets = (
        (0.0, 0.0, 0.0),
        (-clearance, 0.0, 0.0),
        (clearance, 0.0, 0.0),
        (0.0, -clearance, 0.0),
        (0.0, clearance, 0.0),
        (0.0, 0.0, -clearance),
        (0.0, 0.0, clearance),
    )
    return [translate(raw, offset) for offset in offsets]


def floor_pin_socket_cutters(kind):
    y, z = FLOOR_GUIDE_PIN_SITES[kind]
    return [
        cylinder_x(1.60, 7.0, (-33.50, y, z), sections=32),
        cylinder_x(1.60, 7.0, (33.50, y, z), sections=32),
    ]


def make_inlet_clamp():
    """One full-width sheet clamp, nested around the two flat guides."""
    clamp = fuse([clamp_jaw("left"), clamp_jaw("right")])
    cutters = [
        *guide_clearance_cutters("left"),
        *guide_clearance_cutters("right"),
    ]
    return remove_tiny_components(cut(clamp, cutters))


def make_output_floor():
    """One full-width outlet floor, keyed between the two flat guides."""
    floor = fuse([output_chute("left"), output_chute("right")])
    cutters = [
        *guide_clearance_cutters("left"),
        *guide_clearance_cutters("right"),
        *floor_pin_socket_cutters("outlet"),
        indexed_can_sweep_cutter(),
    ]
    return remove_tiny_components(cut(floor, cutters), minimum_volume=50.0)


def make_frame_components():
    """Return the six support-free flat-pack frame bodies in assembly coordinates."""
    main_left = make_frame_main("left")
    main_right = make_frame_main("right")
    guide_left = make_rear_guide("left")
    guide_right = make_rear_guide("right")
    inlet = make_inlet_clamp()
    outlet = make_output_floor()
    return main_left, main_right, guide_left, guide_right, inlet, outlet


def make_frame(side):
    """Compatibility assembly mesh used by the viewer and collision sweep."""
    main_left, main_right, guide_left, guide_right, inlet, outlet = make_frame_components()
    if side == "left":
        return clean(trimesh.util.concatenate([main_left, guide_left, inlet, outlet]))
    return clean(trimesh.util.concatenate([main_right, guide_right]))


# ---------------------------------------------------------------------------
# B03: passive gravity pawl
# ---------------------------------------------------------------------------
def pawl_profile():
    py = P["pawl_pivot_y"]
    pz = P["pawl_pivot_z"]
    index_y = P["rotor_center_y"]
    index_z = P["rotor_center_z"] + P["index_radius"]
    rear_tip = (index_y - P["index_lug_radius"] - 3.20, index_z + 8.0)
    upper_elbow = (index_y + 18.0, index_z + 16.0)
    # Rounded follower sits below the active stock-horn tip.  During the
    # 210° -> 238° sweep the horn intrudes into its HOME envelope and can
    # only continue by pushing B03 clockwise to the -7.1° hard stop.
    tail = (135.5319, 22.9688 + P["mechanism_z_shift"])
    profile = unary_union(
        [
            Point(py, pz).buffer(5.1, resolution=36),
            LineString([(py, pz), upper_elbow, rear_tip]).buffer(2.8, cap_style=1, join_style=1),
            # Low dog-leg clears the horn arm; actuation occurs only on the
            # rounded end follower, not by rubbing along the lever web.
            LineString([(py, pz), (119.0, 10.0), tail]).buffer(
                2.8, cap_style=1, join_style=1
            ),
            # Square rear face is self-locking against positive rotor torque.
            Polygon(
                [
                    (index_y - 8.2, index_z - 2.5),
                    (index_y - P["index_lug_radius"] - 0.12, index_z - 2.5),
                    (index_y - P["index_lug_radius"] - 0.12, index_z + 2.3),
                    (index_y - 8.2, index_z + 6.5),
                ]
            ),
            # Integral stop tab, no separate spring or fastener.
            Point(py, pz - 18.0).buffer(2.0, resolution=24),
            LineString([(py, pz - 4.0), (py, pz - 18.0)]).buffer(1.9),
        ]
    )
    return profile


def make_pawl():
    pawl = extrude_yz(
        pawl_profile(), P["pawl_axial_thickness"], P["pawl_axial_x0"]
    )
    bore = cylinder_x(
        1.50 + P["pawl_pivot_clearance"] / 2,
        P["pawl_axial_thickness"] + 1.0,
        (
            P["pawl_axial_x0"] + P["pawl_axial_thickness"] / 2,
            P["pawl_pivot_y"],
            P["pawl_pivot_z"],
        ),
        sections=40,
    )
    return cut(pawl, [bore])


# ---------------------------------------------------------------------------
# Motion, state assets and hardware
# ---------------------------------------------------------------------------
def rotate_about_rotor(mesh, angle_deg):
    return rotate(
        mesh,
        angle_deg,
        [1, 0, 0],
        (0.0, P["rotor_center_y"], P["rotor_center_z"]),
    )


def rotate_about_pawl(mesh, angle_deg):
    return rotate(
        mesh,
        angle_deg,
        [1, 0, 0],
        (0.0, P["pawl_pivot_y"], P["pawl_pivot_z"]),
    )


def pawl_angle_for_rotor(angle_deg):
    if angle_deg <= 0.01 or angle_deg >= 89.99:
        return 0.0
    # The servo lifts before gravity motion begins and stays lifted until the
    # next lobe is aligned.  The pawl then drops rapidly into the index stop.
    return -P["pawl_lift_deg"]


def horn_angle_for_rotor(angle_deg):
    fraction = abs(pawl_angle_for_rotor(angle_deg)) / P["pawl_lift_deg"]
    return P["servo_home_horn_deg"] + fraction * (
        P["servo_unlock_horn_deg"] - P["servo_home_horn_deg"]
    )


def pocket_center(angle_deg, home_angle=180.0):
    theta = math.radians(home_angle + angle_deg)
    return (
        P["can_axis_x"],
        P["rotor_center_y"] + P["pocket_center_radius"] * math.cos(theta),
        P["rotor_center_z"] + P["pocket_center_radius"] * math.sin(theta),
    )


def queue_center(index=1):
    home = np.asarray(pocket_center(0.0), dtype=float)
    pitch = P["can_diameter_envelope"] * index
    slope = math.radians(P["lower_slope_deg"])
    return tuple(home + np.asarray([0.0, -pitch * math.cos(slope), pitch * math.sin(slope)]))


def axis_reference():
    cy = P["rotor_center_y"]
    cz = P["rotor_center_z"]
    # 75 mm is measured under the head; the shaft starts exactly at the
    # recessed head face and reaches the right heat-set insert.
    shaft = cylinder_x(2.0, 75.0, (-0.95, cy, cz), sections=36)
    head = cylinder_x(3.9, 1.0, (P["left_frame_outer_x"] + 0.5, cy, cz), sections=44)
    return fuse([shaft, head])


def clamp_screw_references():
    result = []
    for x in (-18.0, 18.0):
        y = -7.0
        z = exact_sheet_lower(y) - 3.10
        result.append(
            fuse(
                [
                    cylinder_normal(2.0, 13.0, (x, y, z), P["lower_slope_deg"], sections=28),
                    cylinder_normal(3.7, 3.0, (x, y, z - 5.5), P["lower_slope_deg"], sections=32),
                ]
            )
        )
    return result


def state_asset_meshes(frame_left, frame_right, rotor_home, pawl_home, angle_deg):
    servo = servo_reference_parts(horn_angle_for_rotor(angle_deg))
    wires = servo_wire_reference()
    dividers = adjacent_divider_references()
    screws = clamp_screw_references()
    return {
        "frame_left.stl": frame_left,
        "frame_right.stl": frame_right,
        "rotor_star.stl": rotate_about_rotor(rotor_home, angle_deg),
        "gravity_pawl.stl": rotate_about_pawl(pawl_home, pawl_angle_for_rotor(angle_deg)),
        "servo_body.stl": servo["servo_body"],
        "servo_shaft.stl": servo["servo_shaft"],
        "servo_horn.stl": servo["servo_horn"],
        "servo_center_screw.stl": servo["servo_center_screw"],
        "fixed_M4_axis.stl": axis_reference(),
        "clamp_screw_left.stl": screws[0],
        "clamp_screw_right.stl": screws[1],
        "lower_sheet_A02.stl": cassette_reference(),
        "divider_left.stl": dividers["divider_left"],
        "divider_right.stl": dividers["divider_right"],
        "can_indexed.stl": can_mesh(pocket_center(angle_deg)),
        "can_queue_1.stl": can_mesh(queue_center(1)),
        "can_queue_2.stl": can_mesh(queue_center(2)),
        "wire_brown.stl": wires["brown"],
        "wire_red.stl": wires["red"],
        "wire_orange.stl": wires["orange"],
    }


def assembly_state(frame_left, frame_right, rotor, pawl, angle_deg):
    return clean(
        trimesh.util.concatenate(
            list(state_asset_meshes(frame_left, frame_right, rotor, pawl, angle_deg).values())
        )
    )


# ---------------------------------------------------------------------------
# Print orientations and 3MF plates
# ---------------------------------------------------------------------------
def make_pivot_coupon():
    base = bx((100.0, 22.0, 4.0), (0.0, 0.0, 2.0))
    cutters = []
    for idx, (x, clearance) in enumerate(zip((-32.0, 0.0, 32.0), (0.20, 0.35, 0.50)), start=1):
        hole = trimesh.creation.cylinder(radius=(3.0 + clearance) / 2, height=6.0, sections=36)
        hole.apply_translation([x, 0.0, 3.0])
        cutters.append(hole)
        for dot in range(idx):
            witness = trimesh.creation.cylinder(radius=0.9, height=6.0, sections=20)
            witness.apply_translation([x + (dot - (idx - 1) / 2) * 3.0, -8.5, 3.0])
            cutters.append(witness)
    return on_bed(cut(base, cutters))


def prepare_print_parts(
    main_left,
    main_right,
    guide_left,
    guide_right,
    inlet,
    outlet,
    rotor_body,
    rotor_indexer,
    pawl,
):
    inlet_print = inlet.copy()
    inlet_transform = np.eye(4)
    # Old Y becomes print Z: the open mouth faces upward and no roof is bridged.
    inlet_transform[:3, :3] = np.array(
        [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]]
    )
    inlet_print.apply_transform(inlet_transform)
    return {
        "B01A_ротор_4_кармана_открытый.stl": on_bed(rotate(rotor_body, -90, [0, 1, 0])),
        "B01B_индексатор_3_ключа.stl": on_bed(rotate(rotor_indexer, -90, [0, 1, 0])),
        "B02L_левая_щека_серва.stl": on_bed(rotate(main_left, -90, [0, 1, 0])),
        "B02R_правая_щека_ось.stl": on_bed(rotate(main_right, 90, [0, 1, 0])),
        "B02GL_левая_направляющая.stl": on_bed(rotate(guide_left, -90, [0, 1, 0])),
        "B02GR_правая_направляющая.stl": on_bed(rotate(guide_right, 90, [0, 1, 0])),
        "B02I_входной_зажим_A02.stl": on_bed(inlet_print),
        "B02O_выходной_лоток.stl": on_bed(
            rotate(rotate(outlet, 12.0, [1, 0, 0]), 90.0, [0, 0, 1])
        ),
        "B03_гравитационная_собачка.stl": on_bed(rotate(pawl, -90, [0, 1, 0])),
        "T01_coupon_ось_собачки_020_035_050.stl": make_pivot_coupon(),
    }


def placed_on_plate(mesh, center_xy):
    result = mesh.copy()
    current = (result.bounds[0, :2] + result.bounds[1, :2]) / 2
    result.apply_translation([center_xy[0] - current[0], center_xy[1] - current[1], 0.0])
    return result


def export_3mf_plate(filename, entries):
    scene = trimesh.Scene()
    placed = []
    for name, mesh, center in entries:
        item = placed_on_plate(mesh, center)
        scene.add_geometry(item, geom_name=name, node_name=name)
        placed.append((name, item))
    data = scene.export(file_type="3mf")
    path = PLATE_DIR / filename
    path.write_bytes(data)
    bounds = np.stack([mesh.bounds for _, mesh in placed], axis=0)
    plate_min = bounds[:, 0, :].min(axis=0)
    plate_max = bounds[:, 1, :].max(axis=0)
    pair_overlaps = []
    minimum_aabb_gap = float("inf")
    for i, (name_a, a) in enumerate(placed):
        for name_b, b in placed[i + 1 :]:
            dx = max(float(b.bounds[0, 0] - a.bounds[1, 0]), float(a.bounds[0, 0] - b.bounds[1, 0]), 0.0)
            dy = max(float(b.bounds[0, 1] - a.bounds[1, 1]), float(a.bounds[0, 1] - b.bounds[1, 1]), 0.0)
            gap = math.hypot(dx, dy)
            minimum_aabb_gap = min(minimum_aabb_gap, gap)
            if dx == 0.0 and dy == 0.0:
                pair_overlaps.append(f"{name_a} / {name_b}")
    with zipfile.ZipFile(path, "r") as archive:
        package_ok = "3D/3dmodel.model" in archive.namelist()
    reloaded = trimesh.load(path, force="scene", process=True)
    reloaded_meshes = list(reloaded.dump()) if isinstance(reloaded, trimesh.Scene) else [reloaded]
    reloaded_pair_intersections = []
    for i, a in enumerate(reloaded_meshes):
        for j, b in enumerate(reloaded_meshes[i + 1 :], start=i + 1):
            value = intersection_volume(a, b)
            if value is None or value > 0.01:
                reloaded_pair_intersections.append(
                    {"objects": [i, j], "volume_mm3": None if value is None else round(value, 5)}
                )
    return {
        "file": path.name,
        "objects": [name for name, _ in placed],
        "bounds_xy_mm": [
            round(float(plate_min[0]), 2),
            round(float(plate_min[1]), 2),
            round(float(plate_max[0]), 2),
            round(float(plate_max[1]), 2),
        ],
        "minimum_bed_edge_margin_mm": round(
            min(
                float(plate_min[0]),
                float(plate_min[1]),
                P["bed_x"] - float(plate_max[0]),
                P["bed_y"] - float(plate_max[1]),
            ),
            2,
        ),
        "fits_220x220": bool(
            plate_min[0] >= -0.01
            and plate_min[1] >= -0.01
            and plate_max[0] <= P["bed_x"] + 0.01
            and plate_max[1] <= P["bed_y"] + 0.01
        ),
        "pair_aabb_overlaps": pair_overlaps,
        "minimum_aabb_gap_mm": None if len(placed) < 2 else round(minimum_aabb_gap, 2),
        "valid_3mf_package": package_ok,
        "reloaded_object_count": len(reloaded_meshes),
        "reloaded_pair_intersections": reloaded_pair_intersections,
    }


def make_3mf_plates(print_parts):
    return [
        export_3mf_plate(
            "B11_plate_01_ротор_собачка_coupon.3mf",
            [
                ("B01_rotor", print_parts["B01A_ротор_4_кармана_открытый.stl"], (66.0, 60.0)),
                ("B01_indexer", print_parts["B01B_индексатор_3_ключа.stl"], (170.0, 40.0)),
                ("B03_pawl", print_parts["B03_гравитационная_собачка.stl"], (170.0, 120.0)),
                ("T01_coupon", print_parts["T01_coupon_ось_собачки_020_035_050.stl"], (110.0, 195.0)),
            ],
        ),
        export_3mf_plate(
            "B11_plate_02_левая_рама.3mf",
            [
                ("B02I_inlet_clamp", print_parts["B02I_входной_зажим_A02.stl"], (110.0, 24.0)),
                ("B02L_left_cheek", print_parts["B02L_левая_щека_серва.stl"], (110.0, 132.0)),
            ],
        ),
        export_3mf_plate(
            "B11_plate_03_правая_рама.3mf",
            [("B02R_right_cheek", print_parts["B02R_правая_щека_ось.stl"], (110.0, 110.0))],
        ),
        export_3mf_plate(
            "B11_plate_04_левая_направляющая_лоток.3mf",
            [
                ("B02O_outlet", print_parts["B02O_выходной_лоток.stl"], (110.0, 52.0)),
                ("B02GL_left_guide", print_parts["B02GL_левая_направляющая.stl"], (110.0, 158.0)),
            ],
        ),
        export_3mf_plate(
            "B11_plate_05_правая_направляющая.3mf",
            [
                ("B02GR_right_guide", print_parts["B02GR_правая_направляющая.stl"], (110.0, 110.0)),
            ],
        ),
    ]


# ---------------------------------------------------------------------------
# Engineering checks
# ---------------------------------------------------------------------------
def analytical_checks():
    wall = P["rotor_end_spider_thickness"]
    rear_rotor_min_x = (
        P["rotor_left_inner_x"] - wall - P["index_axial_thickness"] + 0.1
    )
    rear_rotor_max_x = P["rotor_right_inner_x"] + wall
    rear_rotor_width = rear_rotor_max_x - rear_rotor_min_x
    return {
        "can_axial_clearance_total_mm": round(P["rotor_axial_clear"] - P["can_length"], 3),
        "can_radial_clearance_at_rails_mm": round(
            P["pocket_cut_radius"] + 1.2 - 1.6 - P["can_diameter_envelope"] / 2,
            3,
        ),
        "module_to_pitch_total_mm": round(P["lane_pitch"] - P["module_envelope_width"], 3),
        "rear_rotor_width_inside_75mm_lane_mm": round(rear_rotor_width, 3),
        "rear_rotor_to_divider_faces_total_mm": round(
            P["lane_clear"] - rear_rotor_width, 3
        ),
        "parts_wider_than_75_location": (
            "only local y > 39.732 mm, 0.25 mm ahead of the exact divider nose "
            "at local y=39.482 mm"
        ),
        "rotation_per_release_deg": 90.0,
        "attachment_points": 2,
        "roadway_drilling_required": False,
        "local_acceleration_lip_extra_drop_mm": round(
            inlet_top(13.0) - local_feed_top(13.0), 3
        ),
    }


def actuator_contact_audit():
    """Quasi-static stock-horn/pawl sweep in the actual YZ profiles.

    Gravity keeps B03 against its HOME stop until the horn reaches it.  After
    contact, for each one-degree horn position we select the first clockwise
    pawl angle with no polygon overlap.  A monotonic branch ending at the
    -7.1 degree hard stop proves the horn pushes rather than pulls the pawl.
    """
    py = P["pawl_pivot_y"]
    pz = P["pawl_pivot_z"]
    base = pawl_profile()
    rows = []
    for horn_deg in np.linspace(P["servo_home_horn_deg"], P["servo_unlock_horn_deg"], 29):
        horn = affinity.translate(
            affinity.rotate(horn_outline(), float(horn_deg), origin=(0.0, 0.0)),
            P["servo_axis_y"],
            P["servo_axis_z"],
        )
        selected = None
        selected_gap = None
        for pawl_deg in np.linspace(0.0, -P["pawl_lift_deg"], 711):
            candidate = affinity.rotate(base, float(pawl_deg), origin=(py, pz))
            if candidate.intersection(horn).area <= 1e-5:
                selected = float(pawl_deg)
                selected_gap = float(candidate.distance(horn))
                break
        rows.append(
            {
                "horn_deg": round(float(horn_deg), 3),
                "pawl_deg": None if selected is None else round(selected, 3),
                "surface_gap_mm": None if selected_gap is None else round(selected_gap, 4),
            }
        )
    valid = all(row["pawl_deg"] is not None for row in rows)
    monotonic = valid and all(
        rows[index]["pawl_deg"] <= rows[index - 1]["pawl_deg"] + 0.011
        for index in range(1, len(rows))
    )
    reaches_stop = valid and rows[-1]["pawl_deg"] <= -P["pawl_lift_deg"] + 0.011
    return {
        "stock_double_horn": True,
        "servo_horn_travel_deg": round(
            P["servo_unlock_horn_deg"] - P["servo_home_horn_deg"], 3
        ),
        "HOME_surface_clearance_mm": rows[0]["surface_gap_mm"],
        "UNLOCK_surface_clearance_mm": rows[-1]["surface_gap_mm"],
        "pawl_target_deg": -P["pawl_lift_deg"],
        "monotonic_push_branch": bool(monotonic),
        "reaches_printed_hard_stop": bool(reaches_stop),
        "samples": rows,
    }


def common_reference_manifest():
    result = {}
    for key, filename in (
        ("cassette_lower_sheet", "cassette_lower_sheet.stl"),
        ("divider", "cassette_divider.stl"),
    ):
        path = COMMON_ASSEMBLY_DIR / filename
        source = _load_common(filename)
        local = _common_to_local(source)
        result[key] = {
            "source": "../" + path.relative_to(HERE.parent).as_posix(),
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "source_bounds_mm": np.round(source.bounds, 5).tolist(),
            "local_bounds_mm": np.round(local.bounds, 5).tolist(),
        }
    result["coordinate_transform"] = (
        "local X=global Y-197.5; local Y=39.48236084-global X; local Z=global Z"
    )
    return result


def collision_sweep(frame_left, frame_right, rotor, pawl):
    dividers = divider_references()
    divider_pair = clean(trimesh.util.concatenate(list(dividers.values())))
    lower_sheet = cassette_reference()
    fixed_printed = clean(trimesh.util.concatenate([frame_left, frame_right]))
    fixed_servo = servo_reference_parts()["servo_body"]
    static_divider = intersection_volume(fixed_printed, divider_pair)
    static_sheet = intersection_volume(fixed_printed, lower_sheet)
    queue_1 = can_mesh(queue_center(1))
    queue_2 = can_mesh(queue_center(2))
    queue_1_sheet = intersection_volume(queue_1, lower_sheet)
    queue_2_sheet = intersection_volume(queue_2, lower_sheet)
    queue_1_frame = intersection_volume(queue_1, fixed_printed)
    queue_2_frame = intersection_volume(queue_2, fixed_printed)
    rows = []
    for angle in (0.0, 2.5, 5.0, 15.0, 30.0, 45.0, 60.0, 75.0, 85.0, 87.5, 90.0):
        moving_rotor = rotate_about_rotor(rotor, angle)
        moving_pawl = rotate_about_pawl(pawl, pawl_angle_for_rotor(angle))
        horn = servo_reference_parts(horn_angle_for_rotor(angle))["servo_horn"]
        local_functional = clean(
            trimesh.util.concatenate(
                [frame_left, frame_right, moving_rotor, moving_pawl, fixed_servo, horn]
            )
        )
        five_modules = [translate(local_functional, (offset, 0.0, 0.0)) for offset in (-158.0, -79.0, 0.0, 79.0, 158.0)]
        divider_total = 0.0
        divider_failed = False
        for module in five_modules:
            value = intersection_volume(module, divider_pair)
            if value is None:
                divider_failed = True
                break
            divider_total += value
        neighbor_total = 0.0
        neighbor_failed = False
        for module_a, module_b in zip(five_modules, five_modules[1:]):
            value = intersection_volume(module_a, module_b)
            if value is None:
                neighbor_failed = True
                break
            neighbor_total += value
        frame_left_v = intersection_volume(moving_rotor, frame_left)
        frame_right_v = intersection_volume(moving_rotor, frame_right)
        frame_v = None if frame_left_v is None or frame_right_v is None else frame_left_v + frame_right_v
        rows.append(
            {
                "angle_deg": angle,
                "pawl_deg": round(pawl_angle_for_rotor(angle), 3),
                "rotor_vs_frame_mm3": None if frame_v is None else round(frame_v, 5),
                "rotor_vs_servo_mm3": _rounded_intersection(moving_rotor, fixed_servo),
                "rotor_vs_pawl_mm3": _rounded_intersection(moving_rotor, moving_pawl),
                "rotor_vs_horn_mm3": _rounded_intersection(moving_rotor, horn),
                "rotor_vs_indexed_can_mm3": _rounded_intersection(
                    moving_rotor, can_mesh(pocket_center(angle))
                ),
                "rotor_vs_dividers_mm3": _rounded_intersection(moving_rotor, divider_pair),
                "rotor_vs_exact_lower_sheet_mm3": _rounded_intersection(moving_rotor, lower_sheet),
                "frame_vs_dividers_mm3": None if static_divider is None else round(static_divider, 5),
                "frame_vs_exact_lower_sheet_mm3": None if static_sheet is None else round(static_sheet, 5),
                "queue_can_1_vs_exact_lower_sheet_mm3": None if queue_1_sheet is None else round(queue_1_sheet, 5),
                "queue_can_2_vs_exact_lower_sheet_mm3": None if queue_2_sheet is None else round(queue_2_sheet, 5),
                "queue_can_1_vs_frame_mm3": None if queue_1_frame is None else round(queue_1_frame, 5),
                "queue_can_2_vs_frame_mm3": None if queue_2_frame is None else round(queue_2_frame, 5),
                "indexed_can_vs_fixed_printed_mm3": _rounded_intersection(
                    can_mesh(pocket_center(angle)), fixed_printed
                ),
                "all5_modules_vs_exact_dividers_mm3": (
                    None if divider_failed else round(divider_total, 5)
                ),
                "all5_neighbor_module_pairs_mm3": (
                    None if neighbor_failed else round(neighbor_total, 5)
                ),
            }
        )
    return rows


def _rounded_intersection(a, b):
    value = intersection_volume(a, b)
    return None if value is None else round(value, 5)


def force_torque_audit(rotor, pawl):
    g = 9.81
    kgf_cm_nm = 0.0980665
    petg_density = 1.27
    rotor_mass = abs(float(rotor.volume)) * petg_density * 1e-6
    rotor_com = np.asarray(rotor.center_mass, dtype=float)
    rotor_ecc = math.hypot(
        rotor_com[1] - P["rotor_center_y"], rotor_com[2] - P["rotor_center_z"]
    ) / 1000.0
    rotor_imbalance = rotor_mass * g * rotor_ecc
    pawl_mass = abs(float(pawl.volume)) * petg_density * 1e-6
    pawl_com = np.asarray(pawl.center_mass, dtype=float)
    pawl_gravity_moment = abs(
        (pawl_com[1] - P["pawl_pivot_y"]) / 1000.0 * pawl_mass * g
    )

    queue_count = 22
    can_mass = 0.200
    slope = math.radians(P["lower_slope_deg"])
    queue_downslope = queue_count * can_mass * g * math.sin(slope)
    feed_radius = P["pocket_center_radius"] / 1000.0
    queue_reaction_torque = queue_downslope * feed_radius * math.sin(slope)
    captured_can_torque = can_mass * g * feed_radius
    locked_torque = queue_reaction_torque + captured_can_torque + rotor_imbalance
    index_force = locked_torque / (P["index_radius"] / 1000.0)
    assumed_contact_mu = 0.18
    release_friction = index_force * assumed_contact_mu
    tip = np.asarray(
        [
            P["rotor_center_y"] - P["index_lug_radius"],
            P["rotor_center_z"] + P["index_radius"],
        ]
    )
    pivot = np.asarray([P["pawl_pivot_y"], P["pawl_pivot_z"]])
    tail_contact = np.asarray([135.5319, 22.9688 + P["mechanism_z_shift"]])
    tip_lever = float(np.linalg.norm(tip - pivot)) / 1000.0
    tail_lever = float(np.linalg.norm(tail_contact - pivot)) / 1000.0
    pawl_release_moment = release_friction * tip_lever + pawl_gravity_moment
    servo_contact_force = pawl_release_moment / tail_lever
    effective_horn_radius = 0.0140
    servo_torque = servo_contact_force * effective_horn_radius * 1.25

    # Last-can acceptance.  The integrated steeper lip leaves the indexed can
    # supported by the star; use a conservative 38 mm effective gravity arm.
    bearing_breakaway_acceptance = 0.006
    effective_last_can_arm = 0.038
    last_can_safety_factor = 2.0
    minimum_mass = (
        last_can_safety_factor * (bearing_breakaway_acceptance + rotor_imbalance)
        / (g * effective_last_can_arm)
    )
    cases = []
    for mass_g in (100, 200):
        drive = mass_g / 1000.0 * g * effective_last_can_arm
        resistance = bearing_breakaway_acceptance + rotor_imbalance
        cases.append(
            {
                "can_mass_g": mass_g,
                "gravity_drive_torque_Nm": round(drive, 5),
                "accepted_resistance_Nm": round(resistance, 5),
                "static_margin_ratio": round(drive / resistance, 2),
                "margin_after_2x_requirement": round(drive / (2 * resistance), 2),
                "pass": bool(drive >= 2 * resistance),
            }
        )

    audit = {
        "locked_target_22x200g": {
            "queue_downslope_force_N": round(queue_downslope, 4),
            "queue_reaction_torque_Nm": round(queue_reaction_torque, 5),
            "captured_can_torque_Nm": round(captured_can_torque, 5),
            "rotor_imbalance_torque_Nm": round(rotor_imbalance, 6),
            "total_reaction_at_pawl_Nm": round(locked_torque, 5),
            "index_contact_force_N": round(index_force, 3),
            "assumed_dry_PETG_contact_mu": assumed_contact_mu,
            "calculated_servo_unlock_torque_Nm": round(servo_torque, 5),
            "target_servo_unlock_torque_Nm": 0.060,
            "pass": bool(servo_torque < 0.060),
            "holding_load_on_servo": False,
        },
        "rotor": {
            "estimated_PETG_mass_g": round(rotor_mass * 1000.0, 2),
            "centre_of_mass_yz_mm": [round(float(rotor_com[1]), 4), round(float(rotor_com[2]), 4)],
            "eccentricity_mm": round(rotor_ecc * 1000.0, 4),
        },
        "pawl": {
            "estimated_PETG_mass_g": round(pawl_mass * 1000.0, 2),
            "gravity_return_moment_Nm": round(pawl_gravity_moment, 6),
            "centre_of_mass_is_locking_side": bool(pawl_com[1] < P["pawl_pivot_y"]),
        },
        "last_can_breakaway": {
            "accepted_measured_bearing_breakaway_Nm_max": bearing_breakaway_acceptance,
            "equivalent_tangential_force_at_42mm_N_max": round(
                bearing_breakaway_acceptance / feed_radius, 3
            ),
            "effective_gravity_arm_mm": effective_last_can_arm * 1000.0,
            "required_static_safety_factor": last_can_safety_factor,
            "calculated_minimum_can_mass_g": round(minimum_mass * 1000.0, 1),
            "cases": cases,
            "acceptance_test": (
                "Lift pawl, disconnect servo power, and measure empty-rotor breakaway at 42 mm: "
                "<=0.143 N tangential (<=0.006 Nm). Then the lightest real can must complete "
                "20 of 20 gravity indexes from HOME without help."
            ),
        },
        "power_and_fail_safe": {
            "indexed_HOME": (
                "nominal CAD load path is mechanical and does not use servo torque; "
                "PWM detach is permitted only after the physical power-cut, contamination, "
                "and loaded-dwell acceptance tests pass"
            ),
            "power_loss": (
                "gravity biases the pawl toward the next index lobe, but reliable re-engagement "
                "through a mid-cycle power cut is not proven by CAD and must be bench-tested"
            ),
            "servo_supply": "external regulated 5 V, common GND with ESP32",
        },
    }

    with (HERE / "FORCE_AUDIT_V11_B.csv").open("w", encoding="utf-8-sig", newline="") as fh:
        writer = csv.writer(fh, delimiter=";")
        writer.writerow(["Проверка", "Значение", "Лимит", "Результат"])
        writer.writerow(
            [
                "Разблокировка собачки, 22×200 г",
                f"{servo_torque:.5f} Н·м",
                "<0.060 Н·м",
                "PASS" if servo_torque < 0.060 else "FAIL",
            ]
        )
        writer.writerow(
            [
                "Страгивание пустого ротора",
                f"измерить <= {bearing_breakaway_acceptance:.3f} Н·м",
                "<=0.006 Н·м",
                "ОБЯЗАТЕЛЬНЫЙ ФИЗИЧЕСКИЙ ТЕСТ",
            ]
        )
        writer.writerow(
            [
                "Минимальная расчётная масса последней банки",
                f"{minimum_mass * 1000.0:.1f} г",
                "легче фактической",
                "PASS для 100/200 г" if minimum_mass <= 0.100 else "FAIL",
            ]
        )
        for row in cases:
            writer.writerow(
                [
                    f"Последняя банка {row['can_mass_g']} г",
                    f"запас {row['margin_after_2x_requirement']:.2f} после 2×",
                    ">=1.00",
                    "PASS" if row["pass"] else "FAIL",
                ]
            )
    return audit


# ---------------------------------------------------------------------------
# BOM and instructions
# ---------------------------------------------------------------------------
def write_bom():
    rows = [
        ("B01A", "Открытый ротор-звёздочка: 4 полноразмерных кармана", "PETG", 1, 5, "Один торец на столе, supports OFF"),
        ("B01B", "Плоский индексатор с 4 упорами и 3 ключами", "PETG", 1, 5, "Прижимается к B01A без клея"),
        ("B02L", "Левая плоская щека: серва, ось собачки, hard stops", "PETG", 1, 5, "Большой плоской гранью на стол"),
        ("B02R", "Правая плоская щека с втулкой M4", "PETG", 1, 5, "Большой плоской гранью на стол"),
        ("B02GL", "Левая внутренняя направляющая с штифтом лотка", "PETG", 1, 5, "Плашмя, supports OFF"),
        ("B02GR", "Правая внутренняя направляющая с штифтом лотка", "PETG", 1, 5, "Плашмя, supports OFF"),
        ("B02I", "Общий входной зажим нижнего листа кассеты", "PETG", 1, 5, "Plate 02, brim 4 мм"),
        ("B02O", "Общий выходной лоток", "PETG", 1, 5, "Плашмя, supports OFF"),
        ("B03", "Гравитационная собачка", "PETG", 1, 5, "Печатать плашмя"),
        ("T01", "Разовый coupon Ø3 pivot +0.20/+0.35/+0.50", "PETG", 0.2, 1, "Один на партию филамента"),
        ("HW01", "SG90/MG90S + штатный двойной рожок и винт", "покупное", 1, 5, "Серва только снимает собачку"),
        ("HW02", "M2x8 самонарезающий винт крепления сервы", "сталь", 2, 10, "В штатные уши"),
        ("HW03", "M4x75 ось-винт с гладкой шейкой и низкой головкой", "сталь", 1, 5, "Головка утоплена"),
        ("HW04", "M4 термовставка L=5 мм, наружный Ø5.0–5.3 мм", "латунь", 1, 5, "Карман модели Ø4.70×5.0; сначала проверить одну посадку"),
        ("HW05", "M4x14 винт зажима нижнего листа кассеты снизу", "сталь", 2, 10, "Без сверления листа"),
        ("HW06", "Мягкая накладка 12x12x1", "TPU/резина", 2, 10, "Под зажимные винты"),
    ]
    with (HERE / "BOM_V11_B.csv").open("w", encoding="utf-8-sig", newline="") as fh:
        writer = csv.writer(fh, delimiter=";")
        writer.writerow(["Код", "Наименование", "Материал", "На канал", "На 5", "Примечание"])
        writer.writerows(rows)


def write_readme(audit):
    unlock = audit["locked_target_22x200g"]["calculated_servo_unlock_torque_Nm"]
    minimum = audit["last_can_breakaway"]["calculated_minimum_can_mass_g"]
    text = f"""# V11-B — гравитационно индексируемая звёздочка

## Почему это уже не прежний прямой привод

SG90/MG90S **не должна крутить ротор или держать очередь**. По расчётной кинематике банка и уклон 15° поворачивают четырёхкарманную звёздочку до следующего индекса; ровный шаг 90° считается подтверждённым только после 20/20 индексаций самой лёгкой банкой и полного теста 300 циклов. Серва штатным двойным рожком кратко нажимает на хвост B03 и снимает верхнюю собачку. Затем рожок возвращается, собачка падает своим весом и следующая из четырёх индексных лопастей должна упереться в неё.

В HOME рожок имеет зазор. Силовой путь: банка → B01 → индексная лопасть → B03 → печатная ось и hard stop B02L → рама → два зажима нижнего листа кассеты. В номинальной геометрии вал сервы не является тормозом. **Это ещё не доказанный fail-safe:** снимать PWM разрешается только после физических проверок возврата собачки при отключении питания, загрязнённом шарнире и длительной полной нагрузке.

## Печать Ender-3

- Открывать готовые `PLATES_3MF/*.3mf`: ориентация и разнос уже сохранены.
- Plate 01: B01A + B01B + B03 + разовый T01. Plate 02: B02I + B02L. Plate 03: B02R. Plate 04: B02O + B02GL. Plate 05: B02GR.
- PETG, 0.20 мм, 4 периметра, 5 верх/низ, 25–30% gyroid.
- **Supports OFF на всех пяти плитах.** B01A печатается единственной торцевой стенкой на столе, все лопасти и продольные рёбра растут вверх; противоположный торец открыт. B01B, обе щеки, обе направляющие, B02O и B03 лежат широкими гранями на столе.
- Только для Plate 02 включить brim 4 мм: у открытого C-профиля B02I узкая контактная полоса. Структурных мостов/висящих траекторий в проверке PrusaSlicer нет; предупреждение только о прилипании B02I.
- Детали на Plate 01 разведены с запасом под skirt/brim; генератор проверяет AABB-зазор и стол 220×220.
- T01 имеет отверстия +0.20/+0.35/+0.50 мм, отмеченные 1/2/3 точками. В генераторе выбран +0.35 мм.

## Детали и крепёж

На канал: **9 рабочих печатных деталей** B01A, B01B, B02L, B02R, B02GL, B02GR, B02I, B02O, B03. Coupon печатается один раз, не на каждый канал. Плоская рама собирается без клея: четыре штифта соединяют щёки с направляющими, два штифта позиционируют B02O, а B02I садится на формовые гнёзда обеих направляющих. После M4-оси и двух винтов зажима узел захвачен механически. B01B надевается на три ключа B01A и также захватывается осью.

Крепёж: 2×M2x8 серва, 1×M4x75 ось, 1×M4 термовставка длиной 5 мм и наружным Ø5.0–5.3 мм, 2×M4x14 зажим нижнего листа кассеты и штатный центральный винт рожка. Карман термовставки в модели Ø4.70×5.0 мм: сначала проверить посадку одной фактической вставки, а при другом наружном диаметре скорректировать параметр до печати пяти комплектов. Собачка вращается на печатной оси — отдельного винта и пружины нет.

Суффикс `_A02` в имени B02I означает фактический нижний лист кассеты `cassette_lower_sheet`, а не печатную деталь A02 варианта A.

## 12 сборочных + 2 установочных действия на канал

Итого 14 сгруппированных действий. Одна макрооперация — один осознанный этап с единым результатом. Парные одинаковые элементы, устанавливаемые или затягиваемые в рамках такого этапа (например, два винта сервы или два нижних винта), считаются одной сгруппированной макрооперацией. Обычная очистка печати/осмотр и электропроводка пяти серв в сравнительный счёт не входят.

1. Вплавить одну M4 вставку в B02R.
2. Посадить B02GL на два осевых штифта B02L.
3. Посадить B02GR на два осевых штифта B02R.
4. Посадить левую сторону B02I на формовое гнездо B02GL и надеть B02O на её штифт.
5. Закрепить серву в B02L двумя M2x8.
6. Подать HOME-команду серве.
7. Поставить штатный рожок по метке HOME и затянуть его штатным винтом.
8. Надеть B03 на печатную ось B02L.
9. Совместить три разных по углу ключа и прижать B01B к B01A до торцевого упора; клей не нужен.
10. Положить собранный B01A+B01B на левую стационарную втулку.
11. Закрыть правую половину: ввести B02GR в правое гнездо B02I, её штифт — в B02O, а втулку — в ротор.
12. Вставить и затянуть M4x75 до упора рама-в-раму, не зажимая ротор.
13. Надвинуть B02I на переднюю кромку нижнего листа и отцентрировать между разделителями, не передавая на них вес.
14. Подтянуть два M4 снизу через мягкие накладки; нижний лист кассеты не сверлить.

## Габарит относительно шести листов

Геометрия не перерисована приблизительно: генератор каждый раз загружает фактические `cassette_lower_sheet.stl` и `cassette_divider.stl` из `00_ОБЩАЯ_КОМПОНОВКА`. После перевода в локальные координаты передняя кромка нижнего листа кассеты — `y=0`, а четыре разделителя выступают до `y=+39.482 мм`. В зоне разделителей ротор целиком уже 75-мм канала; более широкие щеки, собачка и серва начинаются только с `y>39.732 мм`, то есть с гарантированным отступом 0.25 мм перед их носом. Sweep всех пяти модулей против всех четырёх реальных разделителей обязан дать 0 мм³.

Штатный двойной рожок идёт от 210° до 238°. Первые ~10° он выбирает свободный зазор, затем непрерывно толкает округлый follower B03 до печатного hard stop −7.1°. Расчёт профилей проверяет монотонную контактную ветвь: в HOME между рожком и B03 больше 1 мм, в UNLOCK они касаются, но не пересекаются.

## Момент и последняя банка

Для худшего режима 22×200 г расчётный момент сервы при снятии нагруженной собачки — **{unlock:.5f} Н·м**, лимит проекта <0.060 Н·м. В запертом состоянии на серве 0 Н·м.

Порог страгивания пустого ротора — не более 0.006 Н·м, то есть не более 0.143 Н касательной силой на радиусе 42 мм. При этом расчётная минимальная масса последней банки с двукратным запасом — **{minimum:.1f} г**. Короткий встроенный 20° носик опускает банку с общего нижнего листа кассеты в карман и снимает её с неподвижной опоры.

Физическая приёмка обязательна: поднять B03, отключить питание сервы, проверить страгивание; затем самая лёгкая настоящая банка должна сама выполнить 20 из 20 поворотов HOME→следующий HOME. Отдельно отключить 5 В в начале, середине и конце разблокировки, повторить с сухим загрязнённым шарниром, а затем оставить полную очередь в HOME минимум на 8 часов и проверить отсутствие ползучести/самовыдачи. Если хотя бы один тест не пройден — PWM не снимать, механизм не считать самофиксирующимся и не увеличивать импульс сервы: убрать перекос оси/заусенцы втулки и повторить тест.

Перед эксплуатацией закрыть ротор и собачку неподвижным защитным тоннелем/кожухом так, чтобы палец не доставал до движущихся деталей ни со стороны выдачи, ни снизу. Кожух не должен поддерживать раму и не должен пересекать траекторию банки.

Сначала печатать один канал и провести 300 циклов. Только после этого печатать остальные четыре.
"""
    (HERE / "README_СБОРКА.md").write_text(text, encoding="utf-8")


def main():
    print("V11-B gravity star: generating nine functional flat-pack parts...")
    obsolete_print_files = (
        "B01_ротор_4_кармана_90deg.stl",
        "B02L_рама_серва_клипса_A02.stl",
        "B02R_рама_ось_клипса_A02.stl",
    )
    for obsolete_name in obsolete_print_files:
        obsolete = PRINT_DIR / obsolete_name
        if obsolete.exists():
            obsolete.unlink()
    obsolete_plate = PLATE_DIR / "B11_plate_04_направляющие_лоток.3mf"
    if obsolete_plate.exists():
        obsolete_plate.unlink()
    rotor_body = make_rotor_body()
    rotor_indexer = make_rotor_indexer()
    rotor = make_rotor()
    main_left, main_right, guide_left, guide_right, inlet, outlet = make_frame_components()
    # Viewer/collision assets retain the familiar two-colour left/right grouping.
    frame_left = clean(trimesh.util.concatenate([main_left, guide_left, inlet, outlet]))
    frame_right = clean(trimesh.util.concatenate([main_right, guide_right]))
    pawl = make_pawl()

    print_parts = prepare_print_parts(
        main_left,
        main_right,
        guide_left,
        guide_right,
        inlet,
        outlet,
        rotor_body,
        rotor_indexer,
        pawl,
    )
    print_reports = []
    for filename, mesh in print_parts.items():
        report = export_mesh(mesh, PRINT_DIR / filename)
        report["fits_220x220"] = bool(
            report["extents_mm"][0] <= P["bed_x"] + 0.01
            and report["extents_mm"][1] <= P["bed_y"] + 0.01
        )
        print_reports.append(report)

    plate_reports = make_3mf_plates(print_parts)
    viewer_style = {
        "frame_left.stl": ("printed_frame", "#d6a45f"),
        "frame_right.stl": ("printed_frame", "#d6a45f"),
        "rotor_star.stl": ("gravity_rotor", "#f28c28"),
        "gravity_pawl.stl": ("fail_safe_pawl", "#ebca55"),
        "servo_body.stl": ("SG90_MG90S", "#276ca8"),
        "servo_shaft.stl": ("metal", "#c5ced8"),
        "servo_horn.stl": ("stock_horn", "#f0f2f5"),
        "servo_center_screw.stl": ("fastener", "#7d8794"),
        "fixed_M4_axis.stl": ("fixed_axis", "#9ba7b4"),
        "clamp_screw_left.stl": ("fastener", "#7d8794"),
        "clamp_screw_right.stl": ("fastener", "#7d8794"),
        "lower_sheet_A02.stl": ("cassette_lower_sheet", "#b98a55"),
        "divider_left.stl": ("real_divider", "#b98a55"),
        "divider_right.stl": ("real_divider", "#b98a55"),
        "can_indexed.stl": ("can", "#42c986"),
        "can_queue_1.stl": ("can", "#42c986"),
        "can_queue_2.stl": ("can", "#42c986"),
        "wire_brown.stl": ("wire_GND", "#6b3f24"),
        "wire_red.stl": ("wire_5V", "#e94242"),
        "wire_orange.stl": ("wire_PWM", "#f29a2e"),
    }
    states = (("HOME", 0.0), ("MID", 45.0), ("EJECT", 90.0))
    state_reports = []
    viewer_assets = {}
    for name, angle in states:
        state_reports.append(
            export_mesh(
                assembly_state(frame_left, frame_right, rotor, pawl, angle),
                ASSEMBLY_DIR / f"assembly_{name}_with_3_cans.stl",
            )
        )
        state_dir = ASSEMBLY_DIR / name
        state_dir.mkdir(parents=True, exist_ok=True)
        viewer_assets[name] = {}
        for filename, mesh in state_asset_meshes(frame_left, frame_right, rotor, pawl, angle).items():
            path = state_dir / filename
            export_mesh(mesh, path)
            role, color = viewer_style[filename]
            asset_key = "cassette_lower_sheet" if filename == "lower_sheet_A02.stl" else filename.removesuffix(".stl")
            viewer_assets[name][asset_key] = {
                "path": path.relative_to(HERE).as_posix(),
                "role": role,
                "color": color,
            }

    collisions = collision_sweep(frame_left, frame_right, rotor, pawl)
    analytical = analytical_checks()
    actuator_audit = actuator_contact_audit()
    force_audit = force_torque_audit(rotor, pawl)
    flat_pack_components = {
        "B02L": main_left,
        "B02R": main_right,
        "B02GL": guide_left,
        "B02GR": guide_right,
        "B02I": inlet,
        "B02O": outlet,
    }
    flat_pack_pair_intersections = []
    flat_pack_items = list(flat_pack_components.items())
    for index, (name_a, mesh_a) in enumerate(flat_pack_items):
        for name_b, mesh_b in flat_pack_items[index + 1 :]:
            volume = intersection_volume(mesh_a, mesh_b)
            flat_pack_pair_intersections.append(
                {
                    "parts": [name_a, name_b],
                    "volume_mm3": None if volume is None else round(float(volume), 6),
                }
            )
    hard_failures = []
    for report in print_reports:
        if not report["watertight"]:
            hard_failures.append(f"not watertight: {report['file']}")
        if report["components"] != 1:
            hard_failures.append(f"multiple components: {report['file']}={report['components']}")
        if not report["fits_220x220"]:
            hard_failures.append(f"does not fit Ender-3 bed: {report['file']}")
        if abs(report["min_z_mm"]) > 0.01:
            hard_failures.append(f"not on bed: {report['file']}")
        if not report["finite"]:
            hard_failures.append(f"non-finite mesh: {report['file']}")
    for report in plate_reports:
        if not report["fits_220x220"]:
            hard_failures.append(f"3MF plate outside bed: {report['file']}")
        if report["minimum_bed_edge_margin_mm"] < 5.0:
            hard_failures.append(
                f"3MF bed-edge margin below 5 mm: {report['file']}={report['minimum_bed_edge_margin_mm']} mm"
            )
        if report["pair_aabb_overlaps"]:
            hard_failures.append(f"3MF parts overlap: {report['file']} {report['pair_aabb_overlaps']}")
        if report["minimum_aabb_gap_mm"] is not None and report["minimum_aabb_gap_mm"] < 5.0:
            hard_failures.append(f"3MF skirt/brim gap below 5 mm: {report['file']}")
        if report["reloaded_pair_intersections"]:
            hard_failures.append(
                f"reloaded 3MF pair collision: {report['file']} {report['reloaded_pair_intersections']}"
            )
        if not report["valid_3mf_package"]:
            hard_failures.append(f"invalid 3MF package: {report['file']}")
    for pair in flat_pack_pair_intersections:
        value = pair["volume_mm3"]
        if value is None:
            hard_failures.append(f"flat-pack intersection boolean failed: {pair['parts']}")
        elif value > 0.01:
            hard_failures.append(
                f"flat-pack parts interfere: {pair['parts']}={value} mm3"
            )
    collision_keys = [
        "rotor_vs_frame_mm3",
        "rotor_vs_servo_mm3",
        "rotor_vs_pawl_mm3",
        "rotor_vs_horn_mm3",
        "rotor_vs_indexed_can_mm3",
        "rotor_vs_dividers_mm3",
        "rotor_vs_exact_lower_sheet_mm3",
        "frame_vs_dividers_mm3",
        "frame_vs_exact_lower_sheet_mm3",
        "queue_can_1_vs_exact_lower_sheet_mm3",
        "queue_can_2_vs_exact_lower_sheet_mm3",
        "queue_can_1_vs_frame_mm3",
        "queue_can_2_vs_frame_mm3",
        "indexed_can_vs_fixed_printed_mm3",
        "all5_modules_vs_exact_dividers_mm3",
        "all5_neighbor_module_pairs_mm3",
    ]
    for row in collisions:
        for key in collision_keys:
            value = row[key]
            if value is None:
                hard_failures.append(f"collision boolean failed: {key} at {row['angle_deg']}°")
            elif value > 0.25:
                hard_failures.append(f"collision {key} at {row['angle_deg']}°={value} mm3")
    if not force_audit["locked_target_22x200g"]["pass"]:
        hard_failures.append("servo pawl-unlock torque exceeds 0.060 Nm")
    if not all(row["pass"] for row in force_audit["last_can_breakaway"]["cases"]):
        hard_failures.append("last-can gravity margin failed")
    if actuator_audit["HOME_surface_clearance_mm"] < 1.0:
        hard_failures.append("stock horn has less than 1 mm HOME clearance")
    if actuator_audit["UNLOCK_surface_clearance_mm"] > 0.02:
        hard_failures.append("stock horn does not reach the B03 follower")
    if not actuator_audit["monotonic_push_branch"]:
        hard_failures.append("stock horn/pawl contact branch is not monotonic")
    if not actuator_audit["reaches_printed_hard_stop"]:
        hard_failures.append("stock horn does not reach the -7.1 degree hard stop")

    manifest = {
        "project": "V11-B gravity-indexed four-pocket star gate",
        "status": "manufacturing prototype; print and endurance-test one lane first",
        "revision": P["revision"],
        "supersedes": "direct-drive rotary cradle moved to REJECTED_DIRECT_DRIVE and excluded",
        "path_base": ".",
        "path_base_definition": "directory containing manifest_V11_B.json",
        "path_roots": {
            "print_reports[].file": "STL_ПЕЧАТЬ",
            "three_mf_plate_reports[].file": "PLATES_3MF",
            "assembly_state_reports[].file": "STL_СБОРКА_НЕ_ПЕЧАТАТЬ",
            "explicit paths with directories": ".",
        },
        "legacy_filename_note": "_A02 in B02I denotes cassette_lower_sheet, not variant-A part A02",
        "parameters_mm": P,
        "mechanism": {
            "energy": "queue and can gravity rotate B01; servo never drives the rotor",
            "index": "four identical stops, 90 degrees per released can",
            "actuator": "one SG90/MG90S lifts B03 directly with stock double horn",
            "holding_load_path": "B01 -> index lug -> B03 -> printed pivot/hard stop -> B02L -> B02GL keyed guide -> B02I -> cassette lower-sheet clamp",
            "HOME": "nominally mechanical in indexed HOME; PWM detach allowed only after physical power-cut, contamination and loaded-dwell tests",
            "hard_stops": "two integral B02L stops limit B03 to 0 and -7.1 degrees; index lugs stop B01 every 90 degrees",
            "last_can_feature": "integral 20 degree acceleration lip transfers the can fully onto the star",
        },
        "main_functional_printed_parts_per_lane": 9,
        "main_functional_printed_parts_for_five_lanes": 45,
        "main_part_codes": [
            "B01A", "B01B", "B02L", "B02R", "B02GL", "B02GR", "B02I", "B02O", "B03"
        ],
        "fasteners_per_lane": {
            "added_threaded_fasteners": 5,
            "stock_horn_centre_screw": 1,
            "heat_set_inserts_M4": 1,
            "pawl_fastener": 0,
            "items": [
                "2 x M2x8 servo screws",
                "1 x M4x75 recessed fixed axis",
                "2 x M4x14 cassette lower-sheet clamp screws",
                "1 x stock horn centre screw",
            ],
            "M4_insert_spec": "length 5 mm, body OD 5.0-5.3 mm; CAD pocket diameter 4.70 mm x 5.0 mm; verify one actual insert before batch printing",
        },
        "supports": {
            "B01A": "none; one datum end wall on bed, blades and rails grow vertically, opposite end open",
            "B01B": "none; index plate flat on bed, three locating keys grow vertically",
            "B02L": "none",
            "B02R": "none",
            "B02GL": "none; broad guide face on bed, inward locator pins grow vertically",
            "B02GR": "none; broad guide face on bed, inward locator pins grow vertically",
            "B02I": "none; open mouth upward, mandatory 4 mm brim for adhesion",
            "B02O": "none; 12 degree running face flat on bed",
            "B03": "none",
            "T01": "none",
        },
        "plate_count": 5,
        "plate_settings": {
            "all": "PETG 0.20 mm, 4 perimeters, 5 top/bottom, supports OFF",
            "B11_plate_02_левая_рама.3mf": "mandatory brim 4 mm; PrusaSlicer warning is adhesion-only for B02I",
        },
        "flat_pack_joint": {
            "adhesive": False,
            "main_to_guide_pin_radius_mm": 1.15,
            "main_to_guide_socket_radius_mm": 1.40,
            "floor_pin_radius_mm": 1.35,
            "floor_socket_radius_mm": 1.60,
            "guide_clearance_mm": 0.22,
            "capture": "M4 rotor axis captures the two cheeks; two cassette clamp screws capture B02I and the keyed guide/floor stack",
            "pair_intersections": flat_pack_pair_intersections,
            "pass_at_0_01_mm3": all(
                pair["volume_mm3"] is not None and pair["volume_mm3"] <= 0.01
                for pair in flat_pack_pair_intersections
            ),
        },
        "exact_manual_actions_per_lane_from_prints_to_mounted": 14,
        "exact_assembly_actions": 12,
        "exact_attachment_actions": 2,
        "action_counting_basis": "one deliberate assembly step with one outcome; matched identical parts or fasteners handled in that step count as one grouped macro-operation; wiring is separate",
        "assembly_steps": [
            "Heat-set one M4 insert in B02R.",
            "Seat B02GL on the two axial locator pins of B02L.",
            "Seat B02GR on the two axial locator pins of B02R.",
            "Seat the left B02I form socket around B02GL and B02O on its outlet pin.",
            "Fasten SG90/MG90S to B02L with two M2x8 screws.",
            "Command servo to HOME.",
            "Fit the stock double horn at the HOME witness and tighten its stock screw.",
            "Slide B03 onto the printed B02L pivot.",
            "Press B01B onto the three keyed pins of B01A; no adhesive is used and the frame captures the joint axially.",
            "Place the assembled B01A+B01B on the left stationary bearing sleeve.",
            "Close the right half while engaging B02GR with the B02I form socket, B02O outlet pin and right rotor sleeve.",
            "Insert M4x75 and tighten frame-to-frame without clamping B01.",
        ],
        "attachment_steps": [
            "Slide B02I over the cassette lower-sheet edge and centre the module between real dividers without loading them.",
            "Tighten the two lower M4 clamp screws through soft pads.",
        ],
        "attachment_points": 2,
        "cassette_lower_sheet_drilling": False,
        "coupon": {
            "file": "STL_ПЕЧАТЬ/T01_coupon_ось_собачки_020_035_050.stl",
            "frequency": "once per filament/printer batch",
            "pivot_clearances_mm": [0.20, 0.35, 0.50],
            "selected_default_mm": P["pawl_pivot_clearance"],
        },
        "print_reports": print_reports,
        "three_mf_plate_reports": plate_reports,
        "assembly_state_reports": state_reports,
        "exact_common_references": common_reference_manifest(),
        "analytical_clearances": analytical,
        "collision_sweep": collisions,
        "stock_horn_actuator_audit": actuator_audit,
        "force_torque_audit": force_audit,
        "viewer_assets": viewer_assets,
        "hard_failures": hard_failures,
        "required_physical_acceptance": [
            "Measure empty-rotor breakaway <=0.006 Nm with B03 lifted and servo unpowered.",
            "Lightest real can must gravity-index 20/20 from HOME without assistance.",
            "Run 20 empty slow cycles and then 300 loaded cycles on one lane before printing four more.",
            "Cut servo 5 V at the start, middle and end of unlock; the pawl must re-engage without releasing an extra can.",
            "Repeat re-engagement with a deliberately dry/dirty pivot and after at least 8 h fully loaded HOME dwell; no creep or self-release is allowed.",
            "Install a fixed finger-exclusion tunnel/guard around the rotor and pawl before operation.",
            "Verify actual SG90/MG90S current and temperature; use external regulated 5 V with common ESP32 GND.",
        ],
    }
    (HERE / "manifest_V11_B.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8"
    )
    write_bom()
    write_readme(force_audit)
    print(json.dumps({"print": print_reports, "plates": plate_reports, "collisions": collisions, "actuator": actuator_audit, "force": force_audit, "failures": hard_failures}, ensure_ascii=False, indent=2, allow_nan=False))
    if hard_failures:
        raise SystemExit("V11-B validation failed: " + "; ".join(hard_failures))
    print("V11-B OK: 9 functional flat-pack parts, 5 valid 3MF plates, zero hard collisions.")


if __name__ == "__main__":
    main()
