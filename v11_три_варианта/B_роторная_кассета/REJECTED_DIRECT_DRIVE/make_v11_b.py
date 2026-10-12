# -*- coding: utf-8 -*-
"""V11-B — compact coaxial rotary cassette, one 70 x Ø47 can per cycle.

This is a parametric, manufacturing-oriented prototype generator.  It creates
three printable parts for one lane, reference assembly states, BOM and a JSON
manifest with mesh, motion-envelope and 220 mm bed checks.

Coordinate system (millimetres):
    X  across the five lanes and along the can/rotor axis
    Y  can travel, positive towards the customer
    Z  vertical, positive up

The SG90/MG90S sits *inside the hollow rotor envelope*.  Its body is stationary,
belly-down, while its stock double horn keys into an internal rotor web.  This
keeps the complete lane inside one 79 mm pitch; no motor or linkage protrudes
into the neighbouring lane.
"""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Iterable

import numpy as np
import trimesh
from shapely import affinity
from shapely.geometry import LineString, Point, Polygon, box as sbox
from shapely.ops import unary_union


HERE = Path(__file__).resolve().parent
PRINT_DIR = HERE / "STL_ПЕЧАТЬ"
ASSEMBLY_DIR = HERE / "STL_СБОРКА_НЕ_ПЕЧАТАТЬ"
for directory in (PRINT_DIR, ASSEMBLY_DIR):
    directory.mkdir(parents=True, exist_ok=True)


# ---------------------------------------------------------------------------
# Master parameters
# ---------------------------------------------------------------------------
P = {
    "revision": "V11-B-PROTOTYPE-R1",
    "lane_count": 5,
    "lane_clear": 75.0,
    "lane_pitch": 79.0,
    "can_length": 70.0,
    "can_diameter_nominal": 47.0,
    "can_diameter_envelope": 48.0,
    "lower_slope_deg": 15.0,
    "sheet_thickness": 4.0,
    "sheet_front_y": -67.0,
    "rotor_center_y": 0.0,
    "rotor_center_z": -75.8,
    "pocket_offset": 42.0,
    "pocket_clear_radius": 24.8,
    "rotor_outer_radius": 69.8,
    "rotor_shell_inner_radius": 66.8,
    "blocking_shell_inner_radius": 64.0,
    "blocking_shell_outer_radius": 67.2,
    "rotor_axial_clear": 71.2,
    "rotor_end_wall": 1.2,
    "rotor_angle_deg": 84.0,
    "frame_inner_x": 37.20,
    "frame_outer_x": 38.65,
    "module_envelope_width": 77.30,
    "guide_inner_radius": 70.8,
    "guide_outer_radius": 76.0,
    "guide_angle_start": 27.0,
    "guide_angle_end": 69.0,
    "servo_body_length": 28.5,
    "servo_body_width": 12.2,
    "servo_body_height": 22.8,
    "servo_axis_x": -31.0,
    "servo_belly_offset_z": -5.4,
    "servo_web_x0": -36.8,
    "servo_web_thickness": 3.4,
    "servo_horn_arm_radius": 15.8,
    "horn_pocket_clearance": 0.35,
    "right_neck_radius": 5.0,
    "right_bearing_clearance": 0.35,
    "edge_clamp_screws": 2,
    "bed_x": 220.0,
    "bed_y": 220.0,
}


# ---------------------------------------------------------------------------
# Mesh helpers
# ---------------------------------------------------------------------------
def clean(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    mesh = mesh.copy()
    mesh.remove_unreferenced_vertices()
    # Manifold booleans already return welded indexed surfaces.  A second
    # tolerance-based merge can incorrectly join the deliberately close
    # blocking shell and structural end ring, creating a non-manifold edge.
    trimesh.repair.fix_normals(mesh)
    return mesh


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
    tools = [clean(c) for c in cutters if c is not None and len(c.faces)]
    if not tools:
        return clean(mesh)
    result = trimesh.boolean.difference([clean(mesh), *tools], engine="manifold")
    if result is None:
        raise RuntimeError("Boolean difference returned no mesh")
    return clean(result)


def intersection_volume(a: trimesh.Trimesh, b: trimesh.Trimesh) -> float:
    if np.asarray(a.bounds).size == 0 or np.asarray(b.bounds).size == 0:
        return 0.0
    try:
        result = trimesh.boolean.intersection([clean(a), clean(b)], engine="manifold")
        if result is None:
            return 0.0
        return abs(float(result.volume))
    except BaseException:
        return None


def bx(size, center=(0.0, 0.0, 0.0)) -> trimesh.Trimesh:
    mesh = trimesh.creation.box(extents=np.asarray(size, dtype=float))
    mesh.apply_translation(np.asarray(center, dtype=float))
    return mesh


def cylinder_x(radius, length, center=(0.0, 0.0, 0.0), sections=48) -> trimesh.Trimesh:
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


def extrude_yz(poly, thickness, x0=0.0) -> trimesh.Trimesh:
    """Extrude a Shapely polygon expressed as (Y, Z) along X."""
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


def extrude_xy(poly, thickness, z0=0.0) -> trimesh.Trimesh:
    """Extrude a Shapely polygon expressed as (X, Y) along build Z."""
    if poly.geom_type in ("MultiPolygon", "GeometryCollection"):
        pieces = [
            extrude_xy(item, thickness, z0)
            for item in poly.geoms
            if item.geom_type == "Polygon" and not item.is_empty and item.area > 1e-6
        ]
        return clean(trimesh.util.concatenate(pieces))
    mesh = trimesh.creation.extrude_polygon(poly, height=thickness)
    mesh.apply_translation([0.0, 0.0, z0])
    return clean(mesh)


def rotate(mesh, angle_deg, axis, point=(0.0, 0.0, 0.0)) -> trimesh.Trimesh:
    result = mesh.copy()
    result.apply_transform(
        trimesh.transformations.rotation_matrix(math.radians(angle_deg), axis, point)
    )
    return clean(result)


def translate(mesh, xyz) -> trimesh.Trimesh:
    result = mesh.copy()
    result.apply_translation(np.asarray(xyz, dtype=float))
    return result


def on_bed(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    result = clean(mesh)
    result.apply_translation([0, 0, -float(result.bounds[0, 2])])
    return result


def rounded_rectangle(width, height, radius):
    return sbox(
        -width / 2 + radius,
        -height / 2 + radius,
        width / 2 - radius,
        height / 2 - radius,
    ).buffer(radius, resolution=18)


def sector_annulus(r_inner, r_outer, angle0, angle1, center=(0.0, 0.0), steps=96):
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


def export_mesh(mesh, path):
    mesh = clean(mesh)
    mesh.export(path)
    ext = mesh.extents
    return {
        "file": path.name,
        "extents_mm": [round(float(v), 2) for v in ext],
        "watertight": bool(mesh.is_watertight),
        "components": int(len(mesh.split(only_watertight=False))),
        "volume_cm3": round(abs(float(mesh.volume)) / 1000.0, 2),
        "min_z_mm": round(float(mesh.bounds[0, 2]), 4),
    }


# ---------------------------------------------------------------------------
# Profiles and references
# ---------------------------------------------------------------------------
def inlet_top(y):
    return -4.8 - (y + 28.0) * math.tan(math.radians(P["lower_slope_deg"]))


def output_top(y):
    return -95.5 - (y - 69.5) * math.tan(math.radians(10.0))


def horn_outline(clearance=0.0):
    hub_r = 3.7 + clearance
    arm_len = P["servo_horn_arm_radius"] + clearance
    root_w = 6.9 + 2 * clearance
    tip_w = 4.0 + 2 * clearance
    hub = Point(0, 0).buffer(hub_r, resolution=24)
    pieces = [hub]
    for sign in (-1, 1):
        pieces.append(
            Polygon(
                [
                    (sign * 3.0, root_w / 2),
                    (sign * arm_len, tip_w / 2),
                    (sign * arm_len, -tip_w / 2),
                    (sign * 3.0, -root_w / 2),
                ]
            )
        )
        pieces.append(Point(sign * arm_len, 0).buffer(tip_w / 2, resolution=16))
    return unary_union(pieces)


def can_mesh(center=(0.0, 0.0, 0.0)):
    x, y, z = center
    r = P["can_diameter_nominal"] / 2
    body = cylinder_x(r, 66.0, (x, y, z), sections=64)
    left_rim = cylinder_x(r + 0.55, 2.0, (x - 34.0, y, z), sections=64)
    right_rim = cylinder_x(r + 0.55, 2.0, (x + 34.0, y, z), sections=64)
    lid = cylinder_x(r - 1.2, 1.0, (x + 35.0, y, z), sections=64)
    return fuse([body, left_rim, right_rim, lid])


def servo_reference_parts():
    """Approximate, dimensioned SG90/MG90S reference in installed coordinates."""
    cy = P["rotor_center_y"]
    cz = P["rotor_center_z"]
    face_x = P["servo_axis_x"]
    # Output shaft points to -X; servo belly is deliberately rotated down.
    body_cx = face_x + 1.2 + P["servo_body_length"] / 2
    body_cz = cz + P["servo_belly_offset_z"]
    body = bx(
        # Rotated 90° around the shaft: the 22.8 mm side lies across Y and the
        # 12.2 mm side stays low under the loaded can.
        (P["servo_body_length"], P["servo_body_height"], P["servo_body_width"]),
        (body_cx, cy, body_cz),
    )
    # Real micro-servo flange is two ears, not a solid 32.5 x 15 rectangle.
    flange = fuse([
        bx((2.4, 24.0, 10.0), (face_x + 0.2, cy, body_cz)),
        cylinder_x(3.2, 2.4, (face_x + 0.2, cy - 14.5, body_cz), sections=28),
        cylinder_x(3.2, 2.4, (face_x + 0.2, cy + 14.5, body_cz), sections=28),
        bx((2.4, 8.0, 5.8), (face_x + 0.2, cy - 12.0, body_cz)),
        bx((2.4, 8.0, 5.8), (face_x + 0.2, cy + 12.0, body_cz)),
    ])
    top_cap = bx((2.8, 23.4, 12.8), (face_x - 0.2, cy, body_cz))
    # The boss stops 0.4 mm before the moving torque web.
    boss = cylinder_x(5.8, 2.2, (face_x - 1.1, cy, cz), sections=40)
    shaft = cylinder_x(2.35, 5.5, (face_x - 4.0, cy, cz), sections=32)

    # Horn lies horizontally at HOME.  It is the real stock double horn.
    horn_poly = horn_outline()
    horn_x0 = face_x - 5.70
    horn = extrude_yz(horn_poly, 1.8, x0=horn_x0)
    horn = translate(horn, (0, cy, cz))
    screw = fuse(
        [
            cylinder_x(1.0, 6.0, (face_x - 3.8, cy, cz), sections=24),
            cylinder_x(2.15, 1.2, (face_x - 6.1, cy, cz), sections=24),
        ]
    )
    return {
        "servo_body": fuse([body, flange, top_cap, boss]),
        "servo_shaft": shaft,
        "servo_horn": horn,
        "servo_center_screw": screw,
    }


def servo_wire_reference():
    """Three simple assembly-only wire solids: brown, red and orange."""
    cy = P["rotor_center_y"]
    cz = P["rotor_center_z"] + P["servo_belly_offset_z"]
    x0 = P["servo_axis_x"] + P["servo_body_length"] + 1.0
    wires = {}
    for name, dz in (("brown", -2.0), ("red", 0.0), ("orange", 2.0)):
        pieces = [
            cylinder_x(0.65, 30.0, (x0 + 15.0, cy + dz, cz - 5.0), sections=16),
            bx((8.0, 5.0, 7.0), (x0 + 33.5, cy + dz, cz - 5.0)),
        ]
        wires[name] = fuse(pieces)
    return wires


def cassette_reference():
    return slope_slab(
        -37.5,
        37.5,
        -205.0,
        P["sheet_front_y"],
        inlet_top(-205.0),
        inlet_top(P["sheet_front_y"]),
        P["sheet_thickness"],
    )


# ---------------------------------------------------------------------------
# Printable rotor
# ---------------------------------------------------------------------------
def blocking_ring_profile():
    # 60° mouth centred straight up at HOME; 300° outside blocking arc.
    return sector_annulus(
        P["rotor_shell_inner_radius"] - 0.35,
        P["rotor_outer_radius"] - 0.35,
        120.0,
        420.0,
        (P["rotor_center_y"], P["rotor_center_z"]),
    )


def cradle_ring_profile():
    py = P["rotor_center_y"]
    pz = P["rotor_center_z"] + P["pocket_offset"]
    # U-shaped local cradle, open towards the inlet.
    return sector_annulus(
        P["pocket_clear_radius"],
        P["pocket_clear_radius"] + 3.2,
        145.0,
        395.0,
        (py, pz),
        steps=80,
    )


def end_ring_profile(include_right_hub=False):
    cy = P["rotor_center_y"]
    cz = P["rotor_center_z"]
    profile = unary_union([blocking_ring_profile(), cradle_ring_profile()])
    # End-web spokes stay outside the can pocket.
    spokes = []
    for angle in (150.0, 240.0, 330.0):
        a = math.radians(angle)
        p0 = (cy + 29.0 * math.cos(a), cz + 29.0 * math.sin(a))
        p1 = (cy + 67.5 * math.cos(a), cz + 67.5 * math.sin(a))
        spokes.append(LineString([p0, p1]).buffer(2.2, cap_style=1))
    # Two tangential bridges join the offset U-cradle to the blocking ring.
    # They run away from the can surface, outside its Ø48 envelope.
    cradle_bridges = []
    pz = cz + P["pocket_offset"]
    for local_angle in (200.0, 340.0):
        a = math.radians(local_angle)
        p0 = (cy + 27.0 * math.cos(a), pz + 27.0 * math.sin(a))
        radial = math.atan2(p0[1] - cz, p0[0] - cy)
        p1 = (cy + 67.5 * math.cos(radial), cz + 67.5 * math.sin(radial))
        cradle_bridges.append(LineString([p0, p1]).buffer(2.2, cap_style=1))
    profile = unary_union([profile, *spokes, *cradle_bridges])
    if include_right_hub:
        hub = Point(cy, cz).buffer(8.0, resolution=36)
        for angle in (150.0, 240.0, 330.0):
            a = math.radians(angle)
            p1 = (cy + 67.0 * math.cos(a), cz + 67.0 * math.sin(a))
            hub = unary_union(
                [hub, LineString([(cy, cz), p1]).buffer(2.4, cap_style=1)]
            )
        profile = unary_union([profile, hub])
    return profile


def torque_web_profile():
    cy = P["rotor_center_y"]
    cz = P["rotor_center_z"]
    # Stock horn remains horizontal at HOME.  Two torque spokes deliberately
    # avoid both the can sector (53..127°) and the fixed servo-support sector.
    horn_wall = horn_outline(2.6)
    horn_wall = affinity.translate(horn_wall, cy, cz)
    spokes = []
    for angle in (150.0, 240.0):
        a = math.radians(angle)
        p0 = (cy + 13.5 * math.cos(a), cz + 13.5 * math.sin(a))
        p1 = (cy + 67.5 * math.cos(a), cz + 67.5 * math.sin(a))
        spokes.append(LineString([p0, p1]).buffer(2.7, cap_style=1))
    # Deliberately minimal: the full end-cradle brace is placed axially beyond
    # the short fixed servo cantilever, so the rotating web cannot sweep into it.
    return unary_union([blocking_ring_profile(), horn_wall, *spokes])


def cradle_brace_profile():
    """One light U-brace joining the three cradle rails to the axial shell."""
    cy = P["rotor_center_y"]
    cz = P["rotor_center_z"]
    pz = cz + P["pocket_offset"]
    a = math.radians(200.0)
    p0 = (cy + 27.0 * math.cos(a), pz + 27.0 * math.sin(a))
    radial = math.atan2(p0[1] - cz, p0[0] - cy)
    p1 = (cy + 67.5 * math.cos(radial), cz + 67.5 * math.sin(radial))
    bridge = LineString([p0, p1]).buffer(2.2, cap_style=1)
    return unary_union([cradle_ring_profile(), bridge])


def axial_blocking_shell_profile():
    """Only the swept inlet-blocking sector is continuous along X.

    This replaces four heavy, support-hungry transverse hoops.  The feed
    contact is near rotor angle 117° at HOME; the 120° leading edge therefore
    closes it after only about 3° of motion.  Its radius lifts the waiting can
    only 1.4 mm instead of forcing it over the larger structural end rings.
    """
    return sector_annulus(
        P["blocking_shell_inner_radius"],
        P["blocking_shell_outer_radius"],
        120.0,
        210.0,
        (P["rotor_center_y"], P["rotor_center_z"]),
        steps=64,
    )


def make_rotor():
    cy = P["rotor_center_y"]
    cz = P["rotor_center_z"]
    clear_half = P["rotor_axial_clear"] / 2
    wall = P["rotor_end_wall"]
    x_left = -clear_half - wall
    x_right_inner = clear_half

    parts = [
        # The left drive web is also the bottom print face of B01.
        extrude_yz(torque_web_profile(), P["servo_web_thickness"], x_left),
        extrude_yz(end_ring_profile(True), wall, x_right_inner),
        # Continuous, thin longitudinal blade blocks the next can without
        # requiring intermediate horizontal bridges during printing.
        extrude_yz(
            axial_blocking_shell_profile(),
            P["rotor_axial_clear"] + 2 * wall - 0.4,
            x_left + 0.2,
        ),
        # Axially clear of the left servo mounting plate; supports all three
        # long cradle rails near their drive end.
        extrude_yz(cradle_brace_profile(), 1.5, -28.5),
    ]

    # Four longitudinal spars join the end webs; the continuous shell replaces
    # the fifth spar in the feed-blocking sector.
    for angle in (30.0, 210.0, 270.0, 330.0):
        a = math.radians(angle)
        y = cy + 68.25 * math.cos(a)
        z = cz + 68.25 * math.sin(a)
        parts.append(
            cylinder_x(
                1.55,
                P["rotor_axial_clear"] + 2 * P["rotor_end_wall"],
                (0.0, y, z),
                sections=24,
            )
        )

    # Three full-length rails locate the can in the offset pocket.
    pz = cz + P["pocket_offset"]
    for angle in (200.0, 270.0, 340.0):
        a = math.radians(angle)
        y = cy + 26.65 * math.cos(a)
        z = pz + 26.65 * math.sin(a)
        parts.append(cylinder_x(1.80, P["rotor_axial_clear"], (0.0, y, z), sections=24))

    # Right printed bearing neck; it stays inside the 79 mm lane pitch.
    neck_x0 = clear_half + wall
    neck_x1 = P["frame_outer_x"] - 0.10
    parts.append(
        cylinder_x(
            P["right_neck_radius"],
            neck_x1 - neck_x0,
            ((neck_x0 + neck_x1) / 2, cy, cz),
            sections=48,
        )
    )

    rotor = fuse(parts)

    # Stock horn pocket is open from the left and has a backing wall.
    pocket_poly = affinity.translate(horn_outline(P["horn_pocket_clearance"]), cy, cz)
    pocket = extrude_yz(pocket_poly, 2.25, x0=x_left - 0.10)
    cutters = [
        pocket,
        cylinder_x(2.25, 8.0, (x_left + 1.7, cy, cz), sections=32),
        # Flush M4 retaining-head recess and pilot in the right neck.
        cylinder_x(2.15, 12.0, (35.8, cy, cz), sections=32),
        cylinder_x(4.15, 2.2, (37.35, cy, cz), sections=40),
    ]
    return cut(rotor, cutters)


# ---------------------------------------------------------------------------
# Printable split frame with no-drill A02 edge clamp
# ---------------------------------------------------------------------------
def frame_side_profile(side):
    cy = P["rotor_center_y"]
    cz = P["rotor_center_z"]
    profile = sector_annulus(
        P["guide_inner_radius"],
        P["guide_outer_radius"],
        P["guide_angle_start"],
        P["guide_angle_end"],
        (cy, cz),
        steps=60,
    )
    # Skeletal side rail follows inlet, outlet and connects them to the guide.
    curves = [
        LineString([(-82.0, inlet_top(-82.0) - 2.0), (-28.0, inlet_top(-28.0) - 2.0)]).buffer(3.0),
        LineString([(69.5, output_top(69.5) - 2.0), (110.0, output_top(110.0) - 2.0)]).buffer(3.0),
        LineString([(-28.0, -8.0), (-10.0, cz + 8.0)]).buffer(3.4),
        LineString([(8.0, cz - 2.0), (69.5, output_top(69.5) - 2.0)]).buffer(3.4),
    ]
    profile = unary_union([profile, *curves])

    # A compact fixed hub makes the whole cheek one structural part.  On the
    # left it is axially clear of the moving end ring; on the right it carries
    # the printed bearing.
    hub = Point(cy, cz).buffer(10.2, resolution=40)
    for angle in (35.0, 62.0, 205.0):
        a = math.radians(angle)
        p = (cy + 72.0 * math.cos(a), cz + 72.0 * math.sin(a))
        hub = unary_union([hub, LineString([(cy, cz), p]).buffer(3.2)])
    profile = unary_union([profile, hub])

    if side == "right":
        pass
    else:
        # Fixed servo cantilever attaches to the left cheek in the clear lower-
        # front sector, never relying on a plywood divider.
        support_points = [(cy + 10.0, cz - 28.0), (cy + 20.0, cz - 22.0)]
        bosses = [Point(*p).buffer(5.0, resolution=24) for p in support_points]
        connectors = [LineString([p, (-18.0, cz - 8.0)]).buffer(3.0) for p in support_points]
        profile = unary_union([profile, *bosses, *connectors])
    return profile


def clamp_jaw(side):
    # 0.30 mm overlap into the outside cheek avoids a merely coplanar union.
    x0, x1 = (-P["frame_inner_x"] - 0.30, 0.0) if side == "left" else (0.0, P["frame_inner_x"] + 0.30)
    angle = P["lower_slope_deg"]
    # Upper lip is also the smooth transition from common A02 into the rotor.
    upper = slope_slab(
        x0,
        x1,
        -82.0,
        -28.0,
        inlet_top(-82.0) + 0.8,
        inlet_top(-28.0) + 0.8,
        2.8,
    )
    # Lower jaw sits below the 4 mm A02 sheet.  One screw in each half pushes
    # upward; the roadway itself is not drilled.
    lower = slope_slab(
        x0,
        x1,
        -82.0,
        -64.5,
        inlet_top(-82.0) - 4.45,
        inlet_top(-64.5) - 4.45,
        4.0,
    )
    # Front stop closes the C-profile at the sheet edge.
    y = -64.5
    bridge_z = (inlet_top(y) - 5.8 + inlet_top(y) + 0.8) / 2
    bridge = bx((x1 - x0, 3.0, 10.0), ((x0 + x1) / 2, y, bridge_z))
    screw_x = -18.0 if side == "left" else 18.0
    screw_y = -73.0
    surface_z = inlet_top(screw_y) - 4.45
    # A 10 mm printed boss gives an M4 screw enough PETG thread engagement;
    # this deliberately removes two heat-set-insert operations per lane.
    thread_boss = cylinder_normal(
        5.0, 8.0, (screw_x, screw_y, surface_z - 4.8), angle, sections=36
    )
    jaw = fuse([upper, lower, bridge, thread_boss])

    # Ø3.3 pilot is tapped once by the M4 clamp screw itself.
    pilot = cylinder_normal(1.65, 14.0, (screw_x, screw_y, surface_z - 2.5), angle)
    return cut(jaw, [pilot])


def output_chute(side):
    x0, x1 = (-P["frame_inner_x"] - 0.30, 0.0) if side == "left" else (0.0, P["frame_inner_x"] + 0.30)
    return slope_slab(
        x0,
        x1,
        69.5,
        110.0,
        output_top(69.5),
        output_top(110.0),
        3.0,
    )


def seam_features(side):
    sites = [
        (-75.0, inlet_top(-75.0) - 1.8),
        (82.0, output_top(82.0) - 1.8),
    ]
    if side == "left":
        return [cylinder_x(1.85, 6.2, (2.0, y, z), sections=28) for y, z in sites], []
    bosses = [cylinder_x(3.9, 8.0, (4.0, y, z), sections=32) for y, z in sites]
    sockets = [cylinder_x(2.10, 9.0, (4.0, y, z), sections=28) for y, z in sites]
    return bosses, sockets


def servo_cantilever_parts():
    cy = P["rotor_center_y"]
    cz = P["rotor_center_z"]
    face_x = P["servo_axis_x"]
    body_cz = cz + P["servo_belly_offset_z"]

    # Two real servo-ear bosses and a lower U-saddle.  Leaving the upper/front
    # quadrant open is what lets the three rotating cradle rails pass freely.
    support_points = [(cy + 10.0, cz - 28.0), (cy + 20.0, cz - 22.0)]
    ear_left = (cy - 14.5, body_cz)
    ear_right = (cy + 14.5, body_cz)
    saddle = (cy, cz - 18.0)
    plate_profile = unary_union([
        Point(*ear_left).buffer(4.0, resolution=20),
        Point(*ear_right).buffer(4.0, resolution=20),
        LineString([ear_left, (cy - 9.0, cz - 17.0), saddle]).buffer(3.0, cap_style=1),
        LineString([ear_right, (cy + 9.0, cz - 17.0), saddle]).buffer(3.0, cap_style=1),
        *[LineString([saddle, point]).buffer(3.2, cap_style=1) for point in support_points],
    ])
    plate = extrude_yz(plate_profile, 2.0, x0=face_x - 0.1)

    beams = []
    x0 = -P["frame_outer_x"] + 1.0
    x1 = face_x
    for y, z in support_points:
        beams.append(cylinder_x(2.6, x1 - x0, ((x0 + x1) / 2, y, z), sections=28))
    cutters = [
        cylinder_x(1.10, 6.0, (face_x, cy - 14.5, body_cz), sections=24),
        cylinder_x(1.10, 6.0, (face_x, cy + 14.5, body_cz), sections=24),
    ]
    plate = cut(plate, cutters)
    # Keep these as separate valid volumes until they are fused through the
    # cheek in make_frame; the two cantilever rails intentionally do not touch
    # each other in free space.
    return [plate, *beams]


def make_frame(side):
    if side not in ("left", "right"):
        raise ValueError(side)
    if side == "left":
        x0 = -P["frame_outer_x"]
    else:
        x0 = P["frame_inner_x"]
    cheek = extrude_yz(frame_side_profile(side), P["frame_outer_x"] - P["frame_inner_x"], x0=x0)

    parts = [cheek, clamp_jaw(side), output_chute(side)]
    pins_or_bosses, socket_cutters = seam_features(side)
    parts.extend(pins_or_bosses)
    if side == "left":
        parts.extend(servo_cantilever_parts())
    frame = fuse(parts)

    cutters = list(socket_cutters)
    if side == "right":
        cutters.append(
            cylinder_x(
                P["right_neck_radius"] + P["right_bearing_clearance"],
                6.0,
                ((P["frame_inner_x"] + P["frame_outer_x"]) / 2, P["rotor_center_y"], P["rotor_center_z"]),
                sections=48,
            )
        )
    return cut(frame, cutters)


# ---------------------------------------------------------------------------
# Assembly motion
# ---------------------------------------------------------------------------
def rotate_about_rotor(mesh, angle_deg):
    return rotate(
        mesh,
        -angle_deg,
        [1, 0, 0],
        (0.0, P["rotor_center_y"], P["rotor_center_z"]),
    )


def pocket_center(angle_deg):
    theta = math.radians(-angle_deg)
    d = P["pocket_offset"]
    y = P["rotor_center_y"] - d * math.sin(theta)
    z = P["rotor_center_z"] + d * math.cos(theta)
    return (0.0, y, z)


def queue_can_center(y):
    r = P["can_diameter_nominal"] / 2
    return (0.0, y, inlet_top(y) + r / math.cos(math.radians(P["lower_slope_deg"])))


def waiting_can_center(angle_deg):
    """Can immediately above the filled cell; the shell raises it only 1.4 mm."""
    # At HOME it rests on the captured can.  Over the first 10 degrees the
    # shell nose takes the load and raises it by a deliberately small amount.
    lift = 1.4 * min(1.0, max(0.0, angle_deg / 10.0))
    return (
        0.0,
        P["rotor_center_y"],
        P["rotor_center_z"]
        + P["pocket_offset"]
        + P["can_diameter_envelope"]
        + lift,
    )


def upstream_can_center(angle_deg):
    waiting = waiting_can_center(angle_deg)
    pitch = P["can_diameter_envelope"]
    theta = math.radians(P["lower_slope_deg"])
    return (
        0.0,
        waiting[1] - pitch * math.cos(theta),
        waiting[2] + pitch * math.sin(theta),
    )


def m4_retainer_reference():
    cy = P["rotor_center_y"]
    cz = P["rotor_center_z"]
    shaft = cylinder_x(2.0, 11.0, (33.0, cy, cz), sections=28)
    head = cylinder_x(4.0, 2.0, (37.45, cy, cz), sections=40)
    return fuse([shaft, head])


def clamp_screw_references():
    angle = P["lower_slope_deg"]
    result = []
    for x in (-18.0, 18.0):
        y = -73.0
        z = inlet_top(y) - 9.2
        shaft = cylinder_normal(2.0, 13.0, (x, y, z), angle, sections=28)
        head = cylinder_normal(3.7, 3.0, (x, y, z - 5.5), angle, sections=32)
        result.append(fuse([shaft, head]))
    return result


def assembly_state(frame_left, frame_right, rotor_home, angle_deg, include_reference=True):
    servo = servo_reference_parts()
    rotor = rotate_about_rotor(rotor_home, angle_deg)
    horn = rotate_about_rotor(servo["servo_horn"], angle_deg)
    screw = rotate_about_rotor(servo["servo_center_screw"], angle_deg)
    meshes = [frame_left, frame_right, rotor, servo["servo_body"], servo["servo_shaft"], horn, screw]
    meshes.extend(
        [
            can_mesh(pocket_center(angle_deg)),
            can_mesh(waiting_can_center(angle_deg)),
            can_mesh(upstream_can_center(angle_deg)),
        ]
    )
    if include_reference:
        meshes.extend([cassette_reference(), m4_retainer_reference(), *clamp_screw_references()])
    return clean(trimesh.util.concatenate(meshes))


def state_asset_meshes(frame_left, frame_right, rotor_home, angle_deg):
    """Separate, colourable assets in exact assembly coordinates for a viewer."""
    servo = servo_reference_parts()
    wires = servo_wire_reference()
    clamp_screws = clamp_screw_references()
    return {
        "frame_left.stl": frame_left,
        "frame_right.stl": frame_right,
        "rotor.stl": rotate_about_rotor(rotor_home, angle_deg),
        "servo_body.stl": servo["servo_body"],
        "servo_shaft.stl": servo["servo_shaft"],
        "servo_horn.stl": rotate_about_rotor(servo["servo_horn"], angle_deg),
        "servo_center_screw.stl": rotate_about_rotor(servo["servo_center_screw"], angle_deg),
        "right_M4_retainer.stl": m4_retainer_reference(),
        "clamp_screw_left.stl": clamp_screws[0],
        "clamp_screw_right.stl": clamp_screws[1],
        "lower_sheet_A02.stl": cassette_reference(),
        "can_captured.stl": can_mesh(pocket_center(angle_deg)),
        "can_queue_1.stl": can_mesh(waiting_can_center(angle_deg)),
        "can_queue_2.stl": can_mesh(upstream_can_center(angle_deg)),
        "wire_brown.stl": wires["brown"],
        "wire_red.stl": wires["red"],
        "wire_orange.stl": wires["orange"],
    }


# ---------------------------------------------------------------------------
# Export, checks and documentation
# ---------------------------------------------------------------------------
def prepare_print_parts(frame_left, frame_right, rotor):
    # Rotor: axis X becomes build Z.  Frames: their outside cheeks lie on bed.
    return {
        "B01_ротор_лёгкая_клетка.stl": on_bed(rotate(rotor, -90, [0, 1, 0])),
        "B02L_полурама_SG90_клипса_A02.stl": on_bed(rotate(frame_left, -90, [0, 1, 0])),
        "B02R_полурама_M4_клипса_A02.stl": on_bed(rotate(frame_right, 90, [0, 1, 0])),
    }


def make_horn_coupon():
    """One small batch coupon: stock-horn pocket clearances +0.20/+0.35/+0.50."""
    base = bx((120.0, 25.0, 4.0), (0.0, 0.0, 2.0))
    cutters = []
    for index, (x, clearance) in enumerate(zip((-40.0, 0.0, 40.0), (0.20, 0.35, 0.50)), start=1):
        pocket = extrude_xy(horn_outline(clearance), 2.35, z0=2.0)
        pocket.apply_translation([x, 0.0, 0.0])
        cutters.append(pocket)
        # One/two/three witness holes identify the clearance after slicing.
        for dot in range(index):
            marker = trimesh.creation.cylinder(radius=1.0, height=5.0, sections=20)
            marker.apply_translation([x + (dot - (index - 1) / 2) * 3.0, -9.5, 2.5])
            cutters.append(marker)
    return on_bed(cut(base, cutters))


def analytical_checks():
    can_r = P["can_diameter_envelope"] / 2
    return {
        "can_radial_clearance_in_pocket_mm": round(P["pocket_clear_radius"] - can_r, 3),
        "can_axial_clearance_total_mm": round(P["rotor_axial_clear"] - P["can_length"], 3),
        "rotor_to_stationary_guide_radial_mm": round(P["guide_inner_radius"] - P["rotor_outer_radius"], 3),
        "module_to_79mm_pitch_total_mm": round(P["lane_pitch"] - P["module_envelope_width"], 3),
        "inlet_window_chord_mm": round(
            2 * P["rotor_outer_radius"] * math.sin(math.radians(30.0)), 3
        ),
        "inlet_window_minus_can_envelope_mm": round(
            2 * P["rotor_outer_radius"] * math.sin(math.radians(30.0))
            - P["can_diameter_envelope"],
            3,
        ),
        "servo_body_to_home_can_vertical_min_mm": round(
            (P["pocket_offset"] - can_r)
            - (P["servo_belly_offset_z"] + P["servo_body_width"] / 2),
            3,
        ),
        "attachment_points_per_lane": 2,
        "roadway_drilling_required": False,
        "rotation_deg": P["rotor_angle_deg"],
    }


def mesh_and_collision_checks(frame_left, frame_right, rotor):
    servo_body = servo_reference_parts()["servo_body"]
    collision_rows = []
    for angle in (0.0, 10.0, 21.0, 32.0, 42.0, 53.0, 63.0, 74.0, 84.0):
        moving = rotate_about_rotor(rotor, angle)
        volume_left = intersection_volume(moving, frame_left)
        volume_right = intersection_volume(moving, frame_right)
        volume_frame = (
            None
            if volume_left is None or volume_right is None
            else volume_left + volume_right
        )
        volume_servo = intersection_volume(moving, servo_body)
        volume_can = intersection_volume(moving, can_mesh(pocket_center(angle)))
        volume_waiting_can = intersection_volume(
            moving, can_mesh(waiting_can_center(angle))
        )
        collision_rows.append(
            {
                "angle_deg": angle,
                "rotor_vs_frame_mm3": None if volume_frame is None else round(volume_frame, 5),
                "rotor_vs_servo_mm3": None if volume_servo is None else round(volume_servo, 5),
                "rotor_vs_can_mm3": None if volume_can is None else round(volume_can, 5),
                "rotor_vs_waiting_can_mm3": (
                    None
                    if volume_waiting_can is None
                    else round(volume_waiting_can, 5)
                ),
            }
        )
    return collision_rows


def force_torque_audit(rotor):
    """Conservative static/dynamic audit for a directly loaded queue.

    The queue is supported by the 15 degree A02 roadway, so only its down-slope
    gravity component reaches the rotor shell.  That force still acts at the
    large shell radius required by the internal coaxial servo packaging.
    """
    g = 9.81
    kgf_cm_nm = 0.0980665
    petg_density_g_cm3 = 1.27
    volume_mm3 = abs(float(rotor.volume))
    rotor_mass_kg = volume_mm3 * petg_density_g_cm3 * 1e-6
    com = np.asarray(rotor.center_mass, dtype=float)
    if not np.all(np.isfinite(com)):
        raise ValueError("Rotor centre of mass is not finite")
    eccentricity_m = math.hypot(
        float(com[1]) - P["rotor_center_y"],
        float(com[2]) - P["rotor_center_z"],
    ) / 1000.0
    rotor_gravity_nm = rotor_mass_kg * g * eccentricity_m
    slope = math.radians(P["lower_slope_deg"])
    captured_lever_m = P["pocket_offset"] / 1000.0
    blocking_radius_m = P["blocking_shell_outer_radius"] / 1000.0
    friction_fraction = 0.12
    bearing_and_seal_floor_nm = 0.010
    dynamic_factor = 1.25

    cases = []
    for queue_count in (11, 22):
        for can_mass_g in (100, 200):
            can_mass_kg = can_mass_g / 1000.0
            queue_force_n = queue_count * can_mass_kg * g * math.sin(slope)
            captured_can_nm = can_mass_kg * g * captured_lever_m
            queue_shell_nm = (
                queue_force_n * blocking_radius_m * math.cos(slope)
            )
            friction_nm = bearing_and_seal_floor_nm + friction_fraction * (
                captured_can_nm + queue_shell_nm
            )
            design_nm = dynamic_factor * (
                rotor_gravity_nm + captured_can_nm + queue_shell_nm + friction_nm
            )
            cases.append(
                {
                    "queue_can_count": queue_count,
                    "can_mass_g": can_mass_g,
                    "queue_downslope_force_N": round(queue_force_n, 4),
                    "rotor_gravity_Nm": round(rotor_gravity_nm, 5),
                    "captured_can_Nm": round(captured_can_nm, 5),
                    "queue_on_shell_Nm": round(queue_shell_nm, 5),
                    "friction_allowance_Nm": round(friction_nm, 5),
                    "design_torque_Nm": round(design_nm, 5),
                    "design_torque_kgf_cm": round(design_nm / kgf_cm_nm, 2),
                }
            )

    # Common catalogue figures are treated only as assumptions: clone servos
    # vary widely and generally publish stall, not continuous, torque.  Forty
    # percent of stall is used as the maximum engineering design demand.
    assumed_stall_kgf_cm = {"SG90_at_5V": 1.8, "MG90S_at_5V": 2.0}
    design_limit_fraction_of_stall = 0.40
    design_limits_kgf_cm = {
        name: round(value * design_limit_fraction_of_stall, 3)
        for name, value in assumed_stall_kgf_cm.items()
    }

    # With the upstream stack mechanically isolated, solve the allowable mass
    # of the one captured can at the same dynamic/friction factors.
    isolated_mass_limits_g = {}
    fixed_nm = rotor_gravity_nm + bearing_and_seal_floor_nm
    mass_coefficient_nm_per_kg = (
        (1.0 + friction_fraction) * g * captured_lever_m
    )
    for name, limit_kgf_cm in design_limits_kgf_cm.items():
        limit_nm = limit_kgf_cm * kgf_cm_nm
        mass_kg = max(
            0.0,
            (limit_nm / dynamic_factor - fixed_nm) / mass_coefficient_nm_per_kg,
        )
        isolated_mass_limits_g[name] = round(mass_kg * 1000.0, 1)

    csv_path = HERE / "FORCE_AUDIT_V11_B.csv"
    with csv_path.open("w", encoding="utf-8-sig", newline="") as fh:
        writer = csv.writer(fh, delimiter=";")
        writer.writerow(
            [
                "Банок в очереди",
                "Масса одной, г",
                "Сила вдоль ската, Н",
                "Ротор, Н·м",
                "Захваченная банка, Н·м",
                "Очередь на оболочке, Н·м",
                "Трение/подшипник, Н·м",
                "Расчётный момент, Н·м",
                "Расчётный момент, кгс·см",
                "Вердикт SG90/MG90S",
            ]
        )
        for row in cases:
            writer.writerow(
                [
                    row["queue_can_count"],
                    row["can_mass_g"],
                    row["queue_downslope_force_N"],
                    row["rotor_gravity_Nm"],
                    row["captured_can_Nm"],
                    row["queue_on_shell_Nm"],
                    row["friction_allowance_Nm"],
                    row["design_torque_Nm"],
                    row["design_torque_kgf_cm"],
                    "НЕ ДОПУСКАЕТСЯ: выше инженерного лимита обеих серв",
                ]
            )

    return {
        "method": "static gravity + direct queue load + friction allowance, then dynamic factor",
        "assumptions": {
            "PETG_density_g_cm3": petg_density_g_cm3,
            "roadway_slope_deg": P["lower_slope_deg"],
            "friction_allowance_fraction_of_loaded_torque": friction_fraction,
            "bearing_and_seal_floor_Nm": bearing_and_seal_floor_nm,
            "dynamic_factor": dynamic_factor,
            "assumed_stall_torque_kgf_cm": assumed_stall_kgf_cm,
            "maximum_design_fraction_of_stall": design_limit_fraction_of_stall,
            "warning": "catalogue stall torque is not a continuous holding rating; measure the actual servo batch",
        },
        "rotor": {
            "volume_cm3": round(volume_mm3 / 1000.0, 3),
            "estimated_PETG_mass_g": round(rotor_mass_kg * 1000.0, 2),
            "centre_of_mass_mm": [round(float(v), 3) for v in com],
            "gravity_eccentricity_mm": round(eccentricity_m * 1000.0, 3),
            "maximum_gravity_torque_Nm": round(rotor_gravity_nm, 5),
        },
        "servo_design_limits_kgf_cm": design_limits_kgf_cm,
        "direct_queue_cases": cases,
        "isolated_upstream_max_captured_can_mass_g": isolated_mass_limits_g,
        "verdict": (
            "REJECT for a directly loaded 11- or 22-can queue with SG90/MG90S. "
            "Use B only as a one-lane experiment with the upstream stack mechanically "
            "isolated; otherwise choose a mechanism that unloads the servo."
        ),
        "self_hold_and_power": {
            "PWM": "must remain active while queue load reaches the rotor; detach is not self-locking",
            "power_loss": "geartrain may back-drive; this geometry is not a fail-safe brake",
            "supply": "external regulated 5 V, common GND with ESP32; never use ESP32 5 V pin for five servos",
            "conservative_peak_supply_budget": "5 V / 5 A minimum for five channels, then verify stall current of the actual batch",
        },
    }


def write_bom():
    rows = [
        ("B01", "Ротор-клетка с карманом штатного рожка", "PETG", 1, 5, "Печатать осью вертикально"),
        ("B02L", "Левая полурама, консоль SG90 и половина клипсы A02", "PETG", 1, 5, "Печатать наружной щекой на столе"),
        ("B02R", "Правая полурама, M4-опора и половина клипсы A02", "PETG", 1, 5, "Печатать наружной щекой на столе"),
        ("T01", "Одноразовый coupon кармана рожка +0.20/+0.35/+0.50", "PETG", 0.2, 1, "Один на партию серв, не основная деталь"),
        ("HW01", "SG90 или MG90S, штатный двойной рожок и центральный винт", "покупное", 1, 5, "5 В отдельно, PWM ESP32"),
        ("HW02", "M2x8 самонарезающий винт крепления сервы", "сталь", 2, 10, "Под штатные уши SG90"),
        ("HW03", "M4x12 низкопрофильный винт фиксации правой оси", "сталь", 1, 5, "Головка утоплена в envelope"),
        ("HW04", "M4 латунная термовставка", "латунь", 1, 5, "Только обслуживаемая правая ось"),
        ("HW05", "M4x14 винт клипсы A02, снизу", "сталь", 2, 10, "Самонарезается в Ø3.3 пилот; дорожку не сверлить"),
        ("HW06", "Мягкая накладка 12x12x1 под прижимной винт", "TPU/резина", 2, 10, "Не продавить A02"),
    ]
    with (HERE / "BOM_V11_B.csv").open("w", encoding="utf-8-sig", newline="") as fh:
        writer = csv.writer(fh, delimiter=";")
        writer.writerow(["Код", "Наименование", "Материал", "На 1 канал", "На 5 каналов", "Примечание"])
        writer.writerows(rows)


def write_readme(force_audit_data):
    text = """# V11-B — роторная кассета, рабочий прототип

## Что здесь реально приводит банку

Одна **SG90/MG90S на канал** стоит соосно ротору, но утоплена внутрь его пустого центра и повернута корпусом вниз. Штатный двойной рожок входит в фасонный карман внутренней перемычки ротора. Никакой абстрактной вертикальной заслонки, отдельного редуктора или внешней тяги нет.

В HOME полноразмерная ячейка смотрит на очередь. За 84° ячейка поворачивается к выходному скату. Тонкая продольная дуга 120–210° и рёбра образуют лёгкую наружную оболочку: когда окно ушло от очереди, эта оболочка физически держит следующую банку и при закрытии поднимает её всего на 1,4 мм.

## Печатные детали на один канал

1. `B01_ротор_лёгкая_клетка.stl` — одна деталь.
2. `B02L_полурама_SG90_клипса_A02.stl` — одна деталь.
3. `B02R_полурама_M4_клипса_A02.stl` — одна деталь.

Итого: **3 основные печатные детали/канал, 15 на пять каналов**.

## Печать

- PETG, сопло 0.4 мм, слой 0.20 мм, 4 периметра, 5 верх/низ, заполнение 25–30% gyroid.
- Полурамы уже экспортированы наружной щекой на столе: поддержки не нужны.
- Ротор уже экспортирован осью вертикально и приводной перемычкой на стол. Нужна только локальная нарисованная поддержка под верхнюю правую M4-ступицу; продольная блокирующая дуга печатается без поддержки.
- Сначала один раз на партию серв напечатать `T01_coupon_рожок_020_035_050.stl`: 1/2/3 точки означают +0.20/+0.35/+0.50 мм. Выбранный зазор перенести в параметр генератора до пяти больших роторов.
- Текущий генератор использует средний проверяемый зазор кармана **+0.35 мм** (`horn_pocket_clearance`).
- На каждый STL нужен отдельный стол 220×220. Все три файла проверяются генератором по габариту.

## Сборка одного канала

1. Вплавить одну M4 вставку только в правую ступицу ротора.
2. Один раз прогнать два M4 винта через печатные Ø3.3 пилоты губок и выкрутить обратно — отдельные вставки клипсе не нужны.
3. Установить SG90/MG90S в консоль левой полурамы двумя M2x8. Корпус сервы смотрит вниз, вал — строго по оси ротора.
4. Электрически выставить серву в HOME и обесточить сигнал PWM.
5. Поставить штатный двойной рожок горизонтально и затянуть штатный центральный винт.
6. Надвинуть карман внутренней перемычки ротора на рожок. Правую ступицу посадить в отверстие правой полурамы; утопленный M4x12 лишь удерживает ось, не зажимает её намертво.
7. Свести две полурамы по двум печатным штифтам/гнёздам.

## Крепление к общей нижней A02 — без сверления дорожки

1. Надвинуть общую переднюю C-клипсу модуля на переднюю кромку A02 до упора.
2. Проверить центр по разделителям: они только центрируют, вес на них не передаётся.
3. Подтянуть **два M4 винта снизу**, по одному на каждую полураму, через мягкие накладки. Не перетягивать.

Картонные крайние стенки не используются как силовая опора. Все пять модулей крепятся одинаково к общей A02 и не выступают за шаг 79 мм.

От готовых печатных деталей до закреплённого канала получается **10 ручных действий**: 7 сборочных выше и 3 действия крепления ниже. Coupon — отдельная одноразовая проверка на партию серв.

## Жёсткий вывод по моменту

Расчёт лежит в `FORCE_AUDIT_V11_B.csv`. Он учитывает собственную массу и эксцентриситет ротора, момент захваченной банки, силу всей очереди вдоль ската 15°, плечо наружной дуги, 12% добавку на трение, 0.010 Н·м на подшипник/уплотнения и коэффициент динамики 1.25.

| Очередь | Масса банки | Требуется |
|---:|---:|---:|
| 11 | 100 г | 3.53 кгс·см |
| 11 | 200 г | 6.71 кгс·см |
| 22 | 100 г | 6.12 кгс·см |
| 22 | 200 г | 11.88 кгс·см |

**Вердикт: напрямую нагруженную очередь из 11 или 22 банок эта схема с SG90/MG90S не тянет с инженерным запасом.** Наружная дуга действительно блокирует следующую банку, но именно поэтому вся составляющая веса очереди по скату приходит на ротор и вал сервы. Это не скрытая мелочь, а ограничение самой кинематики B.

При принятых типовых stall-значениях 1.8 кгс·см для SG90 и 2.0 кгс·см для MG90S и допустимом расчётном требовании не более 40% stall, верхняя оценка одной захваченной банки при **механически разгруженной сверху очереди** — около {sg_limit:.1f} г для SG90 и {mg_limit:.1f} г для MG90S. Значения конкретного клона надо измерить.

Серва не является самотормозящимся приводом: под нагрузкой PWM должен оставаться активным, а при пропадании питания ротор может провернуться назад. Пять серв питаются от отдельного стабилизированного 5 В источника с общей землёй ESP32; консервативный стартовый бюджет — 5 В / 5 А, затем обязательное измерение пикового тока фактической партии.

## Обязательная проверка прототипа

Этот B печатать сначала **только в одном экземпляре** и только как эксперимент с разгрузкой очереди. Без банок выполнить 20 медленных циклов 0→84→0°, затем 300 циклов с реальной банкой допустимой массы. Для штатных 100–200 г и очереди 11/22 сразу выбирать другой механизм, который механически снимает нагрузку с сервы.
"""
    text = text.format(
        sg_limit=force_audit_data["isolated_upstream_max_captured_can_mass_g"]["SG90_at_5V"],
        mg_limit=force_audit_data["isolated_upstream_max_captured_can_mass_g"]["MG90S_at_5V"],
    )
    (HERE / "README_СБОРКА.md").write_text(text, encoding="utf-8")


def main():
    print("V11-B: building real three-part rotary lane...")
    rotor = make_rotor()
    frame_left = make_frame("left")
    frame_right = make_frame("right")

    print_reports = []
    print_parts = prepare_print_parts(frame_left, frame_right, rotor)
    print_parts["T01_coupon_рожок_020_035_050.stl"] = make_horn_coupon()
    for filename, mesh in print_parts.items():
        report = export_mesh(mesh, PRINT_DIR / filename)
        report["fits_220x220"] = bool(
            report["extents_mm"][0] <= P["bed_x"] + 0.01
            and report["extents_mm"][1] <= P["bed_y"] + 0.01
        )
        print_reports.append(report)

    loose_parts = {
        "frame_left_assembly.stl": frame_left,
        "frame_right_assembly.stl": frame_right,
        "rotor_HOME_assembly.stl": rotor,
        "cassette_A02_front_reference.stl": cassette_reference(),
        "right_M4_retainer_reference.stl": m4_retainer_reference(),
    }
    loose_parts.update({f"{k}_reference.stl": v for k, v in servo_reference_parts().items()})
    loose_parts.update({f"servo_wire_{k}_reference.stl": v for k, v in servo_wire_reference().items()})
    for filename, mesh in loose_parts.items():
        export_mesh(mesh, ASSEMBLY_DIR / filename)

    state_reports = []
    viewer_assets = {}
    viewer_style = {
        "frame_left.stl": ("printed_frame", "#d6a45f"),
        "frame_right.stl": ("printed_frame", "#d6a45f"),
        "rotor.stl": ("moving_rotor", "#f28c28"),
        "servo_body.stl": ("SG90_MG90S", "#276ca8"),
        "servo_shaft.stl": ("metal", "#c5ced8"),
        "servo_horn.stl": ("stock_horn", "#f0f2f5"),
        "servo_center_screw.stl": ("fastener", "#7d8794"),
        "right_M4_retainer.stl": ("fastener", "#7d8794"),
        "clamp_screw_left.stl": ("fastener", "#7d8794"),
        "clamp_screw_right.stl": ("fastener", "#7d8794"),
        "lower_sheet_A02.stl": ("cassette_A02", "#b98a55"),
        "can_captured.stl": ("can", "#42c986"),
        "can_queue_1.stl": ("can", "#42c986"),
        "can_queue_2.stl": ("can", "#42c986"),
        "wire_brown.stl": ("wire_GND", "#6b3f24"),
        "wire_red.stl": ("wire_5V", "#e94242"),
        "wire_orange.stl": ("wire_PWM", "#f29a2e"),
    }
    for name, angle in (("HOME", 0.0), ("MID", 42.0), ("EJECT", 84.0)):
        state = assembly_state(frame_left, frame_right, rotor, angle)
        state_reports.append(export_mesh(state, ASSEMBLY_DIR / f"assembly_{name}_with_3_cans.stl"))
        state_dir = ASSEMBLY_DIR / name
        state_dir.mkdir(parents=True, exist_ok=True)
        viewer_assets[name] = {}
        for filename, mesh in state_asset_meshes(frame_left, frame_right, rotor, angle).items():
            asset_path = state_dir / filename
            export_mesh(mesh, asset_path)
            role, color = viewer_style[filename]
            viewer_assets[name][filename.removesuffix(".stl")] = {
                "path": asset_path.relative_to(HERE).as_posix(),
                "role": role,
                "color": color,
            }

    analytical = analytical_checks()
    force_audit_data = force_torque_audit(rotor)
    print("V11-B: collision sweep 0/10/21/32/42/53/63/74/84 deg...")
    collisions = mesh_and_collision_checks(frame_left, frame_right, rotor)

    hard_failures = []
    for report in print_reports:
        if not report["watertight"]:
            hard_failures.append(f"not watertight: {report['file']}")
        if report["components"] != 1:
            hard_failures.append(f"multiple components: {report['file']} = {report['components']}")
        if not report["fits_220x220"]:
            hard_failures.append(f"does not fit 220 bed: {report['file']}")
        if abs(report["min_z_mm"]) > 0.01:
            hard_failures.append(f"not on bed: {report['file']}")
    for row in collisions:
        for key in (
            "rotor_vs_frame_mm3",
            "rotor_vs_servo_mm3",
            "rotor_vs_can_mm3",
            "rotor_vs_waiting_can_mm3",
        ):
            value = row[key]
            if value is None:
                hard_failures.append(f"collision boolean failed: {key} at {row['angle_deg']} deg")
            elif value > 0.25:
                hard_failures.append(f"collision {key} at {row['angle_deg']} deg = {value} mm3")

    manifest = {
        "project": "V11-B rotary cassette",
        "status": "physical prototype; validate one lane before five-lane production",
        "revision": P["revision"],
        "parameters_mm": P,
        "mechanism": {
            "actuator": "one SG90/MG90S per lane, stock double horn",
            "power": "5 V servo supply separate from ESP32; common GND; PWM from ESP32",
            "kinematics": "direct coaxial 84 degree rotation; servo body nested in hollow rotor",
            "cell": "one full-length cell for one 70 x Ø47 can",
            "blocking": "continuous 90 degree axial shell plus longitudinal spars blocks the next can with 1.4 mm lift",
            "opposite_support": "printed rotating neck in frame bearing, flush M4 retainer",
        },
        "main_printed_parts_per_lane": 3,
        "main_printed_parts_for_five_lanes": 15,
        "main_part_codes": ["B01", "B02L", "B02R"],
        "threaded_fasteners_per_lane": {
            "added_total": 5,
            "included_servo_center_screw": 1,
            "items": [
                "2 x M2x8 servo mounting screws",
                "1 x M4x12 flush right-axis retainer",
                "2 x M4x14 bottom edge-clamp screws",
                "1 x stock horn centre screw included with servo",
            ],
            "heat_set_inserts_M4": 1,
        },
        "supports": {
            "B01": "local painted support only under upper right M4 hub; drive web is on bed",
            "B02L": "none",
            "B02R": "none",
            "T01": "none",
        },
        "assembly_steps": [
            "Install one M4 heat-set insert in the right rotor hub.",
            "Run the two M4 clamp screws once through their printed 3.3 mm pilots, then remove them.",
            "Mount SG90/MG90S in B02L with two M2x8 screws, belly down.",
            "Command HOME electrically, then remove PWM signal.",
            "Install the stock double horn horizontal and tighten its stock centre screw.",
            "Key B01 onto the horn pocket and place its right neck in B02R.",
            "Fit the flush M4 retainer without axial preload; close B02L/B02R on printed pins.",
        ],
        "attachment_points_per_lane": 2,
        "attachment_steps": [
            "Slide the integral C-clip over the common front edge of A02.",
            "Use dividers only to centre the lane; do not load them.",
            "Tighten two M4 screws from below through soft pads; do not drill A02 roadway.",
        ],
        "exact_manual_actions_per_lane_from_prints_to_mounted": 10,
        "coupon": {
            "file": "STL_ПЕЧАТЬ/T01_coupon_рожок_020_035_050.stl",
            "frequency": "one per servo batch, not one per lane",
            "pocket_clearances_mm": [0.20, 0.35, 0.50],
            "identification": "one/two/three witness holes from left to right",
        },
        "print_reports": print_reports,
        "assembly_state_reports": state_reports,
        "analytical_clearances": analytical,
        "collision_sweep": collisions,
        "force_torque_audit": force_audit_data,
        "viewer_assets": viewer_assets,
        "hard_failures": hard_failures,
        "warnings": [
            "Prototype: measure the real can including lid/bottom rims before printing all five lanes.",
            "1.2 mm total axial clearance assumes the real can including seams is at most 70.0 mm; measure it before five-lane printing.",
            "Run a 300-cycle one-lane endurance test with the heaviest real can.",
            "B is rejected for a directly loaded 11/22-can queue with either SG90 or MG90S; see FORCE_AUDIT_V11_B.csv.",
            "Never power five servos from the ESP32 5 V pin.",
        ],
    }
    (HERE / "manifest_V11_B.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    write_bom()
    write_readme(force_audit_data)

    print(json.dumps({"print": print_reports, "collisions": collisions, "failures": hard_failures}, ensure_ascii=False, indent=2))
    if hard_failures:
        raise SystemExit("V11-B validation failed: " + "; ".join(hard_failures))
    print("V11-B OK: all printable meshes watertight, single-component and fit 220 x 220 mm.")


if __name__ == "__main__":
    main()
