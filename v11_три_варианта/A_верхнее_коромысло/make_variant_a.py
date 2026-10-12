# -*- coding: utf-8 -*-
"""V11 comparison concept A: upper two-finger rocker escapement.

This is deliberately a PROTOTYPE package, not a production release.  The
generator creates four easy-to-print parts for one lane, assembly reference
STLs for HOME/MID/EJECT, a five-lane installation reference, a BOM, a 3MF
plate and a machine-readable validation report.

Assembly axes before placement in the box:
    X / u  = downhill, towards the box outlet
    Y / v  = across the lane, along the can axis
    Z / n  = normal to the 15 degree lower deck

The moving rocker lives at the left edge of each lane.  Only two rounded
fingers enter the lane, so the mechanism can be repeated at 79 mm pitch.
"""
from __future__ import annotations

import csv
import hashlib
import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable

import numpy as np
import trimesh
from shapely.geometry import LineString, Point, Polygon, box as sbox
from shapely.ops import unary_union


HERE = Path(__file__).resolve().parent
PRINT_DIR = HERE / "STL_ПЕЧАТЬ"
ASSEMBLY_DIR = HERE / "STL_СБОРКА_НЕ_ПЕЧАТАТЬ"
COMMON_STL_DIR = HERE.parent / "00_ОБЩАЯ_КОМПОНОВКА" / "STL_КАССЕТА_СБОРКА"
for directory in (PRINT_DIR, ASSEMBLY_DIR):
    directory.mkdir(parents=True, exist_ok=True)


P = {
    "status": "PROTOTYPE_FOR_COMPARISON",
    "can_length": 70.0,
    "can_diameter_nominal": 47.0,
    "can_diameter_design": 48.0,
    "lane_clear": 75.0,
    "lane_pitch": 79.0,
    "lane_count": 5,
    "divider_thickness": 4.0,
    "lower_deck_angle_deg": 15.0,
    "cassette_lower_sheet_length": 569.402,
    "cassette_inside_width": 391.0,
    "sheet_thickness": 4.0,
    "sheet_front_edge_u": 80.0,
    "box_inside_depth": 600.0,
    "box_inside_width": 395.0,
    "shelf_supported_depth": 390.0,
    "rocker_pivot_u": 0.0,
    # V-rocker optimization: the active finger centres are ±22.422 mm in u
    # at n=33.868.  The raised opposite finger has 1.59 mm clearance above
    # the Ø48 deck path.
    "rocker_pivot_n": 41.516266235,
    "active_gate_half_spacing": 22.421828949,
    "active_finger_n": 33.868097522,
    "rocker_home_deg": -22.0,
    "rocker_eject_deg": 22.0,
    "rocker_stop_extra_deg": 0.20,
    "finger_radius": 2.0,
    "finger_reach": 19.4,
    "rocker_plate_thickness": 2.2,
    "can_axis_y_offset": 2.2,
    "servo_axis_u": -2.774081347,
    "servo_axis_n": 65.392355356,
    "servo_horn_radius": 9.0,
    "servo_home_geom_deg": -107.941574178,
    "servo_direction": -1.0,
    "servo_motion_deg": 83.699615607,
    "rocker_crank_radius": 16.578964743,
    "rocker_crank_angle_deg": 90.491554673,
    "link_length": 11.625516263,
    "bed_x": 220.0,
    "bed_y": 220.0,
    "bed_margin": 5.0,
}

PIVOT = np.array([P["rocker_pivot_u"], P["rocker_pivot_n"]], dtype=float)
SERVO_AXIS = np.array([P["servo_axis_u"], P["servo_axis_n"]], dtype=float)


# ---------------------------------------------------------------------------
# Mesh helpers
# ---------------------------------------------------------------------------
def clean(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    out = mesh.copy()
    out.remove_unreferenced_vertices()
    # Manifold3D may deliberately return coincident-but-topologically-distinct
    # vertices.  Blind merge_vertices() turns those valid edges into 4-face
    # non-manifold edges on current trimesh builds, so only keep a merge when
    # it does not degrade a watertight result.
    merged = out.copy()
    merged.merge_vertices()
    if (not out.is_watertight) or merged.is_watertight:
        out = merged
    trimesh.repair.fix_normals(out)
    return out


def fuse(meshes: Iterable[trimesh.Trimesh]) -> trimesh.Trimesh:
    items = [clean(m) for m in meshes if m is not None and len(m.faces)]
    if not items:
        raise ValueError("No meshes to fuse")
    if len(items) == 1:
        return items[0]
    result = trimesh.boolean.union(items, engine="manifold")
    if result is None:
        raise RuntimeError("Boolean union failed")
    return clean(result)


def cut(mesh: trimesh.Trimesh, cutters: Iterable[trimesh.Trimesh]) -> trimesh.Trimesh:
    tools = [clean(m) for m in cutters if m is not None and len(m.faces)]
    if not tools:
        return clean(mesh)
    result = trimesh.boolean.difference([clean(mesh), *tools], engine="manifold")
    if result is None:
        raise RuntimeError("Boolean difference failed")
    return clean(result)


def bx(size, center=(0.0, 0.0, 0.0)) -> trimesh.Trimesh:
    mesh = trimesh.creation.box(extents=np.asarray(size, dtype=float))
    mesh.apply_translation(np.asarray(center, dtype=float))
    return mesh


def cylinder_y(radius, length, center=(0.0, 0.0, 0.0), sections=64):
    mesh = trimesh.creation.cylinder(radius=radius, height=length, sections=sections)
    mesh.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0]))
    mesh.apply_translation(np.asarray(center, dtype=float))
    return mesh


def cylinder_z(radius, length, center=(0.0, 0.0, 0.0), sections=64):
    mesh = trimesh.creation.cylinder(radius=radius, height=length, sections=sections)
    mesh.apply_translation(np.asarray(center, dtype=float))
    return mesh


def sphere(radius, center, subdivisions=3):
    mesh = trimesh.creation.icosphere(subdivisions=subdivisions, radius=radius)
    mesh.apply_translation(np.asarray(center, dtype=float))
    return mesh


def cone_y(radius, length, base_center, direction=1.0, sections=64):
    mesh = trimesh.creation.cone(radius=radius, height=length, sections=sections)
    angle = -math.pi / 2 if direction > 0 else math.pi / 2
    mesh.apply_transform(trimesh.transformations.rotation_matrix(angle, [1, 0, 0]))
    mesh.apply_translation(np.asarray(base_center, dtype=float))
    return mesh


def revolved_y(profile, base_center, sections=64):
    """Create one watertight turned profile along +Y without union seams."""
    mesh = trimesh.creation.revolve(np.asarray(profile, dtype=float), sections=sections)
    mesh.apply_transform(trimesh.transformations.rotation_matrix(-math.pi / 2, [1, 0, 0]))
    mesh.apply_translation(np.asarray(base_center, dtype=float))
    return clean(mesh)


def extrude_xz(poly, thickness, y0=0.0) -> trimesh.Trimesh:
    """Extrude a Shapely polygon in (X,Z) along +Y."""
    if poly.geom_type != "Polygon":
        pieces = [g for g in getattr(poly, "geoms", []) if g.geom_type == "Polygon" and g.area > 1e-7]
        if not pieces:
            raise ValueError(f"No polygonal area in {poly.geom_type}")
        return fuse([extrude_xz(g, thickness, y0) for g in pieces])
    mesh = trimesh.creation.extrude_polygon(poly, height=thickness)
    transform = np.array(
        [[1, 0, 0, 0], [0, 0, 1, y0], [0, 1, 0, 0], [0, 0, 0, 1]],
        dtype=float,
    )
    mesh.apply_transform(transform)
    return clean(mesh)


def capsule_2d(a, b, radius):
    return LineString([tuple(a), tuple(b)]).buffer(radius, resolution=24)


def rounded_box_2d(x0, z0, x1, z1, radius):
    return sbox(x0 + radius, z0 + radius, x1 - radius, z1 - radius).buffer(
        radius, resolution=24
    )


def rotate_rocker(mesh, q_deg):
    out = mesh.copy()
    point = [P["rocker_pivot_u"], 0.0, P["rocker_pivot_n"]]
    out.apply_transform(
        trimesh.transformations.rotation_matrix(
            math.radians(q_deg), [0, -1, 0], point=point
        )
    )
    return clean(out)


def translate(mesh, xyz):
    out = mesh.copy()
    out.apply_translation(np.asarray(xyz, dtype=float))
    return out


def slope_to_box(mesh):
    """Map local (downhill, across, normal) into the exact common cassette."""
    out = mesh.copy()
    out.apply_transform(local_to_common_transform())
    return clean(out)


def local_to_common_transform():
    theta = math.radians(P["lower_deck_angle_deg"])
    # Columns map local u=downhill, y=across, n=road normal.  This is the
    # exact coordinate convention used by 00_ОБЩАЯ_КОМПОНОВКА.
    basis = np.array([
        [-math.cos(theta), 0.0, -math.sin(theta)],
        [0.0, 1.0, 0.0],
        [-math.sin(theta), 0.0, math.cos(theta)],
    ], dtype=float)
    front_top_global = np.array([
        40.0 - 2.0 * math.sin(theta),
        P["box_inside_width"] / 2,
        40.0 * math.tan(theta) + 2.0 * math.cos(theta),
    ])
    local_front = np.array([P["sheet_front_edge_u"], 0.0, 0.0])
    transform = np.eye(4)
    transform[:3, :3] = basis
    transform[:3, 3] = front_top_global - basis @ local_front
    return transform


def side_part_on_bed(mesh):
    """Outer Y face on the bed; all lane-facing features grow upward."""
    out = mesh.copy()
    out.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0]))
    out.apply_translation([0.0, 0.0, -out.bounds[0, 2]])
    return clean(out)


def export_mesh(mesh, path):
    mesh = clean(mesh)
    mesh.export(path)
    # Validate the bytes a slicer will actually ingest.  An in-memory mesh can
    # appear watertight while STL vertex welding exposes a 4-face seam.
    reloaded = trimesh.load_mesh(path, process=True)
    edge_use = np.bincount(reloaded.edges_unique_inverse)
    boundary_edges = int(np.count_nonzero(edge_use == 1))
    overused_edges = int(np.count_nonzero(edge_use > 2))
    return {
        "file": path.name,
        "extents_mm": [round(float(v), 3) for v in reloaded.extents],
        "bounds_mm": [[round(float(v), 3) for v in row] for row in reloaded.bounds],
        "watertight": bool(reloaded.is_watertight),
        "is_volume": bool(reloaded.is_volume),
        "components": int(len(reloaded.split(only_watertight=False))),
        "euler_number": int(reloaded.euler_number),
        "boundary_edges": boundary_edges,
        "overused_edges": overused_edges,
        "volume_cm3": round(abs(float(reloaded.volume)) / 1000.0, 3),
    }


# ---------------------------------------------------------------------------
# Kinematics
# ---------------------------------------------------------------------------
def rocker_point(local_xy, q_deg):
    a = math.radians(q_deg)
    c, s = math.cos(a), math.sin(a)
    v = np.asarray(local_xy, dtype=float) - PIVOT
    return PIVOT + np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


def rotate_2d(vector, angle_deg):
    a = math.radians(angle_deg)
    c, s = math.cos(a), math.sin(a)
    v = np.asarray(vector, dtype=float)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


def finger_local_centres():
    """Local V-arm tips derived from the two loaded endpoint constraints."""
    h = P["active_gate_half_spacing"]
    low_n = P["active_finger_n"]
    front_home = np.array([h, low_n])
    rear_eject = np.array([-h, low_n])
    front_local = PIVOT + rotate_2d(front_home - PIVOT, -P["rocker_home_deg"])
    rear_local = PIVOT + rotate_2d(rear_eject - PIVOT, -P["rocker_eject_deg"])
    return front_local, rear_local


def finger_centres(q_deg):
    front_local, rear_local = finger_local_centres()
    return rocker_point(front_local, q_deg), rocker_point(rear_local, q_deg)


def crank_point(q_deg):
    a = math.radians(P["rocker_crank_angle_deg"])
    local = PIVOT + P["rocker_crank_radius"] * np.array([math.cos(a), math.sin(a)])
    return rocker_point(local, q_deg)


def horn_point(servo_motion_deg):
    beta = math.radians(P["servo_home_geom_deg"] + P["servo_direction"] * servo_motion_deg)
    return SERVO_AXIS + P["servo_horn_radius"] * np.array([math.cos(beta), math.sin(beta)])


def linkage_error(q_deg, servo_motion_deg):
    return float(np.linalg.norm(crank_point(q_deg) - horn_point(servo_motion_deg)) - P["link_length"])


def rocker_q_for_servo(servo_motion_deg):
    # Analytic circle/circle closure.  A sign-change bisection is unreliable
    # near the shallow transmission minimum because both interval endpoints
    # can lie on the same side of the distance equation.
    horn = horn_point(servo_motion_deg)
    dvec = horn - PIVOT
    d = float(np.linalg.norm(dvec))
    r = P["rocker_crank_radius"]
    cosine = (r * r + d * d - P["link_length"] ** 2) / (2 * r * d)
    if abs(cosine) > 1.0 + 1e-8:
        raise RuntimeError(f"No linkage closure at servo {servo_motion_deg} deg")
    gamma = math.acos(max(-1.0, min(1.0, cosine)))
    phi = math.atan2(dvec[1], dvec[0])
    alpha = math.radians(P["rocker_crank_angle_deg"])
    expected = math.radians(
        P["rocker_home_deg"]
        + (P["rocker_eject_deg"] - P["rocker_home_deg"])
        * servo_motion_deg / P["servo_motion_deg"]
    )
    candidates = []
    for raw in (phi - alpha + gamma, phi - alpha - gamma):
        adjusted = raw + 2 * math.pi * round((expected - raw) / (2 * math.pi))
        candidates.append(adjusted)
    q = min(candidates, key=lambda value: abs(value - expected))
    result = math.degrees(q)
    if abs(linkage_error(result, servo_motion_deg)) > 1e-5:
        raise RuntimeError("Analytic linkage closure residual")
    return result


def servo_for_rocker_q(q_deg):
    """Inverse of the monotonic four-bar branch used by this prototype."""
    lo, hi = 0.0, P["servo_motion_deg"]
    increasing = rocker_q_for_servo(hi) > rocker_q_for_servo(lo)
    for _ in range(80):
        mid = (lo + hi) / 2
        qmid = rocker_q_for_servo(mid)
        if (qmid < q_deg) == increasing:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


STATES = {
    "HOME": {"servo_deg": 0.0, "rocker_deg": P["rocker_home_deg"]},
    "MID": {"servo_deg": P["servo_motion_deg"] / 2,
            "rocker_deg": rocker_q_for_servo(P["servo_motion_deg"] / 2)},
    "EJECT": {"servo_deg": P["servo_motion_deg"], "rocker_deg": P["rocker_eject_deg"]},
}


def home_can_centres():
    front, _ = finger_centres(P["rocker_home_deg"])
    rr = P["can_diameter_design"] / 2
    rp = P["finger_radius"]
    dn = front[1] - rr
    dx = math.sqrt((rr + rp) ** 2 - dn**2)
    first = front[0] - dx
    return [first, first - 48.0, first - 96.0]


# ---------------------------------------------------------------------------
# Printable parts
# ---------------------------------------------------------------------------
ROCKER_Y0 = -37.25
ROCKER_Y1 = ROCKER_Y0 + P["rocker_plate_thickness"]


def rocker_mesh(q_deg=0.0):
    p = PIVOT
    right, left = finger_local_centres()
    crank_angle = math.radians(P["rocker_crank_angle_deg"])
    crank = p + P["rocker_crank_radius"] * np.array(
        [math.cos(crank_angle), math.sin(crank_angle)]
    )
    lug = p + 14.0 * np.array([0.0, -1.0])
    outline = unary_union(
        [
            capsule_2d(p, left, 4.4),
            capsule_2d(p, right, 4.4),
            Point(*p).buffer(8.0, resolution=32),
            capsule_2d(p, crank, 3.4),
            capsule_2d(p, lug, 3.0),
            Point(*crank).buffer(4.1, resolution=24),
        ]
    )
    plate = extrude_xz(outline, P["rocker_plate_thickness"], ROCKER_Y0)

    fingers = []
    start_y = ROCKER_Y1 - 0.25
    reach = P["finger_reach"] + 0.25
    for pt in (left, right):
        fingers.append(
            cylinder_y(
                P["finger_radius"], reach,
                (pt[0], start_y + reach / 2, pt[1]), sections=64,
            )
        )
        fingers.append(sphere(P["finger_radius"], (pt[0], start_y + reach, pt[1])))

    pin_start = ROCKER_Y1 - 0.25
    shaft_len = 7.0
    # One turned solid avoids the near-coincident cylinder/cone seam which
    # some STL readers weld into non-manifold edges after the rocker rotates.
    snap_stud = revolved_y(
        [(0.0, 0.0), (1.65, 0.0), (1.65, shaft_len - 1.35),
         (2.25, shaft_len - 1.35), (0.0, shaft_len), (0.0, 0.0)],
        (crank[0], pin_start, crank[1]),
    )
    # This stud carries the printed lug into the plane of the two fixed frame
    # stops.  Like the fingers and snap pin it grows vertically when printed.
    stop_stud = cylinder_y(3.0, 2.2, (lug[0], pin_start + 1.1, lug[1]), sections=64)
    result = fuse([plate, *fingers, snap_stud, stop_stud])

    pivot_bore = cylinder_y(2.25, 8.0, (p[0], ROCKER_Y0 + 2.0, p[1]))
    inner_recess = cylinder_y(4.15, 0.85, (p[0], ROCKER_Y1 - 0.425, p[1]))
    result = cut(result, [pivot_bore, inner_recess])
    return rotate_rocker(result, q_deg)


def stop_centres():
    lug_r = 14.0
    contact = 3.0 + 3.0 + 0.05
    out = []
    q_min = min(P["rocker_home_deg"], P["rocker_eject_deg"])
    q_max = max(P["rocker_home_deg"], P["rocker_eject_deg"])
    for q, side in ((q_min - P["rocker_stop_extra_deg"], -1.0),
                    (q_max + P["rocker_stop_extra_deg"], 1.0)):
        phi = math.radians(-90.0 + q)
        tip = PIVOT + lug_r * np.array([math.cos(phi), math.sin(phi)])
        tangent = np.array([-math.sin(phi), math.cos(phi)])
        out.append(tip + side * contact * tangent)
    return out


def frame_mesh():
    p = PIVOT
    servo = SERVO_AXIS
    outer = unary_union(
        [
            capsule_2d(p, np.array([84.0, -1.0]), 7.5),
            capsule_2d(servo + np.array([0, 14]), np.array([84.0, -1.0]), 7.0),
            capsule_2d(p, servo, 8.5),
            Point(*p).buffer(13.0, resolution=32),
            # Deliberate 2 mm overlap with the diagonal avoids a tangent seam
            # which STL vertex welding would otherwise turn non-manifold.
            rounded_box_2d(servo[0] - 11.0, servo[1] - 10.0,
                           servo[0] + 22.0, servo[1] + 26.0, 4),
            rounded_box_2d(76, -7, 92, 8, 3),
        ]
    )
    # The entire side frame stays inside the 75 mm lane.  It locally guides
    # the can 2.2 mm away from the moving side and does not use a divider as a
    # structural member.
    # 0.20 mm union overlap with the cradle/bridge avoids coplanar STL seams;
    # the V-rocker retains 0.25 mm axial clearance to this face.
    base = extrude_xz(outer, 2.0, -34.8)

    stops = []
    for sc in stop_centres():
        stops.append(cylinder_y(3.0, 2.0, (sc[0], -34.0, sc[1]), sections=64))

    # SG90 slides from the lane side into this overhead C cradle and is held
    # by the two small compliant end nubs.  All protrusions grow vertically in
    # the documented print orientation.
    body_u = servo[0] + 5.5
    cradle = [
        bx((28.0, 16.8, 2.2), (body_u, -24.6, servo[1] - 4.1)),
        bx((28.0, 16.8, 2.0), (body_u, -24.6, servo[1] + 23.0)),
        bx((2.2, 16.8, 29.2), (body_u - 15.1, -24.6, servo[1] + 9.45)),
        bx((2.2, 16.8, 29.2), (body_u + 15.1, -24.6, servo[1] + 9.45)),
        bx((5.0, 1.8, 3.0), (servo[0] - 7.0, -15.3, servo[1] + 20.8)),
        bx((5.0, 1.8, 3.0), (servo[0] + 18.0, -15.3, servo[1] + 20.8)),
    ]

    # Hole-free edge clamp.  The upper bridge is entirely downstream of the
    # plywood edge u=80 except for a 5 mm long, 1.35 mm high lead-in ramp.
    upper_bridge = bx((12.0, 66.0, 5.0), (86.25, 0.0, -1.5))
    ramp_poly = Polygon([(75.0, 0.30), (80.0, 0.30), (80.0, 1.60)])
    upper_ramp = extrude_xz(ramp_poly, 66.0, -33.0)
    frame = fuse([base, *cradle, upper_bridge, upper_ramp])

    cutters = []
    cutters.append(cylinder_y(2.25, 8.0, (p[0], -34.0, p[1])))
    cutters.append(cylinder_y(4.15, 0.85, (p[0], -34.575, p[1])))

    # Swept passages through A01 for the two lane fingers and the snap crank.
    qs = np.linspace(P["rocker_home_deg"] - 1.0, P["rocker_eject_deg"] + 1.0, 65)
    front_path, rear_path, crank_path, stop_stud_path = [], [], [], []
    for q in qs:
        f, r = finger_centres(float(q))
        front_path.append(tuple(f))
        rear_path.append(tuple(r))
        crank_path.append(tuple(crank_point(float(q))))
        stop_stud_path.append(tuple(rocker_point(PIVOT + np.array([0.0, -14.0]), float(q))))
    slots = unary_union([
        LineString(front_path).buffer(P["finger_radius"] + 0.45, resolution=16),
        LineString(rear_path).buffer(P["finger_radius"] + 0.45, resolution=16),
        LineString(crank_path).buffer(2.55, resolution=16),
        LineString(stop_stud_path).buffer(3.35, resolution=16),
    ])
    cutters.append(extrude_xz(slots, 5.0, -36.0))

    # Clearance tunnel for the moving stock horn and short link.  It only
    # opens the lane-facing part of the servo cradle; the 2 mm side spine is
    # untouched and remains the structural load path.
    link_paths = []
    for sd in np.linspace(0.0, P["servo_motion_deg"], 43):
        qq = rocker_q_for_servo(float(sd))
        link_paths.append(capsule_2d(crank_point(qq), horn_point(float(sd)), 3.85))
    drive_clear = unary_union([
        *link_paths,
        Point(*SERVO_AXIS).buffer(18.20, resolution=40),
    ])
    cutters.append(extrude_xz(drive_clear, 4.6, -32.95))

    # The output boss physically crosses the 2 mm outer spine; give the
    # nominal Ø4.8 SG90/MG90S boss a true Ø5.6 through bore.  The wide servo
    # ears likewise need slots through the two cradle side walls.  They are
    # retained axially by the separate compliant entry nubs, not by an
    # impossible solid-on-solid interference in the final position.
    cutters.append(cylinder_y(2.8, 9.0, (servo[0], -33.5, servo[1]), sections=64))
    cutters.append(bx((33.2, 13.0, 3.0), (body_u, -24.7, servo[1] + 20.8)))

    for yy in (-23.0, 23.0):
        cutters.append(cylinder_z(2.25, 9.0, (86.0, yy, -1.5)))
    # Real 4 mm plywood occupies u<=80, -4<=n<=0.  Relieve the rounded end
    # of the diagonal and leave 0.25 mm insertion clearance; without this
    # notch the visually plausible frame would physically overlap the sheet.
    cutters.append(bx((705.0, 80.0, 4.5), (-272.25, 0.0, -2.0)))
    frame = cut(frame, cutters)
    # Add the two hard-stop bosses after opening the stud sweep.  This leaves
    # 0.05 mm nominal contact allowance at q=±22.2° without filling the slot.
    return fuse([frame, *stops])


def clamp_mesh():
    bar = bx((20.0, 64.0, 4.0), (82.0, 0.0, -7.0))
    pads = [
        bx((9.0, 13.0, 0.8), (76.5, -23.0, -4.6)),
        bx((9.0, 13.0, 0.8), (76.5, 23.0, -4.6)),
    ]
    body = fuse([bar, *pads])
    holes = [cylinder_z(2.25, 8.0, (86.0, yy, -7.0)) for yy in (-23.0, 23.0)]
    head_recesses = [cylinder_z(4.2, 1.0, (86.0, yy, -8.5)) for yy in (-23.0, 23.0)]
    return cut(body, [*holes, *head_recesses])


def link_print_mesh():
    length = P["link_length"]
    outline = capsule_2d((0, 0), (length, 0), 3.2)
    body = extrude_xz(outline, 2.0, 0.0)
    holes = [
        cylinder_y(2.05, 4.0, (0, 1.0, 0)),
        cylinder_y(1.15, 4.0, (length, 1.0, 0)),
    ]
    return cut(body, holes)


def link_state_mesh(q_deg, servo_deg):
    q = crank_point(q_deg)
    h = horn_point(servo_deg)
    outline = capsule_2d(q, h, 3.2)
    body = extrude_xz(outline, 2.0, -30.9)
    holes = [
        cylinder_y(2.05, 4.0, (q[0], -29.9, q[1])),
        cylinder_y(1.15, 4.0, (h[0], -29.9, h[1])),
    ]
    return cut(body, holes)


# ---------------------------------------------------------------------------
# Assembly reference solids
# ---------------------------------------------------------------------------
def servo_reference():
    # Conservative SG90/MG90S envelope.  Output shaft faces the divider.
    body_u = P["servo_axis_u"] + 5.5
    body = bx((23.0, 12.2, 24.5), (body_u, -24.7, P["servo_axis_n"] + 9.7))
    ears = bx((32.0, 12.2, 2.2), (body_u, -24.7, P["servo_axis_n"] + 20.8))
    shaft = cylinder_y(2.4, 5.4, (P["servo_axis_u"], -33.5, P["servo_axis_n"]), sections=48)
    return fuse([body, ears, shaft])


def horn_reference(servo_deg):
    beta = math.radians(P["servo_home_geom_deg"] + P["servo_direction"] * servo_deg)
    a = SERVO_AXIS - 15.5 * np.array([math.cos(beta), math.sin(beta)])
    b = SERVO_AXIS + 15.5 * np.array([math.cos(beta), math.sin(beta)])
    poly = unary_union(
        [
            capsule_2d(a, b, 2.25),
            Point(*SERVO_AXIS).buffer(3.8, resolution=24),
        ]
    )
    body = extrude_xz(poly, 1.5, -32.7)
    hole = horn_point(servo_deg)
    cutters = [
        cylinder_y(1.15, 3.0, (hole[0], -31.95, hole[1])),
        cylinder_y(1.1, 3.0, (SERVO_AXIS[0], -31.95, SERVO_AXIS[1])),
    ]
    return cut(body, cutters)


def can_mesh(u, y=None, n=24.0):
    if y is None:
        y = P["can_axis_y_offset"]
    return cylinder_y(P["can_diameter_design"] / 2, P["can_length"], (u, y, n), 72)


def state_cans(state):
    centres = home_can_centres()
    if state == "EJECT":
        # First can is already beyond the front stop and beginning to leave
        # the deck; the rear finger retains can two.
        _, rear = finger_centres(P["rocker_eject_deg"])
        dz = rear[1] - P["can_diameter_design"] / 2
        dx = math.sqrt((P["can_diameter_design"] / 2 + P["finger_radius"]) ** 2 - dz**2)
        retained = rear[0] - dx
        positions = [(115.0, -20.0), (retained, 24.0), (retained - P["can_diameter_design"], 24.0)]
    else:
        positions = [(u, 24.0) for u in centres]
    return [can_mesh(u, None, n) for u, n in positions]


_EXACT_CASSETTE_CACHE = None


def exact_cassette_local():
    """Load, without substitutes, the shared V11 lower sheet and divider."""
    global _EXACT_CASSETTE_CACHE
    if _EXACT_CASSETTE_CACHE is not None:
        lower, dividers = _EXACT_CASSETTE_CACHE
        return lower.copy(), [item.copy() for item in dividers]
    lower_path = COMMON_STL_DIR / "cassette_lower_sheet.stl"
    divider_path = COMMON_STL_DIR / "cassette_divider.stl"
    if not lower_path.exists() or not divider_path.exists():
        raise FileNotFoundError(
            "Exact common cassette STL files are required: " + str(COMMON_STL_DIR)
        )
    inverse = np.linalg.inv(local_to_common_transform())
    lower = trimesh.load_mesh(lower_path, process=True)
    lower.apply_transform(inverse)
    lower = clean(lower)
    source = trimesh.load_mesh(divider_path, process=True)
    dividers = []
    # Actual common-cassette positions; exactly four internal sheets.
    for global_y in (79.0, 158.0, 237.0, 316.0):
        divider = source.copy()
        divider.apply_translation([0.0, global_y, 0.0])
        divider.apply_transform(inverse)
        dividers.append(clean(divider))
    _EXACT_CASSETTE_CACHE = (lower.copy(), [item.copy() for item in dividers])
    return lower, dividers


def lower_deck_local(width=None, u0=None, u1=None):
    # Signature retained for callers; synthetic substitutes are forbidden.
    return exact_cassette_local()[0]


def one_lane_state_parts(state):
    spec = STATES[state]
    q = spec["rocker_deg"]
    sd = spec["servo_deg"]
    return {
        "frame": frame_mesh(),
        "clamp": clamp_mesh(),
        "rocker": rocker_mesh(q),
        "link": link_state_mesh(q, sd),
        "servo": servo_reference(),
        "horn": horn_reference(sd),
        "deck": lower_deck_local(),
        "cans": fuse(state_cans(state)),
    }


def assembly_state_mesh(state):
    parts = one_lane_state_parts(state)
    dividers = exact_cassette_local()[1]
    return slope_to_box(trimesh.util.concatenate([*parts.values(), dividers[1], dividers[2]]))


def five_lane_mesh(state="HOME"):
    lane_centres = [-158.0, -79.0, 0.0, 79.0, 158.0]
    exact_lower, exact_dividers = exact_cassette_local()
    meshes = [exact_lower, *exact_dividers]
    for lane_y in lane_centres:
        parts = one_lane_state_parts(state)
        for name in ("frame", "clamp", "rocker", "link", "servo", "horn"):
            meshes.append(translate(parts[name], (0, lane_y, 0)))
        for can in state_cans(state):
            meshes.append(translate(can, (0, lane_y, 0)))
    return slope_to_box(trimesh.util.concatenate(meshes))


def installation_reference_mesh():
    cassette = five_lane_mesh("HOME")
    bottom = bx((595.0, 395.0, 4.0), (297.5, 197.5, -2.0))
    left_wall = bx((595.0, 4.0, 210.0), (297.5, -2.0, 103.0))
    right_wall = bx((595.0, 4.0, 210.0), (297.5, 397.0, 103.0))
    # Rear 390 mm rests on the shelf; the outlet/front is at global x≈0.
    shelf = bx((390.0, 455.0, 20.0), (400.0, 197.5, -14.0))
    shelf_edge = bx((12.0, 455.0, 65.0), (205.0, 197.5, -36.5))
    return clean(trimesh.util.concatenate([cassette, bottom, left_wall, right_wall, shelf, shelf_edge]))


# ---------------------------------------------------------------------------
# Validation and files
# ---------------------------------------------------------------------------
def intersection_volume(a, b):
    if np.any(a.bounds[1] <= b.bounds[0]) or np.any(b.bounds[1] <= a.bounds[0]):
        return 0.0
    try:
        result = trimesh.boolean.intersection([clean(a), clean(b)], engine="manifold")
        if result is None or not len(result.faces):
            return 0.0
        return abs(float(result.volume))
    except BaseException:
        return float("nan")


def print_plate_report(items):
    occupied = []
    records = []
    for name, mesh, x, y in items:
        ext = mesh.extents
        rect = (x, y, x + float(ext[0]), y + float(ext[1]))
        inside = (
            rect[0] >= P["bed_margin"] - 1e-6
            and rect[1] >= P["bed_margin"] - 1e-6
            and rect[2] <= P["bed_x"] - P["bed_margin"] + 1e-6
            and rect[3] <= P["bed_y"] - P["bed_margin"] + 1e-6
        )
        overlap = []
        for other_name, other in occupied:
            dx = min(rect[2], other[2]) - max(rect[0], other[0])
            dy = min(rect[3], other[3]) - max(rect[1], other[1])
            if dx > 0 and dy > 0:
                overlap.append(other_name)
        occupied.append((name, rect))
        records.append({"part": name, "xy_bounds_mm": [round(v, 2) for v in rect], "inside": inside, "overlaps": overlap})
    return records


def analytic_sweep_report():
    cans = home_can_centres()
    rr = P["can_diameter_design"] / 2
    rp = P["finger_radius"]
    min_unintended = (1e9, None)
    front_contact = None
    samples = []
    for servo_deg in np.linspace(0.0, P["servo_motion_deg"], 87):
        q = rocker_q_for_servo(float(servo_deg))
        front, rear = finger_centres(q)
        row_min = 1e9
        for pin_name, pin in (("front", front), ("rear", rear)):
            for ci, cu in enumerate(cans):
                clearance = math.hypot(pin[0] - cu, pin[1] - rr) - (rr + rp)
                if pin_name == "front" and ci == 0 and servo_deg == 0:
                    front_contact = clearance
                else:
                    row_min = min(row_min, clearance)
                    if clearance < min_unintended[0]:
                        min_unintended = (clearance, (float(servo_deg), q, pin_name, ci + 1))
        samples.append({
            "servo_deg": round(float(servo_deg), 3),
            "rocker_deg": round(float(q), 5),
            "front_n": round(float(front[1]), 4),
            "rear_n": round(float(rear[1]), 4),
            "min_clearance_to_fixed_HOME_display_cans_mm": round(float(row_min), 4),
        })
    return {
        "sample_count": len(samples),
        "front_can_intended_contact_clearance_mm": round(float(front_contact), 6),
        # The three HOME cylinders in this diagnostic deliberately remain
        # fixed.  Once motion starts the leading can follows the moving front
        # barrier and is later released, so a negative fixed-cylinder value
        # near EJECT is not a physical collision.  The moving-can proof below
        # is the acceptance check.
        "minimum_pin_to_fixed_HOME_display_can_clearance_mm": round(float(min_unintended[0]), 4),
        "fixed_HOME_display_cans_used_for_acceptance": False,
        "fixed_display_minimum_location": {
            "servo_deg": round(min_unintended[1][0], 3),
            "rocker_deg": round(min_unintended[1][1], 4),
            "pin": min_unintended[1][2],
            "can": min_unintended[1][3],
        },
        "note": "Fixed HOME display cans are only a diagnostic; see continuous_can_barrier_proof.moving_can_handoff for the kinematic acceptance proof.",
        "samples": samples,
    }


def continuous_barrier_report():
    """Prove that a deck-running Ø48 can never sees both gates open.

    A can centre travels at n=24.  A round Ø4 finger excludes an interval of
    that path whenever its centre is less than Rcan+Rfinger=26 mm from it.
    This is a geometric barrier proof and does not rely on timing, friction or
    the assumed positions of the three display cans.
    """
    can_n = P["can_diameter_design"] / 2
    sum_r = can_n + P["finger_radius"]

    def gate_interval(pin):
        dz = float(pin[1] - can_n)
        if abs(dz) >= sum_r:
            half = 0.0
            blocks = False
        else:
            half = math.sqrt(max(0.0, sum_r * sum_r - dz * dz))
            blocks = half > 0.0
        return half, float(pin[0] - half), float(pin[0] + half), blocks

    def gate_record(pin):
        half, lower, upper, blocks = gate_interval(pin)
        return {
            "centre_u_n_mm": [round(float(pin[0]), 5), round(float(pin[1]), 5)],
            "excluded_path_halfwidth_mm": round(float(half), 5),
            "excluded_u_interval_mm": [round(lower, 5), round(upper, 5)],
            "blocks_deck_can": blocks,
        }

    rows = []
    both_free = []
    for sd in np.linspace(0.0, P["servo_motion_deg"], 337):
        q = rocker_q_for_servo(float(sd))
        front, rear = finger_centres(q)
        fr = gate_record(front)
        rr = gate_record(rear)
        if not fr["blocks_deck_can"] and not rr["blocks_deck_can"]:
            both_free.append(float(sd))
        rows.append({
            "servo_deg": round(float(sd), 4),
            "rocker_deg": round(float(q), 5),
            "front_blocks": fr["blocks_deck_can"],
            "rear_blocks": rr["blocks_deck_can"],
        })

    height_limit = can_n + sum_r

    def height_margin(sd, which):
        pins = finger_centres(rocker_q_for_servo(sd))
        return height_limit - float(pins[0 if which == "front" else 1][1])

    def find_transition(which, rising):
        scan = np.linspace(0.0, P["servo_motion_deg"], 1001)
        previous_sd = float(scan[0])
        previous = height_margin(previous_sd, which)
        bracket = None
        for value in scan[1:]:
            sd = float(value)
            current = height_margin(sd, which)
            crossed = (previous <= 0 < current) if rising else (previous > 0 >= current)
            if crossed:
                bracket = (previous_sd, sd)
                break
            previous_sd, previous = sd, current
        if bracket is None:
            raise RuntimeError(f"No {which} barrier transition")
        lo, hi = bracket
        for _ in range(70):
            mid = (lo + hi) / 2
            value = height_margin(mid, which)
            if (value > 0) == rising:
                hi = mid
            else:
                lo = mid
        sd = (lo + hi) / 2
        return sd, rocker_q_for_servo(sd)

    servo_rear_on, q_rear_on = find_transition("rear", True)
    servo_front_off, q_front_off = find_transition("front", False)

    # Explicit moving-can handoff, rather than pretending the three HOME
    # display cylinders remain frozen.  Positive u is downhill.  While the
    # front gate is active, can 1 rests at its upstream exclusion boundary.
    # Can 2 is packed one diameter behind it until the rear gate touches it;
    # afterwards can 2 rests at the rear gate's upstream boundary.  There
    # must always be a non-negative centre corridor for can 1 between gates,
    # and the two retained centres may never be closer than one diameter.
    def packed_rear_clearance(sd):
        q = rocker_q_for_servo(sd)
        front, rear = finger_centres(q)
        front_half, front_lower, _, front_blocks = gate_interval(front)
        if not front_blocks or front_half <= 0.0:
            return float("nan")
        packed_can2_u = front_lower - P["can_diameter_design"]
        return math.hypot(rear[0] - packed_can2_u, rear[1] - can_n) - sum_r

    scan = np.linspace(0.0, servo_front_off, 2001)
    transfer_bracket = None
    previous_sd = float(scan[0])
    previous = packed_rear_clearance(previous_sd)
    for value in scan[1:]:
        sd = float(value)
        current = packed_rear_clearance(sd)
        if previous > 0.0 >= current:
            transfer_bracket = (previous_sd, sd)
            break
        previous_sd, previous = sd, current
    if transfer_bracket is None:
        raise RuntimeError("Rear finger never hands off the packed second can")
    lo, hi = transfer_bracket
    for _ in range(70):
        mid = (lo + hi) / 2.0
        if packed_rear_clearance(mid) <= 0.0:
            hi = mid
        else:
            lo = mid
    servo_transfer = (lo + hi) / 2.0
    q_transfer = rocker_q_for_servo(servo_transfer)

    corridor_rows = []
    retained_spacing_rows = []
    handoff_samples = sorted(set(
        [float(v) for v in np.linspace(servo_rear_on, servo_front_off, 2001)]
        + [float(servo_transfer)]
    ))
    for value in handoff_samples:
        sd = float(value)
        q = rocker_q_for_servo(sd)
        front, rear = finger_centres(q)
        _, front_lower, _, front_blocks = gate_interval(front)
        _, rear_lower, rear_upper, rear_blocks = gate_interval(rear)
        if front_blocks and rear_blocks:
            corridor_rows.append((front_lower - rear_upper, sd, q))
        if sd + 1e-9 >= servo_transfer and front_blocks and rear_blocks:
            retained_spacing_rows.append((front_lower - rear_lower, sd, q))
    min_corridor = min(corridor_rows, key=lambda row: row[0])
    min_retained_spacing = min(retained_spacing_rows, key=lambda row: row[0])

    q_release = rocker_q_for_servo(servo_front_off)
    front_release, rear_at_release = finger_centres(q_release)
    _, _, rear_release_upper, rear_release_blocks = gate_interval(rear_at_release)
    released_can_downstream_clearance = float(front_release[0] - rear_release_upper)

    _, eject_rear = finger_centres(P["rocker_eject_deg"])
    _, eject_rear_lower, _, eject_rear_blocks = gate_interval(eject_rear)
    handoff_pass = bool(
        servo_transfer < servo_front_off
        and min_corridor[0] >= -1e-6
        and min_retained_spacing[0] >= P["can_diameter_design"] - 1e-5
        and released_can_downstream_clearance > 0.0
        and eject_rear_blocks
    )

    home_front, home_rear = finger_centres(P["rocker_home_deg"])
    eject_front, eject_rear = finger_centres(P["rocker_eject_deg"])
    cans = home_can_centres()
    key = {}
    for name, q in (("HOME", P["rocker_home_deg"]), ("MID", STATES["MID"]["rocker_deg"]),
                    ("EJECT", P["rocker_eject_deg"])):
        front, rear = finger_centres(q)
        key[name] = {"front": gate_record(front), "rear": gate_record(rear)}
    return {
        "model": "Ø48 can centre constrained to deck line n=24; Ø4 circular finger",
        "can_path_n_mm": can_n,
        "blocking_centre_height_limit_mm": height_limit,
        "sample_count": len(rows),
        "no_both_gates_free": not both_free,
        "both_free_servo_samples_deg": [round(v, 4) for v in both_free],
        "rear_begins_blocking": {
            "rocker_deg": round(q_rear_on, 6),
            "servo_deg": round(servo_rear_on, 6),
        },
        "front_stops_blocking": {
            "rocker_deg": round(q_front_off, 6),
            "servo_deg": round(servo_front_off, 6),
        },
        "both_block_overlap": {
            "rocker_deg": round(q_front_off - q_rear_on, 6),
            "servo_deg": round(servo_front_off - servo_rear_on, 6),
        },
        "moving_can_handoff": {
            "model": "can 1 follows front upstream boundary; packed can 2 transfers to rear upstream boundary; no friction/timing assumption",
            "rear_contacts_packed_can2": {
                "servo_deg": round(servo_transfer, 6),
                "rocker_deg": round(q_transfer, 6),
            },
            "front_releases_can1": {
                "servo_deg": round(servo_front_off, 6),
                "rocker_deg": round(q_front_off, 6),
            },
            "positive_handoff_overlap_servo_deg": round(servo_front_off - servo_transfer, 6),
            "minimum_can1_centre_corridor_between_gates_mm": round(float(min_corridor[0]), 6),
            "minimum_can1_can2_centre_spacing_after_transfer_mm": round(float(min_retained_spacing[0]), 6),
            "required_can1_can2_centre_spacing_mm": P["can_diameter_design"],
            "can1_downstream_of_rear_barrier_at_release_mm": round(released_can_downstream_clearance, 6),
            "retained_can2_centre_u_at_EJECT_mm": round(eject_rear_lower, 6),
            "pass": handoff_pass,
        },
        "raised_finger_endpoint_clearance_above_blocking_limit_mm": round(
            min(float(home_rear[1]), float(eject_front[1])) - height_limit, 6
        ),
        "gate_planes_and_cans_home_mm": {
            "front_finger_u_n": [round(float(v), 5) for v in home_front],
            "rear_finger_u_n": [round(float(v), 5) for v in home_rear],
            "can_centres_u_n": [[round(float(u), 5), can_n] for u in cans],
            "front_finger_to_can1_u_offset_mm": round(float(home_front[0] - cans[0]), 5),
            "rear_finger_between_can1_can2": bool(cans[1] < home_rear[0] < cans[0]),
            "note": "Пальцы стоят в плоскостях затворов: перед первой банкой и между первой/второй, не над центрами.",
        },
        "key_states": key,
        "pass": bool((not both_free) and handoff_pass),
    }


def force_report():
    """Quasi-static load table using the exact V-rocker/four-bar derivative."""
    can_n = P["can_diameter_design"] / 2
    sum_r = can_n + P["finger_radius"]
    gravity_factor = 9.81 * math.sin(math.radians(P["lower_deck_angle_deg"]))
    torque_limit = 0.060

    def horizontal_offset(pin):
        dz = float(pin[1] - can_n)
        if abs(dz) >= sum_r:
            return None
        return math.sqrt(max(0.0, sum_r * sum_r - dz * dz))

    def packed_rear_clearance(sd):
        q = rocker_q_for_servo(sd)
        front, rear = finger_centres(q)
        dx = horizontal_offset(front)
        if dx is None:
            return 1e6
        can2 = front[0] - dx - P["can_diameter_design"]
        return math.hypot(rear[0] - can2, rear[1] - can_n) - sum_r

    scan = np.linspace(0.0, P["servo_motion_deg"], 1001)
    transfer_bracket = None
    prior_sd, prior = float(scan[0]), packed_rear_clearance(float(scan[0]))
    for value in scan[1:]:
        sd = float(value)
        current = packed_rear_clearance(sd)
        if prior > 0 >= current:
            transfer_bracket = (prior_sd, sd)
            break
        prior_sd, prior = sd, current
    if transfer_bracket is None:
        raise RuntimeError("Rear gate never reaches packed can 2")
    lo, hi = transfer_bracket
    for _ in range(80):
        mid = (lo + hi) / 2
        if packed_rear_clearance(mid) <= 0:
            hi = mid
        else:
            lo = mid
    sd_transfer = (lo + hi) / 2
    q_transfer = rocker_q_for_servo(sd_transfer)

    def dq_dservo(sd):
        eps = 0.005
        aa = max(0.0, sd - eps)
        bb = min(P["servo_motion_deg"], sd + eps)
        return (rocker_q_for_servo(bb) - rocker_q_for_servo(aa)) / (bb - aa)

    def signed_rocker_torque(q, gate, downhill_force):
        pin = finger_centres(q)[0 if gate == "front" else 1]
        dx = horizontal_offset(pin)
        if dx is None or dx < 1e-7:
            return None
        dz = float(pin[1] - can_n)
        fx = downhill_force
        fz = downhill_force * dz / dx
        arm = pin - PIVOT
        return float(arm[0] * fz - arm[1] * fx) / 1000.0

    def torque_curve(count, mass_kg):
        records = []
        for value in scan:
            sd = float(value)
            q = rocker_q_for_servo(sd)
            before = sd <= sd_transfer
            gate = "front" if before else "rear"
            loaded = count if before else max(0, count - 1)
            downhill = loaded * mass_kg * gravity_factor
            rocker_tau = 0.0 if loaded == 0 else signed_rocker_torque(q, gate, downhill)
            if rocker_tau is None:
                servo_tau = float("inf")
            else:
                servo_tau = abs(rocker_tau * dq_dservo(sd))
            records.append((sd, q, gate, loaded, rocker_tau, servo_tau))
        return records

    def curve_summary(count, mass_g):
        curve = torque_curve(count, mass_g / 1000.0)
        peak_row = max(curve, key=lambda row: row[5])
        average = float(np.trapezoid([row[5] for row in curve],
                                    x=np.radians([row[0] for row in curve])) /
                        math.radians(P["servo_motion_deg"]))
        return {
            "cans": count,
            "mass_each_g": mass_g,
            "downhill_force_N": round(count * mass_g / 1000.0 * gravity_factor, 6),
            "peak_servo_torque_Nm": round(float(peak_row[5]), 6),
            "peak_at": {
                "servo_deg": round(float(peak_row[0]), 6),
                "rocker_deg": round(float(peak_row[1]), 6),
                "gate": peak_row[2],
                "loaded_cans": peak_row[3],
            },
            "ideal_average_servo_torque_Nm": round(average, 6),
            "pass_0_06Nm": bool(peak_row[5] <= torque_limit),
        }

    load_table = []
    for mass_g in (50, 100, 200):
        rows = [curve_summary(count, mass_g) for count in range(1, 23)]
        passing = [row for row in rows if row["pass_0_06Nm"]]
        strict_max = max(row["cans"] for row in passing) if passing else 0
        at_max = rows[strict_max - 1] if strict_max else None
        next_row = rows[strict_max] if strict_max < 22 else None
        load_table.append({
            "mass_each_g": mass_g,
            "strict_quasistatic_max_cans_at_0_06Nm": strict_max,
            "peak_at_max_Nm": None if at_max is None else at_max["peak_servo_torque_Nm"],
            "next_count": None if next_row is None else next_row["cans"],
            "next_count_peak_Nm": None if next_row is None else next_row["peak_servo_torque_Nm"],
            "physical_channel_capacity_cap": 22,
        })

    case_22x200 = curve_summary(22, 200)
    case_22x100 = curve_summary(22, 100)
    case_11x200 = curve_summary(11, 200)
    checkpoints = []
    key_sd = sorted(set([0.0, P["servo_motion_deg"] * 0.25,
                         sd_transfer, P["servo_motion_deg"] * 0.5,
                         P["servo_motion_deg"] * 0.75, P["servo_motion_deg"]]))
    curve = torque_curve(22, 0.100)
    for sd in key_sd:
        row = min(curve, key=lambda item: abs(item[0] - sd))
        checkpoints.append({
            "servo_deg": round(row[0], 6), "rocker_deg": round(row[1], 6),
            "gate": row[2], "loaded_cans": row[3],
            "servo_torque_Nm_for_22x100g": round(row[5], 6),
        })

    home_signed = signed_rocker_torque(P["rocker_home_deg"], "front",
                                       22 * 0.200 * gravity_factor)
    eject_signed = signed_rocker_torque(P["rocker_eject_deg"], "rear",
                                        21 * 0.200 * gravity_factor)
    home_is_min = P["rocker_home_deg"] < P["rocker_eject_deg"]
    home_into_stop = home_signed < 0 if home_is_min else home_signed > 0
    eject_into_stop = eject_signed > 0 if home_is_min else eject_signed < 0
    strict_200 = next(row["strict_quasistatic_max_cans_at_0_06Nm"]
                      for row in load_table if row["mass_each_g"] == 200)
    return {
        "model": "quasi-static frictionless Ø4/Ø48 contact; exact four-bar derivative; peak limit, not average, determines table",
        "sg90_design_torque_limit_Nm": torque_limit,
        "rear_first_contacts_packed_can2": {
            "rocker_deg": round(q_transfer, 6), "servo_deg": round(sd_transfer, 6),
        },
        "target_cases": {"22x100g": case_22x100, "11x200g": case_11x200,
                         "22x200g": case_22x200},
        "checkpoints": checkpoints,
        "load_table": load_table,
        "full_22x200g_pass": case_22x200["pass_0_06Nm"],
        "full_22x100g_pass": case_22x100["pass_0_06Nm"],
        "11x200g_pass": case_11x200["pass_0_06Nm"],
        "checkpoint_peak_servo_torque_Nm": case_22x200["peak_servo_torque_Nm"],
        "ideal_average_servo_torque_full_queue_Nm": case_22x200["ideal_average_servo_torque_Nm"],
        "strict_quasistatic_max_cans_200g": strict_200,
        "prototype_recommended_initial_max_cans_200g": min(8, strict_200),
        "endpoint_load_direction": {
            "home_signed_rocker_torque_Nm_22x200g": round(home_signed, 6),
            "eject_signed_rocker_torque_Nm_21x200g": round(eject_signed, 6),
            "home_load_reacts_into_hard_stop": home_into_stop,
            "eject_load_reacts_into_hard_stop": eject_into_stop,
            "power_off_self_hold": bool(home_into_stop and eject_into_stop),
        },
        "power_safety": {
            "external_signal_pulldown": "10 kΩ from each servo signal to GND",
            "hold_pwm_required": True,
            "continuous_pwm_requirement": "while any can remains in the channel, valid calibrated 50 Hz PWM must remain continuously present in HOME, EJECT and during motion; servo power alone is not a holding command",
            "startup": ["servo 5 V OFF", "boot ESP32 and hold all servo signal pins LOW",
                        "start valid 50 Hz PWM and command calibrated HOME", "enable servo 5 V", "load cans"],
            "shutdown": ["stop dispensing", "command HOME", "fully unload every can from the channel",
                         "disable servo 5 V", "stop PWM only after servo 5 V is absent", "power down controller"],
            "loaded_power_loss_safe": False,
            "loaded_signal_loss_safe": False,
        },
        "warning": "Peak table is quasi-static. Friction, impacts and print flex require a one-lane endurance test; loaded power loss is not safe.",
    }


def collision_report():
    checks = []
    for state in STATES:
        parts = one_lane_state_parts(state)
        for moving in ("rocker", "link", "horn", "servo", "frame", "clamp"):
            volume = intersection_volume(parts[moving], parts["cans"])
            checks.append({
                "state": state,
                "pair": f"{moving}/cans",
                "intersection_mm3": None if math.isnan(volume) else round(volume, 6),
                "pass": (not math.isnan(volume)) and volume < 0.01,
            })
        # These pairs exclude their intentional pin/screw joints.
        for a, b in (("frame", "rocker"), ("frame", "link"), ("frame", "horn"),
                     ("frame", "servo"), ("clamp", "rocker"), ("servo", "rocker")):
            volume = intersection_volume(parts[a], parts[b])
            checks.append({
                "state": state,
                "pair": f"{a}/{b}",
                "intersection_mm3": None if math.isnan(volume) else round(volume, 6),
                "pass": (not math.isnan(volume)) and volume < 0.01,
            })

    # Adjacent channels: all solid mechanism parts at every key state.
    neighbor = []
    for state in STATES:
        p0 = one_lane_state_parts(state)
        p1 = {k: translate(v, (0, P["lane_pitch"], 0)) for k, v in p0.items() if k not in ("deck", "cans")}
        m0 = trimesh.util.concatenate([p0[k] for k in ("frame", "clamp", "rocker", "link", "servo", "horn")])
        m1 = trimesh.util.concatenate(list(p1.values()))
        vol = intersection_volume(m0, m1)
        neighbor.append({
            "state": state,
            "pitch_mm": P["lane_pitch"],
            "intersection_mm3": None if math.isnan(vol) else round(vol, 6),
            "pass": (not math.isnan(vol)) and vol < 0.01,
            "aabb_gap_y_mm": round(float(m1.bounds[0, 1] - m0.bounds[1, 1]), 3),
        })
    # The frame slots were generated from the same kinematic path.  Confirm
    # them with independent solid booleans at 13 positions, including the two
    # hard-stop endpoints and intermediate servo positions.
    continuous = []
    fixed = {"frame": frame_mesh(), "clamp": clamp_mesh(), "servo": servo_reference()}
    for sd in np.linspace(0.0, P["servo_motion_deg"], 13):
        q = rocker_q_for_servo(float(sd))
        moving = {
            "rocker": rocker_mesh(q),
            "link": link_state_mesh(q, float(sd)),
            "horn": horn_reference(float(sd)),
        }
        for a, b in (("frame", "rocker"), ("frame", "link"), ("frame", "horn"),
                     ("clamp", "rocker"), ("servo", "rocker")):
            volume = intersection_volume(fixed[a], moving[b])
            continuous.append({
                "servo_deg": round(float(sd), 4),
                "rocker_deg": round(float(q), 5),
                "pair": f"{a}/{b}",
                "intersection_mm3": None if math.isnan(volume) else round(volume, 6),
                "pass": (not math.isnan(volume)) and volume < 0.01,
            })
    # Direct placement against the exact shared cassette files, not boxes or
    # shortened stand-ins: 3 states × 5 lanes × 6 roles × 5 cassette targets.
    exact_lower, exact_dividers = exact_cassette_local()
    lane_offsets = (-158.0, -79.0, 0.0, 79.0, 158.0)
    exact_checks = []
    exact_lane_maxima = {f"lane_{index}": 0.0 for index in range(1, 6)}
    for state in STATES:
        source = one_lane_state_parts(state)
        for lane_index, y_offset in enumerate(lane_offsets, start=1):
            lane_key = f"lane_{lane_index}"
            for part_name in ("frame", "clamp", "rocker", "link", "servo", "horn"):
                part = translate(source[part_name], (0.0, y_offset, 0.0))
                targets = [("exact_lower_sheet", exact_lower)] + [
                    (f"exact_divider_{idx}", divider)
                    for idx, divider in enumerate(exact_dividers, start=1)
                ]
                for target_name, target in targets:
                    volume = intersection_volume(part, target)
                    passed = (not math.isnan(volume)) and volume < 0.01
                    exact_checks.append({
                        "state": state, "lane": lane_index, "part": part_name,
                        "target": target_name,
                        "intersection_mm3": None if math.isnan(volume) else round(volume, 6),
                        "pass": passed,
                    })
                    if not math.isnan(volume):
                        exact_lane_maxima[lane_key] = max(exact_lane_maxima[lane_key], volume)
    exact_paths = [COMMON_STL_DIR / "cassette_lower_sheet.stl",
                   COMMON_STL_DIR / "cassette_divider.stl"]
    exact_report = {
        "sources": [{"path": "../" + path.relative_to(HERE.parent).as_posix(),
                     "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
                    for path in exact_paths],
        "pair_count": len(exact_checks),
        "states": list(STATES),
        "lanes": 5,
        "lane_pitch_mm": P["lane_pitch"],
        "divider_count": len(exact_dividers),
        "maximum_intersection_by_lane_mm3": {
            key: round(value, 6) for key, value in exact_lane_maxima.items()
        },
        "checks": exact_checks,
        "pass": all(item["pass"] for item in exact_checks),
    }
    return {"key_state_pairs": checks, "drive_sweep_13_positions": continuous,
            "adjacent_channels": neighbor, "exact_common_cassette": exact_report}


def mounting_report():
    frame = frame_mesh()
    clamp = clamp_mesh()
    servo = servo_reference()
    deck, dividers = exact_cassette_local()
    fdeck = intersection_volume(frame, deck)
    cdeck = intersection_volume(clamp, deck)
    fclamp = intersection_volume(frame, clamp)
    fservo = intersection_volume(frame, servo)
    supported = P["shelf_supported_depth"]
    overhang = P["box_inside_depth"] - supported
    return {
        "common_lower_sheet": {
            "source": "../00_ОБЩАЯ_КОМПОНОВКА/STL_КАССЕТА_СБОРКА/cassette_lower_sheet.stl",
            "thickness_mm": P["sheet_thickness"],
            "front_edge_local_u_mm": round(float(deck.bounds[1, 0]), 6),
            "local_bounds_mm": [[round(float(v), 6) for v in row] for row in deck.bounds],
        },
        "identical_all_five_channels": True,
        "internal_divider_count": 4,
        "exact_divider_source": "../00_ОБЩАЯ_КОМПОНОВКА/STL_КАССЕТА_СБОРКА/cassette_divider.stl",
        "exact_divider_outlet_extension_beyond_lower_sheet_mm": round(
            float(max(item.bounds[1, 0] for item in dividers) - deck.bounds[1, 0]), 6
        ),
        "extra_structural_vertical_sheets_required": 0,
        "existing_box_side_walls_used_as_guides": 2,
        "dividers_used_for_structure": False,
        "attachment": "A01+A04 clamp the common lower-sheet front edge with two M4 screws outside the sheet; no sheet holes",
        "designed_clearances_mm": {
            "frame_relief_past_sheet_edge": 0.25,
            "upper_leadin_above_sheet": 0.30,
            "lower_pad_below_sheet": 0.20,
        },
        "solid_intersections_mm3": {
            "frame_lower_sheet": None if math.isnan(fdeck) else round(fdeck, 6),
            "clamp_lower_sheet_before_tightening": None if math.isnan(cdeck) else round(cdeck, 6),
            "frame_clamp_before_screws_tightened": None if math.isnan(fclamp) else round(fclamp, 6),
            "frame_servo_installed": None if math.isnan(fservo) else round(fservo, 6),
        },
        "servo_snap_fit": {
            "output_boss_nominal_diameter_mm": 4.8,
            "output_boss_frame_bore_mm": 5.6,
            "ear_slot_clearance_x_total_mm": 1.2,
            "ear_slot_clearance_y_each_side_mm": 0.4,
            "ear_slot_clearance_n_each_side_mm": 0.4,
            "note": "Side walls flex only during insertion; the ears occupy non-intersecting capture pockets in the installed position.",
        },
        "box_shelf_layout_mm": {
            "box_depth": P["box_inside_depth"],
            "shelf_supported_depth": supported,
            "front_overhang": overhang,
            "separate_beam_or_plinth": False,
        },
        "pass": all((not math.isnan(v)) and v < 0.01 for v in (fdeck, cdeck, fclamp, fservo)),
    }


def write_3mf(filename, items):
    scene = trimesh.Scene()
    for name, mesh, x, y in items:
        placed = mesh.copy()
        placed.apply_translation([x - placed.bounds[0, 0], y - placed.bounds[0, 1], -placed.bounds[0, 2]])
        scene.add_geometry(placed, node_name=name, geom_name=name)
    payload = scene.export(file_type="3mf")
    (HERE / filename).write_bytes(payload)


def write_tables(print_reports):
    bom_rows = [
        ("A01", "Рама с упорами и защёлкой SG90", 5, "PETG", "1 на канал; печать внешней плоскостью вниз"),
        ("A02", "Коромысло с 2 округлыми пальцами", 5, "PETG/PA", "1 на канал; пальцы печатаются вертикально"),
        ("A03", "Короткая тяга 11.63 мм", 5, "PETG/PA", "1 на канал; один конец защёлкивается"),
        ("A04", "Нижний прижим передней кромки ската", 5, "PETG", "1 на канал; два M4, лист не сверлить"),
        ("S01", "SG90 или MG90S + штатная двойная качалка", 5, "имеется у владельца", "внешнее отверстие horn около 9 мм"),
        ("H01", "M4 переплётный винт/shoulder post, рабочая длина 4.5–5 мм", 5, "сталь", "главный pivot; тонкие фланцы"),
        ("H02", "M4×16 винт + самоконтрящаяся гайка", 10, "сталь", "2 на канал: без отверстий в скате, за его передней кромкой"),
        ("H03", "M2 шарнирный винт/палец Ø2×6 с гладкой рабочей шейкой и фиксатором", 5, "сталь", "гладкая шейка работает в Ø2.3 отверстии A03; резьба не должна тереть PETG"),
        ("H04", "Штатный винт качалки к шлицу сервы", 5, "из комплекта SG90/MG90S", "повторно использовать; отдельная покупка не нужна"),
        ("E01", "Резистор 10 кОм 0.125–0.25 Вт", 5, "электроника", "сигнал каждой сервы → GND; исключает плавающий вход при старте ESP32"),
        ("C01", "Нижний скат 15°", 1, "берёзовая фанера 4 мм", "примерно 590×391; без силовой балки снизу"),
        ("C02", "Внутренний разделитель канала", 4, "берёзовая фанера 4 мм", "чистый канал 75 мм; только боковое направление банок"),
        ("C03", "Боковые стенки коробки", 2, "существующая коробка/фанера", "работают как крайние разделители"),
        ("L01", "UHMW/PET самоклеящаяся лента", 5, "опционально", "полоса по дорожке уменьшает зависание последней банки"),
    ]
    with (HERE / "BOM_вариант_A.csv").open("w", encoding="utf-8-sig", newline="") as fh:
        wr = csv.writer(fh, delimiter=";")
        wr.writerow(("Код", "Наименование", "Количество", "Материал", "Примечание"))
        wr.writerows(bom_rows)

    with (HERE / "КАРТА_ПЕЧАТИ.csv").open("w", encoding="utf-8-sig", newline="") as fh:
        wr = csv.writer(fh, delimiter=";")
        wr.writerow(("Файл", "Количество на канал", "Всего на 5", "Сопла/слой", "Поддержки"))
        for item in print_reports:
            wr.writerow((item["file"], 1, 5, "0.4 / 0.20 мм", "нет"))


def write_readme(forces, barrier):
    load_lines = "\n".join(
        f"| {row['mass_each_g']} г | {row['strict_quasistatic_max_cans_at_0_06Nm']} | "
        f"{row['peak_at_max_Nm']:.4f} Н·м |"
        for row in forces["load_table"]
    )
    handoff = barrier["moving_can_handoff"]
    text = f"""# V11 — вариант A: верхнее коромысло

**Статус: ПРОТОТИП ДЛЯ СРАВНЕНИЯ. Сначала напечатать и испытать один канал минимум на 300 циклах.**

## Что здесь принципиально другое

Одна SG90/MG90S через штатную двойную качалку и короткую тягу качает одно V-образное коромысло. В HOME передний палец держит первую банку, в EJECT он поднимается, а задний уже удерживает вторую. Жёсткие упоры ограничивают аварийный перелёт механизма, но эта геометрия **не является пассивной защёлкой**: пока в канале остаётся хотя бы одна банка, серва должна получать питание и непрерывный корректный PWM 50 Гц. Одного питания без управляющих импульсов недостаточно.

Коробка остаётся глубиной около 600 мм и частично нависает перед полкой. Каждый из пяти одинаковых модулей зажимается A01+A04 двумя M4 за **общую переднюю кромку нижнего ската**, без сверления листа. Четыре внутренних разделителя только направляют банки и ничего не держат. Отдельная балка, пятый/шестой разделитель и подиум не нужны.

## Печать одного канала

- A01, A02, A03, A04 — по одной детали; все уже лежат правильной стороной.
- PETG или PA, сопло 0.4, слой 0.20, 4 периметра, 35–45% gyroid.
- Поддержки не нужны. Пальцы A02 печатаются вертикально, поэтому не являются мостами.
- Для высокой A01 обязателен brim 4 мм: PrusaSlicer предупреждает о малой площади контакта со столом. Раскладка оставляет под такой brim не меньше 1 мм свободного поля до края стола и не сводит соседние детали.
- Один комплект помещается на `A_ПЛИТА_220x220_1_канал.3mf` с полем не меньше 5 мм.

## Ровно 8 сгруппированных действий от снятия с печати до закреплённого модуля

Правило общее для A/B/C: одна макрооперация — один осознанный этап с единым результатом. Парные одинаковые элементы, устанавливаемые или затягиваемые в рамках такого этапа (например, два винта сервы, две оси/гайки или два нижних винта), считаются одной сгруппированной макрооперацией; электропроводка пяти серв считается отдельно.

1. Осмотреть и снять только заусенцы с четырёх печатных деталей.
2. Защёлкнуть SG90/MG90S в A01.
3. Поставить A02 на один M4 переплётный pivot, не пережимая шарнир.
4. Защёлкнуть большой конец A03 на печатный палец A02.
5. Закрепить малый конец A03 одним M2 шарнирным винтом/пальцем с **гладкой рабочей шейкой** во внешнем отверстии штатного horn (радиус около 9 мм); резьба не должна скользить по PETG.
6. Выставить HOME и закрепить horn штатным винтом на шлице сервы.
7. Одним движением завести A01 сверху и A04 снизу на общую переднюю кромку ската.
8. Вставить два комплекта M4×16 с самоконтрящимися гайками и равномерно попеременно затянуть их за торцом листа.

После крепления без банок проверить ручной ход между упорами и откалибровать импульсы. Микросекунды другого канала вслепую не копировать.

## Геометрия

- design envelope банки: Ø{P['can_diameter_design']:.0f}×{P['can_length']:.0f} мм;
- чистый канал: {P['lane_clear']:.0f} мм, шаг каналов {P['lane_pitch']:.0f} мм;
- нижний скат: {P['lower_deck_angle_deg']:.0f}°;
- коромысло: {P['rocker_home_deg']:.0f}…+{P['rocker_eject_deg']:.0f}°;
- серва: {P['servo_motion_deg']:.2f}°;
- тяга: {P['link_length']:.3f} мм между центрами;
- пальцы Ø{2*P['finger_radius']:.1f} мм, вылет {P['finger_reach']:.1f} мм.

Численная проверка движения банки, а не замороженных демонстрационных цилиндров: задний палец принимает упакованную вторую банку при {handoff['rear_contacts_packed_can2']['servo_deg']:.2f}° сервы, передний освобождает первую при {handoff['front_releases_can1']['servo_deg']:.2f}°. Положительное перекрытие передачи — {handoff['positive_handoff_overlap_servo_deg']:.2f}°, минимальный свободный коридор центра первой банки между двумя барьерами — {handoff['minimum_can1_centre_corridor_between_gates_mm']:.2f} мм, минимальный шаг центров первой/второй после передачи — {handoff['minimum_can1_can2_centre_spacing_after_transfer_mm']:.2f} мм при требуемых {handoff['required_can1_can2_centre_spacing_mm']:.2f} мм.

## Что обязательно проверить на физическом прототипе

- фактический радиус отверстия штатного horn (комплекты SG90 бывают 8–10 мм);
- люфт pivot и тяги после 100 циклов;
- H03: гладкая Ø2 мм шейка должна работать в Ø2.3 отверстии A03, а резьбовая часть находиться только в зоне фиксации. Обычный полностью резьбовой M2 допустим лишь для макета и считается точкой износа с осмотром каждые 300 циклов;
- не цепляет ли банка Ø48 палец при перекосе;
- хватает ли Ø4 пальца при полной очереди и ударе последней банки;
- потеря питания или PWM при загруженном канале может провернуть коромысло; перед снятием питания или сигнала канал нужно полностью разгрузить.

## Честное ограничение нагрузки

Лимит определён по **пику**, а не по более красивому среднему значению. 22×100 г дают {forces['target_cases']['22x100g']['peak_servo_torque_Nm']:.4f} Н·м, 11×200 г — {forces['target_cases']['11x200g']['peak_servo_torque_Nm']:.4f} Н·м: обе контрольные загрузки укладываются в 0.060 Н·м. Полные 22×200 г требуют {forces['target_cases']['22x200g']['peak_servo_torque_Nm']:.4f} Н·м и запрещены.

| Масса банки | Строгий расчётный максимум до 0.060 Н·м | Пик на максимуме |
|---:|---:|---:|
{load_lines}

Это квазистатика без трения, ударов и изгиба пластика. Первый физический тест начинать максимум с {forces['prototype_recommended_initial_max_cans_200g']} банок по 200 г и повышать загрузку только после 100 циклов.

## Питание и безопасный запуск

На каждый оранжевый сигнальный провод поставить внешний 10 кОм на GND. Запуск: 5 В серв выключены → загрузить ESP32 и держать сигнальные пины LOW → запустить корректный PWM 50 Гц и подать калиброванный HOME → включить 5 В серв → только затем загрузить банки. Пока в канале есть банки, PWM должен непрерывно присутствовать в HOME, EJECT и во время движения. Выключение: остановить выдачу → HOME → **полностью разгрузить канал** → снять 5 В серв → только после снятия 5 В прекратить PWM → выключить контроллер. Потеря питания или сигнала с банками безопасной не считается.

`ПРОТОТИП_ОТЧЕТ_ПРОВЕРКИ.json` содержит воспроизводимые численные проверки. STL сборки предназначены только для просмотра, печатать их не нужно.
"""
    (HERE / "README_ВАРИАНТ_A.md").write_text(text, encoding="utf-8")


def main():
    # Remove only development renders produced by this generator's audit;
    # they are not part of the deliverable package.
    for temporary_name in ("audit_home.png", "audit_local.png"):
        temporary = HERE / temporary_name
        if temporary.exists():
            temporary.unlink()
    print_parts = {
        "A01_рама_верхнего_коромысла.stl": side_part_on_bed(frame_mesh()),
        "A02_коромысло_с_2_пальцами.stl": side_part_on_bed(rocker_mesh(0.0)),
        "A03_короткая_тяга_12мм.stl": side_part_on_bed(link_print_mesh()),
        "A04_нижний_прижим_ската.stl": clean(translate(clamp_mesh(), (0, 0, -clamp_mesh().bounds[0, 2]))),
    }
    # The generator owns only these two output folders.  Remove previous STL
    # exports so renamed/stale parts cannot silently inflate manifest counts.
    for directory in (PRINT_DIR, ASSEMBLY_DIR):
        for old in directory.glob("*.stl"):
            old.unlink()
    print_reports = []
    for filename, mesh in print_parts.items():
        print_reports.append(export_mesh(mesh, PRINT_DIR / filename))

    assembly_reports = []
    for state in STATES:
        assembly_reports.append(
            export_mesh(assembly_state_mesh(state), ASSEMBLY_DIR / f"A_ОДИН_КАНАЛ_{state}_3_банки.stl")
        )
    assembly_reports.append(export_mesh(five_lane_mesh("HOME"), ASSEMBLY_DIR / "A_5_КАНАЛОВ_HOME.stl"))
    assembly_reports.append(export_mesh(installation_reference_mesh(), ASSEMBLY_DIR / "A_КРЕПЛЕНИЕ_КОРОБКА_НАВИСАЕТ_НАД_ПОЛКОЙ.stl"))

    viewer_assets = []
    colors = {
        "frame": "#4F83CC", "clamp": "#5BA0DF", "rocker": "#FF8A2B",
        "link": "#E8CF42", "servo": "#325A96", "horn": "#D8DCE3",
        "lower_sheet": "#B48750", "can_1": "#2FBD79", "can_2": "#35C984",
        "can_3": "#45D994",
    }
    for state in STATES:
        parts = one_lane_state_parts(state)
        for role, source in (("frame", "frame"), ("clamp", "clamp"), ("rocker", "rocker"),
                             ("link", "link"), ("servo", "servo"), ("horn", "horn"),
                             ("lower_sheet", "deck")):
            filename = f"A_VIEW_{state}_{role}.stl"
            mesh_report = export_mesh(slope_to_box(parts[source]), ASSEMBLY_DIR / filename)
            viewer_assets.append({
                "filename": filename, "role": role, "state": state,
                "color": colors[role], "mesh": mesh_report,
            })
        for index, can in enumerate(state_cans(state), start=1):
            role = f"can_{index}"
            filename = f"A_VIEW_{state}_{role}.stl"
            mesh_report = export_mesh(slope_to_box(can), ASSEMBLY_DIR / filename)
            viewer_assets.append({
                "filename": filename, "role": role, "state": state,
                "color": colors[role], "mesh": mesh_report,
            })

    plate_items = [
        ("A01_frame", print_parts["A01_рама_верхнего_коромысла.stl"], 5.0, 5.0),
        ("A02_rocker", print_parts["A02_коромысло_с_2_пальцами.stl"], 5.0, 115.0),
        ("A04_clamp", print_parts["A04_нижний_прижим_ската.stl"], 75.0, 115.0),
        ("A03_link", print_parts["A03_короткая_тяга_12мм.stl"], 110.0, 115.0),
    ]
    write_3mf("A_ПЛИТА_220x220_1_канал.3mf", plate_items)
    plate_report = print_plate_report(plate_items)

    sweep = analytic_sweep_report()
    barrier = continuous_barrier_report()
    forces = force_report()
    collisions = collision_report()
    mounting = mounting_report()
    print_ok = all(
        r["watertight"] and r["is_volume"] and r["components"] == 1
        and r["boundary_edges"] == 0 and r["overused_edges"] == 0
        for r in print_reports
    )
    plate_ok = all(r["inside"] and not r["overlaps"] for r in plate_report)
    collision_ok = all(x["pass"] for x in (
        collisions["key_state_pairs"] + collisions["drive_sweep_13_positions"] + collisions["adjacent_channels"]
    )) and collisions["exact_common_cassette"]["pass"]
    geometry_ok = bool(print_ok and plate_ok and collision_ok and barrier["pass"] and mounting["pass"])
    report = {
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "status": P["status"],
        "prototype_warning": "Сначала один канал и 300 циклов; не печатать сразу все пять комплектов.",
        "print_meshes": print_reports,
        "print_meshes_all_watertight_one_component": print_ok,
        "bed_220x220": {"items": plate_report, "pass": plate_ok},
        "kinematic_sweep": sweep,
        "continuous_can_barrier_proof": barrier,
        "force_validation": forces,
        "collisions": collisions,
        "collision_pass": collision_ok,
        "mounting_and_support_chain": mounting,
        "overall_geometry_pass": geometry_ok,
        "prototype_ready_for_one_lane_test": geometry_ok,
        "production_ready": False,
        "production_blocker": "Prototype endurance and power-loss containment remain unverified; 22×200 g exceeds the 0.06 N·m target.",
    }
    (HERE / "ПРОТОТИП_ОТЧЕТ_ПРОВЕРКИ.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    assembly_steps = [
        "Осмотреть и снять только заусенцы с четырёх печатных деталей.",
        "Защёлкнуть SG90/MG90S в A01.",
        "Поставить A02 на один главный M4 pivot.",
        "Защёлкнуть большой конец A03 на палец A02.",
        "Закрепить малый конец A03 одним M2 шарнирным винтом/пальцем с гладкой рабочей шейкой во внешнем отверстии штатного horn.",
        "Выставить HOME и закрепить штатный horn штатным винтом на шлице сервы.",
    ]
    attachment_steps = [
        "Одним движением завести A01 сверху и A04 снизу на общую переднюю кромку ската.",
        "Вставить два комплекта M4×16 с самоконтрящимися гайками и равномерно попеременно затянуть их за торцом листа.",
    ]
    actual_print_stls = len(list(PRINT_DIR.glob("*.stl")))
    actual_assembly_stls = len(list(ASSEMBLY_DIR.glob("*.stl")))
    manifest = {
        "name": "V11 A — верхнее коромысло",
        "status": P["status"],
        "prototype": True,
        "production_ready": False,
        "prototype_ready_for_one_lane_test": geometry_ok,
        "path_base": ".",
        "path_base_definition": "directory containing manifest_A.json",
        "path_roots": {
            "print_files[].file": "STL_ПЕЧАТЬ",
            "assembly_files[].file": "STL_СБОРКА_НЕ_ПЕЧАТАТЬ",
            "viewer_assets[].items[].file": "STL_СБОРКА_НЕ_ПЕЧАТАТЬ",
            "plate_3mf and validation_report": ".",
            "explicit paths with directories": ".",
        },
        "parameters_mm_deg": P,
        "states": {name: {k: round(float(v), 6) for k, v in spec.items()} for name, spec in STATES.items()},
        "print_parts_count": {"per_lane": 4, "for_5_lanes": 20},
        "print_parts_count_per_lane": 4,
        "print_parts_count_for_5_lanes": 20,
        "fasteners_count": {"per_lane_total_including_stock_spline_screw": 5, "user_added_per_lane": 4},
        "fasteners_count_per_lane": 5,
        "fasteners_breakdown_per_lane": {"M4_pivot": 1, "M4_attachment": 2, "M2_horn_link": 1, "stock_servo_spline_screw": 1},
        "supports": "none",
        "brim_mm": 4.0,
        "adhesion_note": "Brim 4 mm required for tall A01; verified plate edge and part spacing remain clear.",
        "assembly_operations": {"count_before_attachment": len(assembly_steps), "steps": assembly_steps},
        "attachment_operations": {"count": len(attachment_steps), "steps": attachment_steps},
        "action_counting_basis": "one deliberate assembly step with one outcome; matched identical parts or fasteners handled in that step count as one grouped macro-operation; wiring is separate",
        "total_operations_print_removal_to_mounted_per_lane": len(assembly_steps) + len(attachment_steps),
        "assembly_operations_per_lane": len(assembly_steps),
        "attachment_operations_per_lane": len(attachment_steps),
        "stock_horn": True,
        "separate_beam_or_plinth": False,
        "print_files": print_reports,
        "assembly_files": assembly_reports,
        "viewer_assets": viewer_assets,
        "actual_stl_counts": {"print_folder": actual_print_stls, "assembly_folder": actual_assembly_stls},
        "exact_common_cassette_validation": {
            "sources": collisions["exact_common_cassette"]["sources"],
            "pair_count": collisions["exact_common_cassette"]["pair_count"],
            "states": collisions["exact_common_cassette"]["states"],
            "lanes": collisions["exact_common_cassette"]["lanes"],
            "maximum_intersection_by_lane_mm3": collisions["exact_common_cassette"]["maximum_intersection_by_lane_mm3"],
            "pass": collisions["exact_common_cassette"]["pass"],
        },
        "moving_can_handoff_validation": barrier["moving_can_handoff"],
        "plate_3mf": "A_ПЛИТА_220x220_1_канал.3mf",
        "validation_report": "ПРОТОТИП_ОТЧЕТ_ПРОВЕРКИ.json",
        "force_validation": {
            "full_22x200g_pass": forces["full_22x200g_pass"],
            "full_22x100g_pass": forces["full_22x100g_pass"],
            "11x200g_pass": forces["11x200g_pass"],
            "strict_quasistatic_max_cans_200g": forces["strict_quasistatic_max_cans_200g"],
            "recommended_initial_max_cans_200g": forces["prototype_recommended_initial_max_cans_200g"],
            "strict_load_table": forces["load_table"],
            "power_off_self_hold": forces["endpoint_load_direction"]["power_off_self_hold"],
            "report": "ПРОТОТИП_ОТЧЕТ_ПРОВЕРКИ.json",
        },
        "power_safety": forces["power_safety"],
        "critical_note": "Prototype comparison geometry; print/test one lane with the stated load limit before multiplying by five.",
    }
    (HERE / "manifest_A.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    write_tables(print_reports)
    write_readme(forces, barrier)

    print(json.dumps({
        "print_parts": len(print_reports),
        "watertight_one_component": print_ok,
        "plate_220_pass": plate_ok,
        "collision_pass": collision_ok,
        "barrier_pass": barrier["pass"],
        "mounting_pass": mounting["pass"],
        "full_22x200g_force_pass": forces["full_22x200g_pass"],
        "full_22x100g_force_pass": forces["full_22x100g_pass"],
        "11x200g_force_pass": forces["11x200g_pass"],
        "overall_geometry_pass": report["overall_geometry_pass"],
        "prototype_ready_for_one_lane_test": report["prototype_ready_for_one_lane_test"],
        "actual_print_stls": actual_print_stls,
        "actual_assembly_stls": actual_assembly_stls,
        "mid_rocker_deg": STATES["MID"]["rocker_deg"],
    }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
