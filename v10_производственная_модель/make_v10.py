# -*- coding: utf-8 -*-
"""V10 — production-oriented five-lane can dispenser.

The script creates:
  * printable STL parts for one SG90/MG90S-powered lane;
  * exact six-sheet cassette solids and cutting drawings;
  * assembly-coordinate STL parts for the interactive viewer;
  * BOM/print maps and a machine-readable manifest.

Axes used by the gate module:
  X = across the lane / along the can axis
  Y = downstream (towards the customer)
  Z = normal to the sloped deck

Axes used by the box:
  X = depth, 0 at the front wall, 590 at the rear
  Y = width, left to right
  Z = vertical
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
CUT_DIR = HERE / "РАСКРОЙ_6_ЛИСТОВ"
for directory in (PRINT_DIR, ASSEMBLY_DIR, CUT_DIR):
    directory.mkdir(parents=True, exist_ok=True)


# ---------------------------------------------------------------------------
# Master dimensions (millimetres)
# ---------------------------------------------------------------------------
P = {
    "can_length": 70.0,
    "can_diameter_nominal": 47.0,
    "can_diameter_design": 48.0,
    "box_inside_depth": 595.0,
    "box_inside_width": 395.0,
    "box_inside_height": 395.0,
    "cassette_depth": 590.0,
    "cassette_width": 391.0,
    "cassette_height": 390.0,
    "sheet_thickness": 4.0,
    "sheet_slot_clearance_cnc": 0.25,
    "sheet_slot_clearance_manual": 0.45,
    "lane_clear": 75.0,
    "lane_pitch": 79.0,
    "divider_centres_from_cassette_left": [77.0, 156.0, 235.0, 314.0],
    "lower_x0": 40.0,
    "lower_x1": 590.0,
    "lower_angle_deg": 15.0,
    "upper_x0": 0.0,
    "upper_x1": 535.0,
    "upper_front_height": 345.0,
    "upper_angle_deg": 14.0,
    "module_width": 78.4,
    "module_length": 210.0,
    "module_y_min": -140.0,
    "module_y_max": 70.0,
    "gate_rear_box_x": -15.0,
    "gate_front_y": 0.0,
    "gate_rear_y": -50.5,
    "blade_open_z": -2.0,
    "blade_closed_z": 26.0,
    "blade_thickness": 4.0,
    "blade_working_width": 66.5,
    "blade_guide_gap": 4.55,
    "floor_slot_width": 4.8,
    "servo_cam_axis_y": -25.0,
    "servo_cam_axis_z": -52.0,
    "cam_angle_deg": 84.0,
    "cam_profile_margin_deg": 4.0,
    "cam_rear_close_end_deg": 46.0,
    "cam_front_open_start_deg": 52.0,
    "cam_front_open_end_deg": 82.0,
    "cam_radius": 48.5,
    "cam_thickness": 5.4,
    "cam_slot_width": 6.8,
    "cam_follower_diameter": 6.0,
    "blade_pin_below_tip": 45.0,
    "follower_y_offset": 2.1,
    "blade_boss_guide_gap": 8.7,
    "horn_pocket_depth": 3.1,
}


# ---------------------------------------------------------------------------
# Mesh helpers
# ---------------------------------------------------------------------------
def _clean(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    mesh = mesh.copy()
    mesh.remove_unreferenced_vertices()
    mesh.merge_vertices()
    trimesh.repair.fix_normals(mesh)
    return mesh


def fuse(meshes: Iterable[trimesh.Trimesh]) -> trimesh.Trimesh:
    items = [_clean(m) for m in meshes if m is not None and len(m.faces)]
    if not items:
        raise ValueError("No meshes to fuse")
    if len(items) == 1:
        return items[0]
    try:
        result = trimesh.boolean.union(items, engine="manifold")
        if result is not None:
            return _clean(result)
    except BaseException:
        pass
    return _clean(trimesh.util.concatenate(items))


def cut(mesh: trimesh.Trimesh, cutters: Iterable[trimesh.Trimesh]) -> trimesh.Trimesh:
    tools = [_clean(c) for c in cutters if c is not None and len(c.faces)]
    if not tools:
        return _clean(mesh)
    try:
        result = trimesh.boolean.difference([_clean(mesh), *tools], engine="manifold")
        if result is not None:
            return _clean(result)
    except BaseException as exc:
        raise RuntimeError(f"Boolean cut failed: {exc}") from exc
    raise RuntimeError("Boolean cut returned no mesh")


def bx(size, center=(0.0, 0.0, 0.0)) -> trimesh.Trimesh:
    mesh = trimesh.creation.box(extents=np.asarray(size, dtype=float))
    mesh.apply_translation(np.asarray(center, dtype=float))
    return mesh


def cylinder_x(radius, length, center=(0.0, 0.0, 0.0), sections=48) -> trimesh.Trimesh:
    mesh = trimesh.creation.cylinder(radius=radius, height=length, sections=sections)
    # Default cylinder is Z; rotate it to X.
    mesh.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0]))
    mesh.apply_translation(np.asarray(center, dtype=float))
    return mesh


def cylinder_y(radius, length, center=(0.0, 0.0, 0.0), sections=48) -> trimesh.Trimesh:
    mesh = trimesh.creation.cylinder(radius=radius, height=length, sections=sections)
    mesh.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0]))
    mesh.apply_translation(np.asarray(center, dtype=float))
    return mesh


def extrude_yz(poly, thickness, x0=0.0) -> trimesh.Trimesh:
    """Extrude a Shapely polygon defined as (Y,Z) along global X."""
    mesh = trimesh.creation.extrude_polygon(poly, height=thickness)
    # local (x,y,z) = (Y,Z,extrusion); map to global (X,Y,Z)=(extrusion,Y,Z)
    transform = np.array(
        [[0, 0, 1, x0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]],
        dtype=float,
    )
    mesh.apply_transform(transform)
    return _clean(mesh)


def extrude_xz(poly, thickness, y0=0.0) -> trimesh.Trimesh:
    """Extrude a Shapely polygon defined as (X,Z) along global Y."""
    mesh = trimesh.creation.extrude_polygon(poly, height=thickness)
    # local (x,y,z)=(X,Z,extrusion) -> global (X,Y,Z)=(X,extrusion,Z)
    transform = np.array(
        [[1, 0, 0, 0], [0, 0, 1, y0], [0, 1, 0, 0], [0, 0, 0, 1]],
        dtype=float,
    )
    mesh.apply_transform(transform)
    return _clean(mesh)


def rotate(mesh, angle_deg, axis, point=(0.0, 0.0, 0.0)) -> trimesh.Trimesh:
    result = mesh.copy()
    result.apply_transform(
        trimesh.transformations.rotation_matrix(math.radians(angle_deg), axis, point)
    )
    return result


def translate(mesh, xyz) -> trimesh.Trimesh:
    result = mesh.copy()
    result.apply_translation(np.asarray(xyz, dtype=float))
    return result


def on_bed(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    result = _clean(mesh)
    result.apply_translation([0, 0, -result.bounds[0, 2]])
    return result


def export_mesh(mesh: trimesh.Trimesh, path: Path) -> dict:
    mesh = _clean(mesh)
    mesh.export(path)
    ext = mesh.extents
    return {
        "file": path.name,
        "extents_mm": [round(float(v), 2) for v in ext],
        "watertight": bool(mesh.is_watertight),
        "components": int(len(mesh.split(only_watertight=False))),
        "volume_cm3": round(abs(float(mesh.volume)) / 1000.0, 2),
    }


def rounded_rectangle(width, height, radius) -> Polygon:
    return sbox(-width / 2 + radius, -height / 2 + radius,
                width / 2 - radius, height / 2 - radius).buffer(radius, resolution=24)


def d_bushing_profile(clearance=0.0):
    """D-shaped M09 outside profile in the local Y-Z plane."""
    radius = 3.90 + clearance
    flat_z = 3.05 + clearance
    return Point(0, 0).buffer(radius, resolution=48).intersection(
        sbox(-10.0, -10.0, 10.0, flat_z))


# ---------------------------------------------------------------------------
# Servo, horn, cans, fasteners (assembly reference solids; not printed)
# ---------------------------------------------------------------------------
def horn_outline(clearance=0.0):
    hub_r = 3.7 + clearance
    arm_len = 15.8 + clearance
    root_w = 6.9 + 2 * clearance
    tip_w = 4.0 + 2 * clearance
    hub = Point(0, 0).buffer(hub_r, resolution=24)
    arms = []
    for sign in (-1, 1):
        poly = Polygon([
            (sign * 3.0, root_w / 2),
            (sign * arm_len, tip_w / 2),
            (sign * arm_len, -tip_w / 2),
            (sign * 3.0, -root_w / 2),
        ])
        arms.append(poly)
        arms.append(Point(sign * arm_len, 0).buffer(tip_w / 2, resolution=16))
    return unary_union([hub, *arms])


def servo_reference_parts():
    """Approximate SG90/MG90S using the project mounting envelope."""
    cy = P["servo_cam_axis_y"]
    cz = P["servo_cam_axis_z"]
    # Shaft faces left.  The mounting flange touches (rather than intersects)
    # the right face of the printed servo wall at X=-25.5.
    # The servo is rotated 90° around its output axis: its narrow 12.2 mm side
    # lies between the two gate planes.  This is what makes a standard micro
    # servo physically fit between gates 50 mm apart.
    body = bx((28.5, 12.2, 22.8), (-8.65, cy, cz + 5.4))
    top_cap = bx((2.6, 12.8, 23.4), (-24.2, cy, cz + 5.4))
    flange = bx((2.6, 15.0, 32.5), (-24.2, cy, cz + 5.4))
    boss = cylinder_x(5.8, 3.0, (-27.0, cy, cz), sections=40)
    shaft = cylinder_x(2.35, 5.0, (-30.5, cy, cz), sections=32)
    # The real stock horn sits inside the moulded pocket on the cam's inner face.
    horn_shape = affinity.rotate(horn_outline(), 90, origin=(0, 0))
    horn = extrude_yz(horn_shape, 1.8, x0=-33.35)
    horn = translate(horn, (0, cy, cz))
    screw = cylinder_x(1.0, 7.0, (-34.2, cy, cz), sections=24)
    screw_head = cylinder_x(2.2, 1.2, (-38.3, cy, cz), sections=24)
    return {
        "servo_body": fuse([body, top_cap, flange, boss]),
        "servo_shaft": shaft,
        "servo_horn": horn,
        "servo_screw": fuse([screw, screw_head]),
    }


def can_parts(center=(0.0, 0.0, 24.0), color_index=0):
    x, y, z = center
    radius = P["can_diameter_nominal"] / 2
    body_len = P["can_length"] - 7.0
    body = cylinder_x(radius, body_len, (x - 1.5, y, z), sections=64)
    lid = cylinder_x(radius + 0.6, 7.0, (x + body_len / 2 - 1.5 + 3.5, y, z), sections=64)
    bottom_ring = cylinder_x(radius + 0.25, 1.4, (x - body_len / 2 - 1.5, y, z), sections=64)
    return {"body": body, "lid": lid, "bottom_ring": bottom_ring, "sku": color_index}


def screw_m3_x(length, center):
    shaft = cylinder_x(1.5, length, center, sections=24)
    head_center = (center[0] - length / 2 - 1.25, center[1], center[2])
    head = cylinder_x(3.0, 2.5, head_center, sections=6)
    return fuse([shaft, head])


def flat_head_screw_z(diameter, length, head_diameter, head_height, center):
    """Assembly reference for an ISO 10642 screw installed flush at Z=0."""
    x, y, z_top = center
    shaft = trimesh.creation.cylinder(
        radius=diameter / 2, height=length, sections=28,
        transform=trimesh.transformations.translation_matrix(
            [x, y, z_top - length / 2]))
    head = trimesh.creation.cone(radius=head_diameter / 2,
                                  height=head_height, sections=40,
                                  radius_top=diameter / 2)
    head = rotate(head, 180, [1, 0, 0])
    head.apply_translation([x, y, z_top])
    return fuse([shaft, head])


def tray_fastener_reference_parts():
    """All ten road-side screws for one lane, with heads below Z=0."""
    m4 = [flat_head_screw_z(4.0, 16.0, 8.9, 2.45, (x, y, -0.10))
          for x in (-27.0, 27.0) for y in (39.0, 57.0)]
    m3_frame = [flat_head_screw_z(3.0, 12.0, 6.8, 1.90, (x, y, -0.10))
                for x in (-15.0, 20.0) for y in (-40.0, 12.5)]
    m3_bridge = [flat_head_screw_z(3.0, 12.0, 6.8, 1.90, (x, -60.0, -0.10))
                 for x in (-18.0, 18.0)]
    return {
        "m4_tray_flathead_screws": fuse(m4),
        "m3_frame_flathead_screws": fuse(m3_frame),
        "m3_bridge_flathead_screws": fuse(m3_bridge),
    }


# ---------------------------------------------------------------------------
# Six-sheet cassette
# ---------------------------------------------------------------------------
def lower_z(depth_x):
    return depth_x * math.tan(math.radians(P["lower_angle_deg"]))


def upper_z(depth_x):
    return P["upper_front_height"] - depth_x * math.tan(math.radians(P["upper_angle_deg"]))


def divider_profile(clearance=None):
    if clearance is None:
        clearance = P["sheet_slot_clearance_cnc"]
    depth = P["cassette_depth"]
    height = P["cassette_height"]
    slot_w = P["sheet_thickness"] + clearance
    outline = sbox(0, 0, depth, height)

    # Upper sheet slides from the front and stops 55 mm before the rear.
    upper_line = LineString([
        (-6, upper_z(-6)),
        (P["upper_x1"], upper_z(P["upper_x1"])),
    ]).buffer(slot_w / 2, cap_style=2, join_style=2)
    upper_relief = Point(P["upper_x1"], upper_z(P["upper_x1"])).buffer(4.0, resolution=24)

    # Lower sheet slides from the rear and stops 40 mm after the front.
    lower_line = LineString([
        (P["lower_x0"], lower_z(P["lower_x0"])),
        (depth + 6, lower_z(depth + 6)),
    ]).buffer(slot_w / 2, cap_style=2, join_style=2)
    lower_relief = Point(P["lower_x0"], lower_z(P["lower_x0"])).buffer(4.0, resolution=24)

    result = outline.difference(unary_union([upper_line, upper_relief, lower_line, lower_relief]))
    if result.geom_type != "Polygon":
        # The designed profile must remain one physical sheet.
        largest = max(result.geoms, key=lambda g: g.area)
        result = largest
    return result


def divider_mesh():
    return extrude_xz(divider_profile(), P["sheet_thickness"], y0=-P["sheet_thickness"] / 2)


def sloped_sheet(x0, x1, angle_deg, z_func, width=None):
    if width is None:
        width = P["cassette_width"]
    run = x1 - x0
    length = run / math.cos(math.radians(angle_deg))
    sheet = bx((length, width, P["sheet_thickness"]), (0, 0, 0))
    # Positive global slope needs -Y rotation; negative needs +Y.
    sign_angle = -angle_deg if z_func(x1) > z_func(x0) else angle_deg
    sheet = rotate(sheet, sign_angle, [0, 1, 0])
    sheet.apply_translation([(x0 + x1) / 2, P["box_inside_width"] / 2,
                             (z_func(x0) + z_func(x1)) / 2])
    return _clean(sheet)


def cassette_parts():
    offset_y = (P["box_inside_width"] - P["cassette_width"]) / 2
    lower = sloped_sheet(P["lower_x0"], P["lower_x1"], P["lower_angle_deg"], lower_z)
    upper = sloped_sheet(P["upper_x0"], P["upper_x1"], P["upper_angle_deg"], upper_z)
    # Four positive Ø4.2 lock holes sit 8 mm in from each open sheet mouth
    # and 6 mm to one side of the divider centreline. Axes are normal to the
    # corresponding sloped sheet, exactly matching A04L/A04U.
    offset_y = (P["box_inside_width"] - P["cassette_width"]) / 2
    lock_ys = [offset_y + centre + 6.0
               for centre in P["divider_centres_from_cassette_left"]]
    lower_x = P["lower_x1"] - 8.0 * math.cos(math.radians(P["lower_angle_deg"]))
    upper_x = P["upper_x0"] + 8.0 * math.cos(math.radians(P["upper_angle_deg"]))
    lower_holes = []
    upper_holes = []
    for yy in lock_ys:
        hole = trimesh.creation.cylinder(radius=2.10, height=10.0, sections=36)
        hole = rotate(hole, -P["lower_angle_deg"], [0, 1, 0])
        hole.apply_translation([lower_x, yy, lower_z(lower_x)])
        lower_holes.append(hole)
        hole = trimesh.creation.cylinder(radius=2.10, height=10.0, sections=36)
        hole = rotate(hole, P["upper_angle_deg"], [0, 1, 0])
        hole.apply_translation([upper_x, yy, upper_z(upper_x)])
        upper_holes.append(hole)
    lower = cut(lower, lower_holes)
    upper = cut(upper, upper_holes)
    divider = divider_mesh()
    dividers = []
    for centre in P["divider_centres_from_cassette_left"]:
        d = divider.copy()
        d.apply_translation([0, offset_y + centre, 0])
        dividers.append(d)
    return {"lower_sheet": lower, "upper_sheet": upper, "dividers": dividers}


def cassette_lock_mesh(angle_deg, reverse=False):
    """Screw-clamped edge lock for one open sloped-sheet/divider joint."""
    t = P["sheet_thickness"]
    body = bx((22.0, 20.0, 14.0), (0, 0, 0))
    direction = -1.0 if not reverse else 1.0
    slope_sign = 1.0 if not reverse else -1.0
    mouth_x = direction * 11.0
    inner_x = 0.0
    centre_x = (mouth_x + inner_x) / 2
    sheet_slot = bx((11.0, 22.0, t + 0.35),
                    (centre_x, 0, -2.0))
    sheet_slot = rotate(sheet_slot, slope_sign * angle_deg, [0, 1, 0],
                        point=(mouth_x, 0, -2.0))
    divider_slot = bx((11.0, t + 0.35, 9.0),
                      (centre_x, 0, 2.5))

    # M4 locking screw: 8 mm inward from the open mouth, 6 mm beside the
    # divider, and normal to the sloped sheet. This is a real Ø4.2 through
    # hole, not a decorative mark.
    pin_x = mouth_x - direction * 8.0
    pin = trimesh.creation.cylinder(radius=2.10, height=24.0, sections=36)
    pin = rotate(pin, slope_sign * angle_deg, [0, 1, 0])
    pin.apply_translation([pin_x, 6.0, -2.0])
    return cut(body, [sheet_slot, divider_slot, pin])


# ---------------------------------------------------------------------------
# Two-gate cam mechanism
# ---------------------------------------------------------------------------
def gate_height_front(angle_deg):
    closed = P["blade_closed_z"]
    opened = P["blade_open_z"]
    start = P["cam_front_open_start_deg"]
    end = P["cam_front_open_end_deg"]
    if angle_deg <= start:
        return closed
    if angle_deg >= end:
        return opened
    q = (angle_deg - start) / (end - start)
    e = trapezoidal_ease(q)
    return closed + (opened - closed) * e


def gate_height_rear(angle_deg):
    opened = P["blade_open_z"]
    closed = P["blade_closed_z"]
    end = P["cam_rear_close_end_deg"]
    if angle_deg <= 0:
        return opened
    if angle_deg >= end:
        return closed
    q = angle_deg / end
    e = trapezoidal_ease(q)
    return opened + (closed - opened) * e


def trapezoidal_ease(q, ramp=0.11):
    """Linear travel with 11% constant-acceleration ramps at both ends.

    Peak slope is 1/(1-ramp)=1.124 times the mean, rather than 1.5 for a
    cubic smoothstep. This materially lowers peak SG90/MG90S load.
    """
    q = min(1.0, max(0.0, float(q)))
    vmax = 1.0 / (1.0 - ramp)
    if q < ramp:
        return 0.5 * vmax * q * q / ramp
    if q > 1.0 - ramp:
        tail = 1.0 - q
        return 1.0 - 0.5 * vmax * tail * tail / ramp
    return vmax * (q - ramp / 2.0)


def cam_slot_path(gate_y, height_fn):
    cy = P["servo_cam_axis_y"]
    cz = P["servo_cam_axis_z"]
    pin_drop = P["blade_pin_below_tip"]
    points = []
    # Four degrees of real groove beyond both software endpoints prevent a
    # calibration error from driving the roller into a closed slot end.
    margin = P["cam_profile_margin_deg"]
    for angle in np.linspace(-margin, P["cam_angle_deg"] + margin, 265):
        # Fixed follower location in world (Y,Z), transformed into rotating cam coordinates.
        world_y = gate_y + P["follower_y_offset"] - cy
        world_z = height_fn(float(angle)) - pin_drop - cz
        # The cam is rendered/assembled with world rotation R(-angle), hence
        # the fixed world follower must be mapped into cam coordinates by the
        # inverse transform R(+angle).
        a = math.radians(float(angle))
        local_y = math.cos(a) * world_y - math.sin(a) * world_z
        local_z = math.sin(a) * world_y + math.cos(a) * world_z
        points.append((local_y, local_z))
    return LineString(points)


def cam_profiles():
    radius = P["cam_radius"]
    slot_radius = P["cam_slot_width"] / 2
    disk = Point(0, 0).buffer(radius, resolution=128)
    # Flat top leaves 0.5 mm below the tray underside at absolute Z=-3.0.
    # With the 28 mm gate stroke this also preserves 2.05 mm above the
    # front groove: groove top 36.05, clipped disk top 38.10 mm.
    cam_top = -3.5 - P["servo_cam_axis_z"]
    disk = disk.intersection(sbox(-100.0, -100.0, 100.0, cam_top))
    front_slot = cam_slot_path(P["gate_front_y"], gate_height_front).buffer(
        slot_radius, cap_style=1, join_style=1, resolution=12
    )
    rear_slot = cam_slot_path(P["gate_rear_y"], gate_height_rear).buffer(
        slot_radius, cap_style=1, join_style=1, resolution=12
    )
    # Ø8.3 running bore accepts a separate flanged support bushing. Keeping
    # the bushing separate lets the entire cam disk print flat on the bed.
    center_hole = Point(0, 0).buffer(4.15, resolution=40)
    lock_hole = Point(-20.5, -20.5).buffer(1.7, resolution=24)
    through = disk.difference(unary_union([front_slot, rear_slot, center_hole, lock_hole]))
    horn_pocket = affinity.rotate(horn_outline(clearance=0.35), 90, origin=(0, 0))
    inner = through.difference(horn_pocket)
    return through, inner, front_slot, rear_slot


def cam_mesh():
    through, inner, _, _ = cam_profiles()
    cam_t = P["cam_thickness"]
    pocket = P["horn_pocket_depth"]
    # Shift 0.6 mm inward to leave a 1.0 mm cover-to-cam gap. The trapped
    # 0.8 mm M09 thrust flange then retains 0.2 mm axial running clearance.
    x_outer = -35.8
    outer_layer = extrude_yz(through, cam_t - pocket, x0=x_outer)
    inner_layer = extrude_yz(inner, pocket + 0.02, x0=x_outer + cam_t - pocket - 0.02)
    cy = P["servo_cam_axis_y"]
    cz = P["servo_cam_axis_z"]
    cam = fuse([translate(outer_layer, (0, cy, cz)), translate(inner_layer, (0, cy, cz))])
    return cam


def cam_support_bushing_mesh(center=(0.0, 0.0, 0.0)):
    """Internally trapped D-keyed sleeve: cover bearing + cam support."""
    x, y, z = center
    # In assembly x=-37.45: the D-body is flush with the cover outside at
    # -39.20, crosses M06, and supports 2.25 mm of the cam up to -33.55.
    # The Ø11×0.8 flange remains between cover (-36.8) and cam (-35.8), so
    # the assembled cover traps it positively without a snap bead or glue.
    tube = extrude_yz(d_bushing_profile(), 5.65, x0=x - 1.75)
    flange_profile = Point(0, 0).buffer(5.50, resolution=56).intersection(
        sbox(-10.0, -10.0, 10.0, 3.05))
    flange = extrude_yz(flange_profile, 0.80, x0=x + 0.65)
    flange.apply_translation([0, y, z])
    bore = cylinder_x(2.60, 8.0, (x + 0.5, y, z), sections=40)
    return cut(fuse([tube, flange]), [bore])


def blade_profile(rear=False):
    t = P["blade_thickness"]
    half = t / 2
    bottom = -52.0
    # Both gates use the same long symmetric 18 mm taper. At the 50.5 mm
    # pitch their 1.2 mm tips leave ~1 mm around a Ø48 can in BOTH-CLOSED,
    # rather than wedging and lifting it between two blunt 4 mm bars.
    pts = [(-half, bottom), (half, bottom), (half, -18.0),
           (0.6, 0.0), (-0.6, 0.0), (-half, -18.0)]
    return Polygon(pts)


def blade_mesh(rear=False):
    width = P["blade_working_width"]
    x0 = -30.5
    profile = blade_profile(rear=rear)
    stem = extrude_yz(profile, width, x0=x0)
    # The final 10 mm of the gate spans 72 mm, leaving 0.7 mm total inside
    # the 73.4 mm tray. The lower 66.5 mm stem stays inside the existing
    # two-sided guides and keeps the moving mass low.
    wide_profile = profile.intersection(sbox(-10.0, -10.0, 10.0, 0.0))
    wide_edge = extrude_yz(wide_profile, 72.0, x0=-36.0)
    blade = fuse([stem, wide_edge])
    # Local follower boss on the cam side.  V10 deliberately does NOT drill a
    # 70 mm tunnel through the whole blade: an M3 heat-set insert is installed
    # from the left edge and a short M3x12 low-head screw carries the roller.
    # The boss slides in its own 8.7 mm guide; the blade body is guided at the
    # opposite edge, so the boss cannot jam a 4.55 mm blade slot.
    pin_z = -P["blade_pin_below_tip"]
    boss_len = 5.5
    # Offset the circular boss to one face.  Its lowest Y is exactly the flat
    # blade face Y=-2, so the whole blade prints broad-side down without any
    # support while the insert still has a full 2 mm radial wall.
    follower_y = P["follower_y_offset"]
    boss = cylinder_x(4.1, boss_len, (x0 + boss_len / 2, follower_y, pin_z), sections=40)
    insert_hole = cylinder_x(2.10, 5.75, (x0 + 2.875, follower_y, pin_z), sections=32)
    insert_lead = cylinder_x(2.30, 0.8, (x0 + 0.4, follower_y, pin_z), sections=32)
    # Cut the insert bore AFTER fusing.  Otherwise the untouched blade body
    # would refill the middle of the bore during the boolean union.
    reinforced = fuse([blade, boss])
    return cut(reinforced, [insert_hole, insert_lead])


def tray_mesh():
    width = P["module_width"]
    y1 = P["module_y_max"]
    # The rigid tray and all moving parts remain outside the box.  A separate
    # keyed input bridge enters the 75 mm cassette lane.
    main_y0 = -65.0
    floor = bx((width, y1 - main_y0, 3.0), (0, (main_y0 + y1) / 2, -1.5))
    # 72.0 mm gate head + 0.25 mm running clearance at either side.
    slot_x0 = -36.25
    slot_wx = 72.50
    cutters = []
    for gy in (P["gate_rear_y"], P["gate_front_y"]):
        cutters.append(bx((slot_wx, P["floor_slot_width"], 6.0),
                          (slot_x0 + slot_wx / 2, gy, -1.5)))
    # Four countersunk/clearance holes attach the module to the common beam.
    for x in (-27.0, 27.0):
        for y in (39.0, 57.0):
            # 8 mm fore/aft slot lets each channel align to an already-cut
            # facade window without changing the cassette pitch.
            slot_parts = [
                trimesh.creation.cylinder(radius=2.25, height=8, sections=28,
                    transform=trimesh.transformations.translation_matrix([x, y - 4.0, -2])),
                trimesh.creation.cylinder(radius=2.25, height=8, sections=28,
                    transform=trimesh.transformations.translation_matrix([x, y + 4.0, -2])),
                bx((4.5, 8.0, 8.0), (x, y, -2.0)),
            ]
            cutters.append(fuse(slot_parts))
            # Elongated 90-degree countersink for an ISO M4 flat-head screw.
            # The Ø9.0 top opening is flush with the can path and tapers to
            # the Ø4.5 clearance slot 2.25 mm below the running surface.
            cones = []
            for yy in (y - 4.0, y + 4.0):
                cone = trimesh.creation.cone(radius=4.70, height=2.45,
                                              sections=40, radius_top=2.25)
                cone = rotate(cone, 180, [1, 0, 0])
                cone.apply_translation([x, yy, 0.02])
                cones.append(cone)
            cutters.append(trimesh.util.concatenate(cones).convex_hull)
    # Four M3 through-holes join the removable drive frame to the tray.
    for x in (-15.0, 20.0):
        for y in (-40.0, 12.5):
            cutters.append(trimesh.creation.cylinder(radius=1.7, height=8, sections=28,
                                                      transform=trimesh.transformations.translation_matrix([x, y, -2])))
            cone = trimesh.creation.cone(radius=3.50, height=1.90,
                                          sections=36, radius_top=1.70)
            cone = rotate(cone, 180, [1, 0, 0])
            cone.apply_translation([x, y, 0.02])
            cutters.append(cone)
    # Two more M3 holes clamp the removable input bridge from above.
    for x in (-18.0, 18.0):
        cutters.append(trimesh.creation.cylinder(radius=1.7, height=8, sections=28,
                                                  transform=trimesh.transformations.translation_matrix([x, -60.0, -2])))
        cone = trimesh.creation.cone(radius=3.50, height=1.90,
                                      sections=36, radius_top=1.70)
        cone = rotate(cone, 180, [1, 0, 0])
        cone.apply_translation([x, -60.0, 0.02])
        cutters.append(cone)
    floor = cut(floor, cutters)

    wall_t = 2.5
    wall_h = 29.0
    outside_length = y1 - main_y0
    walls = [
        bx((wall_t, outside_length, wall_h),
           (-width / 2 + wall_t / 2, (main_y0 + y1) / 2, wall_h / 2)),
        bx((wall_t, outside_length, wall_h),
           (width / 2 - wall_t / 2, (main_y0 + y1) / 2, wall_h / 2)),
    ]
    return fuse([floor, *walls])


def input_bridge_mesh():
    """Narrow removable bridge from the six-sheet cassette to the gate tray."""
    width = P["module_width"] - 5.5  # 72.9 mm; 0.5 mm total in the M01 throat
    y0, y1 = P["module_y_min"], -65.0
    # The exact transformed plywood edge is Y=-106.404.  Starting the printed
    # web at -106.10 leaves 0.30 mm assembly clearance while the shelf still
    # supports the plywood underside.
    sheet_edge_y = -106.10
    # Rabbet joint: the upstream shelf supports 33 mm of plywood from below at
    # Z=-4; exactly at the plywood edge a vertical web rises to a new rolling
    # surface at Z=0.  Therefore the can crosses a flush butt seam—no 4 mm
    # drop and no uphill ramp.
    shelf = sbox(y0, -7.0, sheet_edge_y, -4.0)
    deck = sbox(sheet_edge_y, -3.0, y1, 0.0)
    web = sbox(sheet_edge_y, -7.0, sheet_edge_y + 2.0, 0.0)
    profile = unary_union([shelf, deck, web])
    body = extrude_yz(profile, width, x0=-width / 2)

    # Two under-tabs overlap the tray underside without creating a bump in the
    # can path.  M3x12 screws through the tray make the bridge replaceable.
    parts = [body]
    for x in (-18.0, 18.0):
        parts.append(bx((14.0, 15.0, 3.0), (x, -62.5, -4.5)))
        parts.append(bx((14.0, 5.0, 5.0), (x, -67.5, -3.5)))
    bridge = fuse(parts)
    holes = [trimesh.creation.cylinder(
        radius=1.7, height=10, sections=28,
        transform=trimesh.transformations.translation_matrix([x, -60.0, -3.0]))
        for x in (-18.0, 18.0)]
    return cut(bridge, holes)


def guide_jaws(gate_y, x_center, gap=None):
    if gap is None:
        gap = P["blade_guide_gap"]
    jaw_t = 2.8
    jaw_x = 5.0
    # Stop at the underside of the 3 mm tray floor; the guide must touch the
    # tray, never occupy the same material volume.
    z_center = (-58.0 - 3.0) / 2
    z_height = 55.0
    return [
        bx((jaw_x, jaw_t, z_height),
           (x_center, gate_y - gap / 2 - jaw_t / 2, z_center)),
        bx((jaw_x, jaw_t, z_height),
           (x_center, gate_y + gap / 2 + jaw_t / 2, z_center)),
    ]


def drive_frame_mesh():
    parts = []
    # Top skeleton bolts under the tray and joins both two-sided guides.
    parts += [
        bx((67.0, 5.0, 4.0), (2.5, -40.0, -5.0)),
        bx((67.0, 5.0, 4.0), (2.5, 12.5, -5.0)),
        bx((5.0, 70.0, 4.0), (-28.5, -25.0, -5.0)),
        bx((5.0, 70.0, 4.0), (33.5, -25.0, -5.0)),
        # Continuous lower backbones route below the lowest blade position.
        # They keep the frame one printable part after the upper rails receive
        # their two unavoidable blade passages.
        bx((5.0, 95.0, 5.0), (-28.5, -25.0, -60.5)),
        bx((5.0, 95.0, 5.0), (33.5, -25.0, -60.5)),
    ]
    for gy in (P["gate_rear_y"], P["gate_front_y"]):
        # Cam-side guide follows the reinforced follower boss; opposite guide
        # follows the 4 mm blade body.  Both are two-sided and 62 mm apart.
        parts.extend(guide_jaws(gy + P["follower_y_offset"], -28.5,
                                gap=P["blade_boss_guide_gap"]))
        parts.extend(guide_jaws(gy, 33.5, gap=P["blade_guide_gap"]))

    # Servo mounting wall on the inner side of the cam.
    cy, cz = P["servo_cam_axis_y"], P["servo_cam_axis_z"]
    mount = bx((3.0, 19.0, 43.0), (-27.0, cy, cz + 5.4))
    shaft_clear = cylinder_x(6.2, 8.0, (-27.0, cy, cz), sections=40)
    ear_holes = []
    for zz in (cz + 5.4 - 14.3, cz + 5.4 + 14.3):
        ear_holes.append(cylinder_x(1.05, 8.0, (-27.0, cy, zz), sections=24))
    mount = cut(mount, [shaft_clear, *ear_holes])
    parts.append(mount)
    # Neck from the servo wall to the upper skeleton; it sits halfway between
    # the two blades and therefore cannot enter either blade path.
    parts.append(bx((5.0, 9.0, 23.0), (-28.5, cy, -14.5)))

    # Cover stand-offs, safely outside the rotating cam.
    cover_points = ((-43, -34), (-43, 34), (43, -34), (43, 34))
    for dy, dz in cover_points:
        boss = cylinder_x(5.2, 8.2, (-32.3, cy + dy, cz + dz), sections=32)
        hole = cylinder_x(1.65, 10.0, (-32.3, cy + dy, cz + dz), sections=24)
        parts.append(cut(boss, [hole]))
    # Two slim outboard columns tie the enlarged Ø97 cam-cover posts back to
    # the lower skeleton while staying 1.4 mm outside the cam's axial plane.
    for dy in (-43.0, 43.0):
        parts.append(bx((3.0, 5.0, 71.0),
                        (-27.5, cy + dy, cz)))

    frame = fuse(parts)
    # The upper longitudinal rails cross both gate planes.  Open a precise
    # 4.5 mm passage at each plane so the 4.0 mm blade never hits a rail.
    rail_slots = [bx((72.5, 4.5, 5.0), (0.0, gy, -5.0))
                  for gy in (P["gate_rear_y"], P["gate_front_y"])]
    frame = cut(frame, rail_slots)
    # Matching tray/frame M3 holes in the two upper crossbars.
    joint_holes = []
    for xx in (-15.0, 20.0):
        for yy in (-40.0, 12.5):
            joint_holes.append(trimesh.creation.cylinder(
                radius=1.7, height=10, sections=28,
                transform=trimesh.transformations.translation_matrix([xx, yy, -5])))
    return cut(frame, joint_holes)


def cam_cover_mesh():
    cy, cz = P["servo_cam_axis_y"], P["servo_cam_axis_z"]
    base = Point(0, 0).buffer(P["cam_radius"] + 4.2, resolution=96)
    tabs = []
    cover_points = ((-43, -34), (-43, 34), (43, -34), (43, 34))
    for dy, dz in cover_points:
        tabs.append(Point(dy, dz).buffer(6.2, resolution=20))
        tabs.append(LineString([(0, 0), (dy, dz)]).buffer(4.0, cap_style=2))
    profile = unary_union([base, *tabs])
    # The circular cover used to enter the left tray wall by 1.2 mm.  Keep the
    # upper edge 0.5 mm below the tray underside instead.
    cover_top = -3.5 - cz
    profile = profile.intersection(sbox(-100.0, -100.0, 100.0, cover_top))
    # D-keyed support bushing, four M3 screws and a HOME lock pin.
    holes = [d_bushing_profile(clearance=0.15),
             Point(-20.5, -20.5).buffer(1.7, resolution=24)]
    holes += [Point(dy, dz).buffer(1.7, resolution=24)
              for dy, dz in cover_points]
    # Narrow vertical service slots follow the two roller screws.  Their heads
    # may project through the cover instead of colliding with it; the 7.2 mm
    # width remains too small for a finger to reach the rotating cam.
    follower_travel = P["blade_closed_z"] - P["blade_open_z"]
    service_h = follower_travel + 7.0
    service_z = ((P["blade_closed_z"] + P["blade_open_z"]) / 2
                 - P["blade_pin_below_tip"] - cz)
    holes += [affinity.translate(rounded_rectangle(7.2, service_h, 3.2),
                                 P["gate_front_y"] + P["follower_y_offset"] - cy, service_z),
              affinity.translate(rounded_rectangle(7.2, service_h, 3.2),
                                 P["gate_rear_y"] + P["follower_y_offset"] - cy, service_z)]
    profile = profile.difference(unary_union(holes))
    cover = extrude_yz(profile, 2.4, x0=-39.2)
    cover.apply_translation([0, cy, cz])
    return cover


def follower_roller_mesh():
    roller = cylinder_x(P["cam_follower_diameter"] / 2, 5.0, (0, 0, 0), sections=40)
    hole = cylinder_x(1.65, 7.0, (0, 0, 0), sections=24)
    return cut(roller, [hole])


def follower_screw_mesh(center=(0.0, 0.0, 0.0)):
    """M3x12 low-head follower screw, assembly reference only."""
    x, y, z = center
    shaft = cylinder_x(1.5, 11.5, (x, y, z), sections=28)
    # Head sits on the cover side and travels inside the cover service slot.
    head = cylinder_x(3.0, 2.2, (x - 6.85, y, z), sections=40)
    socket = cylinder_x(1.35, 1.2, (x - 7.4, y, z), sections=6)
    return cut(fuse([shaft, head]), [socket])


def guide_fit_coupon_mesh():
    """Three blade-guide gaps: 4.35 / 4.55 / 4.75 mm on one plate."""
    base = bx((86.0, 34.0, 3.0), (0, 0, 1.5))
    parts = [base]
    gaps = (4.35, 4.55, 4.75)
    for idx, (cx, gap) in enumerate(zip((-28.0, 0.0, 28.0), gaps), start=1):
        for sign in (-1, 1):
            parts.append(bx((3.0, 25.0, 10.0),
                            (cx + sign * (gap / 2 + 1.5), 0, 8.0)))
        # One/two/three edge nicks identify the increasing-clearance samples.
        for mark in range(idx):
            parts.append(bx((1.2, 3.0, 2.0),
                            (cx - 2.0 + mark * 2.0, -15.5, 4.0)))
    return fuse(parts)


def horn_fit_coupon_mesh():
    """Recesses with +0.20 / +0.35 / +0.50 mm stock-horn clearance."""
    base = bx((118.0, 30.0, 3.2), (0, 0, 1.6))
    cutters = []
    for idx, (cx, clearance) in enumerate(zip((-39.0, 0.0, 39.0),
                                               (0.20, 0.35, 0.50)), start=1):
        pocket = affinity.translate(horn_outline(clearance), xoff=cx)
        pocket_mesh = trimesh.creation.extrude_polygon(pocket, height=2.6)
        pocket_mesh.apply_translation([0, 0, 0.8])
        cutters.append(pocket_mesh)
        for mark in range(idx):
            cutters.append(bx((1.3, 4.0, 4.0),
                               (cx - 2.2 + mark * 2.2, -14.5, 2.0)))
    return cut(base, cutters)


def bushing_cam_fit_coupon_mesh():
    """M09 key/cam bore and M07 groove clearances on one flat coupon."""
    base = bx((132.0, 50.0, 4.0), (0, 0, 2.0))
    cutters = []
    clearances = (0.10, 0.15, 0.25)
    groove_widths = (6.60, 6.80, 7.00)
    for idx, (xx, clearance, groove_w) in enumerate(
            zip((-44.0, 0.0, 44.0), clearances, groove_widths), start=1):
        keyed = affinity.translate(d_bushing_profile(clearance),
                                   xoff=xx, yoff=12.0)
        keyed_cut = trimesh.creation.extrude_polygon(keyed, height=8.0)
        keyed_cut.apply_translation([0, 0, -2.0])
        cutters.append(keyed_cut)

        round_cut = trimesh.creation.cylinder(
            radius=3.90 + clearance, height=8.0, sections=40,
            transform=trimesh.transformations.translation_matrix([xx, 0, 2.0]))
        cutters.append(round_cut)

        slot = affinity.translate(
            rounded_rectangle(groove_w, 14.0, groove_w / 2),
            xoff=xx, yoff=-16.0)
        slot_cut = trimesh.creation.extrude_polygon(slot, height=8.0)
        slot_cut.apply_translation([0, 0, -2.0])
        cutters.append(slot_cut)

        # One/two/three edge nicks identify the increasing-clearance station.
        for mark in range(idx):
            cutters.append(bx((1.2, 4.0, 8.0),
                               (xx - 2.0 + mark * 2.0, 24.0, 2.0)))
    return cut(base, cutters)


def gate_parts_reference(angle_deg=0.0):
    front_h = gate_height_front(angle_deg)
    rear_h = gate_height_rear(angle_deg)
    front = translate(blade_mesh(False), (0, P["gate_front_y"], front_h))
    rear = translate(blade_mesh(True), (0, P["gate_rear_y"], rear_h))
    cam = cam_mesh()
    cam = rotate(cam, -angle_deg, [1, 0, 0],
                 point=(0, P["servo_cam_axis_y"], P["servo_cam_axis_z"]))
    front_pin_z = front_h - P["blade_pin_below_tip"]
    rear_pin_z = rear_h - P["blade_pin_below_tip"]
    front_pin_y = P["gate_front_y"] + P["follower_y_offset"]
    rear_pin_y = P["gate_rear_y"] + P["follower_y_offset"]
    roller_x = -33.7
    screw_x = -30.75
    servo = servo_reference_parts()
    servo["servo_horn"] = rotate(servo["servo_horn"], -angle_deg, [1, 0, 0],
                                  point=(0, P["servo_cam_axis_y"], P["servo_cam_axis_z"]))
    servo["servo_screw"] = rotate(servo["servo_screw"], -angle_deg, [1, 0, 0],
                                   point=(0, P["servo_cam_axis_y"], P["servo_cam_axis_z"]))
    parts = {
        "tray": tray_mesh(),
        "input_bridge": input_bridge_mesh(),
        "drive_frame": drive_frame_mesh(),
        "cam_cover": cam_cover_mesh(),
        "cam": cam,
        "cam_support_bushing": cam_support_bushing_mesh(
            (-37.45, P["servo_cam_axis_y"], P["servo_cam_axis_z"])),
        "front_blade": front,
        "rear_blade": rear,
        "front_roller": translate(follower_roller_mesh(),
                                  (roller_x, front_pin_y, front_pin_z)),
        "rear_roller": translate(follower_roller_mesh(),
                                 (roller_x, rear_pin_y, rear_pin_z)),
        "front_follower_screw": follower_screw_mesh(
            (screw_x, front_pin_y, front_pin_z)),
        "rear_follower_screw": follower_screw_mesh(
            (screw_x, rear_pin_y, rear_pin_z)),
    }
    parts.update(servo)
    parts.update(tray_fastener_reference_parts())
    return parts


# ---------------------------------------------------------------------------
# Transforms, printable orientations and reports
# ---------------------------------------------------------------------------
def orient_blade_for_print(mesh):
    # Blade's thin Y dimension becomes print Z; broad X-Z face lies on bed.
    return on_bed(rotate(mesh, 90, [1, 0, 0]))


def orient_yz_part_for_print(mesh):
    # Y-Z plane becomes the bed plane; X thickness becomes print Z.
    return on_bed(rotate(mesh, -90, [0, 1, 0]))


def module_to_box_transform(lane_center):
    """Map local gate coordinates to the front of the 15-degree lower deck."""
    theta = math.radians(P["lower_angle_deg"])
    # Put the rear separator 15 mm outside the facade.  The 210 mm one-piece
    # tray can then reach 33 mm under the lower sheet while every moving drive
    # component remains outside the four cassette dividers.
    tx = P["gate_rear_box_x"] + math.cos(theta) * P["gate_rear_y"]
    # Local Z=0 is the TOP of the 4 mm lower sheet, not its mid-plane.
    tz = math.tan(theta) * tx + (P["sheet_thickness"] / 2) / math.cos(theta)
    # Basis columns: local X, local Y, local Z in box coordinates.
    matrix = np.array([
        [0, -math.cos(theta), -math.sin(theta), tx],
        [1, 0, 0, lane_center],
        [0, -math.sin(theta), math.cos(theta), tz],
        [0, 0, 0, 1],
    ], dtype=float)
    return matrix


def support_beam_mesh():
    """Nominal 2040 aluminium extrusion shared by all five gate modules."""
    # The rail deliberately overhangs the 395 mm box by 30 mm at each side;
    # its two support brackets therefore live completely outside all lanes.
    outer = bx((455.0, 40.0, 20.0), (0, 50.0, -13.0))
    # Visual hollow section; production may use any standard 2040 T-slot rail.
    inner = bx((459.0, 34.0, 14.0), (0, 50.0, -13.0))
    beam = cut(outer, [inner])
    beam.apply_transform(module_to_box_transform(P["box_inside_width"] / 2))
    return beam


def support_wedge_mesh():
    """15° saddle between a horizontal steel bracket and the sloped 2040 rail."""
    # Physical X direction, bottom at Z=0.  The top follows the underside of
    # the 15° rail with 0.2 mm clearance for a rubber/cork pad.
    profile = Polygon([(0.0, 0.0), (46.0, 0.0),
                       (46.0, 16.55), (0.0, 4.23)])
    wedge = extrude_xz(profile, 25.0, y0=-12.5)

    slope = math.tan(math.radians(15.0))
    top_z = lambda xx: 4.23 + slope * xx
    holes = []

    # Two vertical M5 clearances clamp the wedge to the horizontal arm of
    # the steel shelf bracket. Recessed Ø10.5 pockets accept washers/nuts
    # below the sloped rail seat, so nothing can foul the aluminium profile.
    for yy in (-7.0, 7.0):
        xx = 24.0
        zz = top_z(xx)
        holes.append(trimesh.creation.cylinder(
            radius=2.75, height=zz + 4.0, sections=36,
            transform=trimesh.transformations.translation_matrix(
                [xx, yy, zz / 2])))
        holes.append(trimesh.creation.cylinder(
            radius=5.25, height=7.5, sections=40,
            transform=trimesh.transformations.translation_matrix(
                [xx, yy, zz - 1.75])))

    # Two further Ø5.5 bores are normal to the 15-degree upper face and line
    # up with the two underside T-slots of a standard 2040 extrusion. A
    # Ø10×4.2 counterbore from below seats low-head M5 screws inside the wedge.
    normal = np.array([-math.sin(math.radians(15.0)), 0.0,
                       math.cos(math.radians(15.0))])
    for xx in (15.0, 34.0):
        zz = top_z(xx)
        top_point = np.array([xx, 0.0, zz])
        travel = zz / normal[2]
        bottom_point = top_point - normal * travel
        midpoint = (top_point + bottom_point) / 2
        bore = trimesh.creation.cylinder(radius=2.75, height=travel + 6.0,
                                          sections=36)
        bore = rotate(bore, -15.0, [0, 1, 0])
        bore.apply_translation(midpoint)
        holes.append(bore)

        pocket = trimesh.creation.cylinder(radius=5.0, height=4.6, sections=40)
        pocket = rotate(pocket, -15.0, [0, 1, 0])
        pocket.apply_translation(bottom_point + normal * 2.1)
        holes.append(pocket)

    return cut(wedge, holes)


def support_reference_parts():
    beam = support_beam_mesh()
    plinth = bx((600.0, 470.0, 100.0), (300.0, 200.0, -50.0))
    # The five mechanisms hang a maximum of 15.3 mm behind the facade plane.
    # A continuous 20 mm-deep front service rebate clears them while leaving
    # 35/40 mm solid side cheeks for the two outboard steel brackets.
    service_rebate = bx((20.0, 397.0, 95.0), (10.0, 197.5, -42.5))
    plinth = cut(plinth, [service_rebate])

    wedge_base = support_wedge_mesh()
    wedges = []
    brackets = []
    for yy in (-17.5, 412.5):
        w = translate(wedge_base, (-130.0, yy, -61.0))
        wedges.append(w)

        vertical = bx((8.0, 20.0, 80.0), (-4.0, yy, -55.0))
        arm = bx((135.0, 20.0, 8.0), (-67.5, yy, -65.0))
        # Diagonal steel brace, rectangular 10 mm section in X-Z.
        p1 = np.array([-5.0, -91.0])
        p2 = np.array([-123.0, -69.0])
        vec = p2 - p1
        length = float(np.linalg.norm(vec))
        diagonal = bx((length, 20.0, 10.0),
                      ((p1[0] + p2[0]) / 2, yy, (p1[1] + p2[1]) / 2))
        diagonal = rotate(diagonal, -math.degrees(math.atan2(vec[1], vec[0])),
                          [0, 1, 0],
                          point=((p1[0] + p2[0]) / 2, yy,
                                 (p1[1] + p2[1]) / 2))
        bracket = fuse([vertical, arm, diagonal])
        bracket_holes = [
            cylinder_x(2.75, 12.0, (-4.0, yy + dy, zz), sections=32)
            for dy in (-5.0, 5.0) for zz in (-35.0, -75.0)
        ]
        bracket_holes += [trimesh.creation.cylinder(
            radius=2.75, height=12.0, sections=32,
            transform=trimesh.transformations.translation_matrix(
                [-106.0, yy + dy, -65.0])) for dy in (-7.0, 7.0)]
        brackets.append(cut(bracket, bracket_holes))

    return {
        "support_beam_2040": beam,
        "support_wedges_pair": fuse(wedges),
        "support_brackets_pair": fuse(brackets),
        "plinth_reference": plinth,
    }


def write_cut_svg():
    # True 1:1 millimetre artwork.  The viewBox also uses millimetres, so the
    # file can be sent to a CNC/laser application without screen-scale maths.
    margin = 20.0
    width = P["cassette_depth"] + 2 * margin
    height = P["cassette_height"] + 2 * margin
    prof = divider_profile(P["sheet_slot_clearance_cnc"])
    exterior = "M " + " L ".join(
        f"{margin + x:.3f} {margin + (P['cassette_height']-z):.3f}"
        for x, z in prof.exterior.coords) + " Z"
    holes = []
    for ring in prof.interiors:
        holes.append("M " + " L ".join(
            f"{margin + x:.3f} {margin + (P['cassette_height']-z):.3f}"
            for x, z in ring.coords) + " Z")
    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width:.1f}mm" height="{height:.1f}mm" viewBox="0 0 {width:.1f} {height:.1f}">
<rect width="100%" height="100%" fill="white"/>
<style>.cut{{fill:none;stroke:#d11;stroke-width:.35}} .dim{{font:7px Segoe UI,Arial;fill:#123}} .note{{font:6px Segoe UI,Arial;fill:#456}}</style>
<path class="cut" fill-rule="evenodd" d="{exterior} {' '.join(holes)}"/>
<text x="{margin}" y="9" class="dim">V10 · РАЗДЕЛИТЕЛЬ 590×390×4 — 4 шт. Чертёж 1:1, единицы мм.</text>
<text x="{margin}" y="{height-5}" class="note">Верхний паз открыт спереди; нижний — сзади. Закрытые концы Ø8.</text>
</svg>'''
    (CUT_DIR / "01_РАЗДЕЛИТЕЛЬ_590x390_4шт.svg").write_text(svg, encoding="utf-8")

    # DXF carries the same exact polygon (including both long slots) for CNC.
    for suffix, clearance in (("CNC_зазор_0.25", P["sheet_slot_clearance_cnc"]),
                              ("РУЧНОЙ_зазор_0.45", P["sheet_slot_clearance_manual"])):
        path2d = trimesh.load_path(divider_profile(clearance))
        (CUT_DIR / f"01_РАЗДЕЛИТЕЛЬ_{suffix}_4шт.dxf").write_text(
            path2d.export(file_type="dxf"), encoding="utf-8")

    def rect_svg(name, length, sheet_width, angle, direction, hole_u):
        margin2 = 20.0
        w = length + 2 * margin2
        h = sheet_width + 2 * margin2
        hole_vs = [centre + 6.0 for centre in P["divider_centres_from_cassette_left"]]
        circles = "".join(
            f'<circle class="hole" cx="{margin2 + hole_u:.3f}" cy="{margin2 + vv:.3f}" r="2.100"/>'
            for vv in hole_vs)
        return f'''<svg xmlns="http://www.w3.org/2000/svg" width="{w:.2f}mm" height="{h:.2f}mm" viewBox="0 0 {w:.2f} {h:.2f}">
<rect width="100%" height="100%" fill="white"/><style>text{{font-family:Segoe UI,Arial;fill:#123}} .p{{fill:#f4dfbd;stroke:#d11;stroke-width:.35}} .hole{{fill:white;stroke:#d11;stroke-width:.35}}</style>
<rect class="p" x="{margin2}" y="{margin2}" width="{length:.3f}" height="{sheet_width:.3f}"/>
{circles}
<text x="{margin2}" y="9" font-size="7" font-weight="700">{name}: {length:.2f}×{sheet_width:.2f}×{P['sheet_thickness']:.1f} мм · 1:1</text>
<text x="{margin2}" y="{h-5:.2f}" font-size="6">Угол {angle}°; вставлять {direction}. 4 отверстия Ø4.2 под A04, центры отмечены.</text></svg>'''

    lower_len = (P["lower_x1"] - P["lower_x0"]) / math.cos(math.radians(P["lower_angle_deg"]))
    upper_len = (P["upper_x1"] - P["upper_x0"]) / math.cos(math.radians(P["upper_angle_deg"]))
    lower_hole_u = lower_len - 8.0
    upper_hole_u = 8.0
    (CUT_DIR / "02_НИЖНИЙ_ЦЕЛЬНЫЙ_ЛИСТ_1шт.svg").write_text(
        rect_svg("Нижний скат", lower_len, P["cassette_width"], P["lower_angle_deg"], "сзади вперёд", lower_hole_u), encoding="utf-8")
    (CUT_DIR / "03_ВЕРХНИЙ_ЦЕЛЬНЫЙ_ЛИСТ_1шт.svg").write_text(
        rect_svg("Верхний скат", upper_len, P["cassette_width"], P["upper_angle_deg"], "спереди назад", upper_hole_u), encoding="utf-8")
    for code, length, hole_u in (("02_НИЖНИЙ_ЦЕЛЬНЫЙ_ЛИСТ_1шт", lower_len, lower_hole_u),
                                 ("03_ВЕРХНИЙ_ЦЕЛЬНЫЙ_ЛИСТ_1шт", upper_len, upper_hole_u)):
        poly = Polygon([(0, 0), (length, 0), (length, P["cassette_width"]),
                        (0, P["cassette_width"])])
        hole_vs = [centre + 6.0 for centre in P["divider_centres_from_cassette_left"]]
        poly = poly.difference(unary_union(
            [Point(hole_u, vv).buffer(2.10, resolution=40) for vv in hole_vs]))
        (CUT_DIR / f"{code}.dxf").write_text(
            trimesh.load_path(poly).export(file_type="dxf"), encoding="utf-8")


def write_csvs(print_reports):
    bom = [
        ("A01", "Разделитель кассеты", 4, "фанера 4 мм", "590×390; два встречных длинных паза"),
        ("A02", "Нижний цельный скат", 1, "фанера 4 мм", "569.40×391"),
        ("A03", "Верхний цельный скат", 1, "фанера 4 мм", "551.38×391"),
        ("A04U", "Замок верхнего ската 14°", 4, "PETG", "Ø4.2; 8 мм от переднего края"),
        ("A04L", "Замок нижнего ската 15°", 4, "PETG", "Ø4.2; 8 мм от заднего края"),
        ("M01", "Лоток-корпус", 5, "PETG", "по одному на канал"),
        ("M02", "Нижняя рама привода", 5, "PETG", "двусторонние направляющие"),
        ("M03", "Передняя заслонка", 5, "PETG", "верх 72; нижний стержень 66.5"),
        ("M04", "Задний сепаратор-клин", 5, "PETG", "верх 72; нижний стержень 66.5"),
        ("M05", "Двухпрофильный кулачок", 5, "PETG", "карман штатной качалки"),
        ("M06", "Крышка кулачка", 5, "PETG", "вторая опора + HOME-штифт"),
        ("M07", "Ролик толкателя Ø6/М3", 10, "PETG", "2 на канал"),
        ("M08", "Входной мост с нижней полкой", 5, "PETG", "33 мм под фанеру; съёмный"),
        ("M09", "D-образная опорная втулка кулачка", 5, "PETG/PA", "ставится изнутри M06; фланец заперт между M06/M05; доступ Ø5.2"),
        ("T01", "Купон зазора направляющей", 1, "PETG", "4.35/4.55/4.75 мм; печатать первым"),
        ("T02", "Купон кармана штатной качалки", 1, "PETG", "+0.20/+0.35/+0.50 мм; печатать первым"),
        ("T03", "Купон M09/кулачок/ролик", 1, "PETG", "D/круг +0.10/+0.15/+0.25; пазы 6.6/6.8/7.0"),
        ("B01", "Клин-седло балки 15°", 2, "PETG", "4 отверстия Ø5.5; при ином рисунке покупной консоли сверлить по месту"),
        ("H01", "SG90 или MG90S", 5, "готовая", "используются уже купленные"),
        ("H02", "M3×12 с низкой головкой", 10, "металл", "оси роликов; головка Ø6 максимум"),
        ("H03", "Резьбовая термовставка M3 OD4.6×5", 10, "латунь", "в бобышки заслонок"),
        ("H04", "M3×12 ISO 10642 + гайки", 30, "металл", "рама–лоток и M08; потай Ø7.0×1.9 заподлицо"),
        ("H04B", "M3×16 + гайки", 20, "металл", "крышка кулачка; длина учитывает стойку 8.2 мм"),
        ("H05", "M4×16 ISO 10642", 20, "металл", "крепление к 2040; продольный потай Ø9.4×2.45 заподлицо"),
        ("H06", "Штифт HOME M3×20", 5, "металл", "физическая блокировка при сборке"),
        ("H07", "Алюминиевый профиль 2040", 1, "алюминий", "длина 455 мм; по 30 мм за краями коробки"),
        ("H08", "Стальная полочная консоль 150×100", 2, "сталь", "к подиуму; паспортная нагрузка от 15 кг"),
        ("H09", "T-гайка M4 для профиля 2040", 20, "металл", "по 4 на канал"),
        ("H10", "M2×10/12 для ушей сервы", 10, "металл", "2 на SG90/MG90S; отверстия рамы Ø2.1"),
        ("H11", "M4×16 + самоконтрящаяся гайка", 8, "металл", "винты замков A04U/A04L; отверстия Ø4.2"),
        ("H12", "M5×25 + шайба + гайка", 4, "металл", "по 2 вертикально: клин B01 к полочной консоли"),
        ("H13", "M5×12 низкая головка + T-гайка", 2, "металл", "низкая сторона B01 к пазу 2040"),
        ("H14", "M5×16 низкая головка + T-гайка", 2, "металл", "высокая сторона B01 к пазу 2040"),
        ("H15", "Конструкционный шуруп 5×40/50 + шайба", 8, "металл", "по 4 на вертикальную лапу консоли к подиуму"),
        ("S01", "Подиум / тумба", 1, "фанера/мебельный щит", "600×470×100 мм; спереди центральный сервисный паз 20×397×95"),
        ("S02", "Лист берёзовой фанеры 4 мм 1525×1525", 1, "фанера", "шесть деталей помещаются 2×3; толщину измерить до реза пазов"),
        ("L01", "Самоклеящаяся UHMW/PET лента", 1, "опция", "на нижние дорожки против зависания последней банки"),
    ]
    with (HERE / "BOM_МАТЕРИАЛЫ_И_КРЕПЕЖ.csv").open("w", newline="", encoding="utf-8-sig") as fh:
        wr = csv.writer(fh, delimiter=";")
        wr.writerow(("Код", "Деталь", "Кол-во", "Материал", "Примечание"))
        wr.writerows(bom)

    with (HERE / "КАРТА_ПЕЧАТИ.csv").open("w", newline="", encoding="utf-8-sig") as fh:
        wr = csv.writer(fh, delimiter=";")
        wr.writerow(("Файл", "Габарит X×Y×Z мм", "Кол-во на 5 каналов", "Ориентация", "Поддержки"))
        orientations = {
            "M01_лоток_корпус.stl": (5, "дном на стол", "нет"),
            "M02_рама_привода.stl": (5, "верхней рамой вниз", "локально под 4 стойки крышки"),
            "M03_передняя_заслонка.stl": (5, "широкой гранью", "нет"),
            "M04_задний_сепаратор.stl": (5, "широкой гранью", "нет"),
            "M05_кулачок_SG90_MG90S.stl": (5, "плоско, карманом вверх", "нет"),
            "M06_крышка_кулачка.stl": (5, "наружной плоскостью", "нет"),
            "M07_ролик_толкателя.stl": (10, "торцом", "нет"),
            "M08_входной_мост.stl": (5, "боковой плоскостью, brim 5 мм", "нет"),
            "M09_опорная_втулка_кулачка.stl": (5, "D-стержнем вниз, brim 4 мм", "только под фланец"),
            "A04U_замок_верхнего_ската_14град.stl": (4, "боковой гранью", "нет"),
            "A04L_замок_нижнего_ската_15град.stl": (4, "боковой гранью", "нет"),
            "T01_КУПОН_зазоры_направляющей.stl": (1, "основанием вниз", "нет"),
            "T02_КУПОН_карман_качалки.stl": (1, "основанием вниз", "нет"),
            "T03_КУПОН_втулка_кулачок_ролик.stl": (1, "основанием вниз", "нет"),
            "B01_клин_балки_15град.stl": (2, "плоским основанием", "нет"),
        }
        for rep in print_reports:
            qty, orientation, supports = orientations.get(rep["file"], (1, "см. README", "нет"))
            wr.writerow((rep["file"], "×".join(f"{v:.2f}" for v in rep["extents_mm"]), qty, orientation, supports))


def _write_3mf_plate(filename, items):
    """Write named, already print-oriented meshes at explicit XY positions."""
    scene = trimesh.Scene()
    for name, mesh, x, y in items:
        placed = mesh.copy()
        placed.apply_translation([
            x - placed.bounds[0, 0],
            y - placed.bounds[0, 1],
            -placed.bounds[0, 2],
        ])
        scene.add_geometry(placed, node_name=name, geom_name=name)
    payload = scene.export(file_type="3mf")
    (HERE / filename).write_bytes(payload)


def write_3mf_packages():
    """Two Ender-3-sized plates for one-channel prototype + fit coupons."""
    p = {
        "M01": on_bed(tray_mesh()),
        "M02": on_bed(rotate(drive_frame_mesh(), 180, [1, 0, 0])),
        "M03": orient_blade_for_print(blade_mesh(False)),
        "M04": orient_blade_for_print(blade_mesh(True)),
        "M05": orient_yz_part_for_print(cam_mesh()),
        "M06": orient_yz_part_for_print(cam_cover_mesh()),
        "M07": on_bed(rotate(follower_roller_mesh(), 90, [0, 1, 0])),
        "M08": orient_yz_part_for_print(input_bridge_mesh()),
        "M09": on_bed(rotate(cam_support_bushing_mesh(), -90, [0, 1, 0])),
        "A04U": on_bed(rotate(cassette_lock_mesh(P["upper_angle_deg"], False), 90, [1, 0, 0])),
        "A04L": on_bed(rotate(cassette_lock_mesh(P["lower_angle_deg"], True), 90, [1, 0, 0])),
        "T01": on_bed(guide_fit_coupon_mesh()),
        "T02": on_bed(horn_fit_coupon_mesh()),
        "T03": on_bed(bushing_cam_fit_coupon_mesh()),
        "B01": on_bed(support_wedge_mesh()),
    }
    _write_3mf_plate("3MF_01_ОДИН_КАНАЛ_СТОЛ_A.3mf", [
        ("M01_tray", p["M01"], 5, 5),
        ("M02_frame", p["M02"], 90, 5),
        ("M08_bridge", p["M08"], 170, 5),
        ("M03_front_blade", p["M03"], 5, 155),
        ("M05_cam", p["M05"], 90, 105),
    ])
    _write_3mf_plate("3MF_02_ОДИН_КАНАЛ_СТОЛ_B_И_КУПОНЫ.3mf", [
        ("M04_rear_blade", p["M04"], 5, 5),
        ("M06_cover", p["M06"], 85, 5),
        ("T02_horn_coupon", p["T02"], 5, 95),
        ("T01_guide_coupon", p["T01"], 128, 95),
        ("B01_beam_wedge", p["B01"], 5, 140),
        ("A04U_lock", p["A04U"], 58, 140),
        ("A04L_lock", p["A04L"], 85, 140),
        ("M07_roller_1", p["M07"], 112, 140),
        ("M07_roller_2", p["M07"], 123, 140),
        ("M09_bushing", p["M09"], 136, 140),
        ("T03_bushing_cam_coupon", p["T03"], 5, 170),
    ])


def main():
    print_reports = []
    assembly_reports = []

    tray = tray_mesh()
    frame = drive_frame_mesh()
    blade_front_ref = blade_mesh(False)
    blade_rear_ref = blade_mesh(True)
    cam = cam_mesh()
    cover = cam_cover_mesh()
    roller = follower_roller_mesh()
    bridge = input_bridge_mesh()
    cam_bushing = cam_support_bushing_mesh()
    lock_upper = cassette_lock_mesh(P["upper_angle_deg"], False)
    lock_lower = cassette_lock_mesh(P["lower_angle_deg"], True)
    guide_coupon = guide_fit_coupon_mesh()
    horn_coupon = horn_fit_coupon_mesh()
    bushing_coupon = bushing_cam_fit_coupon_mesh()
    support_wedge = support_wedge_mesh()

    # Print-oriented unique parts.
    print_reports.append(export_mesh(on_bed(tray), PRINT_DIR / "M01_лоток_корпус.stl"))
    print_reports.append(export_mesh(on_bed(rotate(frame, 180, [1, 0, 0])), PRINT_DIR / "M02_рама_привода.stl"))
    print_reports.append(export_mesh(orient_blade_for_print(blade_front_ref), PRINT_DIR / "M03_передняя_заслонка.stl"))
    print_reports.append(export_mesh(orient_blade_for_print(blade_rear_ref), PRINT_DIR / "M04_задний_сепаратор.stl"))
    print_reports.append(export_mesh(orient_yz_part_for_print(cam), PRINT_DIR / "M05_кулачок_SG90_MG90S.stl"))
    print_reports.append(export_mesh(orient_yz_part_for_print(cover), PRINT_DIR / "M06_крышка_кулачка.stl"))
    print_reports.append(export_mesh(on_bed(rotate(roller, 90, [0, 1, 0])), PRINT_DIR / "M07_ролик_толкателя.stl"))
    print_reports.append(export_mesh(orient_yz_part_for_print(bridge), PRINT_DIR / "M08_входной_мост.stl"))
    print_reports.append(export_mesh(on_bed(rotate(cam_bushing, -90, [0, 1, 0])),
                                     PRINT_DIR / "M09_опорная_втулка_кулачка.stl"))
    print_reports.append(export_mesh(on_bed(rotate(lock_upper, 90, [1, 0, 0])),
                                     PRINT_DIR / "A04U_замок_верхнего_ската_14град.stl"))
    print_reports.append(export_mesh(on_bed(rotate(lock_lower, 90, [1, 0, 0])),
                                     PRINT_DIR / "A04L_замок_нижнего_ската_15град.stl"))
    print_reports.append(export_mesh(on_bed(guide_coupon), PRINT_DIR / "T01_КУПОН_зазоры_направляющей.stl"))
    print_reports.append(export_mesh(on_bed(horn_coupon), PRINT_DIR / "T02_КУПОН_карман_качалки.stl"))
    print_reports.append(export_mesh(on_bed(bushing_coupon), PRINT_DIR / "T03_КУПОН_втулка_кулачок_ролик.stl"))
    print_reports.append(export_mesh(on_bed(support_wedge), PRINT_DIR / "B01_клин_балки_15град.stl"))

    # Assembly-coordinate parts loaded directly by the viewer.
    assembly_sources = {
        "tray.stl": tray,
        "drive_frame.stl": frame,
        "front_blade_ref.stl": blade_front_ref,
        "rear_blade_ref.stl": blade_rear_ref,
        "cam_home.stl": cam,
        "cam_cover.stl": cover,
        "follower_roller.stl": roller,
        "follower_screw.stl": follower_screw_mesh(),
        "input_bridge.stl": bridge,
        "cam_support_bushing.stl": cam_support_bushing_mesh(
            (-37.45, P["servo_cam_axis_y"], P["servo_cam_axis_z"])),
        "cassette_lock_upper_14deg.stl": lock_upper,
        "cassette_lock_lower_15deg.stl": lock_lower,
    }
    cassette = cassette_parts()
    assembly_sources["cassette_lower_sheet.stl"] = cassette["lower_sheet"]
    assembly_sources["cassette_upper_sheet.stl"] = cassette["upper_sheet"]
    assembly_sources["cassette_divider.stl"] = divider_mesh()
    servo = servo_reference_parts()
    assembly_sources.update({f"{key}.stl": value for key, value in servo.items()})
    assembly_sources.update({f"{key}.stl": value
                             for key, value in tray_fastener_reference_parts().items()})
    supports = support_reference_parts()
    assembly_sources.update({f"{key}.stl": value for key, value in supports.items()})

    for name, mesh in assembly_sources.items():
        assembly_reports.append(export_mesh(mesh, ASSEMBLY_DIR / name))

    # Reference assemblies for slicer/viewer sanity checks (never print as one object).
    for state_name, angle in (("HOME", 0.0), ("REAR_CLOSED", 46.0),
                              ("BOTH_CLOSED", 52.0), ("EJECT", 84.0)):
        parts = gate_parts_reference(angle)
        joined = fuse(parts.values())
        assembly_reports.append(export_mesh(joined, ASSEMBLY_DIR / f"one_lane_{state_name}.stl"))

    # One-can spacing proof in HOME and EJECT.
    home_gate = gate_parts_reference(0.0)
    cans = []
    # Tangency positions for the Ø48 design envelope against the 17 mm front
    # stop.  They are separated by 48.2 mm, so the reference cans do not
    # overlap each other or either blade in HOME.
    for idx, yy in enumerate((-26.2, -74.4, -122.6)):
        cp = can_parts((0, yy, P["can_diameter_design"] / 2), idx)
        cans.extend((cp["body"], cp["lid"], cp["bottom_ring"]))
    assembly_reports.append(export_mesh(fuse([*home_gate.values(), *cans]),
                                        ASSEMBLY_DIR / "one_lane_HOME_with_3_cans.stl"))

    write_cut_svg()
    write_csvs(print_reports)
    write_3mf_packages()

    # Analytical safety/fit checks written into the manifest.
    samples = []
    for angle in np.linspace(0, P["cam_angle_deg"], 85):
        f = gate_height_front(float(angle))
        r = gate_height_rear(float(angle))
        samples.append({"angle": round(float(angle), 2),
                        "front": round(f, 3), "rear": round(r, 3),
                        "max_gate": round(max(f, r), 3)})
    min_max_gate = min(s["max_gate"] for s in samples)
    _, _, front_slot, rear_slot = cam_profiles()
    manifest = {
        "version": "V10-production-prototype",
        "parameters_mm": P,
        "mechanism": {
            "type": "one-servo dual-gate positive grooved cam",
            "servo": "SG90 or MG90S with the stock double horn",
            "home_to_eject_deg": P["cam_angle_deg"],
            "sequence": [
                "0–46°: rear separator closes while front stop remains closed",
                "46–52°: both gates closed",
                "52–82°: front stop opens while rear stays closed",
                "82–84°: eject dwell",
            ],
            "minimum_of_higher_gate_mm": min_max_gate,
            "cam_slots_min_distance_mm": round(front_slot.distance(rear_slot), 3),
        },
        "print_parts": print_reports,
        "assembly_parts": assembly_reports,
        "warning": "Print one channel first. Production release requires a 300-cycle physical test with measured cans.",
    }
    (HERE / "V10_MANIFEST.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")

    print("V10 generated")
    print(f"Printable files: {len(print_reports)}")
    print(f"Assembly files: {len(assembly_reports)}")
    print(f"Safety check: minimum higher gate = {min_max_gate:.2f} mm")
    for report in print_reports:
        print(f"{report['file']}: {report['extents_mm']} watertight={report['watertight']} components={report['components']}")


if __name__ == "__main__":
    main()
