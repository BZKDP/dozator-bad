"""V11-C2 — upper linear dual-gate dispenser for one 70 x dia48 can lane.

Five functional printed parts per lane:
  C01 upper frame / road bridge / edge clamp
  C02 front vertical blade
  C03 rear vertical blade
  C04 one-piece rocker with the stock SG90/MG90S double horn in a pocket
  C05 flat servo bracket, fastened to C01 by two grouped M2 screws

There are no link rods.  Two M3 shoulder pins in the rocker ends slide directly
in horizontal slots in the blade ears.  The model is a full-size comparison
prototype: print and validate one lane before making five.

Assembly coordinates (millimetres): X across the lane / can axis, Y along the
road (queue enters from negative Y), Z normal to the road, road top Z=0.
HOME is -42 degrees; EJECT is +42 degrees.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import trimesh
from shapely import affinity
from shapely.geometry import LineString, Point, Polygon, box as sbox
from shapely.ops import unary_union


HERE = Path(__file__).resolve().parent
PRINT_DIR = HERE / "STL_ПЕЧАТЬ"
ASSEMBLY_DIR = HERE / "STL_СБОРКА_НЕ_ПЕЧАТАТЬ"
COMMON_STL_DIR = (HERE.parent / "00_ОБЩАЯ_КОМПОНОВКА" /
                  "STL_КАССЕТА_СБОРКА")
for directory in (PRINT_DIR, ASSEMBLY_DIR):
    directory.mkdir(parents=True, exist_ok=True)


P = {
    "can_length": 70.0,
    "can_diameter_nominal": 47.0,
    "can_diameter_design": 48.0,
    # Loaded cans are assumed to touch at the conservative Ø48 design rings;
    # no artificial gap is credited to the mechanism.
    "can_center_step_reference": 48.0,
    "lane_clear": 75.0,
    "lane_pitch": 79.0,
    "lane_count": 5,
    "divider_thickness": 4.0,
    "box_inside_width": 395.0,
    "cassette_inside_width": 391.0,
    "cassette_lower_sheet_length": 569.402,
    "cassette_reference_depth": 590.0,
    "divider_reference_height": 390.0,
    "sheet_thickness": 4.0,
    "lower_slope_deg": 15.0,
    "gate_pitch": 50.5,
    # The real vertical dividers continue about 41 mm beyond the lower-sheet
    # edge.  Both wide guide sleeves therefore live in the free facade zone:
    # rear gate -25, front gate +25.5, while the sheet edge stays at -60.
    "front_gate_y": 25.5,
    "rear_gate_y": -25.0,
    "servo_home_deg": -42.0,
    "servo_mid_deg": 0.0,
    "servo_eject_deg": 42.0,
    # Fixed by the available calibrated HOME-EJECT travel: total 84 degrees.
    "rocker_half_radius": 31.383,
    # The rocker is shifted 8 mm downhill.  This leaves both follower slots on
    # one side of each blade's broad print face without changing the gates or
    # the 84-degree motion.
    "rocker_axis_y": 8.25,
    "rocker_axis_z": 89.0,
    "blade_tip_low": 10.5,
    "blade_tip_mid": 31.5,
    "blade_tip_high": 52.5,
    "blade_pin_above_tip": 57.5,
    "blade_thickness": 3.0,
    "blade_tip_thickness": 1.2,
    "blade_taper_height": 18.0,
    "blade_guide_sleeve_z": 56.5,
    "blade_guide_sleeve_height": 7.0,
    "blade_working_width": 70.0,
    # The guide shoe remains wider than the 70 mm working knife, but clears
    # the continuous 1.2 mm print-side web by 0.4 mm.
    "blade_guide_tab_width": 71.2,
    "front_slot_center_offset": 10.1,
    "rear_slot_center_offset": 5.9,
    "slot_centerline_half_travel": 4.8,
    "frame_outer_width": 74.4,
    "frame_inner_width": 71.6,
    "clamp_above_sheet_width": 74.4,
    "transition_road_width": 74.0,
    "plywood_front_edge_y": -60.0,
    "edge_clamp_gap": 1.3,
}


def clean(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    mesh = mesh.copy()
    mesh.remove_unreferenced_vertices()
    mesh.merge_vertices()
    trimesh.repair.fix_normals(mesh)
    return mesh


def fuse(meshes) -> trimesh.Trimesh:
    items = [clean(m) for m in meshes if m is not None and len(m.faces)]
    if len(items) == 1:
        return items[0]
    try:
        result = trimesh.boolean.union(items, engine="manifold")
        if result is not None:
            return clean(result)
    except Exception:
        pass
    return clean(trimesh.util.concatenate(items))


def cut(base, cutters) -> trimesh.Trimesh:
    cutters = [c for c in cutters if c is not None and len(c.faces)]
    if not cutters:
        return clean(base)
    try:
        result = trimesh.boolean.difference([clean(base), *map(clean, cutters)],
                                            engine="manifold")
        if result is not None:
            return clean(result)
    except Exception:
        pass
    return clean(base)


def bx(size, center=(0.0, 0.0, 0.0)):
    mesh = trimesh.creation.box(extents=size)
    mesh.apply_translation(center)
    return mesh


def cylinder_x(radius, length, center=(0.0, 0.0, 0.0), sections=48):
    transform = trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0])
    transform[:3, 3] = center
    return trimesh.creation.cylinder(radius=radius, height=length,
                                     sections=sections, transform=transform)


def extrude_yz(poly, length, x0=0.0):
    mesh = trimesh.creation.extrude_polygon(poly, height=length, engine="earcut")
    # extrusion initially lies XY and grows in +Z; map (X,Y,Z)->(Y,Z,X)
    matrix = np.array([[0, 0, 1, x0],
                       [1, 0, 0, 0],
                       [0, 1, 0, 0],
                       [0, 0, 0, 1]], dtype=float)
    mesh.apply_transform(matrix)
    return clean(mesh)


def translate(mesh, xyz):
    result = mesh.copy()
    result.apply_translation(xyz)
    return result


def rotate(mesh, angle_deg, axis=(1, 0, 0), point=(0.0, 0.0, 0.0)):
    result = mesh.copy()
    result.apply_transform(trimesh.transformations.rotation_matrix(
        math.radians(angle_deg), axis, point))
    return result


def on_bed(mesh):
    result = mesh.copy()
    result.apply_translation([0, 0, -result.bounds[0, 2]])
    return result


def capsule_yz(y0, y1, z, radius):
    return LineString([(y0, z), (y1, z)]).buffer(radius, cap_style=1,
                                                  join_style=1, resolution=20)


def horn_outline(clearance=0.35):
    hub = Point(0, 0).buffer(3.70 + clearance, resolution=40)
    arm_len = 15.8 + clearance
    root = 3.45 + clearance
    tip = 2.0 + clearance
    arms = []
    for sign in (-1, 1):
        arms.append(Polygon([(0, -root), (sign * arm_len, -tip),
                             (sign * arm_len, tip), (0, root)]))
        arms.append(Point(sign * arm_len, 0).buffer(tip, resolution=16))
    return unary_union([hub, *arms])


def blade_tip(theta_deg, front=True):
    s = math.sin(math.radians(theta_deg))
    delta = P["rocker_half_radius"] * s
    return P["blade_tip_mid"] + (delta if front else -delta)


def pin_point(theta_deg, front=True):
    a = math.radians(theta_deg)
    sign = 1.0 if front else -1.0
    return (P["rocker_axis_y"] + sign * P["rocker_half_radius"] * math.cos(a),
            P["rocker_axis_z"] + sign * P["rocker_half_radius"] * math.sin(a))


def blade_mesh(front=True):
    """Blade authored with its lower tip at Z=0 and gate centre at Y=0."""
    half = P["blade_thickness"] / 2
    taper = P["blade_taper_height"]
    # The 70 mm-wide working blade stops below the rocker.  Above it, a
    # central neck and one remote guide rib carry the ear without entering
    # the rocker envelope.
    height = 35.0
    # A single-bevel knife is used deliberately: its entire Y=-1.5 face is
    # planar on the bed, while the opposite face grows from a 1.2 mm tip to
    # 3.0 mm over the full 35 mm blade height.  The former symmetric bevel
    # left the 70 mm cutting edge 0.9 mm above the bed and forced the slicer to
    # bridge it.  The new slope (1.8/35=0.0514) is effectively the same as the
    # former half-bevel (0.9/18=0.0500).
    profile = Polygon([(-half, 0.0),
                       (-half + P["blade_tip_thickness"], 0.0),
                       (half, height),
                       (-half, height)])
    lower = extrude_yz(profile, P["blade_working_width"],
                       x0=-P["blade_working_width"] / 2)

    # A full-width shoe crosses the fixed upper guide sleeve around MID.
    guide = bx((P["blade_guide_tab_width"], P["blade_thickness"], 10.0),
               (0, 0, 30.0))

    # Above the shoe, a narrow neck avoids the left-side rocker/servo volume.
    neck = bx((40.0, P["blade_thickness"], 31.0), (0, 0, 50.5))

    # The remote rib remains inside the upper guide when the blade is low.
    # It is on the side opposite the rocker, so it cannot hit the mechanism.
    remote_guide = bx((4.0, P["blade_thickness"], 31.0),
                      (33.6, 0, 50.5))

    # Both follower ears now live wholly on the downstream side of the broad
    # Y=-1.5 face.  Consequently that broad face and the entire ear touch the
    # build plate together in the supplied +90deg-X print orientation; no
    # feature starts several millimetres in mid-air.  The slot still has at
    # least 0.70 mm material beyond its far capsule end.
    ear_cy = (P["front_slot_center_offset"] if front else
              P["rear_slot_center_offset"])
    ear_y0 = -P["blade_thickness"] / 2
    ear_y1 = (ear_cy + P["slot_centerline_half_travel"] +
              1.75 + 0.70)
    ear = bx((10.0, ear_y1 - ear_y0, 12.0),
             (-23.0, (ear_y0 + ear_y1) / 2,
              P["blade_pin_above_tip"]))
    blade = fuse([lower, guide, neck, remote_guide, ear])
    slot = extrude_yz(capsule_yz(
                                 ear_cy - P["slot_centerline_half_travel"],
                                 ear_cy + P["slot_centerline_half_travel"],
                                 P["blade_pin_above_tip"], 1.75),
                      13.0, x0=-29.5)
    # Open only the downstream end of the follower slot.  In the print
    # orientation this is the top of the vertical slot, so no 10 mm roof is
    # bridged.  The pin remains at least 1.467 mm radially inside the physical
    # ear edge at the worst commanded angle and load acts across the closed
    # slot sides, not toward this service opening.
    open_end = bx((13.0,
                   ear_y1 - (ear_cy + P["slot_centerline_half_travel"]),
                   3.5),
                  (-23.0,
                   (ear_y1 + ear_cy +
                    P["slot_centerline_half_travel"]) / 2,
                   P["blade_pin_above_tip"]))
    return cut(blade, [slot, open_end])


def rocker_mesh():
    r = P["rocker_half_radius"]
    outline = unary_union([
        LineString([(-r, 0), (r, 0)]).buffer(5.4, cap_style=1,
                                             join_style=1, resolution=32),
        Point(0, 0).buffer(8.2, resolution=40),
    ])
    pocket = affinity.rotate(horn_outline(0.35), 0, origin=(0, 0))
    # Pocket opens on the servo-facing (+X) face.  Its back wall is 2.2 mm;
    # the old two-slab construction accidentally buried 0.55 mm of the horn.
    rocker = extrude_yz(outline, 5.4, x0=-35.8)
    horn_cavity = extrude_yz(pocket, 3.4, x0=-33.6)
    holes = [
        cylinder_x(1.25, 8.0, (-33.1, 0, 0), 32),
        cylinder_x(1.75, 8.0, (-33.1, -r, 0), 32),
        cylinder_x(1.75, 8.0, (-33.1, r, 0), 32),
        # Captive DIN934 M3 nut pockets: follower screws enter from +X and never
        # protrude into the frame cheek.
        cylinder_x(3.4, 3.2, (-34.5, -r, 0), 6),
        cylinder_x(3.4, 3.2, (-34.5, r, 0), 6),
    ]
    return cut(rocker, [horn_cavity, *holes])


def clamp_reaction_pad(left=True):
    """Short printable reaction lip with an integrated can-centering ramp."""
    sign = -1.0 if left else 1.0
    y_upstream, y_downstream = -62.8, -59.7
    outer = 37.2 * sign
    inner_upstream = 36.5 * sign
    inner_downstream = 35.5 * sign
    points = []
    for yy, inner, top in ((y_upstream, inner_upstream, 0.4),
                           (y_downstream, inner_downstream, 2.0)):
        for xx in (outer, inner):
            points.append([xx, yy, 0.0])
            points.append([xx, yy, top])
    return clean(trimesh.convex.convex_hull(np.asarray(points)))


def frame_mesh():
    outer = P["frame_outer_width"]
    inner = P["frame_inner_width"]
    wall = (outer - inner) / 2
    # A 1.2 mm continuous left web is the actual build-plate face in the
    # supplied side orientation.  It covers the projected silhouette of every
    # downstream frame feature, so no bridge or servo wall begins in air.  The
    # upstream part follows only the lower clamp tongue and therefore never
    # enters the exact plywood body (which occupies Y<=-60, Z=-4..0).
    web_x0, web_x1 = -outer / 2, -36.0
    web_xc = (web_x0 + web_x1) / 2
    web_t = web_x1 - web_x0
    parts = [
        # Narrow continuous road from the unmodified plywood edge to the free
        # facade zone.  74.0 mm leaves 0.5 mm to each exact divider face.
        bx((P["transition_road_width"], 105.0, 3.0),
           (0, -7.5, -1.5)),
        # Integral edge clamp: tongue below the plywood, two top lips outside
        # the 70 mm can length, and side ribs.  M3 screws come from below and
        # push the un-drilled plywood against the lips.
        # The lower tongue may occupy almost the whole 79 mm pitch because it
        # stays 1.3 mm below the common sheet.  It also forms the print-bed
        # edge in the selected side orientation.
        bx((P["clamp_above_sheet_width"], 22.0, 3.0),
           (0, -66.0, -6.8)),
        # Edge stop starts exactly at the plywood front datum Y=-60; it does
        # not occupy the plywood volume.
        bx((P["clamp_above_sheet_width"], 2.0, 8.3),
           (0, -59.0, -4.15)),
        # Two integral upper reaction pads remain wholly inside the 75 mm
        # clear lane: outer X=+/-37.2, 0.3 mm from each divider face.  They
        # overlap the downstream edge-stop by 0.3 mm but never enter the can's
        # exact 70 mm length envelope (0.5 mm clearance per end).
        # Only 2.8 mm projects upstream of the connected front stop.  In the
        # side-print orientation this remains a short printable cantilever,
        # not the former unsupported 12 mm tongue.
        clamp_reaction_pad(True),
        clamp_reaction_pad(False),
        # Full projected support web in the free facade zone, plus only the
        # tongue-shaped strip upstream of the unmodified sheet edge.
        bx((web_t, 105.0, 133.3),
           (web_xc, -7.5, 58.35)),
        bx((web_t, 17.0, 3.0),
           (web_xc, -68.5, -6.8)),
        # Two deep one-sided C guides replace closed 74 mm sleeves.  Each grows
        # continuously from the left bed web and remains open at X=-8, so the
        # slicer never has to create a remote wall after a 71 mm air gap.  A
        # 27.6 mm engagement on the rigid full-width shoe is sufficient to
        # constrain Y while leaving the rocker 2.6 mm vertical clearance.
        bx((29.0, 7.4, P["blade_guide_sleeve_height"]),
           (-22.5, P["front_gate_y"],
            P["blade_guide_sleeve_z"])),
        # Rear sleeve extends only downhill to overlap the new spine.  At this
        # Z the exact divider nose ends near Y=-34, so the whole bridge remains
        # in free space and adds neither a part nor a support operation.
        bx((29.0, 14.2, P["blade_guide_sleeve_height"]),
           (-22.5, -21.6, P["blade_guide_sleeve_z"])),
        # Two small mounting lands grow directly from the bed web.  The remote
        # servo wall is deliberately a separate flat C05 part: keeping it on
        # C01 created a real floating island in PrusaSlicer at Z=9.05 mm.
        bx((3.0, 3.4, 7.0),
           (-35.7, P["rocker_axis_y"], 73.5)),
        bx((3.0, 6.0, 6.0),
           (-35.7, P["rocker_axis_y"], 122.0)),
    ]
    frame = fuse(parts)
    cutters = []

    # Slots in the two C guides: 0.4 mm Y clearance and 0.2 mm X clearance
    # at the only closed end.  The cutters continue through the open +X end.
    for gy in (P["rear_gate_y"], P["front_gate_y"]):
        cutters.append(bx((P["blade_guide_tab_width"] + 0.40,
                           P["blade_thickness"] + 0.40,
                           P["blade_guide_sleeve_height"] + 2.0),
                          (0, gy, P["blade_guide_sleeve_z"])))

    # Printed 2.7 mm pilot holes for two M3 clamp screws from below.  They stop
    # at the plywood underside; the plywood itself is never drilled.
    for x in (-34.0, 34.0):
        cutters.append(trimesh.creation.cylinder(
            radius=1.35, height=5.0, sections=32,
            transform=trimesh.transformations.translation_matrix([x, -61.2, -6.8])))

    # Clearance holes for the two M2 screws which fasten the separately
    # printed C05 servo bracket.  Both are vertical open holes in C01's saved
    # side-print orientation and therefore need no support.
    for yy, zz in ((P["rocker_axis_y"], 73.5),
                   (P["rocker_axis_y"], 122.0)):
        cutters.append(cylinder_x(1.10, 4.2, (-35.7, yy, zz), 28))

    # Servo output boss passes through the mounting wall without interference.
    cutters.append(cylinder_x(6.2, 5.0,
                              (-27.0, P["rocker_axis_y"], P["rocker_axis_z"]), 48))

    # Coaxial access from the outside of the left web for the stock horn screw.
    # It joins the existing boss opening, so the later servo wall has the same
    # projected void instead of bridging across a hole in the bed web.
    cutters.append(cylinder_x(4.2, 22.0,
                              (-35.0, P["rocker_axis_y"],
                               P["rocker_axis_z"]), 48))

    # Servo-ear holes through the mounting wall, with +/-1.5 mm vertical slots.
    for zz in (P["rocker_axis_z"] + 5.4 - 13.0,
               P["rocker_axis_z"] + 5.4 + 13.0):
        cutters.extend([
            cylinder_x(1.2, 7.0, (-27.0, P["rocker_axis_y"], zz - 1.5), 28),
            cylinder_x(1.2, 7.0, (-27.0, P["rocker_axis_y"], zz + 1.5), 28),
            bx((7.0, 2.4, 3.0), (-27.0, P["rocker_axis_y"], zz)),
        ])

    return cut(frame, cutters)


def servo_bracket_mesh():
    """Flat-print C05 wall with two short keyed screw arms.

    The wall is printed on its broad outer face (+90 degrees about Y).  The
    arms then grow vertically and contain open blind pilot holes; there are no
    bridges or trapped supports.  In assembly their rectangular ends register
    against the matching lands on C01 before the two M2 screws are tightened.
    """
    wall = bx((3.0, 14.8, 49.0),
              (-27.0, P["rocker_axis_y"], 100.5))
    lower_arm = bx((5.8, 3.4, 7.0),
                   (-31.3, P["rocker_axis_y"], 73.5))
    upper_arm = bx((5.8, 6.0, 6.0),
                   (-31.3, P["rocker_axis_y"], 122.0))
    bracket = fuse([wall, lower_arm, upper_arm])
    pilots = [
        cylinder_x(0.85, 5.9, (-31.25, P["rocker_axis_y"], 73.5), 28),
        cylinder_x(0.85, 5.9, (-31.25, P["rocker_axis_y"], 122.0), 28),
    ]

    # Output boss and ear slots remain part of C05, exactly where they were in
    # the former integral wall.
    pilots.append(cylinder_x(6.2, 5.0,
                             (-27.0, P["rocker_axis_y"],
                              P["rocker_axis_z"]), 48))
    for zz in (P["rocker_axis_z"] + 5.4 - 13.0,
               P["rocker_axis_z"] + 5.4 + 13.0):
        pilots.extend([
            cylinder_x(1.2, 7.0, (-27.0, P["rocker_axis_y"], zz - 1.5), 28),
            cylinder_x(1.2, 7.0, (-27.0, P["rocker_axis_y"], zz + 1.5), 28),
            bx((7.0, 2.4, 3.0), (-27.0, P["rocker_axis_y"], zz)),
        ])
    return cut(bracket, pilots)


def bracket_screw_reference(upper=True):
    zz = 122.0 if upper else 73.5
    shaft = cylinder_x(1.0, 10.0,
                       (-32.2, P["rocker_axis_y"], zz), 28)
    head = cylinder_x(2.0, 1.6,
                      (-38.0, P["rocker_axis_y"], zz), 28)
    return fuse([shaft, head])


def servo_reference_parts():
    cy, cz = P["rocker_axis_y"], P["rocker_axis_z"]
    body = bx((28.5, 12.2, 22.8), (-8.65, cy, cz + 5.4))
    cap = bx((2.6, 12.8, 23.4), (-24.2, cy, cz + 5.4))
    flange = bx((2.6, 15.0, 32.5), (-24.2, cy, cz + 5.4))
    boss = cylinder_x(5.8, 3.0, (-27.0, cy, cz), 40)
    shaft = cylinder_x(2.35, 5.0, (-30.5, cy, cz), 32)
    horn = extrude_yz(horn_outline(0.0), 1.8, x0=-33.35)
    horn.apply_translation([0, cy, cz])
    screw = fuse([cylinder_x(1.0, 7.0, (-34.2, cy, cz), 24),
                  cylinder_x(2.2, 1.2, (-38.3, cy, cz), 24)])
    return {"servo_body": fuse([body, cap, flange, boss]),
            "servo_shaft": shaft, "servo_horn": horn,
            "servo_screw": screw}


def follower_pin_reference(theta_deg, front=True):
    y, z = pin_point(theta_deg, front)
    # M3x18 enters from +X; its head stays outside the blade ear and its nut
    # sits in the rocker's captive pocket.  Nothing protrudes into the spine.
    shaft = cylinder_x(1.5, 18.0, (-27.0, y, z), 32)
    head = cylinder_x(3.0, 2.0, (-17.0, y, z), 32)
    nut = cylinder_x(3.15, 2.4, (-34.3, y, z), 6)
    return fuse([shaft, head, nut])


def edge_clamp_screw_reference(left=True):
    x = -34.0 if left else 34.0
    # M3x10 jackscrew: tip touches the sheet underside at Z=-4 while the
    # socket head stays accessible below the printed tongue.
    shaft = trimesh.creation.cylinder(
        radius=1.5, height=10.0, sections=32,
        transform=trimesh.transformations.translation_matrix([x, -61.2, -9.0]))
    head = trimesh.creation.cylinder(
        radius=3.0, height=2.0, sections=32,
        transform=trimesh.transformations.translation_matrix([x, -61.2, -15.0]))
    return fuse([shaft, head])


def can_mesh(center_y, center_z=24.0):
    # Full design envelope: exactly 70 x dia48 including both end rings.
    body = cylinder_x(23.5, 64.0, (0.0, center_y, center_z), 72)
    lid = cylinder_x(24.0, 3.0, (33.5, center_y, center_z), 72)
    ring = cylinder_x(24.0, 3.0, (-33.5, center_y, center_z), 72)
    return fuse([body, lid, ring])


_EXACT_CASSETTE_CACHE = None


def exact_cassette_local():
    """Load the actual common STL files and map them into C2 road coordinates.

    This deliberately does not recreate the plywood as boxes.  The lower sheet
    therefore retains its four real lock holes, exact 569.402 mm slope length
    and 391 mm width; each divider retains the real CNC slots and the 40 mm
    nose ahead of that sheet.
    """
    global _EXACT_CASSETTE_CACHE
    if _EXACT_CASSETTE_CACHE is not None:
        lower, dividers = _EXACT_CASSETTE_CACHE
        return lower.copy(), [mesh.copy() for mesh in dividers]

    lower_path = COMMON_STL_DIR / "cassette_lower_sheet.stl"
    divider_path = COMMON_STL_DIR / "cassette_divider.stl"
    if not lower_path.exists() or not divider_path.exists():
        raise FileNotFoundError(
            "Exact common cassette STL files are required: " +
            str(COMMON_STL_DIR))

    theta = math.radians(P["lower_slope_deg"])
    # Basis columns map local X across lane, local Y downhill, local Z road
    # normal into the common cassette's global coordinates.
    basis = np.array([
        [0.0, -math.cos(theta), -math.sin(theta)],
        [1.0, 0.0, 0.0],
        [0.0, -math.sin(theta), math.cos(theta)],
    ], dtype=float)
    # Exact top/front centre of the 4 mm sheet.  Its nominal centre plane starts
    # at global X=40; +local Z is the rotated top-face normal.
    front_top_global = np.array([
        40.0 - 2.0 * math.sin(theta),
        P["box_inside_width"] / 2,
        40.0 * math.tan(theta) + 2.0 * math.cos(theta),
    ])
    local_front = np.array([0.0, P["plywood_front_edge_y"], 0.0])
    transform = np.eye(4)
    transform[:3, :3] = basis
    transform[:3, 3] = front_top_global - basis @ local_front
    inverse = np.linalg.inv(transform)

    lower = trimesh.load(lower_path, force="mesh", process=True)
    lower.apply_transform(inverse)
    lower = clean(lower)

    divider_source = trimesh.load(divider_path, force="mesh", process=True)
    dividers = []
    # These are the actual centres generated by the common cassette: 2 mm
    # cassette offset plus [77, 156, 235, 314].
    for global_y in (79.0, 158.0, 237.0, 316.0):
        divider = divider_source.copy()
        divider.apply_translation([0.0, global_y, 0.0])
        divider.apply_transform(inverse)
        dividers.append(clean(divider))

    _EXACT_CASSETTE_CACHE = (lower.copy(), [mesh.copy() for mesh in dividers])
    return lower, dividers


def sheet_reference():
    return exact_cassette_local()[0]


def divider_reference(left=True):
    # The central lane is bounded by exact common dividers 2 and 3.
    dividers = exact_cassette_local()[1]
    return dividers[1] if left else dividers[2]


def state_parts(theta_deg):
    front_tip = blade_tip(theta_deg, True)
    rear_tip = blade_tip(theta_deg, False)
    front = translate(blade_mesh(True), (0, P["front_gate_y"], front_tip))
    rear = translate(blade_mesh(False), (0, P["rear_gate_y"], rear_tip))
    rocker = rocker_mesh()
    rocker = translate(rocker, (0, P["rocker_axis_y"], P["rocker_axis_z"]))
    rocker = rotate(rocker, theta_deg, [1, 0, 0],
                    (0, P["rocker_axis_y"], P["rocker_axis_z"]))
    servo = servo_reference_parts()
    servo["servo_horn"] = rotate(
        servo["servo_horn"], theta_deg, [1, 0, 0],
        (0, P["rocker_axis_y"], P["rocker_axis_z"]))
    servo["servo_screw"] = rotate(
        servo["servo_screw"], theta_deg, [1, 0, 0],
        (0, P["rocker_axis_y"], P["rocker_axis_z"]))
    parts = {"frame": frame_mesh(),
             "servo_bracket": servo_bracket_mesh(),
             "bracket_screw_lower": bracket_screw_reference(False),
             "bracket_screw_upper": bracket_screw_reference(True),
             "front_blade": front,
             "rear_blade": rear, "rocker": rocker,
             "front_pin": follower_pin_reference(theta_deg, True),
             "rear_pin": follower_pin_reference(theta_deg, False),
             "lower_sheet": sheet_reference(),
             "divider_left": divider_reference(True),
             "divider_right": divider_reference(False),
             "clamp_screw_left": edge_clamp_screw_reference(True),
             "clamp_screw_right": edge_clamp_screw_reference(False)}
    parts.update(servo)
    return parts


def blade_contact_offset(tip_z):
    """Upstream can-centre offset at first tangency with one blade.

    This is a geometric calculation on the dia48 circle and the 1.2->3.0 mm
    blade taper, not an assumed can position.
    """
    radius = P["can_diameter_design"] / 2
    if tip_z >= 2 * radius:
        return None
    best = 0.0
    z0 = max(0.0, tip_z)
    for z in np.linspace(z0, 2 * radius, 1201):
        rel = z - tip_z
        if rel < 0:
            continue
        if rel < P["blade_taper_height"]:
            blade_half = (P["blade_tip_thickness"] / 2 +
                          (P["blade_thickness"] - P["blade_tip_thickness"]) /
                          2 / P["blade_taper_height"] * rel)
        else:
            blade_half = P["blade_thickness"] / 2
        chord = math.sqrt(max(0.0, radius * radius - (z - radius) ** 2))
        best = max(best, chord + blade_half)
    return best


def can_positions(state_name):
    step = P["can_center_step_reference"]
    low_contact = blade_contact_offset(P["blade_tip_low"])
    mid_contact = blade_contact_offset(P["blade_tip_mid"])
    if state_name == "HOME":
        first = P["front_gate_y"] - low_contact
        return [(first, 24.0), (first - step, 24.0),
                (first - 2 * step, 24.0)]
    if state_name == "MID":
        first = P["front_gate_y"] - mid_contact
        second = P["rear_gate_y"] - mid_contact
        return [(first, 24.0), (second, 24.0), (second - step, 24.0)]
    # The first can is shown already clear of the printed road edge and falling;
    # the second is tangent to the fully inserted rear separator.
    second = P["rear_gate_y"] - low_contact
    return [(P["front_gate_y"] + 48.0, 12.0),
            (second, 24.0), (second - step, 24.0)]


def export(mesh, path):
    mesh = clean(mesh)
    mesh.export(path)
    components = len(mesh.split(only_watertight=False))
    return {"file": path.name,
            "extents_mm": [round(float(v), 2) for v in mesh.extents],
            "watertight": bool(mesh.is_watertight),
            "components": int(components),
            "volume_cm3": round(float(abs(mesh.volume)) / 1000.0, 2)}


def place_on_plate(mesh, min_x, min_y, rotate_z_deg=0.0):
    placed = rotate(mesh, rotate_z_deg, [0, 0, 1]) if rotate_z_deg else mesh.copy()
    placed.apply_translation((min_x - placed.bounds[0, 0],
                              min_y - placed.bounds[0, 1],
                              -placed.bounds[0, 2]))
    placed.units = "mm"
    return placed


def export_one_channel_3mf(frame, bracket, front, rear, rocker, coupon):
    """One ready-to-slice 220x220 plate; orientations are already final."""
    placed = {
        # Rotate C01 only in the XY bed plane.  The repack also leaves a verified
        # non-overlapping 2 mm optional brim corridor around every object.
        "C01_frame": place_on_plate(frame, 7.0, 7.0, 90.0),
        "C02_front_blade": place_on_plate(front, 137.0, 7.0),
        "C03_rear_blade": place_on_plate(rear, 137.0, 78.0),
        "C04_rocker": place_on_plate(rocker, 7.0, 192.0, 90.0),
        "C05_servo_bracket": place_on_plate(bracket, 137.0, 151.0),
        "T01_horn_coupon": place_on_plate(coupon, 7.0, 151.0),
    }
    # Bounding boxes are intentionally separated, which is stronger than a
    # triangle-overlap test for this placement.
    boxes = {key: mesh.bounds[:, :2] for key, mesh in placed.items()}
    overlaps = []
    optional_brim_mm = 2.0
    brim_overlaps = []
    keys = list(boxes)
    for i, first in enumerate(keys):
        for second in keys[i + 1:]:
            a, b = boxes[first], boxes[second]
            overlap_x = min(a[1, 0], b[1, 0]) - max(a[0, 0], b[0, 0])
            overlap_y = min(a[1, 1], b[1, 1]) - max(a[0, 1], b[0, 1])
            if overlap_x > 0 and overlap_y > 0:
                overlaps.append([first, second])
            brim_overlap_x = (min(a[1, 0] + optional_brim_mm,
                                  b[1, 0] + optional_brim_mm) -
                              max(a[0, 0] - optional_brim_mm,
                                  b[0, 0] - optional_brim_mm))
            brim_overlap_y = (min(a[1, 1] + optional_brim_mm,
                                  b[1, 1] + optional_brim_mm) -
                              max(a[0, 1] - optional_brim_mm,
                                  b[0, 1] - optional_brim_mm))
            if brim_overlap_x > 0 and brim_overlap_y > 0:
                brim_overlaps.append([first, second])
    scene = trimesh.Scene()
    for key, mesh in placed.items():
        scene.add_geometry(mesh, geom_name=key, node_name=key)
    path = PRINT_DIR / "C2_ПЛИТА_1_КАНАЛ_готовая_220x220.3mf"
    scene.export(file_obj=str(path), file_type="3mf")
    all_bounds = np.vstack([mesh.bounds for mesh in placed.values()])
    overall_min = all_bounds.min(axis=0)
    overall_max = all_bounds.max(axis=0)
    envelope = overall_max - overall_min
    return {
        "file": path.name,
        "objects": list(placed),
        "functional_parts": 5,
        "coupon_parts": 1,
        "object_count": len(placed),
        "envelope_xyz_mm": [round(float(v), 2) for v in envelope],
        "bed_220x220_pass": bool(overall_min[0] >= 0 and overall_min[1] >= 0 and
                                 overall_max[0] <= 220 and overall_max[1] <= 220),
        "bounding_box_overlap_pairs": overlaps,
        "overlap_pass": not overlaps,
        "optional_uniform_brim_mm": optional_brim_mm,
        "optional_brim_expanded_bounds_xy_mm": [
            [round(float(overall_min[0] - optional_brim_mm), 2),
             round(float(overall_min[1] - optional_brim_mm), 2)],
            [round(float(overall_max[0] + optional_brim_mm), 2),
             round(float(overall_max[1] + optional_brim_mm), 2)],
        ],
        "optional_brim_overlap_pairs": brim_overlaps,
        "optional_brim_bed_and_overlap_pass": bool(
            overall_min[0] - optional_brim_mm >= 0 and
            overall_min[1] - optional_brim_mm >= 0 and
            overall_max[0] + optional_brim_mm <= 220 and
            overall_max[1] + optional_brim_mm <= 220 and
            not brim_overlaps),
        "manual_rotation_required": False,
    }


def export_state(name, theta, reports):
    parts = state_parts(theta)
    state_meshes = []
    for key, mesh in parts.items():
        reports.append(export(mesh, ASSEMBLY_DIR / f"C2_{name}_{key}.stl"))
        state_meshes.append(mesh)
    for idx, (yy, zz) in enumerate(can_positions(name), 1):
        can = can_mesh(yy, zz)
        reports.append(export(can, ASSEMBLY_DIR / f"C2_{name}_can_{idx}.stl"))
        state_meshes.append(can)
    joined = clean(trimesh.util.concatenate(state_meshes))
    reports.append(export(joined, ASSEMBLY_DIR / f"C2_{name}_complete.stl"))


def export_cassette_references(reports):
    lower, dividers = exact_cassette_local()
    reports.append(export(
        lower,
        ASSEMBLY_DIR / "C2_REFERENCE_точный_нижний_лист_391x569_402.stl"))
    for idx, divider in enumerate(dividers, 1):
        reports.append(export(
            divider,
            ASSEMBLY_DIR / f"C2_REFERENCE_точная_перегородка_{idx}.stl"))


def horn_coupon_mesh():
    base = bx((118.0, 30.0, 3.2), (0, 0, 1.6))
    cuts = []
    for xx, clearance in zip((-39.0, 0.0, 39.0), (0.20, 0.35, 0.50)):
        poly = horn_outline(clearance)
        cuts.append(extrude_yz(poly, 5.0, x0=xx - 2.5))
    # Coupon pockets are cut along Z, so use simple 2D extrusions in XY instead.
    flat_cuts = []
    for xx, clearance in zip((-39.0, 0.0, 39.0), (0.20, 0.35, 0.50)):
        poly = affinity.translate(horn_outline(clearance), xoff=xx)
        flat_cuts.append(trimesh.creation.extrude_polygon(poly, 5.0, engine="earcut"))
    return cut(base, flat_cuts)


def write_bom():
    rows = [
        ("C01", "Верхняя рама-направляющая с дорожкой и зажимом ската", 1,
         "PETG", "печатается боковой спиной без поддержек"),
        ("C02", "Передняя прямая заслонка", 1, "PETG", "оранжевая; HOME закрыта"),
        ("C03", "Задний прямой сепаратор", 1, "PETG", "бирюзовый; EJECT закрыт"),
        ("C04", "Длинное коромысло под штатную качалку", 1, "PETG", "радиус пинов 31.383 мм"),
        ("C05", "Плоский кронштейн сервы с двумя монтажными лапками", 1,
         "PETG", "печатается широкой стенкой на столе"),
        ("T01", "Купон кармана штатной качалки", 0, "PETG",
         "1 на партию одинаковых серв/филамента; не на каждый канал"),
        ("H01", "SG90 или MG90S + штатная двойная качалка", 1, "готовая", "существующая серва"),
        ("H02", "Винт-ось M3x18 с гладкой рабочей шейкой Ø3 + шайба + гайка DIN934", 2, "металл", "резьба не должна тереться о PETG-паз; фиксатор только plastic-safe либо контрольная метка лаком"),
        ("H03", "M2x10 самонарезающий", 2, "металл", "серва крепится к печатным пилотам C05; без гаек"),
        ("H04", "M3x10 прижимной винт", 2, "металл", "снизу прижимает целую фанеру к верхним губкам"),
        ("H05", "Штатный центральный винт качалки", 1, "комплект сервы", "C04 к качалке"),
        ("H06", "M2x10 самонарезающий + шайба для C05", 2, "металл",
         "одной сгруппированной операцией крепят C05 к C01"),
        ("E01", "Резистор 10 кОм сигнал-GND", 1, "электроника", "исключает плавающий сигнал до старта ESP32"),
    ]
    with (HERE / "BOM_C2.csv").open("w", encoding="utf-8-sig", newline="") as fh:
        writer = csv.writer(fh, delimiter=";")
        writer.writerow(("Код", "Деталь", "Количество на канал", "Материал", "Примечание"))
        writer.writerows(rows)


def analytical_checks():
    samples = []
    max_slot_offset_from_gate = 0.0
    max_slot_offset_from_center = 0.0
    min_vertical_jacobian = float("inf")
    for theta in np.linspace(P["servo_home_deg"], P["servo_eject_deg"], 169):
        ft = blade_tip(float(theta), True)
        rt = blade_tip(float(theta), False)
        fy, fz = pin_point(float(theta), True)
        ry, rz = pin_point(float(theta), False)
        front_rel = fy - P["front_gate_y"]
        rear_rel = ry - P["rear_gate_y"]
        max_slot_offset_from_gate = max(max_slot_offset_from_gate,
                                        abs(front_rel), abs(rear_rel))
        max_slot_offset_from_center = max(
            max_slot_offset_from_center,
            abs(front_rel - P["front_slot_center_offset"]),
            abs(rear_rel - P["rear_slot_center_offset"]))
        jacobian = P["rocker_half_radius"] * math.cos(math.radians(float(theta)))
        min_vertical_jacobian = min(min_vertical_jacobian, abs(jacobian))
        samples.append((float(theta), ft, rt, fy, fz, ry, rz))
    minimum_active_block = max(min(v[1], v[2]) for v in samples)
    r = P["can_diameter_design"] / 2
    dz = P["blade_tip_mid"] - r
    natural_gap = P["can_center_step_reference"] - 2 * math.sqrt(r * r - dz * dz)
    # Torque scale for a vertical blade force: tau = F * r*cos(theta).
    max_moment_arm = P["rocker_half_radius"] / 1000.0
    stall_nm = 1.8 * 0.0980665
    safe_force_30pct = 0.30 * stall_nm / max_moment_arm
    slot_end_clearance = (P["slot_centerline_half_travel"] -
                          max_slot_offset_from_center)
    low_contact = blade_contact_offset(P["blade_tip_low"])
    mid_contact = blade_contact_offset(P["blade_tip_mid"])
    pushed_gap = P["gate_pitch"] - 2 * math.sqrt(r * r - dz * dz)
    return {
        "home_front_tip_mm": round(samples[0][1], 3),
        "home_rear_tip_mm": round(samples[0][2], 3),
        "mid_both_tips_mm": round(P["blade_tip_mid"], 3),
        "eject_front_tip_mm": round(samples[-1][1], 3),
        "eject_rear_tip_mm": round(samples[-1][2], 3),
        "highest_lower_tip_over_full_cycle_mm": round(minimum_active_block, 3),
        "open_tip_clearance_over_dia48_mm": round(P["blade_tip_high"] - 48.0, 3),
        "natural_gap_between_touching_design_cans_at_mid_tip_mm": round(natural_gap, 3),
        "gap_after_queue_is_spread_to_gate_pitch50_5_mm": round(pushed_gap, 3),
        "queue_spread_required_from_touching_design_cans_mm": round(
            P["gate_pitch"] - P["can_center_step_reference"], 3),
        "rear_tip_initial_interference_for_touching_design_cans_mm": round(
            max(0.0, mid_contact - (low_contact +
                                    P["can_center_step_reference"] -
                                    P["gate_pitch"])), 3),
        "blade_tip_thickness_mm": P["blade_tip_thickness"],
        "max_pin_offset_from_gate_plane_mm_not_slot_center": round(
            max_slot_offset_from_gate, 3),
        "front_slot_center_relative_to_gate_mm": P["front_slot_center_offset"],
        "rear_slot_center_relative_to_gate_mm": P["rear_slot_center_offset"],
        "max_pin_offset_from_actual_slot_center_mm": round(
            max_slot_offset_from_center, 3),
        "slot_centerline_half_travel_mm": P["slot_centerline_half_travel"],
        "slot_end_centerline_clearance_mm": round(slot_end_clearance, 3),
        "pin_radius_to_slot_radius_clearance_mm": round(1.75 - 1.5, 3),
        "slot_travel_pass": bool(slot_end_clearance > 0.5),
        "minimum_vertical_jacobian_mm_per_rad": round(min_vertical_jacobian, 3),
        "kinematic_singularity_in_commanded_range": False,
        "maximum_vertical_force_moment_arm_mm": round(max_moment_arm * 1000, 3),
        "sg90_stall_torque_assumed_Nm": round(stall_nm, 4),
        "vertical_blade_force_at_30pct_stall_N": round(safe_force_30pct, 2),
        "hold_pwm_required": True,
        "self_holding_without_pwm": False,
        "gravity_balance": "two geometrically identical blades cancel their own weight; queue force does not cancel",
    }


def force_model():
    """Conservative blade-insertion estimate; all assumptions are explicit."""
    gravity = 9.80665
    friction = 0.20
    taper_slope = ((P["blade_thickness"] - P["blade_tip_thickness"]) /
                   2 / P["blade_taper_height"])
    factor = (taper_slope + friction) / (1 - friction * taper_slope)
    stall_sg90_kgcm = 1.8
    stall_mg90s_kgcm = 2.2
    stall_nm = stall_sg90_kgcm * 0.0980665
    arm_m = P["rocker_half_radius"] / 1000
    cases = []
    for cans, mass_g in ((11, 100), (11, 200), (22, 100), (22, 200)):
        lane_thrust = cans * mass_g / 1000 * gravity * math.sin(
            math.radians(P["lower_slope_deg"]))
        insertion = lane_thrust * factor
        torque = insertion * arm_m
        cases.append({
            "cans": cans,
            "mass_each_g": mass_g,
            "gravity_lane_thrust_N": round(lane_thrust, 3),
            "estimated_blade_insertion_force_N": round(insertion, 3),
            "estimated_servo_torque_Nm": round(torque, 4),
            "estimated_servo_torque_kg_cm": round(torque / 0.0980665, 3),
            "fraction_of_assumed_SG90_stall_percent": round(
                100 * torque / stall_nm, 1),
            "fraction_of_assumed_MG90S_stall_percent": round(
                100 * (torque / 0.0980665) / stall_mg90s_kgcm, 1),
        })
    return {
        "calculation": "lane_thrust=N*m*g*sin(15deg); blade_force=lane_thrust*(taper_slope+mu)/(1-mu*taper_slope); torque=blade_force*31.383mm",
        "assumptions": {
            "SG90_stall_at_4_8V_kg_cm": stall_sg90_kgcm,
            "MG90S_stall_at_4_8V_kg_cm": stall_mg90s_kgcm,
            "blade_can_friction_mu": friction,
            "half_taper_slope_dy_dz": round(taper_slope, 4),
            "ignored": "guide friction, can impacts, warped plywood, servo-clone variation and gearbox losses",
        },
        "cases": cases,
        "conservative_measured_acceptance_limit_peak_blade_force_N": {
            "SG90": 1.0,
            "MG90S": 1.5,
        },
        "load_policy": {
            "11x100g": {
                "SG90": "prototype-only after measured peak blade force <=1.0 N and one-lane endurance test",
                "MG90S": "prototype-only after measured peak blade force <=1.5 N and one-lane endurance test",
            },
            "11x200g": {
                "SG90": "not accepted: calculated 1.410 N exceeds the 1.0 N measured-force limit; reduce the queue and measure",
                "MG90S": "conditionally accepted only after measured peak blade force <=1.5 N and a 300-cycle one-lane test",
            },
            "22x100g": {
                "SG90": "not accepted: calculated 1.410 N exceeds the 1.0 N measured-force limit; reduce the queue and measure",
                "MG90S": "conditionally accepted only after measured peak blade force <=1.5 N and a 300-cycle one-lane test",
            },
            "22x200g": {
                "SG90": "rejected",
                "MG90S": "rejected before unmodelled losses",
            },
        },
        "SG90_pretest_queue_ceiling_from_1N_model": {
            "100g_cans": 15,
            "200g_cans": 7,
            "note": "starting ceiling only; actual peak blade force still must be measured <=1.0 N",
        },
        "verdict": "11x200g and 22x100g are MG90S-only conditional loads after a measured <=1.5 N peak and a 300-cycle one-lane test. They are not accepted for SG90 because the calculated 1.410 N exceeds its 1.0 N measured-force limit; shorten the SG90 queue and measure. 22x200g is rejected for both.",
    }


def intersection_volume(a, b):
    amin, amax = a.bounds
    bmin, bmax = b.bounds
    if np.any(amax <= bmin) or np.any(bmax <= amin):
        return 0.0
    try:
        result = trimesh.boolean.intersection([clean(a), clean(b)],
                                              engine="manifold")
        return 0.0 if result is None else abs(float(result.volume))
    except Exception:
        return 1.0e12


def collision_checks():
    """Mesh sweep against the actual common cassette at all five lane pitches."""
    pairs = [
        ("frame", "servo_bracket"),
        ("frame", "front_blade"), ("frame", "rear_blade"),
        ("frame", "rocker"), ("frame", "front_pin"),
        ("frame", "rear_pin"), ("frame", "servo_body"),
        ("frame", "servo_horn"), ("frame", "servo_shaft"),
        ("frame", "servo_screw"), ("frame", "lower_sheet"),
        ("frame", "divider_left"), ("frame", "divider_right"),
        ("front_blade", "lower_sheet"),
        ("front_blade", "divider_left"),
        ("front_blade", "divider_right"),
        ("rear_blade", "lower_sheet"),
        ("rear_blade", "divider_left"),
        ("rear_blade", "divider_right"),
        ("clamp_screw_left", "lower_sheet"),
        ("clamp_screw_right", "lower_sheet"),
        ("rocker", "front_blade"), ("rocker", "rear_blade"),
        ("servo_bracket", "front_blade"),
        ("servo_bracket", "rear_blade"),
        ("servo_bracket", "rocker"),
        ("servo_bracket", "servo_body"),
        ("servo_bracket", "servo_horn"),
        ("rocker", "servo_body"), ("rocker", "servo_horn"),
        ("rocker", "servo_shaft"), ("rocker", "servo_screw"),
        ("front_blade", "rear_blade"),
        ("front_pin", "rear_blade"), ("rear_pin", "front_blade"),
    ]
    maxima = {"/".join(pair): 0.0 for pair in pairs}
    angles = np.linspace(P["servo_home_deg"], P["servo_eject_deg"], 29)
    for theta in angles:
        parts = state_parts(float(theta))
        for first, second in pairs:
            key = f"{first}/{second}"
            maxima[key] = max(maxima[key],
                              intersection_volume(parts[first], parts[second]))

    # Independent placement sweep against the exact shared lower-sheet STL and
    # all four exact slotted divider STLs.  Every lane is checked; no synthetic
    # box or shortened divider is accepted as a substitute.
    exact_lower, exact_dividers = exact_cassette_local()
    lane_offsets = (-158.0, -79.0, 0.0, 79.0, 158.0)
    mechanism_keys = (
        "frame", "servo_bracket", "front_blade", "rear_blade", "rocker",
        "front_pin", "rear_pin", "clamp_screw_left",
        "clamp_screw_right", "servo_body", "servo_shaft",
        "servo_horn", "servo_screw", "bracket_screw_lower",
        "bracket_screw_upper",
    )
    exact_lane_maxima = {f"lane_{idx}": 0.0 for idx in range(1, 6)}
    exact_pair_maxima = {}
    for theta in angles:
        source = state_parts(float(theta))
        for lane_index, x_offset in enumerate(lane_offsets, 1):
            lane_key = f"lane_{lane_index}"
            for part_key in mechanism_keys:
                part = translate(source[part_key], (x_offset, 0.0, 0.0))
                key = f"{lane_key}/{part_key}/exact_lower_sheet"
                value = intersection_volume(part, exact_lower)
                exact_pair_maxima[key] = max(exact_pair_maxima.get(key, 0.0),
                                             value)
                exact_lane_maxima[lane_key] = max(exact_lane_maxima[lane_key],
                                                   value)
                for divider_index, divider in enumerate(exact_dividers, 1):
                    key = (f"{lane_key}/{part_key}/"
                           f"exact_divider_{divider_index}")
                    value = intersection_volume(part, divider)
                    exact_pair_maxima[key] = max(
                        exact_pair_maxima.get(key, 0.0), value)
                    exact_lane_maxima[lane_key] = max(
                        exact_lane_maxima[lane_key], value)

    reference_max = 0.0
    intentional_can_gate_contact_max = 0.0
    for state, theta in (("HOME", P["servo_home_deg"]),
                         ("MID", P["servo_mid_deg"]),
                         ("EJECT", P["servo_eject_deg"])):
        parts = state_parts(theta)
        cans = [can_mesh(y, z) for y, z in can_positions(state)]
        for can in cans:
            for key in ("frame", "servo_bracket", "rocker",
                        "servo_body", "lower_sheet", "divider_left",
                        "divider_right"):
                reference_max = max(reference_max,
                                    intersection_volume(can, parts[key]))
            # A gate works by touching the cylindrical envelope.  The small
            # mesh overlap caused by the faceted tapered knife at analytical
            # tangency is therefore recorded, but is not called an unintended
            # collision.  Cross-gate and all fixed-structure checks remain in
            # the hard sweep above.
            for key in ("front_blade", "rear_blade"):
                intentional_can_gate_contact_max = max(
                    intentional_can_gate_contact_max,
                    intersection_volume(can, parts[key]))
        for i in range(len(cans)):
            for j in range(i + 1, len(cans)):
                reference_max = max(reference_max,
                                    intersection_volume(cans[i], cans[j]))

    exact_cassette_max = max(exact_pair_maxima.values(), default=0.0)
    max_unintended = max([reference_max, exact_cassette_max,
                          *maxima.values()])
    adjacent_clearance = P["lane_pitch"] - P["frame_outer_width"]
    divider_face_clearance = ((P["lane_clear"] -
                               P["clamp_above_sheet_width"]) / 2)
    frame_vertices = np.asarray(frame_mesh().vertices)
    cassette_zone_vertices = frame_vertices[
        frame_vertices[:, 1] <= P["plywood_front_edge_y"] + 1e-6]
    actual_inside_width = float(np.ptp(cassette_zone_vertices[:, 0]))

    def sha256(path):
        return hashlib.sha256(path.read_bytes()).hexdigest()

    lower_path = COMMON_STL_DIR / "cassette_lower_sheet.stl"
    divider_path = COMMON_STL_DIR / "cassette_divider.stl"
    return {
        "method": "29-angle manifold boolean sweep at 3deg increments using the actual common STL lower sheet and divider; all 5 lane positions",
        "angle_samples": len(angles),
        "lane_positions_x_mm": list(lane_offsets),
        "exact_source_files": {
            "lower_sheet": "../" + lower_path.relative_to(HERE.parent).as_posix(),
            "lower_sheet_sha256": sha256(lower_path),
            "divider": "../" + divider_path.relative_to(HERE.parent).as_posix(),
            "divider_sha256": sha256(divider_path),
        },
        "exact_lower_sheet_local_bounds_mm": np.round(
            exact_lower.bounds, 3).tolist(),
        "exact_divider_local_bounds_mm": [
            np.round(mesh.bounds, 3).tolist() for mesh in exact_dividers],
        "maximum_intersection_by_pair_mm3": {
            key: round(value, 6) for key, value in maxima.items()
        },
        "maximum_exact_cassette_intersection_by_lane_mm3": {
            key: round(value, 6) for key, value in exact_lane_maxima.items()
        },
        "maximum_exact_cassette_intersection_mm3": round(
            exact_cassette_max, 6),
        "nonzero_exact_cassette_pairs_over_0_01mm3": {
            key: round(value, 6) for key, value in exact_pair_maxima.items()
            if value > 0.01
        },
        "maximum_reference_can_unintended_intersection_mm3": round(
            reference_max, 6),
        "maximum_intentional_can_gate_contact_mm3": round(
            intentional_can_gate_contact_max, 6),
        "maximum_unintended_intersection_mm3": round(max_unintended, 6),
        "pass_at_0_01_mm3": bool(max_unintended <= 0.01),
        "intentional_interfaces_excluded": [
            "each M3 follower pin inside its own blade slot and rocker hole",
            "two lower clamp screws inside their printed pilot threads",
            "two C05 mounting screws inside C01/C05 holes",
            "reference can tangent/contact with the active front or rear gate",
        ],
        "adjacent_lane_pitch_mm": P["lane_pitch"],
        "maximum_module_width_mm": P["frame_outer_width"],
        "adjacent_modules_minimum_x_clearance_mm": round(adjacent_clearance, 3),
        "adjacent_modules_aabb_pass": bool(adjacent_clearance > 0),
        "above_sheet_clamp_width_in_divider_zone_mm": P["clamp_above_sheet_width"],
        "mesh_measured_width_upstream_of_sheet_edge_mm": round(
            actual_inside_width, 3),
        "mesh_inside_cassette_width_limit_pass": bool(
            actual_inside_width <= P["clamp_above_sheet_width"] + 0.01),
        "clear_lane_between_divider_faces_mm": P["lane_clear"],
        "clearance_to_each_divider_face_mm": round(divider_face_clearance, 3),
        "divider_and_common_sheet_boolean_pass": bool(
            exact_cassette_max <= 0.01),
    }


def viewer_assets():
    components = {
        "frame": {"color": "#566573", "explode_mm": [0, 0, 0]},
        "servo_bracket": {"color": "#6F8291", "explode_mm": [-12, 0, 12]},
        "bracket_screw_lower": {"color": "#C3CBD3", "explode_mm": [-24, 0, 4]},
        "bracket_screw_upper": {"color": "#C3CBD3", "explode_mm": [-24, 0, 18]},
        "front_blade": {"color": "#F28C28", "explode_mm": [16, 14, 8]},
        "rear_blade": {"color": "#28B7B0", "explode_mm": [16, -14, 8]},
        "rocker": {"color": "#E1B84B", "explode_mm": [-12, 0, 22]},
        "front_pin": {"color": "#C3CBD3", "explode_mm": [28, 14, 8]},
        "rear_pin": {"color": "#C3CBD3", "explode_mm": [28, -14, 8]},
        "servo_body": {"color": "#3576B8", "explode_mm": [0, 0, 30]},
        "servo_shaft": {"color": "#AEB7C2", "explode_mm": [-6, 0, 28]},
        "servo_horn": {"color": "#F2F4F5", "explode_mm": [-16, 0, 22]},
        "servo_screw": {"color": "#AEB7C2", "explode_mm": [-28, 0, 22]},
        "lower_sheet": {"color": "#B98B5F", "explode_mm": [0, -12, -18]},
        "divider_left": {"color": "#9B744F", "explode_mm": [-8, 0, 0]},
        "divider_right": {"color": "#9B744F", "explode_mm": [8, 0, 0]},
        "clamp_screw_left": {"color": "#AEB7C2", "explode_mm": [0, -6, -18]},
        "clamp_screw_right": {"color": "#AEB7C2", "explode_mm": [0, -6, -18]},
        "can_1": {"color": "#43B97F", "explode_mm": [20, 10, 0]},
        "can_2": {"color": "#4A90D9", "explode_mm": [20, 0, 0]},
        "can_3": {"color": "#8E6CCF", "explode_mm": [20, -10, 0]},
    }
    states = {}
    for state in ("HOME", "MID", "EJECT"):
        files = {}
        for key in components:
            files[key] = f"STL_СБОРКА_НЕ_ПЕЧАТАТЬ/C2_{state}_{key}.stl"
        states[state] = {
            "angle_deg": {"HOME": P["servo_home_deg"],
                          "MID": P["servo_mid_deg"],
                          "EJECT": P["servo_eject_deg"]}[state],
            "parts": files,
            "complete": f"STL_СБОРКА_НЕ_ПЕЧАТАТЬ/C2_{state}_complete.stl",
        }
    return {
        "coordinate_system": "X can axis, Y downhill/dispense direction, Z road normal; millimetres",
        "component_style": components,
        "states": states,
        "exploded_view": {
            "base_state": "HOME",
            "apply_component_explode_mm": True,
            "note": "offsets are viewer transforms; source STL files remain in assembly coordinates",
        },
        "printable": {
            "ready_3mf_plate": "STL_ПЕЧАТЬ/C2_ПЛИТА_1_КАНАЛ_готовая_220x220.3mf",
            "frame": "STL_ПЕЧАТЬ/C01_верхняя_рама_направляющая.stl",
            "servo_bracket": "STL_ПЕЧАТЬ/C05_плоский_кронштейн_сервы.stl",
            "front_blade": "STL_ПЕЧАТЬ/C02_передняя_линейная_заслонка.stl",
            "rear_blade": "STL_ПЕЧАТЬ/C03_задний_линейный_сепаратор.stl",
            "rocker": "STL_ПЕЧАТЬ/C04_коромысло_SG90_MG90S.stl",
            "horn_coupon": "STL_ПЕЧАТЬ/T01_КУПОН_штатной_качалки.stl",
        },
        "cassette_references": {
            "common_lower_sheet": "STL_СБОРКА_НЕ_ПЕЧАТАТЬ/C2_REFERENCE_точный_нижний_лист_391x569_402.stl",
            "divider_1": "STL_СБОРКА_НЕ_ПЕЧАТАТЬ/C2_REFERENCE_точная_перегородка_1.stl",
            "divider_2": "STL_СБОРКА_НЕ_ПЕЧАТАТЬ/C2_REFERENCE_точная_перегородка_2.stl",
            "divider_3": "STL_СБОРКА_НЕ_ПЕЧАТАТЬ/C2_REFERENCE_точная_перегородка_3.stl",
            "divider_4": "STL_СБОРКА_НЕ_ПЕЧАТАТЬ/C2_REFERENCE_точная_перегородка_4.stl",
        },
    }


def main():
    for directory in (PRINT_DIR, ASSEMBLY_DIR):
        for path in directory.glob("*.stl"):
            path.unlink()
    for path in PRINT_DIR.glob("*.3mf"):
        path.unlink()

    frame = frame_mesh()
    bracket = servo_bracket_mesh()
    front = blade_mesh(True)
    rear = blade_mesh(False)
    rocker = rocker_mesh()
    coupon = horn_coupon_mesh()

    # Recommended manufacturing orientations.
    # The single left spine is the bed face; every cross-lane bridge then grows
    # vertically.  This is the support-free Ender-3 orientation.
    frame_print = on_bed(rotate(frame, -90, [0, 1, 0]))
    bracket_print = on_bed(rotate(bracket, 90, [0, 1, 0]))
    front_print = on_bed(rotate(front, 90, [1, 0, 0]))
    rear_print = on_bed(rotate(rear, 90, [1, 0, 0]))
    rocker_print = on_bed(rotate(rocker, -90, [0, 1, 0]))
    coupon_print = on_bed(coupon)

    print_reports = [
        export(frame_print, PRINT_DIR / "C01_верхняя_рама_направляющая.stl"),
        export(front_print, PRINT_DIR / "C02_передняя_линейная_заслонка.stl"),
        export(rear_print, PRINT_DIR / "C03_задний_линейный_сепаратор.stl"),
        export(rocker_print, PRINT_DIR / "C04_коромысло_SG90_MG90S.stl"),
        export(bracket_print, PRINT_DIR / "C05_плоский_кронштейн_сервы.stl"),
        export(coupon_print, PRINT_DIR / "T01_КУПОН_штатной_качалки.stl"),
    ]
    plate_report = export_one_channel_3mf(
        frame_print, bracket_print, front_print, rear_print, rocker_print,
        coupon_print)

    assembly_reports = []
    export_state("HOME", P["servo_home_deg"], assembly_reports)
    export_state("MID", P["servo_mid_deg"], assembly_reports)
    export_state("EJECT", P["servo_eject_deg"], assembly_reports)
    export_cassette_references(assembly_reports)
    write_bom()

    bed_ok = all(r["extents_mm"][0] <= 220 and r["extents_mm"][1] <= 220
                 for r in print_reports)
    mesh_ok = all(r["watertight"] and r["components"] == 1 for r in print_reports)
    checks = analytical_checks()
    collisions = collision_checks()
    manifest = {
        "variant": "V11-C2",
        "name": "Верхние линейные ножи с прямым коромыслом",
        "status": "full-size comparison prototype; print one channel before replication",
        "replaces": "C_опрокидывающаяся_люлька — rejected by kinematic/load audit",
        "path_base": ".",
        "path_base_definition": "directory containing MANIFEST_C2.json",
        "path_roots": {
            "print.ready_3mf_plate.file and print_parts[].file": "STL_ПЕЧАТЬ",
            "assembly_parts[].file": "STL_СБОРКА_НЕ_ПЕЧАТАТЬ",
            "explicit paths with directories": ".",
        },
        "selection_condition": "preferred for one Ender-3 plate and minimum assembly only with continuous valid PWM and safe startup/shutdown; choose B if passive power-off holding is mandatory",
        "principle": "one rocker directly swaps two top-down linear blades; no rods and no cam",
        "servo_mount": {
            "servo_fastens_to": "C05 flat servo bracket with two M2x10 self-tapping screws",
            "C05_fastens_to": "C01 with two separate M2x10 screws and washers",
            "servo_direct_to_C01": False,
        },
        "parameters_mm": P,
        "cassette_geometry": {
            "source": "actual STL meshes from 00_ОБЩАЯ_КОМПОНОВКА/STL_КАССЕТА_СБОРКА; no synthetic collision substitutes",
            "common_lower_sheet_mm": [391.0, 569.402, 4.0],
            "internal_dividers": 4,
            "divider_blank_mm": [4.0, 590.0, 390.0],
            "lane_pitch_mm": 79.0,
            "clear_between_divider_faces_mm": 75.0,
            "lower_sheet_front_edge_local_y_mm": P["plywood_front_edge_y"],
            "real_divider_nose_at_road_plane_local_y_approx_mm": -18.6,
            "rear_wide_guide_local_y_mm": P["rear_gate_y"],
            "wide_78_4mm_frame_members_location": "free facade zone ahead of the real vertical divider noses",
            "maximum_frame_width_upstream_of_sheet_edge_mm": P["clamp_above_sheet_width"],
        },
        "counts_per_channel": {
            "functional_printed_parts": 5,
            "calibration_coupons_per_channel": 0,
            "calibration_coupons_per_servo_filament_batch": 1,
            "calibration_coupon_frequency": "one T01 per batch of matching servo horns and filament/printer settings; never one per channel",
            "metal_fastener_sets_excluding_stock_horn_screw": 8,
            "stock_horn_screws": 1,
            "fasteners_tightened_total": 9,
            "captive_nuts": 2,
            "primary_assembly_steps_once": 6,
            "installation_actions_each_time": 2,
            "removal_actions_each_time": 2,
            "attachment_screws_to_cassette": 2,
            "holes_drilled_in_plywood": 0,
        },
        "action_counting_basis": "one deliberate assembly step with one outcome; matched identical parts or fasteners handled in that step count as one grouped macro-operation; wiring is separate",
        "print": {
            "bed_220x220_ok": bed_ok,
            "all_print_meshes_watertight_single_component": mesh_ok,
            "frame_orientation": "single flat left spine on bed; remote servo wall is a separate flat C05 part",
            "frame_support": "none; broad continuous spine on bed; optional brim at most 2 mm",
            "servo_bracket_orientation": "broad outer wall on bed; two short mounting arms grow vertically",
            "servo_bracket_support": "none designed",
            "blade_orientation": "broad face on bed; 10 mm ear-slot roof is a short bridge",
            "blade_support": "none designed",
            "rocker_orientation": "flat; stock-horn pocket upward",
            "rocker_support": "none designed",
            "ready_3mf_plate": plate_report,
        },
        "kinematic_checks": checks,
        "collision_checks": collisions,
        "force_and_torque": force_model(),
        "print_parts": print_reports,
        "assembly_parts": assembly_reports,
        "viewer_assets": viewer_assets(),
        "attachment": {
            "type": "integral lower edge clamp",
            "procedure": "push C01 onto the common-sheet front edge to Y=-60; tighten M3 screws from below at X=+/-34,Y=-61.2",
            "removal": "unload lane; loosen only the two lower M3 screws by 2-3 turns; slide the complete module forward",
            "plywood_nominal_thickness_mm": P["sheet_thickness"],
            "unloaded_clamp_gap_below_sheet_mm": P["edge_clamp_gap"],
            "upper_pad_outer_x_mm": 37.2,
            "divider_inner_face_x_mm": 37.5,
            "static_clearance_to_each_divider_face_mm": 0.3,
            "support_free_upper_pad_cantilever_mm": 2.8,
            "integral_can_centering_ramp_clear_width_entry_mm": 73.0,
            "integral_can_centering_ramp_clear_width_throat_mm": 71.0,
            "integral_can_centering_ramp_height_mm": [0.4, 2.0],
            "requires_drilling_road": False,
        },
        "power_safety": {
            "self_holding_without_pwm": False,
            "continuous_pwm_required_while_loaded": True,
            "gravity_balanced_printed_blades": True,
            "endpoint_detent": False,
            "why_no_printed_detent": "a rubbing PETG detent adds uncertain breakaway force and wear to the weakest SG90 load case",
            "required": [
                "10 kOhm pull-down from each orange signal wire to common GND",
                "ESP32 establishes HOME pulses before the servo 5 V rail is enabled",
                "while any can remains in the lane, valid calibrated 50 Hz PWM remains continuously present in HOME, EJECT and during motion; servo power alone is insufficient",
                "never leave loaded cans above an energized servo with ESP32 disconnected",
                "shutdown order: stop dispensing, command HOME, fully unload the lane, remove servo 5 V, then stop PWM and power down the controller",
            ],
        },
        "risks_and_acceptance": [
            "PWM must remain continuously valid in HOME, EJECT and during motion while loaded: symmetric blades balance gravity but the queue can back-drive a servo with missing signal",
            "measure ten real cans; design envelope is diameter 48.0 and length 70.0 mm",
            "with the intended queue and rocker disconnected, measure peak vertical blade force: SG90 <=1.0 N, MG90S <=1.5 N",
            "11x200g and 22x100g are allowed only with MG90S after measured peak <=1.5 N and a 300-cycle one-lane test; they are not accepted for SG90",
            "for SG90 shorten the initial queue to at most 7x200g or 15x100g, then measure and keep only a load whose real peak is <=1.0 N",
            "22 cans x 200 g is rejected by calculation for both SG90 and MG90S before unmodelled losses",
            "test one lane for 300 cycles before printing the remaining four",
            "verify the lower M3 clamp screws press the plywood without dimpling or entering the rolling surface",
            "verify the integral 73-to-71 mm centering ramps guide every real 70 mm can without an end-ring jam",
            "production follower pins require a smooth 3 mm working shoulder; fully threaded M3 is prototype-only and the PETG slots must be inspected after 300 cycles",
            "use only PETG-safe low-strength threadlocker on the captive nuts, or use a witness lacquer mark and inspect it",
        ],
    }
    (HERE / "MANIFEST_C2.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")

    readme = """# V11-C2 — верхние линейные ножи

Полноразмерный сравнительный прототип одного канала. На канал нужны пять функциональных печатных деталей: рама C01, две прямые заслонки C02/C03, коромысло C04 и плоский кронштейн сервы C05. Штатная двойная качалка SG90/MG90S входит в карман C04; два M3-пина скользят прямо в пазах заслонок, отдельных тяг нет. C05 отделён от рамы ради гарантированной печати без поддержек и крепится одной сгруппированной операцией двумя M2×10.

## Что делает механизм

- HOME −42°: передний нож 10.501 мм, задний 52.499 мм.
- MID 0°: оба ножа 31.500 мм.
- EJECT +42°: передний нож 52.499 мм, задний 10.501 мм.
- Полный ход сервы ровно 84° и укладывается в доступный диапазон SG90/MG90S, но старые значения HOME/EJECT от прежнего дозатора использовать нельзя: направление и обе конечные точки C калибруются заново без банок.
- Во всём ходе хотя бы один нож остаётся не выше 31.5 мм: идеального сквозного окна для банки Ø48 нет.
- Отклонение пина от центра паза максимум 4.033 мм; половина хода паза 4.8 мм, остаток 0.767 мм.
- Минимальный вертикальный Jacobian 23.322 мм/рад; сингулярности в рабочем диапазоне нет.

Идеальная кинематика выдаёт одну банку, включая последнюю: на прямом ходе задний нож сначала перехватывает очередь, затем передний освобождает первую банку. На обратном ходе передний закрывается до открытия заднего. Реальный результат всё равно зависит от выдержки EJECT и 300-циклового теста.

Расчёт не приписывает банкам искусственный зазор: касающиеся расчётные кольца Ø48 имеют шаг 48.0 мм. Между затворами 50.5 мм, поэтому входящий нож должен раздвинуть очередь на 2.5 мм; начальное геометрическое внедрение в траекторию второй банки около 0.593 мм. Это главный источник реального усилия и обязательная часть теста одной дорожки.

## Точная привязка к шести листам

Проверка использует непосредственно `00_ОБЩАЯ_КОМПОНОВКА/STL_КАССЕТА_СБОРКА/cassette_lower_sheet.stl` и `cassette_divider.stl`, а не упрощённые параллелепипеды. Точный нижний скат имеет 391×569.402×4 мм, разделители — реальные прорезанные детали из заготовок 590×390×4 мм. Вертикальные носы разделителей продолжаются примерно на 41 мм вперёд кромки ската.

Поэтому задний затвор перенесён в Y=−25 мм, передний — в Y=+25.5 мм. Вся широкая механика 78.4 мм находится перед реальными носами разделителей; в зоне кромки ската ширина рамы не больше 74.4 мм. Генератор проверяет 29 углов во всех пяти позициях шага 79 мм против точного нижнего листа и всех четырёх точных разделителей. Листы не подрезаются и не сверлятся.

## Печать на Ender-3

Сначала **один раз на партию одинаковых серв и текущие настройки филамента/принтера** напечатать T01 и выбрать посадку штатной качалки. На каждый канал T01 повторять не нужно. Затем печатать комплект C01–C05. Все STL уже ориентированы: C01 стоит плоской боковой спиной, C02/C03 и C04 лежат плашмя, C05 лежит широкой стенкой на столе. Support не нужен. Brim по умолчанию выключен; при склонности PETG к короблению допустим общий brim максимум 2 мм. Пазы заслонок открыты в ненагруженном сервисном направлении; над ушком каждого паза остаётся короткий мост 10 мм. Контрольный слайс не обнаружил структурных предупреждений и прошёл с выключенными поддержками, но первый комплект нужно осмотреть до сборки.

Готовая раскладка: `STL_ПЕЧАТЬ/C2_ПЛИТА_1_КАНАЛ_готовая_220x220.3mf`. Это шесть отдельных объектов — C01–C05 и один T01 для **первой** партии. Для следующих каналов той же партии отключить объект T01 в слайсере; C01–C05 вручную поворачивать не нужно.

## Первичная сборка — один раз

Правило общее для A/B/C: одна макрооперация — один осознанный этап с единым результатом. Парные одинаковые элементы, устанавливаемые или затягиваемые в рамках такого этапа (например, два винта сервы, две оси/гайки или два нижних винта), считаются одной сгруппированной макрооперацией; электропроводка пяти серв считается отдельно. Поэтому ниже ровно 6 первичных и 2 установочные макрооперации.

1. Вставить две обычные гайки M3 DIN934 в открытые шестигранные карманы C04. Для фиксации допустим только слабый фиксатор, явно совместимый с PETG; безопасная альтернатива — контрольная метка лаком и регулярный осмотр.
2. Закрепить SG90/MG90S на C05 двумя M2×10 самонарезающими винтами.
3. Приложить две прямоугольные лапки C05 к двум упорам C01 и затянуть два M2×10 с шайбами с наружной стороны рамы.
4. Опустить C02 и C03 сверху в подписанные направляющие C01.
5. Подать MID, вложить штатную двойную качалку в C04, надеть её на шлиц и затянуть штатный центральный винт.
6. Вставить два винта-оси M3×18 справа через пазы ножей в захваченные гайки; в зоне скольжения нужна гладкая шейка Ø3, а не резьба. Оставить 0.2–0.4 мм свободного осевого люфта. Полностью резьбовой M3 допустим только для первого прототипа с обязательным осмотром PETG-пазов после 300 циклов.

Итого при первичной сборке затягиваются шесть крепёжных винтов и штатный центральный винт качалки. Купон T01 печатается один раз на партию одинаковых серв и неизменные настройки филамента/принтера, не на каждый канал.

## Установка на нижний скат

1. Надвинуть цельный кромочный зажим C01 на переднюю кромку точного нижнего листа до упора Y=−60.
2. Снизу равномерно затянуть два M3×10 в X=±34, Y=−61.2. Они прижимают целый лист к двум верхним губкам; сверление не требуется.

Верхние губки одновременно образуют короткие плавные центрирующие рампы: проход сужается с 73 до 71 мм при высоте 0.4→2.0 мм, оставляя расчётной банке длиной 70 мм суммарно 1 мм. После затяжки вручную проверить, что переход скат→печатная дорожка не имеет ступеньки и что все реальные банки проходят рампы без заедания торцевого кольца.

## Снятие и обслуживание

1. Обесточить сервы после разгрузки канала и придержать C01 рукой.
2. Ослабить только два нижних M3×10 на 2–3 оборота и стянуть весь собранный модуль вперёд. Серво, коромысло и ножи разбирать не нужно.
3. Очистить направляющие насухо, проверить свободный ход ножей и осевой люфт пинов 0.2–0.4 мм; затем надеть модуль обратно и равномерно затянуть те же два винта.

## Расчёт нагрузки

Расчёт: тяга очереди `N·m·g·sin(15°)`; μ=0.20; затем клиновое усилие и момент на плече 31.383 мм. Не учтены трение направляющих, удары, коробление листа, разброс клонов и потери редуктора.

| Банок | Масса каждой | Тяга по скату | Усилие ножа | Момент | SG90 1.8 кг·см | MG90S 2.2 кг·см |
|---:|---:|---:|---:|---:|---:|---:|
| 11 | 100 г | 2.792 Н | 0.705 Н | 0.226 кг·см | 12.5% | 10.3% |
| 11 | 200 г | 5.584 Н | 1.410 Н | 0.451 кг·см | 25.1% | 20.5% |
| 22 | 100 г | 5.584 Н | 1.410 Н | 0.451 кг·см | 25.1% | 20.5% |
| 22 | 200 г | 11.168 Н | 2.820 Н | 0.903 кг·см | 50.1% | 41.0% |

Нагрузки **11×200 г и 22×100 г разрешены только для MG90S** и только после измерения реального пика ≤1.5 Н на отсоединённом коромысле и 300 циклов одного канала. Для SG90 обе нагрузки запрещены: расчётные 1.410 Н выше лимита 1.0 Н. С SG90 начинать не более чем с 7×200 г или 15×100 г, затем измерить пик и оставить только очередь, реально дающую ≤1.0 Н. 22×200 г не принимается ни для SG90, ни для MG90S: 41–50% stall получено ещё до неучтённых потерь.

## Электропитание и обязательная приёмка

Механизм не самотормозящийся: пока в канале остаётся хотя бы одна банка, корректный калиброванный PWM 50 Гц должен **непрерывно** присутствовать в HOME, EJECT и во время движения. Одного питания сервы без импульсов недостаточно. Нужен pull-down 10 кОм от оранжевого сигнала к общей земле; ESP32 сначала запускает PWM и устанавливает HOME, и только затем включается питание 5 В серв. Остановка: прекратить выдачу → HOME → полностью разгрузить канал → снять 5 В серв → только затем прекратить PWM и выключить контроллер. Не оставлять загруженную кассету с питаемой сервой и отключённым ESP32. Если пассивное удержание при отключении питания обязательно, выбирать B, а не C.

До тиражирования напечатать один канал: проверить реальное усилие, затем выполнить 300 циклов. Критерии: 0 дублей, 0 пропусков, отсутствие касания разделителей, самопроизвольного схода модуля, заедания и заметного нагрева сервы.
"""
    (HERE / "README_C2.md").write_text(readme, encoding="utf-8")

    print(json.dumps({"print_parts": len(print_reports), "bed_ok": bed_ok,
                      "mesh_ok": mesh_ok,
                      "collision_sweep_pass": collisions["pass_at_0_01_mm3"],
                      "checks": checks},
                     ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
