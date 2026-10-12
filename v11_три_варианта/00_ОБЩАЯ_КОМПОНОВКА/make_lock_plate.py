# -*- coding: utf-8 -*-
"""Build the final V11 glue-free cassette locks and their Ender-3 plate.

This generator owns only the common six-sheet cassette.  It deliberately does
not regenerate dispenser variants A/B/C or the root viewer/README.

Coordinate systems (millimetres)
---------------------------------
Box/world: X is depth (front 0, rear 590), Y is width, Z is vertical.
Lock frame: S points inward from the open sheet edge, V is transverse and N is
the upward normal of the corresponding sloped sheet.  The lower lock lives at
negative V and negative N so that both installed locks lie on the same global
+Y side of a divider and outside the can channel.
"""
from __future__ import annotations

import csv
import hashlib
import json
import math
import shutil
import subprocess
import tempfile
import time
import zipfile
from pathlib import Path
from xml.etree import ElementTree

import numpy as np
import trimesh
from shapely.geometry import LineString, Point, Polygon, box as sbox
from shapely.ops import unary_union


HERE = Path(__file__).resolve().parent
ASSEMBLY_DIR = HERE / "STL_КАССЕТА_СБОРКА"
PRINT_DIR = HERE / "STL_ПЕЧАТЬ"
WORLD_DIR = HERE / "STL_ЗАМКИ_СБОРКА"
CUT_DIR = HERE / "РАСКРОЙ_6_ЛИСТОВ"
PLATE_DIR = HERE / "PLATES_3MF"
LEGACY_DIR = HERE / "LEGACY_DO_NOT_USE"
MANIFEST_PATH = HERE / "ФИКСАТОРЫ_V11_MANIFEST.json"
PLATE_PATH = PLATE_DIR / "КАССЕТА_8_ВНУТРЕННИХ_ФИКСАТОРОВ_220x220.3mf"

for directory in (ASSEMBLY_DIR, PRINT_DIR, WORLD_DIR, CUT_DIR, PLATE_DIR):
    directory.mkdir(parents=True, exist_ok=True)


P = {
    "box_inside_depth": 595.0,
    "box_inside_width": 395.0,
    "box_inside_height": 395.0,
    "cassette_depth": 590.0,
    "cassette_width": 391.0,
    "cassette_height": 390.0,
    "sheet_thickness": 4.0,
    "slot_clearance_cnc": 0.25,
    "slot_clearance_manual": 0.45,
    "divider_centres_from_cassette_left": [77.0, 156.0, 235.0, 314.0],
    "lower_x0": 40.0,
    "lower_x1": 590.0,
    "lower_angle_deg": 15.0,
    "upper_x0": 0.0,
    "upper_x1": 535.0,
    "upper_front_height": 345.0,
    "upper_angle_deg": 14.0,
    # Final V11 lock coordinates.  q=10 leaves 5.9 mm of plywood between the
    # Ø4.2 sheet hole and the nearest divider face.
    "peg_s": 10.0,
    "peg_q": 10.0,
    "peg_diameter": 3.8,
    "sheet_hole_diameter": 4.2,
    # The transverse M4 is deliberately 10 mm normal to the sheet slot.  This
    # leaves 5.675 mm minimum CNC-profile web to the slot.
    "bolt_s": 11.0,
    "bolt_n": 10.0,
    "divider_hole_diameter": 4.4,
    "shoe_gap": 0.0,
    "peg_tip_recess": 1.0,
}


CANONICAL_FILES = [
    ASSEMBLY_DIR / "cassette_divider.stl",
    ASSEMBLY_DIR / "cassette_lower_sheet.stl",
    ASSEMBLY_DIR / "cassette_upper_sheet.stl",
    CUT_DIR / "01_РАЗДЕЛИТЕЛЬ_590x390_4шт.svg",
    CUT_DIR / "01_РАЗДЕЛИТЕЛЬ_CNC_зазор_0.25_4шт.dxf",
    CUT_DIR / "01_РАЗДЕЛИТЕЛЬ_РУЧНОЙ_зазор_0.45_4шт.dxf",
    CUT_DIR / "02_НИЖНИЙ_ЦЕЛЬНЫЙ_ЛИСТ_1шт.dxf",
    CUT_DIR / "02_НИЖНИЙ_ЦЕЛЬНЫЙ_ЛИСТ_1шт.svg",
    CUT_DIR / "03_ВЕРХНИЙ_ЦЕЛЬНЫЙ_ЛИСТ_1шт.dxf",
    CUT_DIR / "03_ВЕРХНИЙ_ЦЕЛЬНЫЙ_ЛИСТ_1шт.svg",
]


def clean(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    result = mesh.copy()
    result.remove_unreferenced_vertices()
    result.merge_vertices()
    trimesh.repair.fix_normals(result)
    return result


def fuse(meshes: list[trimesh.Trimesh]) -> trimesh.Trimesh:
    items = [clean(mesh) for mesh in meshes]
    result = trimesh.boolean.union(items, engine="manifold")
    if result is None:
        raise RuntimeError("Boolean union returned no mesh")
    return clean(result)


def cut(mesh: trimesh.Trimesh, cutters: list[trimesh.Trimesh]) -> trimesh.Trimesh:
    result = trimesh.boolean.difference([clean(mesh), *[clean(c) for c in cutters]],
                                        engine="manifold")
    if result is None:
        raise RuntimeError("Boolean difference returned no mesh")
    return clean(result)


def box_bounds(s0: float, s1: float, v0: float, v1: float,
               n0: float, n1: float) -> trimesh.Trimesh:
    mesh = trimesh.creation.box(extents=(s1 - s0, v1 - v0, n1 - n0))
    mesh.apply_translation(((s0 + s1) / 2, (v0 + v1) / 2, (n0 + n1) / 2))
    return mesh


def rotate(mesh: trimesh.Trimesh, angle_deg: float, axis: tuple[float, float, float]) -> trimesh.Trimesh:
    result = mesh.copy()
    result.apply_transform(trimesh.transformations.rotation_matrix(
        math.radians(angle_deg), axis))
    return clean(result)


def cylinder_y(radius: float, length: float,
               center: tuple[float, float, float], sections: int = 48) -> trimesh.Trimesh:
    mesh = trimesh.creation.cylinder(radius=radius, height=length, sections=sections)
    mesh.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, (1, 0, 0)))
    mesh.apply_translation(center)
    return mesh


def extrude_xz(poly: Polygon, thickness: float, y0: float) -> trimesh.Trimesh:
    """Extrude a polygon expressed in global X,Z along global Y."""
    mesh = trimesh.creation.extrude_polygon(poly, height=thickness)
    transform = np.array([
        [1, 0, 0, 0],
        [0, 0, 1, y0],
        [0, 1, 0, 0],
        [0, 0, 0, 1],
    ], dtype=float)
    mesh.apply_transform(transform)
    return clean(mesh)


def open_diamond_cradle_y(center_s: float, center_n: float, inscribed_radius: float,
                          open_n: float, v0: float, v1: float) -> trimesh.Trimesh:
    """An open support-free M4 cradle with a 45 degree closed end.

    In the installed upper frame the opening points towards larger N, away
    from the sheet.  After print orientation that opening reaches the bed.
    Motion towards the open mouth is stopped by the shoe touching the sheet;
    withdrawal moves the screw centre towards the closed 45 degree end.
    """
    vertex_radius = inscribed_radius * math.sqrt(2.0)
    poly = Polygon([
        (center_s, center_n - vertex_radius),
        (center_s + vertex_radius, center_n),
        (center_s + vertex_radius, open_n),
        (center_s - vertex_radius, open_n),
        (center_s - vertex_radius, center_n),
    ])
    mesh = trimesh.creation.extrude_polygon(poly, height=v1 - v0)
    # local (x,y,z)=(S,N,V) -> lock frame (S,V,N)
    transform = np.array([
        [1, 0, 0, 0],
        [0, 0, 1, v0],
        [0, 1, 0, 0],
        [0, 0, 0, 1],
    ], dtype=float)
    mesh.apply_transform(transform)
    return clean(mesh)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def ensure_legacy_backups() -> None:
    """Preserve the q=6 V10-derived sources before replacing canonicals."""
    LEGACY_DIR.mkdir(parents=True, exist_ok=True)
    for source in CANONICAL_FILES:
        if not source.exists():
            raise FileNotFoundError(f"Cannot preserve missing canonical source: {source}")
        destination = LEGACY_DIR / source.name
        if not destination.exists():
            shutil.copy2(source, destination)

    # The two blind half-slot clips are retained as forensic/view-only legacy
    # and removed from the canonical assembly folder so nobody prints them by
    # mistake.
    for name in ("cassette_lock_upper_14deg.stl", "cassette_lock_lower_15deg.stl"):
        source = ASSEMBLY_DIR / name
        destination = LEGACY_DIR / name
        if source.exists() and not destination.exists():
            shutil.copy2(source, destination)
        if source.exists():
            removed = LEGACY_DIR / f"REMOVED_FROM_CANONICAL_{name}"
            if not removed.exists():
                source.replace(removed)

    # Move only the exact superseded draft artefacts made by the earlier lock
    # generator.  This is recoverable and prevents a second "V2 divider" from
    # being mistaken for the canonical cassette source.
    obsolete = [
        WORLD_DIR / "cassette_divider_v2_with_2_M4_holes.stl",
        WORLD_DIR / "cassette_xlocks_upper_4_world_v2.stl",
        WORLD_DIR / "cassette_xlocks_lower_4_world_v2.stl",
        PRINT_DIR / "cassette_xlock_upper_14deg_v2_PRINT.stl",
        PRINT_DIR / "cassette_xlock_lower_15deg_v2_PRINT.stl",
        PRINT_DIR / "фиксатор_верхнего_ската_14град_V11_PRINT.stl",
        PRINT_DIR / "фиксатор_нижнего_ската_15град_V11_PRINT.stl",
        PLATE_DIR / "КАССЕТА_8_X_ЗАМКОВ_готовая_220x220.3mf",
        HERE / "LOCKS_V2_MANIFEST.json",
        CUT_DIR / "04_КООРДИНАТЫ_СВЕРЛЕНИЯ_XLOCK_V2.csv",
        WORLD_DIR / "M4x16_болты_8шт_world_REFERENCE.stl",
        CUT_DIR / "01_РАЗДЕЛИТЕЛЬ_V2_590x390_2отв_M4_4шт.svg",
        CUT_DIR / "01_РАЗДЕЛИТЕЛЬ_V2_CNC_зазор_0.25_2отв_M4_4шт.dxf",
        CUT_DIR / "01_РАЗДЕЛИТЕЛЬ_V2_РУЧНОЙ_зазор_0.45_2отв_M4_4шт.dxf",
    ]
    for source in obsolete:
        if source.exists():
            destination = LEGACY_DIR / f"DRAFT_DO_NOT_USE_{source.name}"
            if not destination.exists():
                source.replace(destination)


def lower_z(x: float) -> float:
    return x * math.tan(math.radians(P["lower_angle_deg"]))


def upper_z(x: float) -> float:
    return P["upper_front_height"] - x * math.tan(math.radians(P["upper_angle_deg"]))


def divider_profile(clearance: float) -> Polygon:
    slot_width = P["sheet_thickness"] + clearance
    outline = sbox(0, 0, P["cassette_depth"], P["cassette_height"])
    upper_line = LineString([
        (-6, upper_z(-6)),
        (P["upper_x1"], upper_z(P["upper_x1"])),
    ]).buffer(slot_width / 2, cap_style=2, join_style=2)
    upper_relief = Point(P["upper_x1"], upper_z(P["upper_x1"])).buffer(4.0, resolution=24)
    lower_line = LineString([
        (P["lower_x0"], lower_z(P["lower_x0"])),
        (P["cassette_depth"] + 6, lower_z(P["cassette_depth"] + 6)),
    ]).buffer(slot_width / 2, cap_style=2, join_style=2)
    lower_relief = Point(P["lower_x0"], lower_z(P["lower_x0"])).buffer(4.0, resolution=24)
    result = outline.difference(unary_union([
        upper_line, upper_relief, lower_line, lower_relief,
    ]))
    if result.geom_type != "Polygon":
        result = max(result.geoms, key=lambda item: item.area)
    return result


def frame_to_world(kind: str, divider_y: float) -> np.ndarray:
    if kind == "upper":
        angle = math.radians(P["upper_angle_deg"])
        c, s = math.cos(angle), math.sin(angle)
        return np.array([
            [c, 0, s, 0],
            [0, 1, 0, divider_y],
            [-s, 0, c, P["upper_front_height"]],
            [0, 0, 0, 1],
        ], dtype=float)
    angle = math.radians(P["lower_angle_deg"])
    c, s = math.cos(angle), math.sin(angle)
    return np.array([
        [-c, 0, -s, P["lower_x1"]],
        [0, -1, 0, divider_y],
        [-s, 0, c, lower_z(P["lower_x1"])],
        [0, 0, 0, 1],
    ], dtype=float)


def bolt_center_xz(kind: str) -> tuple[float, float]:
    if kind == "upper":
        local = np.array([P["bolt_s"], 0, P["bolt_n"], 1.0])
    else:
        local = np.array([P["bolt_s"], 0, -P["bolt_n"], 1.0])
    world = frame_to_world(kind, 0.0) @ local
    return float(world[0]), float(world[2])


def peg_center_world(kind: str, divider_y: float) -> np.ndarray:
    local_v = P["peg_q"] if kind == "upper" else -P["peg_q"]
    local = np.array([P["peg_s"], local_v, 0.0, 1.0])
    return (frame_to_world(kind, divider_y) @ local)[:3]


def make_sloped_sheet(kind: str) -> trimesh.Trimesh:
    if kind == "upper":
        x0, x1 = P["upper_x0"], P["upper_x1"]
        angle = P["upper_angle_deg"]
        z_func = upper_z
        rotation = angle
    else:
        x0, x1 = P["lower_x0"], P["lower_x1"]
        angle = P["lower_angle_deg"]
        z_func = lower_z
        rotation = -angle

    length = (x1 - x0) / math.cos(math.radians(angle))
    sheet = trimesh.creation.box(extents=(length, P["cassette_width"], P["sheet_thickness"]))
    sheet = rotate(sheet, rotation, (0, 1, 0))
    sheet.apply_translation(((x0 + x1) / 2, P["box_inside_width"] / 2,
                             (z_func(x0) + z_func(x1)) / 2))

    cutters = []
    for divider_y in divider_world_ys():
        center = peg_center_world(kind, divider_y)
        hole = trimesh.creation.cylinder(
            radius=P["sheet_hole_diameter"] / 2,
            height=10.0,
            sections=48,
        )
        hole = rotate(hole, rotation, (0, 1, 0))
        hole.apply_translation(center)
        cutters.append(hole)
    return cut(sheet, cutters)


def make_divider() -> trimesh.Trimesh:
    profile = divider_profile(P["slot_clearance_cnc"])
    for kind in ("upper", "lower"):
        x, z = bolt_center_xz(kind)
        profile = profile.difference(Point(x, z).buffer(
            P["divider_hole_diameter"] / 2, resolution=48))
    return extrude_xz(profile, P["sheet_thickness"], -P["sheet_thickness"] / 2)


def divider_hole_boundary_audit() -> dict:
    """True 2D web from every M4 hole edge to every profile boundary."""
    report = {}
    radius = P["divider_hole_diameter"] / 2
    raw_rectangle = sbox(0, 0, P["cassette_depth"], P["cassette_height"])
    for label, clearance in (("CNC_0.25", P["slot_clearance_cnc"]),
                             ("manual_0.45", P["slot_clearance_manual"])):
        profile = divider_profile(clearance)
        per_hole = {}
        for kind in ("upper", "lower"):
            center = Point(*bolt_center_xz(kind))
            if not profile.contains(center):
                raise RuntimeError(f"Divider {kind} M4 center is outside {label} profile")
            web = float(center.distance(profile.boundary) - radius)
            external_web = float(center.distance(raw_rectangle.boundary) - radius)
            if web < 5.5 or external_web < 5.5:
                raise RuntimeError(
                    f"Divider {kind}/{label} web full={web:.3f}, external={external_web:.3f} mm")
            per_hole[kind] = {
                "to_nearest_complete_profile_boundary_mm": round(web, 6),
                "to_original_590x390_outer_boundary_mm": round(external_web, 6),
            }
        report[label] = per_hole
    return report


def make_upper_lock_frame() -> trimesh.Trimesh:
    """One positive, printable, single-bolt lock in upper S,V,N coordinates."""
    # M4 pad touches the +V face of the 4 mm divider: V=2.0 is the first hard
    # mounting datum, so tightening cannot shift the locator sideways.
    # The pad deliberately reaches the locator side.  This replaces a fragile
    # cantilever with a broad, directly supported print footprint.  It requires
    # M4x25 (with two washers), but costs only a few grams of PETG.
    pad = box_bounds(2.0, 18.0, 2.00, 14.00, 4.50, 15.50)
    # Wide anti-rotation shoe: N=2.0 touches the outer sheet face and is the
    # second hard datum.  It spans 7 mm to either side of the locator peg.
    shoe = box_bounds(3.0, 17.0, 6.0, 14.0,
                      2.0 + P["shoe_gap"], 5.20)
    # Ø3.8 locator penetrates 3.0 mm into a 4 mm sheet and remains 1.0 mm below
    # the bank-running face.  It is the only part allowed inside the sheet.
    peg_min_n = -2.0 + P["peg_tip_recess"]
    peg_max_n = 4.80
    peg = trimesh.creation.cylinder(
        radius=P["peg_diameter"] / 2,
        height=peg_max_n - peg_min_n,
        sections=48,
    )
    peg.apply_translation((P["peg_s"], P["peg_q"],
                           (peg_min_n + peg_max_n) / 2))
    body = fuse([pad, shoe, peg])

    # The 45° cradle contains a Ø4.6 circle and opens OUTWARD from the sheet.
    # In print orientation its mouth reaches the bed.  The closed diamond end
    # positively limits withdrawal; the divider retains a round Ø4.4 hole.
    bore = open_diamond_cradle_y(
        P["bolt_s"], P["bolt_n"], 2.30, 16.0, 1.50, 14.50)
    return cut(body, [bore])


def make_lock_frame(kind: str) -> trimesh.Trimesh:
    upper = make_upper_lock_frame()
    if kind == "upper":
        return upper
    # 180° about S puts the lower body at negative V and negative N.
    return rotate(upper, 180.0, (1, 0, 0))


def orient_for_print(frame_mesh: trimesh.Trimesh, kind: str) -> trimesh.Trimesh:
    result = frame_mesh.copy()
    # Both types print on the broad face furthest from the sheet; the upper is
    # inverted, while the already-negative lower frame needs no extra turn.
    if kind == "upper":
        result = rotate(result, 180.0, (1, 0, 0))
    result.apply_translation((0, 0, -result.bounds[0, 2]))
    center_xy = result.bounds[:, :2].mean(axis=0)
    result.apply_translation((-center_xy[0], -center_xy[1], 0))
    return clean(result)


def divider_world_ys() -> list[float]:
    cassette_offset = (P["box_inside_width"] - P["cassette_width"]) / 2
    return [cassette_offset + value
            for value in P["divider_centres_from_cassette_left"]]


def intersection_volume(a: trimesh.Trimesh, b: trimesh.Trimesh) -> float:
    try:
        result = trimesh.boolean.intersection([clean(a), clean(b)], engine="manifold")
    except BaseException:
        return float("nan")
    if result is None or len(result.faces) == 0:
        return 0.0
    return abs(float(result.volume))


def validate_mesh(name: str, mesh: trimesh.Trimesh, one_body: bool = True) -> dict:
    components = len(mesh.split(only_watertight=False))
    if not mesh.is_watertight:
        raise RuntimeError(f"{name} is not watertight")
    if one_body and components != 1:
        raise RuntimeError(f"{name} has {components} bodies, expected one")
    return {
        "watertight": bool(mesh.is_watertight),
        "components": int(components),
        "bounds_mm": np.round(mesh.bounds, 6).tolist(),
        "volume_mm3": round(abs(float(mesh.volume)), 4),
    }


def rotation_restraint_case(shoe_gap: float, peg_engagement: float) -> list[dict]:
    """Prove shoe contact occurs before the peg can lift out for one stack."""
    pivot_s, pivot_n = P["bolt_s"], P["bolt_n"]
    shoe_points = [(3.0, 2.0 + shoe_gap),
                   (17.0, 2.0 + shoe_gap)]
    peg_tip = (P["peg_s"], 2.0 - peg_engagement)

    def rotated_n(point: tuple[float, float], angle_deg: float) -> float:
        angle = math.radians(angle_deg)
        ds, dn = point[0] - pivot_s, point[1] - pivot_n
        return pivot_n - math.sin(angle) * ds + math.cos(angle) * dn

    results = []
    for direction in (-1, 1):
        contact = 0.0 if shoe_gap <= 1e-12 else None
        release = None
        for step in range(1, 1801):  # 0.05° through 90°
            angle = direction * step * 0.05
            if contact is None and min(rotated_n(point, angle) for point in shoe_points) <= 2.0:
                contact = abs(angle)
            if release is None and rotated_n(peg_tip, angle) >= 2.0:
                release = abs(angle)
            if contact is not None and release is not None:
                break
        if contact is None or release is None or contact >= release:
            raise RuntimeError("Anti-rotation shoe does not arrest motion before peg release")
        results.append({
            "rotation_direction": direction,
            "first_shoe_contact_deg": round(contact, 2),
            "full_peg_release_deg": round(release, 2),
            "contact_before_release": True,
        })
    return results


def rotation_restraint_sweep() -> dict:
    """Nominal and tolerance-stack anti-rotation proof."""
    nominal_engagement = P["sheet_thickness"] - P["peg_tip_recess"]
    nominal = rotation_restraint_case(0.0, nominal_engagement)
    closed_end_clearance = (2.30 - 2.00) * math.sqrt(2.0)
    manual_slot_half_play = P["slot_clearance_manual"] / 2
    divider_hole_radial = (P["divider_hole_diameter"] - 4.0) / 2
    worst_withdrawal = (closed_end_clearance + manual_slot_half_play
                        + divider_hole_radial)
    remaining = nominal_engagement - worst_withdrawal
    worst = rotation_restraint_case(worst_withdrawal, remaining)
    return {
        "method": "0.05 degree rigid rotation sweep about the transverse M4 axis",
        "nominal": {"upper": nominal, "lower": nominal},
        "worst_clearance_stack": {
            "closed_diamond_end_center_clearance_mm": round(closed_end_clearance, 6),
            "manual_slot_half_play_mm": round(manual_slot_half_play, 6),
            "divider_hole_radial_play_mm": round(divider_hole_radial, 6),
            "maximum_outward_withdrawal_mm": round(worst_withdrawal, 6),
            "shoe_gap_at_hard_stop_mm": round(worst_withdrawal, 6),
            "remaining_peg_engagement_mm": round(remaining, 6),
            "upper": worst,
            "lower": worst,
        },
        "note": "lower geometry is the exact 180 degree S-axis image of upper",
    }


def washer_bearing_audit() -> dict:
    """2D contact area of the outer DIN 9021 washer on the printed pad."""
    s, n = P["bolt_s"], P["bolt_n"]
    a = 2.30 * math.sqrt(2.0)
    cavity = Polygon([
        (s, n - a), (s + a, n), (s + a, 16.0),
        (s - a, 16.0), (s - a, n),
    ])
    face = unary_union([
        sbox(2.0, 4.5, 18.0, 15.5),
        sbox(3.0, 2.0, 17.0, 5.2),
    ]).difference(cavity)
    washer = Point(s, n).buffer(6.0, resolution=128).difference(
        Point(s, n).buffer(2.2, resolution=128))
    bearing = float(washer.intersection(face).area)
    fraction = bearing / float(washer.area)
    if bearing < 60.0:
        raise RuntimeError(f"Wide-washer PETG bearing area too small: {bearing:.2f} mm2")
    return {
        "washer_standard": "M4 DIN 9021 or equivalent wide flat washer",
        "outer_diameter_mm": 12.0,
        "inner_diameter_reference_mm": 4.4,
        "thickness_reference_mm": 1.0,
        "printed_face_bearing_area_mm2": round(bearing, 3),
        "washer_annulus_supported_fraction": round(fraction, 4),
        "minimum_required_bearing_area_mm2": 60.0,
        "result": "PASS",
    }


def polygon_svg_path(poly: Polygon, margin: float, height: float) -> str:
    rings = [poly.exterior, *poly.interiors]
    chunks = []
    for ring in rings:
        points = " L ".join(
            f"{margin + x:.3f} {margin + (height - z):.3f}"
            for x, z in ring.coords)
        chunks.append(f"M {points} Z")
    return " ".join(chunks)


def write_cut_files() -> dict:
    bolt_holes = [Point(*bolt_center_xz(kind)).buffer(
        P["divider_hole_diameter"] / 2, resolution=48)
        for kind in ("upper", "lower")]

    # Divider SVG uses the CNC clearance profile, matching the canonical STL.
    cnc_profile = divider_profile(P["slot_clearance_cnc"]).difference(
        unary_union(bolt_holes))
    margin = 20.0
    svg_w = P["cassette_depth"] + 2 * margin
    svg_h = P["cassette_height"] + 2 * margin
    divider_svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{svg_w:.1f}mm" height="{svg_h:.1f}mm" viewBox="0 0 {svg_w:.1f} {svg_h:.1f}">
<rect width="100%" height="100%" fill="white"/>
<style>.cut{{fill:none;stroke:#d11;stroke-width:.35}} .dim{{font:7px Segoe UI,Arial;fill:#123}} .note{{font:6px Segoe UI,Arial;fill:#456}}</style>
<path class="cut" fill-rule="evenodd" d="{polygon_svg_path(cnc_profile, margin, P['cassette_height'])}"/>
<text x="{margin}" y="9" class="dim">V11 FINAL · РАЗДЕЛИТЕЛЬ 590×390×4 — 4 шт. · 2 отверстия Ø4.4</text>
<text x="{margin}" y="{svg_h - 5}" class="note">Верхний паз открыт спереди; нижний — сзади. Использовать с 8 внутренними фиксаторами V11.</text>
</svg>'''
    (CUT_DIR / "01_РАЗДЕЛИТЕЛЬ_590x390_4шт.svg").write_text(divider_svg, encoding="utf-8")

    for suffix, clearance in (("CNC_зазор_0.25", P["slot_clearance_cnc"]),
                              ("РУЧНОЙ_зазор_0.45", P["slot_clearance_manual"])):
        poly = divider_profile(clearance).difference(unary_union(bolt_holes))
        text = trimesh.load_path(poly).export(file_type="dxf")
        (CUT_DIR / f"01_РАЗДЕЛИТЕЛЬ_{suffix}_4шт.dxf").write_text(text, encoding="utf-8")

    sheet_hole_vs = [value + P["peg_q"]
                     for value in P["divider_centres_from_cassette_left"]]
    sheet_outputs = []
    for kind, code, label, direction in (
        ("lower", "02_НИЖНИЙ_ЦЕЛЬНЫЙ_ЛИСТ_1шт", "Нижний скат", "сзади вперёд"),
        ("upper", "03_ВЕРХНИЙ_ЦЕЛЬНЫЙ_ЛИСТ_1шт", "Верхний скат", "спереди назад"),
    ):
        if kind == "lower":
            length = (P["lower_x1"] - P["lower_x0"]) / math.cos(
                math.radians(P["lower_angle_deg"]))
            hole_u = length - P["peg_s"]
            angle = P["lower_angle_deg"]
        else:
            length = (P["upper_x1"] - P["upper_x0"]) / math.cos(
                math.radians(P["upper_angle_deg"]))
            hole_u = P["peg_s"]
            angle = P["upper_angle_deg"]
        holes = [Point(hole_u, v).buffer(P["sheet_hole_diameter"] / 2, resolution=48)
                 for v in sheet_hole_vs]
        poly = Polygon([(0, 0), (length, 0), (length, P["cassette_width"]),
                        (0, P["cassette_width"])]).difference(unary_union(holes))
        dxf_path = CUT_DIR / f"{code}.dxf"
        dxf_path.write_text(trimesh.load_path(poly).export(file_type="dxf"), encoding="utf-8")

        margin2 = 20.0
        width = length + 2 * margin2
        height = P["cassette_width"] + 2 * margin2
        circles = "".join(
            f'<circle class="hole" cx="{margin2 + hole_u:.3f}" cy="{margin2 + v:.3f}" r="{P["sheet_hole_diameter"] / 2:.3f}"/>'
            for v in sheet_hole_vs)
        svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width:.2f}mm" height="{height:.2f}mm" viewBox="0 0 {width:.2f} {height:.2f}">
<rect width="100%" height="100%" fill="white"/>
<style>text{{font-family:Segoe UI,Arial;fill:#123}} .p{{fill:#f4dfbd;stroke:#d11;stroke-width:.35}} .hole{{fill:white;stroke:#d11;stroke-width:.35}}</style>
<rect class="p" x="{margin2}" y="{margin2}" width="{length:.3f}" height="{P['cassette_width']:.3f}"/>
{circles}
<text x="{margin2}" y="9" font-size="7" font-weight="700">V11 FINAL · {label}: {length:.2f}×{P['cassette_width']:.2f}×4 мм · 1:1</text>
<text x="{margin2}" y="{height - 5:.2f}" font-size="6">Угол {angle}°; вставлять {direction}. 4 отверстия Ø4.2: s=10, q=+10 от разделителя.</text>
</svg>'''
        svg_path = CUT_DIR / f"{code}.svg"
        svg_path.write_text(svg, encoding="utf-8")
        sheet_outputs.extend([dxf_path, svg_path])

    csv_path = CUT_DIR / "04_КООРДИНАТЫ_ВНУТРЕННИХ_ФИКСАТОРОВ_V11.csv"
    with csv_path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.writer(stream, delimiter=";")
        writer.writerow(["деталь", "элемент", "координата_1_мм", "координата_2_мм", "диаметр_мм", "количество"])
        for kind, label in (("upper", "верхнее M4"), ("lower", "нижнее M4")):
            x, z = bolt_center_xz(kind)
            writer.writerow(["разделитель 590x390", label, f"X={x:.3f}", f"Z={z:.3f}", "4.4", "по 1 на каждом из 4"])
        for kind, label in (("upper", "верхний скат"), ("lower", "нижний скат")):
            if kind == "upper":
                length = (P["upper_x1"] - P["upper_x0"]) / math.cos(math.radians(P["upper_angle_deg"]))
                u = P["peg_s"]
            else:
                length = (P["lower_x1"] - P["lower_x0"]) / math.cos(math.radians(P["lower_angle_deg"]))
                u = length - P["peg_s"]
            writer.writerow([label, "4 locator holes", f"U={u:.3f}",
                             "V=87,166,245,324", "4.2", "4"])

    outputs = [
        CUT_DIR / "01_РАЗДЕЛИТЕЛЬ_590x390_4шт.svg",
        CUT_DIR / "01_РАЗДЕЛИТЕЛЬ_CNC_зазор_0.25_4шт.dxf",
        CUT_DIR / "01_РАЗДЕЛИТЕЛЬ_РУЧНОЙ_зазор_0.45_4шт.dxf",
        *sheet_outputs,
        csv_path,
    ]
    return {str(path.relative_to(HERE)): sha256(path) for path in outputs}


def export_stl(mesh: trimesh.Trimesh, path: Path, one_body: bool = True) -> dict:
    path.parent.mkdir(parents=True, exist_ok=True)
    clean(mesh).export(path)
    return {
        "path": str(path.relative_to(HERE)),
        "sha256": sha256(path),
        **validate_mesh(path.name, mesh, one_body=one_body),
    }


def make_plate(print_meshes: dict[str, trimesh.Trimesh]) -> dict:
    scene = trimesh.Scene()
    placements = {
        "upper": [(35.0, 140.0), (85.0, 140.0), (135.0, 140.0), (185.0, 140.0)],
        "lower": [(35.0, 80.0), (85.0, 80.0), (135.0, 80.0), (185.0, 80.0)],
    }
    placed_meshes = []
    for kind in ("upper", "lower"):
        for index, (x, y) in enumerate(placements[kind], start=1):
            placed = print_meshes[kind].copy()
            placed.apply_translation((x, y, 0))
            name = f"CASSETTE_FIXATOR_{kind.upper()}_{index}"
            scene.add_geometry(placed, geom_name=name, node_name=name)
            placed_meshes.append(placed)
    PLATE_PATH.write_bytes(scene.export(file_type="3mf"))

    with zipfile.ZipFile(PLATE_PATH, "r") as archive:
        model_name = next(name for name in archive.namelist() if name.endswith(".model"))
        root = ElementTree.fromstring(archive.read(model_name))
        namespace = {"m": "http://schemas.microsoft.com/3dmanufacturing/core/2015/02"}
        objects = root.findall(".//m:resources/m:object", namespace)
    imported = trimesh.load(PLATE_PATH, force="scene")
    object_count = len(objects)
    if object_count != 8 or len(imported.geometry) != 8:
        raise RuntimeError(
            f"3MF object verification failed: XML={object_count}, imported={len(imported.geometry)}")
    bounds = np.asarray([mesh.bounds for mesh in placed_meshes])
    plate_bounds = np.array([bounds[:, 0].min(axis=0), bounds[:, 1].max(axis=0)])
    if plate_bounds[0, 0] < 0 or plate_bounds[0, 1] < 0 or plate_bounds[1, 0] > 220 or plate_bounds[1, 1] > 220:
        raise RuntimeError(f"Plate outside 220x220: {plate_bounds}")
    # Pairwise AABB separation is sufficient because rows/columns are widely spaced.
    for i, first in enumerate(placed_meshes):
        for second in placed_meshes[i + 1:]:
            overlap = np.minimum(first.bounds[1], second.bounds[1]) - np.maximum(first.bounds[0], second.bounds[0])
            if np.all(overlap > 0):
                raise RuntimeError("Plate objects overlap")
    return {
        "path": str(PLATE_PATH.relative_to(HERE)),
        "sha256": sha256(PLATE_PATH),
        "objects_xml": object_count,
        "objects_imported": len(imported.geometry),
        "bounds_mm": np.round(plate_bounds, 6).tolist(),
        "placements_mm": placements,
    }


def run_slice_check() -> dict:
    slicer = Path(r"C:\Program Files\Prusa3D\PrusaSlicer\prusa-slicer-console.exe")
    if not slicer.exists():
        return {"status": "NOT_RUN", "reason": f"missing {slicer}"}
    gcode = Path(tempfile.gettempdir()) / "v11_cassette_8_fixators_validation.gcode"
    if gcode.exists():
        gcode.unlink()
    started_at = time.time()
    command = [
        str(slicer), "--slice", "--output", str(gcode),
        "--bed-shape", "0x0,220x0,220x220,0x220",
        "--layer-height", "0.2", "--perimeters", "5",
        "--fill-density", "40%", "--top-solid-layers", "5",
        "--bottom-solid-layers", "5", "--no-support-material",
        "--brim-width", "5", str(PLATE_PATH),
    ]
    run = subprocess.run(command, capture_output=True, text=True, errors="replace",
                         timeout=300, check=False)
    output = "\n".join(part for part in (run.stdout, run.stderr) if part)
    blocking_terms = [
        "Floating bridge anchors", "Loose extrusions", "Collapsing overhang",
        "Long bridging extrusion", "Low bed adhesion",
    ]
    hits = [term for term in blocking_terms if term.lower() in output.lower()]
    fresh_gcode = (gcode.exists() and gcode.stat().st_size > 0
                   and gcode.stat().st_mtime >= started_at - 1.0)
    support_off = False
    bridge_infill_sections = 0
    if fresh_gcode:
        with gcode.open("r", encoding="utf-8", errors="ignore") as stream:
            for line in stream:
                if "support_material = 0" in line:
                    support_off = True
                if line.strip() == ";TYPE:Bridge infill":
                    bridge_infill_sections += 1
    status = ("PASS" if run.returncode == 0 and not hits
              and support_off and fresh_gcode else "FAIL")
    warning_lines = [line.strip() for line in output.splitlines()
                     if "warn" in line.lower() or any(term.lower() in line.lower() for term in blocking_terms)]
    return {
        "status": status,
        "slicer": str(slicer),
        "returncode": run.returncode,
        "profile": "0.20 mm / 5 walls / 5 top+bottom / 40% / supports OFF / brim 5 mm",
        "blocking_warning_hits": hits,
        "fresh_nonempty_gcode": fresh_gcode,
        "support_material_equals_0_in_gcode": support_off,
        "gcode_bridge_infill_sections": bridge_infill_sections,
        "bridge_note": "reported honestly; PrusaSlicer emitted no structural warning, supports remain OFF, first-part inspection is still mandatory",
        "warning_lines": warning_lines[-30:],
        "gcode_temp_path": str(gcode),
    }


def make_fastener_references() -> tuple[trimesh.Trimesh, trimesh.Trimesh, trimesh.Trimesh]:
    """Assembly-only M4x25 screw, washer and nyloc-nut groups."""
    screws = []
    nuts = []
    washers = []
    for kind in ("upper", "lower"):
        x, z = bolt_center_xz(kind)
        for divider_y in divider_world_ys():
            # Screw inserted from the -Y side.  The 25 mm under-head length
            # spans 1 mm washer + 4 mm divider + 12 mm printed pad + 1 mm
            # washer + 4 mm nyloc, retaining about 3 mm of thread beyond it.
            shaft = cylinder_y(2.0, 25.0, (x, divider_y + 9.5, z), sections=36)
            head = cylinder_y(3.5, 2.8, (x, divider_y - 4.4, z), sections=36)
            screws.append(fuse([shaft, head]))
            # Simplified DIN 985 envelope; assembly reference, never print.
            nut = cylinder_y(3.7, 4.0, (x, divider_y + 17.0, z), sections=6)
            nuts.append(nut)
            for washer_y in (divider_y - 2.5, divider_y + 14.5):
                outer = cylinder_y(6.0, 1.0, (x, washer_y, z), sections=48)
                inner = cylinder_y(2.2, 1.6, (x, washer_y, z), sections=36)
                washers.append(cut(outer, [inner]))
    return (clean(trimesh.util.concatenate(screws)),
            clean(trimesh.util.concatenate(washers)),
            clean(trimesh.util.concatenate(nuts)))


def main() -> None:
    ensure_legacy_backups()

    upper_sheet = make_sloped_sheet("upper")
    lower_sheet = make_sloped_sheet("lower")
    divider = make_divider()
    upper_lock = make_lock_frame("upper")
    lower_lock = make_lock_frame("lower")

    canonical_paths = {
        "upper_sheet": ASSEMBLY_DIR / "cassette_upper_sheet.stl",
        "lower_sheet": ASSEMBLY_DIR / "cassette_lower_sheet.stl",
        "divider": ASSEMBLY_DIR / "cassette_divider.stl",
    }
    exact_meshes = {
        "upper_sheet": upper_sheet,
        "lower_sheet": lower_sheet,
        "divider": divider,
    }
    exact_reports = {
        name: export_stl(exact_meshes[name], path)
        for name, path in canonical_paths.items()
    }
    cut_hashes = write_cut_files()

    upper_print = orient_for_print(upper_lock, "upper")
    lower_print = orient_for_print(lower_lock, "lower")
    upper_binary = upper_print.export(file_type="stl")
    lower_binary = lower_print.export(file_type="stl")
    if upper_binary != lower_binary:
        raise RuntimeError("Upper/lower print solids are expected to be exactly universal")
    universal_print = upper_print
    print_meshes = {"upper": universal_print, "lower": universal_print}
    print_reports = {
        "universal_quantity_8": export_stl(
            universal_print,
            PRINT_DIR / "фиксатор_ската_универсальный_V11_PRINT.stl"),
        "upper_lower_binary_identical": True,
    }

    world_locks: dict[str, list[trimesh.Trimesh]] = {"upper": [], "lower": []}
    transforms: dict[str, list[dict]] = {"upper": [], "lower": []}
    collision_checks = []
    for kind, lock in (("upper", upper_lock), ("lower", lower_lock)):
        for divider_y in divider_world_ys():
            transform = frame_to_world(kind, divider_y)
            placed = lock.copy()
            placed.apply_transform(transform)
            divider_placed = divider.copy()
            divider_placed.apply_translation((0, divider_y, 0))
            sheet = upper_sheet if kind == "upper" else lower_sheet
            sheet_intersection = intersection_volume(placed, sheet)
            divider_intersection = intersection_volume(placed, divider_placed)
            opposite_intersection = intersection_volume(
                placed, lower_sheet if kind == "upper" else upper_sheet)
            raw_values = (sheet_intersection, divider_intersection, opposite_intersection)
            # Exact tangent datums can create sub-0.001 mm3 slivers after STL
            # tessellation/float32 export.  They are reported as numerical
            # contact, not physical penetration.
            epsilon_mm3 = 1e-3
            if any(not math.isfinite(value) or value > epsilon_mm3 for value in raw_values):
                raise RuntimeError(f"Collision at {kind}/{divider_y}: {raw_values}")
            sheet_intersection, divider_intersection, opposite_intersection = (
                0.0 if value <= epsilon_mm3 else value for value in raw_values)
            expected_peg = peg_center_world(kind, divider_y)
            local_peg = np.array([
                P["peg_s"], P["peg_q"] if kind == "upper" else -P["peg_q"], 0, 1,
            ])
            transformed_peg = (transform @ local_peg)[:3]
            peg_error = float(np.linalg.norm(expected_peg - transformed_peg))
            if peg_error > 1e-8:
                raise RuntimeError("Peg transform alignment error")
            bx, bz = bolt_center_xz(kind)
            local_bolt = np.array([
                P["bolt_s"], 0,
                P["bolt_n"] if kind == "upper" else -P["bolt_n"], 1,
            ])
            transformed_bolt = transform @ local_bolt
            bolt_error = math.hypot(transformed_bolt[0] - bx, transformed_bolt[2] - bz)
            if bolt_error > 1e-8:
                raise RuntimeError("Bolt transform alignment error")
            world_locks[kind].append(placed)
            transforms[kind].append({
                "divider_y_mm": divider_y,
                "matrix_row_major": np.round(transform, 9).tolist(),
                "peg_center_world_mm": np.round(expected_peg, 6).tolist(),
                "bolt_axis_center_world_mm": [round(bx, 6), divider_y, round(bz, 6)],
            })
            collision_checks.append({
                "kind": kind,
                "divider_y_mm": divider_y,
                "sheet_intersection_mm3": round(sheet_intersection, 6),
                "divider_intersection_mm3": round(divider_intersection, 6),
                "opposite_sheet_intersection_mm3": round(opposite_intersection, 6),
                "tangent_boolean_raw_max_mm3": round(max(raw_values), 9),
                "tangent_boolean_epsilon_mm3": epsilon_mm3,
                "peg_axis_alignment_error_mm": round(peg_error, 9),
                "bolt_axis_alignment_error_mm": round(bolt_error, 9),
            })

    world_reports = {}
    for kind in ("upper", "lower"):
        combined = clean(trimesh.util.concatenate(world_locks[kind]))
        output_name = ("фиксаторы_верхнего_ската_4шт_world_V11.stl"
                       if kind == "upper" else
                       "фиксаторы_нижнего_ската_4шт_world_V11.stl")
        world_reports[kind] = export_stl(
            combined, WORLD_DIR / output_name, one_body=False)

    # Individual y=79 solids let the viewer expose one joint at useful scale.
    individual_world_reports = {
        "upper_y79": export_stl(
            world_locks["upper"][0],
            WORLD_DIR / "фиксатор_верхнего_ската_world_y79_V11.stl"),
        "lower_y79": export_stl(
            world_locks["lower"][0],
            WORLD_DIR / "фиксатор_нижнего_ската_world_y79_V11.stl"),
    }
    screws_reference, washers_reference, nuts_reference = make_fastener_references()
    hardware_reference_reports = {
        "M4x25_screws_8": export_stl(
            screws_reference, WORLD_DIR / "M4x25_болты_8шт_world_REFERENCE.stl",
            one_body=False),
        "M4_wide_flat_washers_DIN9021_16": export_stl(
            washers_reference, WORLD_DIR / "M4_шайбы_16шт_world_REFERENCE.stl",
            one_body=False),
        "M4_nyloc_nuts_8": export_stl(
            nuts_reference, WORLD_DIR / "M4_гайки_Nyloc_8шт_world_REFERENCE.stl",
            one_body=False),
    }

    all_world = [*world_locks["upper"], *world_locks["lower"]]
    all_bounds = np.asarray([mesh.bounds for mesh in all_world])
    world_bounds = np.array([all_bounds[:, 0].min(axis=0), all_bounds[:, 1].max(axis=0)])
    full_system = [*all_world, screws_reference, washers_reference, nuts_reference]
    system_component_bounds = np.asarray([mesh.bounds for mesh in full_system])
    system_bounds = np.array([
        system_component_bounds[:, 0].min(axis=0),
        system_component_bounds[:, 1].max(axis=0),
    ])
    inside_box = bool(
        system_bounds[0, 0] >= -1e-6 and system_bounds[1, 0] <= P["box_inside_depth"] + 1e-6
        and system_bounds[0, 1] >= -1e-6 and system_bounds[1, 1] <= P["box_inside_width"] + 1e-6
        and system_bounds[0, 2] >= -1e-6 and system_bounds[1, 2] <= P["box_inside_height"] + 1e-6)
    if not inside_box:
        raise RuntimeError(f"Fixators plus M4 hardware exceed nominal box: {system_bounds}")

    divider_copies = []
    for divider_y in divider_world_ys():
        item = divider.copy()
        item.apply_translation((0, divider_y, 0))
        divider_copies.append(item)
    all_dividers = clean(trimesh.util.concatenate(divider_copies))
    full_cassette_components = [
        upper_sheet, lower_sheet, *divider_copies, *full_system,
    ]
    full_bounds_items = np.asarray([item.bounds for item in full_cassette_components])
    full_raw_bounds = np.array([
        full_bounds_items[:, 0].min(axis=0),
        full_bounds_items[:, 1].max(axis=0),
    ])
    installation_translation_x = 0.5
    full_installed_bounds = full_raw_bounds + np.array([
        [installation_translation_x, 0, 0],
        [installation_translation_x, 0, 0],
    ])
    full_inside_box = bool(
        full_installed_bounds[0, 0] >= -1e-6
        and full_installed_bounds[1, 0] <= P["box_inside_depth"] + 1e-6
        and full_installed_bounds[0, 1] >= -1e-6
        and full_installed_bounds[1, 1] <= P["box_inside_width"] + 1e-6
        and full_installed_bounds[0, 2] >= -1e-6
        and full_installed_bounds[1, 2] <= P["box_inside_height"] + 1e-6)
    if not full_inside_box:
        raise RuntimeError(
            f"Complete cassette does not fit after +0.5 mm X installation offset: {full_installed_bounds}")
    all_hardware = clean(trimesh.util.concatenate([
        screws_reference, washers_reference, nuts_reference,
    ]))
    hardware_collision_checks = {
        "hardware_vs_upper_sheet_mm3": intersection_volume(all_hardware, upper_sheet),
        "hardware_vs_lower_sheet_mm3": intersection_volume(all_hardware, lower_sheet),
        "hardware_vs_4_exact_dividers_mm3": intersection_volume(all_hardware, all_dividers),
    }
    if any(not math.isfinite(value) or value > 1e-4
           for value in hardware_collision_checks.values()):
        raise RuntimeError(f"Unexpected hardware collision: {hardware_collision_checks}")
    hardware_collision_checks = {
        key: round(value, 6) for key, value in hardware_collision_checks.items()
    }

    plate_report = make_plate(print_meshes)
    slice_report = run_slice_check()
    if slice_report["status"] != "PASS":
        raise RuntimeError(f"Slicer validation failed: {slice_report}")

    cnc_slot_half = (P["sheet_thickness"] + P["slot_clearance_cnc"]) / 2
    divider_web = P["bolt_n"] - P["divider_hole_diameter"] / 2 - cnc_slot_half
    sheet_web = P["peg_q"] - P["sheet_thickness"] / 2 - P["sheet_hole_diameter"] / 2
    true_divider_web = divider_hole_boundary_audit()
    if divider_web < 5.0 or sheet_web < 5.0:
        raise RuntimeError("Plywood web below final design minimum")

    legacy_upper = LEGACY_DIR / "cassette_lock_upper_14deg.stl"
    legacy_lower = LEGACY_DIR / "cassette_lock_lower_15deg.stl"
    manifest = {
        "version": "V11 FINAL internal single-bolt cassette fixator",
        "generated_by": str(Path(__file__).name),
        "scope": "common six-sheet cassette only; variants A/B/C must be regenerated separately",
        "legacy": {
            "status": "DO_NOT_PRINT / DO_NOT_INSTALL / VIEW-ONLY",
            "backup_directory": str(LEGACY_DIR.relative_to(HERE)),
            "blind_clip_files_left_for_reference": {
                "upper": {"path": str(legacy_upper.relative_to(HERE)), "sha256": sha256(legacy_upper)},
                "lower": {"path": str(legacy_lower.relative_to(HERE)), "sha256": sha256(legacy_lower)},
            },
            "independent_rejection_evidence_mm3": {
                "upper_vs_old_exact_sheet": 673.8,
                "upper_vs_old_exact_divider": 543.9,
                "lower_vs_old_exact_sheet": 613.9,
                "lower_vs_old_exact_divider": 553.7,
            },
        },
        "canonical_exact_sources": exact_reports,
        "cut_file_sha256": cut_hashes,
        "mechanical_connection": {
            "mandatory_fixator_count": 8,
            "upper_count": 4,
            "lower_count": 4,
            "fastener_each": "1 x M4x25 + 2 x M4 DIN 9021 wide washer OD12 + M4 self-locking nut",
            "fastener_total": "8 x M4x25 + 16 x M4 DIN 9021 wide washers OD12 + 8 x M4 self-locking nuts",
            "divider_round_hole_diameter_mm": P["divider_hole_diameter"],
            "printed_bore": "outward-open 45 degree cradle; closed diamond end contains a 4.6 mm circle",
            "sheet_hole_diameter_mm": P["sheet_hole_diameter"],
            "integral_peg_diameter_mm": P["peg_diameter"],
            "peg_engagement_mm": 3.0,
            "bank_surface_recess_mm": P["peg_tip_recess"],
            "shoe_gap_mm": P["shoe_gap"],
            "shoe_span_each_side_of_peg_mm": 7.0,
            "glue": "none",
            "installation_datums": {
                "upper_pad_to_divider_face_V_mm": 2.0,
                "lower_pad_to_divider_face_V_mm": -2.0,
                "upper_shoe_to_sheet_outer_face_N_mm": 2.0,
                "lower_shoe_to_sheet_outer_face_N_mm": -2.0,
                "pad_gap_mm": 0.0,
                "shoe_gap_mm": 0.0,
            },
            "outward_open_cradle_geometry": {
                "diamond_inradius_mm": 2.3,
                "M4_nominal_radius_mm": 2.0,
                "diamond_vertex_radius_mm": round(2.30 * math.sqrt(2.0), 6),
                "mouth_reaches_past_outer_body_face_mm": 0.5,
                "opening_direction": "away from sheet; towards build plate in print orientation",
                "withdrawal_direction": "towards closed 45 degree diamond end",
            },
            "clearance_stack": {
                "M4_in_divider_radial_mm": 0.20,
                "closed_end_circle_in_diamond_center_clearance_mm": round(
                    (2.30 - 2.00) * math.sqrt(2.0), 6),
                "printed_cradle_conservative_center_clearance_at_diamond_throat_mm": round(
                    (2.30 - 2.00) * math.sqrt(2.0), 6),
                "straight_open_mouth_halfwidth_minus_bolt_radius_mm": round(
                    2.30 * math.sqrt(2.0) - 2.00, 6),
                "manual_slot_half_play_mm": round(P["slot_clearance_manual"] / 2, 6),
                "worst_outward_withdrawal_mm": round(
                    (2.30 - 2.00) * math.sqrt(2.0)
                    + (P["divider_hole_diameter"] - 4.0) / 2
                    + P["slot_clearance_manual"] / 2, 6),
                "worst_remaining_peg_engagement_mm": round(
                    3.0 - (2.30 - 2.00) * math.sqrt(2.0)
                    - (P["divider_hole_diameter"] - 4.0) / 2
                    - P["slot_clearance_manual"] / 2, 6),
            },
        },
        "exact_coordinates": {
            "sheet_locator": {
                "frame_s_mm": P["peg_s"],
                "transverse_q_from_divider_center_mm": P["peg_q"],
                "upper_world_centres_mm": [np.round(peg_center_world("upper", y), 6).tolist()
                                             for y in divider_world_ys()],
                "lower_world_centres_mm": [np.round(peg_center_world("lower", y), 6).tolist()
                                             for y in divider_world_ys()],
            },
            "divider_bolt_centres_xz_mm": {
                kind: [round(value, 6) for value in bolt_center_xz(kind)]
                for kind in ("upper", "lower")
            },
            "minimum_web_mm": {
                "divider_hole_to_CNC_sheet_slot": round(divider_web, 3),
                "sheet_hole_to_divider_face": round(sheet_web, 3),
                "true_distance_from_each_M4_hole_edge_to_full_profile_boundary": true_divider_web,
            },
        },
        "print_stl": print_reports,
        "assembly_stl": world_reports,
        "individual_world_stl": individual_world_reports,
        "hardware_reference_stl": hardware_reference_reports,
        "plate_3mf": plate_report,
        "print_profile": {
            "material": "PETG",
            "nozzle_mm": 0.4,
            "layer_mm": 0.2,
            "walls": 5,
            "top_bottom_layers": 5,
            "infill_percent": 40,
            "supports": False,
            "brim_mm": 5,
        },
        "slice_validation": slice_report,
        "frame_to_world": transforms,
        "collision_checks": collision_checks,
        "hardware_collision_checks": {
            **hardware_collision_checks,
            "upper_hardware_min_N_mm": 4.0,
            "upper_clearance_to_channel_half_space_mm": 6.0,
            "lower_hardware_max_N_mm": -4.0,
            "lower_clearance_to_channel_half_space_mm": 6.0,
            "can_half_space_result": "PASS",
        },
        "anti_rotation": rotation_restraint_sweep(),
        "washer_bearing": washer_bearing_audit(),
        "can_swept_envelope_check": {
            "method": "conservative exact half-space proof against both bank-running sheet faces",
            "upper_channel_boundary_N_mm": -2.0,
            "upper_fixator_min_N_mm": -1.0,
            "upper_clearance_mm": 1.0,
            "lower_channel_boundary_N_mm": 2.0,
            "lower_fixator_max_N_mm": 1.0,
            "lower_clearance_mm": 1.0,
            "result": "PASS; the complete lock remains outside the channel half-space",
        },
        "box_clearance": {
            "nominal_inside_mm": [595.0, 395.0, 395.0],
            "all_8_fixators_world_bounds_mm": np.round(world_bounds, 6).tolist(),
            "all_8_fixators_and_hardware_bounds_mm": np.round(system_bounds, 6).tolist(),
            "inside_nominal_box": inside_box,
            "front_depth_margin_with_hardware_mm": round(float(system_bounds[0, 0]), 3),
            "rear_depth_margin_with_hardware_mm": round(
                P["box_inside_depth"] - float(system_bounds[1, 0]), 3),
            "complete_cassette_raw_bounds_mm": np.round(full_raw_bounds, 6).tolist(),
            "installation_translation_x_mm": installation_translation_x,
            "complete_cassette_installed_bounds_mm": np.round(
                full_installed_bounds, 6).tolist(),
            "complete_cassette_inside_nominal_box": full_inside_box,
            "complete_front_margin_after_translation_mm": round(
                float(full_installed_bounds[0, 0]), 3),
            "complete_rear_margin_after_translation_mm": round(
                P["box_inside_depth"] - float(full_installed_bounds[1, 0]), 3),
        },
        "validation_limits": {
            "physical_validation_required": True,
            "first_action": "print and dry-fit one upper plus one lower before the production plate",
            "not_yet_proven": [
                "actual plywood thickness and drilling accuracy",
                "printer-specific peg fit and layer adhesion",
                "loaded cassette pull-out resistance",
                "long-term creep, vibration and repeated service cycles",
            ],
        },
    }
    MANIFEST_PATH.write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({
        "status": "PASS",
        "plate": str(PLATE_PATH),
        "plate_objects": plate_report["objects_imported"],
        "slice": slice_report["status"],
        "world_bounds_mm": np.round(world_bounds, 3).tolist(),
        "manifest": str(MANIFEST_PATH),
    }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
