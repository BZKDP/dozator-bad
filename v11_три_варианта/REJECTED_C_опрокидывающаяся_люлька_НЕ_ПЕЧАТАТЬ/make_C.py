# -*- coding: utf-8 -*-
"""V11-C — простая опрокидывающаяся люлька на одну банку.

Два рабочих печатных элемента на канал:
  C01 — несущая рама с креплением к передней кромке ската;
  C02 — люлька с передним упором, задним ножом и карманом штатной качалки.

Ось люльки совпадает с осью SG90/MG90S. Справа люлька поддержана винтом
M4 с гладким плечом. HOME = 0°, EJECT = -50°. При начале движения задняя
кромка поднимается между первой и второй банками раньше, чем передний упор
уходит ниже траектории первой банки.
"""
from __future__ import annotations

import csv
import json
import math
import shutil
from pathlib import Path

import numpy as np
import trimesh
from shapely import affinity
from shapely.geometry import Point, Polygon, box as sbox
from shapely.ops import unary_union


HERE = Path(__file__).resolve().parent
PRINT_DIR = HERE / "STL_ПЕЧАТЬ"
ASSEMBLY_DIR = HERE / "STL_СБОРКА_НЕ_ПЕЧАТАТЬ"
for directory in (PRINT_DIR, ASSEMBLY_DIR):
    directory.mkdir(parents=True, exist_ok=True)


P = {
    "can_length": 70.0,
    "can_diameter_nominal": 47.0,
    "can_diameter_design": 48.0,
    "lane_clear": 75.0,
    "lane_pitch": 79.0,
    "sheet_thickness": 4.0,
    "lower_slope_deg": 15.0,
    "shelf_overhang_nominal": 150.0,
    "servo_motion_deg": 50.0,
    "cradle_axis_y": -2.0,
    "cradle_axis_z": -8.0,
    "cradle_rear_y": -28.0,
    "cradle_front_y": 24.0,
    "cradle_width": 60.0,
    "frame_outer_width": 78.4,
    "frame_inner_width": 72.4,
    "attachment_hole_pitch": 50.0,
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
    result = trimesh.boolean.union(items, engine="manifold")
    if result is None:
        raise RuntimeError("Boolean union failed")
    return clean(result)


def cut(mesh, cutters) -> trimesh.Trimesh:
    tools = [clean(c) for c in cutters if c is not None and len(c.faces)]
    if not tools:
        return clean(mesh)
    result = trimesh.boolean.difference([clean(mesh), *tools], engine="manifold")
    if result is None:
        raise RuntimeError("Boolean difference failed")
    return clean(result)


def bx(size, center=(0.0, 0.0, 0.0)):
    mesh = trimesh.creation.box(extents=np.asarray(size, dtype=float))
    mesh.apply_translation(np.asarray(center, dtype=float))
    return mesh


def cylinder_x(radius, length, center=(0.0, 0.0, 0.0), sections=48):
    mesh = trimesh.creation.cylinder(radius=radius, height=length, sections=sections)
    mesh.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0]))
    mesh.apply_translation(np.asarray(center, dtype=float))
    return mesh


def rotate(mesh, angle_deg, axis=(1, 0, 0), point=(0.0, 0.0, 0.0)):
    result = mesh.copy()
    result.apply_transform(trimesh.transformations.rotation_matrix(
        math.radians(angle_deg), axis, point))
    return result


def translate(mesh, xyz):
    result = mesh.copy()
    result.apply_translation(np.asarray(xyz, dtype=float))
    return result


def on_bed(mesh):
    result = clean(mesh)
    result.apply_translation([0, 0, -result.bounds[0, 2]])
    return result


def extrude_yz(poly, thickness, x0=0.0):
    mesh = trimesh.creation.extrude_polygon(poly, height=thickness)
    transform = np.array(
        [[0, 0, 1, x0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]],
        dtype=float,
    )
    mesh.apply_transform(transform)
    return clean(mesh)


def export(mesh, path: Path):
    mesh = clean(mesh)
    mesh.export(path)
    return {
        "file": path.name,
        "extents_mm": [round(float(x), 2) for x in mesh.extents],
        "watertight": bool(mesh.is_watertight),
        "components": len(mesh.split(only_watertight=False)),
        "volume_cm3": round(abs(float(mesh.volume)) / 1000.0, 2),
    }


def horn_outline(clearance=0.35):
    """Envelope of the common stock SG90/MG90S double horn, not a spline."""
    hub = Point(0, 0).buffer(3.70 + clearance, resolution=40)
    arm = sbox(-15.8 - clearance, -3.45 - clearance,
               15.8 + clearance, 3.45 + clearance)
    tips = unary_union([
        Point(-15.8, 0).buffer(2.0 + clearance, resolution=24),
        Point(15.8, 0).buffer(2.0 + clearance, resolution=24),
    ])
    return unary_union([hub, arm, tips])


def teardrop_x_hole(radius, length, center):
    """Horizontal printable M4 clearance: round lower half + 45° roof."""
    y, z = 0.0, 0.0
    profile = Point(y, z).buffer(radius, resolution=32)
    roof = Polygon([(-radius, 0), (0, radius * 1.42), (radius, 0)])
    return translate(extrude_yz(unary_union([profile, roof]), length, x0=-length / 2), center)


def frame_mesh():
    """One-piece U-frame; road is Z=0, queue enters from negative Y."""
    ow = P["frame_outer_width"]
    iw = P["frame_inner_width"]
    wall = (ow - iw) / 2
    y0, y1 = -82.0, 43.0
    z0, z1 = -43.0, 34.0
    parts = [
        # Two side cheeks are outside the 70 mm can ends.
        bx((wall, y1 - y0, z1 - z0),
           (-ow / 2 + wall / 2, (y0 + y1) / 2, (z0 + z1) / 2)),
        bx((wall, y1 - y0, z1 - z0),
           (ow / 2 - wall / 2, (y0 + y1) / 2, (z0 + z1) / 2)),
        # Lower bridge keeps the frame square and sits entirely below the bucket.
        bx((ow, 25.0, 5.0), (0, 23.0, -40.5)),
        # 32 mm tongue under the front edge of the 4 mm plywood chute.
        bx((ow, 32.0, 3.0), (0, -66.0, -5.5)),
        # Short underside ribs transmit queue load into the tongue.
        bx((7.0, 33.0, 34.0), (-34.7, -57.5, -22.0)),
        bx((7.0, 33.0, 34.0), (34.7, -57.5, -22.0)),
        # EJECT hard stop: the bucket underside lands here at -50°.
        bx((ow - 5.0, 5.0, 5.0), (0, 31.0, -31.0)),
        # HOME hard-stop pads at the rear underside.
        bx((10.0, 8.0, 7.0), (-31.5, -29.0, -10.5)),
        bx((10.0, 8.0, 7.0), (31.5, -29.0, -10.5)),
    ]
    frame = fuse(parts)
    cutters = []
    axis = (0, P["cradle_axis_y"], P["cradle_axis_z"])
    # Right support accepts one M4 shoulder screw; left opening clears horn/shaft.
    cutters.append(cylinder_x(2.20, 10.0, (ow / 2 - wall / 2, axis[1], axis[2])))
    cutters.append(cylinder_x(6.50, 10.0, (-ow / 2 + wall / 2, axis[1], axis[2])))

    # Two M2.2 holes for the actual SG90/MG90S mounting ears.
    # Their slots provide ±2 mm accommodation for clone housings.
    for yy in (-18.3, 14.3):
        cutters.extend([
            cylinder_x(1.15, 9.0, (-ow / 2 + wall / 2, yy - 2.0, -22.0), 28),
            cylinder_x(1.15, 9.0, (-ow / 2 + wall / 2, yy + 2.0, -22.0), 28),
            bx((9.0, 4.0, 2.3), (-ow / 2 + wall / 2, yy, -22.0)),
        ])

    # Two M3 attachment holes through the under-tongue. Screw heads are below;
    # the matching plywood holes are countersunk from the rolling surface.
    for xx in (-P["attachment_hole_pitch"] / 2, P["attachment_hole_pitch"] / 2):
        cutters.append(trimesh.creation.cylinder(
            radius=1.70, height=10.0, sections=32,
            transform=trimesh.transformations.translation_matrix([xx, -66.0, -5.5])))
        cutters.append(trimesh.creation.cylinder(
            radius=3.35, height=2.4, sections=36,
            transform=trimesh.transformations.translation_matrix([xx, -66.0, -7.0])))

    # Large lightening windows; keep continuous borders and all hard stops.
    for xx in (-ow / 2 + wall / 2, ow / 2 - wall / 2):
        cutters.append(bx((wall + 2.0, 66.0, 39.0), (xx, 10.0, 13.0)))
        cutters.append(bx((wall + 2.0, 28.0, 24.0), (xx, -55.0, 14.0)))
    return cut(frame, cutters)


def cradle_mesh():
    """One-piece moving bucket, exported at HOME."""
    width = P["cradle_width"]
    rear = P["cradle_rear_y"]
    front = P["cradle_front_y"]
    axis_y = P["cradle_axis_y"]
    axis_z = P["cradle_axis_z"]
    parts = [
        # 3 mm floor, flush with the plywood road in HOME.
        bx((width, front - rear, 3.0), (5.5, (rear + front) / 2, -1.5)),
        # Downstream stop. At EJECT its entire top moves below the exit line.
        bx((width, 3.2, 25.0), (5.5, front - 1.6, 12.5)),
        # Rear fin starts below the road; rotation raises it between cans.
        bx((width, 3.0, 19.0), (5.5, rear + 1.5, -9.5)),
        # Two triangular side gussets connect floor and front stop.
    ]
    gusset_yz = Polygon([(front - 8.0, -3.0), (front, -3.0),
                         (front, 15.0)])
    parts.append(extrude_yz(gusset_yz, 3.0, x0=-24.5))
    parts.append(extrude_yz(gusset_yz, 3.0, x0=32.5))

    # Coaxial underside tube. The can remains above Z=0; all drive hardware is below.
    hub = cylinder_x(7.2, width, (5.5, axis_y, axis_z), sections=56)
    parts.append(hub)
    cradle = fuse(parts)

    # Right M4 blind support bore and central stock-horn pocket from the left.
    right_bore = cylinder_x(2.10, 14.0, (31.0, axis_y, axis_z), sections=32)
    pocket_poly = affinity.rotate(horn_outline(0.35), 90, origin=(0, 0))
    pocket = extrude_yz(pocket_poly, 3.25, x0=-24.7)
    center_screw = cylinder_x(1.35, width + 6.0, (5.5, axis_y, axis_z), sections=28)
    return cut(cradle, [right_bore, pocket, center_screw])


def horn_coupon_mesh():
    base = bx((118.0, 30.0, 3.2), (0, 0, 1.6))
    cuts = []
    for idx, (xx, clearance) in enumerate(zip((-39.0, 0.0, 39.0),
                                               (0.20, 0.35, 0.50)), start=1):
        poly = affinity.translate(horn_outline(clearance), xoff=xx)
        pocket = trimesh.creation.extrude_polygon(poly, height=2.6)
        pocket.apply_translation([0, 0, 0.8])
        cuts.append(pocket)
        for mark in range(idx):
            cuts.append(bx((1.2, 4.0, 4.0),
                           (xx - 2.0 + mark * 2.0, -14.5, 2.0)))
    return cut(base, cuts)


def servo_reference():
    """Envelope only; SG90 and MG90S fit the same adjustable mount."""
    # Shaft points along +X. Body stays inside the 78.4 mm module width.
    body = bx((12.2, 22.8, 28.5), (-31.1, 0.0, -22.25))
    ears = bx((3.0, 32.5, 2.5), (-24.0, -2.0, -22.0))
    shaft = cylinder_x(2.4, 5.0, (-23.5, P["cradle_axis_y"], P["cradle_axis_z"]), 40)
    return fuse([body, ears, shaft])


def stock_horn_reference():
    poly = affinity.rotate(horn_outline(0.0), 90, origin=(0, 0))
    horn = extrude_yz(poly, 2.0, x0=-25.1)
    hole = cylinder_x(1.25, 4.0,
                      (-24.1, P["cradle_axis_y"], P["cradle_axis_z"]), 28)
    return cut(horn, [hole])


def can_mesh(center_y, center_z=24.0):
    body = cylinder_x(P["can_diameter_design"] / 2, P["can_length"],
                      (0, center_y, center_z), sections=64)
    # Shallow end rings make the direction and true 70 mm length obvious.
    rings = [
        cylinder_x(24.4, 1.2, (-34.4, center_y, center_z), 64),
        cylinder_x(24.4, 1.2, (34.4, center_y, center_z), 64),
    ]
    return fuse([body, *rings])


def sheet_reference():
    # Local flat reference. In the five-lane assembly it is rotated by 15°.
    return bx((75.0, 115.0, 4.0), (0, -111.5, -2.0))


def assembly_state(angle_deg):
    axis = (0, P["cradle_axis_y"], P["cradle_axis_z"])
    moving = rotate(cradle_mesh(), angle_deg, point=axis)
    horn = rotate(stock_horn_reference(), angle_deg, point=axis)
    parts = {
        "frame": frame_mesh(),
        "cradle": moving,
        "servo": servo_reference(),
        "stock_horn": horn,
        "lower_sheet": sheet_reference(),
    }
    return parts


def can_positions(angle_deg):
    """Visual motion: first can follows the bucket early, then exits by gravity."""
    axis_y, axis_z = P["cradle_axis_y"], P["cradle_axis_z"]
    a = math.radians(angle_deg)
    # First can is nearly concentric with the bucket pivot; rotation mainly opens its support.
    y0, z0 = -1.5, 24.0
    if angle_deg >= -38.0:
        dy, dz = y0 - axis_y, z0 - axis_z
        y = axis_y + math.cos(a) * dy - math.sin(a) * dz
        z = axis_z + math.sin(a) * dy + math.cos(a) * dz
    else:
        q = min(1.0, (-angle_deg - 38.0) / 12.0)
        dy, dz = y0 - axis_y, z0 - axis_z
        yr = axis_y + math.cos(a) * dy - math.sin(a) * dz
        zr = axis_z + math.sin(a) * dy + math.cos(a) * dz
        y, z = yr + 23.0 * q, zr - 20.0 * q
    return [(y, z), (-49.8, 24.0), (-98.1, 24.0)]


def export_state(name, angle_deg, reports):
    parts = assembly_state(angle_deg)
    for key, mesh in parts.items():
        reports.append(export(mesh, ASSEMBLY_DIR / f"C_{name}_{key}.stl"))
    cans = []
    for idx, (yy, zz) in enumerate(can_positions(angle_deg), start=1):
        c = can_mesh(yy, zz)
        reports.append(export(c, ASSEMBLY_DIR / f"C_{name}_can_{idx}.stl"))
        cans.append(c)
    # A viewing assembly deliberately contains separate touching bodies;
    # concatenation preserves them without pretending they form one volume.
    joined = clean(trimesh.util.concatenate([*parts.values(), *cans]))
    reports.append(export(joined, ASSEMBLY_DIR / f"C_{name}_complete.stl"))


def write_bom():
    rows = [
        ("C01", "Одноцельная рама с подлистовым языком", 1, "PETG", "на боку; только локальная поддержка верхней щеки"),
        ("C02", "Опрокидывающаяся люлька", 1, "PETG", "торцом, карманом качалки вверх; без поддержек; 5 стенок"),
        ("T01", "Купон штатной качалки", 1, "PETG", "печатать до люльки"),
        ("H01", "SG90 или MG90S + штатная двойная качалка", 1, "готовая", "существующая серва"),
        ("H02", "M4×16 с гладким плечом + самоконтрящаяся гайка", 1, "металл", "правая опора оси"),
        ("H03", "M2×10/12 + гайка", 2, "металл", "уши сервы"),
        ("H04", "M3×12 ISO10642 + гайка", 2, "металл", "рама к нижнему листу; потай сверху"),
        ("H05", "Штатный центральный винт качалки", 1, "комплект сервы", "люлька к качалке"),
    ]
    with (HERE / "BOM_C.csv").open("w", encoding="utf-8-sig", newline="") as f:
        w = csv.writer(f, delimiter=";")
        w.writerow(("Код", "Деталь", "Количество на канал", "Материал", "Примечание"))
        w.writerows(rows)


def analytical_checks():
    rear_y = P["cradle_rear_y"]
    front_y = P["cradle_front_y"]
    axis_y = P["cradle_axis_y"]
    axis_z = P["cradle_axis_z"]
    values = []
    for angle in np.linspace(0.0, -P["servo_motion_deg"], 101):
        a = math.radians(float(angle))
        def rz(y, z):
            return axis_z + math.sin(a) * (y - axis_y) + math.cos(a) * (z - axis_z)
        rear_top = rz(rear_y, 0.0)
        front_top = rz(front_y, 25.0)
        front_bottom = rz(front_y, 0.0)
        values.append((float(angle), rear_top, front_top, front_bottom))
    rear_10 = next(v for v in values if v[0] <= -10.0)[1]
    front_10 = next(v for v in values if v[0] <= -10.0)[2]
    eject = values[-1]
    return {
        "rear_edge_height_at_minus10_deg_mm": round(rear_10, 2),
        "front_stop_top_at_minus10_deg_mm": round(front_10, 2),
        "eject_rear_edge_height_mm": round(eject[1], 2),
        "eject_front_stop_top_mm": round(eject[2], 2),
        "eject_front_stop_bottom_mm": round(eject[3], 2),
        "sequence_overlap": bool(rear_10 > 4.0 and front_10 > 18.0),
        "first_can_static_gravity_torque_Nm": round(0.2 * 9.81 * 0.0005, 4),
        "note": "масса 200 г, центр банки в HOME в 0.5 мм от вертикали оси; динамика проверяется физически",
    }


def main():
    # Remove only outputs owned by this generator.
    for directory in (PRINT_DIR, ASSEMBLY_DIR):
        for path in directory.glob("*.stl"):
            path.unlink()

    print_reports = []
    frame_print = on_bed(rotate(frame_mesh(), 90, [0, 1, 0]))
    # Grow the bucket along its 60 mm width. All lips then become vertical
    # walls and the stock-horn pocket remains open upward.
    cradle_print = on_bed(rotate(cradle_mesh(), 90, [0, 1, 0]))
    coupon_print = on_bed(horn_coupon_mesh())
    print_reports.append(export(frame_print, PRINT_DIR / "C01_рама_крепление.stl"))
    print_reports.append(export(cradle_print, PRINT_DIR / "C02_люлька_одной_банки.stl"))
    print_reports.append(export(coupon_print, PRINT_DIR / "T01_КУПОН_штатной_качалки.stl"))

    assembly_reports = []
    export_state("HOME", 0.0, assembly_reports)
    export_state("TRANSFER", -15.0, assembly_reports)
    export_state("EJECT", -50.0, assembly_reports)
    write_bom()

    checks = analytical_checks()
    bed_ok = all(r["extents_mm"][0] <= 220 and r["extents_mm"][1] <= 220
                 for r in print_reports)
    mesh_ok = all(r["watertight"] and r["components"] == 1 for r in print_reports)
    manifest = {
        "variant": "V11-C",
        "name": "Опрокидывающаяся люлька",
        "status": "REJECTED_DO_NOT_PRINT",
        "principle": "rear edge isolates queue before front stop drops and releases the loaded can",
        "parameters_mm": P,
        "simplicity": {
            "functional_printed_parts_per_channel": 2,
            "calibration_coupons_once": 1,
            "fasteners_per_channel_excluding_stock_horn_screw": 5,
            "assembly_operations": 4,
            "attachment_operations": 2,
            "supports": "cradle: none; frame: one simple local support under the upper cheek",
        },
        "kinematic_checks": checks,
        "print_bed_220_ok": bed_ok,
        "print_meshes_ok": mesh_ok,
        "print_parts": print_reports,
        "assembly_parts": assembly_reports,
        "rejection_reasons": [
            "effective rear stop contacts the approaching Ø48 can only about 5.6 mm above the road at -50 deg",
            "reliable dumping over the 25 mm front wall would require about 80 deg and makes full-queue return torque unsafe for SG90",
            "mesh audit found cradle/frame, frame/servo and sheet/frame collisions plus a 26 mm unsupported road gap",
        ],
        "risks": [
            "actual horn pocket must be selected with T01 because clone horns vary",
            "front outlet needs at least 55 mm free height below/forward of the chute",
            "the first physical channel must be tested with the heaviest full queue for 300 cycles",
            "the existing box window coordinates are not yet measured",
        ],
    }
    (HERE / "MANIFEST_C.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")

    readme = f"""# ОТБРАКОВАНО — опрокидывающаяся люлька\n\n
Этот вариант **не печатать и не использовать**. Он сохранён только как проверенная и отклонённая ветка, чтобы не вернуться к той же ошибке.\n\n
Причина: при ходе 50° задняя кромка касается следующей банки слишком низко; при ходе, достаточном для надёжной выдачи, обратный момент полной очереди приближается к пределу SG90 или превышает его. Дополнительно аудит обнаружил коллизии и разрыв дорожки. В итоговое сравнение вариант не входит.\n\n
Рабочих печатных деталей на канал: **2**. Серва — одна существующая SG90/MG90S.\n\n
HOME: дорожка люльки продолжает скат, первая банка упирается в переднюю стенку.\n
Переход: задняя кромка поднимается между первой и второй банками, пока передний упор ещё закрыт.\n
EJECT: передний упор полностью уходит вниз, наружу выходит банка, уже находившаяся в люльке.\n\n
Крепление: два одинаковых M3 с потайной головкой через передний край нижнего листа. Рама находится в зоне свеса коробки и не требует балки или подиума.\n\n
Проверки генератора: сетки={mesh_ok}, стол 220×220={bed_ok}, перекрытие фаз={checks['sequence_overlap']}. Это полноразмерный прототип для сравнения; до тиражирования обязателен один физический канал и 300 циклов.\n"""
    (HERE / "README_ОТБРАКОВАНО.md").write_text(readme, encoding="utf-8")
    print(json.dumps({"print_parts": len(print_reports), "mesh_ok": mesh_ok,
                      "bed_ok": bed_ok, "checks": checks}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
