# -*- coding: utf-8 -*-
"""Generate dimensioned V9 concept meshes and nominal cut data.

These meshes are intentionally marked as concepts.  They are for checking
scale, clearances and kinematics before a single-lane production prototype is
dimensioned around the real sheet thickness and the real can sample.
"""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import numpy as np
import trimesh
from shapely.geometry import Polygon


HERE = Path(__file__).resolve().parent
MESH_DIR = HERE / "3D_концепты_НЕ_ПЕЧАТАТЬ"
MESH_DIR.mkdir(exist_ok=True)

# Latest project branch: korobka_v8_usilennaya.html + stl_gate12.
P = {
    "can_length": 70.0,
    "can_diameter": 47.0,
    "box_inside_depth": 595.0,
    "box_inside_width": 395.0,
    "box_inside_height": 395.0,
    "cassette_depth": 590.0,
    "cassette_width": 391.0,
    "cassette_height": 390.0,
    "divider_thickness": 4.0,
    "lane_clear": 75.0,
    "lane_pitch": 79.0,
    "strip_clearance": 0.6,
    "lower_angle_deg": 15.0,
    "upper_angle_deg": 14.0,
    "upper_run": 535.0,
    "upper_front_height": 345.0,
    "outside_nose_run": 240.0,
    "gate_pitch": 52.0,
    "gate_blade_width": 72.0,
    "gate_blade_thickness": 4.0,
    "gate_closed_gap": 12.0,
    "gate_open_gap": 53.0,
    "rocking_pocket_length": 57.0,
    "rocking_eject_angle_deg": 32.0,
}


def box(extents, center=(0, 0, 0)) -> trimesh.Trimesh:
    mesh = trimesh.creation.box(extents=np.asarray(extents, dtype=float))
    mesh.apply_translation(np.asarray(center, dtype=float))
    return mesh


def cylinder_x(radius: float, length: float, center=(0, 0, 0), sections=32) -> trimesh.Trimesh:
    mesh = trimesh.creation.cylinder(radius=radius, height=length, sections=sections)
    mesh.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0]))
    mesh.apply_translation(np.asarray(center, dtype=float))
    return mesh


def can(center) -> trimesh.Trimesh:
    return cylinder_x(P["can_diameter"] / 2, P["can_length"], center, sections=40)


def sloped_sheet(width: float, run: float, thickness: float, front_z: float,
                 angle_deg: float, center_x: float = 0.0) -> trimesh.Trimesh:
    """Sheet whose horizontal projection follows +Y from the front edge."""
    angle = math.radians(angle_deg)
    length = run / math.cos(angle)
    mesh = box((width, length, thickness))
    mesh.apply_transform(trimesh.transformations.rotation_matrix(angle, [1, 0, 0]))
    mesh.apply_translation((center_x, run / 2, front_z + run * math.tan(angle_deg * math.pi / 180) / 2))
    return mesh


def side_panel(x_center: float, thickness: float = 4.0) -> trimesh.Trimesh:
    run = P["cassette_depth"]
    rise = run * math.tan(math.radians(P["lower_angle_deg"]))
    outline = Polygon([(0, 2.2), (run, rise + 2.2),
                       (run, P["cassette_height"]), (0, P["cassette_height"])])
    raw = trimesh.creation.extrude_polygon(outline, height=thickness)
    # extruded (u=depth, v=height, w=thickness) -> world (X,Y,Z)
    transform = np.array([
        [0, 0, 1, x_center - thickness / 2],
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 0, 1],
    ], dtype=float)
    raw.apply_transform(transform)
    return raw


def lane_centers() -> list[float]:
    left = -P["cassette_width"] / 2
    return [left + P["lane_clear"] / 2 + i * P["lane_pitch"] for i in range(5)]


def divider_centers() -> list[float]:
    left = -P["cassette_width"] / 2
    return [left + P["lane_clear"] + P["divider_thickness"] / 2 + i * P["lane_pitch"]
            for i in range(4)]


def support_bar(y: float, deck_z: float, height=24.0, depth=18.0) -> trimesh.Trimesh:
    return box((P["cassette_width"], depth, height),
               (0, y, deck_z - height / 2 - 2.0))


def cassette_a_parts(include_cans=False) -> list[trimesh.Trimesh]:
    """Hand-cut flat pack: dividers + cross combs + five removable strips."""
    parts: list[trimesh.Trimesh] = []
    lower_run = P["cassette_depth"]
    strip_w = P["lane_clear"] - P["strip_clearance"]
    for x in lane_centers():
        parts.append(sloped_sheet(strip_w, lower_run, 4.0, 0.0,
                                  P["lower_angle_deg"], x))
        parts.append(sloped_sheet(strip_w, P["upper_run"], 4.0,
                                  P["upper_front_height"], -P["upper_angle_deg"], x))
    for x in divider_centers():
        parts.append(side_panel(x))

    # The first rib starts only where there is enough height above the box
    # floor; the front 90 mm of the deck rests directly on the floor/lintel.
    for y in (90.0, 250.0, 410.0, 570.0):
        z = y * math.tan(math.radians(P["lower_angle_deg"]))
        parts.append(support_bar(y, z, height=min(20.0, z - 3.0)))
    for y in (12.0, 185.0, 360.0, 523.0):
        z = P["upper_front_height"] - y * math.tan(math.radians(P["upper_angle_deg"]))
        parts.append(support_bar(y, z, height=20.0, depth=16.0))

    # Two removable top combs stop the tall dividers from racking.
    parts.append(box((P["cassette_width"], 18, 24), (0, 22, 378)))
    parts.append(box((P["cassette_width"], 18, 24), (0, 568, 378)))

    if include_cans:
        for li, x in enumerate(lane_centers()):
            for y in (95, 145, 195, 245, 295):
                z = y * math.tan(math.radians(P["lower_angle_deg"])) + P["can_diameter"] / 2 + 5
                parts.append(can((x, y, z)))
            for y in (80, 130, 180, 230):
                z = (P["upper_front_height"] - y * math.tan(math.radians(P["upper_angle_deg"]))
                     + P["can_diameter"] / 2 + 4)
                parts.append(can((x, y, z)))
    return parts


def cassette_b_parts() -> list[trimesh.Trimesh]:
    """CNC option: two continuous decks, shown for comparison."""
    parts = [sloped_sheet(P["cassette_width"], P["cassette_depth"], 6.0, 0.0,
                          P["lower_angle_deg"]),
             sloped_sheet(P["cassette_width"], P["upper_run"], 6.0,
                          P["upper_front_height"], -P["upper_angle_deg"])]
    parts.extend(side_panel(x, 6.0) for x in (-117.3, -39.1, 39.1, 117.3))
    parts.append(box((P["cassette_width"], 20, 28), (0, 20, 375)))
    parts.append(box((P["cassette_width"], 20, 28), (0, 570, 375)))
    return parts


def base_gate_frame() -> list[trimesh.Trimesh]:
    return [
        box((79, 130, 4), (0, 15, -4)),
        box((5, 130, 92), (-37, 15, 42)),
        box((5, 130, 92), (37, 15, 42)),
        box((79, 10, 8), (0, -45, 80)),
        box((79, 10, 8), (0, 75, 80)),
    ]


def gate_a_parts() -> list[trimesh.Trimesh]:
    """Minimal retrofit: wide shoes on the old strict-counterphase racks."""
    parts = base_gate_frame()
    front_y, rear_y = -11.0, 41.0
    parts += [
        box((P["gate_blade_width"], 4, 52), (0, front_y, 38)),
        box((P["gate_blade_width"], 4, 52), (0, rear_y, 79)),
        box((10, 10, 92), (-8, front_y, 50)),
        box((10, 10, 92), (-8, rear_y, 68)),
        cylinder_x(26, 6, (-8, 15, 64), sections=28),
        box((75, 175, 4), (0, 0, 0)),
    ]
    return parts


def rocking_cradle_moving(angle_deg=0.0) -> list[trimesh.Trimesh]:
    L = P["rocking_pocket_length"]
    moving = [
        box((72, L, 4), (0, 0, 0)),
        box((72, 4, 24), (0, L / 2 - 2, 10)),
        box((72, 4, 15), (0, -L / 2 + 2, 5.5)),
        box((8, L, 8), (-32, 0, -1)),
        box((8, L, 8), (32, 0, -1)),
    ]
    if angle_deg:
        rot = trimesh.transformations.rotation_matrix(math.radians(angle_deg), [1, 0, 0])
        for mesh in moving:
            mesh.apply_transform(rot)
    for mesh in moving:
        mesh.apply_translation((0, 18, 12))
    return moving


def rocking_gate_parts(eject=False) -> list[trimesh.Trimesh]:
    """Additional compact idea: one-can rocking pocket."""
    parts = [
        box((79, 165, 5), (0, 2, -6)),
        box((5, 165, 66), (-37, 2, 27)),
        box((5, 165, 66), (37, 2, 27)),
        box((72, 55, 4), (0, -40, 12)),
        box((72, 55, 4), (0, 76, -3)),
        cylinder_x(4, 78, (0, 18, 12), sections=24),
        box((23, 32, 29), (0, 18, -29)),
        cylinder_x(14, 8, (0, 18, -8), sections=24),
        box((8, 36, 5), (0, 28, -6)),
    ]
    angle = -P["rocking_eject_angle_deg"] if eject else 0.0
    parts.extend(rocking_cradle_moving(angle))
    return parts


def gate_b_parts(eject=False) -> list[trimesh.Trimesh]:
    """Recommended vertical shuttle: one enclosed 72x49 mm can cell."""
    parts = [
        # Static side frame and two port ledges, 56 mm centre-to-centre.
        box((79, 82, 5), (0, 15, -34)),
        box((5, 82, 126), (-37, 15, 58)),
        box((5, 82, 126), (37, 15, 58)),
        box((72, 48, 4), (0, 39, 80)),
        box((72, 48, 4), (0, -17, -53)),
        box((23, 32, 29), (0, -32, 34)),
        cylinder_x(18, 8, (0, -12, 34), sections=28),
    ]
    # Moving carriage: cell centre is 80 HOME and 24 EJECT.
    dz = -56.0 if eject else 0.0
    carriage = [
        box((72, 60, 20), (0, 116 + dz, 15)),   # solid inlet blocker above cell
        box((72, 60, 18), (0, 44 + dz, 15)),    # solid web below cell
        box((6, 60, 110), (-33, 80 + dz, 15)),
        box((6, 60, 110), (33, 80 + dz, 15)),
        box((72, 52, 4), (0, 54.5 + dz, 15)),   # cell floor
        box((9, 12, 112), (-27, 80 + dz, 50)),  # drive rack representation
    ]
    parts.extend(carriage)
    parts.append(can((0, 15, 80 + dz)))
    # Waiting cans are static at the inlet.
    parts.append(can((0, 92, 80)))
    parts.append(can((0, 141, 80)))
    return parts


def gate_c_parts(eject=False) -> list[trimesh.Trimesh]:
    """Rotary single-cell airlock, HOME <-> 105 degrees."""
    parts = [
        # Outer 130 mm envelope and port ledges.
        cylinder_x(65, 78, (0, 0, 65), sections=48),
        box((72, 70, 4), (0, 65, 99)),
        box((72, 70, 4), (0, 8, -54)),
        box((23, 32, 29), (0, -44, 65)),
    ]
    # Visual hollowing: the STL stays a concept; the viewer shows transparency.
    angle = math.radians(105.0 if eject else 0.0)
    rotor = [
        cylinder_x(45, 6, (-33, 0, 65), sections=40),
        cylinder_x(45, 6, (33, 0, 65), sections=40),
        box((72, 52, 5), (0, 38, 65)),
        box((72, 5, 26), (0, 55, 76)),
        box((72, 5, 26), (0, 21, 76)),
    ]
    pivot = np.array((0.0, 0.0, 65.0))
    transform = (trimesh.transformations.translation_matrix(pivot)
                 @ trimesh.transformations.rotation_matrix(angle, [1, 0, 0])
                 @ trimesh.transformations.translation_matrix(-pivot))
    for item in rotor:
        item.apply_transform(transform)
    jar = can((0, 38, 65))
    jar.apply_transform(transform)
    parts.extend(rotor)
    parts.append(jar)
    parts.append(can((0, 88, 89)))
    parts.append(can((0, 137, 89)))
    return parts


def dual_gate_parts() -> list[trimesh.Trimesh]:
    """Maximum-control option: two independent full-width guillotines."""
    parts = base_gate_frame()
    for y, bottom in ((-11.0, P["gate_closed_gap"]),
                      (41.0, P["gate_open_gap"])):
        parts.append(box((72, 4, 55), (0, y, bottom + 27.5)))
        parts.append(box((8, 14, 78), (-31, y, 50)))
        parts.append(box((8, 14, 78), (31, y, 50)))
    parts += [
        box((23, 32, 29), (-21, -31, 99)),
        box((23, 32, 29), (21, 61, 99)),
        box((75, 175, 4), (0, 0, 0)),
    ]
    return parts


def translated(parts: list[trimesh.Trimesh], offset) -> list[trimesh.Trimesh]:
    result = []
    for item in parts:
        copy = item.copy()
        copy.apply_translation(offset)
        result.append(copy)
    return result


def five_lane_system_parts() -> list[trimesh.Trimesh]:
    parts = cassette_a_parts(include_cans=True)
    # Separate noses make the cassette removable upward.
    strip_w = P["lane_clear"] - P["strip_clearance"]
    for x in lane_centers():
        nose = sloped_sheet(strip_w, P["outside_nose_run"], 4.0, 0.0,
                            P["lower_angle_deg"], x)
        # Put the outside run in front of the box (negative Y).
        nose.apply_transform(trimesh.transformations.rotation_matrix(math.pi, [0, 0, 1]))
        parts.append(nose)
        parts.extend(translated(gate_b_parts(False), (x, -125, -18)))

    # Lightweight box reference frame, not a structural part.
    bw, bd, bh = P["box_inside_width"], P["box_inside_depth"], P["box_inside_height"]
    parts += [
        box((6, bd, bh), (-bw / 2 - 3, bd / 2, bh / 2)),
        box((6, bd, bh), (bw / 2 + 3, bd / 2, bh / 2)),
        box((bw, 6, bh), (0, bd + 3, bh / 2)),
        box((bw, 16, 24), (0, -8, 65)),
        box((bw, 18, 30), (0, -120, -36)),
    ]
    return parts


def export(name: str, parts: list[trimesh.Trimesh]) -> dict:
    mesh = trimesh.util.concatenate(parts)
    target = MESH_DIR / name
    mesh.export(target)
    return {
        "file": str(target.name),
        "extents_mm": [round(float(v), 2) for v in mesh.extents],
        "components": len(parts),
        "watertight": bool(mesh.is_watertight),
    }


def write_cut_csv() -> None:
    lower_len = P["cassette_depth"] / math.cos(math.radians(P["lower_angle_deg"]))
    upper_len = P["upper_run"] / math.cos(math.radians(P["upper_angle_deg"]))
    rows = [
        ("Продольная перегородка", "4", "590×390", "4 мм фанера", "Контур низа по скату; окна под гребёнки по шаблону"),
        ("Нижняя полоса", "5", f"{lower_len:.1f}×74.4", "4 мм фанера", "Горизонтальная проекция 590; угол 15°"),
        ("Верхняя полоса", "5", f"{upper_len:.1f}×74.4", "4 мм фанера", "Горизонтальная проекция 535; угол 14°"),
        ("Поперечная гребёнка нижняя", "4", "391×24", "6 мм фанера", "Пазы только после замера толщины"),
        ("Поперечная гребёнка верхняя", "4", "391×20", "6 мм фанера", "Пазы только после замера толщины"),
        ("Внешний съёмный нос", "5", "248.5×74.4", "4 мм фанера", "Проекция 240; отдельный от кассеты"),
        ("Верхняя стяжка-гребёнка", "2", "391×24", "6–9 мм фанера", "Перед+зад, съёмные клинья"),
        ("Фасадная силовая планка", "2", "395×35×9–12", "фанера/дерево", "Снаружи+изнутри, 6 болтов M4"),
        ("Общий мост под затворами", "1", "395×40×20", "дерево/профиль", "Опирается на подиум, не на коробку"),
    ]
    with (HERE / "РАСКРОЙ_НОМИНАЛ_ДО_НАТУРНОГО_ЗАМЕРА.csv").open("w", newline="", encoding="utf-8-sig") as fh:
        writer = csv.writer(fh, delimiter=";")
        writer.writerow(("Деталь", "Количество", "Размер мм", "Материал", "Примечание"))
        writer.writerows(rows)


def write_nominal_svg() -> None:
    lower_len = P["cassette_depth"] / math.cos(math.radians(P["lower_angle_deg"]))
    upper_len = P["upper_run"] / math.cos(math.radians(P["upper_angle_deg"]))
    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="1200" height="820" viewBox="0 0 1200 820">
<rect width="1200" height="820" fill="#111820"/>
<style>text{{font-family:Segoe UI,Arial;fill:#eaf1f7}} .p{{fill:#c9a36d;stroke:#ffe0a6;stroke-width:2}} .r{{fill:#73b7e8;stroke:#bfe6ff;stroke-width:2}} .n{{font-size:18px;font-weight:700}} .s{{font-size:14px;fill:#aebdca}}</style>
<text x="35" y="42" class="n">V9 — номинальный раскрой кассеты (НЕ резать до замера коробки и толщины листа)</text>
<text x="35" y="68" class="s">База: внутри 595×395×395; кассета 590×391; банка 70×Ø47; 5×75; перегородка 4.</text>
<g transform="translate(35,105)"><path class="p" d="M0 0 H590 V390 H0 Z"/><line x1="0" y1="390" x2="590" y2="232" stroke="#73b7e8" stroke-width="3"/><text x="18" y="32" class="n">Перегородка 590×390 — 4 шт</text><text x="18" y="58" class="s">голубая линия = низ по скату 15°; реальные окна гребёнок — по шаблону</text></g>
<g transform="translate(660,105)"><rect class="p" width="74.4" height="{lower_len:.1f}"/><text x="95" y="30" class="n">Низ {lower_len:.1f}×74.4 — 5 шт</text><rect x="0" y="635" class="r" width="74.4" height="{upper_len:.1f}" transform="rotate(-90 0 635)"/><text x="95" y="665" class="n">Верх {upper_len:.1f}×74.4 — 5 шт</text></g>
<g transform="translate(35,535)"><rect class="r" width="391" height="24"/><text x="0" y="55" class="n">Гребёнка 391×24 — 4+2 шт</text><rect x="0" y="85" class="r" width="391" height="20"/><text x="0" y="136" class="s">Ширина паза = фактическая толщина +0.2 CNC / +0.4 ручной рез</text></g>
<g transform="translate(500,535)"><rect class="p" width="248.5" height="74.4"/><text x="0" y="105" class="n">Съёмный нос 248.5×74.4 — 5 шт</text><text x="0" y="132" class="s">Сначала снять носы/затворы, затем кассету поднять вверх.</text></g>
<text x="35" y="785" class="s">Файл показывает габариты и объём материала, а не готовые траектории CNC. После выбора механизма будет финальный DXF/SVG.</text>
</svg>'''
    (HERE / "РАСКРОЙ_НОМИНАЛ_ДО_НАТУРНОГО_ЗАМЕРА.svg").write_text(svg, encoding="utf-8")


def main() -> None:
    reports = [
        export("КОНЦЕПТ_кассета_A_гребенки_и_полосы.stl", cassette_a_parts(True)),
        export("КОНЦЕПТ_кассета_B_цельные_листы_CNC.stl", cassette_b_parts()),
        export("КОНЦЕПТ_затвор_A_широкие_насадки_HOME.stl", gate_a_parts()),
        export("КОНЦЕПТ_затвор_B_вертикальный_шлюз_HOME.stl", gate_b_parts(False)),
        export("КОНЦЕПТ_затвор_B_вертикальный_шлюз_ВЫДАЧА.stl", gate_b_parts(True)),
        export("КОНЦЕПТ_затвор_C_роторный_шлюз_HOME.stl", gate_c_parts(False)),
        export("КОНЦЕПТ_затвор_C_роторный_шлюз_ВЫДАЧА.stl", gate_c_parts(True)),
        export("КОНЦЕПТ_затвор_D_две_независимые_гильотины.stl", dual_gate_parts()),
        export("КОНЦЕПТ_система_5_каналов_вариант_B.stl", five_lane_system_parts()),
    ]
    write_cut_csv()
    write_nominal_svg()
    payload = {"parameters_mm": P, "generated_meshes": reports,
               "warning": "CONCEPT ONLY — do not print before a one-lane fit test"}
    (HERE / "ПАРАМЕТРЫ_КОНЦЕПТОВ.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    for row in reports:
        print(f"{row['file']}: {row['extents_mm']} mm, {row['components']} bodies")


if __name__ == "__main__":
    main()
