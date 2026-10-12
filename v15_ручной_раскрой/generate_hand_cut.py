"""Integer manual layout, isolated from V14 and all printable dock files.

All manufacturing coordinates on the drawings are integers. Derived rotation
coordinates remain precise internally: rounding only the text is not sufficient.
Run with Python 3.11 and the geometry dependencies of the existing project.
"""
from __future__ import annotations

import hashlib
import html
import json
import math
import shutil
import sys
from pathlib import Path

import numpy as np
import trimesh
from scipy.optimize import brentq
from shapely.geometry import Polygon, box
from shapely.ops import unary_union

sys.stdout.reconfigure(encoding="utf-8")
ROOT = Path(__file__).resolve().parent
OLD = ROOT.parent / "v14_кассета_ПНД4"
CUT = ROOT / "cutting"
MESH = ROOT / "reference_models_not_for_printing"
FRIENDLY = ROOT.parent.parent / "dozator-cassette-v15"

# X: distance from FRONT; Z: height above BOTTOM, not from the top edge.
# A/B are the upper edge of each slot; C/D are its lower edge.
LOWER = [(24, 42), (566, 188), (567, 182), (25, 37)]
UPPER = [(24, 331), (535, 241), (535, 235), (23, 326)]
REAR_CUTS = [(109, 116), (189, 196), (269, 276), (349, 356)]
REAR_DEPTH = 195
REAR_NOTCH_X = 584

source_path = OLD / "generate_cassette.py"
ns = {"__file__": str(source_path), "__name__": "manual_layout_base"}
exec(compile(source_path.read_text(encoding="utf-8"), str(source_path), "exec"), ns)
P = ns["P"]
P.update(version="V15-integer-manual-layout", lower_length=559.0,
         upper_length=517.0, upper_z0=330.0,
         rear_slot_width=7.0, rear_slot_clearance=2.0)
P["lower_x1"] = P["lower_x0"] + P["lower_length"] * math.cos(math.radians(15))
P["upper_x1"] = P["upper_x0"] + P["upper_length"] * math.cos(math.radians(10))
P["lower_rear_z"] = ns["rz_lower"](P["lower_x1"])
P["upper_rear_z"] = ns["rz_upper"](P["upper_x1"])
P["lower_rear_gap"] = 585 - P["lower_x1"]
P["upper_rear_gap"] = 585 - P["upper_x1"]
P["manual_lower_slot_corners"] = LOWER
P["manual_upper_slot_corners"] = UPPER
P["manual_rear_slot_edges"] = REAR_CUTS
P["manual_internal_rear_notch"] = [REAR_NOTCH_X, 195, 590, 390]
P["plastic_slot_width"] = None  # integer corner polygons, NOT uniform-width slots
ns.update(ROOT=ROOT, CUT=CUT, MESH=MESH)


def wall_profile(internal):
    outline = box(0, 0, 590 if internal else 585, 390)
    cuts = [Polygon(LOWER), Polygon(UPPER)]
    if internal:
        cuts.append(box(REAR_NOTCH_X, 195, 591, 391))
    return outline.difference(unary_union(cuts))


def rear_profile():
    cuts = [box(a - 30, -1, b - 30, 195) for a, b in REAR_CUTS]
    return box(0, 0, 405, 390).difference(unary_union(cuts))


ns.update(wall_profile=wall_profile, rear_profile=rear_profile)
parts, side_profile, internal_profile = ns["build_parts"]()
for item in parts.values():
    item["mesh"].fix_normals()
for i in range(1, 7):
    parts[f"wall_{i}"]["note"] = (
        f"{'590' if 1 < i < 6 else '585'}×390×5 мм. Пазы по целым координатам A–D; "
        + ("сзади полупаз 6×195 мм." if 1 < i < 6 else "передняя и задняя кромки целые."))
parts["lower"]["note"] = "559×405×4 мм. Наклон 15°. Передний верх листа: X=25, Z=42 мм — без изменения стыка с доком."
parts["upper"]["note"] = "517×405×4 мм. Наклон 10°. Передний верх: X=25, Z=330 мм. Задний проход около 51 мм."
parts["rear"]["note"] = "Заготовка 465×390 мм; сгибы 30 и 435 мм; четыре паза шириной 7 мм, глубиной 195 мм."


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def overlap(a, b):
    if np.any(a.bounds[1] <= b.bounds[0]) or np.any(b.bounds[1] <= a.bounds[0]):
        return 0.0
    # Boolean errors must fail the check, never masquerade as a zero volume.
    result = trimesh.boolean.intersection([a, b], engine="manifold")
    return float(result.volume) if result is not None and len(result.faces) else 0.0


def sheet_section(name):
    m = parts[name]["mesh"]
    return Polygon([(v[0], v[2]) for v in m.vertices]).convex_hull


def check_geometry(check_dock=True):
    watched = [source_path, OLD / "make_dock_full_3d.py"]
    watched += sorted((OLD / "ПЕЧАТЬ_ДОКА").glob("*.stl"))
    before = {str(p): digest(p) for p in watched}
    result = {"version": P["version"], "nominal_material_mm": {"cardboard": 5, "HDPE": 4},
              "slot_vertices_integer": all(isinstance(v, int) for poly in (LOWER, UPPER) for pt in poly for v in pt),
              "sheet_cut_dimensions_mm": {"lower": [559, 405, 4], "upper": [517, 405, 4]},
              "sheet_fits": {}, "part_intersections_mm3": {}, "dock_intersections_mm3": {},
              "original_file_hashes": before}
    assert side_profile.geom_type == "Polygon" and internal_profile.geom_type == "Polygon"
    assert len(side_profile.interiors) == len(internal_profile.interiors) == 2
    for name, points in (("lower", LOWER), ("upper", UPPER)):
        slot = Polygon(points)
        section = sheet_section(name)
        assert slot.covers(section), f"{name}: sheet does not fit integer slot"
        clearance = section.boundary.distance(slot.boundary)
        assert clearance > .15, (name, clearance)
        result["sheet_fits"][name] = {"contained": True, "minimum_geometric_clearance_mm": clearance}
    for k, item in parts.items():
        assert item["mesh"].is_watertight, k
    for i in range(1, 7):
        for name in ("lower", "upper", "rear"):
            value = overlap(parts[f"wall_{i}"]["mesh"], parts[name]["mesh"])
            assert value < .01, (i, name, value)
            result["part_intersections_mm3"][f"wall_{i}_{name}"] = value
    result["closed_front_ligament_min_mm"] = min(v[0] for poly in (LOWER, UPPER) for v in poly)
    result["rear_passage_mm"] = 585 - P["upper_x1"]
    result["lower_front_running_surface_mm"] = [25, 42]
    result["wall_near_faces_mm"] = [0, 80, 160, 240, 320, 400]
    result["rear_half_lap_side_clearance_mm"] = 1
    result["rear_notch_depth_clearance_mm"] = 1
    # Shared edge/zero nominal overlap does not prove tolerance to sloppy cutting.
    if check_dock:
        dock = {}
        dock_path = OLD / "make_dock_full_3d.py"
        src = dock_path.read_text(encoding="utf-8")
        marker = "# ============ экспорт"
        assert marker in src
        exec(compile(src.split(marker)[0], str(dock_path), "exec"), dock)
        obstacles = {"dock_" + str(i + 1): v for i, v in enumerate(dock["MONO"])}
        obstacles.update({"cheek_" + k: v for k, v in dock["CHK"].items()})
        transformed = {k: dock["rot_cas"](v["mesh"]) for k, v in parts.items()}
        for k, a in transformed.items():
            value = sum(overlap(a, b) for b in obstacles.values())
            assert value < .02, ("cassette/dock", k, value)
            result["dock_intersections_mm3"][k] = value

        # Static packing only. Does not erase the rear feeding risk from the audit.
        R = 23.5
        first = (dock["mbase"](dock["YCS"][2]) @ np.array([dock["uc1"], 0, R, 1]))[[0, 2]]
        lower = []
        for j in range(40):
            pt = first + j * 47 * np.array([-dock["C15"], dock["S15"]])
            if pt[0] < -585 + R - 1e-6:
                break
            lower.append(pt)
        previous = lower[-1]
        tx = -585 + R
        transition = np.array([tx, previous[1] + math.sqrt(47**2 - (tx - previous[0])**2)])
        angle = math.radians(10)

        def upper_z(x):
            return 330 + (x + 25) * math.tan(angle) + R / math.cos(angle)

        ux = brentq(lambda x: (x - tx)**2 + (upper_z(x) - transition[1])**2 - 47**2,
                    tx + .01, tx + 57)
        upper = []
        for j in range(40):
            x = ux + j * 47 * math.cos(angle)
            if x + R * math.sin(angle) > -25 + 1e-6:
                break
            upper.append(np.array([x, upper_z(x)]))
        positions = lower + [transition] + upper
        assert len(positions) >= 25, len(positions)
        hits = []
        jar_template = trimesh.creation.cylinder(radius=R, height=67, sections=96)
        jar_template.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0]))
        jar_obstacles = {**transformed, **obstacles}
        for lane, yc in enumerate(dock["YCS"]):
            for j, (x, z) in enumerate(positions[:25]):
                jar = jar_template.copy()
                jar.apply_translation([x, yc, z])
                for k, ob in jar_obstacles.items():
                    value = overlap(jar, ob)
                    if value > .1:
                        hits.append({"lane": lane + 1, "jar": j + 1, "part": k, "mm3": value})
        assert not hits, hits
        result["static_125_jars"] = {"checked": 125, "diameter_mm": 47, "length_mm": 67,
                                     "capacity_candidate_per_lane": len(positions),
                                     "lower": len(lower), "transition": 1, "upper": len(upper),
                                     "structure_collisions": hits,
                                     "scope": "nominal static upright geometry; not a dynamic feeding or strength certification"}
    assert before == {str(p): digest(p) for p in watched}, "Original files unexpectedly changed"
    result["original_files_unchanged"] = True
    return result


def profile_svg(profile, w, h, title, slot_points=False, rear=False):
    margin = 40
    d = ns["path_d"](profile, h)
    marks = []
    if slot_points:
        for name, points, color in (("Н", LOWER, "#166b9d"), ("В", UPPER, "#7957bc")):
            for letter, (x, z) in zip("ABCD", points):
                y = h - z
                # The two close corners at each end use separate callout baselines.
                dx = 10 if x < 100 else -87
                dy = -14 if letter in "AB" else 16
                if name == "Н" and letter == "C":
                    dy = 35  # keep the lower rear callout clear of the uphill slot
                if name == "Н" and letter == "A":
                    dy = -40
                if name == "В" and letter == "D":
                    dy = 40
                if name == "В" and letter == "B":
                    dy = -35
                marks.append(f'<circle cx="{x}" cy="{y}" r="2" fill="{color}"/>'
                             f'<line x1="{x}" y1="{y}" x2="{x+dx}" y2="{y+dy}" stroke="{color}" stroke-width=".5"/>'
                             f'<rect x="{x+dx-2}" y="{y+dy-7}" width="86" height="10" fill="white" fill-opacity=".92" rx="2"/>'
                             f'<text x="{x+dx}" y="{y+dy}" fill="{color}" class="point">{name}-{letter}: {x} / {z}</text>')
    if rear:
        for x in (30, 435):
            marks.append(f'<line x1="{x}" y1="0" x2="{x}" y2="390" class="fold"/>'
                         f'<text x="{x+3}" y="16" class="point" fill="#1677aa">Сгиб {x}</text>')
        for j, (a, b) in enumerate(REAR_CUTS):
            marks.append(f'<text x="{a-13}" y="{h-REAR_DEPTH-10}" class="point">{a}–{b}</text>')
        marks.append('<text x="370" y="280" class="point">Глубина пазов 195</text>')
    if w == 590 and slot_points:
        marks.append('<text x="480" y="20" class="point">Полупаз сзади: 6×195</text>')
    extras = "".join(marks)
    origin_caption = "0: левый край / низ" if rear else "0: перед / низ"
    base_caption = "X от левого края заготовки; Z от низа" if rear else "X от переднего края; Z от низа"
    return f'''<svg xmlns="http://www.w3.org/2000/svg" width="{w+80}mm" height="{h+95}mm" viewBox="0 0 {w+80} {h+95}" role="img" aria-label="{html.escape(title)}">
<style>.outline{{fill:#f4ead6;stroke:#c33432;stroke-width:.6;fill-rule:evenodd}}.point{{font:6px Arial,sans-serif;font-weight:700}}.label{{font:bold 8px Arial,sans-serif}}.dimension{{stroke:#343c46;stroke-width:.5}}.fold{{stroke:#1677aa;stroke-width:.5;stroke-dasharray:5 3}}</style>
<rect width="100%" height="100%" fill="white"/>
<text x="40" y="15" class="label">{html.escape(title)}</text>
<g transform="translate({margin},{margin})"><path d="{d}" class="outline"/>{extras}
<line x1="0" y1="{h+17}" x2="{w}" y2="{h+17}" class="dimension"/>
<line x1="0" y1="{h+10}" x2="0" y2="{h+22}" class="dimension"/><line x1="{w}" y1="{h+10}" x2="{w}" y2="{h+22}" class="dimension"/>
<text x="{w//2-15}" y="{h+30}" class="label">{w} мм</text>
<line x1="-17" y1="0" x2="-17" y2="{h}" class="dimension"/>
<text x="-22" y="{h//2}" class="label" transform="rotate(-90,-22,{h//2})">{h} мм</text>
<circle cx="0" cy="{h}" r="2" fill="#c33432"/><text x="6" y="{h-8}" class="point">{origin_caption}</text>
</g><text x="40" y="{h+86}" class="point">Красная линия — рез. {base_caption}. Все числа в миллиметрах.</text></svg>'''


def write_viewer():
    ns["build_html"](parts)
    legacy = ROOT / "ОТКРОЙ_V13_КАССЕТУ_3D.html"
    content = legacy.read_text(encoding="utf-8")
    content = content.replace("V13 · Открытая кассета", "V15 · Ручной раскрой")
    content = content.replace("банка 70×Ø47", "банка 67×Ø47")
    content = content.replace("toFixed(2)", "toFixed(0)")
    content = content.replace(f"{P['lower_length']:.2f} /", "559 /")
    content = content.replace(f"{P['upper_length']:.2f} /", "517 /")
    content = content.replace("Задний зазор верху", "Проход до задней стенки ≈")
    # Total rendered jar length and diameter match the nominal 67 x 47 envelope.
    content = content.replace("P.can_length,40),canMat", "P.can_length-2.6,40),canMat")
    content = content.replace("P.can_diameter_nominal/2+0.7", "P.can_diameter_nominal/2")
    content = content.replace("P.can_length/2+.65", "P.can_length/2-.65")
    content = content.replace("До резки измерить фактическую толщину.",
                              "До резки измерить фактическую толщину. 35 банок в этом виде — иллюстрация, не полная загрузка. "
                              "Задний переход требует физического теста с 25 банками на канал.")
    content = content.replace("Нажми на деталь в 3D.", "Нажми на деталь в 3D. Чертёж с целыми координатами — в index.html.")
    (ROOT / "viewer.html").write_text(content, encoding="utf-8")
    # Retain the generated original-named HTML as an alias rather than delete it.
    legacy.write_text(content, encoding="utf-8")


def write_guide():
    rear_blank = box(0, 0, 465, 390).difference(unary_union([box(a, -1, b, 195) for a, b in REAR_CUTS]))
    specs = [
        ("side", "01_side_585x390_2pcs.svg", side_profile, 585, 390, "Боковая стенка: 585×390 мм · 2 шт.", True, False),
        ("inner", "02_divider_590x390_4pcs.svg", internal_profile, 590, 390, "Внутренняя перегородка: 590×390 мм · 4 шт.", True, False),
        ("rear", "05_rear_465x390_1pc.svg", rear_blank, 465, 390, "Задняя стенка: 465×390 мм · 1 шт.", False, True),
        ("lower", "03_lower_559x405_1pc.svg", box(0, 0, 559, 405), 559, 405, "Нижний поддон ПНД: 559×405×4 мм · 1 шт.", False, False),
        ("upper", "04_upper_517x405_1pc.svg", box(0, 0, 517, 405), 517, 405, "Верхний поддон ПНД: 517×405×4 мм · 1 шт.", False, False),
    ]
    diagrams = {}
    CUT.mkdir(exist_ok=True)
    for key, filename, prof, w, h, title, points, rear in specs:
        svg = profile_svg(prof, w, h, title, points, rear)
        (CUT / filename).write_text(svg, encoding="utf-8")
        diagrams[key] = {"filename": filename, "title": title, "svg": svg}
    rows = "".join(f'<tr><td>{name}-{letter}</td><td>{x}</td><td>{z}</td></tr>'
                   for name, points in (("Н", LOWER), ("В", UPPER))
                   for letter, (x, z) in zip("ABCD", points))
    buttons = "".join(f'<button data-part="{key}">{d["title"].split(":")[0]}</button>' for key, d in diagrams.items())
    downloads = "".join(f'<a download href="cutting/{d["filename"]}">{d["title"]} — SVG 1:1</a>' for d in diagrams.values())
    guide = '''<!doctype html><html lang="ru"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>V15 — кассета: удобный ручной раскрой</title><style>
*{box-sizing:border-box}body{margin:0;color:#1d2936;background:#f0f3f7;font:16px/1.5 Segoe UI,Arial,sans-serif}header{background:#172a3b;color:white;padding:22px max(24px,calc((100vw - 1240px)/2))}h1{font-size:27px;margin:0 0 6px}h2{font-size:21px;margin:0 0 14px}h3{font-size:17px}p{margin:8px 0 14px}main{max-width:1290px;margin:auto;padding:24px;display:grid;gap:22px}.card{background:white;border:1px solid #d6dfe8;border-radius:14px;padding:24px}.intro{display:grid;grid-template-columns:1.1fr 1fr;gap:25px}.chips{display:flex;flex-wrap:wrap;gap:8px}.chip{background:#eaf4fa;border:1px solid #c4dfed;border-radius:7px;padding:6px 11px;font-weight:650}.notice{padding:13px 17px;background:#fff7df;border-left:4px solid #d89b24;border-radius:6px}.ok{background:#e9f6ed;border-color:#2b8d52}table{border-collapse:collapse;width:100%;margin:12px 0}th,td{padding:9px 12px;border-bottom:1px solid #dfe6ed;text-align:left}th{background:#edf3f8}td.num{font-weight:750;font-variant-numeric:tabular-nums}.layout{display:grid;grid-template-columns:minmax(0,1fr) 270px;gap:24px}.toolbar{display:flex;flex-wrap:wrap;gap:8px;margin-bottom:15px}button,.button{border:1px solid #a6bdcc;border-radius:8px;background:#edf4f9;color:#17364d;font:600 15px Segoe UI;padding:10px 16px;cursor:pointer;text-decoration:none}button.active{background:#176994;color:white;border-color:#176994}button:hover,.button:hover{filter:brightness(.94)}#diagram{width:100%;overflow:auto;background:white;border:1px solid #dfe6ed;border-radius:8px}#diagram svg{display:block;width:100%;height:auto;min-width:520px}.small{font-size:14px;color:#50677b}.links{display:grid;gap:9px}a{color:#106b9b}iframe{display:block;border:0;border-radius:10px;width:100%;height:740px;background:#09121b}ol{padding-left:24px}li{margin:10px 0}.point-table{font-size:14px}.point-table th,.point-table td{padding:7px 8px}.caption{font-size:14px;color:#425a70}.counter{white-space:nowrap}details>summary{cursor:pointer;font-weight:700}footer{padding:15px 0;color:#60788b}.model-hidden{display:none}@media(max-width:850px){.intro,.layout{grid-template-columns:1fr}main{padding:12px}.card{padding:17px}h1{font-size:23px}iframe{height:820px}}@media print{header{background:none;color:black;padding:10px}.toolbar,.button,iframe,#model,.no-print{display:none}main{display:block;padding:0}.card{break-inside:avoid;border:0;padding:12px}.layout,.intro{display:block}#diagram svg{min-width:0}a{color:black}.notice{background:none}}
</style></head><body><header><h1>V15 · Кассета — раскрой целыми миллиметрами</h1><div>Одна кассета · 5 каналов · 6 продольных стенок + 2 поддона + задняя стенка</div></header><main>
<section class="card intro"><div><h2>Размеры, по которым можно резать</h2><div class="chips"><span class="chip">Нижний поддон 559×405</span><span class="chip">Верхний поддон 517×405</span><span class="chip">Корпус 590×405×390</span><span class="chip">Шаг 80</span></div><p>Все числа для ручной разметки — целые миллиметры. Не нужно ловить 559,05 или вычислять угол транспортиром: точки пазов уже пересчитаны под новые заготовки.</p><p class="small">Точные тригонометрические координаты внутри 3D могут оставаться дробными — они не являются размерами для резки.</p></div><div><div class="notice ok">Стык с уже печатающимся доком сохранён: передний верх нижнего листа — 25 мм от переднего края и 42 мм от низа, наклон 15°. Шаг стенок — 80 мм. Печатные файлы дока не менялись.</div><p class="small">Номинал: ПНД 4 мм, картон 5 мм. Если реальные толщины отличаются, сначала примерить обрезки в пазах. Не передавливать картон.</p></div></section>
<section class="card"><h2>1. Заготовки на одну кассету</h2><table><thead><tr><th>Деталь</th><th>Размер, мм</th><th>Количество</th><th>Что вырезать внутри</th></tr></thead><tbody>
<tr><td>Боковые стенки</td><td class="num">585×390</td><td>2</td><td>Два закрытых наклонных паза</td></tr>
<tr><td>Внутренние перегородки</td><td class="num">590×390</td><td>4</td><td>Те же два паза + задний полупаз 6×195</td></tr>
<tr><td>Нижний поддон, ПНД 4 мм</td><td class="num">559×405</td><td>1</td><td>Простой прямоугольник, без вырезов</td></tr>
<tr><td>Верхний поддон, ПНД 4 мм</td><td class="num">517×405</td><td>1</td><td>Простой прямоугольник, без вырезов</td></tr>
<tr><td>Задняя стенка</td><td class="num">465×390</td><td>1</td><td>Четыре полупаза 7×195; сгибы 30 и 435</td></tr></tbody></table>
<p class="small">Размеры стенок — заготовки из листового материала, не детали для 3D-печати. Поддоны не требуется делить на пять полос: каждый лист цельный и проходит через все шесть стенок.</p></section>
<section class="card"><h2>2. Разметка без дробей</h2><div class="toolbar">__BUTTONS__</div><div class="layout"><div><div id="diagram"></div><p class="caption">Чертёж на экране уменьшен. Координаты вида «24 / 42» означают: 24 мм от переднего края, 42 мм от низа. Все измерения от одной базы — не прибавляй их последовательно.</p><a id="active-download" class="button" download>Скачать выбранный чертёж SVG 1:1</a></div>
<aside><h3>Точки наклонных пазов</h3><table class="point-table"><thead><tr><th>Точка</th><th>X от переда</th><th>Z от низа</th></tr></thead><tbody>__ROWS__</tbody></table><p class="small">Н — нижний паз, В — верхний. В каждой группе соединить A → B → C → D → A. Внутренность получившегося четырёхугольника удалить.</p><div class="notice">Пазы слегка отличаются по ширине вдоль длины — это намеренно. Не заменяй их произвольной полосой «5 мм»: используй четыре указанные точки.</div></aside></div>
<h3>Внутренняя перегородка: дополнительный полупаз сзади</h3><p>От верхнего края вниз на <b>195 мм</b>. Полоса между <b>X=584 и X=590</b>, то есть ширина <b>6 мм</b>. Вырез открыт только сверху и с заднего торца. Остальные четыре перегородки одинаковые. На двух наружных стенках этого выреза нет.</p>
<h3>Задняя стенка: четыре паза снизу</h3><p>База здесь другая: X измеряется от <b>левого края всей заготовки 465 мм</b>. Вырезы идут от низа вверх на <b>195 мм</b>.</p><table><thead><tr><th>Паз</th><th>Левый край X</th><th>Правый край X</th><th>Ширина</th><th>Глубина</th></tr></thead><tbody><tr><td>1</td><td>109</td><td>116</td><td>7</td><td>195</td></tr><tr><td>2</td><td>189</td><td>196</td><td>7</td><td>195</td></tr><tr><td>3</td><td>269</td><td>276</td><td>7</td><td>195</td></tr><tr><td>4</td><td>349</td><td>356</td><td>7</td><td>195</td></tr></tbody></table>
<p>Линии сгиба — <b>30 и 435 мм</b> от левого края. Между ними 405 мм, по краям крылья 30 мм. Сгибать, не прорезая насквозь. Паз 7 мм оставляет по 1 мм посадочного зазора для картона 5 мм; точную позицию каналов задаёт также передний док.</p>
</section>
<section class="card"><h2>3. Как собрать</h2><ol>
<li>Сначала вырежи <b>одну боковину и одну внутреннюю перегородку</b>. Примерь оба ПНД-листа в их пазы. Они должны проходить без силового продавливания. Остальные стенки размечай по одной проверенной детали.</li>
<li>Нанижи все шесть стенок на два цельных листа сбоку. Гофру картона желательно направить вертикально. Передние и задние края стенок остаются цельными.</li>
<li>Поставь на ровную опору. Края стенок с одной стороны должны оказаться на отметках <b>0, 80, 160, 240, 320, 400 мм</b> по ширине. Это <b>края одной стороны</b> стенок, не их центральные оси.</li>
<li>Установи заднюю стенку: её пазы снизу соединяются с пазами сверху внутренних перегородок. Загни два крыла вперёд. Зафиксируй имеющимися клипсами; для первого теста допустим скотч снаружи.</li>
<li>Посади передние края стенок в уже напечатанный док. Проверь низ нижнего листа и высоту его <b>верхней рабочей поверхности 42 мм</b> у переднего торца. Она должна плавно переходить на печатный скат, без выступа, щели и возможности банки задеть край. Посадочный зазор пазов не заменяет проверку фактического стыка.</li>
<li>Загрузи <b>25 наполненных банок в один полный двухэтажный канал</b> и проверь выдачу, задний переход, последнюю банку и удержание очереди без команд. Только затем повторяй на остальных четырёх каналах.</li></ol>
<div class="notice">Округление не исправляет ранее найденный риск зависания банок у заднего перехода. Геометрическая проверка подтверждает размещение 125 номинальных банок в сборке «кассета + док», но не надёжность подачи, прочность картона или безопасность отключённой сервы. Все 7,5 кг банок плюс конструкция должны иметь устойчивую опору.</div></section>
<section class="card" id="model"><h2>4. Обновлённая 3D-модель</h2><p>Реальные формы новых пазов, поддоны 559 и 517 мм. Вращение мышью, масштаб колёсиком. Камера сама не перемещается. В 3D показано 35 банок для видимости деталей, не вся загрузка.</p><button id="load-model">Показать 3D</button> <a href="viewer.html" target="_blank" class="button">Открыть 3D отдельным окном</a><iframe id="model-frame" title="Кассета V15" class="model-hidden" data-src="viewer.html"></iframe></section>
<section class="card"><h2>5. Файлы</h2><div class="links">__DOWNLOADS__<a download href="parameters.json">Все параметры, включая точную внутреннюю геометрию — JSON</a><a download href="verification.json">Результаты геометрических проверок — JSON</a></div><p class="small">SVG сделаны в физическом масштабе 1:1. Если печатаешь бумажный шаблон, нужен режим «постер/плитка», масштаб 100%, без «вписать в страницу». После печати измерь контрольный габарит линейкой. Обычная печать этого сайта не является шаблоном 1:1.</p></section>
<footer>Отдельная версия V15. Исходная V14 и все STL дозатора оставлены без изменений.</footer></main>
<script>const diagrams=__DIAGRAMS__;function choosePart(key){const d=diagrams[key];document.getElementById('diagram').innerHTML=d.svg;const a=document.getElementById('active-download');a.href='cutting/'+d.filename;document.querySelectorAll('[data-part]').forEach(b=>b.classList.toggle('active',b.dataset.part===key));const caption=document.querySelector('#diagram + .caption');caption.textContent=key==='rear'?'Чертёж уменьшен. X — от левого края всей заготовки 465 мм, Z — от низа. Все измерения от одной базы.':'Чертёж на экране уменьшен. Координаты вида «24 / 42» означают: 24 мм от переднего края, 42 мм от низа. Все измерения от одной базы — не прибавляй их последовательно.';}document.querySelectorAll('[data-part]').forEach(b=>b.onclick=()=>choosePart(b.dataset.part));choosePart('side');document.getElementById('load-model').onclick=function(){const f=document.getElementById('model-frame');f.src=f.dataset.src;f.classList.remove('model-hidden');this.style.display='none';};</script></body></html>'''
    guide = guide.replace("__BUTTONS__", buttons).replace("__ROWS__", rows)
    guide = guide.replace("__DOWNLOADS__", downloads).replace("__DIAGRAMS__", json.dumps(diagrams, ensure_ascii=False))
    (ROOT / "index.html").write_text(guide, encoding="utf-8")


def write_meshes():
    MESH.mkdir(exist_ok=True)
    # Reference geometry only, deliberately not in the dock's print folder.
    for k, item in parts.items():
        item["mesh"].export(MESH / f"{k}.stl")
    scene = trimesh.Scene({k: v["mesh"] for k, v in parts.items()})
    (MESH / "cassette_assembly.glb").write_bytes(scene.export(file_type="glb"))


def main():
    checks = check_geometry(check_dock="--skip-dock" not in sys.argv)
    (ROOT / "verification.json").write_text(json.dumps(checks, ensure_ascii=False, indent=2), encoding="utf-8")
    (ROOT / "parameters.json").write_text(json.dumps(P, ensure_ascii=False, indent=2), encoding="utf-8")
    write_meshes()
    write_viewer()
    write_guide()
    # ASCII alias for reliable links in the chat/browser. No original files are moved.
    FRIENDLY.mkdir(exist_ok=True)
    for name in ("index.html", "viewer.html", "parameters.json", "verification.json"):
        shutil.copy2(ROOT / name, FRIENDLY / name)
    shutil.copytree(CUT, FRIENDLY / "cutting", dirs_exist_ok=True)
    print(json.dumps({"output": str(ROOT), "open": str(FRIENDLY / "index.html"),
                      "slot_fit": checks["sheet_fits"],
                      "static_capacity": checks.get("static_125_jars"),
                      "original_files_unchanged": checks["original_files_unchanged"]}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
