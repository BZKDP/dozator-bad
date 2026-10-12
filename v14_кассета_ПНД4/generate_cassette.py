from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import numpy as np
import trimesh
from shapely.geometry import LineString, Polygon, box as sbox
from shapely.ops import unary_union


ROOT = Path(__file__).resolve().parent
PROJECT = ROOT.parent
THREE_PATH = PROJECT / "libs" / "three.module.js"
CUT = ROOT / "РАСКРОЙ"
MESH = ROOT / "3D_МОДЕЛИ"


P = {
    "version": "V14-HDPE4-cassette",
    "can_length": 67.0,
    "can_diameter_nominal": 47.0,
    "can_diameter_check": 49.0,
    "lane_count": 5,
    "lane_clear": 75.0,
    "lane_pitch": 80.0,
    "cardboard_thickness": 5.0,
    "plastic_thickness": 4.0,
    "plastic_slot_clearance": 0.6,
    "rear_slot_clearance": 0.6,
    "slot_end_clearance": 0.8,
    "depth": 590.0,
    "body_width": 405.0,
    "height": 390.0,
    "rear_wing": 30.0,
    "rear_half_lap_height": 195.0,
    "lower_x0": 25.0,
    "lower_x1": 565.0,
    "lower_z0": 42.0,
    "lower_angle": 15.0,
    "upper_x0": 25.0,
    "upper_x1": 534.0,
    "upper_z0": 329.75,
    "upper_angle": 10.0,
    "support_tongue_min": 30.0,
}


def rz_lower(x: float) -> float:
    return P["lower_z0"] + (x - P["lower_x0"]) * math.tan(math.radians(P["lower_angle"]))


def rz_upper(x: float) -> float:
    return P["upper_z0"] - (x - P["upper_x0"]) * math.tan(math.radians(P["upper_angle"]))


P["lower_length"] = (P["lower_x1"] - P["lower_x0"]) / math.cos(math.radians(P["lower_angle"]))
P["upper_length"] = (P["upper_x1"] - P["upper_x0"]) / math.cos(math.radians(P["upper_angle"]))
P["lower_rear_z"] = rz_lower(P["lower_x1"])
P["upper_rear_z"] = rz_upper(P["upper_x1"])
P["upper_rear_gap"] = P["depth"] - P["upper_x1"]
P["lower_rear_gap"] = P["depth"] - P["lower_x1"]
P["rear_blank_width"] = P["body_width"] + 2 * P["rear_wing"]
P["plastic_slot_width"] = P["plastic_thickness"] + P["plastic_slot_clearance"]
P["rear_slot_width"] = P["cardboard_thickness"] + P["rear_slot_clearance"]

WALL_CENTRES = [P["cardboard_thickness"] / 2 + i * P["lane_pitch"] for i in range(6)]
INNER_CENTRES = WALL_CENTRES[1:-1]


COLORS = {
    "side": "#c99552",
    "divider": "#d6ae72",
    "rear": "#b67b3b",
    "lower": "#68bfe3",
    "upper": "#7ed6f2",
    "clip": "#f28b35",
}


def extrude_xz(profile, thickness: float, y0: float) -> trimesh.Trimesh:
    mesh = trimesh.creation.extrude_polygon(profile, height=thickness)
    matrix = np.array([
        [1, 0, 0, 0],
        [0, 0, 1, y0],
        [0, 1, 0, 0],
        [0, 0, 0, 1],
    ], dtype=float)
    mesh.apply_transform(matrix)
    return mesh


def extrude_yz(profile, thickness: float, x0: float) -> trimesh.Trimesh:
    mesh = trimesh.creation.extrude_polygon(profile, height=thickness)
    matrix = np.array([
        [0, 0, 1, x0],
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 0, 1],
    ], dtype=float)
    mesh.apply_transform(matrix)
    return mesh


def wall_profile(internal: bool) -> Polygon:
    # Outer walls butt against the rear panel. Internal dividers enter it
    # through complementary half-laps.
    wall_depth = P["depth"] if internal else P["depth"] - P["cardboard_thickness"]
    outline = sbox(0, 0, wall_depth, P["height"])
    sw = P["plastic_slot_width"]
    ta = math.radians(P["lower_angle"])
    ua = math.radians(P["upper_angle"])

    # Slot centre lines follow the sheet mid-planes. The running surface is
    # the top face, while a real 2 mm sheet occupies the space below it.
    def lower_mid(x):
        return (x + math.sin(ta) * P["plastic_thickness"] / 2,
                rz_lower(x) - math.cos(ta) * P["plastic_thickness"] / 2)

    def upper_mid(x):
        return (x - math.sin(ua) * P["plastic_thickness"] / 2,
                rz_upper(x) - math.cos(ua) * P["plastic_thickness"] / 2)

    lower_dx = P["slot_end_clearance"] * math.cos(ta)
    upper_dx = P["slot_end_clearance"] * math.cos(ua)
    lower = LineString([
        lower_mid(P["lower_x0"] - lower_dx),
        lower_mid(P["lower_x1"] + lower_dx),
    ]).buffer(sw / 2, cap_style=2, join_style=2)
    upper = LineString([
        upper_mid(P["upper_x0"] - upper_dx),
        upper_mid(P["upper_x1"] + upper_dx),
    ]).buffer(sw / 2, cap_style=2, join_style=2)
    cuts = [lower, upper]
    if internal:
        cuts.append(sbox(
            P["depth"] - P["rear_slot_width"],
            P["rear_half_lap_height"],
            P["depth"] + 1,
            P["height"] + 1,
        ))
    return outline.difference(unary_union(cuts))


def rear_profile() -> Polygon:
    outline = sbox(0, 0, P["body_width"], P["height"])
    slots = [
        sbox(c - P["rear_slot_width"] / 2, -1,
             c + P["rear_slot_width"] / 2, P["rear_half_lap_height"])
        for c in INNER_CENTRES
    ]
    return outline.difference(unary_union(slots))


def sloped_sheet(length: float, width: float, thickness: float,
                 angle_deg: float, origin: tuple[float, float, float]) -> trimesh.Trimesh:
    mesh = trimesh.creation.box(extents=[length, width, thickness])
    mesh.apply_translation([length / 2, width / 2, -thickness / 2])
    matrix = trimesh.transformations.rotation_matrix(math.radians(angle_deg), [0, 1, 0])
    mesh.apply_transform(matrix)
    mesh.apply_translation(origin)
    return mesh


def clip_mesh() -> trimesh.Trimesh:
    # U-clip for two adjacent 5 mm cardboard layers. It slides over their top edge.
    length = 22.0
    outside = 15.0
    opening = 10.8
    height = 13.0
    bridge = 2.4
    outer = sbox(-outside / 2, 0, outside / 2, height)
    # The inner rectangle deliberately continues below Z=0, leaving the U open.
    inner = sbox(-opening / 2, -1, opening / 2, height - bridge)
    return extrude_yz(outer.difference(inner), length, 0)


def color_mesh(mesh: trimesh.Trimesh, color: str) -> None:
    rgb = tuple(int(color[i:i+2], 16) for i in (1, 3, 5)) + (255,)
    mesh.visual.face_colors = rgb


def build_parts():
    parts = {}
    side_prof = wall_profile(False)
    inner_prof = wall_profile(True)
    for i, centre in enumerate(WALL_CENTRES):
        internal = 0 < i < 5
        mesh = extrude_xz(inner_prof if internal else side_prof,
                          P["cardboard_thickness"], centre - P["cardboard_thickness"] / 2)
        key = f"wall_{i+1}"
        parts[key] = {
            "mesh": mesh,
            "kind": "divider" if internal else "side",
            "title": f"Перегородка {i+1}" if internal else ("Левая стенка" if i == 0 else "Правая стенка"),
            "color": COLORS["divider" if internal else "side"],
            "explode": [0, (i - 2.5) * 24, 12 if internal else 0],
            "step": 1 if i == 0 else 2,
            "note": ("590×390×5 мм; два закрытых паза + задний полупаз."
                     if internal else
                     "585×390×5 мм; два закрытых паза, цельные передняя и задняя кромки.")
        }
    rear = extrude_yz(rear_profile(), P["cardboard_thickness"], P["depth"] - P["cardboard_thickness"])
    parts["rear"] = {
        "mesh": rear, "kind": "rear", "title": "Задняя стенка",
        "color": COLORS["rear"], "explode": [70, 0, 0], "step": 3,
        "note": "405×390×5 мм в сборке; заготовка 465×390 мм с крыльями по 30 мм."
    }
    # Folded rear wings lie outside the two outer walls.
    for key, y0 in (("rear_wing_left", -P["cardboard_thickness"]),
                    ("rear_wing_right", P["body_width"])):
        wing = trimesh.creation.box([P["rear_wing"], P["cardboard_thickness"], P["height"]])
        wing.apply_translation([
            P["depth"] - P["rear_wing"] / 2,
            y0 + P["cardboard_thickness"] / 2,
            P["height"] / 2,
        ])
        parts[key] = {
            "mesh": wing, "kind": "rear", "title": "Боковое крыло задней стенки",
            "color": COLORS["rear"], "explode": [70, -12 if "left" in key else 12, 0], "step": 3,
            "note": "Крыло 30 мм; обхватывает внешнюю стенку."
        }
    lower = sloped_sheet(P["lower_length"], P["body_width"], P["plastic_thickness"],
                         -P["lower_angle"], (P["lower_x0"], 0, P["lower_z0"]))
    parts["lower"] = {
        "mesh": lower, "kind": "lower", "title": "Нижний поддон",
        "color": COLORS["lower"], "explode": [0, 0, -55], "step": 1,
        "note": f"{P['lower_length']:.2f}×405×2 мм; 15°; передний торец X=25 мм."
    }
    upper = sloped_sheet(P["upper_length"], P["body_width"], P["plastic_thickness"],
                         P["upper_angle"], (P["upper_x0"], 0, P["upper_z0"]))
    parts["upper"] = {
        "mesh": upper, "kind": "upper", "title": "Верхний поддон",
        "color": COLORS["upper"], "explode": [0, 0, 70], "step": 1,
        "note": f"{P['upper_length']:.2f}×405×2 мм; 10°; задний зазор {P['upper_rear_gap']:.0f} мм."
    }
    clip = clip_mesh()
    for key, y, flip in (("clip_left", 0, False), ("clip_right", P["body_width"], True)):
        c = clip.copy()
        if flip:
            c.apply_transform(trimesh.transformations.rotation_matrix(math.pi, [0, 0, 1]))
            c.apply_translation([22, 0, 0])
        # The bridge starts above the cardboard top instead of intersecting it.
        c.apply_translation([P["depth"] - 27, y,
                             P["height"] - (13.0 - 2.4)])
        parts[key] = {
            "mesh": c, "kind": "clip", "title": "Верхняя клипса заднего крыла",
            "color": COLORS["clip"], "explode": [20, -35 if not flip else 35, 35], "step": 4,
            "note": "Опция без скотча: 2 одинаковые печатные U-клипсы."
        }
    for item in parts.values():
        color_mesh(item["mesh"], item["color"])
    return parts, side_prof, inner_prof


def export_meshes(parts, side_prof, inner_prof):
    MESH.mkdir(exist_ok=True)
    for old in MESH.glob("*.stl"):
        old.unlink()
    extrude_xz(side_prof, P["cardboard_thickness"], 0).export(MESH / "A01_БОКОВАЯ_СТЕНКА_МАКЕТ.stl")
    extrude_xz(inner_prof, P["cardboard_thickness"], 0).export(MESH / "A02_ВНУТРЕННЯЯ_ПЕРЕГОРОДКА_МАКЕТ.stl")
    parts["lower"]["mesh"].export(MESH / "A03_НИЖНИЙ_ПОДДОН_МАКЕТ.stl")
    parts["upper"]["mesh"].export(MESH / "A04_ВЕРХНИЙ_ПОДДОН_МАКЕТ.stl")
    clip_mesh().export(MESH / "K01_ЗАДНЯЯ_U_КЛИПСА_ПЕЧАТЬ_2ШТ.stl")
    assembly = trimesh.util.concatenate([v["mesh"] for v in parts.values()])
    assembly.export(MESH / "V13_КАССЕТА_СБОРКА_ДЛЯ_ПРОСМОТРА.stl")
    scene = trimesh.Scene()
    for key, value in parts.items():
        scene.add_geometry(value["mesh"], geom_name=key, node_name=key)
    (MESH / "V13_КАССЕТА_СБОРКА.glb").write_bytes(scene.export(file_type="glb"))


def path_d(poly, height):
    polys = list(poly.geoms) if poly.geom_type == "MultiPolygon" else [poly]
    chunks = []
    for pg in polys:
        rings = [pg.exterior] + list(pg.interiors)
        for ring in rings:
            pts = list(ring.coords)
            chunks.append("M " + " L ".join(f"{x:.3f},{height-y:.3f}" for x, y in pts) + " Z")
    return " ".join(chunks)


def svg_profile(filename, title, profile, width, height, notes):
    margin = 24
    doc_w, doc_h = width + 2 * margin, height + 2 * margin + 18
    d = path_d(profile, height)
    note_svg = "".join(f'<text x="{margin}" y="{height+margin+16+i*7}" class="note">{n}</text>' for i, n in enumerate(notes))
    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{doc_w}mm" height="{doc_h+len(notes)*7}mm" viewBox="0 0 {doc_w} {doc_h+len(notes)*7}">
<style>.cut{{fill:none;stroke:#d94841;stroke-width:.35;fill-rule:evenodd}}.dim{{font:bold 5px Arial;fill:#111}}.note{{font:4px Arial;fill:#333}}.guide{{stroke:#1677aa;stroke-width:.25;stroke-dasharray:3 2}}</style>
<text x="{margin}" y="9" class="dim">{title}</text>
<g transform="translate({margin},{margin})"><path d="{d}" class="cut"/></g>
<text x="{margin}" y="{height+margin+8}" class="dim">Габарит {width:.2f} × {height:.2f} мм. Красная линия = рез.</text>{note_svg}
</svg>'''
    (CUT / filename).write_text(svg, encoding="utf-8")


def export_cutting(side_prof, inner_prof):
    CUT.mkdir(exist_ok=True)
    for old in CUT.glob("*.svg"):
        old.unlink()
    side_depth = P["depth"] - P["cardboard_thickness"]
    svg_profile("01_БОКОВАЯ_СТЕНКА_585x390_2шт.svg",
                "V13 · Боковая стенка · 2 шт. · картон 5 мм", side_prof, side_depth, P["height"],
                [f"Закрытые пазы под ПНД: {P['plastic_slot_width']:.1f} мм; вход листов сбоку.", "Перед = X0; задний торец = X585; кромки целые."])
    svg_profile("02_ВНУТРЕННЯЯ_ПЕРЕГОРОДКА_590x390_4шт.svg",
                "V13 · Внутренняя перегородка · 4 шт. · картон 5 мм", inner_prof, P["depth"], P["height"],
                [f"Задний полупаз: {P['rear_slot_width']:.1f} мм, от Z195 до Z390.", "Оба наклонных паза закрыты: перегородка надевается на ПНД сбоку."])
    for fn, title, length in [
        ("03_НИЖНИЙ_ПОДДОН.svg", "Нижний ПНД-поддон · 1 шт. · 4 мм", P["lower_length"]),
        ("04_ВЕРХНИЙ_ПОДДОН.svg", "Верхний ПНД-поддон · 1 шт. · 4 мм", P["upper_length"]),
    ]:
        svg_profile(fn, f"V13 · {title}", sbox(0, 0, length, P["body_width"]), length, P["body_width"],
                    ["Прямоугольник без пазов; перегородки поочерёдно надеваются на лист сбоку."])
    blank_w = P["rear_blank_width"]
    outline = sbox(0, 0, blank_w, P["height"])
    slot_centres = [P["rear_wing"] + c for c in INNER_CENTRES]
    slots = [sbox(c-P["rear_slot_width"]/2, -1, c+P["rear_slot_width"]/2, P["rear_half_lap_height"]) for c in slot_centres]
    profile = outline.difference(unary_union(slots))
    base = (CUT / "05_ЗАДНЯЯ_СТЕНКА_465x390.svg")
    margin = 24
    d = path_d(profile, P["height"])
    score1 = margin + P["rear_wing"]
    score2 = margin + P["rear_wing"] + P["body_width"]
    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{blank_w+48}mm" height="455mm" viewBox="0 0 {blank_w+48} 455">
<style>.cut{{fill:none;stroke:#d94841;stroke-width:.35;fill-rule:evenodd}}.score{{stroke:#1677aa;stroke-width:.3;stroke-dasharray:4 2}}.t{{font:bold 5px Arial}}.n{{font:4px Arial}}</style>
<text x="24" y="9" class="t">V13 · Задняя стенка · 1 шт. · картон 5 мм</text>
<g transform="translate(24,24)"><path d="{d}" class="cut"/></g>
<line x1="{score1}" y1="24" x2="{score1}" y2="414" class="score"/><line x1="{score2}" y1="24" x2="{score2}" y2="414" class="score"/>
<text x="24" y="425" class="t">Заготовка {blank_w:.0f}×390; центр 405; крылья 30+30 мм.</text>
<text x="24" y="434" class="n">Красная = рез; синяя пунктирная = биговка/линия сгиба.</text>
</svg>'''
    base.write_text(svg, encoding="utf-8")


def mesh_payload(mesh: trimesh.Trimesh):
    m = mesh.copy()
    m.remove_unreferenced_vertices()
    return {
        "v": np.round(m.vertices, 3).reshape(-1).tolist(),
        "f": m.faces.astype(int).reshape(-1).tolist(),
    }


def build_html(parts):
    three = THREE_PATH.read_text(encoding="utf-8")
    data = {}
    for key, item in parts.items():
        data[key] = {
            **mesh_payload(item["mesh"]),
            "title": item["title"], "color": item["color"], "kind": item["kind"],
            "note": item["note"], "explode": item["explode"], "step": item["step"],
        }
    p_json = json.dumps(P, ensure_ascii=False)
    data_json = json.dumps(data, ensure_ascii=False, separators=(",", ":"))
    app = r'''
const DATA = __DATA__;
const P = __PARAMS__;
const scene = new Scene();
scene.background = new Color(0x09121b);
const camera = new PerspectiveCamera(38, 1, 1, 5000);
camera.up.set(0,0,1);
const renderer = new WebGLRenderer({antialias:true, alpha:false});
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.outputColorSpace = SRGBColorSpace;
renderer.shadowMap.enabled = true;
document.getElementById('view').appendChild(renderer.domElement);
scene.add(new HemisphereLight(0xcde8ff, 0x17202c, 2.5));
const sun = new DirectionalLight(0xffffff, 3.2); sun.position.set(-500,-350,850); sun.castShadow=true; scene.add(sun);
const floor = new Mesh(new PlaneGeometry(1100,900), new MeshStandardMaterial({color:0x111e29,roughness:.92}));
floor.rotation.x=0; floor.position.set(300,202,-4); floor.receiveShadow=true; scene.add(floor);
const grid = new GridHelper(900,18,0x315064,0x1c3544); grid.rotation.x=Math.PI/2; grid.position.set(300,202,-3.5); scene.add(grid);
const root = new Group(); scene.add(root);
const parts = {};
for (const [key,d] of Object.entries(DATA)) {
  const g=new BufferGeometry(); g.setAttribute('position',new Float32BufferAttribute(d.v,3)); g.setIndex(d.f); g.computeVertexNormals();
  const mat=new MeshStandardMaterial({color:new Color(d.color),roughness:.72,metalness:.02,side:DoubleSide,transparent:true,opacity:1});
  const m=new Mesh(g,mat); m.castShadow=true; m.receiveShadow=true; m.userData={key,...d}; root.add(m); parts[key]=m;
}
const canGroup=new Group(); root.add(canGroup);
const canMat=new MeshStandardMaterial({color:0x59d889,metalness:.15,roughness:.38});
const lidMat=new MeshStandardMaterial({color:0xe9f2f5,metalness:.55,roughness:.28});
function addCan(x,y,z,angle){
  const g=new Group(); const body=new Mesh(new CylinderGeometry(P.can_diameter_nominal/2,P.can_diameter_nominal/2,P.can_length,40),canMat); body.castShadow=true; g.add(body);
  for(const sy of [-1,1]){const lid=new Mesh(new CylinderGeometry(P.can_diameter_nominal/2+0.7,P.can_diameter_nominal/2+0.7,1.3,40),lidMat);lid.position.y=sy*(P.can_length/2+.65);g.add(lid)}
  g.position.set(x,y,z); g.rotation.y=0; canGroup.add(g);
}
for(let lane=0;lane<5;lane++){
 const y=42.5+lane*80;
 for(const x of [150,245,340]){const a=P.lower_angle*Math.PI/180;const zz=P.lower_z0+(x-P.lower_x0)*Math.tan(a)+(P.can_diameter_nominal/2)/Math.cos(a);addCan(x,y,zz,a)}
 for(const x of [115,225,335,445]){const a=P.upper_angle*Math.PI/180;const zz=P.upper_z0-(x-P.upper_x0)*Math.tan(a)+(P.can_diameter_nominal/2)/Math.cos(a);addCan(x,y,zz,-a)}
}
const dims=new Group(); scene.add(dims); const labels=[];
function dim(a,b,text,pos){
 const geo=new BufferGeometry().setFromPoints([new Vector3(...a),new Vector3(...b)]);const ln=new Line(geo,new LineBasicMaterial({color:0xffdf70}));dims.add(ln);
 for(const p of [a,b]){const q=new Vector3(...p);const tick=new Line(new BufferGeometry().setFromPoints([q.clone().add(new Vector3(0,0,-4)),q.clone().add(new Vector3(0,0,4))]),new LineBasicMaterial({color:0xffdf70}));dims.add(tick)}
 const el=document.createElement('div');el.className='dimlabel';el.textContent=text;document.getElementById('view').appendChild(el);labels.push({el,p:new Vector3(...pos)});
}
dim([0,-18,0],[590,-18,0],'590 мм — глубина',[295,-18,0]);
dim([-18,0,0],[-18,405,0],'405 мм — корпус',[-18,202,0]);
dim([605,415,0],[605,415,390],'390 мм — высота',[605,415,195]);
dim([25,5,28],[25,80,28],'75 мм чистый канал',[25,42.5,28]);
dim([P.upper_x0,425,P.upper_z0],[P.upper_x1,425,P.upper_rear_z],`${P.upper_length.toFixed(2)} мм / 10°`,[(P.upper_x0+P.upper_x1)/2,425,300]);
dim([P.lower_x0,430,P.lower_z0],[P.lower_x1,430,P.lower_rear_z],`${P.lower_length.toFixed(2)} мм / 15°`,[(P.lower_x0+P.lower_x1)/2,430,120]);
let yaw=-2.35,pitch=.48,dist=1050,target=new Vector3(300,202,190);let drag=false,lastX=0,lastY=0;
function updateCamera(){const cp=Math.cos(pitch);camera.position.set(target.x+dist*cp*Math.cos(yaw),target.y+dist*cp*Math.sin(yaw),target.z+dist*Math.sin(pitch));camera.lookAt(target)}
renderer.domElement.addEventListener('pointerdown',e=>{drag=true;lastX=e.clientX;lastY=e.clientY;renderer.domElement.setPointerCapture(e.pointerId)});
renderer.domElement.addEventListener('pointerup',()=>drag=false);
renderer.domElement.addEventListener('pointermove',e=>{if(!drag)return;yaw-=(e.clientX-lastX)*.006;pitch=Math.max(-.05,Math.min(1.42,pitch+(e.clientY-lastY)*.005));lastX=e.clientX;lastY=e.clientY;updateCamera()});
renderer.domElement.addEventListener('wheel',e=>{e.preventDefault();dist=Math.max(360,Math.min(1800,dist*Math.exp(e.deltaY*.001)));updateCamera()},{passive:false});
const ray=new Raycaster(),mouse=new Vector2();renderer.domElement.addEventListener('click',e=>{if(Math.abs(e.clientX-lastX)>3)return;const r=renderer.domElement.getBoundingClientRect();mouse.x=(e.clientX-r.left)/r.width*2-1;mouse.y=-(e.clientY-r.top)/r.height*2+1;ray.setFromCamera(mouse,camera);const hit=ray.intersectObjects(Object.values(parts))[0];if(hit){const d=hit.object.userData;document.getElementById('selected').innerHTML=`<b>${d.title}</b><br>${d.note}`}});
function setExploded(on){for(const m of Object.values(parts)){const e=m.userData.explode;m.position.set(on?e[0]:0,on?e[1]:0,on?e[2]:0)}dims.visible=!on;document.querySelectorAll('.dimlabel').forEach(x=>x.style.display=on?'none':'block')}
function setStep(n){for(const m of Object.values(parts))m.visible=m.userData.step<=n;canGroup.visible=n>=4}
document.querySelectorAll('[data-step]').forEach(b=>b.onclick=()=>setStep(+b.dataset.step));
document.getElementById('explode').onclick=()=>{const on=root.userData.exploded=!root.userData.exploded;setExploded(on)};
document.getElementById('cans').onchange=e=>canGroup.visible=e.target.checked;
document.getElementById('dimensions').onchange=e=>{dims.visible=e.target.checked;document.querySelectorAll('.dimlabel').forEach(x=>x.style.display=e.target.checked?'block':'none')};
function preset(name){if(name==='iso'){yaw=-2.35;pitch=.48;dist=1050}else if(name==='front'){yaw=Math.PI;pitch=.08;dist=950}else if(name==='top'){yaw=-Math.PI/2;pitch=1.48;dist=980}else{yaw=-Math.PI/2;pitch=.08;dist=1000}updateCamera()}
document.querySelectorAll('[data-view]').forEach(b=>b.onclick=()=>preset(b.dataset.view));
function resize(){const el=document.getElementById('view');renderer.setSize(el.clientWidth,el.clientHeight,false);camera.aspect=el.clientWidth/el.clientHeight;camera.updateProjectionMatrix()}
addEventListener('resize',resize);resize();updateCamera();setStep(4);
function animate(){requestAnimationFrame(animate);for(const l of labels){const v=l.p.clone().project(camera);const w=renderer.domElement.clientWidth,h=renderer.domElement.clientHeight;l.el.style.transform=`translate(-50%,-50%) translate(${(v.x*.5+.5)*w}px,${(-v.y*.5+.5)*h}px)`;l.el.style.opacity=(v.z>1||v.z<-1)?0:1}renderer.render(scene,camera)}animate();
'''.replace('__DATA__', data_json).replace('__PARAMS__', p_json)
    html = f'''<!doctype html><html lang="ru"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>V13 · Открытая кассета</title><style>
*{{box-sizing:border-box}}body{{margin:0;background:#09121b;color:#eaf3f8;font:15px/1.4 Segoe UI,Arial,sans-serif;overflow:hidden}}#app{{display:grid;grid-template-columns:minmax(0,1fr) 390px;height:100vh}}#view{{position:relative;min-width:0}}canvas{{display:block;width:100%;height:100%}}aside{{background:#111e29;border-left:1px solid #294253;padding:18px;overflow:auto}}h1{{font-size:22px;margin:0 0 6px}}h2{{font-size:15px;color:#8dd9f5;margin:22px 0 8px}}.muted{{color:#9bb0bd;font-size:13px}}.buttons{{display:grid;grid-template-columns:1fr 1fr;gap:7px}}button{{background:#1c3140;color:#fff;border:1px solid #36596e;border-radius:8px;padding:9px;cursor:pointer;font-weight:650}}button:hover{{background:#294a5d}}table{{border-collapse:collapse;width:100%;font-size:13px}}td{{padding:5px;border-bottom:1px solid #243947;vertical-align:top}}td:last-child{{text-align:right;color:#ffdf70;font-weight:650}}label{{display:block;margin:8px 0}}#selected{{margin-top:10px;background:#0b1720;border:1px solid #284456;border-radius:9px;padding:10px;min-height:68px}}.dimlabel{{position:absolute;left:0;top:0;color:#241b00;background:#ffdf70;border-radius:5px;padding:2px 6px;font:bold 12px Segoe UI;pointer-events:none;white-space:nowrap;box-shadow:0 2px 8px #0008}}.warning{{background:#382b13;border:1px solid #8e6b25;border-radius:8px;padding:9px;color:#ffe4a1}}@media(max-width:900px){{#app{{grid-template-columns:1fr;grid-template-rows:58vh 42vh}}aside{{border-left:0;border-top:1px solid #294253}}}}
</style></head><body><div id="app"><main id="view"></main><aside><h1>V13 · Открытая кассета</h1><div class="muted">5 каналов · банка 70×Ø47 · без внешней коробки</div>
<h2>Вид</h2><div class="buttons"><button data-view="iso">Изометрия</button><button data-view="side">Сбоку</button><button data-view="top">Сверху</button><button data-view="front">Спереди</button><button id="explode">Разнести детали</button></div><label><input id="dimensions" type="checkbox" checked> Размеры</label><label><input id="cans" type="checkbox" checked> Показать банки</label>
<h2>Сборка</h2><div class="buttons"><button data-step="1">1. Скаты + боковина</button><button data-step="2">2. Все 6 стенок</button><button data-step="3">3. Задняя стенка</button><button data-step="4">4. Готово</button></div>
<h2>Главные размеры</h2><table><tr><td>Корпус Г×Ш×В</td><td>590×405×390</td></tr><tr><td>Чистый канал</td><td>75</td></tr><tr><td>Шаг каналов</td><td>80</td></tr><tr><td>Картон, номинал</td><td>5</td></tr><tr><td>ПНД-поддоны</td><td>4</td></tr><tr><td>Нижний скат</td><td>{P['lower_length']:.2f} / 15°</td></tr><tr><td>Верхний скат</td><td>{P['upper_length']:.2f} / 10°</td></tr><tr><td>Задний зазор верху</td><td>{P['upper_rear_gap']:.0f}</td></tr><tr><td>Заготовка задней стенки</td><td>{P['rear_blank_width']:.0f}×390</td></tr></table>
<div id="selected">Нажми на деталь в 3D.</div><h2>Принцип</h2><div class="muted">Оба ската проходят через закрытые пазы первой боковины. Остальные пять стенок поочерёдно нанизываются на скаты сбоку с шагом 80 мм. Затем задняя стенка полупазами выставляет 4 внутренние перегородки, а её крылья обхватывают внешние. Две U-клипсы заменяют скотч.</div><p class="warning">Это номинал под картон 5,0 и ПНД 4,0 мм. До резки измерить фактическую толщину.</p></aside></div><script type="module">{three}\n{app}</script></body></html>'''
    (ROOT / "ОТКРОЙ_V13_КАССЕТУ_3D.html").write_text(html, encoding="utf-8")


def export_docs():
    (ROOT / "ПАРАМЕТРЫ_V13.json").write_text(json.dumps(P, ensure_ascii=False, indent=2), encoding="utf-8")
    rows = [
        ["Код", "Деталь", "Кол-во", "Материал", "Размер, мм", "Примечание"],
        ["A01", "Боковая стенка", 2, "5-слойный гофрокартон", "585×390×5", "Два закрытых паза; передняя и задняя кромки целые"],
        ["A02", "Внутренняя перегородка", 4, "5-слойный гофрокартон", "590×390×5", "Два закрытых паза + задний полупаз"],
        ["A03", "Нижний поддон", 1, "Листовой ПНД 4 мм", f"{P['lower_length']:.2f}×405×4", "15°; стенки надеваются сбоку"],
        ["A04", "Верхний поддон", 1, "Листовой ПНД 4 мм", f"{P['upper_length']:.2f}×405×4", "10°; стенки надеваются сбоку"],
        ["A05", "Задняя стенка с крыльями", 1, "Гофрокартон 5 мм", "465×390×5", "Центр 405; крылья 30+30"],
        ["K01", "U-клипса заднего крыла", 2, "Hyper PLA", "22×15×13", "Опция без скотча"],
    ]
    with (ROOT / "BOM_КАССЕТА_V13.csv").open("w", newline="", encoding="utf-8-sig") as f:
        csv.writer(f, delimiter=";").writerows(rows)
    dimensions = [
        ["Группа", "Параметр", "Значение, мм", "Примечание"],
        ["Габарит", "Глубина в сборке", 590, "Включая заднюю панель 5 мм"],
        ["Габарит", "Ширина корпуса", 405, "Без наружной толщины задних крыльев"],
        ["Габарит", "Высота", 390, ""],
        ["Каналы", "Чистая ширина", 75, "Длина банки 70; по 2,5 мм общего запаса"],
        ["Каналы", "Шаг осей", 80, "При картоне 5,0"],
        ["Каналы", "Оси 6 стенок от левого края", "2,5 / 82,5 / 162,5 / 242,5 / 322,5 / 402,5", ""],
        ["Материал", "Картон, номинал", 5, "Измерить до резки"],
        ["Материал", "ПНД, номинал", 4, "Измерить до резки"],
        ["Пазы", "Паз под ПНД", P["plastic_slot_width"], "4,0 + 0,6 мм общего зазора"],
        ["Пазы", "Задний полупаз", P["rear_slot_width"], "5,0 + 0,6 мм"],
        ["Нижний скат", "Размер заготовки", f"{P['lower_length']:.2f} × 405 × 4", ""],
        ["Нижний скат", "Угол", "15°", ""],
        ["Нижний скат", "Передний верх X/Z", f"{P['lower_x0']:.0f} / {P['lower_z0']:.2f}", "Передняя кромка стенки остаётся цельной"],
        ["Нижний скат", "Задний верх X/Z", f"{P['lower_x1']:.0f} / {P['lower_rear_z']:.2f}", f"До внутренней плоскости задней стенки {P['depth'] - P['cardboard_thickness'] - P['lower_x1']:.0f}"],
        ["Верхний скат", "Размер заготовки", f"{P['upper_length']:.2f} × 405 × 4", ""],
        ["Верхний скат", "Угол", "10°", ""],
        ["Верхний скат", "Передний верх X/Z", f"{P['upper_x0']:.0f} / {P['upper_z0']:.2f}", "Передняя кромка стенки остаётся цельной"],
        ["Верхний скат", "Задний верх X/Z", f"{P['upper_x1']:.0f} / {P['upper_rear_z']:.2f}", f"До внутренней плоскости задней стенки {P['depth'] - P['cardboard_thickness'] - P['upper_x1']:.0f}"],
        ["Задняя стенка", "Плоская заготовка", "465 × 390 × 5", "Центр 405 + крылья 30+30"],
        ["Задняя стенка", "Линии сгиба от левого края", "30 / 435", ""],
        ["Задняя стенка", "Оси полупазов на плоской заготовке", "112,5 / 192,5 / 272,5 / 352,5", "От левого края заготовки"],
        ["Задняя стенка", "Высота полупаза", 195, "От низа"],
    ]
    with (ROOT / "ВСЕ_РАЗМЕРЫ_V13.csv").open("w", newline="", encoding="utf-8-sig") as f:
        csv.writer(f, delimiter=";").writerows(dimensions)
    readme = f'''# V13 — открытая кассета на 5 каналов

Это отдельная новая ветка. Старые файлы не изменены. Модель описывает только кассету; новый дозатор ещё не проектировался. Оба паза под скаты закрытые: передняя и задняя кромки всех шести стенок сохраняются целыми.

## Номинал

- Банка: 70×Ø47 мм; контрольный диаметр 49 мм.
- Кассета без задних наружных крыльев: 590×405×390 мм.
- 5 каналов по 75 мм; шаг 80 мм.
- Картон: номинал 5,0 мм.
- ПНД: номинал 4,0 мм (домашний лист 1000×1000; раскрой на 2 кассеты уже готов).
- Нижний поддон: {P['lower_length']:.2f}×405 мм, 15°.
- Верхний поддон: {P['upper_length']:.2f}×405 мм, 10°.
- Задняя заготовка: 465×390 мм; два крыла по 30 мм.

## Почему сборка простая

1. Нижний и верхний ПНД-поддоны продеваются сбоку через два закрытых паза первой боковой стенки.
2. На оба поддона сбоку поочерёдно нанизываются ещё пять стенок с шагом 80 мм.
3. Четыре внутренние перегородки соединяются с задней стенкой полупазами — это точно выставляет каналы.
4. Задние крылья загибаются вперёд, обхватывают внешние стенки и фиксируются двумя K01. Для первой проверки можно использовать скотч.
5. Закрытые пазы сами удерживают поддоны по глубине; печатный дозаторный док затем состыкуется с их передними торцами без разрыва траектории банки.

## Важно до резки

Измерить штангенциркулем фактическую толщину картона и ПНД. В этой модели паз под ПНД равен {P['plastic_slot_width']:.1f} мм, задний полупаз — {P['rear_slot_width']:.1f} мм. Файлы раскроя не следует печатать 1:1 до натурного замера.
'''
    (ROOT / "00_РАЗМЕРЫ_И_СБОРКА.md").write_text(readme, encoding="utf-8")
    (ROOT / "ОТКРЫТЬ_3D.cmd").write_text(
        '@echo off\r\nfor %%F in ("%~dp0*.html") do start "" "%%~fF"\r\n',
        encoding="ascii")


def main():
    parts, side_prof, inner_prof = build_parts()
    export_meshes(parts, side_prof, inner_prof)
    export_cutting(side_prof, inner_prof)
    export_docs()
    build_html(parts)
    print(json.dumps({
        "output": str(ROOT),
        "parts": len(parts),
        "body_mm": [P["depth"], P["body_width"], P["height"]],
        "lower_mm": [round(P["lower_length"], 2), P["body_width"], P["plastic_thickness"]],
        "upper_mm": [round(P["upper_length"], 2), P["body_width"], P["plastic_thickness"]],
    }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
