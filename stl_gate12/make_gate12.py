# -*- coding: utf-8 -*-
"""
Дозатор БАД — затвор №12 «печатная шестерня + рейки», версия 5.

Изменения v3 (после примерки пользователя):
  • WALL_H = 47 (замер) — верх банки вровень с гребнем стенки.
  • Шасси = СКЕЛЕТ-РАМА 4 мм (2 балки над прорезями + 2 рельса + хребет) вместо
    плиты: минус ~40% пластика, низ плоский, поддержек ноль.
  • Ножки: ШИП-ЛЕЗВИЕ В ТОРЕЦ КАРТОНА (клин 2 мм входит в гофру) + плечики 5×12
    на гребень + колонна 16 мм + штырь 6.2 в гнездо рамы.
    В ПРОСВЕТ ДОРОЖКИ НЕ ВЫСТУПАЕТ НИЧЕГО (зазор банка-стенка 1–3 мм не тронут).
  • Колонна 16 мм поднимает раму: банки проходят под ней свободно, а зубья Г1
    не налезают на клин рейки (проверено расчётом зоны зацепления).
  • Шестерня 8 -> 6 мм + 4 окна Ø9 по диагоналям.
  • assembly_view.stl — сборка для ПРОСМОТРА зазоров (НЕ печатать!).

Файлы: plate_rest.stl (шасси+рейки+ножки+палец), plate_gear.stl (шестерня),
gear_m2_z24 / rack_G1 / rack_G2 / chassis / foot_x4 / gear_stub,
assembly_view.stl, README_печать.txt.
"""
import os, math
import numpy as np
import trimesh
from shapely.geometry import Polygon, Point
from shapely.ops import unary_union

OUT = os.path.dirname(os.path.abspath(__file__))

# ---------------- параметры ----------------
NOZ_CLR   = 0.25
M         = 2.0
Z_TEETH   = 24
PA        = math.radians(20)
BACKLASH  = 0.30

GEAR_TH   = 6.0
HORN_POCKET_DEPTH = 2.4
HORN_HUB_R   = 3.7
HORN_ARM_LEN = 15.8
HORN_ARM_W0  = 6.9
HORN_ARM_W1  = 4.0
HORN_CLR     = 0.35
CENTER_HOLE  = 5.5

RACK_W    = 10.0
RACK_BODY = 5.0
TOOTH_H   = 2.25*M
PITCH     = math.pi*M

R_PITCH   = M*Z_TEETH/2      # 24
GY        = 40.0             # ось шестерни над рамой (подняли, чтобы укоротить колонны ножек)
STROKE    = 36.0            # ход ограничен Г1 (её зубья иначе налезают на колено-клин).

# ===== посадка (замеры пользователя) =====
WALL_H    = 47.0    # высота картонной стенки от деки до гребня
WALL_T    = 3.0     # ТОЛЩИНА стенки-перегородки (подтверждено: 3 мм) -> шаг 75+3=78.
                    # Рама, ножки, плечики и пазы пересчитываются от этого числа.
LANE_IN   = 75.0
FOOT_WEB  = 2.5     # плечики ножки на гребне
FOOT_LIFT = 6.0     # короткий столбик-зазор (банки проходят под рамой с запасом 8.5)
PLATE_TH  = 4.0
DECK      = -(WALL_H + FOOT_WEB + FOOT_LIFT + PLATE_TH)   # -59.5
G2_BOT_CL = DECK+18    # v6: было +13 (открытая Г2 давала зазор над банкой всего +2мм).
                       # +18: блок по экватору сохраняется, открытая Г2 на deck+54 -> зазор +7мм.
G1_BOT_CL = DECK+32

PLATE_W   = LANE_IN + WALL_T   # 78 = шаг дорожки, рама ось-ось стенок
PLATE_L   = 84.0
TOWER_H   = 38.0    # v4: БЫЛО 18 — рейки 78-96мм гнулись. Высокие направляющие до
                    # зоны зацепления (z~42) => рейка держится по длине, не косит.
BACK_H    = 54.0    # v5: задняя стенка канала ВЫШЕ оси шестерни (44) — держит рейку
                    # по обе стороны точки толчка, антизаклин от косения. За z-габаритом
                    # шестерни (26) => коллизий нет (проверено булевым чеком).

# --- v4: ДАЛЬНЯЯ ОПОРА ШЕСТЕРНИ (2 опоры вместо консоли; убирает перекос и уход от серво) ---
GEAR_AXZ  = 40.0 + 4.0   # ось шестерни в системе шасси = GY(40)+PLATE_TH(4) = 44
FAR_X     = -15.5        # плоскость дальней стенки (за передней гранью шестерни X=-11)
FAR_TH    = 3.4         # толщина дальней стенки
BUSH_D    = 8.5         # сквозная втулка под цапфу пальца (цапфа 7.9 + зазор 0.6)

GEAR_X    = -8.0
SERVO_BACK = 5.0           # v7.2: серво-мостик отодвинут от шестерни на 5мм (был зазор
                           # шестерня(зад -5)↔стенка(-0.5)=4.5мм, серво упирался). Стало ~9.5мм.
SLOT_CLR  = 0.55           # v7: паз рейки шире (было 0.25) — рейка входит без напильника.
SLOT_X0, SLOT_X1 = GEAR_X-RACK_W/2-SLOT_CLR, GEAR_X+RACK_W/2+SLOT_CLR
SLOT_Z    = (21.4, 31.5)   # v4: БЫЛО (19.4,30.1) => межосевое 22 (перезацеп 2мм, клин).
                           # Отодвинуто на 2мм: рейка упором в наружную стенку даёт R=24 точно.
RACK_REST = 31.5           # координата наружной стенки = где стоит спинка рейки (=R_PITCH+7.5)

# УНИВЕРСАЛЬНАЯ пазл-стыковка: свои ножки правая стенка {+30,-14}, левая {+14,-30};
# у соседа на общей стенке получаются {+14,-30} — четыре ножки без конфликтов.
HW = PLATE_W/2
FOOT_POS     = [(+HW, +30), (+HW, -14), (-HW, +14), (-HW, -30)]   # мои (пазы в кромке)
NEIGHBOR_POS = [(+HW, +14), (+HW, -30), (-HW, +30), (-HW, -14)]   # соседские (вырезы-пропуски)
FOOT_PIN  = 6.2

report = []

# ---------------- утилиты ----------------
def poly_circle(cx, cy, r, n=64):
    a = np.linspace(0, 2*math.pi, n, endpoint=False)
    return Polygon(np.c_[cx+r*np.cos(a), cy+r*np.sin(a)])

def _rect(x0, z0, x1, z1):
    return Polygon([(x0, z0), (x1, z0), (x1, z1), (x0, z1)])

def ex(poly, h, dz=0.0):
    m = trimesh.creation.extrude_polygon(poly, height=h)
    m.apply_translation([0, 0, dz])
    return m

def fuse(meshes):
    try:
        return trimesh.boolean.union(meshes, engine='manifold')
    except BaseException:
        return trimesh.util.concatenate(meshes)

def save(mesh, name):
    mesh.merge_vertices()
    mesh.export(os.path.join(OUT, name))
    b = mesh.bounds
    report.append(f"{name:20s} {b[1][0]-b[0][0]:6.1f} x {b[1][1]-b[0][1]:6.1f} x {b[1][2]-b[0][2]:5.1f} мм  "
                  f"водонепр={mesh.is_watertight}  объём={mesh.volume/1000:.1f} см3")
    return mesh

def dedup(pts):
    out = [pts[0]]
    for p in pts[1:]:
        if (p[0]-out[-1][0])**2 + (p[1]-out[-1][1])**2 > 1e-10:
            out.append(p)
    if (out[0][0]-out[-1][0])**2 + (out[0][1]-out[-1][1])**2 < 1e-10:
        out.pop()
    return out

# ---------------- 1. ШЕСТЕРНЯ ----------------
def involute_gear_polygon():
    rb = R_PITCH*math.cos(PA)
    ra = R_PITCH + M
    rr = R_PITCH - 1.25*M
    inv = lambda a: math.tan(a) - a
    s_half = math.pi/(2*Z_TEETH) - (BACKLASH/2)/R_PITCH
    def flank_ang(r):
        a = math.acos(rb/r)
        return s_half + inv(PA) - inv(a)
    pts = []
    n_fl = 8
    for i in range(Z_TEETH):
        c = i*2*math.pi/Z_TEETH
        aB = flank_ang(rb)
        pts.append((rr*math.cos(c-aB), rr*math.sin(c-aB)))
        for k in range(n_fl+1):
            r = rb + (ra-rb)*k/n_fl
            a = c - flank_ang(r)
            pts.append((r*math.cos(a), r*math.sin(a)))
        aT = flank_ang(ra)
        pts.append((ra*math.cos(c+aT), ra*math.sin(c+aT)))
        for k in range(n_fl, -1, -1):
            r = rb + (ra-rb)*k/n_fl
            a = c + flank_ang(r)
            pts.append((r*math.cos(a), r*math.sin(a)))
        pts.append((rr*math.cos(c+aB), rr*math.sin(c+aB)))
        c2 = (i+1)*2*math.pi/Z_TEETH
        a0, a1 = c+aB, c2-aB
        for k in range(1, 4):
            a = a0 + (a1-a0)*k/4
            pts.append((rr*math.cos(a), rr*math.sin(a)))
    return Polygon(dedup(pts))

def horn_pocket_poly():
    hub = poly_circle(0, 0, HORN_HUB_R, 48)
    arms = []
    for s in (+1, -1):
        arms.append(Polygon([(s*3.0,  HORN_ARM_W0/2), (s*HORN_ARM_LEN,  HORN_ARM_W1/2),
                             (s*HORN_ARM_LEN, -HORN_ARM_W1/2), (s*3.0, -HORN_ARM_W0/2)]))
        arms.append(poly_circle(s*HORN_ARM_LEN, 0, HORN_ARM_W1/2, 24))
    return unary_union([hub] + arms).buffer(HORN_CLR, join_style=1)

def build_gear():
    outline = involute_gear_polygon()
    pocket = horn_pocket_poly()
    # окна облегчения по диагоналям (мимо лучей качалки)
    lite = [poly_circle(15*math.cos(a), 15*math.sin(a), 4.5, 28)
            for a in (math.pi/4, 3*math.pi/4, 5*math.pi/4, 7*math.pi/4)]
    front = Polygon(outline.exterior.coords,
                    [poly_circle(0, 0, CENTER_HOLE/2).exterior.coords] +
                    [h.exterior.coords for h in lite])
    back  = Polygon(outline.exterior.coords,
                    [pocket.exterior.coords] + [h.exterior.coords for h in lite])
    g = fuse([ex(front, GEAR_TH-HORN_POCKET_DEPTH, 0),
              ex(back,  HORN_POCKET_DEPTH+0.02, GEAR_TH-HORN_POCKET_DEPTH-0.02)])
    return save(g, "gear_m2_z24.stl")

# ---------------- 2–3. РЕЙКИ ----------------
def rack_teeth_edge(u0, u1, body_v):
    ht_root = PITCH/4 + 1.25*M*math.tan(PA)
    ht_tip  = PITCH/4 - M*math.tan(PA)
    tip_v   = body_v + TOOTH_H
    pts = [(body_v, u0)]
    c = u0 + PITCH/2
    while c + ht_root < u1:
        pts += [(body_v, c-ht_root), (tip_v, c-ht_tip),
                (tip_v, c+ht_tip),   (body_v, c+ht_root)]
        c += PITCH
    pts.append((body_v, u1))
    return pts

def build_rack_G2():
    cont_cl, cont_op = GY - G2_BOT_CL, GY - (G2_BOT_CL + STROKE)
    t0, t1 = cont_op - 6, cont_cl + 6
    L = cont_cl + 10
    pts = [(0,1.5),(1.5,0),(RACK_BODY-1.5,0),(RACK_BODY,1.5),(RACK_BODY,t0)] \
          + rack_teeth_edge(t0,t1,RACK_BODY)[1:] + [(RACK_BODY,L),(0,L)]
    return save(ex(Polygon(dedup(pts)), RACK_W), "rack_G2.stl")

def build_rack_G1():
    cont_cl, cont_op = GY - G1_BOT_CL, GY - (G1_BOT_CL + STROKE)
    t0, t1 = cont_op - 6, cont_cl + 6
    L = cont_cl + 10
    bth, bofs = 3.0, 4.5
    bin_, bout = bofs + RACK_BODY - bth, bofs + RACK_BODY
    jog0, jog1 = 14.0, 24.0
    assert t0 >= jog1 + 1.0, f"зубья Г1 ({t0:.1f}) налезают на клин (до {jog1})"
    pts = [(0, jog1), (bin_, jog0), (bin_, 5.0),
           (bin_+0.9, 0.0), (bout-0.9, 0.0), (bout, 5.0),
           (bout, jog0), (RACK_BODY, jog1), (RACK_BODY, t0)]
    pts += rack_teeth_edge(t0, t1, RACK_BODY)[1:]
    pts += [(RACK_BODY, L), (0, L)]
    return save(ex(Polygon(dedup(pts)), RACK_W), "rack_G1.stl")

# ---------------- 4. ШАССИ-РАМА (универсальная, тайлится пазлом) ----------------
def chassis_parts():
    hw, hl = PLATE_W/2, PLATE_L/2       # ровно 78 шириной, БЕЗ ушей за ось стенки
    frame = unary_union([
        _rect(-hw, -hl, -hw+6, hl), _rect(hw-6, -hl, hw, hl),      # рельсы 6
        _rect(-hw, 17.0, hw, 32.6), _rect(-hw, -32.6, hw, -17.0),  # балки над прорезями
        _rect(-2.5, -17, 13, 17),                                  # хребет под серво
        _rect(-17, -12, 13, 12)])                                  # площадка дальней опоры
    cuts = [_rect(SLOT_X0,  SLOT_Z[0], SLOT_X1,  SLOT_Z[1]),
            _rect(SLOT_X0, -SLOT_Z[1], SLOT_X1, -SLOT_Z[0])]
    for fx, fz in FOOT_POS:      # пазы под СВОИ штыри (охват 3 стороны, 4-я — сосед)
        s = 1 if fx > 0 else -1
        cuts.append(_rect(min(s*(hw-3.1), s*(hw+0.6)), fz-3.35, max(s*(hw-3.1), s*(hw+0.6)), fz+3.35))
    for fx, fz in NEIGHBOR_POS:  # вырезы-пропуски под колонны СОСЕДА
        s = 1 if fx > 0 else -1
        cuts.append(_rect(min(s*(hw-3.6), s*(hw+0.6)), fz-4.3, max(s*(hw-3.6), s*(hw+0.6)), fz+4.3))
    prof = frame.difference(unary_union(cuts))
    parts = [ex(prof, PLATE_TH, 0)]
    # башни-направляющие (тоньше и ниже)
    for sgn in (+1, -1):
        z0, z1 = sgn*SLOT_Z[0], sgn*SLOT_Z[1]
        zlo, zhi = min(z0, z1), max(z0, z1)
        clo = zlo-2.4 if sgn < 0 else zlo
        chi = zhi+2.4 if sgn > 0 else zhi
        for cx0, cx1 in ((SLOT_X0-2.4, SLOT_X0), (SLOT_X1, SLOT_X1+2.4)):
            parts.append(ex(_rect(cx0, clo, cx1, chi), TOWER_H, PLATE_TH-0.02))
        back = (zhi, zhi+2.4) if sgn > 0 else (zlo-2.4, zlo)
        parts.append(ex(_rect(SLOT_X0-2.4, back[0], SLOT_X1+2.4, back[1]), BACK_H, PLATE_TH-0.02))
    # стенка-окно SG90 (отодвинута от шестерни на SERVO_BACK)
    WIN_W, WIN_H = 12.9, 23.4
    WALL_X0, WALL_TH = -0.5 + SERVO_BACK, 3.6
    shaft_pz = GY + PLATE_TH
    win_top  = shaft_pz + 5.9 + 0.35
    win_bot  = win_top - WIN_H
    wall_top = win_top + 5.0
    win  = _rect(-WIN_W/2, win_bot, WIN_W/2, win_top)
    prof_w = Polygon([(-13, PLATE_TH-0.02), (13, PLATE_TH-0.02),
                      (13, wall_top), (-13, wall_top)],
                     [win.exterior.coords,
                      poly_circle(0, win_top+2.6, 0.9, 16).exterior.coords,
                      poly_circle(0, win_bot-2.6, 0.9, 16).exterior.coords])
    wall = trimesh.creation.extrude_polygon(prof_w, height=WALL_TH)
    wall.apply_transform(np.array([[0,0,1,WALL_X0],[1,0,0,0],[0,1,0,0],[0,0,0,1]], float))
    parts.append(wall)
    for sgn in (+1, -1):
        z0, z1 = (8, 12) if sgn > 0 else (-12, -8)
        parts.append(ex(_rect(WALL_X0+WALL_TH-0.02, z0, WALL_X0+10, z1), 26, PLATE_TH-0.02))
    # ДАЛЬНЯЯ ОПОРА ШЕСТЕРНИ: стенка ⊥оси со сквозной втулкой + 2 контрфорса (по просьбе)
    fa = poly_circle(0, GEAR_AXZ, BUSH_D/2, 40)
    prof_far = Polygon([(-14, PLATE_TH-0.02), (14, PLATE_TH-0.02),
                        (14, GEAR_AXZ+8), (-14, GEAR_AXZ+8)],
                       [fa.exterior.coords])
    farwall = trimesh.creation.extrude_polygon(prof_far, height=FAR_TH)
    farwall.apply_transform(np.array([[0,0,1,FAR_X],[1,0,0,0],[0,1,0,0],[0,0,0,1]], float))
    parts.append(farwall)
    for gy in (-9.0, 9.0):   # рёбра-контрфорсы стенка->площадка (слева от шестерни)
        parts.append(ex(_rect(FAR_X, gy-1.7, FAR_X+3.0, gy+1.7), 18, PLATE_TH-0.02))
    parts += weight_bosses()
    return parts

# ---------------- ГРУЗИКИ: 4 бобышки с шестигранным карманом под гайку M10 ----------------
# (гайка ~11г × 4 = 44г; сами бобышки добавляют пластик). Плюс сверху плоско —
# можно клеить самоклеящиеся балансировочные грузики (5/10г полоски с Ozon/WB).
NUT_AF   = 17.4          # под ключ гайки M10 (17) + зазор
NUT_DEPTH = 8.5          # глубина кармана (толщина M10 ~8.4)
BOSS_H   = 11.0
def weight_bosses():
    out = []
    R = (NUT_AF/2) / math.cos(math.radians(30))   # радиус по вершинам шестигранника
    for sx in (+1, -1):
        for sz in (+1, -1):
            cx, cz = sx*25, sz*35
            solid = ex(_rect(cx-11, cz-9, cx+11, cz+9), BOSS_H, PLATE_TH-0.02)
            hexpts = [(cx+R*math.cos(math.radians(60*k)), cz+R*math.sin(math.radians(60*k))) for k in range(6)]
            pocket = ex(Polygon(hexpts), NUT_DEPTH+0.1, PLATE_TH-0.02+BOSS_H-NUT_DEPTH)
            try:
                out.append(trimesh.boolean.difference([solid, pocket], engine='manifold'))
            except BaseException:
                out.append(solid)
    return out

def build_chassis_mesh():
    return save(fuse(chassis_parts()), "chassis.stl")

# ---------------- 4б. НОЖКА: точка-опора (шип-игла в 10 раз меньше) ----------------
def foot_print_mesh():
    """узел СТОИТ на 4 плечиках; в картон входит только игла 2×3×6 с остриём.
    Печать штырём вниз: сечения кверху сужаются, поддержек ноль."""
    parts = [ex(_rect(-FOOT_PIN/2, -FOOT_PIN/2, FOOT_PIN/2, FOOT_PIN/2), 5.4, 0),
             ex(_rect(-3, -4, 3, 4), FOOT_LIFT, 5.4-0.02),
             ex(_rect(-WALL_T/2, -6, WALL_T/2, 6), FOOT_WEB, 5.4+FOOT_LIFT-0.04),
             ex(_rect(-1, -1.5, 1, 1.5), 6, 5.4+FOOT_LIFT+FOOT_WEB-0.06)]
    zt = 5.4+FOOT_LIFT+FOOT_WEB+6-0.06
    tip = trimesh.convex.convex_hull(trimesh.PointCloud(np.array([
        [-1,-1.5,zt],[1,-1.5,zt],[1,1.5,zt],[-1,1.5,zt],[0,0,zt+3]], float)))
    parts.append(tip)
    return save(fuse(parts), "foot_x4.stl")

# ---------------- 4в. ЖУРНАЛ-ПАЛЕЦ дальней опоры ----------------
def build_stub():
    """v4: вставляется СНАРУЖИ через дальнюю втулку. Порядок ступеней (хвост->наружу):
    хвостовик 5.25 (натяг в бор шестерни 5.5) -> цапфа 7.9 (крутится во втулке 8.5) ->
    буртик-ручка 10 (упор в наружную грань стенки, осевой датум). Печать стоя, без поддержек.
    Ступени только РАСТУТ по диаметру => ни одного нависания."""
    tail   = ex(poly_circle(0, 0, 5.25/2, 48), 3.5, 0)            # в бор 5.5 (клей)
    journ  = ex(poly_circle(0, 0, 7.9/2,  48), 5.0, 3.5-0.02)     # цапфа во втулку 8.5
    knob   = ex(poly_circle(0, 0, 10.0/2, 48), 2.0, 3.5+5.0-0.04) # буртик-упор снаружи
    # СПЛОШНОЙ (без сквозного канала — длинный винт не нужен: качалку к серво
    # крепим её штатным винтом ДО шестерни, палец держит только осево).
    s = fuse([tail, journ, knob])
    return save(s, "gear_stub.stl")

# ---------------- сборка для просмотра ----------------
def build_assembly(chas, gear, r1, r2, foot, stub):
    A = []
    def T(mesh, mat):
        m = mesh.copy(); m.apply_transform(np.array(mat, float)); return m
    # шасси: печатные (x, z, h) -> сборка (x, h-4, z)
    A.append(T(chas, [[1,0,0,0],[0,0,1,-PLATE_TH],[0,1,0,0],[0,0,0,1]]))
    # шестерня: профиль XY толщина 6 -> плоскость ⊥X, центр GEAR_X, высота GY
    A.append(T(gear, [[0,0,1,GEAR_X-GEAR_TH/2],[0,1,0,GY],[1,0,0,0],[0,0,0,1]]))
    # рейки (Г2 закрыта, Г1 открыта — состояние покоя)
    A.append(T(r2, [[0,0,1,GEAR_X-RACK_W/2],[0,1,0,G2_BOT_CL],[1,0,0,-RACK_REST],[0,0,0,1]]))
    A.append(T(r1, [[0,0,1,GEAR_X-RACK_W/2],[0,1,0,G1_BOT_CL+STROKE],[-1,0,0,RACK_REST],[0,0,0,1]]))
    A.append(T(stub, [[0,0,-1,-7.5],[1,0,0,GY],[0,1,0,0],[0,0,0,1]]))   # журнал-палец в дальнюю втулку
    # ножки: печатную повернуть шипом вверх и поставить в пазы кромок;
    # плюс 2 СОСЕДСКИЕ ножки на правой общей стенке — показать пазл-стыковку
    for fx, fz in FOOT_POS + NEIGHBOR_POS[:2]:
        f = foot.copy()
        f.apply_transform(trimesh.transformations.rotation_matrix(math.pi, [1,0,0]))
        b = f.bounds   # после переворота игла внизу
        f.apply_translation([fx-(b[0][0]+b[1][0])/2, -PLATE_TH+1.4-b[1][1], fz-(b[0][2]+b[1][2])/2])
        A.append(f)
    # контекст: стенки, дека и КРОМКА РАМЫ СОСЕДА (только для просмотра)
    for sx in (+1, -1):
        w = trimesh.creation.box(extents=[WALL_T, WALL_H, PLATE_L])
        w.apply_translation([sx*HW, DECK+WALL_H/2, 0]); A.append(w)
    d = trimesh.creation.box(extents=[PLATE_W+16, 2, PLATE_L])
    d.apply_translation([0, DECK-1, 0]); A.append(d)
    nb = trimesh.creation.box(extents=[6, PLATE_TH, PLATE_L])
    nb.apply_translation([HW+0.4+3, -PLATE_TH/2, 0]); A.append(nb)
    combo = trimesh.util.concatenate(A)
    combo.export(os.path.join(OUT, "assembly_view.stl"))
    b = combo.bounds
    report.append(f"{'assembly_view.stl':20s} {b[1][0]-b[0][0]:6.1f} x {b[1][1]-b[0][1]:6.1f} x {b[1][2]-b[0][2]:5.1f} мм  (ТОЛЬКО ПРОСМОТР)")

# ---------------- листы печати (шестерня отдельно — свои настройки) ----------------
def build_plate(parts, name):
    placed, x, y, row_h = [], 0.0, 0.0, 0.0
    gap, MAXW = 8.0, 200.0
    for m in parts:
        mm = m.copy()
        b = mm.bounds
        w, d = b[1][0]-b[0][0], b[1][1]-b[0][1]
        if x > 0 and x + w > MAXW:
            x, y, row_h = 0.0, y + row_h + gap, 0.0
        mm.apply_translation([-b[0][0]+x, -b[0][1]+y, -b[0][2]])
        placed.append(mm)
        x += w + gap
        row_h = max(row_h, d)
    combo = trimesh.util.concatenate(placed)
    b = combo.bounds
    combo.apply_translation([-(b[0][0]+b[1][0])/2, -(b[0][1]+b[1][1])/2, 0])
    return save(combo, name)

gear = build_gear()
r2   = build_rack_G2()
r1   = build_rack_G1()
chas = build_chassis_mesh()
foot = foot_print_mesh()
stub = build_stub()
build_plate([chas, r2, r1, foot, foot, foot, foot, stub], "plate_rest.stl")
build_plate([chas], "plate_chassis.stl")   # ТОЛЬКО главная деталь (для допечатки)
build_plate([gear], "plate_gear.stl")
build_plate([gear]*5, "plate_gear5.stl")   # лист из 5 шестерён на 5 каналов
one_set = [chas, r2, r1, foot, foot, foot, foot, stub]
build_plate(one_set*2, "plate_rest2.stl")  # 2 комплекта (рама+рейки+ножки+палец) на лист
# --- мелочь на 4 комплекта одним листом (рамы/шестерни уже напечатаны) ---
Rz = trimesh.transformations.rotation_matrix(math.pi/2, [0,0,1])
r2r = r2.copy(); r2r.apply_transform(Rz)   # рейки кладём вдоль X, чтобы влезли по глубине
r1r = r1.copy(); r1r.apply_transform(Rz)
small4 = [r2r]*4 + [r1r]*4 + [foot]*16 + [stub]*4
build_plate(small4, "plate_small4.stl")
# --- ножки на ОБЩЕЙ платформе-гребёнке: печатается как один кусок, не оторвать ---
def feet_raft(nx=6, ny=4, pitch=18.0, th=1.2):
    """24 ножки стоят на сплошной плите th мм. Плита прилипает всей площадью,
    ножки — один монолит, ни одна не отвалится. После печати откусить кусачками."""
    W = (nx-1)*pitch + 16
    D = (ny-1)*pitch + 16
    parts = [ex(_rect(0, 0, W, D), th, 0)]        # сплошная база
    for i in range(nx):
        for j in range(ny):
            f = foot.copy()
            f.apply_translation([i*pitch + 8, j*pitch + 8, 0])
            parts.append(f)
    return fuse(parts)
build_plate([feet_raft()], "plate_feet_raft.stl")   # 24 ножки (20 нужных + 4 запас)
build_assembly(chas, gear, r1, r2, foot, stub)

print("\n".join(report))
print("\nГотово ->", OUT)
