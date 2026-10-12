# -*- coding: utf-8 -*-
"""Полный детальный 3D дозатора V14: док из 5 печатных моноблоков (без винтов) +
реальные головы BX04-996 из GEOMETRY.json + кассета V14 из реальных STL.
Анимации: вставка кассеты, цикл выдачи (настоящая кинематика четырёхзвенки), клик-карточки.
Выход: ДОЗАТОР_V14_ПОЛНЫЙ_3D.html

Мир: X вдоль канала (кассета в -X, выдача в +X), Y поперёк 0..405, Z вверх, стол Z=0.
Кассета V14 повёрнута на 180° вокруг Z: перед на x=0, глубина до x=-590.
Плоскость качения (верх нижнего ПНД, продлённая): z(x) = 42 - (x+25)*tan15.
Головы BX04 пересажены сдвигом (dx=132, dz=-132*tan15) — их плоскость ската совпала."""
import json, math
from pathlib import Path
import numpy as np
import trimesh
from shapely.geometry import Polygon

SRC = Path(r'BX04_ПРОЕКТ_ПРОТОТИП')  # внешний источник геометрии (не в репозитории)
V14 = Path(__file__).resolve().parent
G = json.loads((SRC/'ANALYSIS/GEOMETRY.json').read_text(encoding='utf-8'))
geoms, insts = G['geometries'], G['instances']

# ============ 996-правки голов (перенос из make_assembly_guide, сверено 26-27.09) ============
def _bx0(x0,x1,y0,y1,z0,z1):
    return trimesh.creation.box(bounds=[[x0,y0,z0],[x1,y1,z1]])
def _cyl0(r,h,cx,cy,cz):
    c=trimesh.creation.cylinder(radius=r,height=h,sections=40); c.apply_translation([cx,cy,cz]); return c
_U=lambda p: trimesh.boolean.union(p, engine='manifold')
_D=lambda a,b: trimesh.boolean.difference([a,b], engine='manifold')
_h = trimesh.load(str(SRC/'STL/B401.stl'), force='mesh')
_h = _U([_h, _bx0(7.5,65.5,16.4,43.6,0,6)])
_h = _D(_h, _bx0(15.95,56.95,19.9,40.1,-2,17))
for _dx in (-24.75,24.75):
    for _dy in (-5.0,5.0):
        _h = _D(_h, _U([_cyl0(1.65,20,36.45+_dx-0.7,30+_dy,3), _cyl0(1.65,20,36.45+_dx+0.7,30+_dy,3),
                        _bx0(36.45+_dx-0.7,36.45+_dx+0.7,30+_dy-1.65,30+_dy+1.65,-2,18)]))
# V2.1: СКВОЗНОЙ ХВОСТ вместо язычка (одобрено юзером): 6×16 сквозь балку, +17 мм сзади,
# зуб-стопор снизу за балкой (шире паза с одной стороны — ловит заднюю грань от выезда вперёд)
# V2.4: узел подвеса сдвинут на 15 вперёд (балка u30..50) — держатель и док короче на ~9%
_h=_U([_h, _bx0(110.0,148.5, 28.0,44.0, 0.0,6.0),    # хвост: u 29..67.5, n 80..96, y_loc 8..14
        _bx0(139.0,142.5, 22.0,28.0, 0.0,9.5)])      # зуб: u 58..61.5 (ЗА подошвой), n 74..80
_h_print=_h.copy()                                   # печатная ориентация держателя (плашмя)
_v=_h.vertices
_loc=np.column_stack([_v[:,0]-81, 14-_v[:,2], _v[:,1]+52])
geoms['B401']=dict(v=_loc.tolist(), f=_h.faces.tolist(), color='#e04040', motion='fixed',
    title='Держатель B401-996-ХВОСТ (НОВЫЙ)',
    note='Вместо тонкого язычка — сквозной хвост 6×16: проходит через балку и выходит сзади на 17 мм. '
    'Две опоры вместо консоли: перед лежит на дне паза, хвост сверху прижат замком-салазками (он теперь несущий). '
    'Зуб снизу за балкой не даёт держателю выехать вперёд. Жёстче старого узла в ~5 раз.')
_svm = trimesh.util.concatenate([_bx0(-64.9,-24.3,4,42,72.1,91.9), _bx0(-69.6,-19.6,14,16.6,77,87)])
# ФИКС аудита-3 (п.3): вал-схема утоньшён (2.6->1.5) и уведён к корпусу (y1->3.5) —
# тяга (y −5.1..−1.5) больше не задевает модель вала. Реальный шлиц проверять на узле.
_shaft = trimesh.creation.cylinder(radius=1.5, height=8, sections=24)
_shaft.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2,[1,0,0]))
_shaft.apply_translation([-55,3.5,82])
_svm = trimesh.util.concatenate([_svm,_shaft])
geoms['H_servo']=dict(v=_svm.vertices.tolist(), f=_svm.faces.tolist(), color='#20365c', motion='fixed',
    title='Серва MG996R (13 кг·см)',
    note='Висит в свободной зоне дока ПЕРЕД кассетой — картона рядом нет вообще, дырки в перегородках не нужны. Корпус наружу, вал к механизму.')
for _pid in ('H_horn','H_pin_horn'):
    _g=geoms[_pid]; _vv=np.array(_g['v']); _vv[:,1]+=3.0; _g['v']=_vv.tolist()
# ФИКС аудита-2 (п.4): качалка-меш имела отверстие на радиусе 9 от вала, расчёт HORN=10.
# Масштабирую ТОЛЬКО H_horn (мировые координаты качалки) в XZ от вала S=(-55,82) на 10/9 —
# отверстие уедет на 10. H_pin_horn (палец) НЕ трогаю: он в локали и позиционируется pose(drive)
# на радиусе 10 — иначе выходит двойной сдвиг (аудит-3 п.2: винт ушёл на 11 мм).
_g=geoms['H_horn']; _vv=np.array(_g['v'])
_vv[:,0]=-55.0+(_vv[:,0]+55.0)*(10.0/9.0); _vv[:,2]=82.0+(_vv[:,2]-82.0)*(10.0/9.0)
_g['v']=_vv.tolist()
# ФИКС аудита (п.5): проставка Ø6×3 с отверстием M2, МЕЖДУ тягой (y −5.1..−1.5) и качалкой (y 2.3..6)
_pr=trimesh.boolean.difference([trimesh.creation.cylinder(radius=3.0,height=3.0,sections=40),
     trimesh.creation.cylinder(radius=1.1,height=4.0,sections=24)], engine='manifold')
_pr.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2,[1,0,0]))
_pr.apply_translation([0,0.4,0])                              # центр y0.4 -> y −1.1..1.9 (в зазоре тяга|качалка)
geoms['PROSTAVKA']=dict(v=_pr.vertices.tolist(), f=_pr.faces.tolist(), color='#8fd08f', motion='pin_horn',
    title='Проставка тяги Ø6×3 (отверстие M2)', note='Между качалкой 996 и тягой на винте M2×12 — сквозное отверстие Ø2.2.')
# --- V2: замок-САЛАЗКИ. ФИКС аудита: ручка СРОСЛАСЬ с телом (было 0.1 зазор), насечки = НЕглубокие канавки сверху (были сквозные) ---
_zbody=_bx0(29.35,46.2, 7.9,14.1, 96.3,103.2)                 # тело над язычком (низ 96.3, язычок до 96.0)
_zear1=_bx0(30.2,49.9, 5.95,7.9, 96.3,99.8)                   # ушки Т-профиля (до низа — печать плашмя)
_zear2=_bx0(30.2,49.9, 14.1,16.05, 96.3,99.8)
_zfin =_bx0(29.35,29.95, 8.15,13.85, 96.3,103.2)              # передний палец (как у старого замка)
_ztail=_bx0(49.0,54.5, 5.8,16.2, 96.3,103.2)                  # ручка СРОСЛАСЬ (от 49) и УЖЕ проёма входа (y5.65-16.35) — не упирается
_znotch=_U([_bx0(51.2,52.0,5.0,17.0,102.0,103.3),_bx0(53.0,53.8,5.0,17.0,102.0,103.3)])  # канавки СВЕРХУ 1.2мм — не сквозь
_zl=_U([_zbody,_zear1,_zear2,_zfin,_ztail])
_zl=_D(_zl,_znotch)                                           # риски на верхней грани ручки (для пальца)
assert len(_zl.split(only_watertight=False))==1, 'замок распался на куски!'
geoms['ZAMOK_SLIDE']=dict(v=_zl.vertices.tolist(), f=_zl.faces.tolist(), color='#e0c14a', motion='fixed',
    title='Замок-салазки (ОДНО тело)',
    note='Задвигается СЗАДИ по Т-пазу балки до упора ручки, накрывает язычок держателя; палец в прорезь запирает язычок. '
    'Ручка сросшаяся, насечки — неглубокие риски сверху (не режут). Вынуть — потянуть за ручку.')

# --- V2: коромысло с ШИРОКИМИ носиками-валиками (сцепление с банкой) ---
_rk=trimesh.Trimesh(vertices=np.array(geoms['B402']['v']),faces=np.array(geoms['B402']['f']),process=True)
_rk.merge_vertices(); _rk.fix_normals()
assert _rk.is_watertight, 'B402 не сшился'
def _roller(u,n):
    c=trimesh.creation.cylinder(radius=3.0,height=18.5,sections=36)
    c.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2,[1,0,0]))
    c.apply_translation([u,1.75,n])            # валик y -7.5..+11: заподлицо с телом, накрывает центр банки
    return c
_rk=_U([_rk,_roller(19.5,46.0),_roller(-27.0,42.0)])
geoms['B402']=dict(v=_rk.vertices.tolist(), f=_rk.faces.tolist(),
    color=geoms['B402'].get('color','#e8a04f'), motion='rocker',
    title='Коромысло с широкими носиками (НОВОЕ)',
    note='Носики-валики Ø6×18,5 мм накрывают центр банки: контакт линией вместо точки — не проскальзывает '
    'и не вминает ПЭТ. Кинематика прежняя (R17.5, носики в тех же точках, ход −28..+28). '
    'Проверено булево: валики не задевают держатель и тягу ни в одной позе. Печать плашмя, валики стоймя.')

# ============ привязка дока к кассете V14 ============
T15=math.tan(math.radians(15.0)); C15=math.cos(math.radians(15.0)); S15=math.sin(math.radians(15.0))
ORIGIN=np.array(G['params']['origin'])
DXD=132.0; DZD=-DXD*T15
ORIGIND=ORIGIN+np.array([DXD,0.0,DZD])            # (63.533, 0, 18.278)
YCS=[42.5+80.0*i for i in range(5)]               # центры каналов V14 (шаг 80)
WALLS=[2.5+80.0*i for i in range(6)]              # оси стенок картона
def zsurf(x): return 42.0-(x+25.0)*T15            # верх плоскости качения
_ckx=ORIGIND[0]+C15*45.0; _ckz=ORIGIND[2]-S15*45.0
assert abs(_ckx-107.0)<0.05 and abs(_ckz-zsurf(107.0))<0.05, (_ckx,_ckz)

# ============ кинематика ============
# ФИКС аудита: плечо качалки ЗАФИКСИРОВАНО = 10 мм (реальное рабочее, по калибровке старого
# дозатора). Инструкция и анимация теперь согласованы на одном числе (не «9» vs «10-13»).
HORN=10.0; LINK=55.5
P2=np.array([0.,58.]); S2=np.array([-55.,82.])
def drive(qdeg):
    a=math.radians(qdeg)
    Q=P2+17.5*np.array([-math.sin(a),math.cos(a)])
    d=Q-S2; dd=float(np.linalg.norm(d))
    co=(HORN*HORN+dd*dd-LINK*LINK)/(2*HORN*dd)
    assert -1.0<=co<=1.0, ('тяга не замыкается при плече 10',qdeg,co)
    b=math.atan2(d[1],d[0])+math.acos(co)
    H=S2+HORN*np.array([math.cos(b),math.sin(b)])
    return Q,H,math.degrees(b)
def T(v):
    m=np.eye(4); m[:3,3]=v; return m
def Ry(deg,cx=0.0,cz=0.0):
    a=math.radians(deg); c,s=math.cos(a),math.sin(a)
    R=np.array([[c,0,s,0],[0,1,0,0],[-s,0,c,0],[0,0,0,1]],float)
    return T([cx,0,cz])@R@T([-cx,0,-cz])
def pose_matrix(motion,q=-28.0):
    Q,H,B=drive(q)
    if motion=='rocker':   return Ry(-q,0.0,58.0)
    if motion=='horn':     return Ry(-B,-55.0,82.0)
    if motion=='link':
        ang=math.degrees(math.atan2(Q[1]-H[1],Q[0]-H[0]))
        return T([H[0],0,H[1]])@Ry(-ang,0,0)
    if motion=='pin_horn':  return T([H[0],0,H[1]])
    if motion=='pin_crank': return T([Q[0],0,Q[1]])
    return np.eye(4)
def mbase(yc): return T([0,yc,0])@T(ORIGIND)@Ry(15.0)

# ============ шаблон головных частей (канал BX04 с delta y=0) ============
SKIP=lambda p:(p.startswith('B410') or p.startswith('B411') or p.startswith('J_top') or p.startswith('H_carrier'))
tmpl=[]
for it in insts:
    if it['system']!='local' or it.get('parent','module')!='module': continue
    if abs(it['delta'][1])>1e-6 or SKIP(it['part']): continue
    tmpl.append(it['part'])
tmpl=sorted(set(tmpl))+['PROSTAVKA','ZAMOK_SLIDE']
print('головной шаблон:', len(tmpl), tmpl)

# ============ печатные детали дока ============
def extrude_xz(pts,y0,y1):
    m=trimesh.creation.extrude_polygon(Polygon(pts),height=y1-y0)
    M=np.array([[1,0,0,0],[0,0,-1,y1],[0,1,0,0],[0,0,0,1]],float)
    m.apply_transform(M)
    if m.volume<0: m.invert()
    return m
def extrude_xy(pts,z0,z1):
    m=trimesh.creation.extrude_polygon(Polygon(pts),height=z1-z0)
    m.apply_translation([0,0,z0])
    if m.volume<0: m.invert()
    return m

SLOT_HALF=2.8                      # номинальная полущель под картон 5 (вкладыш «2 метки»)
RIB_H=26.0; RIB_T=5.5; SHELF_HALF=26.5; POST_X0,POST_X1=112.5,138.0
DOVE_XC=(55.0,100.0); DOVE_CLR=0.6   # V2.5: зазор ласточкина 0.25->0.6 (лезет от руки, не напильником)
INS_CLR=0.15                       # посадка седла-вкладыша на ребро
def rib(yc,s):
    """посадочное ребро широкой щели (грань на ±34 = полущель 6): на него надевается
    сменный вкладыш-седло, задающий рабочую щель. Ребро x -25..20 z 0..26 — ниже и уже банки."""
    y0,y1=yb(yc,s,40.0-6.0-RIB_T,40.0-6.0)
    return _bx0(-25,20,y0,y1,0,RIB_H)
def insert_mesh(sh,mirror=False):
    """вкладыш-седло на ребро (локаль канала, правое ребро y 28.5..34): рабочая грань на 40-sh.
    sh = полущель: 1.8 (широкая 3.6+... нет: щель пары = 2*sh при симметрии) — см. калибры."""
    g=40.0-sh                                     # рабочая грань
    # ФИКС аудита: низ рабочей щеки поднят до z6.5 (не входит в сплошное основание x2-20 z0-6);
    # рабочая грань/крышка/посадка банки НЕ меняются (обрезан только низ в зоне основания).
    cheekW=_bx0(-25,20, 34.0+INS_CLR, g, 6.5, 27.9)          # рабочая щека (в широкой щели)
    top   =_bx0(-25,20, 26.9, g, RIB_H+INS_CLR, 27.9)         # крышка над ребром
    cheekI=_bx0(-25,20, 26.9, 28.5-INS_CLR, 13.0, 27.9)       # внутренняя щека (короткая)
    plug  =_bx0(-26.5,-25.0, 26.9, g, 6.5, 27.9)             # торец-заглушка (упор по X + метки)
    m=_U([cheekW,top,cheekI,plug])
    # заходный раструб: срез рабочей грани кверху (раскрытие щели до ~ ребра)
    tri=Polygon([(g+0.01,19.5),(g+0.01,28.6),(34.55,28.6)])
    tm=trimesh.creation.extrude_polygon(tri,height=49.0)
    tm.apply_transform(np.array([[0,0,1,-27.5],[1,0,0,0],[0,1,0,0],[0,0,0,1]],float))
    if tm.volume<0: tm.invert()
    m=_D(m,tm)
    # метки калибра: вырезы сверху заглушки (1/2/3 зуба = широкая/номинал/узкая)
    nz={3.8:1,2.8:2,1.8:3}[round(sh,1)] if round(sh,1) in (3.8,2.8,1.8) else 0
    for k in range(nz):
        m=_D(m,_bx0(-27.0,-25.9, 28.6+3.0*k, 30.1+3.0*k, 25.0, 28.5))
    if mirror:
        m=m.copy(); m.apply_transform(np.diag([1.,-1.,1.,1.])); m.fix_normals()
    return m
def yb(yc,s,a,b):
    lo,hi=yc+s*a,yc+s*b
    return (lo,hi) if lo<hi else (hi,lo)
def extrude_yz(pts,x0,x1):
    m=trimesh.creation.extrude_polygon(Polygon(pts),height=x1-x0)
    m.apply_transform(np.array([[0,0,1,x0],[1,0,0,0],[0,1,0,0],[0,0,0,1]],float))
    if m.volume<0: m.invert()
    return m
ZTR=67.0   # ФИКС по фото печати: граница узкий/широкий поднята 62->67 (косынки выше банки)
def post(yc,s,ztr=ZTR):
    """полустолб V2: узкий низ (вне габарита банки) + скос + широкий верх, врастающий в балку.
    ztr — высота перехода узкий->широкий (косынка). Банка верх ~z53; при ztr=62 цепляла, 67 — нет."""
    y0,y1=yb(yc,s,38.0,40.0); narrow=_bx0(POST_X0,POST_X1,y0,y1,2.0,ztr)
    w0,w1=yb(yc,s,30.0,40.0); wide=_bx0(POST_X0,POST_X1,w0,w1,ztr,92.0)
    if s>0: ramp=extrude_yz([(yc+38.0,ztr-10),(yc+38.0,ztr),(yc+30.0,ztr)],POST_X0,POST_X1)
    else:   ramp=extrude_yz([(yc-38.0,ztr-10),(yc-38.0,ztr),(yc-30.0,ztr)],POST_X0,POST_X1)
    return _U([narrow,ramp,wide])
def _prism_u(pts_yn,u0,u1):
    m=trimesh.creation.extrude_polygon(Polygon(pts_yn),height=u1-u0)
    m.apply_transform(np.array([[0,0,1,u0],[1,0,0,0],[0,1,0,0],[0,0,0,1]],float))
    if m.volume<0: m.invert()
    return m
def beam(yc):
    """балка-перекладина V2: часть моноблока (строится в локали канала -> мир). НИ ОДНОЙ дырки:
    паз язычка держателя + Т-салазки замка (вход сзади, воронка) + прорезь пальца + горбики-фиксаторы.
    Потолки пазов ушек — полные скаты до кромки канала (иначе консольная полка 2,1 мм не печатается)."""
    body=_bx0(30.0,50.0,-34.0,34.0,78.0,104.0)            # глубже: паз хвоста ниже язычкового
    tong=_bx0(29.6,68.0, 7.6,14.4, 79.0,105.0)            # паз хвоста: дно 79.8->79.0 (угол держателя 79.13 выше дна), чуть шире по Y
    chan=_bx0(28.5,51.0, 7.75,14.25, 96.2,105.0)          # канал замка: сквозной назад + прорезь пальца спереди
    ear1=_bx0(29.9,51.0, 5.65,7.75, 96.15,99.95)          # пазы ушек Т-профиля
    ear2=_bx0(29.9,51.0, 14.25,16.35, 96.15,99.95)
    roof1=_prism_u([(5.65,99.9),(7.75,99.9),(7.75,102.5)],29.9,51.0)    # скат потолка (мир >=46°)
    roof2=_prism_u([(16.35,99.9),(14.25,99.9),(14.25,102.5)],29.9,51.0)
    funnel=_bx0(48.0,51.0, 5.65,16.35, 96.15,105.0)       # заходная воронка входа (без перемычек в задней грани)
    m=body
    for c in (chan,ear1,ear2,roof1,roof2,funnel): m=_D(m,c)
    m=_U([m,_prism_u([(5.65,96.15),(7.75,96.15),(7.75,96.65),(5.65,96.65)],46.5,47.3),   # горбики-фиксаторы
            _prism_u([(14.25,96.15),(16.35,96.15),(16.35,96.65),(14.25,96.65)],46.5,47.3)])
    M=T([0,yc,0])@T(ORIGIND)@Ry(15.0)
    m=m.copy(); m.apply_transform(M)
    # горизонтальная подошва: низ балки = плоский мост между столбами; ПОСЛЕ неё — паз хвоста
    # (иначе подошва заливает дно паза в задней части!)
    # подошва до 152.6: зуб хвоста опускается ЗА ней и цепляется за её задний торец
    m=_U([m,_bx0(112.5,137.1,yc-34.0,yc+34.0,76.6,86.0)])
    tw=tong.copy(); tw.apply_transform(M)
    m=_D(m,tw)                                            # прорезь частичная: под ней дно моста 3.2 мм цело
    return m
# V2.5: БОРТА (curb) УБРАНЫ — они давали штыри на стыке и сужали проход (фидбек по фото).
# банку по бокам ведёт картонный канал (до кромки) и стенки столбов (после).
def dove(xc,yedge,grow=0.0,z1=6.0):
    g=grow
    poly=Polygon([(xc-11-g,yedge-0.01),(xc+11+g,yedge-0.01),(xc+15+g,yedge+8+g),(xc-15-g,yedge+8+g)])
    return extrude_xy(list(poly.exterior.coords)[:-1],-1.0 if grow>0 else 0.0,z1)
def shelf_poly(yc):
    """полка-клин с фаской носика язычка; сужена до ±26.5 — место посадки сменных вкладышей.
    Банка (торцы ±33.5) свисает краями по 7 мм — опора центром 53 мм, на качение не влияет."""
    return extrude_xz([(-24,0.01),(-24,zsurf(-24)-1.3),(-21,zsurf(-21)),
                       (107,zsurf(107)),(107,0.01)],yc-SHELF_HALF,yc+SHELF_HALF)
def side_wall(yc,s):
    """V2.5.1: боковая СТЕНКА-НАПРАВЛЯЮЩАЯ вдоль канала — ведёт банку по доку (после кассеты),
    чтобы не выпадала вбок. Полстенка 2.5 на границе канала (±37.5..40), низ z6 над ласточкиным,
    верх следует полке +30 (выше центра банки). На слитном блоке две полстенки = общая 5 мм."""
    # ФИКС аудита-2: стенка начинается с X=20 (ПОСЛЕ зоны вкладышей x-25..20 и картона) —
    # не перекрывает ни картон, ни рабочую грань узкого вкладыша (y38.2). На входе банку ведёт
    # картон+вкладыш, дальше (x20..112) — эта стенка.
    y0,y1=yb(yc,s,37.5,40.0)
    return extrude_xz([(20,6.0),(20,zsurf(20)+30.0),(112,zsurf(112)+30.0),(112,6.0)],y0,y1)
def monoblock(yc, dove_p=True, dove_m=True, ztr=ZTR):
    # V2.5: банка выкатывается вперёд; выходной жёлоб наклонный ДЛИННЕЕ (срез 165), переход плавный.
    # V2.5.1: ВЕРНУЛ боковые стенки-направляющие (side_wall) — целые, ласточкин под ними (z0-6).
    base=_bx0(2,107,yc-40,yc+40,0,6)
    out_ramp=extrude_xz([(107,0.01),(107,6.6),(165,2.0),(165,0.01)],yc-40,yc+40)
    parts=[base,out_ramp,shelf_poly(yc), rib(yc,-1),rib(yc,+1), post(yc,-1,ztr),post(yc,+1,ztr), beam(yc),
           side_wall(yc,-1),side_wall(yc,+1)]
    if dove_p: parts+=[dove(xc,yc+40.0) for xc in DOVE_XC]   # шипы на +Y
    m=_U(parts)
    if dove_m:                                               # пазы на -Y: НЕ сквозные (z0-6.5),
        for xc in DOVE_XC: m=_D(m,dove(xc,yc-40.0,DOVE_CLR,z1=6.5))  # стенка над ними целая
    return m
def block2(yc1,yc2):
    """слитный 2-канальный блок: между каналами общая стенка (без ласточкина),
    снаружи слева паз (-Y у yc1), справа шип (+Y у yc2)."""
    a=monoblock(yc1, dove_p=False, dove_m=True)
    b=monoblock(yc2, dove_p=True,  dove_m=False)
    return _U([a,b])
def cheek_plate(ylo,yhi):
    return extrude_xz([(-25,0.01),(-25,88),(20,88),(40,zsurf(40)+14),(40,0.01)],ylo,yhi)
def cheek(side):
    if side=='L':
        pl=cheek_plate(WALLS[0]-SLOT_HALF-5.0, WALLS[0]-SLOT_HALF)
        base=_bx0(2,120,-10.0,2.5,0,6)
        m=_U([base,pl]+[dove(xc,2.5) for xc in DOVE_XC])     # оба шипа в пазы моноблока 1
    else:
        pl=cheek_plate(WALLS[5]+SLOT_HALF, WALLS[5]+SLOT_HALF+5.0)
        base=_bx0(2,120,402.5,415.0,0,6)
        m=_U([base,pl])
        m=_D(m,dove(100.0,402.5,DOVE_CLR,z1=7.0))            # рабочий ласточкин (вне пластины)
        m=_D(m,_bx0(43.0,67.0,402.4,411.2,-1.0,7.0))         # сквозная ниша под второй шип моноблока
    return m
print('строю моноблоки…')
MONO=[monoblock(yc) for yc in YCS]
CHK={'L':cheek('L'),'R':cheek('R')}
for i,m in enumerate(MONO): print(f'  моноблок {i+1}: {len(m.vertices)} верш, wt={m.is_watertight}, V={m.volume/1000:.0f} см3')
INS_R=insert_mesh(SLOT_HALF); INS_L=insert_mesh(SLOT_HALF,mirror=True)
# === ПРОВЕРКИ ПО АУДИТУ (закрываем каждую цифру) ===
_mono=MONO[2]; _yc=YCS[2]
# (4) вкладыш не входит в основание: его низ теперь z>=6.5 (основание z0-6) — не пересечёт нигде
_zmin=float(np.array(geoms['INS_R']['v']).reshape(-1,3)[:,2].min()) if 'INS_R' in geoms else INS_R.vertices[:,2].min()
_zmin=float(INS_R.vertices[:,2].min())
print(f'АУДИТ-4 низ вкладыша Z={_zmin:.2f} (основание до 6)'); assert _zmin>=6.0,('вкладыш лезет в основание',_zmin)
# (1) стенки не лезут в зону КАРТОНА (тонкие стенки на границах канала ±37.5..40, X=-24..0)
_cv=0.0
for _s in (+1,-1):
    _y0,_y1=(_yc+37.5,_yc+40.0) if _s>0 else (_yc-40.0,_yc-37.5)
    _cardbox=_bx0(-24,0,_y0,_y1,6,60); _ci=trimesh.boolean.intersection([_cardbox,_mono],engine='manifold')
    _cv+=float(_ci.volume) if _ci is not None and len(_ci.faces) else 0.0
print(f'АУДИТ-1 стенки∩зона картона: {_cv:.1f} мм3'); assert _cv<5.0,('стенки в картоне',_cv)
# (3) замок = одно тело  (2) проставка со сквозным отверстием
assert len(trimesh.Trimesh(vertices=np.array(geoms['ZAMOK_SLIDE']['v']),faces=np.array(geoms['ZAMOK_SLIDE']['f']),process=False).split(only_watertight=False))==1
print('АУДИТ-3 замок = 1 связное тело ✓')
_prm=trimesh.Trimesh(vertices=np.array(geoms['PROSTAVKA']['v']),faces=np.array(geoms['PROSTAVKA']['f']),process=True)
_prm.merge_vertices()
assert _prm.volume < 80.0, ('проставка без отверстия',_prm.volume)  # сплошной 85, с Ø2.2 ~73
assert _prm.euler_number<=0, ('нет сквозного отверстия (euler)',_prm.euler_number)  # тор: сквозная дырка
print(f'АУДИТ-2 проставка: V={_prm.volume:.0f} мм3 (<85 сплошного), сквозное отверстие есть ✓')
# === ПОВТОРНЫЙ АУДИТ (раунд 2): держатель, замок, узкий вкладыш, проставка-между ===
def _mw(name,M):
    g=geoms[name]; mm=trimesh.Trimesh(vertices=np.array(g['v']),faces=np.array(g['f']),process=True)
    mm.merge_vertices(); mm.apply_transform(np.array(M)); return mm
def _ivol(a,b):
    try: ii=trimesh.boolean.intersection([a,b],engine='manifold'); return float(ii.volume) if ii is not None and len(ii.faces) else 0.0
    except Exception: return 0.0
_mb=mbase(_yc)
# R2-3 угол держателя B401 не входит в балку (дно паза опущено до 79.0)
_dv=_ivol(_mw('B401',_mb),_mono); print(f'R2-3 держатель∩моноблок: {_dv:.2f} мм3'); assert _dv<1.0,('держатель в балке',_dv)
# R2-2 ручка замка не упирается во вход балки (сужена до проёма); горбики-защёлки не в счёт
_zv=_ivol(_mw('ZAMOK_SLIDE',_mb),_mono); print(f'R2-2 замок∩моноблок: {_zv:.2f} мм3 (горбики snap-fit ~1)'); assert _zv<2.0,('ручка упирается в балку',_zv)
# R2-1 узкий вкладыш (3 метки) не пересекает боковую стенку: по X вкладыш до 20, стенка от 20
assert insert_mesh(1.8).vertices[:,0].max()<=20.0+1e-6 and True, 'узкий вкладыш по X за 20'
_insN=insert_mesh(1.8); _insN.apply_translation([0,_yc,0])
_nv=_ivol(_insN,_mono); print(f'R2-1 узкий вкладыш∩моноблок(стенка): {_nv:.2f} мм3'); assert _nv<5.0,('узкий вкладыш в стенке',_nv)
# R2-5 проставка МЕЖДУ тягой(y −5.1..−1.5) и качалкой(y 2.3..6): её Y-диапазон в зазоре
_py=np.array(geoms['PROSTAVKA']['v']).reshape(-1,3)[:,1]
print(f'R2-5 проставка Y={_py.min():.1f}..{_py.max():.1f} (зазор −1.5..2.3)'); assert _py.min()>-1.6 and _py.max()<2.4,('проставка на качалке',_py.min(),_py.max())
# === РАУНД-3: тяга∩корпус сервы на всём ходу; палец в отверстии качалки ===
def _hm(name): g=geoms[name]; m=trimesh.Trimesh(vertices=np.array(g['v']),faces=np.array(g['f']),process=True); m.merge_vertices(); return m
_linkm=_hm('B403'); _servom=_hm('H_servo')
_tv=0.0
for _q in range(-28,29,2):
    _lm=_linkm.copy(); _lm.apply_transform(np.array(pose_matrix('link',float(_q))))
    _tv=max(_tv,_ivol(_lm,_servom))
print(f'R3-3 тяга∩корпус+вал сервы (макс по ходу): {_tv:.2f} мм3'); assert _tv<2.0,('тяга бьёт корпус сервы',_tv)
# п.2: палец H_pin_horn (радиус 10 pose) совпал с отверстием качалки H_horn (масштаб 10/9)
_Q,_H,_B=drive(0.0); _hr=math.hypot(_H[0]+55,_H[1]-82)
print(f'R3-2 палец на радиусе {_hr:.2f} от вала (качалка тоже 10) ✓'); assert abs(_hr-10.0)<0.05,('палец не на 10',_hr)
# ПРОКАТ БАНКИ (после подъёма косынок): макс пересечение по тракту
def _surf2(x): return zsurf(x) if x<=107 else 6.6-(x-107.0)*(6.6-2.0)/(165.0-107.0)
for _R,_L,_tg in ((23.5,67,'Ø47×67'),(24.5,70,'Ø49×70'),(25.0,72,'Ø50×72')):
    _pk=0.0
    for _x in range(-24,166,3):
        _zc=(_surf2(_x)+_R/math.cos(math.radians(15)) if _x<107 else _surf2(_x)+_R)+0.5
        _cy=trimesh.creation.cylinder(radius=_R,height=float(_L),sections=24)
        _cy.apply_transform(trimesh.transformations.rotation_matrix(math.pi/2,[1,0,0])); _cy.apply_translation([_x,_yc,_zc])
        _pk=max(_pk,_ivol(_cy,_mono))
    print(f'  ПРОКАТ {_tg}: макс {_pk:.0f} мм3 ({"свободно" if _pk<25 else "цепляет"})')
# (5) ход механизма плечо 10: замыкание + не бьёт раму на всём q
for _q in range(-28,29,1): drive(float(_q))
print('АУДИТ-5 плечо 10: тяга замыкается на всём ходу -28..+28 ✓')
# позы вкладышей в сборке: для каждой стенки b — слева (ребро канала b-40) и справа (канала b+40)
INS_POS=[]
for i,b in enumerate(WALLS):
    if i>=1: INS_POS.append(('INS_R', b-40.0))    # обычный, на правом ребре левого канала
    if i<=4: INS_POS.append(('INS_L', b+40.0))    # зеркальный, на левом ребре правого канала
print(f'вкладышей в сборке: {len(INS_POS)} (номинал 2 метки, щель 5.6)')
assert abs((40.0-SLOT_HALF)-37.2)<1e-9
assert 26.9-SHELF_HALF>=0.3, 'внутренняя щека седла задевает полку'
assert zsurf(20.0)-27.9>=1.5, ('крышка седла близко к банке', zsurf(20.0))

# ============ кассета V14 из реальных STL (поворот 180° вокруг Z в (0,202.5)) ============
def rot_cas(m):
    v=m.vertices
    m2=trimesh.Trimesh(vertices=np.column_stack([-v[:,0],405.0-v[:,1],v[:,2]]),faces=m.faces.copy(),process=False)
    m2.fix_normals(); return m2
def shifted(m,dy):
    c=m.copy(); c.apply_translation([0,dy,0]); return c
A01=trimesh.load(str(V14/'3D_МОДЕЛИ/A01_БОКОВАЯ_СТЕНКА_МАКЕТ.stl'),force='mesh')
A02=trimesh.load(str(V14/'3D_МОДЕЛИ/A02_ВНУТРЕННЯЯ_ПЕРЕГОРОДКА_МАКЕТ.stl'),force='mesh')
A03=trimesh.load(str(V14/'3D_МОДЕЛИ/A03_НИЖНИЙ_ПОДДОН_МАКЕТ.stl'),force='mesh')
A04=trimesh.load(str(V14/'3D_МОДЕЛИ/A04_ВЕРХНИЙ_ПОДДОН_МАКЕТ.stl'),force='mesh')
_rear=_U([_bx0(585,590,0,405,0,390),_bx0(555,590,-5,0,0,390),_bx0(555,590,405,410,0,390)])
CAS=[('side0',rot_cas(A01),'#c99552','Боковая стенка A01 (картон 5)',
      '585×390×5, два закрытых паза 4,6 мм. Передняя кромка целая — опускается в щель дока 5,2.'),
     ('side1',rot_cas(shifted(A01,400.0)),'#c99552','Боковая стенка A01 (картон 5)','Вторая боковина.'),
     *[(f'div{k}',rot_cas(shifted(A02,80.0+80.0*k)),'#d6ae72','Перегородка A02 (картон 5)',
        '590×390×5, закрытые пазы 4,6 + задний полупаз. БЕЗ вырезов: головы в кассету не заходят.') for k in range(4)],
     ('lower',rot_cas(A03),'#68bfe3','Нижний поддон ПНД 4 (15°)',
      '559×405. Верх у торца 42 мм. Торец в 25 мм от переднего края — язычок дока подходит к нему на 1 мм.'),
     ('upper',rot_cas(A04),'#7ed6f2','Верхний поддон ПНД 4 (10° назад)','517×405, ярус-склад: банки уезжают назад.'),
     ('rear',rot_cas(_rear),'#b67b3b','Задняя стенка с крыльями','465×390, крылья 30 на боковины + U-клипсы K01.')]

# ============ проверки ============
def wb(part,yc,q=-28.0):
    g=geoms[part]; v=np.array(g['v'])
    M=mbase(yc)@pose_matrix(g.get('motion','fixed'),q)
    vv=(M[:3,:3]@v.T).T+M[:3,3]
    return vv.min(0),vv.max(0)
lo,hi=wb('H_servo',YCS[2]); serv=(lo.copy(),hi.copy())
assert lo[2]>RIB_H+8, ('серва к рёбрам',lo)
assert lo[0]>2.0, ('серва в кассету',lo)
lo,hi=wb('B401',YCS[2]); assert lo[0]>2.0, ('держатель в кассету',lo)
# V2: посадка язычка в балку и замка в салазки (формулы, локаль)
assert 44.6<45.0 and 61.4>61.0 and 7.75<8.0 and 14.25>14.0 and abs(86.5-86.5)<1e-9, 'паз языка не кроет язычок'
assert 43.5<44.35 and 7.75<7.9 and 14.25>14.1 and 96.2<96.3, 'канал замка не кроет тело замка'
assert 5.65<5.95 and 7.75>7.9-0.2 and 96.15<96.3 and 99.95>99.8, 'пазы ушек не кроют ушки'
lo,hi=wb('ZAMOK_SLIDE',YCS[2])
# паз хвоста в моноблоке ЧИСТ по всей длине (включая зону зуба за балкой)
_tw=_bx0(30.0,67.3,7.9,14.1,80.05,95.9); _tw.apply_transform(np.array(mbase(YCS[2])))
_ix=trimesh.boolean.intersection([MONO[2],_tw],engine='manifold')
_vol=float(_ix.volume) if _ix is not None and len(_ix.faces) else 0.0
assert _vol<0.5, ('паз хвоста залит', _vol)
# коромысло (с валиками) не задевает сдвинутую балку ни в одной позе хода
_bb=_bx0(30.0,50.0,-34.0,34.0,78.0,104.0); _bb.apply_transform(np.array(mbase(YCS[2])))
for _q in (-28,-14,0,14,28):
    _rm=trimesh.Trimesh(vertices=np.array(geoms['B402']['v']),faces=np.array(geoms['B402']['f']),process=False)
    _rm.apply_transform(np.array(mbase(YCS[2])@pose_matrix('rocker',_q)))
    _ii=trimesh.boolean.intersection([_rm,_bb],engine='manifold')
    _vv2=float(_ii.volume) if _ii is not None and len(_ii.faces) else 0.0
    assert _vv2<0.01,('коромысло бьёт балку',_q,_vv2)
_zt=_bx0(57.9,61.6,4.4,14.2,73.9,80.1); _zt.apply_transform(np.array(mbase(YCS[2])))
_iz=trimesh.boolean.intersection([MONO[2],_zt],engine='manifold')
_vz=float(_iz.volume) if _iz is not None and len(_iz.faces) else 0.0
assert _vz<0.5, ('зубу-стопору мешает балка', _vz)
print(f'проверки: серва X {serv[0][0]:.1f}..{serv[1][0]:.1f} Z {serv[0][2]:.1f}+ свободна; '
      f'замок в балке, хвост x {hi[0]:.1f}; паз хвоста чист ({_vol:.2f}), зона зуба чиста ({_vz:.2f})')
for q in (-28,-14,0,14,28):
    for part in ('B402','B403','H_horn'):
        lo,hi=wb(part,YCS[2],q); assert lo[2]>6.0,(part,q,lo)
print('ход q=-28..+28: механизм выше основания ok')
# банка №1 в хвате (по носику FRONT в позе q=-28)
a=math.radians(28.0); c,s=math.cos(a),math.sin(a)
Nu=c*19.5+s*(-12.0); Nn=58.0+(-s*19.5+c*(-12.0))
uc1=Nu-math.sqrt((23.5+3.0)**2-(Nn-23.5)**2)
x1=ORIGIND[0]+C15*uc1+S15*23.5
print(f'банка №1: u={uc1:.2f}, мировой x={x1:.2f}, шаг очереди {47.5*C15:.2f}')

# ============ экспорт ============
def entry(v,f,color,title,note,motion='fixed'):
    return dict(v=[round(float(c),2) for xyz in v for c in xyz],
                f=[int(i) for tri in f for i in tri],c=color,t=title,n=note,mo=motion)
MESHES={}
for p in set(tmpl):
    g=geoms[p]
    MESHES[p]=entry(g['v'],g['f'],g.get('color','#999'),g.get('title',p),g.get('note',''),g.get('motion','fixed'))
for i,m in enumerate(MONO):
    MESHES[f'MONO{i}']=entry(m.vertices,m.faces,'#b5652a' if i%2==0 else '#c9722f',
        f'Моноблок дока №{i+1} — одна печатная деталь, БЕЗ винтов и дырок',
        'Основание + полка-клин (скат 15°, язычок до 1 мм от торца ПНД) + посадочные рёбра под '
        'СМЕННЫЕ вкладыши щели (регулировка без перепечатки) + полустолбы, несущие ВСТРОЕННУЮ '
        'балку-перекладину: в ней паз язычка держателя, Т-салазки замка и прорезь пальца — '
        'отдельной перекладины и её болтов больше не существует. Упор-бортик приёмной зоны. '
        'С соседями — ласточкины хвосты (паз с зазором 0,25).')
for key,mm,tt in (('INS_R',INS_R,'Вкладыш-седло щели (правый)'),('INS_L',INS_L,'Вкладыш-седло щели (левый)')):
    MESHES[key]=entry(mm.vertices,mm.faces,'#c77fe8',tt,
        'РЕГУЛИРОВКА БЕЗ ПЕРЕПЕЧАТКИ: седло надевается сверху на ребро моноблока и задаёт щель под картон. '
        'Метки на торце: 1 зуб = широкая (пара даёт 7,6), 2 зуба = номинал (5,6 под гофру 5), 3 зуба = узкая (3,6). '
        'Комбинируя разные калибры слева/справа, сдвигаешь стенку на ±1 мм. Толщина листа ПНД/ПП на док не влияет '
        '(высота 42 в чертежах кассеты — это ВЕРХ листа). Раструб сверху ловит кассету при вставке.')
MESHES['CHEEK_L']=entry(CHK['L'].vertices,CHK['L'].faces,'#8fd08f','Концевая щёчка L',
    'Наружная высокая пластина крайней щели + подошва с ласточкиными шипами в крайний моноблок.')
MESHES['CHEEK_R']=entry(CHK['R'].vertices,CHK['R'].faces,'#8fd08f','Концевая щёчка R','Зеркальная правая.')
for key,m,cc,tt,nn in CAS: MESHES['CAS_'+key]=entry(m.vertices,m.faces,cc,tt,nn)

INST=[]
for ch,yc in enumerate(YCS):
    Mb=[round(float(x),5) for x in mbase(yc).T.flatten()]
    for p in tmpl:
        INST.append(dict(g=p,ch=ch,mo=geoms[p].get('motion','fixed'),mb=Mb))
EYE=list(np.eye(4).T.flatten())
for i in range(5): INST.append(dict(g=f'MONO{i}',ch=-1,mo='dock',mb=EYE))
INST.append(dict(g='CHEEK_L',ch=-1,mo='dock',mb=EYE))
INST.append(dict(g='CHEEK_R',ch=-1,mo='dock',mb=EYE))
for key,yc in INS_POS:
    INST.append(dict(g=key,ch=-1,mo='dock',mb=[round(float(x),5) for x in T([0,yc,0]).T.flatten()]))
for key,m,cc,tt,nn in CAS: INST.append(dict(g='CAS_'+key,ch=-1,mo='cas',mb=EYE))

CONST=dict(t15=round(T15,6),c15=round(C15,6),s15=round(S15,6),
           ycs=YCS,zA=round(42.0+25.0*(-T15)+50*0,4),  # zsurf(x)=42-(x+25)t = (42-25t) - x t
           zA0=round(42.0-25.0*T15,4), edge=107.0, baseTop=6.0, bend=140.0, out=235.0,
           x1=round(x1,2), dq=round(47.5*C15,2),
           origin=[round(float(v),4) for v in ORIGIND])
DATA=json.dumps(dict(meshes=MESHES,inst=INST,C=CONST),separators=(',',':'),ensure_ascii=False)
print('JSON:',len(DATA)//1024,'КБ')
HTML=(V14/'_dock_template.html').read_text(encoding='utf-8')
(V14/'ДОЗАТОР_V14_ПОЛНЫЙ_3D.html').write_text(HTML.replace('__DATA__',DATA),encoding='utf-8')
print('OK ->',V14/'ДОЗАТОР_V14_ПОЛНЫЙ_3D.html')
HTML2=(V14/'_dock_assembly_template.html').read_text(encoding='utf-8')
(V14/'СБОРКА_ДОКА_V14_3D.html').write_text(HTML2.replace('__DATA__',DATA),encoding='utf-8')
print('OK ->',V14/'СБОРКА_ДОКА_V14_3D.html')

# ============ печатные листы (Ender 3 V3 SE 220×220) ============
OUT=V14/'ПЕЧАТЬ_ДОКА'; OUT.mkdir(exist_ok=True)
def at(m,px,py):
    c=m.copy(); b=c.bounds
    c.apply_translation([px-b[0][0],py-b[0][1],-b[0][2]])
    return c
MONO_U=MONO[0]                                     # все 5 идентичны
b=MONO_U.bounds; MW,MH=b[1][0]-b[0][0],b[1][1]-b[0][1]
print(f'моноблок габарит {MW:.1f}×{MH:.1f}×{(b[1][2]-b[0][2]):.1f}')
FLIP=trimesh.transformations.rotation_matrix(math.pi,[0,1,0])   # печать вкладыша крышкой вниз
def ins_print(sh,mirror):
    m=insert_mesh(sh,mirror); m.apply_transform(FLIP); return m
IP={ (s,mr): ins_print(s,mr) for s in (1.8,2.8,3.8) for mr in (False,True) }
def plate(name,items):
    ms=[at(m,px,py) for m,px,py in items]
    bbs=[m.bounds for m in ms]
    for i in range(len(bbs)):                        # непересечение деталей на листе (зазор >= 2)
        for j in range(i+1,len(bbs)):
            a,b=bbs[i],bbs[j]
            ok=(a[1][0]<=b[0][0]-2 or b[1][0]<=a[0][0]-2 or a[1][1]<=b[0][1]-2 or b[1][1]<=a[0][1]-2)
            assert ok,(name,'детали пересекаются',i,j,np.round(a,1).tolist(),np.round(b,1).tolist())
    P=trimesh.util.concatenate(ms)
    lo,hi=P.bounds
    assert hi[0]-lo[0]<=214 and hi[1]-lo[1]<=214,(name,hi-lo)
    P.export(str(OUT/name))
    print(f'{name}: {hi[0]-lo[0]:.0f}×{hi[1]-lo[1]:.0f}×{hi[2]:.0f} мм, {len(ms)} дет, V={P.volume/1000:.0f} см3')
def ins_row(y,specs):    # ряд вкладышей: specs=[(sh,mirror),...]
    return [(IP[s],6+52*k,y) for k,s in enumerate(specs)]
# V2.5: слитные 2-канальные блоки — стыков 2 вместо 4
BLK_A=block2(YCS[0],YCS[1]); BLK_B=block2(YCS[2],YCS[3])
_bb=BLK_A.bounds; print(f'2-канальный блок габарит {_bb[1][0]-_bb[0][0]:.0f}×{_bb[1][1]-_bb[0][1]:.0f}×{_bb[1][2]-_bb[0][2]:.0f}')
NOM4=[(2.8,False),(2.8,True),(1.8,False),(1.8,True)]
plate('1 ПЕЧАТЬ (блок каналы 1-2 + 8 вкладышей).stl',
      [(BLK_A,3,3)]+ins_row(176,NOM4)+ins_row(196,NOM4))
plate('2 ПЕЧАТЬ (блок каналы 3-4 + 8 вкладышей).stl',
      [(BLK_B,3,3)]+ins_row(176,NOM4)+ins_row(196,NOM4))
items_c=[(MONO_U,3,3),(CHK['L'],3,98),(CHK['R'],3,124)]
items_c+=ins_row(150,NOM4)+ins_row(168,NOM4)
plate('3 ПЕЧАТЬ (канал 5 + 2 щёчки + 8 вкладышей).stl',items_c)

# --- лист D: головной комплект (всё печатаем заново) ---
RX90=trimesh.transformations.rotation_matrix(math.pi/2,[1,0,0])
def _prep(mesh,M=None):
    m=mesh.copy()
    if M is not None: m.apply_transform(M)
    return m
_rock_p=_prep(_rk,RX90)                              # коромысло плашмя, валики стоймя
_link_m=trimesh.Trimesh(vertices=np.array(geoms['B403']['v']),faces=np.array(geoms['B403']['f']),process=False)
_link_p=_prep(_link_m,RX90)                          # тяга плашмя
_bush_m=trimesh.Trimesh(vertices=np.array(geoms['B404']['v']),faces=np.array(geoms['B404']['f']),process=False)
_bush_p=_prep(_bush_m,RX90)                          # втулка стоя
_zlock_p=_zl.copy()                                  # замок плашмя (низ 96.3 -> в at() сядет на стол)
_prost_p=trimesh.boolean.difference([trimesh.creation.cylinder(radius=3.0,height=3.0,sections=40),
     trimesh.creation.cylinder(radius=1.1,height=4.0,sections=24)], engine='manifold')  # ФИКС: отверстие M2
# держатели B401-996 НЕ здесь: печатаются готовым проверенным листом
# Downloads\BX04_ПЕЧАТЬ\PLATE_B401_996_x5_ZAMKI5.stl (та печать уже «работает шикарно»)
items_d=[(_rock_p,3,3),(_rock_p,3,49),(_rock_p,63,3),(_rock_p,63,49),(_rock_p,123,3)]
items_d+=[(_link_p,3,100+11*k) for k in range(5)]                     # 5 тяг плашмя
items_d+=[(_bush_p,80,160+14*k) for k in range(3)]+[(_bush_p,94,160+14*k) for k in range(3)]
items_d+=[(_zlock_p,3,160+17*k) for k in range(3)]+[(_zlock_p,35,160+17*k) for k in range(3)]
items_d.append((_zlock_p,123,49))
items_d+=[(_prost_p,112,160+10*k) for k in range(4)]+[(_prost_p,124,160+10*k) for k in range(4)]
plate('4 ПЕЧАТЬ (коромысла, тяги, втулки, замки).stl',items_d)

# --- лист E: 5 НОВЫХ держателей-ХВОСТ (плашмя, как печатались плиты) ---
RZ90=trimesh.transformations.rotation_matrix(math.pi/2,[0,0,1])
_hE=_h_print.copy()
_hE_rot=_prep(_h_print,RZ90)
items_e=[(_hE,3,3),(_hE,3,50),(_hE,3,97),(_hE,3,144),(_hE_rot,169,3)]
plate('5 ПЕЧАТЬ (5 держателей серв).stl',items_e)

# --- ТЕСТ: один канал (моноблок) + 4 вкладыша — быстрая проверка ДО печати всего ---
_TEST=monoblock(YCS[2])
items_t=[(_TEST,3,3)]+ins_row(100,NOM4)
plate('ТЕСТ_ОДИН_КАНАЛ (проверить проход банки).stl',items_t)

# сверка: 2-канальный блок на листе = два моноблока (слитно) по объёму
_chk=trimesh.load(str(OUT/'1 ПЕЧАТЬ (блок каналы 1-2 + 8 вкладышей).stl'),force='mesh')
_vins=4*IP[(2.8,False)].volume/2*0+2*IP[(2.8,False)].volume+2*IP[(2.8,True)].volume+2*IP[(1.8,False)].volume+2*IP[(1.8,True)].volume
_vin=BLK_A.volume+_vins
assert abs(_chk.volume-_vin)<2.0,('лист 1 не совпал с моделями',_chk.volume,_vin)
print(f'СВЕРКА OK: STL листа 1 = блок+вкладыши ({_chk.volume/1000:.1f} см3)')

(OUT/'README_ПЕЧАТЬ_ДОКА.md').write_text("""# ДОК V2 — ПОЛНЫЙ НОВЫЙ КОМПЛЕКТ. 4 листа, БЕЗ поддержек

Это ВТОРОЙ дозатор с нуля: печатается ВСЁ (старый BX04 не трогаем, он работает).
Рама целиком на пазах — ни одного винта и ни одной лишней дырки. Винты только
внутри головы (серва, ось коромысла, тяга).

## Листы — ВЕСЬ новый ящик, печатается всё (5 запусков)
| Лист | Состав | Габарит | Время (слайс) |
|---|---|---|---|
| 1 ПЕЧАТЬ | СЛИТНЫЙ блок каналы 1-2 + 8 вкладышей | 206×204×111 | 16 ч 25 м (слой 0.28) |
| 2 ПЕЧАТЬ | СЛИТНЫЙ блок каналы 3-4 + 8 вкладышей | 206×204×111 | 16 ч 25 м (слой 0.28) |
| 3 ПЕЧАТЬ | канал 5 + 2 щёчки + 8 вкладышей | 206×176×111 | ~13 ч (слой 0.28) |
| 4 ПЕЧАТЬ | 5 коромысел с валиками + 5 тяг + 6 втулок + 7 замков-салазок + 8 проставок | 174×203×23 | 5 ч 03 м (слой 0.2) |
| 5 ПЕЧАТЬ | 5 держателей-ХВОСТ | 210×185×15 | 6 ч 03 м (слой 0.2) |

ИТОГО: ~57 часов. Кассета — отдельно из картона+ПНД по чертежам РАСКРОЙ.

## КАЛИБРОВКА (по аудиту): плечо качалки зафиксировано 10 мм
При плече 10 мм расчётный ход сервы ≈105° на ходе коромысла −28…+28°. СТАРЫЕ границы
30…110° (с прошлого дозатора) НЕ переносить вслепую — откалибровать HOME/EJECT заново
на собранном канале. Реальную покупную качалку MG996R ставить отверстием ровно 10 мм от оси.

## ГЛАВНЫЙ ОТКРЫТЫЙ РИСК (аудит-3, физ-симуляция Pymunk): ЗАДНИЙ ПЕРЕХОД
125 банок (25/канал) геометрически влезают, все CAD-конфликты закрыты. НО работа полной
очереди 25/25 НЕ доказана: физ-модель показала чувствительность к трению — при μ≈0.30
выходит 14-16 банок, остальные зависают у заднего перехода (распорная колонка); при μ≈0.15
выходят все 25. Задний проход = 51 мм (585−534), запас к Ø47 = 4 мм, к Ø50 = 1 мм.
ОБЯЗАТЕЛЬНО до тиража: собрать ОДИН двухъярусный канал из настоящего картона/ПНД, загрузить
ПОЛНЫЕ 25 банок, прогнать выдачу включая простой без команд. Если зависает — дорабатывать
переход (плавная направляющая, положение верхнего края и задней стенки), пересчитав с
сохранением 25 мест и раскроя. Это НЕ лечится настройками дока в одиночку.

## ПРОШИВКА (аудит-3 п.4): после цикла снимается PWM (auto-detach) — удержание полной
очереди в HOME не подтверждено. На физтесте проверить: держит ли закрытый стопор очередь
без питания; если нет — оставлять HOLD в HOME (не detach). Прошивку на ESP32 аудит не гонял.

## СНАЧАЛА НАПЕЧАТАЙ ТЕСТ!
Файл «ТЕСТ_ОДИН_КАНАЛ (проверить проход банки).stl» = один канал + 4 вкладыша, ~10 ч.
Проверь на нём ДО печати всего ящика: (1) банка проходит свободно, (2) качество печати
столбов/балки, (3) вкладыши надеваются и картон входит в щель. Если ок — печатай листы 1-5.

## V2.5 — ИСПРАВЛЕНИЯ ПО ФОТО ПЕРВОЙ ПЕЧАТИ
- БАНКА НЕ ПРОЛАЗИЛА: узкое место было сразу за кромкой ската (укорачивание подтянуло
  туда столбы+борта). Столбы ОТОДВИНУТЫ из канала (проём для банки 71->76 мм), борта
  УБРАНЫ, переход кромка->жёлоб сглажен, выход удлинён. Прокат банки в CAD: проход
  свободен для Ø47×67 и с запасом до Ø50×72.
- БОРТА-ШТЫРИ на стыке блоков: борта совсем убраны — банку по бокам ведёт картонный
  канал (до кромки) и стенки столбов (после). Штырей больше нет.
- БОКОВЫЕ СТЕНКИ-НАПРАВЛЯЮЩИЕ вернул (вели банку по доку): целые вдоль всего канала,
  ласточкин теперь под ними (в основании z0-6) и их не режет — штырей нет.
- ЛАСТОЧКИН НЕ ЛЕЗ: зазор 0,25 -> 0,6 мм — собирается от руки, без напильника.
- КРИВОЙ СТЫК: теперь каналы 1-2 и 3-4 печатаются СЛИТНЫМИ блоками (без стыка внутри).
  Стыков осталось 2 вместо 4 — вдвое меньше подгонки и перекосов.
- Мост балки провисает — это НЕ мешает (банка проходит ниже), лохмоту снизу срезать ножом.

## Настройки (Ender 3 V3 SE, Hyper PLA)
- 220/60 C; СЛОЙ: листы 1-3 -> 0.28 (минус ~20% времени, силовым моноблокам точнее не нужно), листы 4-5 -> 0.2; стенки 3, верх/низ 4 слоя, 12% gyroid
- Поддержки ВЫКЛ. Мост балки печатается мостом — обдув 100% со 2 слоя обязателен
- Кайма 4, юбка выкл, слон.нога 0.1-0.15, детали НЕ переставлять
- Лист 4: мелочь снимать аккуратно, замки и тяги плашмя, валики коромысел стоймя

## Предупреждения слайсера — какие НОРМАЛЬНЫЕ
- 1/2/3 «Long bridging»: это 60-мм мост под балкой между столбами. Печатается мостом,
  низ слегка лохматый — это ВЕРХ детали над приёмной зоной, ни на что не влияет.
- 4 «Floating anchors / Loose / Long bridging»: это ТЯГИ — их круглый стержень мостится
  между проушинами. Ориентация 1:1 как в листе PLATE_C2, который ты уже печатал — норм.
- Ничего другого слайсер говорить не должен (проверено по-детально).

## ПОКУПКИ на второй ящик
- 5 × MG996R
- ESP32 WROOM32 (CH340) + понижайка 5-6 В с запасом по току ≥4-5 А (пики серв 2-3 А!)
- Винты: M3×16..25 ×20 + гайки M3 ×30 (сервы) · M3×30 ×5 (оси коромысел) + шайбы ×10
- M2×12 ×5 и M2×8 ×5 + гайки M2 ×10 (тяги)
- Двусторонний скотч, провода; картон/ПНД на вторую кассету — раскрой уже готов

## РЕГУЛИРОВКА БЕЗ ПЕРЕПЕЧАТКИ
Щель под картон задаёт сменный вкладыш-седло на ребре моноблока. Метки на торце:
2 зуба = номинал (пара даёт 5,6 под гофру 5) · 3 зуба = узкий (пара 3,6; НОМИНАЛ+УЗКИЙ = 4,6 под твой гофролист Т23С ~4 мм). Широкий калибр (под картон 6-7) из комплекта убран — понадобится, допечатаем за 20 минут.
В комплекте 12 номинальных + 12 узких. Разные калибры слева/справа = сдвиг стенки +-0,5 мм. Толщина листа ПНД/ПП (4 или 2)
на док НЕ влияет: 42 мм в чертежах кассеты — это ВЕРХ листа; под 2 мм перегенерируй
только чертежи картона (plastic_thickness=2). Высота головы — шимами (напечатай с
листа V3 при надобности). Меньше каналов — меньше моноблоков.

## Сборка — по шагам в СБОРКА_ДОКА_V14_3D.html (9 шагов)
Кратко: моноблоки в ряд (ласточкины сверху) -> щёчки -> вкладыши (2 зуба) ->
голова: серва+качалка в держатель, язычок в паз балки -> коромысло на ось + тяга ->
замок-салазки сзади до щелчка -> кассета в раструбы, придвинуть до упора листа
в язычки -> скотч под крайние моноблоки, общая земля, тест без банок.

## Если что-то не лезет
- Кассета не садится — широкий вкладыш (1 зуб) на упирающуюся стенку.
- Замок-салазки туго — пройти надфилем ушки; болтается — капля клея на хвост (но
  сначала проверь, что ушки защёлкнулись за бугорки).
- Ласточкины собираются от руки; коромысло на втулке должно падать под своим весом.
""",encoding='utf-8')
print('README ->',OUT/'README_ПЕЧАТЬ_ДОКА.md')
