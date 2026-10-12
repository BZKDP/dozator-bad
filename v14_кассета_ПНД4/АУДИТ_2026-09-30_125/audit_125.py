"""Read-only R3 audit. Full loaded cassette: 5 lanes x 25 jars, 60 g each.
Does NOT call either generator's exporters or touch project design files.
"""
from pathlib import Path
import sys, json, math, hashlib
sys.stdout.reconfigure(encoding='utf-8')
import numpy as np
import trimesh
from scipy.optimize import brentq
from shapely.geometry import Point, LineString

ROOT=Path(__file__).resolve().parents[1]
watched=[ROOT/'make_dock_full_3d.py',ROOT/'generate_cassette.py',ROOT/'ДОЗАТОР_V14_ПОЛНЫЙ_3D.html',ROOT/'СБОРКА_ДОКА_V14_3D.html']+list((ROOT/'ПЕЧАТЬ_ДОКА').glob('*.stl'))
hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in watched}
source=(ROOT/'make_dock_full_3d.py').read_text(encoding='utf-8')
ns={}
exec(compile(source.split('# ============ экспорт')[0],str(ROOT/'make_dock_full_3d.py'),'exec'),ns)
globals().update({k:v for k,v in ns.items() if k not in ('__builtins__','ROOT','source','ns','watched','hashes')})

RESULTS={}
def emit(k,v):
    RESULTS[k]=v
    print(k,json.dumps(v,ensure_ascii=False,default=lambda x:x.item() if hasattr(x,'item') else str(x)),flush=True)
def mesh(key):
    g=geoms[key]; m=trimesh.Trimesh(vertices=g['v'],faces=g['f'],process=True); m.fix_normals();return m
def placed(key,q=-28,yc=YCS[2]):return mesh(key).apply_transform(mbase(yc)@pose_matrix(geoms[key].get('motion','fixed'),q))
def intersection(a,b):
    if np.any(a.bounds[1]<=b.bounds[0]) or np.any(b.bounds[1]<=a.bounds[0]):return None
    return trimesh.boolean.intersection([a,b],engine='manifold')
def volume(a,b):
    m=intersection(a,b);return float(m.volume) if m is not None and len(m.faces) else 0.
def parts(m):return [{'volume':float(c.volume),'bounds':c.bounds.tolist()} for c in m.split(only_watertight=False) if abs(c.volume)>.001]

emit('SNAPSHOT_HASHES',hashes)
emit('INSERTS',{str((sh,mr)):volume(MONO[2],shifted(insert_mesh(sh,mr),YCS[2])) for sh in (1.8,2.8,3.8) for mr in (False,True)})
emit('FIXED_PARTS',{k:volume(MONO[2],placed(k)) for k in ('B401','B404','H_servo','ZAMOK_SLIDE')})
lockix=intersection(MONO[2],placed('ZAMOK_SLIDE'))
lockix.apply_transform(np.linalg.inv(mbase(YCS[2])))
emit('LOCK_ONLY_SNAP_CONTACT',parts(lockix))
emit('AXIAL_STACK',{k:mesh(k).bounds[:,1].tolist() for k in ('H_horn','B403','PROSTAVKA','H_pin_horn','H_pin_horn_nut')})
emit('SPACER_VS_HORN_LINK',{'horn':volume(placed('PROSTAVKA'),placed('H_horn')),'link':volume(placed('PROSTAVKA'),placed('B403'))})

# Pin is origin-local; horn is axis-local at (-55,82). Scaling them around the same point is incorrect.
pin=mesh('H_pin_horn');pc=pin.bounds.mean(axis=0)
hc=np.array([-45.,0.,82.,1.])
emit('HORN_PIN_LOCAL_CENTRE',{'actual_pin_xz':pc[[0,2]].tolist(),'expected':[0.,0.],
    'unposed_horn_hole_xz':[-45.,82.], 'pin_offset_mm':float(np.linalg.norm(pc[[0,2]]))})
emit('HORN_AND_PIN_ALIGNMENT',{str(q):{'hole_to_link':float(np.linalg.norm((pose_matrix('horn',q)@hc)[[0,2]]-drive(q)[1])),
    'pin_to_link':float(np.linalg.norm((pose_matrix('pin_horn',q)@np.r_[pc,1])[[0,2]]-drive(q)[1])),
    'pin_spacer_mm3':volume(placed('H_pin_horn',q),placed('PROSTAVKA',q)),
    'pin_link_mm3':volume(placed('H_pin_horn',q),placed('B403',q)),
    'pin_horn_mm3':volume(placed('H_pin_horn',q),placed('H_horn',q))} for q in (-28,0,28)})

fixed={'frame':MONO[2],'holder':placed('B401'),'servo':placed('H_servo'),'lock':placed('ZAMOK_SLIDE')}
stats={k+'_'+f:{'max_mm3':0.,'poses':0,'q':None} for k in ('B402','B403','H_horn','H_pin_horn','PROSTAVKA') for f in fixed}
for q in np.linspace(-28,28,113):
    for k in ('B402','B403','H_horn','H_pin_horn','PROSTAVKA'):
        a=placed(k,q)
        for f,b in fixed.items():
            v=volume(a,b);s=stats[k+'_'+f]
            if v>s['max_mm3']:s.update(max_mm3=v,q=float(q))
            if v>.1:s['poses']+=1
emit('113_POSE_COLLISIONS',stats)

# Build the real cassette, not the simplified solid rear rectangle in the dock viewer.
cs={'__file__':str(ROOT/'generate_cassette.py'),'__name__':'read_only_cassette'}
exec(compile((ROOT/'generate_cassette.py').read_text(encoding='utf-8'),str(ROOT/'generate_cassette.py'),'exec'),cs)
cp,side_profile,inner_profile=cs['build_parts']()
P=cs['P']
for v in cp.values():v['mesh'].fix_normals()
emit('CASSETTE_REAL_PARTS',{k:{'watertight':bool(v['mesh'].is_watertight),'extents':v['mesh'].extents.tolist()} for k,v in cp.items()})
emit('CASSETTE_REAL_INTERSECTIONS',{a+'_'+b:volume(cp[a]['mesh'],cp[b]['mesh']) for a,b in
    [(f'wall_{i}',k) for i in range(1,7) for k in ('lower','upper','rear')]+[('rear','rear_wing_left'),('rear','rear_wing_right')]})
whole=_U(MONO)
emit('CASSETTE_TO_DOCK',{k:volume(whole,rot_cas(v['mesh'])) for k,v in cp.items()})

# Upper rear passage: use actual inside face, and diagonal circle clearance from the finite corner.
rearx=P['depth']-P['cardboard_thickness']
upper_corner=(P['upper_x1'],P['upper_rear_z'])
lower_z=cs['rz_lower'](rearx)
emit('REAR_TRANSFER',{'rear_inside_x':rearx,'upper_end_x':P['upper_x1'],'real_gap_mm':rearx-P['upper_x1'],
    'advertised_gap_mm':P['upper_rear_gap'],'lower_actual_end_x':P['lower_x1'],'lower_end_gap_mm':rearx-P['lower_x1'],
    'vertical_top_gap_mm':P['upper_rear_z']-P['lower_rear_z'],
    'diameter_clearance':{str(D):rearx-P['upper_x1']-D for D in (47,49,50)}})
emit('SHEET_PROFILE_CLOSED_EDGES',{'side_connected':side_profile.geom_type,'inner_connected':inner_profile.geom_type,
    'side_holes':len(side_profile.interiors),'inner_holes':len(inner_profile.interiors),
    'lower_front_ligament_mm':min(p[0] for ring in side_profile.interiors for p in ring.coords),
    'upper_top_ligament_mm':P['height']-max(p[1] for ring in side_profile.interiors for p in ring.coords)})

# Static contiguous queue: full lower line until rear wall, one transition can touching wall and previous,
# then upper line. Disks are 2D XZ sections of horizontal cans; a 3D check follows.
def queue(D=47,L=67,N=25):
    R=D/2
    a=math.radians(-28);c,s=math.cos(a),math.sin(a)
    front=np.array([19.5*c-(-12)*s,58+19.5*s+(-12)*c])
    firstu=front[0]-math.sqrt((R+3)**2-(front[1]-R)**2)
    first=(mbase(YCS[2])@np.array([firstu,0,R,1]))[[0,2]]
    down=np.array([-C15,S15]);lower=[]
    rear_center=-rearx+R
    for j in range(100):
        p=first+j*D*down
        if p[0]<rear_center-1e-6:break
        lower.append(p)
    p=lower[-1];dx=rear_center-p[0]
    transition=np.array([rear_center,p[1]+math.sqrt(D*D-dx*dx)])
    ta=math.radians(P['upper_angle'])
    def uz(x):return P['upper_z0']+(x+25)*math.tan(ta)+R/math.cos(ta)
    def dist(x):return (x-transition[0])**2+(uz(x)-transition[1])**2-D*D
    ux=brentq(dist,transition[0]+.01,transition[0]+D+10)
    upper=[]
    for j in range(100):
        x=ux+j*D*math.cos(ta)
        # Contact point must stay on the upper sheet; no suspended extra can at the loading edge.
        if x+R*math.sin(ta)>-P['upper_x0']+1e-6:break
        upper.append(np.array([x,uz(x)]))
    arr=np.array(lower+[transition]+upper)
    return {'D':D,'L':L,'lower_count':len(lower),'transition_count':1,'upper_count':len(upper),
       'geometric_capacity_candidate':len(arr),'positions':arr,'first_u':firstu,'centres_for_25':arr[:N]}

def worldjar(D,L,x,z,yc=YCS[2],yaw=0):
    m=trimesh.creation.cylinder(radius=D/2,height=L,sections=96)
    m.apply_transform(trimesh.transformations.rotation_matrix(math.pi/2,[1,0,0]))
    m.apply_transform(trimesh.transformations.rotation_matrix(math.radians(yaw),[0,0,1]))
    m.apply_translation([x,yc,z]);return m

capacity=[]
for D,L in ((47,67),(49,70),(50,72)):
    pack=queue(D,L)
    arr=pack['positions'];selected=pack['centres_for_25']
    obstacles={'wall'+str(i):rot_cas(cp['wall_'+str(i)]['mesh']) for i in range(1,7)}
    obstacles.update({k:rot_cas(cp[k]['mesh']) for k in ('rear','lower','upper')})
    obstacles['dock']=MONO[2]
    obstacles['holder']=placed('B401');obstacles['servo']=placed('H_servo');obstacles['rocker']=placed('B402')
    hits=[]
    jars=[worldjar(D,L,*p) for p in selected]
    for j,jar in enumerate(jars):
        for k,ob in obstacles.items():
            v=volume(jar,ob)
            if v>.1:hits.append({'jar':j+1,'obstacle':k,'mm3':v})
    pair_hits=[]
    for i in range(len(jars)):
        for j in range(i+1,len(jars)):
            v=volume(jars[i],jars[j])
            if v>.1:pair_hits.append({'pair':[i+1,j+1],'mm3':v})
    capacity.append({k:v for k,v in pack.items() if k not in ('positions','centres_for_25')})
    capacity[-1].update(selected_count=len(selected),can_structure_hits=hits,can_can_hits=pair_hits,
        selected_positions=selected.tolist(),upper_front_margin_mm=float(-P['upper_x0']-(arr[-1,0]+D/2*math.sin(math.radians(P['upper_angle'])))))
emit('STATIC_25_PACKING_CANDIDATES',capacity)

# A lower-only virtual queue of 25 is NOT a two-tier loaded cassette test.
ref=Path(r'BX04_ПРОЕКТ_ПРОТОТИП\REFERENCE_D2\SOURCE')  # внешний источник (не в репозитории)
sys.path.insert(0,str(ref))
simsrc=(ref/'simulate.py').read_text(encoding='utf-8')
simsrc=simsrc.replace("if record:(OUT/'trace.json').write_text(json.dumps(trace,separators=(',',':')))","if record:globals()['audit_trace']=trace")
sim={'__file__':str(ref/'simulate.py'),'__name__':'audit_simulation'}
exec(compile(simsrc,str(ref/'simulate.py'),'exec'),sim)
sim.update(r=10.,EDGE=45.,drive=drive,b0=drive(-28)[2],b1=drive(28)[2])
emit('VIRTUAL_LOWER_ONLY_25',[sim['run'](25,D,.75,.02,.004,False) for D in (47,49,50)])
emit('LOAD_ASSUMPTIONS',{'mass_g_each':60,'per_lane_kg':1.5,'all_125_kg':7.5,'support':'entire cassette and dock supported on a plane',
    'conservative_all_25_on_15deg_static_force_N':25*.060*9.81*math.sin(math.radians(15)),
    'upper_11_on_10deg_static_force_N':11*.060*9.81*math.sin(math.radians(10))})

emit('STL_PRINT_FILES',{p.name:{'extents':(m:=trimesh.load(p,force='mesh')).extents.tolist(),'watertight':bool(m.is_watertight),'shells':len(m.split())} for p in (ROOT/'ПЕЧАТЬ_ДОКА').glob('*.stl')})
emit('TEXT_DOCUMENT_FIXES',{'R2_4_assert_present':'R2-4' in source,'boolean_error_is_swallowed':'except Exception: return 0.0' in source,
    'step_3_left_corrected':'СВЕРХУ в уже стоящий блок' in (ROOT/'_dock_assembly_template.html').read_text(encoding='utf-8'),
    'README_old_30_110_mentioned':'30' in (ROOT/'ПЕЧАТЬ_ДОКА/README_ПЕЧАТЬ_ДОКА.md').read_text(encoding='utf-8')})
emit('FILES_STABLE',all(hashlib.sha256(p.read_bytes()).hexdigest()==hashes[str(p)] for p in watched))
emit('AUDIT_FINISHED',True)
(Path(__file__).resolve().parent/'geometry-results.json').write_text(json.dumps(RESULTS,ensure_ascii=False,indent=2,default=lambda x:x.item() if hasattr(x,'item') else str(x)),encoding='utf-8')
