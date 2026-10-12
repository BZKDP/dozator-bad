"""Independent read-only audit of release 2026-09-30 20:36.
Never runs exporters. Outputs JSON evidence only to stdout.
"""
from pathlib import Path
import hashlib, json, math, sys
sys.stdout.reconfigure(encoding='utf-8')
import numpy as np
import trimesh
from scipy.optimize import brentq

ROOT=Path(__file__).resolve().parents[1]
watched=[ROOT/'make_dock_full_3d.py',ROOT/'ДОЗАТОР_V14_ПОЛНЫЙ_3D.html',ROOT/'СБОРКА_ДОКА_V14_3D.html']
watched+=list((ROOT/'ПЕЧАТЬ_ДОКА').glob('*.stl'))
hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in watched}
source=(ROOT/'make_dock_full_3d.py').read_text(encoding='utf-8')
ns={}
exec(compile(source.split('# ============ экспорт')[0],str(ROOT/'make_dock_full_3d.py'),'exec'),ns)
globals().update({k:v for k,v in ns.items() if k not in ('__builtins__','ROOT','source','ns','watched','hashes')})
def emit(label,value):
    print(label,json.dumps(value,ensure_ascii=False,default=lambda v:v.item() if hasattr(v,'item') else str(v)),flush=True)
def mesh(p):
    m=trimesh.Trimesh(vertices=geoms[p]['v'],faces=geoms[p]['f'],process=True);m.fix_normals();return m
def placed(p,q=-28):
    m=mesh(p);m.apply_transform(mbase(YCS[2])@pose_matrix(geoms[p].get('motion','fixed'),q));return m
def cross(a,b):
    if np.any(a.bounds[1]<=b.bounds[0]) or np.any(b.bounds[1]<=a.bounds[0]):return None
    m=trimesh.boolean.intersection([a,b],engine='manifold')
    return m if m is not None and len(m.faces) else None
def volume(a,b):
    m=cross(a,b);return 0. if m is None else float(m.volume)
def detail(a,b,local=False):
    m=cross(a,b)
    if m is None:return None
    if local:m.apply_transform(np.linalg.inv(mbase(YCS[2])))
    return {'mm3':float(m.volume),'bounds':np.round(m.bounds,6).tolist(),
            'components':[{'mm3':float(c.volume),'bounds':np.round(c.bounds,6).tolist()} for c in m.split(only_watertight=False)]}

whole=_U(MONO)
emit('CASSETTE_COLLISIONS',{key:volume(whole,m) for key,m,*_ in CAS})
emit('ALL_INSERT_VARIANTS',{str((sh,mr)):detail(shifted(insert_mesh(sh,mr),YCS[2]),MONO[2])
    for sh in (1.8,2.8,3.8) for mr in (False,True)})
emit('LOCK_FRAME_DETAIL',detail(placed('ZAMOK_SLIDE'),MONO[2],True))
emit('HOLDER_FRAME_DETAIL',detail(placed('B401'),MONO[2],True))
emit('LOCK_RIGID_PATH',{str((du,dn)):volume(MONO[2],(m:=placed('ZAMOK_SLIDE')).apply_translation(mbase(YCS[2])[:3,:3]@np.array([du,0,dn])))
    for du,dn in ((0,0),(1,0),(0,.4),(1,.4),(2,.4),(5,.4),(10,.4),(20,.4))})

testpath=next((ROOT/'ПЕЧАТЬ_ДОКА').glob('ТЕСТ*.stl'))
test=trimesh.load(testpath,force='mesh')
testparts=test.split()
FLIP=trimesh.transformations.rotation_matrix(math.pi,[0,1,0])
realinsert=next(c for c in testparts if np.allclose(c.bounds[0,:2],[110,100],atol=.01))
pred=insert_mesh(1.8);pred.apply_transform(FLIP)
realinsert.apply_translation(pred.bounds[0]-realinsert.bounds[0]);realinsert.apply_transform(FLIP)
realinsert.apply_translation([0,YCS[2],0])
emit('ACTUAL_TEST_NARROW_INSERT',detail(realinsert,MONO[2]))
p4=trimesh.load(next((ROOT/'ПЕЧАТЬ_ДОКА').glob('4 *.stl')),force='mesh')
pieces=p4.split()
locks=[c for c in pieces if abs(c.volume-mesh('ZAMOK_SLIDE').volume)<.02]
spacers=[c for c in pieces if np.allclose(np.sort(c.extents),[3,6,6],atol=.01)]
emit('ACTUAL_SMALL_PARTS',{'shells':len(pieces),'locks':len(locks),'spacers':len(spacers),
    'spacer_euler':[c.euler_number for c in spacers],'spacer_volume':[float(c.volume) for c in spacers]})

# All real moving meshes, not only their five-point bounding boxes.
fixed={'frame':MONO[2],'holder':placed('B401'),'servo':placed('H_servo'),'lock':placed('ZAMOK_SLIDE')}
stats={p+'_'+f:{'maximum_mm3':0.,'poses':0,'q':None} for p in ('B402','B403','H_horn') for f in fixed}
for q in np.linspace(-28,28,113):
    for p in ('B402','B403','H_horn'):
        pm=placed(p,q)
        for f,fm in fixed.items():
            v=volume(pm,fm);s=stats[p+'_'+f]
            if v>s['maximum_mm3']:s.update(maximum_mm3=v,q=float(q))
            if v>.1:s['poses']+=1
emit('113_POSE_COLLISIONS',stats)
emit('AXIAL_STACK',{p:mesh(p).bounds[:,1].tolist() for p in ('H_horn','B403','PROSTAVKA','H_pin_horn','H_pin_horn_nut')})
emit('SPACER_HORN_INTERSECTION',volume(placed('PROSTAVKA'),placed('H_horn')))

# Horn CAD hole retained at radius 9 while closure calculates radius 10.
horn=mesh('H_horn')
centre9=np.array([-46.,82.])
rr=np.hypot(horn.vertices[:,0]-centre9[0],horn.vertices[:,2]-centre9[1])
emit('HORN_PHYSICAL_HOLE',{'closest_vertex_radius_to_9mm_hole':float(rr.min()),
    'hole_vertices_count':int(np.sum(abs(rr-1.15)<.03)),
    'hole_centre_unposed':centre9.tolist(),'calculated_radius':HORN})
emit('HORN_HOLE_TO_LINK_MISMATCH',{str(q):float(np.linalg.norm((pose_matrix('horn',q)@np.array([-46,0,82,1]))[[0,2]]-drive(q)[1]))
    for q in (-28,0,28)})
emit('PIN_THROUGH_HORN_MM3',{str(q):volume(placed('H_pin_horn',q),placed('H_horn',q)) for q in (-28,0,28)})
emit('SERVO_THROW',{'closed_angle':drive(-28)[2],'open_angle':drive(28)[2],
    'required_degrees':drive(28)[2]-drive(-28)[2]})

# Retest contact model for r=10. Do not silently reuse old r=9 results.
ref=Path(r'BX04_ПРОЕКТ_ПРОТОТИП\REFERENCE_D2\SOURCE')  # внешний источник (не в репозитории)
sys.path.insert(0,str(ref))
simsrc=(ref/'simulate.py').read_text(encoding='utf-8')
simsrc=simsrc.replace("if record:(OUT/'trace.json').write_text(json.dumps(trace,separators=(',',':')))","if record:globals()['audit_trace']=trace")
sim={'__file__':str(ref/'simulate.py'),'__name__':'audit_simulation'}
exec(compile(simsrc,str(ref/'simulate.py'),'exec'),sim)
sim.update(r=10.,EDGE=45.,drive=drive,b0=drive(-28)[2],b1=drive(28)[2])
emit('CONTACT_MODEL_10MM',[sim['run'](n,D,.75,.02,.002,False) for n,D in ((1,47),(3,47),(10,47),(10,49))])
sim['run'](3,47,.75,.02,.002,True)

def jar(D,L,u,n,dy=0,yaw=0):
    m=trimesh.creation.cylinder(radius=D/2,height=L,sections=96)
    m.apply_transform(trimesh.transformations.rotation_matrix(math.pi/2,[1,0,0]))
    m.apply_transform(trimesh.transformations.rotation_matrix(math.radians(yaw),[0,0,1]))
    m.apply_translation([u,dy,n]);m.apply_transform(mbase(YCS[2]));return m
hits=[]
for f in sim['audit_trace'][::4]:
    for j,(u,n,vis) in enumerate(f['cans']):
        if not vis or f['states'][j]!=0:continue
        v=volume(jar(47,67,u,n),placed('B402',f['q']))
        if v>.1:hits.append({'time':f['t'],'jar':j,'q':f['q'],'mm3':v})
emit('CAN_ROCKER_CONTACT_SCREEN',{'hits':len(hits),'examples':hits[:6]})

# Capture both saved viewers and verify their math vs Python geometry.
viewers={}
for filename in ('ДОЗАТОР_V14_ПОЛНЫЙ_3D.html','СБОРКА_ДОКА_V14_3D.html'):
    txt=(ROOT/filename).read_text(encoding='utf-8')
    start=txt.index('const DATA=')+len('const DATA=')
    viewers[filename]=json.JSONDecoder().raw_decode(txt[start:])[0]
emit('VIEWER_PAYLOAD_EQUAL',viewers['ДОЗАТОР_V14_ПОЛНЫЙ_3D.html']==viewers['СБОРКА_ДОКА_V14_3D.html'])
data=viewers['ДОЗАТОР_V14_ПОЛНЫЙ_3D.html']
def vm(key):
    g=data['meshes'][key];m=trimesh.Trimesh(vertices=np.asarray(g['v']).reshape(-1,3),faces=np.asarray(g['f']).reshape(-1,3),process=True)
    m.fix_normals();return m
emit('SAVED_VIEWER_LOCK_COLLISION',detail(vm('ZAMOK_SLIDE').apply_transform(mbase(YCS[2])),vm('MONO2')))
emit('MODEL_DEPARTURE_VS_VIEWER',{'real_chute_end_x':165,'viewer_bend':data['C']['bend'],
    'visual_jar_length_mm':60+7-3.5,'reference_jar_length_mm':67})

# Float32 saved STL congruence, including actual fused frame versus generator.
p1=trimesh.load(next((ROOT/'ПЕЧАТЬ_ДОКА').glob('1 *.stl')),force='mesh')
big=max(p1.split(),key=lambda c:c.volume)
expected=block2(YCS[0],YCS[1]);expected.apply_translation(big.bounds[0]-expected.bounds[0])
emit('SAVED_FRAME_MATCH',{'difference_mm3':float(_D(big,expected).volume+_D(expected,big).volume),
    'nominal_volume_difference_mm3':float(big.volume-expected.volume)})

# The left cheek is MALE. The written order puts it into a previously assembled FEMALE block.
emit('LEFT_CHEEK_DOCUMENTED_VERTICAL_PATH',{str(dz):volume(MONO[0],CHK['L'].copy().apply_translation([0,0,dz]))
    for dz in (0,.5,1,2,4,6,10,20,40,80)})
emit('LEFT_CHEEK_CORRECT_VERTICAL_PATH',{str(dz):volume(CHK['L'],MONO[0].copy().apply_translation([0,0,dz]))
    for dz in (0,.5,1,2,4,6,10,20,40,80)})
rear=next(m for key,m,*_ in CAS if key=='rear')
emit('FULL_VIEWER_REAR_CASSETTE_COLLISIONS',{key:volume(rear,m) for key,m,*_ in CAS if key.startswith('div')})
emit('SOURCE_MG_SERVO_BOLTS',{p:{'title':geoms[p].get('title'), 'bounds':mesh(p).bounds.tolist()}
    for p in ('H_servo_screw0','H_servo_screw1','H_servo_nut0','H_servo_nut1')})
emit('REAR_MOCKUP_FILES',[p.name for p in (ROOT/'3D_МОДЕЛИ').glob('*ЗАД*')])
emit('FILES_STABLE',all(hashlib.sha256(p.read_bytes()).hexdigest()==hashes[str(p)] for p in watched))
emit('SNAPSHOT_HASHES',hashes)
emit('RECHECK_FINISHED',True)
