"""Read-only engineering review. Never executes project export section."""
from pathlib import Path
import json, math, time
import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parent.parent
source = (ROOT / 'make_dock_full_3d.py').read_text(encoding='utf-8')
ns = {}
exec(compile(source.split('# ============ экспорт')[0], str(ROOT/'make_dock_full_3d.py'), 'exec'), ns)
globals().update({k:v for k,v in ns.items() if k not in ('__builtins__', 'ROOT', 'source', 'ns')})

def out(label, value):
    print(label, json.dumps(value, ensure_ascii=False, default=lambda x:x.item() if hasattr(x,'item') else str(x)), flush=True)

def mesh(part):
    g=geoms[part]
    m=trimesh.Trimesh(vertices=np.asarray(g['v']), faces=np.asarray(g['f']), process=True)
    m.fix_normals()
    return m

def placed(part, q=-28, yc=YCS[2]):
    m=mesh(part)
    m.apply_transform(mbase(yc) @ pose_matrix(geoms[part].get('motion','fixed'), q))
    return m

def intersect(a,b):
    if np.any(a.bounds[1] <= b.bounds[0]) or np.any(b.bounds[1] <= a.bounds[0]):
        return 0.0
    i=trimesh.boolean.intersection([a,b], engine='manifold')
    return 0.0 if i is None or not len(i.faces) else float(i.volume)

def components(m):
    return [{'extent':np.round(c.extents,4).tolist(), 'volume':round(float(c.volume),4),
             'bounds':np.round(c.bounds,4).tolist()} for c in m.split(only_watertight=False)]

out('PART_COMPONENTS', {p:components(mesh(p)) for p in ('B402','B403','B404','B401','ZAMOK_SLIDE','PROSTAVKA')})
out('PRINT_STLS', {p.name:{'extents':np.round((m:=trimesh.load(p,force='mesh')).extents,4).tolist(),
                         'watertight':bool(m.is_watertight),'components':len(m.split(only_watertight=False)),
                         'volume_cm3':round(float(m.volume/1000),4)} for p in (ROOT/'ПЕЧАТЬ_ДОКА').glob('*.stl')})

whole=_U(MONO)
out('CASSETTE_DOCK_COLLISIONS_MM3', {key:round(intersect(whole,m),4) for key,m,*_ in CAS})
out('INSERT_DOCK_COLLISIONS_MM3', {str((sh,mr)):round(intersect(MONO[2],shifted(insert_mesh(sh,mr),YCS[2])),4)
    for sh in (1.8,2.8,3.8) for mr in (False,True)})
out('FIXED_HEAD_FRAME_MM3', {p:round(intersect(placed(p),MONO[2]),5)
    for p in ('B401','B404','H_servo','ZAMOK_SLIDE')})
out('FIXED_PAIR_MM3', {a+'_'+b:round(intersect(placed(a),placed(b)),5)
    for a,b in [('B401','ZAMOK_SLIDE'),('B401','H_servo'),('B401','B404')]})

# Assembly says blocks are lowered from above: sweep their relative vertical offset.
out('DOVETAIL_VERTICAL_ASSEMBLY_MM3', {str(dz):round(intersect(MONO[0],_shift:=shifted(MONO[1],0)),5)
    if dz==0 else round(intersect(MONO[0],trimesh.Trimesh(vertices=MONO[1].vertices+[0,0,dz],faces=MONO[1].faces,process=True)),5)
    for dz in (0,0.2,0.5,1,2,3,4,6,8,12,20,40)})

# Isolated dovetail male/female block collision removes uncertainty from crossing tall structures.
male=dove(55,YCS[0]+40)
out('DOVETAIL_MALE_ROOF_MM3', {str(dz):round(intersect(MONO[1],trimesh.Trimesh(vertices=male.vertices+[0,0,dz],faces=male.faces,process=True)),5)
    for dz in (0,0.2,0.5,1,2,3,4,6,8,12,20,40)})

head=placed('B401')
lock=placed('ZAMOK_SLIDE')
axis=mbase(YCS[2])[:3,:3] @ np.array([1.,0.,0.])
out('LOCK_INSERTION_SWEEP_MM3', {str(dx):round(intersect(MONO[2],trimesh.Trimesh(vertices=lock.vertices+dx*axis,faces=lock.faces,process=True)),5)
    for dx in (0,0.1,0.3,0.5,1,2,3,4,5,7,10,15,20,25)})

qs=np.linspace(-28,28,113)
pairs=[('B402','FRAME'),('B403','FRAME'),('H_horn','FRAME'),('B402','B401'),('B403','B401'),
       ('B402','H_servo'),('B403','H_servo'),('B402','B403'),('H_horn','B401')]
result={a+'_'+b:{'max_volume':0.,'q_at_max':None,'poses_above_0.1':0} for a,b in pairs}
for qi,q in enumerate(qs):
    parts={p:placed(p,float(q)) for p in ('B402','B403','H_horn','B401','H_servo')}
    parts['FRAME']=MONO[2]
    for a,b in pairs:
        v=intersect(parts[a],parts[b]); r=result[a+'_'+b]
        if v>r['max_volume']: r.update(max_volume=round(v,5),q_at_max=round(float(q),3))
        if v>0.1: r['poses_above_0.1']+=1
    if qi%20==0: out('MOTION_PROGRESS',{'poses':qi+1,'total':len(qs)})
out('MOTION_COLLISIONS_MM3',result)
out('DRIVE_RANGE', {'q_min':-28,'q_max':28,'horn_angle_q_min':drive(-28)[2],
                    'horn_angle_q_max':drive(28)[2], 'servo_travel_deg':drive(-28)[2]-drive(28)[2],
                    'link_length':float(np.linalg.norm(drive(0)[0]-drive(0)[1]))})

# Confirm actual serialized viewer and actual plate contents, not only regeneration formulas.
viewer=(ROOT/'ДОЗАТОР_V14_ПОЛНЫЙ_3D.html').read_text(encoding='utf-8')
start=viewer.index('const DATA=')+len('const DATA=')
data,_end=json.JSONDecoder().raw_decode(viewer[start:])
out('VIEWER_DATA_KEYS', list(data.keys()))
out('VIEWER_PART_KEYS', list(data.get('models',data.get('meshes',{})).keys()))
out('AUDIT_COMPLETE', {'read_only_project':True})

# Confirm the plate contains the very same erroneous solids, not a stale generator.
p4=next((ROOT/'ПЕЧАТЬ_ДОКА').glob('4 *.stl'))
c4=trimesh.load(p4,force='mesh').split(only_watertight=False)
lock_volumes=[round(c.volume,2) for c in mesh('ZAMOK_SLIDE').split()]
out('ACTUAL_PLATE_LOCK_FRAGMENT_COUNTS',{str(v):sum(abs(c.volume-v)<0.03 for c in c4) for v in lock_volumes})
spacers=[c for c in c4 if np.allclose(np.sort(c.extents),[3,6,6],atol=.01)]
out('ACTUAL_PLATE_SPACERS', [{'euler_number':int(c.euler_number),'volume':float(c.volume)} for c in spacers])

def collision_details(a,b):
    i=trimesh.boolean.intersection([a,b],engine='manifold')
    return {'volume':float(i.volume),'bounds':np.round(i.bounds,5).tolist(),
            'components':components(i)} if i is not None and len(i.faces) else None
out('INSERT_COLLISION_DETAIL', collision_details(MONO[2],shifted(insert_mesh(2.8),YCS[2])))
out('WALL_COLLISION_DETAIL', collision_details(whole,next(m for key,m,*_ in CAS if key=='div1')))
out('SMALL_HEAD_COLLISION_DETAIL',collision_details(MONO[2],placed('B401')))
out('LINK_SERVO_COLLISION_DETAIL',collision_details(placed('B403',28),placed('H_servo')))
out('LINKAGE_AXIAL_STACK', {p:np.round(mesh(p).bounds[:,1],3).tolist() for p in ('H_horn','B403','PROSTAVKA','H_pin_horn')})

# Solve the contact transfer and actual horn throw, independently of viewer animations.
from scipy.optimize import brentq
PIVOT=np.array([0.,58.])
def stop_point(pt,q):
    a=math.radians(q); c,s=math.cos(a),math.sin(a)
    return PIVOT+np.array([c*pt[0]-s*pt[1],s*pt[0]+c*pt[1]])
FRONT=np.array([19.5,-12.]); REAR=np.array([-27.,-16.])
def barrier(pt,D):
    dd=(D/2+3)**2-(pt[1]-D/2)**2
    return pt[0]-math.sqrt(dd) if dd>=0 else float('inf')
contact=[]
for D in (47,49,50):
    qt=brentq(lambda q:barrier(stop_point(FRONT,q),D)-D-barrier(stop_point(REAR,q),D),-20,15)
    qr=brentq(lambda q:stop_point(FRONT,q)[1]-3-D,0,28)
    contact.append({'diameter':D,'transfer_q':qt,'release_q':qr,'overlap_rocker_deg':qr-qt,
       'open_front_clear_mm':stop_point(FRONT,28)[1]-3-D,
       'open_rear_clear_mm':stop_point(REAR,-28)[1]-3-D})
out('TWO_STOP_CONTACT_LOGIC', contact)

def horn_angle(q,rh):
    Q=P2+17.5*np.array([-math.sin(math.radians(q)),math.cos(math.radians(q))])
    dd=np.linalg.norm(Q-S2)
    b=math.atan2(Q[1]-S2[1],Q[0]-S2[0])+math.acos((rh*rh+dd*dd-55.5**2)/(2*rh*dd))
    return math.degrees(b)
out('HORN_RADIUS_SENSITIVITY', {str(rh):{'closed':horn_angle(-28,rh),'open':horn_angle(28,rh),
    'throw':horn_angle(28,rh)-horn_angle(-28,rh)} for rh in (9,10,11,12,13)})

# Jar passage between actual side walls and posts, including yaw and lateral offsets.
obstacles=_U([side_wall(YCS[2],s) for s in (-1,1)]+[post(YCS[2],s) for s in (-1,1)]+[beam(YCS[2])])
floor_x=np.linspace(-100,165,5301)
floor_z=np.where(floor_x<=107,zsurf(floor_x),6.6-(floor_x-107)*4.6/58)
def jar(D,L,cx,cz,dy=0,yaw=0):
    m=trimesh.creation.cylinder(radius=D/2,height=L,sections=96)
    m.apply_transform(trimesh.transformations.rotation_matrix(math.pi/2,[1,0,0]))
    m.apply_transform(trimesh.transformations.rotation_matrix(math.radians(yaw),[0,0,1]))
    m.apply_transform(Ry(15))
    m.apply_translation([cx,YCS[2]+dy,cz])
    return m
paths=[]
for D,L in ((47,67),(50,72)):
    for yaw,dy in ((0,0),(0,2),(0,-2),(5,0),(-5,0),(5,2),(-5,-2)):
        maxv=0.; atx=None; hits=0
        for x in np.linspace(20,170,151):
            dx=floor_x-x; mask=abs(dx)<=D/2
            z=float(np.max(floor_z[mask]+np.sqrt(np.maximum(0,(D/2)**2-dx[mask]**2))))+.02
            v=intersect(jar(D,L,x,z,dy,yaw),obstacles)
            if v>maxv: maxv,atx=v,float(x)
            if v>.1: hits+=1
        paths.append({'diameter':D,'length':L,'yaw':yaw,'offset':dy,'max_mm3':round(maxv,5),'x_at_max':atx,'hits':hits})
out('JAR_EXIT_STRUCTURE_SCREEN',paths)

# Rear transfer geometry: front-to-rear drawing convention, radius clearance to rear inside face.
rear_gap=585-534
out('REAR_TRANSFER_CLEARANCE',{'actual_upper_gap_mm':rear_gap,'drawing_gap_claim_mm':590-534,
    'nominal_diameter47_clear_mm':rear_gap-47,'diameter50_clear_mm':rear_gap-50,
    'lower_gap_mm':585-565,'upper_end_z':239.99956681939133,'lower_end_z':186.69256391280626})

# Reduced physical model is explicitly separate from the viewer's scripted coordinates.
import sys
ref=Path(r'BX04_ПРОЕКТ_ПРОТОТИП\REFERENCE_D2\SOURCE')  # внешний источник (не в репозитории)
sys.path.insert(0,str(ref))
simsource=(ref/'simulate.py').read_text(encoding='utf-8')
simsource=simsource.replace("if record:(OUT/'trace.json').write_text(json.dumps(trace,separators=(',',':')))",
                          "if record:globals()['audit_trace']=trace")
simns={'__file__':str(ref/'simulate.py'),'__name__':'audit_simulation'}
exec(compile(simsource,str(ref/'simulate.py'),'exec'),simns)
simns['EDGE']=45.
out('REDUCED_CONTACT_SIMULATION', [simns['run'](count,D,.75,.02,.002,False) for count,D in ((1,47),(3,47),(10,47),(10,49),(10,50))])
simns['run'](3,47,.75,.02,.002,True)
tr=simns['audit_trace']
hit=[]; maxv=0.
for f in tr[::4]:
    q=f['q']; rk=placed('B402',q)
    for j,(u,n,visible) in enumerate(f['cans']):
        if not visible or f['states'][j]!=0: continue
        pt=mbase(YCS[2]) @ np.array([u,0,n,1.])
        v=intersect(jar(47,67,pt[0],pt[2]),rk)
        if v>.1: hit.append({'t':f['t'],'jar':j,'q':q,'mm3':v})
        maxv=max(v,maxv)
out('JAR_FULL_ROCKER_PROFILE_SCREEN',{'samples':len(tr[::4]),'hits':len(hit),'max_mm3':maxv,'examples':hit[:5]})

# Numerical confirmation of viewer / actual STL agreement by volume and translation-invariant geometry.
out('VIEWER_ROUNDER_MESH',data['meshes']['PROSTAVKA'].keys())
out('PHASE2_COMPLETE',True)

def viewer_mesh(key):
    g=data['meshes'][key]
    m=trimesh.Trimesh(vertices=np.asarray(g['v']).reshape(-1,3),faces=np.asarray(g['f']).reshape(-1,3),process=True)
    m.fix_normals()
    return m
out('SERIALIZED_VIEWER_COLLISIONS',{
    'wall_div1_mono2_mm3':intersect(viewer_mesh('CAS_div1'),viewer_mesh('MONO2')),
    'lock_components':len(viewer_mesh('ZAMOK_SLIDE').split()),
    'spacer_euler_number':viewer_mesh('PROSTAVKA').euler_number})
p1=trimesh.load(next((ROOT/'ПЕЧАТЬ_ДОКА').glob('1 *.stl')),force='mesh')
big=max(p1.split(),key=lambda m:m.volume)
pred=block2(YCS[0],YCS[1]); pred.apply_translation(big.bounds[0]-pred.bounds[0])
out('ACTUAL_PRINT_BLOCK_MATCH',{'volume_difference_mm3':float(big.volume-pred.volume),
    'max_bounds_difference_mm':float(np.max(abs(big.bounds-pred.bounds))),
    'symmetric_difference_mm3':float(_D(big,pred).volume+_D(pred,big).volume)})
# Inspect the actual plate's nominal insert located at x=6,y=176 and return to assembly coordinates.
nom=next(c for c in p1.split() if np.allclose(c.bounds[0,:2],[6,176],atol=.01))
FLIP=trimesh.transformations.rotation_matrix(math.pi,[0,1,0])
printed=insert_mesh(2.8); printed.apply_transform(FLIP)
nom.apply_translation(printed.bounds[0]-nom.bounds[0]); nom.apply_transform(FLIP)
nom.apply_translation([0,YCS[2],0])
out('ACTUAL_PRINT_INSERT_COLLISION_MM3',intersect(nom,MONO[2]))

# Main bearing dimensions measured from vertices of the actual solids.
def radial_values(p,axis_z):
    v=mesh(p).vertices
    rr=np.hypot(v[:,0],v[:,2]-axis_z)
    return sorted(set(np.round(rr[rr<4.01],4)))
out('BEARING_DIMENSIONS',{'rocker_centre_radii_mm':radial_values('B402',58),
    'bush_centre_radii_mm':radial_values('B404',58),
    'holder_centre_radii_mm':radial_values('B401',58)})

# Fused block assembly: lower FEMALE block onto already placed MALE block (reverse order is blocked).
right=block2(YCS[2],YCS[3]); left=block2(YCS[0],YCS[1])
out('ACTUAL_FUSED_BLOCK_ASSEMBLY_MM3',{str(dz):intersect(left,trimesh.Trimesh(vertices=right.vertices+[0,0,dz],faces=right.faces,process=True))
    for dz in (0,.5,1,2,4,6,12,30,60,120)})
out('FINAL_AUDIT_COMPLETE',True)
