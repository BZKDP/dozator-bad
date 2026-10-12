"""125 real jar meshes, fit paths, closed-position force direction, old control source check."""
from two_tier_physics import ns,cp,ROOT,HERE,pack,BASE,np,trimesh,math,json

def volume(a,b):
    if np.any(a.bounds[1]<=b.bounds[0]) or np.any(b.bounds[1]<=a.bounds[0]):return 0.
    m=trimesh.boolean.intersection([a,b],engine='manifold')
    return float(m.volume) if m is not None and len(m.faces) else 0.
def gm(k):
    g=ns['geoms'][k];m=trimesh.Trimesh(vertices=g['v'],faces=g['f'],process=True);m.fix_normals();return m
def placed(k,q=-28):return gm(k).apply_transform(BASE@ns['pose_matrix'](ns['geoms'][k].get('motion','fixed'),q))

result={}
obstacles={'cassette_'+k:ns['rot_cas'](v['mesh']) for k,v in cp.items()}
obstacles.update({'dock'+str(i):m for i,m in enumerate(ns['MONO'])})
obstacles.update({'CHEEK_'+s:m for s,m in ns['CHK'].items()})
hits=[];centres=[]
positions=pack(47)*1000
for lane,yc in enumerate(ns['YCS']):
    for idx,(x,z) in enumerate(positions):
        jar=trimesh.creation.cylinder(radius=23.5,height=67,sections=96)
        jar.apply_transform(trimesh.transformations.rotation_matrix(math.pi/2,[1,0,0]))
        jar.apply_translation([x,yc,z]);centres.append([lane+1,idx+1,float(x),float(yc),float(z)])
        for k,b in obstacles.items():
            v=volume(jar,b)
            if v>.1:hits.append({'lane':lane+1,'jar':idx+1,'part':k,'mm3':v})
result['ALL_125_STATIC']={'jar_count':len(centres),'collisions':hits,'centres':centres,'scope':'nominal 47 x 67 mm, no yaw, nominal 5mm wall geometry'}

frame=ns['MONO'][2];axis=BASE[:3,:3]@np.array([1.,0.,0.])
result['LOCK_INSERTION']={str(du):volume(frame,placed('ZAMOK_SLIDE').apply_translation(du*axis)) for du in (-1.1,-1,-.5,0,.5,1,2,5,10,20)}
result['HOLDER_LOWERING']={str(dz):volume(frame,placed('B401').apply_translation([0,0,dz])) for dz in (0,.2,.5,1,2,3,5,10,20,40,80)}

# Real 2-channel STL matches rebuilt block; actual test narrow component matches it too.
p1=trimesh.load(next((ROOT/'ПЕЧАТЬ_ДОКА').glob('1 *.stl')),force='mesh')
actual=max(p1.split(),key=lambda x:x.volume)
expected=ns['block2'](ns['YCS'][0],ns['YCS'][1]);expected.apply_translation(actual.bounds[0]-expected.bounds[0])
result['ACTUAL_SAVED_FRAME_DIFFERENCE_MM3']=float(ns['_D'](actual,expected).volume+ns['_D'](expected,actual).volume)
test=trimesh.load(next((ROOT/'ПЕЧАТЬ_ДОКА').glob('ТЕСТ*.stl')),force='mesh')
actualins=next(m for m in test.split() if np.allclose(m.bounds[0,:2],[110,100],atol=.01))
flip=trimesh.transformations.rotation_matrix(math.pi,[0,1,0]);pred=ns['insert_mesh'](1.8).apply_transform(flip)
actualins.apply_translation(pred.bounds[0]-actualins.bounds[0]);actualins.apply_transform(flip)
actualins.apply_translation([0,ns['YCS'][2],0])
result['ACTUAL_SAVED_TEST_INSERT_COLLISION_MM3']=volume(actualins,frame)

q=math.radians(-28);f=np.array([19.5*math.cos(q)+12*math.sin(q),58+19.5*math.sin(q)-12*math.cos(q)])
u=ns['uc1'];j=np.array([u,23.5]);n=(f-j)/np.linalg.norm(f-j);r=f-np.array([0.,58.])
moment=r[0]*n[1]-r[1]*n[0]
ratio=(ns['drive'](-27.99)[2]-ns['drive'](-28)[2])/.01
result['IDLE_CLOSED_CONTACT_DIRECTION']={'moment_arm_mm_per_newton':float(moment),'positive_q_opens':True,
   'dservo_drocker':ratio,'implication':'load tends toward opening; back-drive resistance of actual powered/unpowered servo is not measured'}
result['SIDE_CLEARANCE']=[{'L':L,'D':D,'yaw':y,'projected_width':L*math.cos(math.radians(y))+D*math.sin(math.radians(y)),
    'one_side_centred_clearance':(75-L*math.cos(math.radians(y))-D*math.sin(math.radians(y)))/2}
    for L,D in ((67,47),(70,49),(72,50)) for y in (0,5,10)]
for k,v in result.items():print(k,json.dumps(v,ensure_ascii=False),flush=True)
(HERE/'extra-results.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print('EXTRA_FINISHED',flush=True)
