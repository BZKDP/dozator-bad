"""Independent 2D rigid-body check of 25 x 60g jars, including finite upper sheet and rear.
This does not certify real motor torque, cardboard deformation or 3D yaw.
No bottle is teleported after initialization. Servo is an ideal kinematic driver.
"""
from pathlib import Path
import sys,json,math
sys.stdout.reconfigure(encoding='utf-8')
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE/'deps'))
import pymunk
import numpy as np
import trimesh
from scipy.optimize import brentq
from shapely.geometry import Polygon
from shapely.ops import triangulate

ROOT=HERE.parent
ns={}
src=(ROOT/'make_dock_full_3d.py').read_text(encoding='utf-8')
exec(compile(src.split('# ============ экспорт')[0],str(ROOT/'make_dock_full_3d.py'),'exec'),ns)
cs={'__file__':str(ROOT/'generate_cassette.py'),'__name__':'readonly'}
exec(compile((ROOT/'generate_cassette.py').read_text(encoding='utf-8'),str(ROOT/'generate_cassette.py'),'exec'),cs)
cp,*_=cs['build_parts']()
P=cs['P'];YC=ns['YCS'][2];BASE=ns['mbase'](YC)
front=(19.5,-12);rear=(-27,-16)

def pack(D):
    R=D/2;q=math.radians(-28)
    fu=19.5*math.cos(q)+12*math.sin(q)
    fn=58+19.5*math.sin(q)-12*math.cos(q)
    firstu=fu-math.sqrt((R+3)**2-(fn-R)**2)
    first=(BASE@np.array([firstu,0,R,1]))[[0,2]]
    lower=[];rearcentre=-(P['depth']-P['cardboard_thickness'])+R
    for i in range(40):
        a=first+i*D*np.array([-ns['C15'],ns['S15']])
        if a[0]<rearcentre-1e-6:break
        lower.append(a)
    a=lower[-1];trans=np.array([rearcentre,a[1]+math.sqrt(D*D-(rearcentre-a[0])**2)])
    ta=math.radians(P['upper_angle'])
    def uz(x):return P['upper_z0']+(x+25)*math.tan(ta)+R/math.cos(ta)
    ux=brentq(lambda x:(x-trans[0])**2+(uz(x)-trans[1])**2-D*D,trans[0]+.01,trans[0]+D+10)
    upper=[]
    for i in range(40):
        x=ux+i*D*math.cos(ta)
        if x+R*math.sin(ta)>-25+1e-6:break
        upper.append(np.array([x,uz(x)]))
    return np.asarray(lower+[trans]+upper)[:25]*.001

def q_from_beta(b):
    br=math.radians(b);H=np.array([-55.,82.])+10*np.array([math.cos(br),math.sin(br)])
    d=H-np.array([0.,58.]);dd=np.linalg.norm(d)
    co=(17.5**2+dd*dd-55.5**2)/(2*17.5*dd)
    return math.degrees(math.atan2(d[1],d[0])-math.acos(max(-1,min(1,co))))-90

def trial(D=47,mu=.3,rolling=.0,dt=.0005,cycles=25,travel=2.7,record=False,rear_friction=None,full_rocker=False):
    space=pymunk.Space();space.gravity=(0,-9.81);space.iterations=60
    space.collision_slop=.00002;space.collision_bias=pow(1-.3,60)
    static=[]
    for key in ('lower','upper','rear'):
        m=ns['rot_cas'](cp[key]['mesh'])
        sec=m.section(plane_normal=[0,1,0],plane_origin=[0,YC,0])
        for vertices in sec.discrete:
            points=vertices[:,[0,2]]*.001
            for i in range(len(points)-1):
                sh=pymunk.Segment(space.static_body,tuple(points[i]),tuple(points[i+1]),0)
                sh.friction=(rear_friction/math.sqrt(mu) if key=='rear' and rear_friction is not None and mu else math.sqrt(mu));sh.elasticity=.1;space.add(sh);static.append(sh)
    # Actual central section of the printable dock supplies its floor, finite chute and beam.
    sec=ns['MONO'][2].section(plane_normal=[0,1,0],plane_origin=[0,YC,0])
    for vertices in sec.discrete:
        points=vertices[:,[0,2]]*.001
        for i in range(len(points)-1):
            sh=pymunk.Segment(space.static_body,tuple(points[i]),tuple(points[i+1]),0)
            sh.friction=math.sqrt(mu);sh.elasticity=.1;space.add(sh);static.append(sh)
    gate=pymunk.Body(body_type=pymunk.Body.KINEMATIC)
    pivot=(BASE@np.array([0,0,58,1]))[[0,2]]*.001
    gate.position=tuple(pivot);gate.angle=math.radians(-28-15)
    space.add(gate)
    if full_rocker:
        rg=ns['geoms']['B402'];rm=trimesh.Trimesh(vertices=rg['v'],faces=rg['f'],process=True)
        sec=rm.section(plane_normal=[0,1,0],plane_origin=[0,0,0]);profile=Polygon()
        for vertices in sec.discrete:
            profile=profile.symmetric_difference(Polygon(vertices[:,[0,2]]))
        for tri in triangulate(profile):
            if not profile.covers(tri):continue
            verts=(np.asarray(tri.exterior.coords)[:-1]-[0,58])*.001
            sh=pymunk.Poly(gate,[tuple(v) for v in verts]);sh.friction=math.sqrt(mu);space.add(sh)
    else:
        for off in (front,rear):
            sh=pymunk.Circle(gate,.003,tuple(np.array(off)*.001));sh.friction=math.sqrt(mu);space.add(sh)
    cans=[];radius=D/2000
    for i,pt in enumerate(pack(D)):
        b=pymunk.Body(.060,.75*.060*radius*radius)
        b.position=tuple(pt+[0,.0001]);sh=pymunk.Circle(b,radius)
        sh.friction=math.sqrt(mu);sh.elasticity=.1;space.add(b,sh)
        cans.append((i,b,sh))
    issued=[];trace=[]
    b0=ns['drive'](-28)[2];b1=ns['drive'](28)[2]
    opening=travel;closing=travel*2.2/2.7
    duration=.5+opening+1.3+closing+.8
    def q_cycle(t):
        sm=lambda x:x*x*(3-2*x)
        if t<.5:beta=b0
        elif t<.5+opening:beta=b0+(b1-b0)*sm((t-.5)/opening)
        elif t<.5+opening+1.3:beta=b1
        elif t<.5+opening+1.3+closing:beta=b1-(b1-b0)*sm((t-(.5+opening+1.3))/closing)
        else:beta=b0
        return q_from_beta(beta)
    # Precompute motion. All jar positions thereafter come only from gravity/collisions.
    nsteps=round(duration/dt)
    motion=np.array([math.radians(q_cycle((j+1)*dt)-15) for j in range(nsteps)])
    pre_exits=[]
    def step(t,cycle,qnext):
        gate.angular_velocity=(qnext-gate.angle)/dt
        for _,b,_ in cans:
            if rolling and abs(b.angular_velocity)>1e-8:
                torque=rolling*.060*9.81*radius
                b.torque=-math.copysign(min(torque,abs(b.angular_velocity)*b.moment/dt),b.angular_velocity)
        space.step(dt)
        for item in list(cans):
            i,b,sh=item
            if b.position.x>.19:
                issued.append({'jar':i+1,'cycle':cycle,'time':t});cans.remove(item);space.remove(sh,b)
                if cycle<0:pre_exits.append(i+1)
        if record and round(t/dt)%round(.05/dt)==0:
            trace.append({'t':t,'cycle':cycle,'q':math.degrees(gate.angle)+15,
                'positions':[[i+1,b.position.x*1000,b.position.y*1000] for i,b,_ in cans]})
    for j in range(round(3/dt)):step((j+1)*dt,-1,math.radians(-28-15))
    settled=[[i+1,b.position.x*1000,b.position.y*1000] for i,b,_ in cans]
    cyclecounts=[]
    for cycle in range(1,cycles+1):
        before=len(issued)
        for j,qnext in enumerate(motion):step(3+(cycle-1)*duration+(j+1)*dt,cycle,qnext)
        cyclecounts.append(len(issued)-before)
    remain=[[i+1,b.position.x*1000,b.position.y*1000] for i,b,_ in cans]
    result={'D':D,'mass_g':60,'friction':mu,'rear_friction_override':rear_friction,'full_rocker':full_rocker,'rolling_torque_ratio':rolling,'dt':dt,'travel_seconds':travel,
        'initial_count':25,'exits_during_settling':pre_exits,'exits_each_cycle':cyclecounts,'total_exits':len(issued),
        'remaining':remain,'settled_initial_positions':settled,'result':'ONE_EACH' if not pre_exits and cyclecounts==[1]*cycles and not cans else 'FAIL',
        'model':'2D circular rigid jars; actual finite sheet/dock/rear sections; kinematic actual rocker section or nose rollers; no motor limit, no 3D yaw, no cardboard flexure'}
    if record:(HERE/'two-tier-trace.json').write_text(json.dumps(trace,separators=(',',':')),encoding='utf-8')
    print('TWO_TIER',json.dumps(result,ensure_ascii=False),flush=True)
    return result

if __name__=='__main__':
    cases=[(47,.3,0.,.0005,2.7),(49,.3,0.,.0005,2.7),(47,.3,.02,.0005,2.7),(49,.3,.02,.0005,2.7),
           (47,.15,.05,.0005,2.7),(47,.3,.02,.0005,.7)]
    results=[]
    for i,(D,mu,rr,dt,travel) in enumerate(cases):
        results.append(trial(D,mu,rr,dt,25,travel,record=i==0))
    (HERE/'two-tier-results.json').write_text(json.dumps({'pymunk':pymunk.version,'cases':results},ensure_ascii=False,indent=2),encoding='utf-8')
    print('TWO_TIER_FINISHED',flush=True)
