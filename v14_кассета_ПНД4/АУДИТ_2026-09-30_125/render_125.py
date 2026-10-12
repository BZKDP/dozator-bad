"""Audit illustrations from actual sections and saved simulation results."""
from two_tier_physics import ns,cp,ROOT,HERE,pack,np,trimesh,math,json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

def section(ax,m,color,origin_y=ns['YCS'][2],lw=1.7,label=None):
    sec=m.section(plane_normal=[0,1,0],plane_origin=[0,origin_y,0])
    if sec is None:return
    for i,v in enumerate(sec.discrete):ax.plot(v[:,0],v[:,2],color=color,lw=lw,label=label if i==0 else None)

def cas(ax):
    for k,c in [('lower','#157cb0'),('upper','#157cb0'),('rear','#805b36')]:section(ax,ns['rot_cas'](cp[k]['mesh']),c,label=k)

fig=plt.figure(figsize=(15,10),layout='constrained')
gs=fig.add_gridspec(2,2,height_ratios=[1.2,1])
ax=fig.add_subplot(gs[0,:]);cas(ax);section(ax,ns['MONO'][2],'#b87532',lw=1.1)
for i,(x,z) in enumerate(pack(47)*1000):
    c='#71abd2' if i<14 else '#e4a141' if i==14 else '#6fa480'
    ax.add_patch(Circle((x,z),23.5,facecolor=c,alpha=.75,edgecolor='#24394d',lw=.8))
    ax.text(x,z,str(i+1),ha='center',va='center',fontsize=8)
ax.axvline(0,color='#888',ls=':',lw=1)
ax.set(xlim=(-620,185),ylim=(-10,415),title='25 банок Ø47×67 на канал: 14 снизу + 1 в переходе + 10 сверху',xlabel='X, мм · кассета слева от 0; док справа',ylabel='Z, мм')
ax.text(-310,390,'5 каналов × 25 = 125 банок · 60 г × 125 = 7,5 кг',ha='center',fontsize=12,fontweight='bold')
ax.annotate('Первые банки уже внутри дока',xy=(59.5,43.7),xytext=(-180,90),arrowprops={'arrowstyle':'->'},fontsize=10)
ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.17)

ax=fig.add_subplot(gs[1,0]);cas(ax)
results=json.loads((HERE/'physics-full-rocker.json').read_text(encoding='utf-8'))
jam=results['cases'][0]
for idx,x,z in jam['remaining']:
    ax.add_patch(Circle((x,z),23.5,facecolor='#d39c64' if idx==17 else '#76a381',alpha=.7,edgecolor='#24394d'))
    ax.text(x,z,str(idx),ha='center',va='center',fontsize=9,clip_on=True)
ax.set(xlim=(-615,-405),ylim=(155,335),title='Риск: после 16 выдач верхняя очередь зависла',xlabel='X, мм',ylabel='Z, мм')
ax.annotate('Банка у задней стенки\nподпёрта следующей',xy=(-561.5,266.2),xytext=(-610,321),arrowprops={'arrowstyle':'->'},fontsize=10)
ax.text(-545,176,'Нижняя очередь пуста',fontsize=10)
ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.17)

ax=fig.add_subplot(gs[1,1])
g=ns['geoms']['H_horn'];horn=trimesh.Trimesh(vertices=g['v'],faces=g['f'],process=True)
horn.apply_transform(ns['pose_matrix']('horn',0))
sec=horn.section(plane_normal=[0,1,0],plane_origin=[0,3,0])
for v in sec.discrete:ax.plot(v[:,0],v[:,2],color='#b87532',lw=1.5)
H=ns['drive'](0)[1]
g=ns['geoms']['H_pin_horn'];m=trimesh.Trimesh(vertices=g['v'],faces=g['f'],process=True)
v=m.vertices;ring=v[np.isclose(v[:,1],v[:,1].max())];pc=ring[:,[0,2]].mean(axis=0)
actual=H+pc
ax.add_patch(Circle(tuple(H),1.15*10/9,fill=False,edgecolor='#27865b',lw=2,label='Отверстие и ось тяги'))
ax.add_patch(Circle(tuple(actual),10/9,fill=False,edgecolor='#d62a3a',lw=2,label='Фактическая ось винта'))
ax.plot([H[0],actual[0]],[H[1],actual[1]],'--',color='#d62a3a')
ax.text((H[0]+actual[0])/2+.2,(H[1]+actual[1])/2,'≈11 мм',fontsize=11,color='#d62a3a')
ax.set(xlim=(H[0]-4,actual[0]+4),ylim=(actual[1]-4,H[1]+4),title='Новый CAD-баг: винт масштабирован от чужого начала',xlabel='u, мм',ylabel='n, мм')
ax.set_aspect('equal',adjustable='box');ax.legend(loc='lower left',fontsize=9);ax.grid(alpha=.17)
fig.suptitle('V14 / раунд 3 — вместимость есть, надёжность подачи ещё не подтверждена',fontsize=17,fontweight='bold')
fig.savefig(HERE/'audit-125.png',dpi=150)
print('AUDIT_125_IMAGE_READY',flush=True)
