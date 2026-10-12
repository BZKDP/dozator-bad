"""Actual sections of the repaired V14. Never executes design exporters."""
from pathlib import Path
import sys, math
sys.stdout.reconfigure(encoding='utf-8')
import numpy as np
import trimesh
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as PatchPolygon, Circle

ROOT=Path(__file__).resolve().parents[1]
source=(ROOT/'make_dock_full_3d.py').read_text(encoding='utf-8')
ns={}
exec(compile(source.split('# ============ экспорт')[0],str(ROOT/'make_dock_full_3d.py'),'exec'),ns)

def model(key):
    g=ns['geoms'][key]
    m=trimesh.Trimesh(vertices=g['v'],faces=g['f'],process=True)
    m.fix_normals()
    return m

def intersection(a,b):
    return trimesh.boolean.intersection([a,b],engine='manifold')

def outline(ax,m,normal,origin,columns,color,label,offset=(0,0),fill=False):
    section=m.section(plane_normal=normal,plane_origin=origin)
    if section is None:return
    for k,v in enumerate(section.discrete):
        xy=v[:,columns]-np.asarray(offset)
        if fill:ax.add_patch(PatchPolygon(xy,facecolor=color,alpha=.5,edgecolor=color))
        ax.plot(xy[:,0],xy[:,1],color=color,lw=1.7,label=label if k==0 else None)

yc=ns['YCS'][2]
mono=ns['MONO'][2]
insert=ns['shifted'](ns['insert_mesh'](1.8),yc)
ix=intersection(mono,insert)
localframe=mono.copy().apply_transform(np.linalg.inv(ns['mbase'](yc)))
lock=model('ZAMOK_SLIDE')
lockix=intersection(localframe,lock)

fig,axs=plt.subplots(2,2,figsize=(14,10),layout='constrained')
fig.suptitle('Повторный аудит V14 — сечения реальных деталей после исправлений',fontsize=16,fontweight='bold')

ax=axs[0,0]
outline(ax,mono,[1,0,0],[10,0,0],[1,2],'#bd7629','Боковая стенка дока',offset=(yc,0),fill=True)
outline(ax,insert,[1,0,0],[10,0,0],[1,2],'#7854ac','Узкий вкладыш (3 метки)',offset=(yc,0))
outline(ax,ix,[1,0,0],[10,0,0],[1,2],'#d52231','Конфликт',offset=(yc,0),fill=True)
ax.set(xlim=(33,41.5),ylim=(3,30),xlabel='Y от центра канала, мм',ylabel='Z, мм',title='1. Узкий вкладыш входит в стенку на 0,7 мм')
ax.annotate('Не основание: стенка выше Z=6',xy=(37.85,12),xytext=(33.2,29),arrowprops={'arrowstyle':'->'},fontsize=9)
ax.legend(loc='lower left',fontsize=8)

ax=axs[0,1]
outline(ax,localframe,[0,0,1],[0,0,100],[0,1],'#bd7629','Балка',fill=True)
outline(ax,lock,[0,0,1],[0,0,100],[0,1],'#558fa0','Замок')
outline(ax,lockix,[0,0,1],[0,0,100],[0,1],'#d52231','Ручка в краях входа',fill=True)
ax.set(xlim=(46,56),ylim=(3.5,18.5),xlabel='u вдоль балки, мм',ylabel='y, мм',title='2. Цельная ручка мешает досадить замок')
ax.annotate('1 мм недохода',xy=(49.5,5.3),xytext=(51,3.9),arrowprops={'arrowstyle':'->'},fontsize=10)
ax.legend(loc='upper right',fontsize=8)

ax=axs[1,0]
horn=model('H_horn').apply_transform(ns['pose_matrix']('horn',0))
Q,H,B=ns['drive'](0)
hole=(ns['pose_matrix']('horn',0)@np.array([-46.,0.,82.,1.]))[[0,2]]
outline(ax,horn,[0,1,0],[0,3,0],[0,2],'#bd7629','Реальная 3D-качалка',fill=True)
ax.add_patch(Circle(tuple(hole),1.15,facecolor='white',edgecolor='#27884d',lw=2,label='Отверстие: плечо 9'))
ax.add_patch(Circle(tuple(H),1.,fill=False,edgecolor='#164aab',ls=':',lw=1.5,label='Габарит винта M2'))
ax.plot(H[0],H[1],'+',color='#164aab',ms=14,mew=2,label='Ось тяги: расчёт 10')
ax.plot([hole[0],H[0]],[hole[1],H[1]],'--',color='#d52231',lw=2)
ax.set(xlim=(H[0]-4,H[0]+3),ylim=(H[1]-4,H[1]+3),xlabel='u, мм',ylabel='n, мм',title='3. Центры отверстия и тяги расходятся на 1 мм')
ax.legend(loc='lower left',fontsize=8)

ax=axs[1,1]
left=ns['CHK']['L'].copy().apply_translation([0,0,4])
frame=ns['MONO'][0]
leftix=intersection(left,frame)
outline(ax,frame,[1,0,0],[55,0,0],[1,2],'#bd7629','Первый блок',fill=True)
outline(ax,left,[1,0,0],[55,0,0],[1,2],'#558fa0','Левая щека выше на 4 мм')
outline(ax,leftix,[1,0,0],[55,0,0],[1,2],'#d52231','Шип упирается в стенку',fill=True)
ax.set(xlim=(-5,16),ylim=(-1,20),xlabel='Y, мм',ylabel='Z, мм',title='4. Левую щеку нельзя опустить сверху в блок')
ax.annotate('Сначала щека на стол,\nзатем блок опустить на неё',xy=(7.5,8),xytext=(-4,18),arrowprops={'arrowstyle':'->'},fontsize=9)
ax.legend(loc='lower right',fontsize=8)

for ax in axs.flat:
    ax.grid(alpha=.18)
    ax.set_aspect('equal',adjustable='box')
fig.savefig(Path(__file__).resolve().parent/'findings.png',dpi=150)
print('FINDINGS_IMAGE_READY')
