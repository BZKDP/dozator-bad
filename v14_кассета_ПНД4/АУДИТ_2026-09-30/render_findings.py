"""Render actual mesh sections. Reads design, writes only audit PNG."""
from pathlib import Path
import numpy as np
import trimesh
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as PatchPolygon, Circle

ROOT=Path(__file__).resolve().parents[1]
ns={}
src=(ROOT/'make_dock_full_3d.py').read_text(encoding='utf-8')
exec(compile(src.split('# ============ экспорт')[0],str(ROOT/'make_dock_full_3d.py'),'exec'),ns)

def outline(ax,m,normal,origin,columns,color,label,offset=(0,0),fill=False):
    section=m.section(plane_normal=normal,plane_origin=origin)
    if section is None:return
    for k,v in enumerate(section.discrete):
        xy=v[:,columns]-np.asarray(offset)
        if fill:ax.add_patch(PatchPolygon(xy,facecolor=color,alpha=.6,edgecolor=color))
        ax.plot(xy[:,0],xy[:,1],color=color,lw=1.6,label=label if k==0 else None)

yc=ns['YCS'][2]
mono=ns['MONO'][2]
ins=ns['shifted'](ns['insert_mesh'](2.8),yc)
wall=next(m for key,m,*_ in ns['CAS'] if key=='div1')
whole=ns['_U'](ns['MONO'])
ix_wall=trimesh.boolean.intersection([whole,wall],engine='manifold')
ix_ins=trimesh.boolean.intersection([mono,ins],engine='manifold')
lock=ns['_zl']
spacer=trimesh.Trimesh(vertices=ns['geoms']['PROSTAVKA']['v'],faces=ns['geoms']['PROSTAVKA']['f'],process=True)

fig,axs=plt.subplots(2,2,figsize=(13,9),layout='constrained')
fig.suptitle('V14 / V2.5.1 — ошибки в реальной геометрии (мм)',fontsize=17,fontweight='bold')
ax=axs[0,0]
outline(ax,wall,[0,1,0],[0,241,0],[0,2],'#967047','Картон')
outline(ax,mono,[0,1,0],[0,241,0],[0,2],'#dc832a','Док')
outline(ax,ix_wall,[0,1,0],[0,241,0],[0,2],'#d73535','Пересечение',fill=True)
ax.set(xlim=(-40,45),ylim=(-2,90),xlabel='X — вдоль канала',ylabel='Z — высота',title='1. Передний край стенки не входит в док')
ax.annotate('24 мм перекрытия',xy=(-12,55),xytext=(8,80),arrowprops={'arrowstyle':'->'},fontsize=11)
ax.legend(loc='lower right',fontsize=9)

ax=axs[0,1]
outline(ax,mono,[1,0,0],[10,0,0],[1,2],'#dc832a','Док',offset=(yc,0))
outline(ax,ins,[1,0,0],[10,0,0],[1,2],'#8159b4','Вкладыш',offset=(yc,0))
outline(ax,ix_ins,[1,0,0],[10,0,0],[1,2],'#d73535','Пересечение',offset=(yc,0),fill=True)
ax.set(xlim=(25,44),ylim=(-2,43),xlabel='Y от центра канала',ylabel='Z — высота',title='2. Нижняя щека вкладыша входит в основание')
ax.annotate('Основание 0–6 мм занято',xy=(35.5,3),xytext=(26,37),arrowprops={'arrowstyle':'->'},fontsize=10)
ax.legend(loc='upper right',fontsize=9)

ax=axs[1,0]
for i,c in enumerate(lock.split()):
    outline(ax,c,[0,0,1],[0,0,97],[0,1],['#d39828','#d73535','#ce5151','#ed7272'][i],f'Часть {i+1}',fill=True)
ax.set(xlim=(28,56),ylim=(3,19),xlabel='u — локальная координата',ylabel='y — локальная координата',title='3. Один замок = четыре несвязанных тела')
ax.annotate('Ручка не соединена; насечки прорезаны насквозь',xy=(52.5,11),xytext=(29,18),arrowprops={'arrowstyle':'->'},fontsize=9)
ax.legend(loc='lower left',fontsize=8,ncol=4)

ax=axs[1,1]
outline(ax,spacer,[0,1,0],[0,4.5,0],[0,2],'#7aaf71','Сплошная проставка',fill=True)
ax.add_patch(Circle((0,0),1,fill=False,edgecolor='#d73535',ls='--',lw=2))
ax.set(xlim=(-4.5,4.5),ylim=(-4.5,4.5),xlabel='u',ylabel='n',title='4. Проставка Ø6×3 не имеет отверстия M2')
ax.annotate('Здесь материал, а не отверстие',xy=(0,0),xytext=(-4,3.6),arrowprops={'arrowstyle':'->'},fontsize=9)
for ax in axs.flat:
    ax.grid(alpha=.2)
    ax.set_aspect('equal',adjustable='box')
fig.savefig(Path(__file__).resolve().parent/'ПРОБЛЕМЫ_ГЕОМЕТРИИ.png',dpi=160)
print('AUDIT_PNG_READY',flush=True)
