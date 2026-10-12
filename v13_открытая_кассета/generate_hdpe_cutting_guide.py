from __future__ import annotations

import math
from pathlib import Path


ROOT = Path(__file__).resolve().parent
THREE_PATH = ROOT.parent / "libs" / "three.module.js"
OUT = ROOT / "РАСКРОЙ_ПНД_1000x1000x4_3D.html"

SHEET = 1000.0
WIDTH = 405.0
THICKNESS = 4.0
LOWER = 540.0 / math.cos(math.radians(15.0))
UPPER = 509.0 / math.cos(math.radians(10.0))


def build() -> None:
    three = THREE_PATH.read_text(encoding="utf-8")
    app = r'''
const D={sheet:1000,width:405,thickness:4,lower:__LOWER__,upper:__UPPER__};
const scene=new Scene(); scene.background=new Color(0x071019);
const camera=new PerspectiveCamera(36,1,1,5000); camera.up.set(0,0,1);
const renderer=new WebGLRenderer({antialias:true});
renderer.setPixelRatio(Math.min(devicePixelRatio,2)); renderer.outputColorSpace=SRGBColorSpace;
renderer.shadowMap.enabled=true; renderer.shadowMap.type=PCFSoftShadowMap;
document.getElementById('view').appendChild(renderer.domElement);
scene.add(new HemisphereLight(0xd8efff,0x13202b,2.7));
const sun=new DirectionalLight(0xffffff,3.3); sun.position.set(-500,-650,1200); sun.castShadow=true; scene.add(sun);

const floor=new Mesh(new PlaneGeometry(1600,1500),new MeshStandardMaterial({color:0x101d27,roughness:.96}));
floor.position.set(500,500,-8); floor.receiveShadow=true; scene.add(floor);
const grid=new GridHelper(1400,28,0x284251,0x182d39); grid.rotation.x=Math.PI/2; grid.position.set(500,500,-7.5); scene.add(grid);

function box(w,h,t,color,opacity=1){
  const m=new Mesh(new BoxGeometry(w,h,t),new MeshStandardMaterial({color,roughness:.82,metalness:.01,transparent:opacity<1,opacity,side:DoubleSide}));
  m.castShadow=true; m.receiveShadow=true; return m;
}
const base=box(D.sheet,D.sheet,D.thickness,0x151a1e); base.position.set(500,500,0); scene.add(base);

const zones=new Group(); scene.add(zones);
function zone(name,x,y,w,h,color,label){
  const m=box(w-2,h-2,.8,color,.78); m.position.set(x+w/2,y+h/2,D.thickness/2+.45); m.userData={name,base:new Vector3(m.position.x,m.position.y,m.position.z)}; zones.add(m);
  const el=document.createElement('div'); el.className='piece'; el.innerHTML=label; document.getElementById('view').appendChild(el);
  return {mesh:m,el,p:new Vector3(x+w/2,y+h/2,D.thickness/2+2)};
}
const zoneLabels=[];
zoneLabels.push(zone('lower',0,0,D.width,D.lower,0x18b6d9,`<b>НИЖНИЙ СКАТ</b><br>${D.lower.toFixed(2)} × 405 × 4 мм`));
zoneLabels.push(zone('upper',405,0,D.width,D.upper,0x67d391,`<b>ВЕРХНИЙ СКАТ</b><br>${D.upper.toFixed(2)} × 405 × 4 мм`));
zoneLabels.push(zone('right',810,0,190,1000,0x46535c,`ОСТАТОК<br>190 × 1000 мм`));
zoneLabels.push(zone('top1',0,D.lower,405,1000-D.lower,0x303a42,`ОСТАТОК<br>405 × ${(1000-D.lower).toFixed(2)} мм`));
zoneLabels.push(zone('top2',405,D.upper,405,1000-D.upper,0x374149,`ОСТАТОК<br>405 × ${(1000-D.upper).toFixed(2)} мм`));

const cuts=[
 {a:[810,0],b:[810,1000],text:'1. Продольный рез: отделить полосу 190 мм'},
 {a:[405,0],b:[405,1000],text:'2. Продольный рез: две полосы по 405 мм'},
 {a:[0,D.lower],b:[405,D.lower],text:`3. Нижний скат: ${D.lower.toFixed(2)} × 405 мм`},
 {a:[405,D.upper],b:[810,D.upper],text:`4. Верхний скат: ${D.upper.toFixed(2)} × 405 мм`}
];
const cutVisuals=[];
for(const c of cuts){
 const dx=c.b[0]-c.a[0],dy=c.b[1]-c.a[1],len=Math.hypot(dx,dy);
 const rail=box(Math.max(3,Math.abs(dx)),Math.max(3,Math.abs(dy)),1.5,0xf6a623,.55);
 rail.position.set((c.a[0]+c.b[0])/2,(c.a[1]+c.b[1])/2,D.thickness/2+1.3); scene.add(rail);
 const progress=box(1,1,2.4,0xffe16d,1); progress.position.z=D.thickness/2+2.4; progress.visible=false; scene.add(progress);
 cutVisuals.push({rail,progress,dx,dy,len});
}

const knife=new Group();
const handle=box(28,92,18,0xf07e22); handle.position.set(0,-32,16); knife.add(handle);
const grip=box(22,58,19,0x24313a); grip.position.set(0,-31,16.2); knife.add(grip);
const bladeShape=new Shape(); bladeShape.moveTo(-9,0); bladeShape.lineTo(9,0); bladeShape.lineTo(3,52); bladeShape.lineTo(-3,52); bladeShape.closePath();
const blade=new Mesh(new ExtrudeGeometry(bladeShape,{depth:2,bevelEnabled:false}),new MeshStandardMaterial({color:0xdce5e8,metalness:.75,roughness:.24})); blade.rotation.x=Math.PI/2; blade.position.set(0,7,5); knife.add(blade);
knife.scale.setScalar(.72); knife.visible=false; scene.add(knife);

const dims=new Group(); scene.add(dims); const dimLabels=[];
function line(a,b,color=0xffdc69){const g=new BufferGeometry().setFromPoints([new Vector3(...a),new Vector3(...b)]);const l=new Line(g,new LineBasicMaterial({color}));dims.add(l);return l}
function dim(a,b,text,p,tickAxis='y'){
 line(a,b); const tick=14;
 for(const q of [a,b]){const aa=[...q],bb=[...q]; if(tickAxis==='y'){aa[1]-=tick;bb[1]+=tick}else if(tickAxis==='x'){aa[0]-=tick;bb[0]+=tick}else{aa[2]-=tick/2;bb[2]+=tick/2} line(aa,bb)}
 const el=document.createElement('div');el.className='dim';el.textContent=text;document.getElementById('view').appendChild(el);dimLabels.push({el,p:new Vector3(...p)});
}
const z=8;
dim([0,-55,z],[1000,-55,z],'1000 мм',[500,-55,z],'y');
dim([-55,0,z],[-55,1000,z],'1000 мм',[-55,500,z],'x');
dim([0,-22,z],[405,-22,z],'405 мм',[202.5,-22,z],'y');
dim([405,-22,z],[810,-22,z],'405 мм',[607.5,-22,z],'y');
dim([810,-22,z],[1000,-22,z],'190 мм',[905,-22,z],'y');
dim([-20,0,z],[-20,D.lower,z],`${D.lower.toFixed(2)} мм`,[-20,D.lower/2,z],'x');
dim([830,0,z],[830,D.upper,z],`${D.upper.toFixed(2)} мм`,[830,D.upper/2,z],'x');
dim([1040,0,-2],[1040,0,2],'4 мм',[1040,0,0],'z');

const originEl=document.createElement('div');originEl.className='factory';originEl.textContent='ЗАВОДСКАЯ КРОМКА — ОТ НЕЁ ОТКЛАДЫВАТЬ ДЛИНУ';document.getElementById('view').appendChild(originEl);
const originPoint=new Vector3(405,2,9);

let yaw=-1.57,pitch=1.55,dist=1480,target=new Vector3(500,470,0),drag=false,lastX=0,lastY=0;
function updateCamera(){const cp=Math.cos(pitch);camera.position.set(target.x+dist*cp*Math.cos(yaw),target.y+dist*cp*Math.sin(yaw),target.z+dist*Math.sin(pitch));camera.lookAt(target)}
renderer.domElement.addEventListener('pointerdown',e=>{drag=true;lastX=e.clientX;lastY=e.clientY;renderer.domElement.setPointerCapture(e.pointerId)});
renderer.domElement.addEventListener('pointerup',()=>drag=false);
renderer.domElement.addEventListener('pointercancel',()=>drag=false);
renderer.domElement.addEventListener('pointermove',e=>{if(!drag)return;yaw-=(e.clientX-lastX)*.006;pitch=Math.max(.08,Math.min(1.56,pitch+(e.clientY-lastY)*.005));lastX=e.clientX;lastY=e.clientY;updateCamera()});
renderer.domElement.addEventListener('wheel',e=>{e.preventDefault();dist=Math.max(650,Math.min(2600,dist*Math.exp(e.deltaY*.001)));updateCamera()},{passive:false});
function preset(v){if(v==='top'){yaw=-Math.PI/2;pitch=1.555;dist=1480}else if(v==='iso'){yaw=-2.18;pitch=.68;dist=1750}else{yaw=-Math.PI/2;pitch=.16;dist=1750}updateCamera()}
document.querySelectorAll('[data-view]').forEach(b=>b.onclick=()=>preset(b.dataset.view));

let playing=false,elapsed=0,lastTime=performance.now(),speed=1;
const CUT_TIME=2.7,PAUSE=.65,TOTAL=cuts.length*(CUT_TIME+PAUSE);
function resetAnimation(){playing=false;elapsed=0;knife.visible=false;for(const v of cutVisuals){v.progress.visible=false;v.progress.scale.set(1,1,1)}setStatus(-1,0);document.getElementById('play').textContent='▶ Запустить анимацию'}
function setStatus(active,progress){
 document.querySelectorAll('.cutstep').forEach((el,i)=>{el.classList.toggle('active',i===active);el.classList.toggle('done',i<active||(active===cuts.length&&i<cuts.length))});
 if(active<0){document.getElementById('status').textContent='Готово к показу раскроя'}
 else if(active>=cuts.length){document.getElementById('status').textContent='Раскрой закончен: две детали + три остатка'}
 else document.getElementById('status').textContent=`Рез ${active+1}/4 · ${Math.round(progress*100)}%`;
}
function updateCut(t){
 for(const v of cutVisuals){v.progress.visible=false}
 if(t>=TOTAL){knife.visible=false;for(const v of cutVisuals){v.progress.visible=true;const vertical=Math.abs(v.dy)>Math.abs(v.dx);v.progress.scale.set(vertical?5:v.len,vertical?v.len:5,1);v.progress.position.set((v===cutVisuals[0]?810:v===cutVisuals[1]?405:(v===cutVisuals[2]?202.5:607.5)),(v===cutVisuals[0]||v===cutVisuals[1])?500:(v===cutVisuals[2]?D.lower:D.upper),D.thickness/2+2.4)}setStatus(cuts.length,1);return}
 const slot=CUT_TIME+PAUSE,idx=Math.min(cuts.length-1,Math.floor(t/slot)),local=t-idx*slot,p=Math.min(1,local/CUT_TIME);
 for(let i=0;i<idx;i++){const c=cuts[i],v=cutVisuals[i],vertical=Math.abs(v.dy)>Math.abs(v.dx);v.progress.visible=true;v.progress.scale.set(vertical?5:v.len,vertical?v.len:5,1);v.progress.position.set((c.a[0]+c.b[0])/2,(c.a[1]+c.b[1])/2,D.thickness/2+2.4)}
 const c=cuts[idx],v=cutVisuals[idx],vertical=Math.abs(v.dy)>Math.abs(v.dx);v.progress.visible=p>0;v.progress.scale.set(vertical?5:Math.max(1,v.len*p),vertical?Math.max(1,v.len*p):5,1);v.progress.position.set(c.a[0]+v.dx*p/2,c.a[1]+v.dy*p/2,D.thickness/2+2.4);
 knife.visible=local<=CUT_TIME;knife.position.set(c.a[0]+v.dx*p,c.a[1]+v.dy*p,34);knife.rotation.z=Math.atan2(v.dy,v.dx)-Math.PI/2;
 setStatus(idx,p);
}
document.getElementById('play').onclick=()=>{if(elapsed>=TOTAL)elapsed=0;playing=!playing;document.getElementById('play').textContent=playing?'⏸ Пауза':'▶ Продолжить';if(!playing)knife.visible=false};
document.getElementById('restart').onclick=()=>{resetAnimation();playing=true;document.getElementById('play').textContent='⏸ Пауза'};
document.getElementById('speed').oninput=e=>{speed=+e.target.value;document.getElementById('speedOut').textContent=speed.toFixed(1)+'×'};
document.getElementById('dimensions').onchange=e=>{dims.visible=e.target.checked;document.querySelectorAll('.dim').forEach(x=>x.style.display=e.target.checked?'block':'none')};
document.getElementById('waste').onchange=e=>{for(const z of zoneLabels.filter(x=>x.mesh.userData.name!=='lower'&&x.mesh.userData.name!=='upper')){z.mesh.visible=e.target.checked;z.el.style.display=e.target.checked?'block':'none'}};
let exploded=false;
document.getElementById('explode').onclick=()=>{exploded=!exploded;const shifts={lower:[-32,-22,22],upper:[25,-22,22],right:[65,0,15],top1:[-25,50,15],top2:[25,55,15]};for(const q of zoneLabels){const s=shifts[q.mesh.userData.name];q.mesh.position.copy(q.mesh.userData.base).add(exploded?new Vector3(...s):new Vector3());q.p.copy(q.mesh.position).add(new Vector3(0,0,3))}document.getElementById('explode').textContent=exploded?'Собрать лист':'Разнести детали'};

function projectLabel(el,p){const v=p.clone().project(camera),w=renderer.domElement.clientWidth,h=renderer.domElement.clientHeight;el.style.transform=`translate(-50%,-50%) translate(${(v.x*.5+.5)*w}px,${(-v.y*.5+.5)*h}px)`;el.style.opacity=(v.z>1||v.z<-1)?0:1}
function resize(){const el=document.getElementById('view');renderer.setSize(el.clientWidth,el.clientHeight,false);camera.aspect=el.clientWidth/el.clientHeight;camera.updateProjectionMatrix()}
addEventListener('resize',resize);resize();updateCamera();resetAnimation();
function animate(now){requestAnimationFrame(animate);const dt=Math.min(.05,(now-lastTime)/1000);lastTime=now;if(playing){elapsed+=dt*speed;if(elapsed>=TOTAL){elapsed=TOTAL;playing=false;document.getElementById('play').textContent='▶ Повторить'}updateCut(elapsed)}for(const l of dimLabels)projectLabel(l.el,l.p);for(const l of zoneLabels)projectLabel(l.el,l.p);projectLabel(originEl,originPoint);renderer.render(scene,camera)}requestAnimationFrame(animate);
'''.replace('__LOWER__', f'{LOWER:.6f}').replace('__UPPER__', f'{UPPER:.6f}')

    html = r'''<!doctype html><html lang="ru"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Раскрой ПНД 1000×1000×4</title><style>
*{box-sizing:border-box}body{margin:0;background:#071019;color:#edf6fa;font:15px/1.42 "Segoe UI",Arial,sans-serif;overflow:hidden}#app{display:grid;grid-template-columns:minmax(0,1fr) 410px;height:100vh}#view{position:relative;min-width:0;overflow:hidden}canvas{display:block;width:100%;height:100%}aside{background:#111e29;border-left:1px solid #2a4557;padding:18px;overflow:auto}h1{font-size:23px;line-height:1.15;margin:0 0 6px}h2{font-size:15px;color:#8eddf8;margin:20px 0 8px}.muted{color:#9fb4c1;font-size:13px}.buttons{display:grid;grid-template-columns:1fr 1fr;gap:7px}button{background:#1c3342;color:#fff;border:1px solid #3d6277;border-radius:8px;padding:10px;cursor:pointer;font-weight:700}button:hover{background:#294b5e}button.primary{background:#167a95;border-color:#39bedb}label{display:block;margin:9px 0}.speed{display:grid;grid-template-columns:1fr 46px;gap:8px;align-items:center}input[type=range]{width:100%}.status{margin:10px 0;background:#0b1720;border:1px solid #2b4b5d;border-radius:8px;padding:9px;color:#ffdf70;font-weight:700}.cutstep{padding:7px 9px;border-left:3px solid #40525d;margin:5px 0;background:#172630;font-size:13px}.cutstep.active{border-color:#ffd65b;background:#3a321a}.cutstep.done{border-color:#53da8a;color:#9ee6b9}.piece,.dim,.factory{position:absolute;left:0;top:0;pointer-events:none;white-space:nowrap;box-shadow:0 2px 10px #0009}.piece{transform-origin:center;background:#071019dc;border:1px solid #55717f;border-radius:6px;padding:4px 7px;text-align:center;font-size:12px}.dim{color:#211900;background:#ffdc69;border-radius:5px;padding:2px 6px;font-weight:750;font-size:12px}.factory{background:#f07822;color:#fff;border-radius:5px;padding:3px 7px;font-size:11px;font-weight:800}.note{background:#172c37;border:1px solid #345365;border-radius:8px;padding:10px;margin-top:8px}.warn{background:#3a2b13;border:1px solid #8d6821;color:#ffe1a0;border-radius:8px;padding:10px;margin-top:9px}.ok{color:#8ee2aa}table{border-collapse:collapse;width:100%;font-size:13px}td{border-bottom:1px solid #263d4a;padding:5px 2px;vertical-align:top}td:last-child{text-align:right;color:#ffdf70;font-weight:700}@media(max-width:920px){#app{grid-template-columns:1fr;grid-template-rows:58vh 42vh}aside{border-left:0;border-top:1px solid #2a4557;padding:14px}}
</style></head><body><div id="app"><main id="view"></main><aside>
<h1>Раскрой ПНД-листа</h1><div class="muted">Лист 1000×1000×4 мм · две наклонные поверхности V13</div>
<h2>Плавная демонстрация</h2><div class="buttons"><button class="primary" id="play">▶ Запустить анимацию</button><button id="restart">↻ С начала</button></div><div class="speed"><input id="speed" type="range" min="0.4" max="2" step="0.1" value="1"><b id="speedOut">1.0×</b></div><div class="status" id="status"></div>
<div class="cutstep">1. По линии 810 мм на всю длину листа</div><div class="cutstep">2. По линии 405 мм на всю длину листа</div><div class="cutstep">3. Полосу 405 укоротить до __LOWER2__ мм</div><div class="cutstep">4. Вторую полосу 405 укоротить до __UPPER2__ мм</div>
<h2>Вид</h2><div class="buttons"><button data-view="top">Сверху</button><button data-view="iso">Изометрия</button><button data-view="edge">С торца</button><button id="explode">Разнести детали</button></div><label><input id="dimensions" type="checkbox" checked> Все размеры</label><label><input id="waste" type="checkbox" checked> Показывать остатки</label>
<h2>Готовые детали</h2><table><tr><td>Нижний скат, 15°</td><td>__LOWER2__×405×4</td></tr><tr><td>Верхний скат, 10°</td><td>__UPPER2__×405×4</td></tr><tr><td>Большой остаток</td><td>1000×190×4</td></tr><tr><td>Лист исходный</td><td>1000×1000×4</td></tr></table>
<h2>Как резать ножом</h2><div class="note"><b>Новый сегмент 18 мм, выдвинуть только на 1–2 деления.</b><br>Лист положить на жертвенную фанеру/ДСП. Стальную линейку крепко прижать струбцинами. Сделать по одной линии 15–30 лёгких проходов без попытки прорезать сразу. Резать насквозь; <b>не переламывать</b> ПНД.</div><div class="warn">Сначала измерь реальную толщину штангенциркулем. Эти детали можно вырезать сейчас, но пазы в картоне под 4 мм я обновлю только по фактическому замеру листа.</div><p class="muted"><span class="ok">Точность для ручной разметки:</span> 559,0 и 516,9 мм допустимо. После реза снять заусенец ножом/наждачкой, не скругляя опорную кромку.</p>
</aside></div><script type="module">__THREE__
__APP__</script></body></html>'''
    html = (html.replace('__THREE__', three).replace('__APP__', app)
            .replace('__LOWER2__', f'{LOWER:.2f}').replace('__UPPER2__', f'{UPPER:.2f}'))
    OUT.write_text(html, encoding="utf-8")


if __name__ == "__main__":
    build()
    print(OUT)
