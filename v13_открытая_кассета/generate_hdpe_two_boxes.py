from __future__ import annotations

import math
from pathlib import Path


ROOT = Path(__file__).resolve().parent
THREE = (ROOT.parent / "libs" / "three.module.js").read_text(encoding="utf-8")
OUT = ROOT / "РАСКРОЙ_ПНД_НА_2_КОРОБКИ_3D.html"

A = 540.0 / math.cos(math.radians(15.0))
B = 509.0 / math.cos(math.radians(10.0))
W = 405.0
S = A + W


def build() -> None:
    js = r'''
const D={sheet:1000,w:405,a:__A__,b:__B__,pack:__S__,edge:1000-__S__,thickness:4};
const scene=new Scene(); scene.background=new Color(0x071019);
const camera=new PerspectiveCamera(36,1,1,5000); camera.up.set(0,0,1);
const renderer=new WebGLRenderer({antialias:true}); renderer.setPixelRatio(Math.min(devicePixelRatio,2)); renderer.outputColorSpace=SRGBColorSpace; renderer.shadowMap.enabled=true;
document.getElementById('view').appendChild(renderer.domElement);
scene.add(new HemisphereLight(0xe1f2ff,0x15232d,2.8)); const sun=new DirectionalLight(0xffffff,3.4);sun.position.set(-500,-600,1200);sun.castShadow=true;scene.add(sun);
const floor=new Mesh(new PlaneGeometry(1550,1500),new MeshStandardMaterial({color:0x101e28,roughness:.96}));floor.position.set(500,500,-8);floor.receiveShadow=true;scene.add(floor);
const grid=new GridHelper(1400,28,0x294553,0x192f3b);grid.rotation.x=Math.PI/2;grid.position.set(500,500,-7.5);scene.add(grid);
function block(w,h,t,color,opacity=1){const m=new Mesh(new BoxGeometry(w,h,t),new MeshStandardMaterial({color,roughness:.8,metalness:.01,transparent:opacity<1,opacity,side:DoubleSide}));m.castShadow=true;m.receiveShadow=true;return m}
const base=block(1000,1000,4,0x161b1f);base.position.set(500,500,0);scene.add(base);
const labels=[],pieces=new Group();scene.add(pieces);
function piece(id,x,y,w,h,color,text){const m=block(w-2,h-2,.9,color,.84);m.position.set(x+w/2,y+h/2,2.5);m.userData={id,home:m.position.clone()};pieces.add(m);const el=document.createElement('div');el.className='piece';el.innerHTML=text;document.getElementById('view').appendChild(el);const o={m,el,p:m.position.clone().add(new Vector3(0,0,3))};labels.push(o);return o}
const parts=[
 piece('u2',0,0,D.w,D.b,0x4fd58c,'<b>КОРОБКА 2 · ВЕРХНИЙ</b><br>516,85 × 405 × 4'),
 piece('l2',D.w,0,D.a,D.w,0x19b9dd,'<b>КОРОБКА 2 · НИЖНИЙ</b><br>559,05 × 405 × 4'),
 piece('l1',0,D.a,D.a,D.w,0x2f9fe2,'<b>КОРОБКА 1 · НИЖНИЙ</b><br>559,05 × 405 × 4'),
 piece('u1',D.a,D.pack-D.b,D.w,D.b,0x71d67f,'<b>КОРОБКА 1 · ВЕРХНИЙ</b><br>516,85 × 405 × 4')
];
const waste=[];
function wasteBox(x,y,w,h){const m=block(Math.max(1,w-1),Math.max(1,h-1),.7,0x414d55,.62);m.position.set(x+w/2,y+h/2,2.35);scene.add(m);waste.push(m);return m}
wasteBox(D.pack,0,D.edge,1000); wasteBox(0,D.pack,D.pack,D.edge);
wasteBox(0,D.b,D.w,D.a-D.b); wasteBox(D.w,D.w,D.a-D.w,D.a-D.w); wasteBox(D.a,D.w,D.w,D.pack-D.b-D.w);

const cuts=[
 {a:[D.pack,0],b:[D.pack,1000],t:`1. Отрезать правую полосу ${D.edge.toFixed(2)} мм`},
 {a:[0,D.pack],b:[D.pack,D.pack],t:`2. Отрезать верхнюю полосу ${D.edge.toFixed(2)} мм`},
 {a:[D.w,0],b:[D.w,D.b],t:'3. Вертикальный рез до 516,85 мм'},
 {a:[0,D.b],b:[D.w,D.b],t:'4. Освободить верхний скат коробки 2'},
 {a:[D.w,D.w],b:[D.pack,D.w],t:'5. Освободить нижний скат коробки 2'},
 {a:[D.a,D.pack-D.b],b:[D.pack,D.pack-D.b],t:'6. Нижняя граница верхнего ската коробки 1'},
 {a:[D.a,D.pack-D.b],b:[D.a,D.pack],t:'7. Левая граница верхнего ската коробки 1'},
 {a:[0,D.a],b:[D.a,D.a],t:'8. Освободить нижний скат коробки 1'}
];
const visuals=[];
for(const c of cuts){const dx=c.b[0]-c.a[0],dy=c.b[1]-c.a[1],len=Math.hypot(dx,dy),vertical=Math.abs(dy)>Math.abs(dx);const guide=block(vertical?3:len,vertical?len:3,1.2,0xf29a26,.62);guide.position.set((c.a[0]+c.b[0])/2,(c.a[1]+c.b[1])/2,3.25);scene.add(guide);const prog=block(1,1,2.2,0xffe06b);prog.visible=false;prog.position.z=4.3;scene.add(prog);visuals.push({c,dx,dy,len,vertical,guide,prog})}

const knife=new Group();const handle=block(28,90,18,0xed7520);handle.position.set(0,-30,16);knife.add(handle);const grip=block(22,56,19,0x26343d);grip.position.set(0,-29,16.2);knife.add(grip);const sh=new Shape();sh.moveTo(-8,0);sh.lineTo(8,0);sh.lineTo(3,50);sh.lineTo(-3,50);sh.closePath();const blade=new Mesh(new ExtrudeGeometry(sh,{depth:2,bevelEnabled:false}),new MeshStandardMaterial({color:0xe5ecee,metalness:.8,roughness:.2}));blade.rotation.x=Math.PI/2;blade.position.set(0,7,5);knife.add(blade);knife.scale.setScalar(.72);knife.visible=false;scene.add(knife);

const dimensions=new Group();scene.add(dimensions);const dlabels=[];
function line(a,b){dimensions.add(new Line(new BufferGeometry().setFromPoints([new Vector3(...a),new Vector3(...b)]),new LineBasicMaterial({color:0xffdc69})))}
function dim(a,b,text,p,axis){line(a,b);for(const q of[a,b]){const aa=[...q],bb=[...q];if(axis==='x'){aa[0]-=11;bb[0]+=11}else{aa[1]-=11;bb[1]+=11}line(aa,bb)}const el=document.createElement('div');el.className='dim';el.textContent=text;document.getElementById('view').appendChild(el);dlabels.push({el,p:new Vector3(...p)})}
const z=8;dim([0,-58,z],[1000,-58,z],'1000 мм',[500,-58,z],'y');dim([-58,0,z],[-58,1000,z],'1000 мм',[-58,500,z],'x');dim([0,-25,z],[D.pack,-25,z],`${D.pack.toFixed(2)} мм`,[D.pack/2,-25,z],'y');dim([D.pack,-25,z],[1000,-25,z],`${D.edge.toFixed(2)} мм`,[(D.pack+1000)/2,-25,z],'y');dim([0,D.pack+24,z],[D.a,D.pack+24,z],`${D.a.toFixed(2)} мм`,[D.a/2,D.pack+24,z],'y');dim([D.a,D.pack+24,z],[D.pack,D.pack+24,z],'405 мм',[(D.a+D.pack)/2,D.pack+24,z],'y');dim([D.pack+24,0,z],[D.pack+24,D.b,z],`${D.b.toFixed(2)} мм`,[D.pack+24,D.b/2,z],'x');

let yaw=-Math.PI/2,pitch=1.555,dist=1490,target=new Vector3(500,475,0),drag=false,lastX=0,lastY=0;
function updateCamera(){const cp=Math.cos(pitch);camera.position.set(target.x+dist*cp*Math.cos(yaw),target.y+dist*cp*Math.sin(yaw),target.z+dist*Math.sin(pitch));camera.lookAt(target)}
renderer.domElement.addEventListener('pointerdown',e=>{drag=true;lastX=e.clientX;lastY=e.clientY;renderer.domElement.setPointerCapture(e.pointerId)});renderer.domElement.addEventListener('pointerup',()=>drag=false);renderer.domElement.addEventListener('pointercancel',()=>drag=false);renderer.domElement.addEventListener('pointermove',e=>{if(!drag)return;yaw-=(e.clientX-lastX)*.006;pitch=Math.max(.08,Math.min(1.56,pitch+(e.clientY-lastY)*.005));lastX=e.clientX;lastY=e.clientY;updateCamera()});renderer.domElement.addEventListener('wheel',e=>{e.preventDefault();dist=Math.max(650,Math.min(2600,dist*Math.exp(e.deltaY*.001)));updateCamera()},{passive:false});
function preset(v){if(v==='top'){yaw=-Math.PI/2;pitch=1.555;dist=1490}else if(v==='iso'){yaw=-2.2;pitch=.68;dist=1770}else{yaw=-Math.PI/2;pitch=.14;dist=1770}updateCamera()}document.querySelectorAll('[data-view]').forEach(b=>b.onclick=()=>preset(b.dataset.view));
let playing=false,elapsed=0,lastTime=performance.now(),speed=1;const CT=2.0,PAUSE=.42,TOTAL=cuts.length*(CT+PAUSE);
function full(v){v.prog.visible=true;v.prog.scale.set(v.vertical?5:v.len,v.vertical?v.len:5,1);v.prog.position.set((v.c.a[0]+v.c.b[0])/2,(v.c.a[1]+v.c.b[1])/2,4.3)}
function status(i,p){document.querySelectorAll('.step').forEach((e,n)=>{e.classList.toggle('active',n===i);e.classList.toggle('done',n<i||i===cuts.length)});document.getElementById('status').textContent=i<0?'Готово к показу раскроя':i>=cuts.length?'Готово: четыре ската для двух коробок':`Рез ${i+1}/8 · ${Math.round(p*100)}%`}
function updateCut(t){for(const v of visuals)v.prog.visible=false;if(t>=TOTAL){for(const v of visuals)full(v);knife.visible=false;status(cuts.length,1);return}const slot=CT+PAUSE,i=Math.floor(t/slot),local=t-i*slot,p=Math.min(1,local/CT);for(let n=0;n<i;n++)full(visuals[n]);const v=visuals[i],c=v.c;v.prog.visible=p>0;v.prog.scale.set(v.vertical?5:Math.max(1,v.len*p),v.vertical?Math.max(1,v.len*p):5,1);v.prog.position.set(c.a[0]+v.dx*p/2,c.a[1]+v.dy*p/2,4.3);knife.visible=local<=CT;knife.position.set(c.a[0]+v.dx*p,c.a[1]+v.dy*p,34);knife.rotation.z=Math.atan2(v.dy,v.dx)-Math.PI/2;status(i,p)}
function reset(){playing=false;elapsed=0;knife.visible=false;for(const v of visuals)v.prog.visible=false;status(-1,0);document.getElementById('play').textContent='▶ Запустить'}
document.getElementById('play').onclick=()=>{if(elapsed>=TOTAL)elapsed=0;playing=!playing;document.getElementById('play').textContent=playing?'⏸ Пауза':'▶ Продолжить';if(!playing)knife.visible=false};document.getElementById('restart').onclick=()=>{reset();playing=true;document.getElementById('play').textContent='⏸ Пауза'};document.getElementById('speed').oninput=e=>{speed=+e.target.value;document.getElementById('speedOut').textContent=speed.toFixed(1)+'×'};
document.getElementById('dimensions').onchange=e=>{dimensions.visible=e.target.checked;for(const d of dlabels)d.el.style.display=e.target.checked?'block':'none'};document.getElementById('waste').onchange=e=>{for(const m of waste)m.visible=e.target.checked};let exploded=false;document.getElementById('explode').onclick=()=>{exploded=!exploded;const shift={u2:[-20,-20,20],l2:[25,-20,20],l1:[-25,25,20],u1:[25,25,20]};for(const p of parts){const s=shift[p.m.userData.id];p.m.position.copy(p.m.userData.home).add(exploded?new Vector3(...s):new Vector3());p.p.copy(p.m.position).add(new Vector3(0,0,3))}document.getElementById('explode').textContent=exploded?'Собрать раскрой':'Разнести 4 детали'};
function project(el,p){const v=p.clone().project(camera),w=renderer.domElement.clientWidth,h=renderer.domElement.clientHeight;el.style.transform=`translate(-50%,-50%) translate(${(v.x*.5+.5)*w}px,${(-v.y*.5+.5)*h}px)`;el.style.opacity=(v.z>1||v.z<-1)?0:1}
function resize(){const e=document.getElementById('view');renderer.setSize(e.clientWidth,e.clientHeight,false);camera.aspect=e.clientWidth/e.clientHeight;camera.updateProjectionMatrix()}addEventListener('resize',resize);resize();updateCamera();reset();
function animate(now){requestAnimationFrame(animate);const dt=Math.min(.05,(now-lastTime)/1000);lastTime=now;if(playing){elapsed+=dt*speed;if(elapsed>=TOTAL){elapsed=TOTAL;playing=false;document.getElementById('play').textContent='▶ Повторить'}updateCut(elapsed)}for(const x of labels)project(x.el,x.p);for(const x of dlabels)project(x.el,x.p);renderer.render(scene,camera)}requestAnimationFrame(animate);
'''.replace('__A__', f'{A:.6f}').replace('__B__', f'{B:.6f}').replace('__S__', f'{S:.6f}')

    steps = [
        f"1. Отрезать справа {1000-S:.2f} мм на всю высоту",
        f"2. Отрезать сверху {1000-S:.2f} мм до X={S:.2f}",
        f"3. Вертикально X=405 до Y={B:.2f}",
        f"4. Горизонтально Y={B:.2f} от X=0 до 405",
        "5. Горизонтально Y=405 от X=405 до 964,05",
        f"6. Горизонтально Y={S-B:.2f} от X={A:.2f} до 964,05",
        f"7. Вертикально X={A:.2f} от Y={S-B:.2f} до 964,05",
        f"8. Горизонтально Y={A:.2f} от X=0 до {A:.2f}",
    ]
    step_html = "".join(f'<div class="step">{s}</div>' for s in steps)
    html = r'''<!doctype html><html lang="ru"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>ПНД 1000×1000 — две коробки</title><style>
*{box-sizing:border-box}body{margin:0;background:#071019;color:#eef7fa;font:15px/1.42 "Segoe UI",Arial,sans-serif;overflow:hidden}#app{display:grid;grid-template-columns:minmax(0,1fr) 430px;height:100vh}#view{position:relative;min-width:0;overflow:hidden}canvas{display:block;width:100%;height:100%}aside{background:#111e29;border-left:1px solid #294657;padding:18px;overflow:auto}h1{font-size:23px;line-height:1.15;margin:0 0 6px}h2{font-size:15px;color:#8eddf8;margin:19px 0 8px}.muted{color:#9eb3c0;font-size:13px}.buttons{display:grid;grid-template-columns:1fr 1fr;gap:7px}button{background:#1c3342;color:#fff;border:1px solid #3d6277;border-radius:8px;padding:10px;cursor:pointer;font-weight:700}button:hover{background:#294b5e}button.primary{background:#177b96;border-color:#3bc1df}.speed{display:grid;grid-template-columns:1fr 46px;gap:8px;align-items:center;margin-top:8px}.status{margin:10px 0;background:#0b1720;border:1px solid #2b4b5d;border-radius:8px;padding:9px;color:#ffdf70;font-weight:700}.step{padding:6px 8px;border-left:3px solid #42545f;margin:4px 0;background:#172630;font-size:12.5px}.step.active{border-color:#ffd65b;background:#3a321a}.step.done{border-color:#53da8a;color:#9ee6b9}.piece,.dim{position:absolute;left:0;top:0;pointer-events:none;white-space:nowrap;box-shadow:0 2px 10px #0009}.piece{background:#071019df;border:1px solid #587684;border-radius:6px;padding:4px 7px;text-align:center;font-size:11.5px}.dim{color:#211900;background:#ffdc69;border-radius:5px;padding:2px 6px;font-weight:750;font-size:12px}label{display:block;margin:8px 0}.note{background:#172c37;border:1px solid #345365;border-radius:8px;padding:10px}.warn{background:#3b2b13;border:1px solid #8e6822;color:#ffe1a0;border-radius:8px;padding:10px;margin-top:9px}table{border-collapse:collapse;width:100%;font-size:13px}td{border-bottom:1px solid #263d4a;padding:5px 2px}td:last-child{text-align:right;color:#ffdf70;font-weight:700}@media(max-width:920px){#app{grid-template-columns:1fr;grid-template-rows:58vh 42vh}aside{border-left:0;border-top:1px solid #294657;padding:14px}}
</style></head><body><div id="app"><main id="view"></main><aside><h1>Один лист → две коробки</h1><div class="muted">ПНД 1000×1000×4 мм · раскладка «вертушка» · использование 87,15%</div><h2>Плавная анимация 8 резов</h2><div class="buttons"><button class="primary" id="play">▶ Запустить</button><button id="restart">↻ С начала</button></div><div class="speed"><input id="speed" type="range" min="0.4" max="2" step="0.1" value="1"><b id="speedOut">1.0×</b></div><div id="status" class="status"></div>__STEPS__<h2>Вид</h2><div class="buttons"><button data-view="top">Сверху</button><button data-view="iso">Изометрия</button><button data-view="edge">С торца</button><button id="explode">Разнести 4 детали</button></div><label><input id="dimensions" type="checkbox" checked> Размеры</label><label><input id="waste" type="checkbox" checked> Остатки</label><h2>Результат</h2><table><tr><td>Нижние скаты</td><td>2 × 559,05×405</td></tr><tr><td>Верхние скаты</td><td>2 × 516,85×405</td></tr><tr><td>Занято материала</td><td>0,8715 м²</td></tr><tr><td>Остаток</td><td>0,1285 м²</td></tr></table><h2>Важно при резке</h2><div class="note">Сначала разметь весь лист угольником и тонким маркером по малярной ленте. Наружные резы №1–2 делай первыми. На внутренних резах №3–8 нож должен <b>остановиться точно в указанной точке</b>: не прорезай цветную соседнюю деталь.</div><div class="warn">Раскладка рассчитана почти без потери размера. Ширина пропила ножа мала. Для лобзика заложи пропил и режь со стороны отхода, иначе готовая деталь получится короче.</div></aside></div><script type="module">__THREE__
__APP__</script></body></html>'''
    html = html.replace('__STEPS__', step_html).replace('__THREE__', THREE).replace('__APP__', js)
    OUT.write_text(html, encoding="utf-8")


if __name__ == "__main__":
    build()
    print(OUT)
