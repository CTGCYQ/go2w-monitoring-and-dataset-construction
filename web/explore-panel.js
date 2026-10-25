/* SLAM 自主探索 — 阶段 0 前端：围栏编辑器 + frontier/目标/位姿叠加 + 安全状态灯。
   深空科技风，复用 index.html 的 --border/--muted/--accent/--green/--red/--amber 变量。
   阶段 0 恒 dry-run：安全灯恒红色「未放行」，绝不提供「开始探索」真机放行按钮。 */
(()=>{'use strict';
 const EPS=1e-9;

 // ---------- 本地几何校验（与后端 explore_fence.py 对齐，双保险）----------
 function segCross(o,a,b){return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0]);}
 function segsIntersect(a,b,c,d){
   function side(p,q,r){const v=segCross(p,q,r);return v>EPS?1:(v<-EPS?-1:0);}
   const s1=side(a,b,c),s2=side(a,b,d),s3=side(c,d,a),s4=side(c,d,b);
   return (s1!==s2&&s3!==s4);
 }
 function polygonArea(pts){let s=0;for(let i=0;i<pts.length;i++){const a=pts[i],b=pts[(i+1)%pts.length];s+=a[0]*b[1]-b[0]*a[1];}return Math.abs(s)/2;}
 function selfIntersect(pts){const n=pts.length;if(n<3)return false;for(let i=0;i<n;i++){const a=pts[i],b=pts[(i+1)%n];for(let j=i+1;j<n;j++){const c=pts[j],d=pts[(j+1)%n];if(j===i||(j+1)%n===i||j===(i+1)%n)continue;if(segsIntersect(a,b,c,d))return true;}}return false;}
 function rectCorners(f){const [a,b]=f.points;const x1=Math.min(a[0],b[0]),x2=Math.max(a[0],b[0]),y1=Math.min(a[1],b[1]),y2=Math.max(a[1],b[1]);return [[x1,y1],[x2,y1],[x2,y2],[x1,y2]];}
 function validateFenceLocal(f){
   if(f.kind==='circle'){if(!(f.radius>=0.3))return '圆形围栏半径须 ≥ 0.3m';return null;}
   const pts=(f.kind==='rectangle')?rectCorners(f):f.points;
   if(!pts||pts.length<3)return '多边形/矩形需要足够顶点';
   if(f.kind==='polygon'&&pts.length<3)return '多边形顶点数须 ≥ 3';
   if(f.kind==='rectangle'&&f.points.length!==2)return '矩形需 2 个对角点';
   if(polygonArea(pts)<1)return '围栏面积须 ≥ 1m²';
   if(f.kind==='polygon'&&selfIntersect(pts))return '围栏边线存在自交';
   if(f.kind==='rectangle'){const [a,b]=f.points;if(Math.abs(b[0]-a[0])<0.5||Math.abs(b[1]-a[1])<0.5)return '矩形宽高须 ≥ 0.5m';}
   return null;
 }

 // ---------- 状态 ----------
 let editingFence=null; let fences=[];
 let drawingRect=false; let dragMode=''; let dragVertex=-1;
 let exploreState=null; let frontiers=[]; let targetPoint=null;

 function el(id){return document.getElementById(id);}
 function gridData(){return window.__slamLastGrid||[];}

 // world → screen 变换（与 drawSlamGrid 同参考系）
 function worldToScreen(w,h){
   const grid=gridData();
   let minX=Infinity,maxX=-Infinity,minY=Infinity,maxY=-Infinity;
   for(const p of grid){if(p[0]<minX)minX=p[0];if(p[0]>maxX)maxX=p[0];if(p[1]<minY)minY=p[1];if(p[1]>maxY)maxY=p[1];}
   if(!isFinite(minX)){minX=-2;maxX=2;minY=-2;maxY=2;}
   const spanX=Math.max(maxX-minX,1),spanY=Math.max(maxY-minY,1);
   const scale=Math.min(w/spanX,h/spanY)*0.92;
   const ox=w/2-(minX+maxX)/2*scale, oy=h/2-(minY+maxY)/2*scale;
   return {scale,ox,oy,sx:x=>ox+x*scale,sy:y=>oy+y*scale};
 }

 function drawOneFence(ctx,tr,f,active){
   let pts;
   if(f.kind==='circle'){
     const cx=tr.sx(f.center[0]),cy=tr.sy(f.center[1]),r=Math.abs(f.radius*tr.scale);
     ctx.beginPath();ctx.arc(cx,cy,r,0,Math.PI*2);ctx.fillStyle='rgba(112,214,187,0.12)';ctx.fill();
     ctx.strokeStyle=active?'#70d6bb':'rgba(112,214,187,0.6)';ctx.lineWidth=active?2:1.2;ctx.stroke();
     if(active){ctx.beginPath();ctx.arc(cx,cy,4,0,Math.PI*2);ctx.fillStyle='#fff';ctx.fill();ctx.beginPath();ctx.arc(cx+r,cy,4,0,Math.PI*2);ctx.fillStyle='#f0b876';ctx.fill();}
     return;
   }
   pts=(f.kind==='rectangle')?rectCorners(f):f.points;
   if(!pts||pts.length<2)return;
   const invalid=validateFenceLocal(f);
   ctx.beginPath();ctx.moveTo(tr.sx(pts[0][0]),tr.sy(pts[0][1]));
   for(let i=1;i<pts.length;i++)ctx.lineTo(tr.sx(pts[i][0]),tr.sy(pts[i][1]));
   ctx.closePath();ctx.fillStyle='rgba(112,214,187,0.12)';ctx.fill();
   ctx.strokeStyle=invalid?'#ef4444':(active?'#70d6bb':'rgba(112,214,187,0.6)');ctx.lineWidth=active?2:1.2;ctx.stroke();
   pts.forEach((p)=>{const px=tr.sx(p[0]),py=tr.sy(p[1]);ctx.beginPath();ctx.arc(px,py,active?5:3,0,Math.PI*2);ctx.fillStyle=invalid?'#ef4444':'#0e1f36';ctx.fill();ctx.strokeStyle=active?'#fff':'#70d6bb';ctx.lineWidth=1;ctx.stroke();});
 }

 function drawOverlay(){
   const cv=el('slamCanvas'); if(!cv)return;
   const ctx=cv.getContext('2d'); const W=cv.clientWidth||800,H=cv.clientHeight||520;
   const pose=window.__slamLastPose||null;
   const tr=worldToScreen(W,H);
   fences.forEach(f=>drawOneFence(ctx,tr,f,false));
   if(editingFence)drawOneFence(ctx,tr,editingFence,true);
   if(frontiers&&frontiers.length){
     const blink=(Math.sin(Date.now()/300)>0);
     ctx.strokeStyle=blink?'#f0b876':'#f8d7b0';ctx.lineWidth=2;
     frontiers.forEach(fr=>{const px=tr.sx(fr.x),py=tr.sy(fr.y);ctx.beginPath();ctx.arc(px,py,5,0,Math.PI*2);ctx.stroke();});
   }
   if(targetPoint){
     const px=tr.sx(targetPoint.x),py=tr.sy(targetPoint.y);
     const blink=Math.sin(Date.now()/250)>0;ctx.beginPath();ctx.arc(px,py,blink?8:5,0,Math.PI*2);ctx.fillStyle='#f0b876';ctx.fill();ctx.strokeStyle='#fff';ctx.lineWidth=1.5;ctx.stroke();
   }
   if(pose){
     const px=tr.sx(pose.x),py=tr.sy(pose.y),yaw=Number(pose.yaw||0);
     ctx.strokeStyle='#34d399';ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(px,py);ctx.lineTo(px+Math.cos(yaw)*18,py+Math.sin(yaw)*18);ctx.stroke();
   }
 }

 function hitVertex(wx,wy){
   if(!editingFence)return -1;
   const pts=(editingFence.kind==='circle')?null:((editingFence.kind==='rectangle')?rectCorners(editingFence):editingFence.points);
   if(!pts)return -1;
   for(let i=0;i<pts.length;i++){if(Math.hypot(pts[i][0]-wx,pts[i][1]-wy)<0.25)return i;}
   return -1;
 }

 function bindEditor(){
   const cv=el('slamCanvas'); if(!cv)return;
   cv.addEventListener('mousedown',e=>{
     if(!editingFence)return;
     const rect=cv.getBoundingClientRect();
     const ox=e.clientX-rect.left, oy=e.clientY-rect.top;
     const tr=worldToScreen(cv.clientWidth,cv.clientHeight);
     const wx=(ox-tr.ox)/tr.scale, wy=(oy-tr.oy)/tr.scale;
     if(editingFence.kind==='circle'){
       const cx=tr.sx(editingFence.center[0]),cy=tr.sy(editingFence.center[1]),r=Math.abs(editingFence.radius*tr.scale);
       if(Math.hypot(ox-(cx+r),oy-cy)<8){dragMode='radius';return;}
       if(Math.hypot(ox-cx,oy-cy)<8){dragMode='center';return;}
       return;
     }
     const v=hitVertex(wx,wy);
     if(v>=0){dragVertex=v;dragMode='vertex';return;}
     const tool=window.__fenceTool||'polygon';
     if(tool==='rect'){drawingRect=true;editingFence={kind:'rectangle',points:[[wx,wy],[wx,wy]]};dragMode='rect';}
     else if(tool==='polygon'){editingFence.points.push([wx,wy]);}
   });
   cv.addEventListener('mousemove',e=>{
     if(!editingFence||!dragMode)return;
     const rect=cv.getBoundingClientRect();
     const ox=e.clientX-rect.left, oy=e.clientY-rect.top;
     const tr=worldToScreen(cv.clientWidth,cv.clientHeight);
     const wx=(ox-tr.ox)/tr.scale, wy=(oy-tr.oy)/tr.scale;
     if(dragMode==='center'&&editingFence.kind==='circle'){editingFence.center=[wx,wy];}
     else if(dragMode==='radius'&&editingFence.kind==='circle'){editingFence.radius=Math.max(0.05,Math.hypot(wx-editingFence.center[0],wy-editingFence.center[1]));}
     else if(dragMode==='rect'&&editingFence.kind==='rectangle'){editingFence.points[1]=[wx,wy];}
     else if(dragMode==='vertex'){editingFence.points[dragVertex]=[wx,wy];}
   });
   cv.addEventListener('mouseup',()=>{dragMode='';dragVertex=-1;drawingRect=false;});
   cv.addEventListener('dblclick',()=>{ if(editingFence&&editingFence.kind==='polygon'&&editingFence.points&&editingFence.points.length>=3) editingFence=null; });
 }

 function toWire(f){return f.kind==='circle'?{kind:'circle',center:[f.center[0],f.center[1]],radius:f.radius,frame:'world_map'}:{kind:f.kind,points:f.points.map(p=>[p[0],p[1]]),frame:'world_map'};}
 function toLocal(w){return w.kind==='circle'?{kind:'circle',center:[w.center[0],w.center[1]],radius:w.radius}:{kind:w.kind,points:w.points.map(p=>[p[0],p[1]])};}

 function msg(text,cls){const m=el('exploreMsg');if(!m)return;m.textContent=text;m.className='msg '+(cls||'');}

 async function saveFence(){
   if(!editingFence){msg('请先绘制一个围栏','amber');return;}
   const err=validateFenceLocal(editingFence);
   if(err){msg('校验失败：'+err,'amber');return;}
   const body=[...fences.map(toWire),toWire(editingFence)];
   try{
     const r=await fetch('/api/explore/fence',{method:'PUT',headers:{'Content-Type':'application/json'},body:JSON.stringify({fences:body})});
     const d=await r.json().catch(()=>({}));
     if(!r.ok){msg('保存失败：'+(d.detail||r.status),'amber');return;}
     fences=(d.fences||[]).map(toLocal);msg('围栏已保存 ('+fences.length+' 个)','green');editingFence=null;
   }catch(e){msg('保存失败：'+e.message,'amber');}
 }
 function setTool(mode){
   window.__fenceTool=mode;editingFence=null;
   if(mode==='circle')editingFence={kind:'circle',center:[0,0],radius:0.5};
   else if(mode==='rect')editingFence=null;
   else if(mode==='polygon')editingFence={kind:'polygon',points:[]};
   document.querySelectorAll('#slam-tools button').forEach(b=>b.classList.toggle('is-active',b.dataset.tool===mode));
 }
 function clearFence(){editingFence=null;}

 async function loadFences(){try{const r=await fetch('/api/explore/fence');const d=await r.json();fences=(d.fences||[]).map(toLocal);}catch(_){}}
 async function pollExplore(){
   try{
     const r=await fetch('/api/explore/state');exploreState=await r.json();
     const fr=await fetch('/api/explore/frontiers');const fd=await fr.json();frontiers=fd.frontiers||[];
     if(exploreState&&exploreState.target)targetPoint=exploreState.target;
     renderStatus();
   }catch(_){}
 }
 function renderStatus(){
   const light=el('exploreSafety'); if(light)light.innerHTML='<span class="badge red">● 未放行 · DRY-RUN</span>';
   const st=el('exploreStatus'); if(st&&exploreState)st.textContent='状态: '+(exploreState.status||'idle')+' （dry-run 规划）';
   const pr=el('exploreProgress'); if(pr&&exploreState){const p=exploreState.progress||{};pr.textContent='前沿剩 '+(p.remaining_frontier!=null?p.remaining_frontier:'-')+' / '+(p.total_frontier!=null?p.total_frontier:'-')+(targetPoint?' · 目标 ('+targetPoint.x+', '+targetPoint.y+')':'');}
 }

 function init(){
   const origDraw=window.drawSlamGrid;
   if(typeof origDraw==='function'){
     window.drawSlamGrid=function(grid,pose){window.__slamLastGrid=grid;window.__slamLastPose=pose;origDraw(grid,pose);drawOverlay();};
   }
   bindEditor();
   loadFences();pollExplore();
   setInterval(pollExplore,2000);setInterval(drawOverlay,350);
 }
 window.ExplorePanel={init,setTool,clearFence,saveFence};
 document.addEventListener('DOMContentLoaded',init);
})();
