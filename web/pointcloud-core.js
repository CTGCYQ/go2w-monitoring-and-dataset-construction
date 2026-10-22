/* Native right-handed XYZ. No inferred body frame or synthetic RGB. */
(function(root,factory){const api=factory();if(typeof module==='object')module.exports=api;else root.PCVizCore=api;})(typeof globalThis!=='undefined'?globalThis:this,function(){
'use strict';
const clamp=(x,a,b)=>Math.min(b,Math.max(a,x));
const palette=[[.22,.40,.62],[.24,.65,.78],[.39,.83,.73],[.83,.88,.64],[.98,.67,.34]];
function color(t){t=clamp(t,0,1)*(palette.length-1);const i=Math.floor(t),f=t-i,a=palette[i],b=palette[Math.min(i+1,palette.length-1)];return a.map((v,j)=>v+(b[j]-v)*f);}
class CloudBuffer{
 constructor(){this.mode='frame';this.window=15;this.voxel=.035;this.range=8;this.limit=120000;this.reset();}
 reset(){this.cells=new Map();this.frames=[];this.keys=new Set();this.lastFrame=null;this.moving=false;this.frameId=null;}
 ingest(packet,now){
  const key=packet.frame_key||String(packet.ts),id=packet.frame_id||'unknown';
  if(!packet.online||!Array.isArray(packet.points))return false;
  if(this.frameId&&this.frameId!==id)this.reset();
  if(this.keys.has(key))return false;
  const moving=packet.motion?.moving===true;
  if(this.mode!=='aggregate'||moving||this.moving)this.reset();
  this.frameId=id;this.moving=moving;this.lastFrame=packet;
  this.keys.add(key);this.frames.push({key,time:now});
  const v=this.voxel,r2=this.range*this.range;
  for(const p of packet.points){
   if(!Array.isArray(p)||p.length<3||!p.slice(0,3).every(Number.isFinite))continue;
   const d=p[0]*p[0]+p[1]*p[1]+p[2]*p[2];if(d<.04||d>r2)continue;
   const k=Math.floor(p[0]/v)+','+Math.floor(p[1]/v)+','+Math.floor(p[2]/v),old=this.cells.get(k);
   const n=old?Math.min(old.n+1,12):1,w=1/n;
   const q=old?{x:old.x+(p[0]-old.x)*w,y:old.y+(p[1]-old.y)*w,z:old.z+(p[2]-old.z)*w,n,t:now,intensity:Number.isFinite(p[3])?p[3]:null}:{x:p[0],y:p[1],z:p[2],n,t:now,intensity:Number.isFinite(p[3])?p[3]:null};
   if(old)this.cells.delete(k);this.cells.set(k,q);
   if(this.cells.size>this.limit)this.cells.delete(this.cells.keys().next().value);
  }
  this.prune(now);return true;
 }
 prune(now){const cutoff=now-this.window;for(const[k,p]of this.cells)if(p.t<cutoff)this.cells.delete(k);while(this.frames.length&&this.frames[0].time<cutoff){this.keys.delete(this.frames.shift().key);}}
 bounds(){const p=Array.from(this.cells.values()),step=Math.max(1,Math.ceil(p.length/2500)),axes=[[],[],[]];for(let i=0;i<p.length;i+=step){axes[0].push(p[i].x);axes[1].push(p[i].y);axes[2].push(p[i].z);}if(!p.length)return{center:[0,0,0],span:8,min:[-4,-4,0],max:[4,4,3]};const ranges=axes.map(a=>{a.sort((x,y)=>x-y);return[a[Math.floor(a.length*.015)],a[Math.floor((a.length-1)*.985)]]});return{min:ranges.map(a=>a[0]),max:ranges.map(a=>a[1]),center:ranges.map(a=>(a[0]+a[1])/2),span:Math.max(2,...ranges.map(a=>a[1]-a[0]))};}
}
return{CloudBuffer,color,clamp};
});
