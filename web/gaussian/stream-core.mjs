/* Pure resource planning, independent of rendering and robot localization. */
export function distanceToBox(point,bounds){let d=0;for(let i=0;i<3;i++){const e=Math.max(bounds[0][i]-point[i],0,point[i]-bounds[1][i]);d+=e*e;}return Math.sqrt(d);}
export function estimatedBytes(level){return 65536+level.decoded_bytes+level.gaussians*96;}
export function planTiles(manifest,position,settings,visible=()=>true){
 const {loadRadius,renderRadius,gaussianBudget,cacheBytes,pixelError=3,focalPixels=450}=settings;
 let count=0,bytes=0;const result=[];
 const tiles=manifest.tiles.map(tile=>({tile,distance:distanceToBox(position,tile.bounds)})).filter(x=>x.distance<=loadRadius).sort((a,b)=>a.distance-b.distance);
 for(const {tile,distance} of tiles){
  const candidates=[...tile.levels].sort((a,b)=>b.level-a.level);
  let selected=candidates.find(l=>l.geometric_error_m*focalPixels/Math.max(.1,distance)<=pixelError)||tile.levels[0];
  const wantedVisible=distance<=renderRadius&&visible(tile);
  if(!wantedVisible)selected=candidates[0];
  if(count+selected.gaussians>gaussianBudget||bytes+estimatedBytes(selected)>cacheBytes)selected=candidates.find(l=>count+l.gaussians<=gaussianBudget&&bytes+estimatedBytes(l)<=cacheBytes);
  if(!selected)continue;
  count+=selected.gaussians;bytes+=estimatedBytes(selected);result.push({tile,level:selected,distance,visible:wantedVisible,key:`${tile.id}:${selected.level}`});
 }
 return {items:result,gaussians:count,estimatedBytes:bytes};
}
export class ResourceBudget{
 constructor(bytes){this.limit=bytes;this.resident=new Map();this.pending=new Map();}
 get bytes(){return [...this.resident.values(),...this.pending.values()].reduce((a,v)=>a+v.bytes,0);}
 reserve(key,bytes){if(this.resident.has(key)||this.pending.has(key))return false;if(bytes>this.limit)return false;this.evict(bytes);if(this.bytes+bytes>this.limit)return false;this.pending.set(key,{bytes});return true;}
 commit(key,value){const entry=this.pending.get(key);if(!entry)return false;this.pending.delete(key);this.resident.set(key,{...entry,value,touched:performance.now()});return true;}
 cancel(key){this.pending.delete(key);}
 touch(key){const entry=this.resident.get(key);if(entry)entry.touched=performance.now();return entry;}
 remove(key){const entry=this.resident.get(key);if(entry){entry.value?.dispose?.();this.resident.delete(key);}}
 evict(required=0,protectedKeys=new Set()){
  for(const [key] of [...this.resident.entries()].sort((a,b)=>a[1].touched-b[1].touched)){if(this.bytes+required<=this.limit)break;if(!protectedKeys.has(key))this.remove(key);}
 }
 clear(){for(const key of this.resident.keys())this.remove(key);this.pending.clear();}
}
