import * as THREE from 'three';
import {OrbitControls} from './vendor/OrbitControls.js';
import {SplatMesh,SplatFileType,SparkRenderer} from './vendor/spark.module.js';
import {planTiles,ResourceBudget,estimatedBytes,distanceToBox} from './stream-core.mjs';
const $=id=>document.getElementById(id),canvas=$('canvas');
const renderer=new THREE.WebGLRenderer({canvas,antialias:false,alpha:false,powerPreference:'high-performance'});renderer.setClearColor(0x152336);renderer.setPixelRatio(Math.min(devicePixelRatio,1.5));
const scene=new THREE.Scene(),world=new THREE.Group();world.rotation.x=Math.PI;scene.add(world);
const camera=new THREE.PerspectiveCamera(65,1,.05,100),controls=new OrbitControls(camera,canvas);controls.enableDamping=true;controls.dampingFactor=.12;controls.minDistance=.05;controls.maxDistance=60;
const spark=new SparkRenderer({renderer,enableLod:false,maxPagedSplats:262144,lodSplatCount:300000,preUpdate:true});scene.add(spark);
const marker=new THREE.Group(),arrow=new THREE.ArrowHelper(new THREE.Vector3(0,0,1),new THREE.Vector3(),.45,0xffb74d,.14,.08);marker.add(arrow);world.add(marker);marker.visible=false;
let manifest=null,active=false,epoch=0,sceneId=null,version=null,desired=[],inflight=new Map(),drawn=0,lastPlan=0,frames=0,lastFps=performance.now(),fps=0,downloads=0,downloadedBytes=0,frameMs=0,lastFrame=performance.now(),follow=false,latestPose=null,keyframe=0;
const budget=new ResourceBudget(64*1024**2),settings={loadRadius:12,renderRadius:10,cacheBytes:64*1024**2,gaussianBudget:300000,pixelError:3,focalPixels:450};
function fail(error){$('error').hidden=false;$('error').textContent=error?.message||String(error);}
function emit(type,value){parent.postMessage({type,...value},location.origin);}
function resize(){const w=innerWidth,h=innerHeight;renderer.setSize(w,h,false);camera.aspect=w/Math.max(1,h);camera.updateProjectionMatrix();}
addEventListener('resize',resize);resize();
function disposeEntry(entry){world.remove(entry.mesh);entry.mesh.dispose();}
function clear(){manifest=null;version=null;sceneId=null;epoch++;for(const value of inflight.values())value.controller.abort();inflight.clear();budget.clear();desired=[];drawn=0;marker.visible=false;latestPose=null;follow=false;}
function cameraPose(matrix){const optical=new THREE.Matrix4().set(...matrix.flat()),conversion=new THREE.Matrix4().makeRotationX(Math.PI),pose=conversion.clone().multiply(optical).multiply(conversion);pose.decompose(camera.position,camera.quaternion,new THREE.Vector3());const forward=new THREE.Vector3(0,0,-1).applyQuaternion(camera.quaternion);controls.target.copy(camera.position).addScaledVector(forward,2);controls.update();}
function home(){if(manifest?.keyframe_poses?.length){keyframe=0;cameraPose(manifest.keyframe_poses[0]);}else{camera.position.set(0,1,3);controls.target.set(0,0,-2);controls.update();}}
async function loadScene(id){clear();manifest=null;sceneId=id;version=null;$('error').hidden=true;$('empty').hidden=false;$('empty').style.display='none';if(!id){$('empty').style.display='grid';return;}
 const current=epoch;try{const response=await fetch(`/api/scenes/${encodeURIComponent(id)}/manifest`,{cache:'no-store'});if(!response.ok)throw new Error('场景未发布或无法读取');const data=await response.json();if(current!==epoch)return;manifest=data;version=data.version;settings.loadRadius=data.recommended?.load_radius_m||12;settings.renderRadius=data.recommended?.render_radius_m||10;home();active=true;lastPlan=0;emit('scene-loaded',{id,version,gaussians:data.gaussian_count});}catch(error){fail(error);}}
function visible(tile){const box=new THREE.Box3(new THREE.Vector3(...tile.bounds[0]),new THREE.Vector3(...tile.bounds[1]));box.applyMatrix4(world.matrixWorld);const matrix=new THREE.Matrix4().multiplyMatrices(camera.projectionMatrix,camera.matrixWorldInverse);return new THREE.Frustum().setFromProjectionMatrix(matrix).intersectsBox(box);}
function repaint(){drawn=0;for(const entry of budget.resident.values())entry.value.mesh.visible=false;
 for(const item of desired){if(!item.visible)continue;let entry=budget.touch(item.key);if(!entry){const fallback=[...budget.resident.values()].find(e=>e.value.tileId===item.tile.id);entry=fallback;}
  if(entry&&drawn+entry.value.count<=settings.gaussianBudget){entry.value.mesh.visible=true;drawn+=entry.value.count;}}
}
async function fetchTile(item){const current=epoch;const controller=new AbortController();const reservation=estimatedBytes(item.level);if(!budget.reserve(item.key,reservation))return;
 const task={controller,epoch:current};inflight.set(item.key,task);
 try{const url=`/api/scenes/${encodeURIComponent(sceneId)}/versions/${version}/assets/${item.level.file}`;const response=await fetch(url,{signal:controller.signal});if(!response.ok)throw new Error(`分块加载失败 (${response.status})`);const bytes=await response.arrayBuffer();if(bytes.byteLength!==item.level.decoded_bytes)throw new Error('分块长度校验失败');
  if(item.level.decoded_sha256&&crypto.subtle){const digest=await crypto.subtle.digest('SHA-256',bytes);const hex=Array.from(new Uint8Array(digest),v=>v.toString(16).padStart(2,'0')).join('');if(hex!==item.level.decoded_sha256)throw new Error('分块完整性校验失败');}
  if(current!==epoch||controller.signal.aborted)return;
  const mesh=new SplatMesh({fileBytes:new Uint8Array(bytes),fileType:SplatFileType.SPLAT,lod:false,raycastable:false});await mesh.initialized;
  if(current!==epoch||controller.signal.aborted){mesh.dispose();return;}
  mesh.position.set(...item.tile.origin);mesh.visible=false;world.add(mesh);const value={mesh,tileId:item.tile.id,count:item.level.gaussians,dispose(){disposeEntry(this);}};
  if(!budget.commit(item.key,value)){value.dispose();return;}
  for(const [key,old] of budget.resident)if(key!==item.key&&old.value.tileId===item.tile.id)budget.remove(key);
  downloads++;downloadedBytes+=Number(response.headers.get('content-length'))||item.level.bytes;repaint();
 }catch(error){if(error.name!=='AbortError'&&current===epoch)fail(error);}finally{if(current===epoch){budget.cancel(item.key);inflight.delete(item.key);repaint();}}
}
function plan(now){if(!manifest)return;world.updateMatrixWorld(true);camera.updateMatrixWorld();const position=world.worldToLocal(camera.position.clone());settings.focalPixels=innerHeight/(2*Math.tan(camera.fov*Math.PI/360));const selection=planTiles(manifest,position.toArray(),settings,visible);desired=selection.items;const wanted=new Set(desired.map(v=>v.key));
 for(const [key,task] of inflight)if(!wanted.has(key))task.controller.abort();
 for(const [key,entry] of budget.resident){const tile=manifest.tiles.find(t=>t.id===entry.value.tileId);if(!tile||distanceToBox(position.toArray(),tile.bounds)>settings.loadRadius+2)budget.remove(key);}
 budget.limit=settings.cacheBytes;budget.evict(0);repaint();
 for(const item of desired){if(inflight.size>=2)break;if(!budget.resident.has(item.key)&&!inflight.has(item.key))fetchTile(item);}
 lastPlan=now;
}
function animate(now){requestAnimationFrame(animate);if(!active||document.hidden){lastFrame=now;return;}frameMs=.9*frameMs+.1*Math.min(500,now-lastFrame);lastFrame=now;controls.update();if(now-lastPlan>200)plan(now);renderer.render(scene,camera);frames++;
 if(now-lastFps>1000){fps=frames*1000/(now-lastFps);frames=0;lastFps=now;const stats={fps:Math.round(fps),frame_ms:Math.round(frameMs),loaded_tiles:budget.resident.size,pending:inflight.size,rendered_gaussians:drawn,resident_estimated_bytes:budget.bytes,cache_limit:settings.cacheBytes,downloads,downloaded_bytes:downloadedBytes,textures:renderer.info.memory.textures,geometries:renderer.info.memory.geometries,scene_id:sceneId,version};$('metrics').textContent=`${stats.loaded_tiles} 块 · ${drawn.toLocaleString()} 高斯 · ${stats.fps} FPS · 估算缓存 ${(budget.bytes/1048576).toFixed(1)} MiB`;emit('scene-metrics',stats);window.gaussianStats=stats;}
}
requestAnimationFrame(animate);
addEventListener('message',event=>{if(event.origin!==location.origin||event.source!==parent)return;const d=event.data||{};if(d.type==='scene-load')loadScene(d.id);if(d.type==='scene-active'){active=!!d.active;if(!active)for(const task of inflight.values())task.controller.abort();else lastPlan=0;}
 if(d.type==='scene-settings'){settings.loadRadius=Math.max(1,Math.min(100,Number(d.loadRadius)||12));settings.renderRadius=Math.min(settings.loadRadius,Math.max(.5,Number(d.renderRadius)||10));settings.gaussianBudget=Math.max(1000,Math.min(1000000,Number(d.gaussianBudget)||300000));settings.cacheBytes=Math.max(4,Math.min(256,Number(d.cacheMiB)||64))*1048576;settings.pixelError=Math.max(1,Math.min(8,Number(d.pixelError)||3));lastPlan=0;}
 if(d.type==='scene-home'){follow=false;home();}if(d.type==='scene-keyframe'&&manifest?.keyframe_poses?.length){follow=false;keyframe=(keyframe+1)%manifest.keyframe_poses.length;cameraPose(manifest.keyframe_poses[keyframe]);}
 if(d.type==='scene-pose'){if(d.version!==version||d.status!=='localized'){marker.visible=false;return;}latestPose=d.matrix;const m=new THREE.Matrix4().set(...d.matrix.flat());m.decompose(marker.position,marker.quaternion,new THREE.Vector3());marker.visible=true;if(follow)cameraPose(d.matrix);}
 if(d.type==='scene-follow'){follow=!!d.enabled;if(follow&&latestPose)cameraPose(latestPose);}
 if(d.type==='scene-clear')clear();
});
addEventListener('pagehide',()=>{active=false;clear();spark.dispose();controls.dispose();renderer.dispose();});
emit('scene-viewer-ready',{});window.gaussianTest={home,setCamera(position,target){camera.position.set(...position);controls.target.set(...target);controls.update();lastPlan=0;},clear,settings};
