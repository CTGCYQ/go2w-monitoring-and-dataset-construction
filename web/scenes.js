/* Central scene catalog, fused into the vision "world map" mode. Robot acquisition is scoped by ownership. */
(()=>{'use strict';
const el=id=>document.getElementById(id);
let active=false,mapKind='voxel',frame=null,ready=false,records=[],selected=null,poll=null,busy=false,loaded=null;

function ensureHost(){
  // The Gaussian viewer lives inside the vision page's world-map stage.
  let host=el('gaussLayer');
  if(!host){host=document.createElement('div');host.id='gaussLayer';host.hidden=true;host.style.cssText='position:absolute;inset:0;z-index:2;background:#152336';const stage=el('pointcloud3d')?.parentElement;if(stage)stage.append(host);}
  return host;
}
function ensureToolbar(){
  if(el('sceneToolbar'))return;
  const root=el('pcStudio');
  if(!root)return;
  const bar=document.createElement('div');
  bar.id='sceneToolbar';
  bar.className='pc-map-sub';
  bar.setAttribute('role','group');
  bar.setAttribute('aria-label','真实场景操作');
  bar.innerHTML=`<span class="pc-map-sub-label">真实场景</span><select id="sceneSelect" aria-label="场景选择" style="min-width:140px"></select><button id="sceneCreate">创建</button><input id="sceneName" maxlength="100" placeholder="场景名称" style="width:120px"><input id="sceneDuration" type="number" min="10" max="600" value="120" aria-label="采集秒数" style="width:64px"><button id="sceneCapture">同步采集</button><button id="sceneBuild">构建发布</button><button id="sceneLocate">重定位</button><button id="sceneRefresh">刷新</button><span id="sceneStatus" role="status" style="font-size:11px;color:#6a7f8f"></span>`;
  const head=root.querySelector('.pc-head');
  if(head&&head.nextSibling)head.parentNode.insertBefore(bar,head.nextSibling);else root.append(bar);
  bar.hidden=true;
  bindToolbar(bar);
}
function bindToolbar(bar){
  bar.querySelector('#sceneCreate').onclick=()=>busyRun(async()=>{const name=bar.querySelector('#sceneName').value.trim();if(!name)throw Error('请填写场景名称');const r=await api('/api/scenes',{method:'POST',body:JSON.stringify({name,robot_id:FleetContext.robotId})});selected=r.id;bar.querySelector('#sceneName').value='';});
  bar.querySelector('#sceneCapture').onclick=()=>busyRun(async()=>{if(!selected)throw Error('请先选择或创建场景');await api(`/api/scenes/${selected}/jobs/scene_capture`,{method:'POST',body:JSON.stringify({duration_s:+bar.querySelector('#sceneDuration').value})});});
  bar.querySelector('#sceneBuild').onclick=()=>busyRun(async()=>{if(!selected)throw Error('请先选择或创建场景');await api(`/api/scenes/${selected}/jobs/scene_build`,{method:'POST',body:JSON.stringify({duration_s:+bar.querySelector('#sceneDuration').value})});});
  bar.querySelector('#sceneLocate').onclick=()=>busyRun(async()=>{if(!selected)return;const rid=FleetContext.robotId;status('正在重定位…');const r=await api(`/api/scenes/${selected}/localize/${encodeURIComponent(rid)}`,{method:'POST'});send('scene-pose',{version:r.version,status:r.status,matrix:r.T_map_camera});status(r.status==='localized'?`定位通过：${r.inliers} 内点，重投影 ${r.reprojection_px.toFixed(2)} px`:`定位未通过：${r.reason||r.status}`);});
  bar.querySelector('#sceneRefresh').onclick=refresh;
  bar.querySelector('#sceneSelect').onchange=e=>{selected=e.target.value;load();};
}
async function api(path,options={}){const r=await FleetContext.nativeFetch(path,{...options,headers:{'Content-Type':'application/json',...options.headers}});const d=await r.json();if(!r.ok)throw Error(typeof d.detail==='string'?d.detail:JSON.stringify(d.detail));return d;}
function send(type,fields={}){if(ready&&frame)frame.contentWindow.postMessage({type,...fields},location.origin);}
function status(text){const s=el('sceneStatus');if(s)s.textContent=text;}
async function busyRun(fn){if(busy)return;busy=true;try{await fn();await refresh();}catch(e){status(e.message);}finally{busy=false;}}
function load(){const record=records.find(r=>r.id===selected);const key=record?.published?`${record.id}:${record.version}`:null;if(loaded!==key){loaded=key;send('scene-load',{id:record?.published?selected:null});const pose=el('scenePose');if(pose)pose.textContent='';}}
async function refresh(){try{const data=await api('/api/scenes');records=data.scenes.filter(r=>r.robot_id===FleetContext.robotId);if(!records.some(r=>r.id===selected))selected=records[0]?.id||null;const sel=el('sceneSelect');if(sel){sel.replaceChildren(...records.map(r=>{const o=document.createElement('option');o.value=r.id;o.textContent=r.name+' · '+(r.published?'已发布':r.stage);return o;}));sel.value=selected||'';}const r=records.find(v=>v.id===selected);status(r?`${r.name} · ${r.stage} · ${Number(r.progress||0).toFixed(0)}%${r.error?' · '+r.error:''}`:selected?'场景为空':'当前机器人暂无真实场景');load();}catch(e){status(e.message);}}
function setVisible(on){const layer=ensureHost();layer.hidden=!on;if(on&&!frame){frame=document.createElement('iframe');frame.title='高斯真实场景';frame.src='/static/gaussian/viewer.html';frame.style.cssText='border:0;width:100%;height:100%;display:block';layer.append(frame);}if(ready)send('scene-active',{active:on});if(on){refresh();poll=setInterval(refresh,3000);}else{clearInterval(poll);poll=null;}}
addEventListener('message',e=>{if(e.origin!==location.origin||e.source!==frame?.contentWindow)return;const d=e.data||{};if(d.type==='scene-viewer-ready'){ready=true;loaded=null;send('scene-active',{active:active&&mapKind==='gaussian'});load();}if(d.type==='scene-loaded'){}if(d.type==='scene-metrics'){const m=el('sceneMetrics');if(m)m.textContent=`${d.fps} FPS · ${d.rendered_gaussians} 高斯 · ${d.loaded_tiles} 分块 · ${d.pending} 请求`;}});
document.addEventListener('pc-map-kind-change',e=>{mapKind=e.detail?.kind||'voxel';const toolbar=el('sceneToolbar');if(toolbar)toolbar.hidden=!(active&&mapKind==='gaussian');if(active)setVisible(mapKind==='gaussian');});
window.SceneUI={activate(value){active=!!value;if(active)ensureToolbar();const toolbar=el('sceneToolbar');if(toolbar)toolbar.hidden=!(active&&mapKind==='gaussian');setVisible(active&&mapKind==='gaussian');window.dispatchEvent(new Event('scene-ui-ready'));}};
})();
