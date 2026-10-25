/* Fleet inventory and operation desk, backed by persistent server state. */
(()=>{'use strict';
 const el=id=>document.getElementById(id), esc=v=>String(v??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
 const statuses={queued:'待执行',running:'执行中',needs_attention:'待处理',canceling:'取消中',succeeded:'已成功',failed:'已失败',canceled:'已取消'};
 const badge=s=>`<span class="badge ${s==='succeeded'?'green':s==='running'?'accent':s==='failed'?'red':s==='needs_attention'?'amber':''}">${esc(statuses[s]||s)}</span>`;
 const time=t=>t?new Date(t*1000).toLocaleString('zh-CN',{hour12:false}):'—',pct=n=>n==null?'—':(n*100).toFixed(1)+'%';
 let robots=[],jobs=[],chosenJob=null,busy=false;
 async function api(path,options){const r=await FleetContext.nativeFetch(path,{...options,headers:{'Content-Type':'application/json',...options?.headers}});const d=await r.json().catch(()=>({}));if(!r.ok)throw new Error(d.detail||`请求失败 (${r.status})`);return d;}
 function options(select,values,chosen){select.innerHTML=values.map(r=>`<option value="${esc(r.id)}">${esc(r.name)} · ${r.enabled?(r.online?'在线':'离线'):'已停用'}</option>`).join('');if(values.some(r=>r.id===chosen))select.value=chosen;}
 function renderRobots(){
  const selected=robots.find(r=>r.id===FleetContext.robotId);options(el('robotSelector'),robots,FleetContext.robotId);
  if(!selected){el('robotSelector').insertAdjacentHTML('afterbegin',`<option selected value="${esc(FleetContext.robotId)}">未知机器人：${esc(FleetContext.robotId)}</option>`);}
  el('robotContextName').textContent=selected?.name||'未知机器人';el('robotContextState').textContent=selected?(selected.enabled?(selected.online?'在线':'离线'):'已停用'):'请在机器人管理中选择有效节点';el('fRobot').textContent=selected?.name||FleetContext.robotId;
  const old=el('jobRobot').value;options(el('jobRobot'),robots.filter(r=>r.enabled),old||FleetContext.robotId);
  el('fleetCount').textContent=`${robots.filter(r=>r.online).length} 在线 / ${robots.length} 已接入`;
  el('fleetCards').innerHTML=robots.map(r=>{
   const battery=typeof r.battery==='number'?r.battery:r.battery?.soc;const active=r.active_job;
   return `<article class="robot-card ${r.id===FleetContext.robotId?'selected':''}"><header><h3>${esc(r.name)}</h3><span class="badge ${r.online?'green':''}">${r.enabled?(r.online?'● 在线':'○ 离线'):'已停用'}</span></header><p>${esc(r.model)} · <code>${esc(r.id)}</code></p><p>${r.local?'本机采集节点':esc(r.endpoint)}</p><div class="robot-specs"><div><span>电池电量</span><b>${battery==null?'—':esc(battery)+'%'}</b></div><div><span>采集频率</span><b>${r.hz==null?'—':Number(r.hz).toFixed(0)+' Hz'}</b></div></div><p>${active?'当前作业：'+esc(active.title)+' · '+esc(statuses[active.status]):r.online?'可查看实时状态与指派作业':esc(r.reason||'等待节点上线')}</p><footer><button data-select-robot="${esc(r.id)}" class="${r.id===FleetContext.robotId?'fleet-btn-primary':''}">${r.id===FleetContext.robotId?'当前机器人':'查看机器人'}</button><button data-edit-robot="${esc(r.id)}">配置</button></footer></article>`;
  }).join('')||'<div class="fleet-empty">还没有接入机器人。点击“接入机器人”配置采集节点。</div>';
 }
 function renderJobs(metrics){
  const stats=[['作业总数',metrics.total,'最近 500 条 · 当前筛选'],['执行中',metrics.running,`${metrics.queued} 待执行 · ${metrics.needs_attention} 待处理`],['执行率',pct(metrics.execution_rate),'已开始 / 全部作业'],['成功率',pct(metrics.success_rate),'成功 /（成功 + 失败）'],['已成功',metrics.succeeded,`${metrics.failed} 失败 · ${metrics.canceled} 取消`]];
  el('jobStats').innerHTML=stats.map(([label,value,note])=>`<div class="fleet-stat"><span>${esc(label)}</span><strong>${esc(value)}</strong><small>${esc(note)}</small></div>`).join('');
  if(!jobs.some(j=>j.id===chosenJob))chosenJob=jobs.find(j=>j.status==='running')?.id||jobs[0]?.id||null;
  el('jobListCount').textContent=jobs.length+' 项';
  el('jobRows').innerHTML=jobs.map(j=>`<button class="job-row ${j.id===chosenJob?'selected':''}" data-job="${esc(j.id)}"><span class="job-row-top"><span class="job-row-title">${esc(j.title)}</span>${badge(j.status)}</span><small>${esc(robots.find(r=>r.id===j.robot_id)?.name||j.robot_id)} · ${time(j.created_at)}</small><div class="job-progress"><i style="width:${Number(j.progress)||0}%"></i></div><small>${esc(j.steps[Math.min(j.step_index,j.steps.length-1)]?.name)} · ${j.progress}%</small></button>`).join('')||'<div class="fleet-empty">暂无作业<br>指派一次传感器巡检，或建立定时采集作业。</div>';
  renderDetail();
 }
 function renderDetail(){
  const j=jobs.find(j=>j.id===chosenJob);if(!j){el('jobDetail').innerHTML='<div class="fleet-empty">选择一项作业，查看机器人状态、步骤进度和执行记录。</div>';return;}
  const robot=robots.find(r=>r.id===j.robot_id);const detail=el('jobDetail');
  detail.innerHTML=`<div class="job-section-head"><span>作业执行详情</span>${badge(j.status)}</div><div class="job-detail-content"><div class="fleet-eyebrow">JOB · ${esc(j.id.slice(0,12))}</div><h2>${esc(j.title)}</h2><p class="job-meta">${esc(robot?.name||j.robot_id)} · ${({capture:'定时数据采集',sensor_check:'传感器巡检',scene_capture:'同步场景采集',scene_build:'高斯地图构建'})[j.kind]||j.kind}<br>指派 ${time(j.created_at)}${j.finished_at?' · 结束 '+time(j.finished_at):''}</p><div class="job-live-strip" style="margin-top:16px"><span>机器人 <b>${robot?.online?'在线':'离线'}</b></span><span>采集频率 <b>${robot?.hz==null?'—':Number(robot.hz).toFixed(0)+' Hz'}</b></span><span>总进度 <b>${j.progress}%</b></span><span>采集样本 <b>${j.sample_count??'—'}</b></span></div>${robot?.online&&document.querySelector('.tab.active')?.dataset.tab==='jobs'?`<img class="job-camera" alt="执行机器人实时画面" src="/api/robots/${encodeURIComponent(j.robot_id)}/image/latest?t=${Date.now()}" onerror="this.hidden=true">`:''}${j.error?`<div class="job-alert">${esc(j.error)}</div>`:''}<ol class="job-timeline">${j.steps.map((s,i)=>`<li class="job-step ${esc(s.status)}"><span class="step-dot">${s.status==='succeeded'?'✓':i+1}</span><div><h4>${esc(s.name)}</h4><small>${({pending:'等待上一环节完成',running:'正在执行 · 等待后端核验',succeeded:'已核验完成',failed:'执行异常'})[s.status]}${s.finished_at?' · '+time(s.finished_at):''}</small></div></li>`).join('')}</ol><div class="fleet-actions"><button data-job-monitor="${esc(j.robot_id)}">查看该机器人实时监控</button>${!['succeeded','failed','canceled'].includes(j.status)?`<button class="red" data-cancel-job="${esc(j.id)}" ${j.status==='canceling'?'disabled':''}>${j.status==='needs_attention'?'取消并核对录制':'取消作业'}</button>`:''}</div>${j.session_id?`<p class="job-meta" style="margin-top:12px">录制会话：${esc(j.session_id)}</p>`:''}<h3 style="font-size:12px;margin:25px 0 10px">执行记录</h3><div class="job-events">${[...j.events].reverse().map(e=>`<div class="job-event"><time>${new Date(e.at*1000).toLocaleTimeString('zh-CN',{hour12:false})}</time><span>${esc(e.message)}</span></div>`).join('')}</div></div>`;
 }
 async function refresh(){
  if(busy)return;busy=true;
  try{const inventory=await api('/api/fleet/robots');robots=inventory.robots;renderRobots();const rid=el('jobScope').value==='all'?'':'?robot_id='+encodeURIComponent(FleetContext.robotId);const d=await api('/api/fleet/jobs'+rid);jobs=d.jobs;renderRobots();renderJobs(d.metrics);el('fleetGlobalError').textContent='';}
  catch(e){el('fleetGlobalError').textContent=e.message;el('robotContextState').textContent='平台连接异常';}finally{busy=false;}
 }
 function openRobot(id){const r=robots.find(v=>v.id===id);el('robotDialogTitle').textContent=r?'配置机器人':'接入机器人';el('robotForm').reset();el('robotId').value=r?.id||'';el('robotId').readOnly=!!r;el('robotName').value=r?.name||'';el('robotModel').value=r?.model||'';el('robotEndpoint').value=r?.endpoint||'';el('robotEndpoint').disabled=!!r?.local;el('robotEndpoint').required=!r?.local;el('robotTls').checked=r?.verify_tls??true;el('robotEnabled').checked=r?.enabled??true;el('robotForm').dataset.edit=id||'';el('robotFormError').textContent='';el('robotDialog').showModal();}
 document.addEventListener('DOMContentLoaded',()=>{
  el('robotSelector').addEventListener('change',e=>FleetContext.select(e.target.value));
  el('addRobotBtn').onclick=()=>openRobot();el('fleetRefreshBtn').onclick=refresh;el('jobScope').onchange=refresh;
  el('createJobBtn').onclick=()=>{el('jobFormError').textContent='';el('jobTitle').value='';el('jobRobot').value=FleetContext.robotId;el('jobDialog').showModal();};
  el('jobKind').onchange=()=>{el('captureDurationField').hidden=el('jobKind').value!=='capture';};
  document.addEventListener('click',async e=>{
   const b=e.target.closest('button');if(!b)return;
   if(b.dataset.closeDialog)el(b.dataset.closeDialog).close();
   if(b.dataset.selectRobot)FleetContext.select(b.dataset.selectRobot);
   if(b.dataset.editRobot)openRobot(b.dataset.editRobot);
   if(b.dataset.job){chosenJob=b.dataset.job;renderJobsFromSelection();}
   if(b.dataset.jobMonitor){const u=new URL(location.href);u.searchParams.set('robot',b.dataset.jobMonitor);u.searchParams.set('tab','monitor');location.assign(u);}
   if(b.dataset.cancelJob){b.disabled=true;try{await api('/api/fleet/jobs/'+b.dataset.cancelJob+'/cancel',{method:'POST'});await refresh();}catch(err){el('fleetGlobalError').textContent=err.message;}finally{b.disabled=false;}}
  });
  el('robotForm').onsubmit=async e=>{e.preventDefault();const submit=e.submitter;submit.disabled=true;try{const edit=e.target.dataset.edit;await api('/api/fleet/robots'+(edit?'/'+encodeURIComponent(edit):''),{method:edit?'PATCH':'POST',body:JSON.stringify({id:el('robotId').value.trim(),name:el('robotName').value.trim(),model:el('robotModel').value.trim(),endpoint:el('robotEndpoint').disabled?null:el('robotEndpoint').value.trim(),verify_tls:el('robotTls').checked,enabled:el('robotEnabled').checked})});el('robotDialog').close();await refresh();}catch(err){el('robotFormError').textContent=err.message;}finally{submit.disabled=false;}};
  el('jobForm').onsubmit=async e=>{e.preventDefault();const submit=e.submitter;submit.disabled=true;try{const j=await api('/api/fleet/jobs',{method:'POST',body:JSON.stringify({robot_id:el('jobRobot').value,title:el('jobTitle').value.trim(),kind:el('jobKind').value,duration_s:Number(el('jobDuration').value)})});chosenJob=j.id;el('jobDialog').close();if(el('jobRobot').value!==FleetContext.robotId)el('jobScope').value='all';switchTab('jobs');await refresh();}catch(err){el('jobFormError').textContent=err.message;}finally{submit.disabled=false;}};
  refresh();setInterval(()=>{if(!document.hidden)refresh();},3000);
  const tab=new URLSearchParams(location.search).get('tab');if(tab&&document.querySelector(`.tab[data-tab="${CSS.escape(tab)}"]`))switchTab(tab);
 });
 function renderJobsFromSelection(){document.querySelectorAll('[data-job]').forEach(b=>b.classList.toggle('selected',b.dataset.job===chosenJob));renderDetail();}
 window.FleetUI={refresh};
})();
