"""Bounded per-robot work executor; real sensor checks and owned recording only."""
import threading,time,logging
from concurrent.futures import ThreadPoolExecutor
from fleet_store import TERMINAL

class JobRunner:
 def __init__(self,store,transport):self.store=store;self.transport=transport;self.stop=threading.Event();self.thread=None
 def start(self):
  self.store.recover_interrupted();self.thread=threading.Thread(target=self.loop,name='fleet-jobs',daemon=True);self.thread.start()
 def close(self):
  self.stop.set()
  if self.thread:self.thread.join(timeout=12)
 def loop(self):
  pending={}
  with ThreadPoolExecutor(max_workers=8,thread_name_prefix='robot-job') as pool:
   while not self.stop.is_set():
    for rid,f in list(pending.items()):
     if f.done():
      try:f.result()
      except Exception:logging.getLogger('fleet.jobs').exception('job tick failed for robot %s',rid)
      del pending[rid]
    for robot in self.store.robots():
     rid=robot['id']
     if robot['enabled'] and rid not in pending and len(pending)<8:pending[rid]=pool.submit(self.tick,rid)
    self.stop.wait(.5)
 def call(self,rid,path,method='GET'):
  value=self.transport(rid,path,method)
  if not isinstance(value,dict):raise ValueError('节点返回格式异常')
  return value
 def complete_step(self,jid,message):
  def change(j):
   if j['status']!='running' or j['cancel_requested']:return
   i=j['step_index'];j['steps'][i].update(status='succeeded',finished_at=time.time());self.store.event(j,'step_completed',message)
   j['step_index']+=1;j['progress']=round(j['step_index']/len(j['steps'])*100,1)
   if j['step_index']==len(j['steps']):j.update(status='succeeded',finished_at=time.time())
   else:j['steps'][j['step_index']].update(status='running',started_at=time.time())
  self.store.mutate(jid,change)
 def cleanup(self,j):
  if not j.get('session_id'):
   if j.get('record_start_requested_at') and self.call(j['robot_id'],'/api/record/status').get('active'):raise ValueError('启动响应未确认，存在归属不明的录制，请在录制页面核对并停止后再取消')
   return
  rec=self.call(j['robot_id'],'/api/record/status')
  if rec.get('active'):
   if rec.get('session_id')!=j['session_id']:raise ValueError('当前录制不属于本作业，拒绝停止')
   self.call(j['robot_id'],'/api/record/stop','POST')
   if self.call(j['robot_id'],'/api/record/status').get('active'):raise ValueError('停止录制尚未确认')
 def tick(self,rid):
  active=next((j for j in self.store.active_jobs(rid) if j['status'] in {'running','needs_attention','canceling'}),None)
  j=active or self.store.claim(rid)
  if not j or j['status']=='needs_attention':return
  i=j['step_index'];kind=j['kind']
  try:
   if j['cancel_requested']:
    self.cleanup(j)
    def canceled(v):
     v.update(status='canceled',finished_at=time.time(),error=None)
     for step in v['steps']:
      if step['status'] in {'pending','running','failed'}:step['status']='canceled'
     self.store.event(v,'canceled','作业已取消，所属录制已核对')
    self.store.mutate(j['id'],canceled);return
   i=j['step_index'];kind=j['kind']
   if i==0:
    st=self.call(rid,'/api/state')
    if not st.get('online'):raise ValueError('机器人离线，作业无法执行')
    self.complete_step(j['id'],'机器人在线，前置检查通过')
   elif kind=='sensor_check':
    if i==1:
     cloud=self.call(rid,'/api/lidar/cloud?max_points=1')
     if not cloud.get('online'):raise ValueError('未收到新鲜激光雷达帧')
     message='激光雷达实时帧正常'
    elif i==2:
     depth=self.call(rid,'/api/depth/status')
     if not depth.get('online'):raise ValueError('深度相机离线或数据过期')
     message='深度相机实时数据正常'
    else:message='传感器巡检完成'
    self.complete_step(j['id'],message)
   elif kind=='capture':
    if i==1:
     if self.call(rid,'/api/record/status').get('active'):raise ValueError('机器人已在录制，不能占用其他录制会话')
     # Persist intent before side effect: restart never silently repeats a start.
     def intent(v):
      v['record_start_requested_at']=time.time();self.store.event(v,'record_start_requested','请求启动作业录制')
     self.store.mutate(j['id'],intent)
     reply=self.call(rid,'/api/record/start?label=fleet-'+j['id'],'POST')
     if reply.get('session_id'):
      self.store.mutate(j['id'],lambda v:v.update(session_id=reply['session_id']))
     deadline=time.monotonic()+5
     rec=self.call(rid,'/api/record/status')
     while not rec.get('active') and time.monotonic()<deadline:
      time.sleep(.2);rec=self.call(rid,'/api/record/status')
     if not rec.get('active') or not rec.get('session_id'):raise ValueError('录制启动未确认，请人工核对节点录制状态')
     def owned(v):v.update(session_id=rec['session_id'],record_started_at=time.time())
     self.store.mutate(j['id'],owned);self.complete_step(j['id'],'录制已确认，会话 '+rec['session_id'])
    elif i==2:
     st=self.call(rid,'/api/state');rec=self.call(rid,'/api/record/status')
     if not st.get('online'):raise ValueError('采集中机器人离线')
     if not rec.get('active') or rec.get('session_id')!=j['session_id']:raise ValueError('作业录制意外停止或会话不一致')
     elapsed=time.time()-j['record_started_at']
     if elapsed>=j['duration_s']:self.complete_step(j['id'],'计划采集时长已达到')
     else:
      def progress(v):
       if v['status']=='running':v.update(progress=round(50+min(1,elapsed/j['duration_s'])*25,1),sample_count=rec.get('sample_count',0))
      self.store.mutate(j['id'],progress)
    else:
     self.cleanup(j);rec=self.call(rid,'/api/record/status')
     if rec.get('session_id')!=j['session_id'] or not rec.get('sample_count',0):raise ValueError('录制结束，但未核验到本会话有效样本')
     self.complete_step(j['id'],'录制已停止，有效样本 '+str(rec['sample_count']))
  except Exception as e:
   cleanup_error=None
   try:self.cleanup(self.store.job(j['id']))
   except Exception as cleanup:cleanup_error=str(cleanup)
   def failed(v):
    if v['status'] in TERMINAL:return
    ambiguous=kind=='capture' and i==1 and bool(v.get('record_start_requested_at')) and not v.get('session_id')
    status='needs_attention' if cleanup_error or ambiguous else 'failed'
    v.update(status=status,error=str(e)[:300],finished_at=time.time() if status=='failed' else None)
    v['steps'][v['step_index']]['status']='failed';self.store.event(v,status,str(e)+(('; 清理未确认：'+cleanup_error) if cleanup_error else ''))
   self.store.mutate(j['id'],failed)
