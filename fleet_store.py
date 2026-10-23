"""Persistent robot inventory and auditable jobs. No robot motion commands."""
import json,re,sqlite3,time,uuid
from contextlib import contextmanager
from pathlib import Path
from urllib.parse import urlsplit

class Conflict(ValueError): pass
class Missing(ValueError): pass
TERMINAL={'succeeded','failed','canceled'}
TEMPLATES={
 'sensor_check': {'name':'传感器巡检','steps':['连接与状态检查','激光雷达检查','深度相机检查','生成巡检结果']},
 'capture': {'name':'定时数据采集','steps':['采集前检查','启动数据录制','持续采集','停止录制并核验']},
}

class FleetStore:
 def __init__(self,path):
  self.path=str(path);Path(path).parent.mkdir(parents=True,exist_ok=True)
  with self.db() as c:
   c.executescript('PRAGMA journal_mode=WAL; CREATE TABLE IF NOT EXISTS robots(id TEXT PRIMARY KEY,data TEXT NOT NULL); CREATE TABLE IF NOT EXISTS jobs(id TEXT PRIMARY KEY,robot_id TEXT NOT NULL,status TEXT NOT NULL,data TEXT NOT NULL); CREATE UNIQUE INDEX IF NOT EXISTS one_active_job ON jobs(robot_id) WHERE status IN (\'running\',\'needs_attention\',\'canceling\');')
   if not c.execute('SELECT 1 FROM robots WHERE id=?',('go2w-01',)).fetchone():
    value={'id':'go2w-01','name':'Go2-W 01','model':'Unitree Go2-W','endpoint':None,'verify_tls':True,'enabled':True,'local':True,'created_at':time.time()};c.execute('INSERT INTO robots VALUES(?,?)',(value['id'],json.dumps(value)))
 @contextmanager
 def db(self):
  c=sqlite3.connect(self.path,timeout=10);c.row_factory=sqlite3.Row
  try:
   with c:yield c
  finally:c.close()
 def robots(self):
  with self.db() as c:return [json.loads(r['data']) for r in c.execute('SELECT data FROM robots ORDER BY id')]
 def robot(self,rid):
  with self.db() as c:row=c.execute('SELECT data FROM robots WHERE id=?',(rid,)).fetchone()
  if not row:raise Missing('机器人不存在')
  return json.loads(row['data'])
 def save_robot(self,value,existing=None):
  rid=value.get('id',existing or '')
  if not isinstance(rid,str) or not re.fullmatch(r'[a-zA-Z0-9][a-zA-Z0-9_-]{0,47}',rid):raise ValueError('机器人编号须为 1～48 位字母、数字、下划线或短横线')
  old=self.robot(existing) if existing else None
  if old and rid!=existing:raise ValueError('机器人编号不可修改')
  merged={**(old or {}),**{k:v for k,v in value.items() if k in {'id','name','model','endpoint','verify_tls','enabled'}}}
  for k in ['name','model']:
   if not isinstance(merged.get(k),str) or not 1<=len(merged[k].strip())<=80:raise ValueError('名称和型号须为 1～80 个字符')
   merged[k]=merged[k].strip()
  for k in ['verify_tls','enabled']:
   if k in merged and not isinstance(merged[k],bool):raise ValueError(k+' 必须为布尔值')
  if old and old.get('local'):
   if merged.get('endpoint') is not None:raise ValueError('本机采集节点地址不可修改')
  else:
   endpoint=merged.get('endpoint','')
   if not isinstance(endpoint,str):raise ValueError('请填写采集节点 URL')
   p=urlsplit(endpoint)
   if p.scheme not in {'http','https'} or not p.hostname or p.username or p.password or p.query or p.fragment or p.path not in {'','/'}:raise ValueError('节点地址须为 http(s)://主机:端口，不包含路径或密码')
   try:p.port
   except ValueError:raise ValueError('节点端口无效')
   merged['endpoint']=endpoint.rstrip('/')
  merged.update(id=rid,local=bool(old and old.get('local')),created_at=(old or {}).get('created_at',time.time()),updated_at=time.time())
  merged.setdefault('enabled',True);merged.setdefault('verify_tls',True)
  with self.db() as c:
   c.execute('BEGIN IMMEDIATE')
   if existing:
    if c.execute("SELECT 1 FROM jobs WHERE robot_id=? AND status NOT IN ('succeeded','failed','canceled')",(rid,)).fetchone():raise Conflict('该机器人仍有待执行或活动作业，请先完成或取消')
   elif c.execute('SELECT 1 FROM robots WHERE id=?',(rid,)).fetchone():raise Conflict('机器人编号已存在')
   for row in c.execute('SELECT data FROM robots WHERE id<>?',(rid,)):
    other=json.loads(row['data'])
    if merged.get('endpoint') and other.get('endpoint')==merged['endpoint']:raise Conflict('该采集节点已被其他机器人使用')
   c.execute('INSERT OR REPLACE INTO robots VALUES(?,?)',(rid,json.dumps(merged)))
  return merged
 def jobs(self,rid=None):
  with self.db() as c:
   query='SELECT data FROM jobs';args=()
   if rid:query+=' WHERE robot_id=?';args=(rid,)
   return [json.loads(r['data']) for r in c.execute(query+' ORDER BY rowid DESC LIMIT 500',args)]
 def active_jobs(self,rid=None):
  with self.db() as c:
   sql="SELECT data FROM jobs WHERE status IN ('running','needs_attention','canceling')";args=()
   if rid:sql+=' AND robot_id=?';args=(rid,)
   return [json.loads(row['data']) for row in c.execute(sql,args)]
 def job(self,jid):
  with self.db() as c:r=c.execute('SELECT data FROM jobs WHERE id=?',(jid,)).fetchone()
  if not r:raise Missing('作业不存在')
  return json.loads(r['data'])
 def create_job(self,value):
  r=self.robot(value.get('robot_id',''))
  if not r['enabled']:raise Conflict('机器人已停用')
  kind=value.get('kind');title=value.get('title','')
  if kind not in TEMPLATES:raise ValueError('不支持的作业类型')
  if not isinstance(title,str) or not 1<=len(title.strip())<=120:raise ValueError('作业名称须为 1～120 个字符')
  duration=value.get('duration_s',60)
  if isinstance(duration,bool) or not isinstance(duration,int) or not 5<=duration<=3600:raise ValueError('采集时长须为 5～3600 秒的整数')
  now=time.time();j={'id':uuid.uuid4().hex,'title':title.strip(),'robot_id':r['id'],'kind':kind,'duration_s':duration,'status':'queued','step_index':0,'progress':0,'created_at':now,'started_at':None,'finished_at':None,'session_id':None,'cancel_requested':False,'revision':0,'steps':[{'name':n,'status':'pending'} for n in TEMPLATES[kind]['steps']],'events':[{'at':now,'type':'created','message':'作业已指派，等待执行'}]}
  with self.db() as c:c.execute('INSERT INTO jobs VALUES(?,?,?,?)',(j['id'],j['robot_id'],j['status'],json.dumps(j)))
  return j
 def mutate(self,jid,fn):
  with self.db() as c:
   c.execute('BEGIN IMMEDIATE');row=c.execute('SELECT data FROM jobs WHERE id=?',(jid,)).fetchone()
   if not row:raise Missing('作业不存在')
   j=json.loads(row['data']);fn(j);j['revision']+=1
   c.execute('UPDATE jobs SET status=?,data=? WHERE id=?',(j['status'],json.dumps(j),jid));return j
 def claim(self,rid):
  with self.db() as c:
   c.execute('BEGIN IMMEDIATE')
   if c.execute("SELECT 1 FROM jobs WHERE robot_id=? AND status IN ('running','needs_attention','canceling')",(rid,)).fetchone():return None
   row=c.execute("SELECT data FROM jobs WHERE robot_id=? AND status='queued' ORDER BY rowid LIMIT 1",(rid,)).fetchone()
   if not row:return None
   j=json.loads(row['data']);j.update(status='running',started_at=time.time());j['steps'][0]['status']='running';self.event(j,'started','执行器已接单')
   c.execute('UPDATE jobs SET status=?,data=? WHERE id=?',(j['status'],json.dumps(j),j['id']));return j
 @staticmethod
 def event(j,kind,message):
  j['events'].append({'at':time.time(),'type':kind,'message':str(message)[:500]});j['events']=j['events'][-150:]
 def cancel(self,jid):
  def change(j):
   if j['status'] in TERMINAL:raise Conflict('已结束作业不能再次取消')
   if j['status']=='queued':
    j.update(status='canceled',finished_at=time.time())
    for step in j['steps']:step['status']='canceled'
   else:j.update(status='canceling',cancel_requested=True)
   self.event(j,'cancel_requested','操作员请求取消作业')
  return self.mutate(jid,change)
 def recover_interrupted(self):
  for j in self.active_jobs():
   if j['status'] in {'running','canceling'}:
    def change(v):
     v['status']='needs_attention';v['error']='平台执行器重启，等待核对；不会重复启动录制';self.event(v,'interrupted',v['error'])
    self.mutate(j['id'],change)

def job_metrics(jobs):
 total=len(jobs);started=sum(j.get('started_at') is not None for j in jobs);succeeded=sum(j['status']=='succeeded' for j in jobs);failed=sum(j['status']=='failed' for j in jobs)
 return {'total':total,'started':started,'running':sum(j['status']=='running' for j in jobs),'queued':sum(j['status']=='queued' for j in jobs),'needs_attention':sum(j['status']=='needs_attention' for j in jobs),'succeeded':succeeded,'failed':failed,'canceled':sum(j['status']=='canceled' for j in jobs),'execution_rate':started/total if total else None,'success_rate':succeeded/(succeeded+failed) if succeeded+failed else None,'scope':'latest_500'}
