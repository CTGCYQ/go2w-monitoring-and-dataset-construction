"""Robot-scoped compatibility gateway and central workbench API."""
import asyncio,json,os,time,threading
from concurrent.futures import ThreadPoolExecutor
from urllib.parse import urlsplit
import requests
from fastapi import HTTPException,Request
from fastapi.responses import JSONResponse,StreamingResponse
from starlette.background import BackgroundTask
from fleet_store import FleetStore,Conflict,Missing,TEMPLATES,job_metrics
from fleet_jobs import JobRunner

ROUTES={'state','health','history','checkpoints','record','sessions','dataset','schedule','session','depth','image','map','system','lidar','slam','agent'}

def install_fleet(app,root,read_state):
 store=FleetStore(root/'fleet.sqlite3');local_url=os.environ.get('FLEET_LOCAL_URL','https://127.0.0.1:8000').rstrip('/')
 cache={};cache_lock=threading.Lock()
 def require(rid):
  try:r=store.robot(rid)
  except Missing as e:raise HTTPException(404,str(e))
  if not r['enabled']:raise HTTPException(409,'机器人已停用')
  return r
 def transport(rid,path,method='GET'):
  r=require(rid);base=r['endpoint'] or local_url
  response=requests.request(method,base+path,timeout=(3,40),verify=r['verify_tls'] if r['endpoint'] else False,allow_redirects=False)
  if response.status_code>=300:raise ValueError('节点请求失败 HTTP '+str(response.status_code))
  return response.json()
 runner=JobRunner(store,transport)
 def robot_status(r):
  if not r['enabled']:return {**r,'online':False,'reason':'机器人已停用'}
  now=time.monotonic()
  with cache_lock:old=cache.get(r['id'])
  if old and now-old[0]<2:return old[1]
  started=time.monotonic()
  try:
   if r['local']:state=read_state()
   else:
    response=requests.get(r['endpoint']+'/api/state',timeout=(1.5,2),verify=r['verify_tls'],allow_redirects=False);response.raise_for_status();state=response.json()
   if not isinstance(state,dict):raise ValueError('节点返回格式异常')
   result={**r,'online':bool(state.get('online')),'battery':state.get('battery'),'hz':state.get('hz'),'last_seen_s':state.get('last_seen_s'),'recording':state.get('recording'),'reason':state.get('reason'),'latency_ms':round((time.monotonic()-started)*1000,1),'checked_at':time.time()}
  except Exception:result={**r,'online':False,'reason':'节点不可达、证书校验失败或响应无效','checked_at':time.time()}
  with cache_lock:cache[r['id']]=(now,result)
  return result
 def wrap(fn,*args):
  try:return fn(*args)
  except Missing as e:raise HTTPException(404,str(e))
  except Conflict as e:raise HTTPException(409,str(e))
  except (ValueError,TypeError) as e:raise HTTPException(422,str(e))
 @app.on_event('startup')
 def fleet_start():runner.start()
 @app.on_event('shutdown')
 def fleet_stop():runner.close()
 @app.get('/api/fleet/robots')
 def robots():
  robots=store.robots()
  with ThreadPoolExecutor(max_workers=8) as pool:items=list(pool.map(robot_status,robots))
  active={j['robot_id']:{k:j[k] for k in ('id','title','status','progress')} for j in store.active_jobs()}
  items=[{**r,'active_job':active.get(r['id'])} for r in items]
  return {'robots':items,'default_robot_id':'go2w-01','platform_name':'具身智能机器人综合管理平台'}
 @app.post('/api/fleet/robots',status_code=201)
 async def add_robot(request:Request):
  value=await request.json();validate_target(value,request)
  return wrap(store.save_robot,value)
 @app.patch('/api/fleet/robots/{rid}')
 async def update_robot(rid:str,request:Request):
  value=await request.json();validate_target(value,request)
  result=wrap(store.save_robot,value,rid)
  with cache_lock:cache.pop(rid,None)
  return result
 def validate_target(value,request):
  if not isinstance(value,dict):raise HTTPException(422,'请求必须是对象')
  if value.get('endpoint'):
   try:p=urlsplit(str(value['endpoint']));port=p.port
   except ValueError:raise HTTPException(422,'节点地址或端口无效')
   host=request.url.hostname
   if p.hostname in {host,'127.0.0.1','localhost','::1'} and (p.port or (443 if p.scheme=='https' else 80))==(request.url.port or (443 if request.url.scheme=='https' else 80)):raise HTTPException(422,'不能将管理平台自身重复注册为远程机器人')
 @app.get('/api/fleet/health')
 def fleet_health():return {'executor_online':bool(runner.thread and runner.thread.is_alive()),'robot_count':len(store.robots())}
 @app.get('/api/fleet/templates')
 def templates():return {'templates':[{'kind':k,**v} for k,v in TEMPLATES.items()]}
 @app.get('/api/fleet/jobs')
 def jobs(robot_id:str=None):
  if robot_id:wrap(store.robot,robot_id)
  values=store.jobs(robot_id);return {'jobs':values,'metrics':job_metrics(values)}
 @app.post('/api/fleet/jobs',status_code=201)
 async def create_job(request:Request):
  value=await request.json()
  if not isinstance(value,dict):raise HTTPException(422,'请求必须是对象')
  return wrap(store.create_job,value)
 @app.get('/api/fleet/jobs/{jid}')
 def job(jid:str):return wrap(store.job,jid)
 @app.post('/api/fleet/jobs/{jid}/cancel')
 def cancel(jid:str):return wrap(store.cancel,jid)
 @app.middleware('http')
 async def scoped_gateway(request,call_next):
  path=request.scope['path'];prefix='/api/robots/'
  if not path.startswith(prefix):return await call_next(request)
  parts=path[len(prefix):].split('/',1)
  if len(parts)!=2:return JSONResponse({'detail':'缺少机器人 API 路径'},status_code=404)
  rid,tail=parts;segments=tail.split('/')
  if segments[0] not in ROUTES or '..' in segments:return JSONResponse({'detail':'未知的机器人 API'},status_code=404)
  try:r=require(rid)
  except HTTPException as e:return JSONResponse({'detail':e.detail,'robot_id':rid},status_code=e.status_code)
  target='/api/'+tail
  if r['local']:
   request.scope['path']=target;request.scope['raw_path']=target.encode()
   response=await call_next(request);response.headers['X-Robot-ID']=rid;response.headers['Cache-Control']='no-store';return response
  if int(request.headers.get('content-length','0') or 0)>32*1024*1024:return JSONResponse({'detail':'上传超过 32 MiB'},status_code=413)
  body=await request.body()
  if len(body)>32*1024*1024:return JSONResponse({'detail':'上传超过 32 MiB'},status_code=413)
  url=r['endpoint']+target
  if request.url.query:url+='?'+request.url.query
  headers={k:v for k,v in request.headers.items() if k.lower() in {'content-type','accept','range'}}
  try:
   upstream=await asyncio.to_thread(requests.request,request.method,url,data=body,headers=headers,timeout=(3,95),verify=r['verify_tls'],allow_redirects=False,stream=True)
  except requests.RequestException:return JSONResponse({'detail':'所选机器人节点连接失败','robot_id':rid},status_code=502)
  if 300<=upstream.status_code<400:
   upstream.close();return JSONResponse({'detail':'节点返回重定向，已拒绝','robot_id':rid},status_code=502)
  def chunks():
   try:yield from upstream.iter_content(chunk_size=65536)
   finally:upstream.close()
  output_headers={k:v for k,v in upstream.headers.items() if k.lower() in {'content-type','content-disposition','content-range','accept-ranges'}}
  output_headers.update({'X-Robot-ID':rid,'Cache-Control':'no-store'})
  return StreamingResponse(chunks(),status_code=upstream.status_code,headers=output_headers,background=BackgroundTask(upstream.close))
 app.state.fleet_store=store;app.state.fleet_runner=runner
 return store
