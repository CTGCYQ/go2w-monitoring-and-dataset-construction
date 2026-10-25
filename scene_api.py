"""Scene acquisition, real build jobs, immutable spatial assets and camera localization."""
import io,json,os,shutil,signal,subprocess,threading,time,uuid
from pathlib import Path
import cv2
import numpy as np
import requests
from fastapi import HTTPException,Request
from fastapi.responses import FileResponse,Response
from depth_stream import DepthReceiver,packet_with_age,split_rgbd_packet
from scene_capture import SceneCapture
from scene_store import SceneCatalog,identifier
from scene_geometry import feature_frame,localize_camera


class SceneService:
    def __init__(self,root,fleet,receiver,local_url='https://127.0.0.1:8000',training_python='/home/dell/go2w_gaussian/.venv/bin/python'):
        self.catalog=SceneCatalog(root);self.fleet=fleet;self.receiver=receiver;self.local_url=local_url;self.training_python=training_python;self.captures={};self.lock=threading.RLock();self.localizations={}
    def frame(self,rid):
        robot=self.fleet.robot(rid)
        if not robot['enabled']:raise ValueError('robot disabled')
        if robot['local']:
            summary,packet=self.receiver.snapshot()
            if not summary.get('online') or packet is None:raise ValueError('paired RGB-D stream offline')
            return summary,packet_with_age(packet,summary.get('arrival_age_s',0))
        response=requests.get(robot['endpoint']+'/api/rgbd/frame',timeout=(3,5),verify=robot['verify_tls'],stream=True,allow_redirects=False)
        try:
            response.raise_for_status();chunks=[];total=0
            for chunk in response.iter_content(65536):
                total+=len(chunk)
                if total>5*1024**2:raise ValueError('RGB-D packet exceeds size limit')
                chunks.append(chunk)
            packet=b''.join(chunks)
            header,_,_=split_rgbd_packet(packet);return {'online':True,**header},packet
        finally:response.close()
    def refresh(self,sid):
        record=self.catalog.get(sid)
        if record['stage']=='capturing':
            if sid not in self.captures:
                return self.catalog.update(sid,stage='interrupted',error='capture worker stopped; start a new capture')
            capture_id=record.get('capture_id');path=self.catalog.root/'raw'/capture_id/'capture.json'
            if path.exists():
                value=json.loads(path.read_text(encoding='utf8'));updates={'capture':value,'progress':min(20,20*value.get('elapsed_s',time.time()-value.get('started_at',time.time()))/record.get('capture_seconds',180))}
                if value['status'] in {'captured','failed'}:updates.update(stage=value['status'],error=value.get('error'),progress=20 if value['status']=='captured' else record['progress'])
                record=self.catalog.update(sid,**updates)
        if record['stage']=='training' and record.get('build_id'):
            path=self.catalog.root/'build'/record['build_id']/'trained'/'training_progress.json'
            if path.exists():
                value=json.loads(path.read_text(encoding='utf8'));record={**record,'training':value,'progress':45+35*value['step']/max(1,value['steps'])}
        if record['stage'] in {'geometry','training','packing'} and record.get('worker_pid'):
            try:
                args=Path(f"/proc/{record['worker_pid']}/cmdline").read_bytes()
                if b'scene_pipeline.py' not in args or sid.encode() not in args:raise OSError()
            except OSError:record=self.catalog.update(sid,stage='interrupted',error='map worker stopped before publication; retry after review')
        return record
    def start_capture(self,sid,seconds,job_id):
        with self.lock:
            record=self.refresh(sid)
            if record['stage'] in {'capturing','geometry','training','packing'}:raise ValueError('场景已有活动任务')
            if any(self.refresh(r['id'])['stage']=='capturing' and r['robot_id']==record['robot_id'] for r in self.catalog.list()):raise ValueError('该机器人已有场景采集任务')
            sizes=self.catalog.storage()
            if sizes['free_bytes']<2*1024**3 or sizes['raw']+256*1024**2>8*1024**3:raise ValueError('原始场景存储配额不足')
            capture_id=uuid.uuid4().hex;capture=SceneCapture(self.catalog.root/'raw'/capture_id,lambda:self.frame(record['robot_id']),record['robot_id'],seconds=seconds)
            self.catalog.update(sid,stage='capturing',capture_id=capture_id,capture_seconds=seconds,job_id=job_id,error=None,progress=0)
            def run():
                try:
                    result=capture.run()
                    if self.catalog.get(sid)['stage']!='canceled':self.catalog.update(sid,stage='captured',capture=result,progress=20)
                except Exception as error:
                    if self.catalog.get(sid)['stage']!='canceled':self.catalog.update(sid,stage='failed',error=str(error))
                finally:
                    with self.lock:self.captures.pop(sid,None)
            self.captures[sid]=capture;threading.Thread(target=run,name='scene-capture',daemon=True).start()
    def start_build(self,sid,job_id):
        with self.lock:
            record=self.refresh(sid)
            if record['stage'] in {'capturing','geometry','training','packing'}:raise ValueError('场景已有活动任务')
            if not record.get('capture_id'):raise ValueError('请先采集真实 RGB-D 场景')
            if not Path(self.training_python).is_file():raise ValueError('高斯训练环境未安装')
            if any(self.refresh(r['id'])['stage'] in {'geometry','training','packing'} for r in self.catalog.list()):raise ValueError('另一地图构建正在使用训练工作位')
            build_id=uuid.uuid4().hex;logs=self.catalog.root/'build'/f'{build_id}.log';env={**os.environ,'CUDA_HOME':'/usr/local/cuda-12.8','MAX_JOBS':'2','TORCH_CUDA_ARCH_LIST':'8.6'};env['PATH']=str(Path(self.training_python).parent)+':/usr/local/cuda-12.8/bin:'+env.get('PATH','')
            self.catalog.update(sid,stage='geometry',build_id=build_id,job_id=job_id,progress=21,error=None)
            with logs.open('wb') as out:
                process=subprocess.Popen([self.training_python,str(Path(__file__).with_name('scene_pipeline.py')),'--root',str(self.catalog.root),'--scene-id',sid,'--capture-id',record['capture_id'],'--build-id',build_id],stdout=out,stderr=subprocess.STDOUT,env=env,start_new_session=True)
            self.catalog.update(sid,worker_pid=process.pid)
    def cancel(self,sid,job_id):
        with self.lock:
            record=self.catalog.get(sid)
            if record.get('job_id')!=job_id:
                return
            capture=self.captures.get(sid)
            if capture:capture.stop.set()
            if record['stage'] in {'geometry','training','packing'} and record.get('worker_pid'):
                try:
                    args=Path(f"/proc/{record['worker_pid']}/cmdline").read_bytes()
                    if b'scene_pipeline.py' in args and sid.encode() in args:os.killpg(record['worker_pid'],signal.SIGTERM)
                except ProcessLookupError:pass
            self.catalog.update(sid,stage='canceled',error=None)
    def job_tick(self,job):
        sid=job.get('scene_id')
        try:
            record=self.catalog.get(sid)
            if record['robot_id']!=job['robot_id']:raise ValueError('作业机器人与场景不一致')
            if job['cancel_requested']:
                self.cancel(sid,job['id'])
                def canceled(v):
                    v.update(status='canceled',finished_at=time.time(),error=None)
                    for step in v['steps']:
                        if step['status']!='succeeded':step['status']='canceled'
                    self.fleet.event(v,'canceled','场景任务已取消')
                self.fleet.mutate(job['id'],canceled);return
            if not job.get('scene_started'):
                if job['kind']=='scene_capture':self.start_capture(sid,job['duration_s'],job['id'])
                else:self.start_build(sid,job['id'])
                self.fleet.mutate(job['id'],lambda v:v.update(scene_started=True));return
            record=self.refresh(sid);stage=record['stage']
            if stage in {'failed','interrupted','canceled'}:raise ValueError(record.get('error') or '场景任务中断')
            completed=(job['kind']=='scene_capture' and stage=='captured') or (job['kind']=='scene_build' and stage=='ready')
            indices={'capturing':1,'captured':2,'geometry':1,'training':2,'packing':3,'ready':6}
            count=len(job['steps']);index=count if completed else min(count-1,indices.get(stage,0))
            def progress(v):
                if v['status']!='running':return
                v.update(progress=100 if completed else round(record.get('progress',0),1),step_index=index)
                for i,step in enumerate(v['steps']):step['status']='succeeded' if i<index else ('running' if i==index else 'pending')
                if completed:v.update(status='succeeded',finished_at=time.time());self.fleet.event(v,'completed','场景采集完成' if job['kind']=='scene_capture' else '高斯地图已完成核验并发布')
            self.fleet.mutate(job['id'],progress)
        except Exception as error:
            def failed(v):v.update(status='failed',finished_at=time.time(),error=str(error));self.fleet.event(v,'failed',str(error))
            self.fleet.mutate(job['id'],failed)
    def localize(self,sid,rid):
        record=self.catalog.get(sid)
        if not record.get('published'):raise ValueError('场景尚未发布')
        started=time.time()
        summary,packet=self.frame(rid);header,raw,rgb=split_rgbd_packet(packet)
        image=cv2.imdecode(np.frombuffer(rgb,np.uint8),cv2.IMREAD_COLOR);depth=np.frombuffer(raw,'<u2').reshape(header['height'],header['width']).astype(np.float32)*header['meters_per_unit'];K=np.array([[header['fx'],0,header['ppx']],[0,header['fy'],header['ppy']],[0,0,1]],np.float64)
        result=localize_camera(self.catalog.root/'build'/record['geometry_id'],image,depth,K)
        result.update(scene_id=sid,version=record['version'],robot_id=rid,source_frame_number=header['frame_number'],valid_until=started+4-header.get('source_age_ms',0)/1000)
        if time.time()>result['valid_until']:result.update(status='lost',reason='localization input expired during processing')
        with self.lock:self.localizations[(sid,rid)]=result
        return result


def install_scenes(app,root,fleet,receiver):
    service=SceneService(root,fleet,receiver,os.environ.get('FLEET_LOCAL_URL','https://127.0.0.1:8000'))
    def guarded(fn,*args):
        try:return fn(*args)
        except FileNotFoundError as error:raise HTTPException(404,str(error))
        except (ValueError,KeyError) as error:raise HTTPException(422,str(error))
    async def payload(request):
        try:body=await request.json()
        except ValueError:raise HTTPException(422,'invalid JSON request')
        if not isinstance(body,dict):raise HTTPException(422,'request body must be an object')
        return body
    @app.get('/api/scenes')
    def scenes():return {'scenes':[service.refresh(r['id']) for r in service.catalog.list()],'storage':service.catalog.storage()}
    @app.post('/api/scenes',status_code=201)
    async def create(request:Request):
        body=await payload(request);guarded(fleet.robot,body.get('robot_id',''));return guarded(service.catalog.create,body.get('name'),body['robot_id'])
    @app.get('/api/scenes/{sid}')
    def scene(sid:str):return guarded(service.refresh,sid)
    @app.get('/api/scenes/{sid}/manifest')
    def manifest(sid:str):return guarded(service.catalog.manifest,sid)
    @app.get('/api/scenes/{sid}/versions/{version}/assets/{filename}')
    def tile(sid:str,version:str,filename:str):
        path=guarded(service.catalog.tile,sid,version,filename)
        return FileResponse(path,media_type='application/octet-stream',headers={'Content-Encoding':'gzip','Cache-Control':'public,max-age=31536000,immutable','ETag':'"'+version+'-'+filename+'"'})
    @app.post('/api/scenes/{sid}/jobs/{kind}',status_code=201)
    async def task(sid:str,kind:str,request:Request):
        record=guarded(service.catalog.get,sid);body=await payload(request)
        if kind not in {'scene_capture','scene_build'}:raise HTTPException(422,'未知场景作业')
        return guarded(fleet.create_job,{'robot_id':record['robot_id'],'kind':kind,'title':record['name']+(' · 同步采集' if kind=='scene_capture' else ' · 高斯建图'),'duration_s':body.get('duration_s',120),'scene_id':sid})
    @app.post('/api/scenes/{sid}/localize/{rid}')
    def localize(sid:str,rid:str):return guarded(service.localize,sid,rid)
    @app.get('/api/scenes/{sid}/localization/{rid}')
    def localization(sid:str,rid:str):
        value=service.localizations.get((sid,rid))
        record=guarded(service.catalog.get,sid)
        if value is None or value.get('version')!=record.get('version') or time.time()>value.get('valid_until',0):return {'status':'lost','reason':'定位结果缺失或已过期','navigation_ready':False,'robot_id':rid,'scene_id':sid}
        return value
    @app.get('/api/rgbd/frame')
    def rgbd():
        summary,packet=receiver.snapshot()
        if not summary.get('online') or packet is None:raise HTTPException(503,'paired RGB-D unavailable')
        return Response(packet_with_age(packet,summary.get('arrival_age_s',0)),media_type='application/octet-stream',headers={'Cache-Control':'no-store'})
    @app.on_event('startup')
    def start():receiver.start()
    @app.on_event('shutdown')
    def stop():
        for capture in list(service.captures.values()):capture.stop.set()
        receiver.stop()
    app.state.scene_service=service
    app.state.fleet_runner.scene_handler=service.job_tick
    return service
