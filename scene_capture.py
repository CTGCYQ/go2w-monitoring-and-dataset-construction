"""Bounded synchronized RGB-D capture; recorded source times are never rewritten."""
import argparse,ast,json,os,threading,time,uuid,zlib
from pathlib import Path
from depth_stream import DepthReceiver,split_rgbd_packet,packet_with_age


def atomic_json(path,value):
    import tempfile
    path=Path(path)
    descriptor,temporary=tempfile.mkstemp(prefix=path.name+'.',suffix='.tmp',dir=path.parent)
    try:
        with os.fdopen(descriptor,'w',encoding='utf8') as out:
            json.dump(value,out,ensure_ascii=False,allow_nan=False)
            out.flush();os.fsync(out.fileno())
        os.replace(temporary,path)
    finally:
        if os.path.exists(temporary):os.unlink(temporary)


class SceneCapture:
    def __init__(self,root,provider,robot_id,seconds=180,interval=.4,max_bytes=256*1024*1024,max_frames=1500):
        self.root=Path(root);self.provider=provider;self.robot_id=robot_id
        self.seconds=max(1,min(600,float(seconds)));self.interval=max(.2,float(interval));self.max_bytes=max_bytes;self.max_frames=max_frames
        self.stop=threading.Event();self.status={'status':'created','robot_id':robot_id,'frame_count':0,'bytes':0,'errors':0}
    def run(self):
        self.root.mkdir(parents=True,exist_ok=False)
        for name in ['rgb','depth']: (self.root/name).mkdir()
        start=time.monotonic();last=None;seen=set();self.status.update(status='recording',started_at=time.time())
        atomic_json(self.root/'capture.json',self.status)
        try:
            with (self.root/'frames.jsonl').open('w',encoding='utf8') as manifest:
                while time.monotonic()-start<self.seconds and not self.stop.is_set():
                    if (self.root/'stop.request').exists():break
                    tick=time.monotonic()
                    summary,packet=self.provider()
                    if not summary.get('online') or packet is None:
                        self.status['errors']+=1
                        if self.status['errors']>100:raise ValueError('RGB-D unavailable repeatedly')
                        self.stop.wait(self.interval);continue
                    header,depth,rgb=split_rgbd_packet(packet)
                    key=(header['frame_number'],header['captured_host_time_s'])
                    if key in seen:self.stop.wait(self.interval);continue
                    if last is not None and header['captured_host_time_s']<=last:raise ValueError('capture clock regressed')
                    packed=zlib.compress(depth,3);size=len(rgb)+len(packed)
                    if self.status['bytes']+size>self.max_bytes or self.status['frame_count']>=self.max_frames:
                        self.status['limit_reached']=True;break
                    i=self.status['frame_count'];stem=f'{i:06d}'
                    (self.root/'rgb'/f'{stem}.jpg').write_bytes(rgb)
                    (self.root/'depth'/f'{stem}.z16.z').write_bytes(packed)
                    record={'index':i,'robot_id':self.robot_id,'header':header,'rgb':f'rgb/{stem}.jpg','depth':f'depth/{stem}.z16.z','received_host_time_s':time.time()}
                    manifest.write(json.dumps(record,allow_nan=False)+'\n');manifest.flush()
                    seen.add(key);last=header['captured_host_time_s'];self.status['frame_count']+=1;self.status['bytes']+=size
                    self.status['last_frame_at']=time.time();atomic_json(self.root/'capture.json',self.status)
                    self.stop.wait(max(0,self.interval-(time.monotonic()-tick)))
            if self.status['frame_count']<3:raise ValueError('insufficient synchronized frames')
            self.status['status']='captured'
        except Exception as error:
            self.status.update(status='failed',error=str(error));raise
        finally:
            self.status.update(finished_at=time.time(),elapsed_s=round(time.monotonic()-start,3));atomic_json(self.root/'capture.json',self.status)
        return self.status


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True);parser.add_argument('--seconds',type=float,default=180);parser.add_argument('--robot-id',default='go2w-01');parser.add_argument('--dock-config',default=str(Path(__file__).with_name('server.py')))
    args=parser.parse_args();settings={}
    for node in ast.parse(Path(args.dock_config).read_text(encoding='utf8')).body:
        if isinstance(node,ast.Assign):
            for target in node.targets:
                if isinstance(target,ast.Name) and target.id in {'DOCK_HOST','DOCK_USER','DOCK_PASS'}:settings[target.id]=ast.literal_eval(node.value)
    receiver=DepthReceiver(settings['DOCK_HOST'],settings['DOCK_USER'],settings['DOCK_PASS'],request=b'R');receiver.start()
    try:
        deadline=time.monotonic()+15
        while not receiver.snapshot()[0].get('online'):
            if time.monotonic()>deadline:raise RuntimeError('paired RGB-D stream startup timeout')
            time.sleep(.2)
        def provider():
            summary,packet=receiver.snapshot()
            return summary,packet_with_age(packet,summary.get('arrival_age_s',0)) if packet else None
        print('CAPTURE_READY',flush=True)
        capture=SceneCapture(args.output,provider,args.robot_id,args.seconds)
        print(json.dumps(capture.run()),flush=True)
    finally:receiver.stop()

if __name__=='__main__':main()
