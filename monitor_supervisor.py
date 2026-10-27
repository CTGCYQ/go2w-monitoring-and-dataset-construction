"""Recover only monitor/telemetry/localization services, never robot motion.

Reconnection starts a NEW estimator/map epoch. Unexplained geometry/frame faults
remain latched for review. Existing healthy tmux services are adopted.
"""
import argparse,json,logging,math,os,signal,subprocess,time
from pathlib import Path
from logging.handlers import RotatingFileHandler
from recovery_policy import LinkGate,Backoff,wait_for_process
from sensor_freshness import cloud_is_fresh

def read_json(path):
    try:
        value=json.loads(Path(path).read_text());return value if isinstance(value,dict) else {}
    except (OSError,ValueError):return {}

def process_matches(pid,name):
    try:return any(Path(arg.decode()).name==name for arg in Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0') if arg)
    except (OSError,UnicodeError):return False

def child_process(pid,name):
    pending=[pid];seen=set()
    while pending and len(seen)<100:
        current=pending.pop()
        if current in seen:continue
        seen.add(current)
        if process_matches(current,name):return current
        try:pending.extend(int(v) for v in Path(f'/proc/{current}/task/{current}/children').read_text().split())
        except (OSError,ValueError):pass
    return None

def run(args,timeout=15):return subprocess.run(args,capture_output=True,text=True,timeout=timeout)

def session_pid(session,name):
    result=run(['tmux','list-panes','-t',session,'-F','#{pane_pid}'])
    if result.returncode:return None
    for line in result.stdout.splitlines():
        if line.isdigit():
            found=child_process(int(line),name)
            if found:return found
    return None

def atomic_json(path,value):
    tmp=path.with_suffix('.json.tmp');tmp.write_text(json.dumps(value,allow_nan=False));os.replace(tmp,path)

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--root',default=str(Path(__file__).resolve().parent));parser.add_argument('--vla-root',default='/home/dell/go2w_vla');a=parser.parse_args()
    root=Path(a.root).resolve();vla=Path(a.vla_root).resolve();logs=root/'logs';logs.mkdir(exist_ok=True)
    import fcntl
    lock=(root/'.supervisor.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    logger=logging.getLogger('recovery');logger.setLevel(logging.INFO);logger.addHandler(RotatingFileHandler(logs/'supervisor.log',maxBytes=2*1024*1024,backupCount=2));logger.addHandler(logging.StreamHandler())
    for handler in logger.handlers:handler.setFormatter(logging.Formatter('%(asctime)s %(levelname)s %(message)s'))
    # SDK and web dependencies are installed in the existing VLA virtual environment.
    python=str(vla/'.venv/bin/python');specs={
      'go2wweb':('server.py',[python,str(root/'server.py')]),
      'pcworker':('pc_worker.py',['bash',str(root/'run_pc_worker.sh')]),
      'imuadapter':('lidar_imu_adapter.py',['bash',str(root/'run_imu_adapter.sh')]),
      'go2wcol':('collector.py',[python,str(root/'collector.py'),'--interface','enp0s31f6']),
      'frontcam':('front_cam_worker.py',[python,str(root/'front_cam_worker.py'),'--interface','enp0s31f6','--fps','10'])}
    import shlex
    stop=False
    def stopping(*_):
        nonlocal stop;stop=True
    signal.signal(signal.SIGTERM,stopping);signal.signal(signal.SIGINT,stopping)
    retries={name:Backoff() for name in specs};restarts={name:0 for name in specs};map_retry=Backoff();gate=LinkGate();started=time.monotonic();last_web_check=0.;web_failures=0;mapping_expected=None;epochs=0;offline_events=0;last_adapter=None;last_lio=None;adapter_epoch=0;healthy_since={};observed_pids={}
    def lio_pid():
        try:
            pid=int((vla/'logs/point_lio.pid').read_text());return pid if process_matches(pid,'pointlio_mapping') else None
        except (OSError,ValueError):return None
    def stop_mapping():
        if lio_pid():run(['bash',str(vla/'deploy/point_lio/stop_point_lio.sh')],timeout=20)
        run(['tmux','kill-session','-t','mapworker']);logger.info('localization stopped; historical map retained')
    def ensure(name,force=False):
        spec,command=specs[name];pid=session_pid(name,spec)
        if pid and not force:return pid
        now=time.monotonic()
        if not retries[name].ready(now):return pid
        run(['tmux','kill-session','-t',name])
        logfile=logs/f'{name}.log';shell=f'cd {shlex.quote(str(root))} && exec '+shlex.join(command)+f' >> {shlex.quote(str(logfile))} 2>&1'
        result=run(['tmux','new-session','-d','-s',name,shell]);retries[name].fail(now);restarts[name]+=1
        logger.info('restart %s attempt=%s launch_rc=%s',name,restarts[name],result.returncode)
        return session_pid(name,spec)
    while not stop:
        try:
            now=time.monotonic();pids={name:ensure(name) for name in specs}
            for name,pid in pids.items():
                if pid:
                    if observed_pids.get(name)!=pid:healthy_since[name]=now;observed_pids[name]=pid
                    if now-healthy_since.get(name,now)>=10:retries[name].success()
                else:healthy_since.pop(name,None);observed_pids.pop(name,None)
            # Detect a web process that exists but no longer serves requests.
            if now-last_web_check>=10:
                import urllib.request,ssl
                try:
                    with urllib.request.urlopen('https://127.0.0.1:8000/api/lidar/cloud?max_points=100',context=ssl._create_unverified_context(),timeout=3) as response:assert response.status==200
                    web_failures=0
                except Exception:web_failures+=1
                if web_failures>=3:pids['go2wweb']=ensure('go2wweb',force=True);web_failures=0
                last_web_check=now
            cache=read_json(root/'pc_cache.json');fresh=cloud_is_fresh(cache);ready=gate.observe(now,fresh,cache.get('frame_key'))
            pid=lio_pid();map_pid=session_pid('mapworker','map_worker.py');map_cache=read_json(root/'map_cache.json')
            if mapping_expected is None:
                mapping_expected=bool(pid or map_pid);last_adapter=pids['imuadapter'];last_lio=pid;adapter_epoch=restarts['imuadapter']
            if gate.offline(now) and mapping_expected:
                stop_mapping();mapping_expected=False;offline_events+=1;map_retry.success()
            adapter_changed=restarts['imuadapter']!=adapter_epoch
            lio_changed=last_lio is not None and pid is not None and pid!=last_lio
            needs_epoch=not mapping_expected or not pid or adapter_changed or lio_changed
            if ready and pids['imuadapter'] and needs_epoch and map_retry.ready(now):
                stop_mapping();result=run(['bash',str(root/'run_mapping_stack.sh')],timeout=30);pid=lio_pid();map_pid=session_pid('mapworker','map_worker.py')
                # tmux launch returns before bash sources ROS and execs Python.
                # Wait for the actual child, rather than mistaking startup for failure.
                if result.returncode==0 and pid and not map_pid:
                    map_pid=wait_for_process(lambda:session_pid('mapworker','map_worker.py'))
                if result.returncode==0 and pid and map_pid:
                    mapping_expected=True;adapter_epoch=restarts['imuadapter'];epochs+=1;map_retry.success();
                    if not pids['frontcam']:retries['frontcam'].success()
                    logger.info('new mapping epoch=%s after confirmed fresh frames',epochs)
                else:map_retry.fail(now);logger.error('mapping startup failed rc=%s; retry delayed',result.returncode)
            elif ready and mapping_expected and pid and not map_pid and map_retry.ready(now):
                shell=f'exec bash {shlex.quote(str(root/"run_map_worker.sh"))} >> {shlex.quote(str(logs/"mapworker.log"))} 2>&1'
                run(['tmux','kill-session','-t','mapworker']);result=run(['tmux','new-session','-d','-s','mapworker',shell]);map_retry.fail(now);logger.info('map process restarted rc=%s',result.returncode)
            if map_pid and pid and mapping_expected and not needs_epoch:map_retry.success()
            # Localization/time/point faults traced to the dock clock regression (or
            # robot re-seat on power-up) are self-recovering: reset the map epoch so
            # the world map re-anchors on stable odometry instead of latching
            # "unavailable" forever. Genuine geometry corruption stays latched.
            _recoverable={'pose_position_jump','pose_rotation_jump','pose_time_regression',
                          'pose_stream_discontinuity','pose_invalid','pose_frame_mismatch',
                          'point_time_invalid','cloud_time_regression','map_extent_exceeded',
                          'cloud_frame_mismatch'}
            if (map_pid and pid and mapping_expected and not needs_epoch
                    and map_cache.get('fault') and map_cache.get('reason_code') in _recoverable
                    and map_retry.ready(now)):
                shell=f'exec bash {shlex.quote(str(root/"run_map_worker.sh"))} >> {shlex.quote(str(logs/"mapworker.log"))} 2>&1'
                run(['tmux','kill-session','-t','mapworker']);result=run(['tmux','new-session','-d','-s','mapworker',shell])
                map_retry.fail(now);logger.info('map epoch reset after recoverable fault %s rc=%s',map_cache.get('reason_code'),result.returncode)
            last_adapter=pids['imuadapter'];last_lio=pid
            # Bounded existing external logs; copy only the last 1MiB before truncating
            # in place, so processes with an open descriptor continue on the same file.
            for path in [*logs.glob('*.log'),root/'server.log',root/'map_worker.log',vla/'logs/point_lio.log']:
                if path.exists() and path.stat().st_size>8*1024*1024:
                    with path.open('rb') as stream:stream.seek(-1024*1024,2);tail=stream.read()
                    path.with_suffix('.log.1').write_bytes(tail)
                    with path.open('r+b') as stream:stream.truncate(0)
            status={'written_at':time.time(),'uptime_s':round(time.monotonic()-started,2),'cloud_fresh':fresh,'link_stable':ready,'mapping_expected':mapping_expected,'mapping_epochs':epochs,'offline_events':offline_events,'process_restarts':restarts,'services':{**pids,'pointlio':pid,'mapworker':map_pid},'map_cell_count':map_cache.get('count',0),'map_fault':bool(map_cache.get('fault')),'map_reason':map_cache.get('reason_code'),'motion_commands_sent':0}
            atomic_json(root/'supervisor_status.json',status)
        except Exception:logger.exception('supervisor iteration failed; retrying without robot commands')
        time.sleep(1)

if __name__=='__main__':main()
