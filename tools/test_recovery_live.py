"""Explicit process-fault acceptance on the configured monitor; never robot motion.
Requires GO2W_SSH_PASSWORD. Run only during an authorized maintenance window.
Images/point clouds are read in RAM, not recorded.
"""
import argparse,json,os,shlex,ssl,time,urllib.request,urllib.error
from pathlib import Path
import paramiko

ROOT='/mnt/85ee0fe8-b944-40f5-8474-40cc274f0cef/go2w-monitor'
def main():
    p=argparse.ArgumentParser();p.add_argument('--host',default='192.168.1.80');p.add_argument('--output',required=True);a=p.parse_args()
    context=ssl._create_unverified_context();base=f'https://{a.host}:8000';checks=[]
    ssh=paramiko.SSHClient();ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy());ssh.connect(a.host,username='dell',password=os.environ['GO2W_SSH_PASSWORD'],timeout=10)
    def get(path):
        with urllib.request.urlopen(base+path,context=context,timeout=6) as r:return json.load(r)
    def wait(predicate,timeout=90):
        start=time.monotonic();last=None
        while time.monotonic()-start<timeout:
            try:
                last=get('/api/system/health')
                if last.get('online') and predicate(last):return last,time.monotonic()-start
            except Exception:pass
            time.sleep(.5)
        raise AssertionError(f'recovery timeout; last health={last}')
    def terminate(service,pid,expected):
        code=f"import sys,os,signal;sys.path.insert(0,{ROOT!r});from monitor_supervisor import process_matches;assert process_matches({pid},{expected!r});os.kill({pid},signal.SIGTERM)"
        i,o,e=ssh.exec_command('python3 -c '+shlex.quote(code));err=e.read().decode();assert o.channel.recv_exit_status()==0,err
    from playwright.sync_api import sync_playwright
    with sync_playwright() as pw:
        browser=pw.chromium.launch(executable_path=r'C:\Program Files\Google\Chrome\Application\chrome.exe',headless=True,args=['--enable-unsafe-swiftshader'])
        page=browser.new_page(ignore_https_errors=True,viewport={'width':1300,'height':900});errors=[];page.on('pageerror',lambda e:errors.append(str(e)))
        page.goto(base,wait_until='domcontentloaded');page.locator('.tab[data-tab="vision"]').click();page.wait_for_function('pcStudio.geometry && pcStudio.geometry.drawRange.count>50')
        identity=page.evaluate('window.recoveryTestIdentity=Math.random()')
        for service,expected in [('go2wweb','server.py'),('pcworker','pc_worker.py'),('mapworker','map_worker.py'),('pointlio','pointlio_mapping'),('imuadapter','lidar_imu_adapter.py'),('frontcam','front_cam_worker.py'),('go2wcol','collector.py')]:
            before,_=wait(lambda h:all(h['services'].values()) and h['cloud_fresh'] and not h['map_fault'])
            old=before['services'][service];terminate(service,old,expected)
            after,elapsed=wait(lambda h:h['services'].get(service) not in (None,old) and all(h['services'].values()) and h['cloud_fresh'] and not h['map_fault'])
            # A restarted estimator/map must actually integrate data, not merely have a PID.
            deadline=time.monotonic()+45
            while True:
                m=get('/api/map?max_points=1000')
                if m.get('online') and not m.get('fault') and m.get('cloud_count',0)>=3:break
                assert time.monotonic()<deadline,'map failed to integrate after recovery';time.sleep(.5)
            page.wait_for_function('pcStudio.geometry.drawRange.count>50',timeout=45000)
            assert page.evaluate('window.recoveryTestIdentity')==identity,'page unexpectedly reloaded'
            checks.append({'service':service,'passed':True,'recovery_seconds':round(elapsed,3),'old_pid':old,'new_pid':after['services'][service],'page_reload_required':False})
            print(json.dumps(checks[-1]),flush=True)
        # The supervisor itself must be restarted by systemd without replacing healthy workers.
        before=get('/api/system/health');cmd='env XDG_RUNTIME_DIR=/run/user/1000 DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus systemctl --user show go2w-monitor.service -p MainPID --value'
        i,o,e=ssh.exec_command(cmd);pid=int(o.read());terminate('supervisor',pid,'monitor_supervisor.py')
        after,elapsed=wait(lambda h:h['uptime_s']<before['uptime_s'] and all(h['services'].values()))
        assert after['services']==before['services'],'supervisor restart duplicated/replaced healthy services'
        checks.append({'service':'supervisor','passed':True,'recovery_seconds':round(elapsed,3),'healthy_workers_adopted':True})
        assert not errors,errors
        report={'passed':True,'checks':checks,'browser_errors':errors,'robot_motion_commands':0,'raw_sensor_data_saved':False}
        Path(a.output).write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2));browser.close()
    ssh.close()

if __name__=='__main__':main()
