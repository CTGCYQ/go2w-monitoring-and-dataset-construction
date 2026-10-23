"""Observe a user-operated robot shutdown/reboot; never sends robot commands."""
import argparse,json,ssl,time,urllib.request,urllib.error
from pathlib import Path

def main():
 p=argparse.ArgumentParser();p.add_argument('--host',default='192.168.1.80');p.add_argument('--output',required=True);a=p.parse_args()
 ctx=ssl._create_unverified_context();base=f'https://{a.host}:8000';events=[];started=time.monotonic();phase='waiting_for_shutdown';offline=None;errors=[]
 def get(path):
  with urllib.request.urlopen(base+path,context=ctx,timeout=5) as r:return json.load(r)
 def image_status():
  try:
   with urllib.request.urlopen(base+'/api/image/latest',context=ctx,timeout=5) as r:return r.status
  except urllib.error.HTTPError as e:return e.code
 from playwright.sync_api import sync_playwright
 with sync_playwright() as pw:
  browser=pw.chromium.launch(executable_path=r'C:\Program Files\Google\Chrome\Application\chrome.exe',headless=True,args=['--enable-unsafe-swiftshader'])
  page=browser.new_page(ignore_https_errors=True);page.on('pageerror',lambda e:errors.append(str(e)))
  page.goto(base,wait_until='domcontentloaded');page.locator('.tab[data-tab="vision"]').click();page.wait_for_function('pcStudio.geometry.drawRange.count>50')
  identity=page.evaluate('window.reconnectIdentity=Math.random()');before=get('/api/lidar/cloud?max_points=100');key=before['frame_key'];old_lio=get('/api/system/health')['services']['pointlio']
  print('POWER_CYCLE_READY',flush=True)
  while time.monotonic()-started<900:
   health=get('/api/system/health');cloud=get('/api/lidar/cloud?max_points=100');state=get('/api/state');depth=get('/api/depth/status')
   if phase=='waiting_for_shutdown' and not cloud.get('online') and not state.get('online'):
    offline=time.monotonic();phase='verifying_offline'
   if phase=='verifying_offline' and time.monotonic()-offline>5:
    code=image_status();empty=page.evaluate('pcStudio.geometry.drawRange.count===0')
    if not health['services']['pointlio'] and not health['services']['mapworker'] and not depth['online'] and code==503 and empty:
     events.append({'phase':'offline_verified','elapsed_s':round(time.monotonic()-started,2),'front_http_status':code,'stale_points_hidden':True,'old_localization_stopped':True,'depth_online':False});phase='waiting_for_reboot';print('POWER_OFF_CONFIRMED '+json.dumps(events[-1]),flush=True)
   if phase=='waiting_for_reboot' and cloud.get('online') and state.get('online'):
    phase='verifying_recovery';online_at=time.monotonic()
   if phase=='verifying_recovery':
    m=get('/api/map?max_points=1000')
    if m.get('online') and m.get('cloud_count',0)>=10 and not m.get('fault') and depth['online'] and image_status()==200:
     page.wait_for_function('pcStudio.geometry.drawRange.count>50',timeout=10000)
     assert page.evaluate('window.reconnectIdentity')==identity
     assert cloud['frame_key']!=key and health['services']['pointlio']!=old_lio
     events.append({'phase':'recovered','elapsed_s':round(time.monotonic()-started,2),'recovery_from_first_cloud_s':round(time.monotonic()-online_at,2),'new_estimator_pid':health['services']['pointlio'],'front_online':True,'depth_online':True,'map_online':True,'page_reload_required':False})
     assert not errors,errors
     result={'passed':True,'events':events,'browser_errors':errors,'robot_motion_commands':0,'raw_sensor_data_saved':False};Path(a.output).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2),flush=True);browser.close();return
   Path(a.output+'.progress').write_text(json.dumps({'phase':phase,'elapsed_s':time.monotonic()-started,'events':events,'health':health}))
   page.wait_for_timeout(1000)
  raise AssertionError('robot power-cycle acceptance timed out')
if __name__=='__main__':main()
