"""Continuous live browser/API/process soak. Stores aggregate metrics, no sensor frames.
Requires GO2W_SSH_PASSWORD; defaults to a 30 minute acceptance run.
"""
import argparse,json,os,shlex,ssl,statistics,time,urllib.request,urllib.error
from pathlib import Path
import paramiko

def main():
 p=argparse.ArgumentParser();p.add_argument('--host',default='192.168.1.80');p.add_argument('--seconds',type=float,default=1800);p.add_argument('--interval',type=float,default=10);p.add_argument('--output',required=True);a=p.parse_args()
 ctx=ssl._create_unverified_context();base=f'https://{a.host}:8000';out=Path(a.output);rows=[];errors=[];http_errors=[];gc_metrics=[];last_key=None;advances=0;first_pids=None;last_print=0
 ssh=paramiko.SSHClient();ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy());ssh.connect(a.host,username='dell',password=os.environ['GO2W_SSH_PASSWORD'],timeout=10)
 def get(path):
  start=time.monotonic()
  with urllib.request.urlopen(base+path,context=ctx,timeout=5) as r:value=json.load(r)
  return value,time.monotonic()-start
 def resources(pids):
  code="import os,json;from pathlib import Path;result={};pids="+repr(pids)+"\nfor name,pid in pids.items():\n try:\n  raw=Path(f'/proc/{pid}/status').read_text();fields=dict(line.split(':',1) for line in raw.splitlines() if ':' in line);stat=Path(f'/proc/{pid}/stat').read_text().split(') ',1)[1].split();result[name]={'pid':pid,'rss_kib':int(fields['VmRSS'].split()[0]),'threads':int(fields['Threads']),'fds':len(os.listdir(f'/proc/{pid}/fd')),'cpu_s':(int(stat[11])+int(stat[12]))/os.sysconf('SC_CLK_TCK')}\n except (OSError,KeyError,ValueError):result[name]={'pid':pid,'missing':True}\nprint(json.dumps(result))"
  i,o,e=ssh.exec_command('python3 -c '+shlex.quote(code));text=o.read().decode();assert o.channel.recv_exit_status()==0,e.read().decode();return json.loads(text)
 from playwright.sync_api import sync_playwright
 with sync_playwright() as pw:
  browser=pw.chromium.launch(executable_path=r'C:\Program Files\Google\Chrome\Application\chrome.exe',headless=True,args=['--enable-unsafe-swiftshader','--enable-precise-memory-info'])
  page=browser.new_page(ignore_https_errors=True,viewport={'width':1400,'height':1000});page.on('pageerror',lambda e:errors.append(str(e)))
  page.on('console',lambda m:errors.append(m.text) if m.type=='error' and ('WebGL' in m.text or 'shader' in m.text.lower()) else None)
  page.goto(base,wait_until='domcontentloaded');page.locator('.tab[data-tab="vision"]').click();page.locator('[data-pc-mode="aggregate"]').click();page.wait_for_function('pcStudio.buffer.frames.length>=5',timeout=20000)
  cdp=page.context.new_cdp_session(page);cdp.send('HeapProfiler.enable');cdp.send('HeapProfiler.collectGarbage')
  started=time.monotonic();last_gc=-10000.;next_sample=started
  print('SOAK_STARTED '+json.dumps({'seconds':a.seconds}),flush=True)
  while time.monotonic()-started<a.seconds:
   if Path(str(out)+'.stop').exists():break
   now=time.monotonic();elapsed=now-started
   if now<next_sample:page.wait_for_timeout(min(1000,(next_sample-now)*1000));continue
   next_sample=now+a.interval;row={'elapsed_s':round(elapsed,3),'time_utc':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime())}
   try:
    h,hl=get('/api/system/health');cloud,cl=get('/api/lidar/cloud?max_points=100');m,ml=get('/api/map?max_points=1000');depth,dl=get('/api/depth/metric')
    row.update(healthy=bool(h.get('online') and cloud.get('online') and m.get('online') and not m.get('fault') and depth.get('online')),latencies_s=[hl,cl,ml,dl],cloud_age_s=cloud.get('age_s'),map_cells=h.get('map_cell_count'),map_cloud_frames=m.get('cloud_count'),map_max_displacement_m=m.get('max_displacement_m'),process_restarts=h['process_restarts'],mapping_epochs=h['mapping_epochs'])
    key=cloud.get('frame_key');advances+=int(last_key is not None and key!=last_key);last_key=key
    row['processes']=resources(h['services'])
    if first_pids is None:first_pids=h['services']
    row['same_processes']=h['services']==first_pids
    row['browser']=page.evaluate('({heap_bytes:performance.memory?.usedJSHeapSize||0,points:pcStudio.geometry.drawRange.count,voxels:pcStudio.buffer.cells.size,position_capacity:pcStudio.geometry.attributes.position.array.length,geometries:pcStudio.renderer.info.memory.geometries,textures:pcStudio.renderer.info.memory.textures,mode:pcStudio.buffer.mode})')
    if elapsed-last_gc>=300:
     cdp.send('HeapProfiler.collectGarbage');gc_metrics.append({'elapsed_s':round(elapsed,3),'heap_bytes':page.evaluate('performance.memory.usedJSHeapSize')});last_gc=elapsed
     modes=['aggregate','map','frame'];mode=modes[(len(gc_metrics)-1)%3];page.locator(f'[data-pc-mode="{mode}"]').click();page.wait_for_function('pcStudio.geometry.drawRange.count>50',timeout=15000)
   except Exception as e:
    row['healthy']=False;row['error']=str(e);http_errors.append({'elapsed_s':elapsed,'error':str(e)})
   rows.append(row)
   progress={'elapsed_s':round(time.monotonic()-started,2),'target_s':a.seconds,'samples':len(rows),'healthy_samples':sum(bool(r.get('healthy')) for r in rows),'last':row,'browser_errors':errors}
   Path(str(out)+'.progress').write_text(json.dumps(progress,indent=2))
   if elapsed-last_print>=60 or len(rows)==1:print('SOAK_PROGRESS '+json.dumps({'elapsed_s':round(elapsed),'samples':len(rows),'healthy':progress['healthy_samples'],'process_rss_mib':{k:round(v.get('rss_kib',0)/1024,1) for k,v in row.get('processes',{}).items()},'browser_heap_mib':round(row.get('browser',{}).get('heap_bytes',0)/1048576,1)}),flush=True);last_print=elapsed
  cdp.send('HeapProfiler.collectGarbage');gc_metrics.append({'elapsed_s':round(time.monotonic()-started,3),'heap_bytes':page.evaluate('performance.memory.usedJSHeapSize')})
  if time.monotonic()-started<a.seconds-1:http_errors.append({'error':'acceptance interrupted before requested duration'})
  health_ratio=sum(bool(r.get('healthy')) for r in rows)/max(1,len(rows));reasons=[]
  if health_ratio<.99:reasons.append('sensor/API availability below 99 percent')
  if http_errors:reasons.append('HTTP/resource sampling errors')
  if errors:reasons.append('browser/shader errors')
  if any(r.get('map_cells',0)>120000 for r in rows if r.get('map_cells') is not None):reasons.append('server voxel bound violated')
  if not all(r.get('same_processes') for r in rows):reasons.append('unexpected process replacement')
  if any(r.get('browser',{}).get('voxels',0)>120000 or r.get('browser',{}).get('position_capacity',0)!=360000 for r in rows):reasons.append('browser point buffer bound violated')
  if gc_metrics[-1]['heap_bytes']-gc_metrics[0]['heap_bytes']>64*1048576:reasons.append('retained browser heap grew over 64 MiB')
  if advances<.95*max(0,len(rows)-1):reasons.append('source frames failed to advance')
  summaries={}
  for name in first_pids or {}:
   samples=[r['processes'][name] for r in rows if name in r.get('processes',{}) and not r['processes'][name].get('missing')]
   if not samples:reasons.append('missing process resource samples: '+name);continue
   first,last=samples[0],samples[-1];growth=last['rss_kib']-first['rss_kib'];fd_growth=last['fds']-first['fds']
   summaries[name]={'first_rss_mib':round(first['rss_kib']/1024,2),'last_rss_mib':round(last['rss_kib']/1024,2),'peak_rss_mib':round(max(v['rss_kib'] for v in samples)/1024,2),'rss_growth_mib':round(growth/1024,2),'first_fds':first['fds'],'last_fds':last['fds'],'peak_fds':max(v['fds'] for v in samples),'average_cpu_percent':round((last['cpu_s']-first['cpu_s'])/max(1,time.monotonic()-started)*100,2),'peak_threads':max(v['threads'] for v in samples)}
   if growth>256*1024:reasons.append('process RSS growth over 256 MiB: '+name)
   if fd_growth>32:reasons.append('file descriptors grew over 32: '+name)
  geometries=[r['browser']['geometries'] for r in rows if 'browser' in r];textures=[r['browser']['textures'] for r in rows if 'browser' in r]
  if geometries and max(geometries)-min(geometries)>4:reasons.append('WebGL geometry count grew')
  if textures and max(textures)-min(textures)>4:reasons.append('WebGL texture count grew')
  report={'passed':not reasons,'failure_reasons':reasons,'duration_s':round(time.monotonic()-started,3),'sample_count':len(rows),'healthy_ratio':health_ratio,'source_frame_advances':advances,'browser_errors':errors,'http_errors':http_errors,'browser_gc_heap':gc_metrics,'webgl_geometry_range':[min(geometries),max(geometries)] if geometries else None,'webgl_texture_range':[min(textures),max(textures)] if textures else None,'process_resources':summaries,'samples':rows,'raw_sensor_data_saved':False,'robot_motion_commands':0}
  out.write_text(json.dumps(report,indent=2)+'\n');print('SOAK_RESULT '+json.dumps({k:v for k,v in report.items() if k!='samples'}),flush=True);browser.close()
 ssh.close()
 if not report['passed']:raise SystemExit(1)
if __name__=='__main__':main()
