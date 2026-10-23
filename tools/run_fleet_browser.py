"""Own the fixture process directly so Windows cleanup closes the actual server."""
import os,socket,subprocess,sys,time,urllib.request
from pathlib import Path
root=Path(__file__).resolve().parents[1]
for port in [18881,18882]:
 with socket.socket() as probe:
  if probe.connect_ex(('127.0.0.1',port))==0:raise SystemExit(f'Fixture port {port} already in use; stop the previous fixture first.')
server=subprocess.Popen([sys.executable,str(root/'tools/fleet_test_server.py')],stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
try:
 deadline=time.monotonic()+45
 while True:
  if server.poll() is not None:raise RuntimeError(server.stderr.read().decode('utf8','replace'))
  try:
   with urllib.request.urlopen('http://127.0.0.1:18881/api/fleet/health',timeout=1) as r:break
  except OSError:
   if time.monotonic()>deadline:raise TimeoutError('Fixture startup timed out')
   time.sleep(.2)
 result=subprocess.run([sys.executable,str(root/'tools/test_fleet_browser.py')]);raise SystemExit(result.returncode)
finally:
 server.terminate()
 try:server.wait(timeout=8)
 except subprocess.TimeoutExpired:server.kill();server.wait(timeout=5)
 server.stderr.close()
