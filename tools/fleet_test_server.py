"""Local-only browser test fixture. Never connects to real robots."""
import os,sys,tempfile,time,json,threading
from pathlib import Path
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from fastapi import FastAPI,Request
from fastapi.responses import FileResponse,JSONResponse
from fastapi.staticfiles import StaticFiles
from fleet_api import install_fleet

def fixture(tail):
 if tail=='state':return {'online':True,'hz':125,'battery':{'soc':82},'last_seen_s':time.time(),'power':{'voltage':28.1},'imu':{'rpy_deg':[0,0,0]}}
 if tail=='sessions':return {'sessions':[]}
 if tail=='schedule':return {'rules':[]}
 if tail=='depth/status':return {'online':True}
 if tail=='record/status':return {'active':False,'sample_count':0}
 if tail.startswith('lidar/cloud'):return {'online':True,'points':[],'frame_key':str(time.time()),'frame_id':'unilidar_lidar','age_s':0}
 if tail=='agent/motion-gate':return {'enabled':False}
 return {'online':False,'enabled':False,'points':[],'entries':[],'frames':[]}

class Remote(BaseHTTPRequestHandler):
 def do_GET(self):
  data=json.dumps(fixture(self.path.split('?',1)[0].removeprefix('/api/'))).encode();self.send_response(200);self.send_header('Content-Type','application/json');self.end_headers();self.wfile.write(data)
 def log_message(self,*args):pass
remote=ThreadingHTTPServer(('127.0.0.1',18882),Remote);threading.Thread(target=remote.serve_forever,daemon=True).start()
os.environ['FLEET_LOCAL_URL']='http://127.0.0.1:18881';temp=tempfile.TemporaryDirectory();app=FastAPI();web=Path(__file__).resolve().parents[1]/'web';store=install_fleet(app,Path(temp.name),lambda:fixture('state'))
store.save_robot({'id':'robot-02','name':'测试机器人 02','model':'Fixture Go2','endpoint':'http://127.0.0.1:18882'})
@app.get('/')
def index():return FileResponse(web/'index.html')
@app.api_route('/api/{tail:path}',methods=['GET','POST','DELETE'])
def legacy(tail:str):return fixture(tail)
app.mount('/static',StaticFiles(directory=web),name='static')
if __name__=='__main__':
 import uvicorn
 try:uvicorn.run(app,host='127.0.0.1',port=18881,log_level='warning')
 finally:remote.shutdown();remote.server_close();temp.cleanup()
