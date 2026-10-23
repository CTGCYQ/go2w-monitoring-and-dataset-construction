"""Robot routing, persistence and job side-effect ownership regressions."""
import json,tempfile,threading,time,unittest
from pathlib import Path
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from fleet_store import FleetStore,Conflict,Missing,job_metrics
from fleet_jobs import JobRunner

class StoreTests(unittest.TestCase):
 def setUp(self):self.tmp=tempfile.TemporaryDirectory();self.path=Path(self.tmp.name)/'fleet.db';self.s=FleetStore(self.path)
 def tearDown(self):self.tmp.cleanup()
 def robot(self,rid='second'):return self.s.save_robot({'id':rid,'name':'Second','model':'Go2','endpoint':'https://other:8000'})
 def job(self,rid='go2w-01',kind='sensor_check'):return self.s.create_job({'robot_id':rid,'kind':kind,'title':'Inspection','duration_s':5})
 def test_registry_persists_and_local_endpoint_cannot_change(self):
  self.robot();self.assertEqual(FleetStore(self.path).robot('second')['name'],'Second')
  with self.assertRaises(ValueError):self.s.save_robot({'endpoint':'https://bad'},'go2w-01')
 def test_bad_registration_and_duplicate_endpoint(self):
  for endpoint in ['file:///etc/passwd','https://user:pass@host','http://host/api','http://host:bad']:
   with self.assertRaises(ValueError):self.s.save_robot({'id':'x','name':'X','model':'M','endpoint':endpoint})
  self.robot()
  with self.assertRaises(Conflict):self.s.save_robot({'id':'third','name':'X','model':'M','endpoint':'https://other:8000'})
 def test_unknown_and_disabled_robot_cannot_receive_jobs(self):
  with self.assertRaises(Missing):self.job('missing')
  self.robot();self.s.save_robot({'enabled':False},'second')
  with self.assertRaises(Conflict):self.job('second')
 def test_per_robot_queue_is_serial_and_independent(self):
  self.robot();a=self.job();b=self.job();c=self.job('second');self.assertEqual(self.s.claim('go2w-01')['id'],a['id']);self.assertIsNone(self.s.claim('go2w-01'));self.assertEqual(self.s.claim('second')['id'],c['id'])
  with self.assertRaises(Conflict):self.s.save_robot({'name':'new'},'go2w-01')
 def test_restart_requires_review_instead_of_duplicate_start(self):
  j=self.job();self.s.claim('go2w-01');self.s.recover_interrupted();self.assertEqual(self.s.job(j['id'])['status'],'needs_attention');self.assertIsNone(self.s.claim('go2w-01'))
 def test_metrics_denominator_excludes_cancel_from_success(self):
  values=[{'status':'succeeded','started_at':1},{'status':'failed','started_at':2},{'status':'canceled','started_at':None}];m=job_metrics(values);self.assertEqual(m['execution_rate'],2/3);self.assertEqual(m['success_rate'],.5);self.assertIsNone(job_metrics([])['success_rate'])
 def test_inspection_requires_real_sensor_status(self):
  calls=[]
  def transport(rid,path,method):calls.append((rid,path,method));return {'online':True}
  runner=JobRunner(self.s,transport);j=self.job()
  for _ in range(4):runner.tick('go2w-01')
  self.assertEqual(self.s.job(j['id'])['status'],'succeeded');self.assertTrue(all(c[0]=='go2w-01' and c[2]=='GET' for c in calls))
 def test_offline_is_failure_and_never_fake_progress(self):
  runner=JobRunner(self.s,lambda *a:{'online':False});j=self.job();runner.tick('go2w-01');self.assertEqual(self.s.job(j['id'])['status'],'failed');self.assertEqual(self.s.job(j['id'])['progress'],0)
 def test_capture_starts_stops_and_checks_samples(self):
  rec={'active':False,'session_id':None,'sample_count':0};calls=[]
  def transport(rid,path,method):
   calls.append(path)
   if path=='/api/state':return {'online':True}
   if path.startswith('/api/record/start'):rec.update(active=True,session_id='owned',sample_count=20);return {'status':'recording','session_id':'owned'}
   if path=='/api/record/stop':rec['active']=False
   return dict(rec)
  runner=JobRunner(self.s,transport);j=self.job(kind='capture');runner.tick('go2w-01');runner.tick('go2w-01');self.s.mutate(j['id'],lambda v:v.update(record_started_at=time.time()-6));runner.tick('go2w-01');runner.tick('go2w-01');self.assertEqual(self.s.job(j['id'])['status'],'succeeded');self.assertEqual(calls.count('/api/record/stop'),1)
 def test_delayed_record_status_does_not_fail_confirmed_start(self):
  calls=[0];rec={'active':False,'session_id':None}
  def transport(rid,path,method):
   if path=='/api/state':return {'online':True}
   if path.startswith('/api/record/start'):rec['session_id']='owned';return {'session_id':'owned','status':'recording'}
   calls[0]+=1
   if calls[0]>=3:rec['active']=True
   return dict(rec)
  runner=JobRunner(self.s,transport);j=self.job(kind='capture');runner.tick('go2w-01');runner.tick('go2w-01');self.assertEqual(self.s.job(j['id'])['step_index'],2);self.assertEqual(self.s.job(j['id'])['session_id'],'owned')
 def test_preexisting_recording_is_not_stopped_or_claimed(self):
  calls=[]
  def transport(rid,path,method):calls.append((path,method));return {'online':True,'active':True,'session_id':'someone-else'}
  runner=JobRunner(self.s,transport);j=self.job(kind='capture');runner.tick('go2w-01');runner.tick('go2w-01');self.assertEqual(self.s.job(j['id'])['status'],'failed');self.assertFalse(any(method=='POST' for _,method in calls))
 def test_cancel_never_stops_another_session(self):
  calls=[]
  def transport(rid,path,method):calls.append(path);return {'active':True,'session_id':'someone-else'}
  runner=JobRunner(self.s,transport);j=self.job(kind='capture');self.s.claim('go2w-01');self.s.mutate(j['id'],lambda v:v.update(session_id='owned'));self.s.cancel(j['id']);runner.tick('go2w-01');self.assertNotIn('/api/record/stop',calls);self.assertEqual(self.s.job(j['id'])['status'],'needs_attention')
 def test_unknown_start_result_cannot_be_silently_canceled(self):
  runner=JobRunner(self.s,lambda *a:{'active':True,'session_id':'unknown'});j=self.job(kind='capture');self.s.claim('go2w-01');self.s.mutate(j['id'],lambda v:v.update(record_start_requested_at=time.time()));self.s.cancel(j['id']);runner.tick('go2w-01');self.assertEqual(self.s.job(j['id'])['status'],'needs_attention')

class GatewayTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.received=[]
  class Handler(BaseHTTPRequestHandler):
   def do_GET(self):self.respond()
   def do_POST(self):self.respond()
   def respond(self):
    body=self.rfile.read(int(self.headers.get('Content-Length','0')));cls.received.append((self.path,self.command,body));data=json.dumps({'source':'remote','path':self.path,'online':True}).encode();self.send_response(200);self.send_header('Content-Type','application/json');self.end_headers();self.wfile.write(data)
   def log_message(self,*a):pass
  cls.remote=ThreadingHTTPServer(('127.0.0.1',0),Handler);cls.thread=threading.Thread(target=cls.remote.serve_forever,daemon=True);cls.thread.start()
 @classmethod
 def tearDownClass(cls):cls.remote.shutdown();cls.remote.server_close();cls.thread.join()
 def setUp(self):
  from fastapi import FastAPI,Request
  from fastapi.testclient import TestClient
  from fleet_api import install_fleet
  self.tmp=tempfile.TemporaryDirectory();app=FastAPI()
  @app.get('/api/state')
  def state():return {'source':'local','online':True}
  @app.post('/api/record/start')
  def start():return {'source':'local'}
  self.store=install_fleet(app,Path(self.tmp.name),lambda:{'online':True});self.client=TestClient(app);self.store.save_robot({'id':'remote','name':'Remote','model':'Go2','endpoint':f'http://127.0.0.1:{self.remote.server_port}'})
 def tearDown(self):self.client.close();self.tmp.cleanup()
 def test_local_and_remote_routes_keep_identity(self):
  local=self.client.get('/api/robots/go2w-01/state');remote=self.client.get('/api/robots/remote/state');self.assertEqual(local.json()['source'],'local');self.assertEqual(remote.json()['source'],'remote');self.assertEqual(remote.headers['x-robot-id'],'remote')
 def test_unknown_or_disabled_never_falls_back_to_local(self):
  self.assertEqual(self.client.get('/api/robots/unknown/state').status_code,404);self.store.save_robot({'enabled':False},'remote');self.assertEqual(self.client.post('/api/robots/remote/record/start').status_code,409)
 def test_remote_mutations_preserve_method_body_query(self):
  response=self.client.post('/api/robots/remote/session/s/annotations?tag=a',json={'text':'robot scoped'});self.assertEqual(response.status_code,200);path,method,body=self.received[-1];self.assertEqual(path,'/api/session/s/annotations?tag=a');self.assertEqual(method,'POST');self.assertEqual(json.loads(body)['text'],'robot scoped')
 def test_invalid_api_registration_is_validation_error(self):
  for endpoint in ['http://host:bad','http://testserver:80','https://user:pass@host']:
   response=self.client.post('/api/fleet/robots',json={'id':'x','name':'x','model':'m','endpoint':endpoint});self.assertIn(response.status_code,[409,422])
 def test_job_filter_and_cancel_api(self):
  j=self.client.post('/api/fleet/jobs',json={'robot_id':'remote','kind':'sensor_check','title':'Remote check'});self.assertEqual(j.status_code,201);self.assertEqual(self.client.get('/api/fleet/jobs?robot_id=go2w-01').json()['jobs'],[]);self.assertEqual(self.client.post('/api/fleet/jobs/'+j.json()['id']+'/cancel').json()['status'],'canceled')
if __name__=='__main__':unittest.main()
