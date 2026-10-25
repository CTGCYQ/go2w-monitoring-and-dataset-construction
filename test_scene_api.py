import tempfile,time,unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from fastapi import FastAPI
from fastapi.testclient import TestClient
from fleet_store import FleetStore
from scene_api import install_scenes
from scene_geometry import match_pose
class Receiver:
 def snapshot(self):return {'online':False},None
 def start(self):pass
 def stop(self):pass
class SceneApiTests(unittest.TestCase):
 def setUp(self):
  self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);root=Path(self.temp.name);self.app=FastAPI();self.app.state.fleet_runner=SimpleNamespace();self.fleet=FleetStore(root/'fleet.db');self.service=install_scenes(self.app,root/'scenes',self.fleet,Receiver());self.client=TestClient(self.app);self.addCleanup(self.client.close);self.record=self.service.catalog.create('Map','go2w-01')
 def test_body_validation(self):
  for body in [[],None,5,'text']:
   self.assertEqual(self.client.post('/api/scenes',json=body).status_code,422)
 def test_offline_rgbd_is_unavailable(self):self.assertEqual(self.client.get('/api/rgbd/frame').status_code,503)
 def test_capture_job_has_real_scene_owner(self):
  r=self.client.post('/api/scenes/'+self.record['id']+'/jobs/scene_capture',json={'duration_s':20});self.assertEqual(r.status_code,201);self.assertEqual(r.json()['scene_id'],self.record['id']);self.assertEqual(r.json()['robot_id'],'go2w-01')
 def test_restart_reconciles_orphan_capture(self):
  self.service.catalog.update(self.record['id'],stage='capturing',capture_id='orphan');self.assertEqual(self.service.refresh(self.record['id'])['stage'],'interrupted')
 def test_map_version_invalidates_old_pose(self):
  self.service.catalog.update(self.record['id'],version='new');self.service.localizations[(self.record['id'],'go2w-01')]={'status':'localized','version':'old','valid_until':time.time()+30};r=self.client.get('/api/scenes/'+self.record['id']+'/localization/go2w-01');self.assertEqual(r.json()['status'],'lost')
 def test_unpublished_map_cannot_localize(self):self.assertEqual(self.client.post('/api/scenes/'+self.record['id']+'/localize/go2w-01').status_code,422)
class MetricPoseTests(unittest.TestCase):
 def test_recovers_known_rotation_and_translation(self):
  import cv2
  rng=np.random.default_rng(14);xyz=rng.uniform([-1,-.7,2],[1,.7,5],(120,3));K=np.array([[500.,0,320],[0,500.,240],[0,0,1]]);rvec=np.array([.02,-.12,.04]);R=cv2.Rodrigues(rvec)[0];t=np.array([.4,-.05,.02]);current=xyz@R.T+t;pixels=current[:,:2]/current[:,2:]*500+[320,240];descriptors=rng.normal(size=(120,128)).astype(np.float32);ref={'xyz':xyz,'descriptors':descriptors};now={'xyz':current,'pixels':pixels,'descriptors':descriptors.copy()};result=match_pose(ref,now,K);self.assertIsNotNone(result);np.testing.assert_allclose(result['T_current_reference'][:3,:3],R,atol=1e-5);np.testing.assert_allclose(result['T_current_reference'][:3,3],t,atol=1e-5)
 def test_depth_inconsistent_matches_rejected(self):
  import cv2
  rng=np.random.default_rng(1);xyz=rng.uniform([-1,-1,2],[1,1,5],(80,3));d=rng.normal(size=(80,128)).astype(np.float32);K=np.array([[500.,0,320],[0,500.,240],[0,0,1]]);pixels=xyz[:,:2]/xyz[:,2:]*500+[320,240];wrong=xyz.copy();wrong[:,2]+=1;self.assertIsNone(match_pose({'xyz':xyz,'descriptors':d},{'xyz':wrong,'pixels':pixels,'descriptors':d},K))
if __name__=='__main__':unittest.main()
