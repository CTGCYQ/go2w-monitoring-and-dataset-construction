import gzip,json,tempfile,unittest
from pathlib import Path
import numpy as np
from scene_store import SceneCatalog,publish_tiles,identifier,safe_path
class SceneStoreTests(unittest.TestCase):
 def setUp(self):
  self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.root=Path(self.temp.name);self.catalog=SceneCatalog(self.root/'assets');self.scene=self.catalog.create('Test map','robot-01');self.trained=self.root/'trained';self.trained.mkdir();self.geometry=self.root/'geometry';self.geometry.mkdir()
  self.quality={'quality_passed':True,'heldout_psnr_db':25.,'heldout_depth_median_error_m':.03};(self.trained/'training.json').write_text(json.dumps(self.quality));(self.geometry/'geometry.json').write_text(json.dumps({'keyframes':[]}))
  self.data={'means':np.tile([[.1,.2,.3]],(20,1)),'scales':np.full((20,3),.01),'quats':np.tile([[1.,0,0,0]],(20,1)),'colors':np.full((20,3),.5),'opacity':np.ones(20)};self.save()
 def save(self):np.savez(self.trained/'gaussians.npz',**self.data)
 def publish(self):return publish_tiles(self.catalog,self.scene['id'],self.trained,self.geometry)
 def test_paths_reject_escape(self):
  for value in ['../bad','a/b','','x'*65]:
   with self.assertRaises(ValueError):identifier(value)
  with self.assertRaises(ValueError):safe_path(self.root,'../outside')
 def test_failed_quality_never_publishes(self):
  self.quality['quality_passed']=False;(self.trained/'training.json').write_text(json.dumps(self.quality))
  with self.assertRaises(ValueError):self.publish()
  self.assertFalse(self.catalog.get(self.scene['id'])['published'])
 def test_nonfinite_geometry_rejected(self):
  self.data['means'][0,0]=np.nan;self.save()
  with self.assertRaises(ValueError):self.publish()
 def test_compressed_tiles_and_immutable_versions(self):
  first=self.publish();level=first['tiles'][0]['levels'][0];path=self.catalog.tile(self.scene['id'],first['version'],level['file']);self.assertEqual(len(gzip.decompress(path.read_bytes())),20*32)
  self.assertEqual(len(first['tiles'][0]['levels']),3);second=self.publish();self.assertNotEqual(first['version'],second['version']);self.assertTrue(path.is_file());self.assertFalse(second['navigation_ready'])
 def test_unlisted_tile_rejected(self):
  m=self.publish()
  with self.assertRaises(FileNotFoundError):self.catalog.tile(self.scene['id'],m['version'],'tile_999_l0.splat')
  with self.assertRaises(ValueError):self.catalog.tile(self.scene['id'],m['version'],'../../secret')
if __name__=='__main__':unittest.main()
