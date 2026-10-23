import os,tempfile,unittest
from pathlib import Path
from camera_health import live_front,FrameProgress
class CameraHealthTests(unittest.TestCase):
 def test_live_frame_requires_both_telemetry_and_file_fresh(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'frame.jpg';p.write_bytes(b'jpeg');os.utime(p,(100,100))
   self.assertEqual(live_front(p,{'last_seen_s':100},101),b'jpeg')
   self.assertIsNone(live_front(p,{'last_seen_s':90},101));self.assertIsNone(live_front(p,{'last_seen_s':104},104))
 def test_stopped_dock_image_not_refreshed_by_polling(self):
  p=FrameProgress();self.assertTrue(p.fresh('color',(1,10),0));self.assertTrue(p.fresh('color',(1,10),2));self.assertFalse(p.fresh('color',(1,10),4));self.assertTrue(p.fresh('color',(2,10),5))
if __name__=='__main__':unittest.main()
