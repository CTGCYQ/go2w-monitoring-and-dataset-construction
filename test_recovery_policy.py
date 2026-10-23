import json,tempfile,unittest
from pathlib import Path
from recovery_policy import LinkGate,Backoff,supervisor_health,wait_for_process,telemetry_status
class RecoveryTests(unittest.TestCase):
 def test_link_requires_time_and_distinct_received_frames(self):
  g=LinkGate();self.assertFalse(g.observe(0,True,'a'));self.assertFalse(g.observe(1,True,'b'));self.assertTrue(g.observe(3,True,'c'))
 def test_repeated_cached_frame_cannot_start_estimator(self):
  g=LinkGate()
  for n in range(10):self.assertFalse(g.observe(n,True,'same'))
 def test_link_flapping_resets_startup_debounce(self):
  g=LinkGate();g.observe(0,True,'a');g.observe(1,True,'b');g.observe(2,False,None)
  self.assertFalse(g.observe(3,True,'c'));self.assertFalse(g.observe(4,True,'d'));self.assertTrue(g.observe(6,True,'e'))
 def test_disconnect_waits_grace_then_stops(self):
  g=LinkGate();self.assertTrue(g.offline(0));g.observe(10,True,'a');g.observe(11,False,None)
  self.assertFalse(g.offline(12));self.assertTrue(g.offline(13))
 def test_backoff_is_bounded_and_success_resets(self):
  b=Backoff()
  for i in range(100):b.fail(i*100);self.assertLessEqual(b.next-i*100,60);self.assertFalse(b.ready(i*100))
  b.success();self.assertTrue(b.ready(0));self.assertEqual(b.failures,0)
 def test_ros_child_startup_delay_is_not_a_failed_launch(self):
  t=[0.];calls=[None,None,123]
  pid=wait_for_process(lambda:calls.pop(0),clock=lambda:t[0],sleep=lambda v:t.__setitem__(0,t[0]+v))
  self.assertEqual(pid,123);self.assertGreater(t[0],0)
 def test_missing_child_startup_times_out(self):
  t=[0.];pid=wait_for_process(lambda:None,timeout=.3,clock=lambda:t[0],sleep=lambda v:t.__setitem__(0,t[0]+v));self.assertIsNone(pid)
 def test_dead_collector_cannot_leave_robot_online_indefinitely(self):
  self.assertTrue(telemetry_status({'online':True,'last_seen_s':99},100)['online'])
  self.assertFalse(telemetry_status({'online':True,'last_seen_s':95},100)['online'])
  self.assertFalse(telemetry_status({'online':True,'last_seen_s':101},100)['online'])
  self.assertFalse(telemetry_status({'online':True,'last_seen_s':float('nan')},100)['online'])
 def test_health_rejects_missing_stale_future_and_malformed(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'status.json';self.assertFalse(supervisor_health(p,100)['online'])
   for value in [{},[],{'written_at':90},{'written_at':101},{'written_at':float('nan')}]:
    p.write_text(json.dumps(value));self.assertFalse(supervisor_health(p,100)['online'])
   p.write_text(json.dumps({'written_at':99,'services':{'web':123}}));self.assertTrue(supervisor_health(p,100)['online'])
if __name__=='__main__':unittest.main()
