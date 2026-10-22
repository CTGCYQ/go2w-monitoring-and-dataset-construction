import unittest
import struct
from types import SimpleNamespace as Obj
from cloud_model import parse_cloud,cloud_response,motion_hint,map_snapshot_response

class CloudTests(unittest.TestCase):
 def message(self,big=False):
  endian='>' if big else '<';data=b''
  for row in range(2):
   data+=b''.join(struct.pack(endian+'ffff',row+1,col+2,.5,.8) for col in range(2))+bytes(8)
  return Obj(width=2,height=2,point_step=16,row_step=40,is_bigendian=big,data=data,
    fields=[Obj(name=n,offset=i*4,datatype=7) for i,n in enumerate(['x','y','z','intensity'])],
    header=Obj(frame_id='utlidar_lidar',stamp=Obj(sec=10,nanosec=20)))
 def test_organized_padding_and_endianness(self):
  for big in [False,True]:
   result=parse_cloud(self.message(big))
   self.assertEqual(result['points'][3],[2.,3.,.5,.8]);self.assertEqual(result['total'],4)
   self.assertEqual(result['frame_id'],'utlidar_lidar')
 def test_point_time_uses_same_indices_as_valid_points(self):
  m=self.message();m.fields[-1].name='time';data=bytearray(m.data);struct.pack_into('<f',data,0,float('nan'));m.data=data
  result=parse_cloud(m,max_points=4,include_point_times=True)
  self.assertEqual(len(result['points']),3);self.assertEqual(len(result['point_offsets_s']),3)
  self.assertAlmostEqual(result['point_offsets_s'][0],.8,places=5)
 def test_missing_time_fails_mapping_not_live_display(self):
  self.assertIsNone(parse_cloud(self.message(),include_point_times=True))
  self.assertIsNotNone(parse_cloud(self.message()))
 def test_bad_map_types_and_stale_online(self):
  self.assertFalse(map_snapshot_response([])['online'])
  self.assertEqual(map_snapshot_response({'points':'bad'})['reason_code'],'map_geometry_invalid')
  self.assertEqual(map_snapshot_response({'points':[[1,2,3]],'trajectory':{}})['reason_code'],'map_geometry_invalid')
  self.assertFalse(map_snapshot_response({'online':True,'written_at':1,'points':[[1,2,3]]})['online'])
 def test_nonfinite_trajectory_rejected(self):
  result=map_snapshot_response({'points':[[1,2,3]],'trajectory':[[float('nan'),0,0]]})
  self.assertEqual(result['reason_code'],'map_geometry_invalid')
 def test_truncation_rejected(self):
  m=self.message();m.data=m.data[:50];self.assertIsNone(parse_cloud(m))
 def test_missing_coordinates_rejected(self):
  m=self.message();m.fields=m.fields[1:];self.assertIsNone(parse_cloud(m))
 def test_response_budget_and_metadata(self):
  cache=dict(online=True,received_at=100,points=[[1,2,3]]*211,frame_id='sensor',frame_key='1')
  result=cloud_response(cache,100,now=100.1)
  self.assertLessEqual(result['count'],100);self.assertEqual(result['frame_id'],'sensor')
  self.assertFalse(result['body_alignment_verified'])
  self.assertFalse(cloud_response(cache,100,now=103)['online'])
 def test_motion_hint_is_not_registration(self):
  state=dict(last_seen_s=100,imu={'gyroscope':[0,0,.2]},sport_mode={'velocity':[0,0,0]})
  self.assertTrue(motion_hint(state,100)['moving'])
  self.assertFalse(motion_hint(state,103)['available'])
 def test_native_axis_values_not_remapped(self):
  result=parse_cloud(self.message());self.assertEqual(result['points'][0][:3],[1.,2.,.5])

 def test_bad_legacy_map_does_not_render_as_valid_room(self):
  result=map_snapshot_response({'online':False,'points':[[10000,20000,-327347]]})
  self.assertEqual(result['reason_code'],'map_geometry_invalid');self.assertEqual(result['points'],[])
 def test_valid_offline_map_remains_a_snapshot(self):
  result=map_snapshot_response({'online':False,'points':[[1,2,3]]*2001,'trajectory':[[0,0,0]]},1000)
  self.assertFalse(result['online']);self.assertTrue(result['geometry_valid']);self.assertLessEqual(result['count'],1000)

if __name__=='__main__':unittest.main()
