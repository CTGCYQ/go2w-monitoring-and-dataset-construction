import math
import unittest
from map_geometry import PoseBuffer,WorldMap,rotation,mv,LIDAR_IN_IMU

Q=(0,0,0,1)
def poses(yaw=0,position=(0,0,0)):
    b=PoseBuffer();q=(0,0,math.sin(yaw/2),math.cos(yaw/2))
    for i in range(31):b.add(10+i*.01,position,q,'lidar_odom','utlidar_imu')
    return b

class MappingTests(unittest.TestCase):
    def test_first_frame_rejects_absolute_offset(self):
        b=poses(position=(100,200,300));m=WorldMap();self.assertTrue(m.add(10.1,'utlidar_lidar',[[1,0,0,2]],b))
        p=m.payload()['points'][0];self.assertAlmostEqual(p[0],1+LIDAR_IN_IMU[0],places=4);self.assertLess(abs(p[2]),.01)
    def test_nonidentity_anchor_uses_inverse_on_left(self):
        b=poses(yaw=math.pi/2,position=(2,4,1));m=WorldMap();m.add(10.1,'utlidar_lidar',[[1,0,0]],b)
        self.assertAlmostEqual(m.payload()['points'][0][0],1+LIDAR_IN_IMU[0],places=4)
        self.assertAlmostEqual(m.payload()['points'][0][1],LIDAR_IN_IMU[1],places=4)
    def test_translation_in_anchor_coordinates(self):
        b=PoseBuffer();q=(0,0,math.sqrt(.5),math.sqrt(.5))
        for i in range(31):b.add(10+i*.01,(0,i*.01,0),q,'lidar_odom','utlidar_imu')
        m=WorldMap();m.add(10.05,'utlidar_lidar',[[1,0,0]],b);m.add(10.25,'utlidar_lidar',[[1,0,0]],b)
        self.assertAlmostEqual(m.payload()['trajectory'][-1][0],.2)
    def test_point_times_deskew_translation(self):
        b=PoseBuffer()
        for i in range(31):b.add(10+i*.01,(i*.01,0,0),Q,'lidar_odom','utlidar_imu')
        m=WorldMap(voxel=.001);m.add(10.1,'utlidar_lidar',[[1,0,0],[.95,0,0]],b,[0,.05])
        self.assertEqual(len(m.cells),1)
    def test_pose_interpolation_slerp(self):
        b=PoseBuffer();b.add(10,(0,0,0),Q,'lidar_odom','utlidar_imu');b.add(10.1,(.1,0,0),(0,0,math.sin(.1),math.cos(.1)),'lidar_odom','utlidar_imu')
        p=b.at(10.05);self.assertAlmostEqual(p[1][0],.05);self.assertAlmostEqual(mv(rotation(p[2]),(1,0,0))[1],math.sin(.1))
    def test_reject_extrapolation_and_large_gaps(self):
        b=poses();self.assertIsNone(b.at(9.9));self.assertIsNone(b.at(10.5))
        m=WorldMap();self.assertFalse(m.add(10.5,'utlidar_lidar',[[1,0,0]],b));self.assertFalse(m.fault)
    def test_invalid_quaternion_latches(self):
        b=PoseBuffer();self.assertFalse(b.add(10,(0,0,0),(0,0,0,0),'lidar_odom','utlidar_imu'));self.assertTrue(b.fault)
    def test_small_out_of_order_pose_ignored(self):
        b=poses();self.assertFalse(b.add(10.295,(0,0,0),Q,'lidar_odom','utlidar_imu'));self.assertFalse(b.fault);self.assertEqual(b.reordered,1);self.assertEqual(b.samples[-1][0],10.3)
    def test_pose_jump_latches(self):
        b=poses();self.assertFalse(b.add(10.31,(10,0,0),Q,'lidar_odom','utlidar_imu'));self.assertEqual(b.reason,'pose_position_jump')
    def test_wrong_child_and_time_regression(self):
        b=PoseBuffer();self.assertFalse(b.add(10,(0,0,0),Q,'lidar_odom','base_link'))
        b=poses();self.assertFalse(b.add(10.1,(0,0,0),Q,'lidar_odom','utlidar_imu'));self.assertTrue(b.fault)
    def test_duplicate_cloud_not_accumulated(self):
        b=poses();m=WorldMap();m.add(10.1,'utlidar_lidar',[[1,0,0]],b);self.assertFalse(m.add(10.1,'utlidar_lidar',[[1,0,0]],b));self.assertEqual(m.frames,1)
    def test_wrong_cloud_frame_and_invalid_times(self):
        b=poses();m=WorldMap();self.assertFalse(m.add(10.1,'base_link',[[1,0,0]],b));self.assertTrue(m.fault)
        m=WorldMap();self.assertFalse(m.add(10.1,'utlidar_lidar',[[1,0,0]],b,[float('nan')]));self.assertTrue(m.fault)
    def test_memory_and_point_filter(self):
        m=WorldMap(limit=5);m.add(10.1,'utlidar_lidar',[[1+i*.1,0,0] for i in range(20)]+[[float('inf'),0,0],[100,0,0]],poses());self.assertEqual(len(m.cells),5)
    def test_no_mutation_on_unsynchronized_frame(self):
        m=WorldMap();m.add(10.1,'utlidar_lidar',[[1,0,0]],poses());before=m.payload()['points'];m.add(11,'utlidar_lidar',[[2,0,0]],poses());self.assertEqual(before,m.payload()['points'])

if __name__=='__main__':unittest.main()
