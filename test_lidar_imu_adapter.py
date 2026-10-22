import unittest
from types import SimpleNamespace as O
from lidar_imu_adapter import corrected_imu
class AdapterTests(unittest.TestCase):
 def test_acceleration_only_and_original_preserved(self):
  m=O(linear_acceleration=O(x=1.,y=2.,z=3.),angular_velocity=O(x=4.,y=5.,z=6.),header=O(frame_id='utlidar_imu'),orientation_covariance=[0.]*9)
  r=corrected_imu(m)
  self.assertEqual((r.linear_acceleration.x,r.linear_acceleration.y,r.linear_acceleration.z),(-1.,-2.,-3.))
  self.assertEqual(m.linear_acceleration.z,3.);self.assertEqual(r.angular_velocity.z,6.);self.assertEqual(r.header.frame_id,'utlidar_imu');self.assertEqual(r.orientation_covariance[0],-1.)
if __name__=='__main__':unittest.main()
