"""Go2 firmware IMU input adapter: gravity-like acceleration -> specific force.
Raw /utlidar/imu is preserved. This node never controls the robot.
Enable only with an empirically validated firmware profile, not arbitrary L2 SDK data.
"""
import copy

def corrected_imu(message):
    result=copy.deepcopy(message)
    for axis in ('x','y','z'):setattr(result.linear_acceleration,axis,-getattr(message.linear_acceleration,axis))
    # The estimator uses acceleration/gyro. Do not assert a vendor orientation convention.
    result.orientation_covariance[0]=-1.
    return result

def main():
    import rclpy
    from sensor_msgs.msg import Imu
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    rclpy.init();node=rclpy.create_node('go2w_imu_specific_force_adapter')
    qos=QoSProfile(depth=20,reliability=ReliabilityPolicy.RELIABLE)
    pub=node.create_publisher(Imu,'/go2w/lidar_imu_specific_force',qos)
    node.create_subscription(Imu,'/utlidar/imu',lambda msg:pub.publish(corrected_imu(msg)),qos)
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
