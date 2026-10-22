"""Independent supervised left-turn check using gyro integration and LIO rotation."""
import argparse,json,math,time,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from map_geometry import rotation,mm,transpose,mv

def main():
    p=argparse.ArgumentParser();p.add_argument('--candidate',required=True);p.add_argument('--stop-file',required=True);a=p.parse_args()
    candidate=json.loads(Path(a.candidate).read_text());v=candidate['rotation_row_major'];display=[v[i:i+3] for i in (0,3,6)]
    import rclpy
    from sensor_msgs.msg import Imu
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    rclpy.init();node=rclpy.create_node('display_left_turn_validation');state={'angle':0.,'imu':0,'first_q':None,'last_q':None,'last_t':None,'skipped_dt':0,'bias_samples':[],'bias':None};start=time.monotonic();progress=0.
    def imu(m):
        stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9;g=m.angular_velocity;z=mv(display,(g.x,g.y,g.z))[2]
        if state['bias'] is None:state['bias_samples'].append(z)
        if state['last_t'] is not None and state['bias'] is not None:
            dt=stamp-state['last_t']
            if 0<dt<.05:state['angle']+=(z-state['bias'])*dt
            else:state['skipped_dt']+=1
        state['last_t']=stamp;state['imu']+=1
    def odom(m):
        q=m.pose.pose.orientation;value=(q.x,q.y,q.z,q.w);state['first_q']=state['first_q'] or value;state['last_q']=value
    node.create_subscription(Imu,'/utlidar/imu',imu,QoSProfile(depth=20,reliability=ReliabilityPolicy.BEST_EFFORT));node.create_subscription(Odometry,'/odom_corrected',odom,100)
    try:
        ready=False
        while time.monotonic()-start<600:
            rclpy.spin_once(node,timeout_sec=.01)
            if not ready and state['imu']>400 and state['first_q']:
                state['bias']=sum(state['bias_samples'])/len(state['bias_samples'])
                print('TURN_VALIDATION_READY',flush=True);ready=True
            if ready and time.monotonic()-progress>5:
                print(json.dumps({'recording':True,'integrated_yaw_degrees':round(math.degrees(state['angle']),2)}),flush=True);progress=time.monotonic()
            if ready and Path(a.stop_file).exists():break
        relative=mm(mm(display,mm(transpose(rotation(state['first_q'])),rotation(state['last_q']))),transpose(display))
        yaw=math.degrees(math.atan2(relative[1][0],relative[0][0]));gyro=math.degrees(state['angle'])
        result={'passed':5<gyro<90 and 5<yaw<90 and abs(yaw-gyro)<5,'gyro_yaw_degrees':gyro,'lio_yaw_degrees':yaw,
                'imu_messages':state['imu'],'stationary_gyro_bias_rps':state['bias'],'skipped_imu_dt':state['skipped_dt'],'raw_data_saved':False}
        print('TURN_RESULT '+json.dumps(result),flush=True)
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
