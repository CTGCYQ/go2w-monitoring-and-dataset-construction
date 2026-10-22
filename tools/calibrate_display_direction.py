"""Read-only direction calibration from a supervised forward/return movement.
Only aggregate calibration metrics are emitted; no raw clouds or trajectories saved.
This estimates display rotation, not sensor translation or navigation-ready extrinsics.
"""
import argparse,json,math,time,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from map_geometry import rotation,transpose,mv,distance

def unit(v):
    n=math.sqrt(sum(x*x for x in v))
    if n<1e-8:raise ValueError('degenerate calibration axis')
    return tuple(x/n for x in v)
def cross(a,b):return (a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0])
def solve(samples,gravity,require_return=True):
    if len(samples)<20:raise ValueError('insufficient poses')
    start=samples[0];ri=transpose(rotation(start[2]));relative=[mv(ri,tuple(p[1][i]-start[1][i] for i in range(3))) for p in samples]
    excursion=max(relative,key=lambda p:distance(p,(0,0,0)));length=distance(excursion,(0,0,0));returned=distance(relative[-1],(0,0,0))
    if not .25<=length<=2.:raise ValueError('direction calibration requires 0.25 to 2m supervised excursion')
    if require_return and returned>.18:raise ValueError('return to start was not observed')
    up=unit(gravity);vertical=sum(a*b for a,b in zip(excursion,up))
    if abs(vertical)>length*.25:raise ValueError('movement not horizontal')
    x=unit(tuple(excursion[i]-vertical*up[i] for i in range(3)));y=unit(cross(up,x));x=unit(cross(y,up))
    cross_error=max(abs(sum(p[i]*y[i] for i in range(3))) for p in relative)
    if cross_error>.1:raise ValueError('forward path insufficiently straight')
    return {'rotation_row_major':list(x+y+up),'excursion_m':round(length,4),'return_error_m':round(returned,4),
            'cross_track_error_m':round(cross_error,4),'pose_samples':len(samples),'gravity_mean_mps2':gravity,
            'reference_orientation_xyzw':list(start[2]),'scope':'display_orientation_only_lidar_origin','body_translation_verified':False,'raw_data_saved':False}

def main():
    p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=600);p.add_argument('--stop-file');p.add_argument('--imu-topic',default='/utlidar/imu');p.add_argument('--forward-only',action='store_true');args=p.parse_args()
    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    rclpy.init();node=rclpy.create_node('go2w_display_direction_calibration');poses=[];gravity=[];started=time.monotonic();ready=False;returned_since=None;maximum=0.;last_sample=0.;last_progress=0.;max_gyro=0.;source_stamps=set();first_stamp=None;last_stamp=None
    def odom(msg):
        nonlocal last_sample,first_stamp,last_stamp
        if msg.header.frame_id!='lidar_odom' or msg.child_frame_id!='utlidar_imu':return
        pp=msg.pose.pose.position;q=msg.pose.pose.orientation
        if ready and time.monotonic()-last_sample>=.025:
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            first_stamp=stamp if first_stamp is None else first_stamp;last_stamp=stamp
            poses.append((time.monotonic(),(pp.x,pp.y,pp.z),(q.x,q.y,q.z,q.w)));last_sample=time.monotonic()
    def imu(msg):
        nonlocal max_gyro
        gg=msg.angular_velocity;max_gyro=max(max_gyro,math.sqrt(gg.x*gg.x+gg.y*gg.y+gg.z*gg.z))
        if not ready:
            a=msg.linear_acceleration;g=msg.angular_velocity
            if math.sqrt(g.x*g.x+g.y*g.y+g.z*g.z)<.08:gravity.append((a.x,a.y,a.z))
    node.create_subscription(Odometry,'/odom_corrected',odom,100)
    node.create_subscription(Imu,args.imu_topic,imu,QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT))
    try:
        while time.monotonic()-started<args.seconds+5:
            rclpy.spin_once(node,timeout_sec=.01)
            if not ready and time.monotonic()-started>5:
                if len(gravity)<100:raise ValueError('stationary IMU baseline missing')
                ready=True;print('CALIBRATION_READY: forward then return; keep still afterward',flush=True)
            if len(poses)>20:
                displacement=distance(poses[-1][1],poses[0][1]);maximum=max(maximum,displacement)
                if maximum>.25 and displacement<.12:
                    returned_since=returned_since or time.monotonic()
                    if time.monotonic()-returned_since>8 and not args.stop_file:break
                else:returned_since=None
            if ready and time.monotonic()-last_progress>5:
                print(json.dumps({'recording':True,'elapsed_s':round(time.monotonic()-started,1),'max_displacement_m':round(maximum,4),'sampled_poses':len(poses),'max_lidar_gyro_rps':round(max_gyro,3)}),flush=True);last_progress=time.monotonic()
            if args.stop_file and Path(args.stop_file).exists() and ready:break
        g=[sum(v[i] for v in gravity)/len(gravity) for i in range(3)]
        try:result={'passed':True,**solve(poses,g,require_return=not args.forward_only)}
        except ValueError as e:result={'passed':False,'reason':str(e),'max_displacement_m':maximum,'pose_samples':len(poses)}
        if args.forward_only and 'return_error_m' in result:result['endpoint_displacement_m']=result.pop('return_error_m')
        result.update(forward_only=args.forward_only,imu_topic=args.imu_topic,first_source_stamp_s=first_stamp,last_source_stamp_s=last_stamp,max_lidar_gyro_rps=max_gyro,elapsed_s=time.monotonic()-started)
        print('CALIBRATION_RESULT '+json.dumps(result),flush=True)
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
