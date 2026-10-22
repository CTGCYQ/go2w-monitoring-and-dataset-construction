"""Build an unvalidated display candidate from a supervised forward-leg map and live IMU.
A separate left-turn check is required. Never enables robot TF or motion.
"""
import argparse,json,sys,time,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from map_geometry import rotation,mv,transpose,distance
from tools.calibrate_display_direction import unit,cross
import rclpy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile,ReliabilityPolicy
from pathlib import Path
parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--map-cache',required=True);parser.add_argument('--output',required=True);parser.add_argument('--supervised-forward-leg',action='store_true',required=True);args=parser.parse_args()
cache=json.loads(Path(args.map_cache).read_text())
if cache.get('mapping_version')!=2 or not cache.get('anchor_orientation_xyzw'):raise ValueError('versioned map anchor required')
traj=cache['trajectory'];start=traj[0];far=max(traj,key=lambda p:distance(p,start));dv=tuple(far[i]-start[i] for i in range(3));ra=rotation(cache['anchor_orientation_xyzw']);fw=mv(ra,dv)
rclpy.init();n=rclpy.create_node('display_candidate_probe');acc=[];quats=[];gyros=[]
def imu(m):
 a=m.linear_acceleration;g=m.angular_velocity;acc.append((a.x,a.y,a.z));gyros.append(math.sqrt(g.x*g.x+g.y*g.y+g.z*g.z))
def odom(m):
 q=m.pose.pose.orientation;quats.append((q.x,q.y,q.z,q.w))
n.create_subscription(Imu,'/utlidar/imu',imu,QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT));n.create_subscription(Odometry,'/odom_corrected',odom,10)
t=time.monotonic()
while time.monotonic()-t<5:rclpy.spin_once(n,timeout_sec=.01)
g=tuple(sum(v[i] for v in acc)/len(acc) for i in range(3));q=quats[-1];f=mv(transpose(rotation(q)),fw);up=unit(g);vertical=sum(a*b for a,b in zip(f,up));x=unit(tuple(f[i]-vertical*up[i] for i in range(3)));y=unit(cross(up,x));x=unit(cross(y,up))
line=unit(dv);cross_error=max(distance(tuple(p[i]-start[i] for i in range(3)),tuple(sum((p[j]-start[j])*line[j] for j in range(3))*v for v in line)) for p in traj)
result={'rotation_row_major':list(x+y+up),'calibration_id':'manual_forward_20261022','source_frame':'utlidar_lidar','validated':False,
 'excursion_m':distance(far,start),'return_error_m':distance(traj[-1],start),'cross_track_error_m':cross_error,
 'vertical_fraction':vertical/distance(far,start),'stationary_max_gyro_rps':max(gyros),'gravity_mean_mps2':list(g),
 'reference_orientation_xyzw':list(q),'anchor_orientation_xyzw':cache['anchor_orientation_xyzw'],
 'scope':'display_orientation_only_lidar_origin','scale_verified':False,'body_translation_verified':False,'raw_data_saved':False}
result['return_error_m']=None;result['return_error_unavailable_due_to_frozen_map']=bool(cache.get('fault'))
Path(args.output).write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2));n.destroy_node();rclpy.shutdown()
