#!/usr/bin/env python3
"""Read-only, bounded mapping from synchronized L2 clouds and Point-LIO IMU poses.

No motion publishers. Only the existing map_cache.json display cache is written.
Fatal frame/time/pose faults latch; restart after fixing upstream localization.
"""
import json
import os
import time
from collections import deque
from pathlib import Path
from cloud_model import parse_cloud
from map_geometry import PoseBuffer, WorldMap

CACHE_FILE=Path(__file__).resolve().parent/'map_cache.json'

def main():
    import rclpy
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    rclpy.init();node=rclpy.create_node('go2w_map_worker')
    poses=PoseBuffer();world=WorldMap();pending=deque(maxlen=8)
    observed={'cloud':0.,'odom':0.,'integrated':0.};last_write=0.;started=time.monotonic()
    def stamp(msg):return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
    def odom(msg):
        p=msg.pose.pose.position;q=msg.pose.pose.orientation
        if poses.add(stamp(msg),(p.x,p.y,p.z),(q.x,q.y,q.z,q.w),msg.header.frame_id,msg.child_frame_id):observed['odom']=time.monotonic()
    def cloud(msg):
        pending.append(msg);observed['cloud']=time.monotonic()
    node.create_subscription(Odometry,'/odom_corrected',odom,QoSProfile(depth=500,reliability=ReliabilityPolicy.RELIABLE))
    node.create_subscription(PointCloud2,'/utlidar/cloud',cloud,QoSProfile(depth=8,reliability=ReliabilityPolicy.BEST_EFFORT))
    print('[map_worker] synchronized mapping v2; warming up 5s; no motion commands',flush=True)
    try:
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.005);mono=time.monotonic()
            if poses.fault:world.reject(poses.reason,True) if not world.fault else None
            if pending and not world.fault and mono-started>5:
                msg=pending[0]
                if world.last_stamp is not None and stamp(msg)<=world.last_stamp:pending.popleft();continue
                if poses.samples and poses.samples[-1][0]>stamp(msg)+.15:
                    pending.popleft();packet=parse_cloud(msg,max_points=2500,include_point_times=True)
                    if packet:
                        if world.add(packet['source_stamp_s'],packet['frame_id'],packet['points'],poses,packet['point_offsets_s']):observed['integrated']=mono
                    else:world.reject('cloud_layout_or_point_time_invalid',True)
                elif mono-observed['odom']>1.5:pending.clear()
            if mono-last_write>=1:
                payload=world.payload();fresh=all(mono-v<1.5 for v in observed.values())
                payload.update(reordered_pose_samples=poses.reordered,max_pose_rollback_s=poses.max_rollback)
                payload.update(online=fresh and not world.fault and world.frames>0,written_at=time.time(),received_at=time.time(),source_stamp_s=world.last_stamp)
                if not fresh:payload['reason_code']='localization_or_cloud_stale'
                if world.frames==0 and not world.fault:payload['reason_code']='waiting_for_synchronized_data'
                if world.fault:payload['reason_code']=world.reason
                tmp=CACHE_FILE.with_suffix('.json.tmp')
                with tmp.open('w') as stream:json.dump(payload,stream,allow_nan=False)
                os.replace(tmp,CACHE_FILE);last_write=mono
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
