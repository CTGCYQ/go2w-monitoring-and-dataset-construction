"""PointCloud2 decoding and read-only visualization metadata; no frame guessing."""
import math
import struct
import time
from sensor_freshness import cloud_is_fresh


def parse_cloud(msg, max_points=16000, include_point_times=False):
    width,height,ps=msg.width,msg.height,msg.point_step
    if width<=0 or height<=0 or ps<=0:return None
    row_step=msg.row_step or width*ps
    if row_step<width*ps or len(msg.data)<(height-1)*row_step+width*ps:return None
    fields={f.name:f for f in msg.fields}
    formats={7:('f',4),8:('d',8)}
    if any(k not in fields or fields[k].datatype not in formats for k in ('x','y','z')):return None
    names=['x','y','z']
    has_intensity='intensity' in fields and fields['intensity'].datatype in formats
    if has_intensity:names.append('intensity')
    for name in names:
        f=fields[name]
        if f.offset<0 or f.offset+formats[f.datatype][1]>ps:return None
    endian='>' if msg.is_bigendian else '<'
    readers=[(endian+formats[fields[k].datatype][0],fields[k].offset) for k in names]
    n=width*height;step=max(1,math.ceil(n/max(1,max_points)));pts=[];times=[]
    tf=fields.get('time') if include_point_times else None
    if include_point_times and (tf is None or tf.datatype not in formats or tf.offset<0 or tf.offset+formats[tf.datatype][1]>ps):return None
    for index in range(0,n,step):
        base=(index//width)*row_step+(index%width)*ps
        values=[struct.unpack_from(fmt,msg.data,base+offset)[0] for fmt,offset in readers]
        if all(math.isfinite(v) for v in values):
            pts.append([round(v,4) for v in values])
            if include_point_times:times.append(struct.unpack_from(endian+formats[tf.datatype][0],msg.data,base+tf.offset)[0])
    stamp=msg.header.stamp
    source_stamp=stamp.sec+stamp.nanosec*1e-9
    frame_id=msg.header.frame_id or 'unknown'
    return ({'point_offsets_s':times} if include_point_times else {}) | {'points':pts,'count':len(pts),'total':n,'has_intensity':has_intensity,
            'frame_id':frame_id,'source_stamp_s':source_stamp,
            'frame_key':frame_id+':'+str(stamp.sec)+':'+str(stamp.nanosec)}


def motion_hint(state, now=None):
    now=time.time() if now is None else now
    unknown={'available':False,'moving':None,'kind':'motion_hint_only'}
    try:
        age=now-float(state['last_seen_s'])
        if not math.isfinite(age) or not 0<=age<=1:return unknown
        gyro=state['imu']['gyroscope']
        velocity=state.get('sport_mode',{}).get('velocity')
        if len(gyro)!=3 or not all(math.isfinite(float(v)) for v in gyro):return unknown
        angular=math.sqrt(sum(float(v)**2 for v in gyro))
        speed=None
        if isinstance(velocity,list) and len(velocity)==3 and all(math.isfinite(float(v)) for v in velocity):
            speed=math.sqrt(sum(float(v)**2 for v in velocity))
        return {'available':True,'moving':angular>.10 or (speed is not None and speed>.04),
                'angular_speed_rps':round(angular,4),'speed_mps':speed,'kind':'motion_hint_only'}
    except (KeyError,TypeError,ValueError,OverflowError):return unknown


def cloud_response(cache,max_points=8000,now=None):
    now=time.time() if now is None else now
    if not cloud_is_fresh(cache,now=now) or not cache.get('points'):
        return {'online':False,'reason':'point cloud missing or stale','points':[],'count':0,'has_intensity':False}
    limit=max(100,min(int(max_points),20000));points=cache['points']
    sampled=points[::max(1,math.ceil(len(points)/limit))]
    return {k:v for k,v in cache.items() if k!='points'} | {
        'online':True,'points':sampled,'count':len(sampled),'age_s':round(now-cache['received_at'],4),
        'coordinate_convention':'source_xyz_right_handed','body_alignment_verified':False}


def map_snapshot_response(cache,max_points=60000):
    """Validate the existing relative-odometry map; never normalize corrupt geometry."""
    if not isinstance(cache,dict):cache={'online':False,'points':[],'reason_code':'map_cache_invalid'}
    points=cache.get('points') or []
    if cache.get('trajectory') is not None and not isinstance(cache.get('trajectory'),list):return {'online':False,'points':[],'trajectory':[],'count':0,'geometry_valid':False,'reason_code':'map_geometry_invalid'}
    if not isinstance(points,list):return {'online':False,'points':[],'trajectory':[],'count':0,'geometry_valid':False,'reason_code':'map_geometry_invalid'}
    if cache.get('online'):
        written=cache.get('written_at')
        if not isinstance(written,(int,float)) or not math.isfinite(written) or not 0<=time.time()-written<=3:
            cache={**cache,'online':False,'reason_code':'map_cache_stale'}
    if not points:return {**cache,'points':[],'count':0,'online':False}
    invalid=False;maximum=0.
    for point in points+(cache.get('trajectory') if isinstance(cache.get('trajectory'),list) else []):
        if not isinstance(point,list) or len(point)<3 or any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in point):
            invalid=True;break
        maximum=max(maximum,*(abs(v) for v in point[:3]))
    if invalid or maximum>1000:
        return {'online':False,'points':[],'trajectory':[],'count':0,'geometry_valid':False,
                'reason_code':'map_geometry_invalid','reason':'relative map coordinates invalid or exceed 1000m',
                'legacy_point_count':len(points),'max_abs_coordinate_m':maximum,'validation_limit_m':1000}
    limit=max(1000,min(int(max_points),120000));step=max(1,math.ceil(len(points)/limit))
    return {**cache,'points':points[::step],'count':len(points[::step]),'geometry_valid':True,
            'coordinate_convention':cache.get('coordinate_convention','legacy_relative_xyz'),'body_alignment_verified':False}
