"""Bounded timestamped LiDAR mapping, independent of ROS and robot control."""
import math
from bisect import bisect_left
from collections import OrderedDict, deque

IDENTITY=((1.,0.,0.),(0.,1.,0.),(0.,0.,1.))
# p_IMU = R_IL p_LIDAR + t_IL; inverse of vendor T_LI.
LIDAR_IN_IMU=(.007698,.014655,-.00667)

def mv(r,p):return tuple(sum(r[i][j]*p[j] for j in range(3)) for i in range(3))
def transpose(r):return tuple(zip(*r))
def mm(a,b):return tuple(tuple(sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)) for i in range(3))
def distance(a,b):return math.sqrt(sum((x-y)**2 for x,y in zip(a,b)))

def rotation(q):
    if len(q)!=4 or not all(math.isfinite(x) for x in q):raise ValueError('invalid quaternion')
    n=math.sqrt(sum(x*x for x in q))
    if not .5<=n<=1.5:raise ValueError('invalid quaternion norm')
    x,y,z,w=(v/n for v in q)
    return ((1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)),
            (2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)),
            (2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)))

def slerp(a,b,f):
    a=tuple(v/math.sqrt(sum(x*x for x in a)) for v in a)
    b=tuple(v/math.sqrt(sum(x*x for x in b)) for v in b)
    dot=sum(x*y for x,y in zip(a,b))
    if dot<0:b=tuple(-v for v in b);dot=-dot
    dot=max(-1.,min(1.,dot))
    if dot>.9995:
        q=tuple(x+f*(y-x) for x,y in zip(a,b));n=math.sqrt(sum(v*v for v in q));return tuple(v/n for v in q)
    angle=math.acos(dot);sn=math.sin(angle)
    return tuple((math.sin((1-f)*angle)*x+math.sin(f*angle)*y)/sn for x,y in zip(a,b))

class PoseBuffer:
    def __init__(self):self.samples=deque(maxlen=2500);self.reason='waiting_for_odometry';self.rejected=0;self.fault=False;self.reordered=0;self.max_rollback=0.
    def add(self,stamp,p,q,parent,child):
        if self.fault:return False
        try:
            if parent!='lidar_odom' or child!='utlidar_imu':raise ValueError('pose_frame_mismatch')
            if not math.isfinite(stamp) or stamp<=0 or len(p)!=3 or not all(math.isfinite(v) and abs(v)<1000 for v in p):raise ValueError('pose_invalid')
            rotation(q)
            if self.samples:
                old=self.samples[-1];dt=stamp-old[0]
                if dt==0:return False
                if dt<0:
                    self.max_rollback=max(self.max_rollback,-dt)
                    if dt>=-.01:self.reordered+=1;return False
                    raise ValueError('pose_time_regression')
                if dt>1.5:raise ValueError('pose_stream_discontinuity')
                if distance(p,old[1])>.03+2.0*dt:raise ValueError('pose_position_jump')
                dot=abs(sum(a*b for a,b in zip(q,old[2])))/math.sqrt(sum(x*x for x in q)*sum(x*x for x in old[2]))
                if 2*math.acos(min(1.,dot))>.1+4*dt:raise ValueError('pose_rotation_jump')
        except (ValueError,TypeError,OverflowError) as e:
            self.reason=str(e);self.rejected+=1;self.fault=True;return False
        self.samples.append((stamp,tuple(p),tuple(q)));self.reason='ok';return True
    def at(self,stamp,max_gap=.1):
        if self.fault or not self.samples:return None
        a=list(self.samples);idx=bisect_left([v[0] for v in a],stamp)
        if idx<len(a) and abs(a[idx][0]-stamp)<1e-6:return a[idx]
        if idx==0 or idx==len(a):return None
        left,right=a[idx-1],a[idx];dt=right[0]-left[0]
        if dt>max_gap:return None
        f=(stamp-left[0])/dt
        return stamp,tuple(x+f*(y-x) for x,y in zip(left[1],right[1])),slerp(left[2],right[2],f)

class WorldMap:
    def __init__(self,voxel=.06,limit=120000):
        self.voxel=voxel;self.limit=limit;self.cells=OrderedDict();self.trajectory=deque(maxlen=10000)
        self.anchor=None;self.frames=0;self.last_stamp=None;self.last_frame_id=None;self.rejected=0;self.reason='waiting_for_synchronized_data';self.fault=False
        self.max_sync_error=0.;self.max_displacement=0.
    def reject(self,reason,fault=False):self.rejected+=1;self.reason=reason;self.fault=self.fault or fault;return False
    def add(self,stamp,frame_id,points,poses,offsets=None):
        if self.fault:return False
        if poses.fault:return self.reject(poses.reason,True)
        if frame_id!='utlidar_lidar':return self.reject('cloud_frame_mismatch',True)
        if self.last_stamp is not None and stamp<=self.last_stamp:
            return False if stamp==self.last_stamp else self.reject('cloud_time_regression',True)
        offsets=[0.]*len(points) if offsets is None else offsets
        if len(offsets)!=len(points):return self.reject('point_time_invalid',True)
        if not all(math.isfinite(v) and 0<=v<=.15 for v in offsets):return self.reject('point_time_invalid',True)
        first=poses.at(stamp);last=poses.at(stamp+max(offsets,default=0))
        if first is None or last is None:return self.reject('waiting_for_synchronized_data')
        anchor=self.anchor or first;ar=transpose(rotation(anchor[2]));ap=anchor[1]
        transformed=[]
        # Per-point pose interpolation compensates scan motion using actual PointCloud2 offsets.
        # Cache per millisecond only for computation; timestamp error <1ms.
        cache={}
        for point,offset in zip(points,offsets):
            if len(point)<3 or not all(math.isfinite(v) for v in point[:3]):continue
            rng=math.sqrt(sum(v*v for v in point[:3]))
            if not .5<=rng<=30:continue
            key=round(offset,3)
            if key not in cache:
                pose=poses.at(stamp+offset)
                if pose is None:return self.reject('waiting_for_synchronized_data')
                cache[key]=(mm(ar,rotation(pose[2])),mv(ar,tuple(x-y for x,y in zip(pose[1],ap))))
            rr,tr=cache[key];pi=tuple(point[i]+LIDAR_IN_IMU[i] for i in range(3));rp=mv(rr,pi)
            out=tuple(rp[i]+tr[i] for i in range(3))
            if not all(math.isfinite(v) and abs(v)<100 for v in out):return self.reject('map_extent_exceeded',True)
            transformed.append((*out,point[3] if len(point)>3 and math.isfinite(point[3]) else 0.))
        if not transformed:return self.reject('empty_valid_cloud')
        self.anchor=anchor;self.last_stamp=stamp;self.last_frame_id=frame_id;self.frames+=1;self.reason='ok'
        tr=mv(ar,tuple(x-y for x,y in zip(last[1],ap)));self.max_displacement=max(self.max_displacement,distance(tr,(0,0,0)))
        if not self.trajectory or distance(tr,self.trajectory[-1])>.025:self.trajectory.append(tr)
        for p in transformed:
            key=tuple(math.floor(v/self.voxel) for v in p[:3]);old=self.cells.pop(key,None)
            n=min(20,old[4]+1) if old else 1
            value=tuple(old[i]+(p[i]-old[i])/n for i in range(4))+(n,) if old else p+(1,)
            self.cells[key]=value
            if len(self.cells)>self.limit:self.cells.popitem(last=False)
        return True
    def payload(self):
        return {'points':[[round(v,4) for v in p[:4]] for p in self.cells.values()],
                'trajectory':[[round(v,4) for v in p] for p in self.trajectory],
                'count':len(self.cells),'cloud_count':self.frames,'voxel':self.voxel,'anchored':self.anchor is not None,
                'frame_id':'initial_imu','coordinate_convention':'initial_imu_xyz','body_alignment_verified':False,
                'mapping_version':2,'anchor_orientation_xyzw':list(self.anchor[2]) if self.anchor else None,'max_displacement_m':round(self.max_displacement,4),'rejected_frames':self.rejected,
                'reason_code':self.reason,'fault':self.fault,'deskew':'per_point_odometry_interpolation_1ms_bins'}
