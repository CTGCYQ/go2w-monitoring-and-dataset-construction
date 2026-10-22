"""Validated display orientation only; never publishes robot TF or enables motion."""
import json,math
from pathlib import Path
from map_geometry import rotation,mm,mv

def valid_rotation(values):
    if not isinstance(values,list) or len(values)!=9 or not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in values):return False
    rows=[values[i:i+3] for i in (0,3,6)]
    if any(abs(sum(x*y for x,y in zip(a,b))-(1 if i==j else 0))>1e-4 for i,a in enumerate(rows) for j,b in enumerate(rows)):return False
    a,b,c=rows;det=a[0]*(b[1]*c[2]-b[2]*c[1])-a[1]*(b[0]*c[2]-b[2]*c[0])+a[2]*(b[0]*c[1]-b[1]*c[0])
    return abs(det-1)<1e-4

def metadata(path,cache,is_map=False):
    if not isinstance(cache,dict):return {}
    try:
        data=json.loads(Path(path).read_text('utf8'));values=data['rotation_row_major']
        if data.get('validated') is not True or not valid_rotation(values):return {}
        if data.get('source_frame')!='utlidar_lidar':return {}
        if is_map:
            if cache.get('mapping_version')!=2:return {}
            ar=rotation(cache['anchor_orientation_xyzw']);x=mv(ar,values[:3]);yaw=math.atan2(x[1],x[0]);c,s=math.cos(yaw),math.sin(yaw)
            rr=mm(((c,s,0),(-s,c,0),(0,0,1)),ar);values=[v for row in rr for v in row]
        elif cache.get('frame_id')!='utlidar_lidar':return {}
        return {'display_rotation_row_major':values,'display_direction_verified':True,'display_alignment_id':data['calibration_id'],
                'display_frame':'initial_forward_gravity' if is_map else 'calibrated_forward_lidar_origin',
                'body_alignment_verified':False,'body_translation_verified':False}
    except (OSError,ValueError,KeyError,TypeError,OverflowError):return {}
