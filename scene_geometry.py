"""Metric RGB-D geometry, keyframe tracking and prior-map camera relocalization.
No body extrinsic is inferred. Camera pose never authorizes robot motion.
"""
import argparse,json,math,time,zlib
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial.transform import Rotation


def contained(root,name):
    path=(Path(root)/name).resolve()
    if not path.is_relative_to(Path(root).resolve()):raise ValueError('dataset path escapes root')
    return path


def load_frame(root,record,scale=1):
    h=record['header'];w,height=h['width'],h['height']
    if not 1<=w<=1280 or not 1<=height<=720:raise ValueError('invalid frame dimensions')
    data=contained(root,record['depth']).read_bytes()
    decoder=zlib.decompressobj();raw=decoder.decompress(data,w*height*2+1)
    if len(raw)!=w*height*2 or not decoder.eof or decoder.unconsumed_tail:raise ValueError('invalid depth payload size')
    depth=np.frombuffer(raw,dtype='<u2').reshape(height,w).astype(np.float32)*float(h['meters_per_unit'])
    rgb=cv2.imdecode(np.frombuffer(contained(root,record['rgb']).read_bytes(),np.uint8),cv2.IMREAD_COLOR)
    if rgb is None or rgb.shape[:2]!=(height,w):raise ValueError('RGB/depth dimensions differ')
    K=np.array([[h['fx'],0,h['ppx']],[0,h['fy'],h['ppy']],[0,0,1]],np.float64)
    if max(abs(v) for v in h.get('coefficients',[0]*5))>1e-6:raise ValueError('nonzero camera distortion must be rectified before mapping')
    if scale!=1:
        rgb=cv2.resize(rgb,None,fx=scale,fy=scale,interpolation=cv2.INTER_AREA)
        depth=cv2.resize(depth,(rgb.shape[1],rgb.shape[0]),interpolation=cv2.INTER_NEAREST);K[:2]*=scale
    return rgb,depth,K


def feature_frame(rgb,depth,K):
    gray=cv2.cvtColor(rgb,cv2.COLOR_BGR2GRAY)
    keypoints,descriptors=cv2.SIFT_create(nfeatures=1800,contrastThreshold=.02).detectAndCompute(gray,None)
    if descriptors is None:return {'pixels':np.empty((0,2)), 'xyz':np.empty((0,3)), 'descriptors':np.empty((0,32),np.uint8)}
    xy=np.array([k.pt for k in keypoints],np.float64)
    ij=np.rint(xy).astype(int);ij[:,0]=np.clip(ij[:,0],0,depth.shape[1]-1);ij[:,1]=np.clip(ij[:,1],0,depth.shape[0]-1)
    z=depth[ij[:,1],ij[:,0]];valid=(z>.3)&(z<8)&np.isfinite(z)
    xyz=np.column_stack(((xy[:,0]-K[0,2])*z/K[0,0],(xy[:,1]-K[1,2])*z/K[1,1],z))
    return {'pixels':xy[valid],'xyz':xyz[valid],'descriptors':descriptors[valid]}


def match_pose(reference,current,K):
    if min(len(reference['descriptors']),len(current['descriptors']))<25:return None
    pairs=cv2.BFMatcher(cv2.NORM_L2 if reference['descriptors'].dtype==np.float32 else cv2.NORM_HAMMING).knnMatch(reference['descriptors'],current['descriptors'],k=2)
    matches=[a for pair in pairs if len(pair)==2 for a,b in [pair] if a.distance<.75*b.distance]
    # Each observed keypoint may support at most one correspondence.
    unique={}
    for m in sorted(matches,key=lambda x:x.distance):unique.setdefault(m.trainIdx,m)
    matches=list(unique.values())
    if len(matches)<25:return None
    obj=np.asarray([reference['xyz'][m.queryIdx] for m in matches],np.float64)
    pixels=np.asarray([current['pixels'][m.trainIdx] for m in matches],np.float64)
    ok,rvec,tvec,inliers=cv2.solvePnPRansac(obj,pixels,K,None,iterationsCount=150,reprojectionError=2.5,confidence=.999,flags=cv2.SOLVEPNP_EPNP)
    if not ok or inliers is None or len(inliers)<20 or len(inliers)/len(matches)<.4:return None
    idx=inliers[:,0]
    ok,rvec,tvec=cv2.solvePnP(obj[idx],pixels[idx],K,None,rvec,tvec,True,flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:return None
    R=cv2.Rodrigues(rvec)[0];T=np.eye(4);T[:3,:3]=R;T[:3,3]=tvec[:,0]
    predicted=(R@obj[idx].T).T+tvec[:,0]
    actual=np.asarray([current['xyz'][matches[i].trainIdx] for i in idx])
    depth_error=float(np.median(np.abs(predicted[:,2]-actual[:,2])))
    reprojection=cv2.projectPoints(obj[idx],rvec,tvec,K,None)[0][:,0]
    error=float(np.sqrt(np.mean(np.sum((reprojection-pixels[idx])**2,axis=1))))
    if depth_error>.12 or error>2.0:return None
    return {'T_current_reference':T,'inliers':len(idx),'matches':len(matches),'ratio':len(idx)/len(matches),'reprojection_px':error,'depth_error_m':depth_error}


def pose_distance(a,b):
    delta=np.linalg.inv(a)@b
    return float(np.linalg.norm(delta[:3,3])),float(np.linalg.norm(Rotation.from_matrix(delta[:3,:3]).as_rotvec()))


def build_geometry(root,output,max_keyframes=120):
    root=Path(root);output=Path(output);output.mkdir(parents=True,exist_ok=True)
    records=[json.loads(l) for l in (root/'frames.jsonl').read_text(encoding='utf8').splitlines() if l]
    if len(records)<3:raise ValueError('insufficient capture frames')
    poses=[];selected=[];last_features=None;last_pose=np.eye(4);last_time=None;tracking_failures=0;rejected=0;last_valid_time=None
    for record in records:
        rgb,depth,K=load_frame(root,record)
        features=feature_frame(rgb,depth,K);stamp=record['header']['captured_host_time_s']
        if last_features is None:
            if len(features['descriptors'])<50:continue
            selected.append((record,features,last_pose.copy()));last_features=features;last_time=stamp
            poses.append({'index':record['index'],'T_world_camera':last_pose.tolist(),'source_time_s':stamp});last_valid_time=stamp;continue
        result=match_pose(last_features,features,K)
        if result is None:
            tracking_failures+=1
            if stamp-last_valid_time>12:raise ValueError('tracking lost for over 12 seconds; resurvey with overlapping views')
            continue
        last_valid_time=stamp
        pose=last_pose@np.linalg.inv(result['T_current_reference'])
        translation,angle=pose_distance(last_pose,pose)
        if translation>.7 or angle>.65:rejected+=1;continue
        # Static frames cannot accumulate random pose noise into a fake trajectory.
        if translation<.035 and angle<.035:continue
        selected.append((record,features,pose.copy()));last_features=features;last_pose=pose;last_time=stamp
        poses.append({'index':record['index'],'T_world_camera':pose.tolist(),'source_time_s':stamp,'inliers':result['inliers'],'reprojection_px':result['reprojection_px'],'depth_error_m':result['depth_error_m']})
        if len(selected)>=max_keyframes:break
    if len(selected)<5:raise ValueError('fewer than five separated keyframes; need more viewpoint coverage')
    positions=np.asarray([item[2][:3,3] for item in selected]);baseline=float(np.max(np.linalg.norm(positions-positions[0],axis=1)))
    if baseline<.15:raise ValueError('translation baseline below 0.15m; in-place/static capture is insufficient')
    means=[];colors=[];clouds=[]
    for k,(record,features,pose) in enumerate(selected):
        if k%5==4:continue # Hold-out RGB never initializes appearance parameters.
        rgb,depth,K=load_frame(root,record);ys,xs=np.mgrid[0:depth.shape[0]:5,0:depth.shape[1]:5];z=depth[ys,xs]
        valid=(z>.3)&(z<7)&np.isfinite(z);x=xs[valid];y=ys[valid];z=z[valid]
        points=np.column_stack(((x-K[0,2])*z/K[0,0],(y-K[1,2])*z/K[1,1],z))
        points=points@pose[:3,:3].T+pose[:3,3]
        means.append(points);colors.append(rgb[y,x,::-1]/255.)
    means=np.concatenate(means);colors=np.concatenate(colors)
    _,indices=np.unique(np.floor(means/.02).astype(np.int64),axis=0,return_index=True)
    if len(indices)>100000:indices=indices[np.linspace(0,len(indices)-1,100000,dtype=int)]
    means=means[indices].astype(np.float32);colors=colors[indices].astype(np.float32)
    np.savez_compressed(output/'geometry.npz',means=means,colors=colors)
    index=[]
    for k,(record,features,pose) in enumerate(selected):
        # Map-space 3D observations support camera re-localization after restart.
        world=features['xyz']@pose[:3,:3].T+pose[:3,3]
        np.savez_compressed(output/f'features_{k:04d}.npz',xyz=world.astype(np.float32),descriptors=features['descriptors'],pixels=features['pixels'])
        index.append({'keyframe':k,'record':record,'T_world_camera':pose.tolist()})
    report={'status':'geometry_ready','coordinate_system':'first_camera_optical_RDF','feature_method':'SIFT','units':'metres','camera_only':True,'body_extrinsics_verified':False,'source_root':str(root.resolve()),'keyframe_count':len(selected),'source_frame_count':len(records),'baseline_m':baseline,'point_count':len(means),'tracking_failures':tracking_failures,'rejected_pose_jumps':rejected,'loop_closures':0,'global_loop_optimization':'not_performed','poses':poses,'keyframes':index}
    (output/'geometry.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf8')
    return report


def localize_camera(geometry,rgb,depth,K):
    geometry=Path(geometry);meta=json.loads((geometry/'geometry.json').read_text(encoding='utf8'));features=feature_frame(rgb,depth,K);candidates=[]
    for item in meta['keyframes']:
        with np.load(geometry/f"features_{item['keyframe']:04d}.npz",allow_pickle=False) as saved:
            result=match_pose({k:saved[k] for k in ['xyz','descriptors','pixels']},features,K)
        if result:
            pose=np.linalg.inv(result['T_current_reference']);score=result['inliers']/(1+result['reprojection_px'])
            candidates.append((score,item['keyframe'],pose,result))
    if not candidates:return {'status':'lost','reason':'no geometrically verified camera/map match','navigation_ready':False}
    candidates.sort(key=lambda v:v[0],reverse=True);best=candidates[0]
    for second in candidates[1:]:
        distance,angle=pose_distance(best[2],second[2])
        if (distance>.5 or angle>.35) and second[0]>.85*best[0]:return {'status':'ambiguous','reason':'conflicting place matches','navigation_ready':False}
    result=best[3]
    return {'status':'localized','pose_frame':'camera_optical','T_map_camera':best[2].tolist(),'matched_keyframe':best[1],'inliers':result['inliers'],'reprojection_px':result['reprojection_px'],'depth_error_m':result['depth_error_m'],'navigation_ready':False,'body_extrinsics_verified':False,'checked_at':time.time()}


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--capture',required=True);parser.add_argument('--output',required=True);args=parser.parse_args()
    result=build_geometry(args.capture,args.output);print(json.dumps({k:v for k,v in result.items() if k not in {'poses','keyframes','source_root'}}),flush=True)
