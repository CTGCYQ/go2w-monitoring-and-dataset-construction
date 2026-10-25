"""Real differentiable 3DGS optimization on synchronized RGB-D and measured camera poses."""
import argparse,gzip,json,math,os,time
from pathlib import Path
import numpy as np
from scene_capture import atomic_json


def export_splat(path,means,scales,quats,colors,opacity):
    count=len(means);result=np.empty((count,32),dtype=np.uint8)
    result[:,:12]=np.ascontiguousarray(means,dtype='<f4').view(np.uint8).reshape(count,12)
    result[:,12:24]=np.ascontiguousarray(scales,dtype='<f4').view(np.uint8).reshape(count,12)
    result[:,24:27]=np.clip(colors*255,0,255).astype(np.uint8)
    result[:,27]=np.clip(opacity*255,0,255).astype(np.uint8)
    result[:,28:32]=np.clip(quats*128+128,0,255).astype(np.uint8)
    Path(path).write_bytes(result.tobytes())


def train(geometry,output,steps=2200,width=640):
    import cv2
    from scene_geometry import load_frame
    import torch
    from gsplat import rasterization
    torch.manual_seed(20261023);np.random.seed(20261023);torch.set_num_threads(4)
    if not torch.cuda.is_available():raise RuntimeError('CUDA training device unavailable')
    torch.cuda.set_per_process_memory_fraction(.30)
    geometry=Path(geometry);output=Path(output);output.mkdir(parents=True,exist_ok=True)
    meta=json.loads((geometry/'geometry.json').read_text(encoding='utf8'))
    with np.load(geometry/'geometry.npz',allow_pickle=False) as initial:
        means0=torch.tensor(initial['means'],device='cuda');colors0=torch.tensor(initial['colors'],device='cuda').clamp(.001,.999)
    n=len(means0)
    means=torch.nn.Parameter(means0.clone());quats=torch.nn.Parameter(torch.tensor([1.,0,0,0],device='cuda').repeat(n,1))
    scales=torch.nn.Parameter(torch.full((n,3),math.log(.018),device='cuda'))
    color_logits=torch.nn.Parameter(torch.logit(colors0));opacity_logits=torch.nn.Parameter(torch.full((n,),1.5,device='cuda'))
    optimizer=torch.optim.Adam([{'params':[means],'lr':.0001},{'params':[quats],'lr':.001},{'params':[scales],'lr':.003},{'params':[color_logits],'lr':.012},{'params':[opacity_logits],'lr':.005}],eps=1e-12)
    views=[]
    for i,keyframe in enumerate(meta['keyframes']):
        factor=width/keyframe['record']['header']['width'];rgb,depth,K=load_frame(meta['source_root'],keyframe['record'],factor)
        views.append({'rgb':torch.tensor(rgb[:,:,::-1].copy()/255.,device='cuda',dtype=torch.float32),'depth':torch.tensor(depth,device='cuda'),'K':torch.tensor(K,device='cuda',dtype=torch.float32)[None],'view':torch.tensor(np.linalg.inv(np.asarray(keyframe['T_world_camera'])),device='cuda',dtype=torch.float32)[None],'holdout':i%5==4,'index':i})
    training=[v for v in views if not v['holdout']];heldout=[v for v in views if v['holdout']]
    if len(training)<4 or not heldout:raise ValueError('need distinct training and held-out viewpoints')
    height=views[0]['rgb'].shape[0]
    def render(v):
        return rasterization(means,torch.nn.functional.normalize(quats,dim=-1),scales.exp(),opacity_logits.sigmoid(),color_logits.sigmoid(),v['view'],v['K'],width,height,render_mode='RGB+ED',near_plane=.2,far_plane=10.,packed=True,rasterize_mode='antialiased')
    start=time.monotonic();history=[]
    for step in range(steps):
        v=training[np.random.randint(len(training))];optimizer.zero_grad(set_to_none=True)
        image,alpha,_=render(v);pred=image[0,:,:,:3];z=image[0,:,:,3]
        mask=(v['depth']>.3)&(v['depth']<7)&(alpha[0,:,:,0]>.5)
        rgb_loss=(pred-v['rgb']).abs().mean()
        depth_loss=(z[mask]-v['depth'][mask]).abs().clamp(max=.3).mean() if mask.any() else rgb_loss*0
        loss=rgb_loss+.12*depth_loss+(.02*(means-means0).square().mean())
        loss.backward();optimizer.step()
        with torch.no_grad():
            means.copy_(torch.maximum(torch.minimum(means,means0+.04),means0-.04));scales.clamp_(math.log(.003),math.log(.07));opacity_logits.clamp_(-5,6)
        if step%100==0 or step==steps-1:
            value={'step':step,'steps':steps,'rgb_l1':float(rgb_loss.detach()),'depth_l1_m':float(depth_loss.detach()),'elapsed_s':round(time.monotonic()-start,2)}
            history.append(value);atomic_json(output/'training_progress.json',value);print(json.dumps(value),flush=True)
    metrics=[]
    with torch.no_grad():
        for v in views:
            image,alpha,_=render(v);rgb=image[0,:,:,:3].clamp(0,1);mse=float((rgb-v['rgb']).square().mean());mask=(v['depth']>.3)&(v['depth']<7)&(alpha[0,:,:,0]>.7)
            metrics.append({'keyframe':v['index'],'holdout':v['holdout'],'psnr_db':-10*math.log10(max(mse,1e-12)),'depth_median_error_m':float(torch.median((image[0,:,:,3][mask]-v['depth'][mask]).abs())) if mask.any() else None})
            if v['holdout']:cv2.imwrite(str(output/f"heldout_{v['index']:03d}.jpg"),(rgb.cpu().numpy()[:,:,::-1]*255).astype(np.uint8))
        q=torch.nn.functional.normalize(quats,dim=-1).cpu().numpy();m=means.cpu().numpy();s=scales.exp().cpu().numpy();c=color_logits.sigmoid().cpu().numpy();o=opacity_logits.sigmoid().cpu().numpy();keep=o>.04
        np.savez_compressed(output/'gaussians.npz',means=m[keep],scales=s[keep],quats=q[keep],colors=c[keep],opacity=o[keep])
        export_splat(output/'scene.splat',m[keep],s[keep],q[keep],c[keep],o[keep])
        peak=torch.cuda.max_memory_allocated()/1048576
    held=[v for v in metrics if v['holdout']];quality=float(np.mean([v['psnr_db'] for v in held]));depth_values=[v['depth_median_error_m'] for v in held if v['depth_median_error_m'] is not None]
    report={'status':'trained','algorithm':'gsplat-1.5.3/RGB-D/optimized-3D-covariance','gaussian_count':int(keep.sum()),'iterations':steps,'heldout_psnr_db':quality,'heldout_depth_median_error_m':float(np.mean(depth_values)) if depth_values else None,'peak_cuda_allocated_mib':round(peak,1),'elapsed_s':round(time.monotonic()-start,2),'width':width,'height':height,'metrics':metrics,'history':history,'quality_passed':quality>=20 and bool(depth_values) and float(np.mean(depth_values))<=.12,'coordinate_system':meta['coordinate_system'],'camera_only':True,'navigation_ready':False}
    atomic_json(output/'training.json',report);print(json.dumps({k:v for k,v in report.items() if k not in {'metrics','history'}}),flush=True);return report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--geometry',required=True);p.add_argument('--output',required=True);p.add_argument('--steps',type=int,default=2200);p.add_argument('--width',type=int,default=640);a=p.parse_args();train(a.geometry,a.output,a.steps,a.width)
