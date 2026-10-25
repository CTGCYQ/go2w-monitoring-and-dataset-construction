"""Isolated map build worker. Only validated outputs become a published version."""
import argparse,fcntl,os,time
from pathlib import Path
from scene_store import SceneCatalog,publish_tiles
from scene_geometry import build_geometry
from scene_train import train


def build(root,sid,capture_id,build_id,steps=2200):
    catalog=SceneCatalog(root);lock=(catalog.root/'.training.lock').open('a')
    try:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:raise RuntimeError('another map training worker owns the GPU build slot')
    directory=catalog.root/'build'/build_id
    try:
        catalog.update(sid,stage='geometry',progress=25,worker_pid=os.getpid(),heartbeat=time.time())
        build_geometry(catalog.root/'raw'/capture_id,directory)
        catalog.update(sid,stage='training',progress=45,heartbeat=time.time(),geometry_id=build_id)
        result=train(directory,directory/'trained',steps=steps)
        catalog.update(sid,stage='packing',progress=85,quality={k:result[k] for k in ['heldout_psnr_db','heldout_depth_median_error_m','quality_passed']},heartbeat=time.time())
        if not result['quality_passed']:raise ValueError('held-out view acceptance failed; map was not published')
        manifest=publish_tiles(catalog,sid,directory/'trained',directory)
        return manifest
    except Exception as error:
        catalog.update(sid,stage='failed',error=str(error),heartbeat=time.time());raise
    finally:lock.close()


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',required=True);p.add_argument('--scene-id',required=True);p.add_argument('--capture-id',required=True);p.add_argument('--build-id',required=True);p.add_argument('--steps',type=int,default=2200);a=p.parse_args()
    build(a.root,a.scene_id,a.capture_id,a.build_id,a.steps)
