"""Versioned spatial Gaussian assets, compact transport and bounded scene inventory."""
import gzip,hashlib,json,math,re,shutil,threading,time,uuid
from pathlib import Path
from contextlib import contextmanager
import numpy as np
from scene_capture import atomic_json
from scene_train import export_splat

ID=re.compile(r'^[a-zA-Z0-9][a-zA-Z0-9_-]{0,63}$')


def identifier(value):
    if not isinstance(value,str) or not ID.fullmatch(value):raise ValueError('invalid asset identifier')
    return value


def safe_path(root,*parts):
    root=Path(root).resolve();target=root.joinpath(*parts).resolve()
    if not target.is_relative_to(root):raise ValueError('asset path escapes storage')
    return target


class SceneCatalog:
    def __init__(self,root):
        self.root=Path(root).resolve();self.lock=threading.RLock()
        for name in ['registry','raw','build','published']: (self.root/name).mkdir(parents=True,exist_ok=True)
    def list(self):
        records=[]
        for p in (self.root/'registry').glob('*.json'):
            try:records.append(json.loads(p.read_text(encoding='utf8')))
            except (OSError,ValueError):continue
        return sorted(records,key=lambda r:r['created_at'],reverse=True)
    def get(self,sid):
        identifier(sid);path=self.root/'registry'/f'{sid}.json'
        if not path.exists():raise FileNotFoundError('scene not found')
        return json.loads(path.read_text(encoding='utf8'))
    def create(self,name,robot_id):
        if not isinstance(name,str) or not 1<=len(name.strip())<=100:raise ValueError('场景名称须为 1 至 100 个字符')
        identifier(robot_id);sid=uuid.uuid4().hex
        record={'id':sid,'name':name.strip(),'robot_id':robot_id,'created_at':time.time(),'updated_at':time.time(),'stage':'created','progress':0,'version':None,'published':False,'camera_only':True,'navigation_ready':False,'body_extrinsics_verified':False}
        with self.lock:atomic_json(self.root/'registry'/f'{sid}.json',record)
        return record
    @contextmanager
    def record_lock(self,sid):
        path=self.root/'registry'/(identifier(sid)+'.lock')
        with path.open('a+b') as stream:
            if __import__('os').name=='nt':
                import msvcrt
                if stream.tell()==0:stream.write(b'0');stream.flush()
                stream.seek(0);msvcrt.locking(stream.fileno(),msvcrt.LK_LOCK,1)
                try:yield
                finally:stream.seek(0);msvcrt.locking(stream.fileno(),msvcrt.LK_UNLCK,1)
            else:
                import fcntl
                fcntl.flock(stream,fcntl.LOCK_EX)
                try:yield
                finally:fcntl.flock(stream,fcntl.LOCK_UN)
    def update(self,sid,**values):
        with self.lock,self.record_lock(sid):
            record=self.get(sid);record.update(values,updated_at=time.time());atomic_json(self.root/'registry'/f'{sid}.json',record);return record
    def manifest(self,sid,version=None):
        r=self.get(sid);version=identifier(version or r.get('version') or '')
        path=safe_path(self.root/'published',sid,version,'manifest.json')
        if not path.exists():raise FileNotFoundError('published scene unavailable')
        return json.loads(path.read_text(encoding='utf8'))
    def tile(self,sid,version,filename):
        identifier(sid);identifier(version)
        if not re.fullmatch(r'tile_[0-9]+_l[0-2]\.splat',filename):raise ValueError('invalid tile resource')
        manifest=self.manifest(sid,version)
        if not any(l['file']==filename for t in manifest['tiles'] for l in t['levels']):raise FileNotFoundError('tile not in manifest')
        path=safe_path(self.root/'published',sid,version,'tiles',filename+'.gz')
        if not path.is_file():raise FileNotFoundError('tile unavailable')
        return path
    def storage(self):
        values={name:sum(p.stat().st_size for p in (self.root/name).rglob('*') if p.is_file() and not p.is_symlink()) for name in ['raw','build','published']}
        values.update(free_bytes=shutil.disk_usage(self.root).free,published_limit_bytes=16*1024**3,capture_limit_bytes=256*1024**2)
        return values


def publish_tiles(catalog,sid,trained,geometry,tile_size=2.):
    identifier(sid)
    if not math.isfinite(tile_size) or not .25<=tile_size<=10:raise ValueError('invalid tile size')
    trained=Path(trained);geometry=Path(geometry);training=json.loads((trained/'training.json').read_text(encoding='utf8'))
    if not training.get('quality_passed'):raise ValueError('trained scene failed held-out appearance/depth acceptance')
    with np.load(trained/'gaussians.npz',allow_pickle=False) as raw:data={k:raw[k].copy() for k in ['means','scales','quats','colors','opacity']}
    n=len(data['means'])
    if n<10 or n>2000000:raise ValueError('gaussian count outside publication limits')
    for k,v in data.items():
        if len(v)!=n or not np.isfinite(v).all():raise ValueError('invalid Gaussian data')
    if data['means'].shape!=(n,3) or data['scales'].shape!=(n,3) or data['quats'].shape!=(n,4):raise ValueError('invalid Gaussian dimensions')
    if data['colors'].shape!=(n,3) or data['opacity'].shape!=(n,):raise ValueError('invalid appearance dimensions')
    if (np.linalg.norm(data['quats'],axis=1)<1e-6).any():raise ValueError('invalid Gaussian orientation')
    data['quats']/=np.linalg.norm(data['quats'],axis=1,keepdims=True)
    if (data['scales']<=0).any() or np.max(data['scales'])>1.:raise ValueError('invalid Gaussian extent')
    sizes=catalog.storage()
    if sizes['free_bytes']<2*1024**3 or sizes['published']+n*48>sizes['published_limit_bytes']:raise ValueError('map storage quota exceeded')
    version=uuid.uuid4().hex;target=catalog.root/'published'/sid/version;(target/'tiles').mkdir(parents=True,exist_ok=False)
    cells=np.floor(data['means']/tile_size).astype(np.int32);keys,inverse=np.unique(cells,axis=0,return_inverse=True);tiles=[];total_bytes=0
    for ti,key in enumerate(keys):
        indices=np.flatnonzero(inverse==ti);xyz=data['means'][indices];origin=(key.astype(float)+.5)*tile_size
        extent=data['scales'][indices].max(axis=1)*3
        bounds=[(xyz-extent[:,None]).min(axis=0).tolist(),(xyz+extent[:,None]).max(axis=0).tolist()]
        tile={'id':ti,'bounds':bounds,'origin':origin.tolist(),'levels':[]}
        for level,voxel in enumerate([0.,.05,.12]):
            idx=indices
            if voxel:
                order=idx[np.argsort(-data['opacity'][idx])];_,pick=np.unique(np.floor(data['means'][order]/voxel).astype(np.int64),axis=0,return_index=True);idx=order[pick]
            filename=f'tile_{ti}_l{level}.splat';plain=target/'tiles'/filename
            # Local coordinates preserve precision; the manifest carries the block transform.
            export_splat(plain,data['means'][idx]-origin,data['scales'][idx],data['quats'][idx],data['colors'][idx],data['opacity'][idx])
            body=plain.read_bytes();packed=gzip.compress(body,compresslevel=6,mtime=0);plain.with_suffix('.splat.gz').write_bytes(packed);plain.unlink()
            total_bytes+=len(packed)
            if sizes['published']+total_bytes>sizes['published_limit_bytes']:raise ValueError('published storage quota exceeded')
            tile['levels'].append({'level':level,'file':filename,'gaussians':len(idx),'decoded_bytes':len(body),'bytes':len(packed),'sha256':hashlib.sha256(packed).hexdigest(),'decoded_sha256':hashlib.sha256(body).hexdigest(),'geometric_error_m':voxel})
        tiles.append(tile)
    pose=json.loads((geometry/'geometry.json').read_text(encoding='utf8'))
    manifest={'version':version,'scene_id':sid,'format':'splat32+gzip','units':'metres','coordinate_system':'first_camera_optical_RDF','viewer_rotation_x_radians':math.pi,'camera_only':True,'body_extrinsics_verified':False,'navigation_ready':False,'created_at':time.time(),'gaussian_count':n,'bounds':[data['means'].min(axis=0).tolist(),data['means'].max(axis=0).tolist()],'tiles':tiles,'published_bytes':total_bytes,'uncompressed_full_lod_bytes':n*32,'keyframe_poses':[r['T_world_camera'] for r in pose['keyframes']],'quality':{k:training[k] for k in ['heldout_psnr_db','heldout_depth_median_error_m','quality_passed']},'recommended':{'load_radius_m':12,'render_radius_m':10,'cache_bytes':64*1024**2,'gaussian_budget':300000,'download_concurrency':2}}
    atomic_json(target/'manifest.json',manifest)
    catalog.update(sid,version=version,published=True,stage='ready',progress=100,quality=manifest['quality'],gaussian_count=n,published_bytes=total_bytes,geometry_id=geometry.name)
    return manifest
