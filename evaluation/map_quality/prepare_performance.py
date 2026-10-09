"""Prepare deterministic inputs from full-intensity observer captures and integrated scan ids."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from common import sha256, write_json

p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
a=p.parse_args();a.output.mkdir(exist_ok=False)
cap=a.run/'capture';mapping=a.output/'mapping';mapping.mkdir();perception=a.output/'perception';perception.mkdir()
diag=list(csv.DictReader((cap/'diagnostics.csv').open()))
integrated=[int(json.loads(r['values_json'])['scan_stamp_ns']) for r in diag if r['name']=='lidar_mapping/integration']
od=list(csv.DictReader((cap/'odometry.csv').open()));od={round(float(r['stamp_s'])*1e9):r for r in od}
tfs=list(csv.DictReader((cap/'tf.csv').open()))
fixed=next(r for r in tfs if r['parent']=='odom' and r['child']=='camera_init')
R=Rotation.from_quat([float(fixed[k]) for k in ['qx','qy','qz','qw']]).as_matrix()
t=np.array([float(fixed[k]) for k in ['x','y','z']])
# 捕获的动态 sensor TF 保留精确整数时间，避免 CSV 秒浮点反推失配
sensor={int(r['stamp_ns']):r for r in tfs if r['parent']=='odom' and r['child']=='nav_sensor'}
manifest={}
for stamp in integrated:
    row=sensor[stamp];origin=np.array([float(row[k]) for k in ['x','y','z']],dtype='<f4')
    arrays=[]
    for label in ['clearing','obstacles','ground']:
        file=cap/f'{label}_{stamp}.npz'
        d=np.load(file);local=np.asarray(d['points'],np.float32)
        # 与 ROS 节点一致，用 float 矩阵和坐标变换
        rot=np.asarray(d['rotation'],np.float32);trans=np.asarray(d['translation'],np.float32)
        arrays.append(np.asarray(local@rot.T+trans,dtype='<f4'))
        manifest[str(file)]=sha256(file)
    with (mapping/f'{stamp}.bin').open('wb') as f:
        f.write(origin.tobytes());f.write(np.array([len(x) for x in arrays],dtype='<u4').tobytes())
        for x in arrays:f.write(x.tobytes())
for file in sorted(cap.glob('body_*.npz')):
    d=np.load(file);stamp=int(d['stamp_ns'])
    # body 扫描由相同时间戳 Odometry 提供姿态，容忍浮点秒表示误差而不跨扫描
    near=min(od,key=lambda s:abs(s-stamp))
    if abs(near-stamp)>2000:raise ValueError('Missing scan odometry: '+str(stamp))
    row=od[near];world=R@Rotation.from_quat([float(row[k]) for k in ['qx','qy','qz','qw']]).as_matrix()
    yaw=np.arctan2(world[1,0],world[0,0]);level=Rotation.from_euler('z',-yaw).as_matrix()@world
    if 'intensity' not in d:raise ValueError('True intensity required')
    points=np.column_stack([d['points'],d['intensity']]).astype('<f4')
    with (perception/f'{stamp}.bin').open('wb') as f:
        f.write(level.astype('<f4').tobytes());f.write(points.tobytes())
write_json(a.output/'manifest.json',dict(source=str(a.run.resolve()),mapping_frames=len(integrated),
    perception_frames=len(list(perception.glob('*.bin'))),mapping_input_sha256={f.name:sha256(f) for f in mapping.glob('*.bin')},
    real_intensity=True,float_transform_reconstruction=True,source_files=manifest))
