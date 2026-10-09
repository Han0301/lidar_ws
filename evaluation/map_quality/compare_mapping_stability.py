"""Replay-map stability on common ~600 ms source stamps and common slow samples."""
import argparse
import csv
from pathlib import Path
import numpy as np
import yaml
from analyze_quality import analyze_maps
from common import write_json

p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);a=p.parse_args()
cfg=yaml.safe_load(Path(__file__).with_name('quality.yaml').read_text())
profiles=[p for p in sorted((a.root/'mapping_core').iterdir()) if (p/'resources.json').exists()]
base=list(csv.DictReader((a.root/'mapping_core/full/metrics.csv').open()));selected=[]
for i,r in enumerate(base):
    if not selected or int(r['stamp_ns'])-int(base[selected[-1]]['stamp_ns'])>=580000000:selected.append(i)
result={};common=None
for profile in sorted(profiles,key=lambda p:p.name!='full'):
    metrics=list(csv.DictReader((profile/'metrics.csv').open()))
    cap=profile/'stability_capture';cap.mkdir(exist_ok=True)
    od=cap/'odometry.csv'
    if not od.exists():od.symlink_to((a.root/'baseline/capture/odometry.csv').resolve())
    files=[]
    for index in selected:
        r=metrics[index];stamp=r['stamp_ns']
        raw=np.fromfile(profile/(stamp+'.grid'),dtype=np.int8).reshape(int(r['height']),int(r['width']))
        costs=np.full(raw.shape,255,np.uint8);costs[raw==0]=0;costs[raw==100]=254
        file=cap/f'costmap_{stamp}.npz';files.append(file)
        np.savez_compressed(file,grid=costs,origin=[float(r['origin_x']),float(r['origin_y'])],resolution=.1,stamp_ns=int(stamp),frame='odom')
    out=profile/'stability';out.mkdir(exist_ok=True)
    result[profile.name]=analyze_maps(cap,out,cfg,files,common)
    if common is None:common=np.array([int(r['slow_sensor']) for r in csv.DictReader((out/'map_transitions.csv').open())],bool)
write_json(a.root/'mapping_stability.json',dict(source_stamps_ns=[int(base[i]['stamp_ns']) for i in selected],
    common_slow_flags=common.tolist(),profiles=result,cadence='common baseline integrated scans ~600 ms; same threshold/margins as compare_optimization',static_truth=False))
