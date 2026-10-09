"""Same scan-id obstacle geometry across ROS runs; LIO output is not assumed identical."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial import cKDTree
from common import describe,write_csv,write_json

p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--candidate',default='optimized_final');a=p.parse_args()
base=a.root/'baseline/capture';candidate=a.root/a.candidate/'capture'
stamps=lambda cap:{int(f.stem.split('_')[1]):f for f in cap.glob('obstacles_*.npz')}
x,y=stamps(base),stamps(candidate);table=[]
for stamp in sorted(x.keys()&y.keys()):
    d,e=np.load(x[stamp]),np.load(y[stamp])
    # 比较同一扫描的调平输出坐标；独立 LIO 运行的位姿仍可能略有差异
    b,c=d['points'],e['points']
    if not len(b) or not len(c):continue
    bc=cKDTree(c).query(b)[0];cb=cKDTree(b).query(c)[0]
    table.append(dict(stamp_ns=stamp,baseline_points=len(b),candidate_points=len(c),
        symmetric_nn_p95_m=max(float(np.percentile(bc,95)),float(np.percentile(cb,95))),
        symmetric_nn_max_m=max(float(bc.max()),float(cb.max())),
        bounds_max_difference_m=float(np.max(abs(np.r_[b.min(0),b.max(0)]-np.r_[c.min(0),c.max(0)])))))
write_csv(a.root/'ros_obstacle_geometry.csv',table)
objects={}
for name,cap in [('baseline',base),(a.candidate,candidate)]:
    groups={}
    for r in csv.DictReader((cap/'objects.csv').open()):groups.setdefault(int(r['stamp_ns']),[]).append([float(r[k]) for k in ['x','y','z','sx','sy','sz']])
    objects[name]=groups
box=[]
for stamp in sorted(objects['baseline'].keys()&objects[a.candidate].keys()):
    b,c=np.asarray(objects['baseline'][stamp]),np.asarray(objects[a.candidate][stamp])
    distance=np.linalg.norm(b[:,None,:3]-c[None,:,:3],axis=2)
    if stamp not in x or stamp not in y:continue
    i,j=linear_sum_assignment(np.where(distance<=.30,distance,1e6))
    valid=distance[i,j]<=.30;i,j=i[valid],j[valid]
    box.append(dict(stamp_ns=stamp,baseline_boxes=len(b),candidate_boxes=len(c),paired_boxes=len(i),
        baseline_unmatched_boxes=len(b)-len(i),candidate_unmatched_boxes=len(c)-len(j),
        max_center_difference_m=float(distance[i,j].max()) if len(i) else None,
        max_size_difference_m=float(abs(b[i,3:]-c[j,3:]).max()) if len(i) else None))
write_csv(a.root/'ros_box_geometry.csv',box)
write_json(a.root/'ros_geometry.json',dict(obstacle_pairs=len(table),boxes_paired_frames=len(box),
    baseline_unmatched_obstacle_scans=len(x.keys()-y.keys()),candidate_unmatched_obstacle_scans=len(y.keys()-x.keys()),
    obstacle_metrics={k:describe([r[k] for r in table]) for k in ['symmetric_nn_p95_m','symmetric_nn_max_m','bounds_max_difference_m']},
    box_metrics={k:describe([r[k] for r in box if r[k] is not None]) for k in ['max_center_difference_m','max_size_difference_m']},
    box_pair_rule='Both ground-reference gates valid; optimal center assignment within 0.30 m; unmatched boxes counted separately',
    baseline_unmatched_boxes=sum(r['baseline_unmatched_boxes'] for r in box),candidate_unmatched_boxes=sum(r['candidate_unmatched_boxes'] for r in box),
    differing_box_count_frames=sum(r['baseline_boxes']!=r['candidate_boxes'] for r in box),
    semantic_accuracy_truth=False,strict_same_input_evidence='perception_paired.json'))
