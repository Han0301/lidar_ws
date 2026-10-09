#!/usr/bin/env python3
"""Real evidence analysis; consistency metrics are not detection/absolute accuracy."""
import argparse
import csv
import json
import re
from collections import Counter
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from common import describe, overlap, write_csv, write_json


def rows(path):
    with Path(path).open(encoding='utf-8', newline='') as f:
        return list(csv.DictReader(f))


def rate(events, field):
    valid = [r for r in events if int(r[field])>0]
    a = np.array([int(r[field]) for r in valid], dtype=np.int64)
    duration = (int(a[-1])-int(a[0]))/1e9 if len(a)>1 else 0
    return dict(count=len(a), excluded_zero_timestamp=len(events)-len(a), hz=(len(a)-1)/duration if duration>0 else None,
                interval_ms=describe(np.diff(a)/1e6), nonincreasing=int(np.sum(np.diff(a)<=0)))


def nearest_stamps(reference, query):
    ref = np.asarray(reference, np.int64)
    x = np.asarray(query, np.int64)
    right = np.clip(np.searchsorted(ref, x), 0, len(ref)-1)
    left = np.maximum(right-1, 0)
    return np.where(np.abs(ref[left]-x)<=np.abs(ref[right]-x), left, right)


def analyze_run(run, output, cfg):
    cap = run/'capture'
    output.mkdir(parents=True, exist_ok=True)
    status = json.loads((run/'run_status.json').read_text())
    if status.get('error'):
        raise ValueError('Incomplete replay refused: '+str(status))
    events = rows(cap/'events.csv')
    topics = {}
    for label in sorted({r['topic'] for r in events}):
        group = [r for r in events if r['topic']==label]
        topics[label] = group
    metrics = dict(mode='offline_rosbag_replay', run=str(run), observer=json.loads((cap/'status.json').read_text()),
                   topics={k: dict(sensor_stamp=rate(v, 'stamp_ns'), wall_arrival=rate(v, 'arrival_mono_ns'))
                           for k,v in topics.items()})
    # 精确扫描结束时间不同于 CustomMsg 首点时间；用最近结束时间且限制 2 ms 配对
    raw = topics['raw_lidar']
    raw_end = np.array([int(r['end_ns']) for r in raw], dtype=np.int64)
    latency = []
    for label in ['body', 'obstacles']:
        group = topics.get(label, [])
        ix = nearest_stamps(raw_end, [int(r['stamp_ns']) for r in group])
        for r, i in zip(group, ix):
            delta = abs(int(r['stamp_ns'])-int(raw[i]['end_ns']))
            latency.append(dict(stage='raw_arrival_to_'+label+'_arrival', stamp_ns=int(r['stamp_ns']),
                                raw_end_delta_ms=delta/1e6, paired=int(delta<=2000000),
                                wall_ms=(int(r['arrival_mono_ns'])-int(raw[i]['arrival_mono_ns']))/1e6,
                                ros_age_scan_end_ms=(int(r['ros_now_ns'])-int(r['stamp_ns']))/1e6))
    body = topics.get('body', [])
    body_stamps = [int(r['stamp_ns']) for r in body]
    ix = nearest_stamps(body_stamps, [int(r['stamp_ns']) for r in topics.get('obstacles', [])])
    for r,i in zip(topics.get('obstacles', []), ix):
        delta = abs(int(r['stamp_ns'])-int(body[i]['stamp_ns']))
        latency.append(dict(stage='body_arrival_to_obstacles_arrival', stamp_ns=int(r['stamp_ns']),
                            raw_end_delta_ms=delta/1e6, paired=int(delta==0),
                            wall_ms=(int(r['arrival_mono_ns'])-int(body[i]['arrival_mono_ns']))/1e6,
                            ros_age_scan_end_ms=(int(r['ros_now_ns'])-int(r['stamp_ns']))/1e6))
    write_csv(output/'latency_samples.csv', latency)
    metrics['latency'] = {}
    metrics['latency_interpretation'] = 'Observer callback arrival differences only. Different subscriptions can already be queued; negative values and deltas below core time prove executor/transport ordering. Do not call these pure compute time or verified software end-to-end latency.'
    metrics['verified_raw_to_obstacle_software_latency_ms'] = None
    for label in sorted({r['stage'] for r in latency}):
        group = [r for r in latency if r['stage']==label]
        valid = [r for r in group if r['paired']]
        metrics['latency'][label] = dict(pair_count=len(valid), unmatched=len(group)-len(valid),
                                       negative_wall_count=sum(r['wall_ms']<0 for r in valid),
                                       observed_callback_delta_ms=describe([r['wall_ms'] for r in valid]),
                                       ros_age_scan_end_ms=describe([r['ros_age_scan_end_ms'] for r in valid]))
    diag = rows(cap/'diagnostics.csv')
    processing = []
    health = Counter()
    for r in diag:
        health[r['name']+':'+r['state']] += 1
        v = json.loads(r['values_json'])
        if 'processing_ms' in v:
            processing.append(dict(state=r['state'], **{k:(int(x) if k == 'scan_stamp_ns' else float(x)) for k,x in v.items() if k != 'reference_status'}))
    write_csv(output/'processing_samples.csv', processing)
    metrics['health_messages'] = dict(health)
    if processing:
        ok = [r for r in processing if r['state']=='OK']
        metrics['perception'] = dict(processed=len(processing), ground_reference_valid=len(ok),
            gate_valid_fraction=len(ok)/len(processing), ground_uncertain=len(processing)-len(ok),
            dropped_final=int(processing[-1]['dropped']),
            processing_ms=describe([r['processing_ms'] for r in processing]),
            obstacle_points_when_gate_valid=describe([r['obstacle_points'] for r in ok]),
            clusters_when_gate_valid=describe([r['clusters'] for r in ok]),
            geometry_empty_valid_frames=sum(r['obstacle_points']==0 for r in ok),
            gate_valid_note='Internal publication gate, not segmentation accuracy')
    structural=[]
    for f in sorted(cap.glob('obstacles_*.npz')):
        with np.load(f) as d:
            xyz=d['points']
            structural.append(dict(stamp_ns=int(d['stamp_ns']),points=len(xyz),
                nonfinite_points=int(np.sum(~np.isfinite(xyz).all(axis=1))),empty=int(len(xyz)==0)))
    write_csv(output/'obstacle_structural_checks.csv',structural)
    metrics['obstacle_message_anomalies']=dict(frames=len(structural),
        empty_frames=sum(r['empty'] for r in structural),
        nonfinite_frames=sum(r['nonfinite_points']>0 for r in structural),
        nonincreasing_stamps=metrics['topics'].get('obstacles',{}).get('sensor_stamp',{}).get('nonincreasing'),
        definition='Empty output, nonfinite XYZ or nonincreasing source stamp; gate-denied frames reported separately. No semantic false positive/negative claims.')
    metrics['production_tf_wait_time_ms']=None
    metrics['production_tf_wait_note']='Pipeline exports dropped counter but no TF wait duration; observer resolved TF is a separate check.'
    log = (run/'costmap.log').read_text() if (run/'costmap.log').exists() else ''
    timing = [dict(log_unix_s=float(s), update_ms=float(t)*1000) for s,t in re.findall(
        r'\[DEBUG\] \[([0-9.]+)\].*Map update time: ([0-9.]+)', log)]
    write_csv(output/'costmap_update_samples.csv', timing)
    # 仅取 observer 收到成功更新 footprint 的稳态区间，排除启动与停止尾段
    fp = topics.get('update_footprint', [])
    metrics['map_update_time_ms'] = describe([r['update_ms'] for r in timing])
    metrics['map_update_timer_note'] = 'Nav2 native ExecutionTimer around updateMap; includes pose/TF, layers and footprint publication; excludes costmap serialization/publication'
    # 地图快照年龄只表示相对于最新已收到障碍流，不能证明地图用了该扫描
    map_age = []
    obstacles = topics.get('obstacles', [])
    obs_arrival = np.array([int(r['arrival_mono_ns']) for r in obstacles], dtype=np.int64)
    for m in topics.get('costmap', []):
        i = np.searchsorted(obs_arrival, int(m['arrival_mono_ns']), side='right')-1
        if i>=0:
            map_age.append(dict(map_stamp_ns=int(m['stamp_ns']), obstacle_stamp_ns=int(obstacles[i]['stamp_ns']),
                ros_latest_observation_age_ms=(int(m['stamp_ns'])-int(obstacles[i]['stamp_ns']))/1e6,
                wall_latest_observation_age_ms=(int(m['arrival_mono_ns'])-int(obstacles[i]['arrival_mono_ns']))/1e6))
    write_csv(output/'map_observation_age.csv', map_age)
    metrics['map_latest_observation_age'] = {k:describe([r[k] for r in map_age]) for k in
        ['ros_latest_observation_age_ms','wall_latest_observation_age_ms']}
    metrics['sensor_to_costmap_causal_latency_ms'] = None
    metrics['causal_latency_reason'] = 'Costmap header is publication clock; no consumed scan id. Latest observation age is not causal end-to-end latency.'
    metrics['costmap'] = analyze_maps(cap, output, cfg)
    gate_maps=[]
    cloud_d=[r for r in diag if r['name']=='lidar_perception.pipeline' and 'processing_ms' in json.loads(r['values_json'])]
    diag_t=np.array([int(r['stamp_ns']) for r in cloud_d],dtype=np.int64)
    for m in topics.get('costmap',[]):
        i=np.searchsorted(diag_t,int(m['stamp_ns']),side='right')-1
        if i>=0:
            row=next((r for r in map_age if r['map_stamp_ns']==int(m['stamp_ns'])),None)
            if row:
                gate_maps.append(dict(**row,latest_processing_state=cloud_d[i]['state'],
                    no_obstacle_within_500ms=int(row['ros_latest_observation_age_ms']>500)))
    write_csv(output/'ground_gate_map_age.csv',gate_maps)
    metrics['ground_gate_map_association']={state:dict(maps=sum(r['latest_processing_state']==state for r in gate_maps),
        latest_observation_age_ms=describe([r['ros_latest_observation_age_ms'] for r in gate_maps if r['latest_processing_state']==state]))
        for state in ['OK','GROUND_UNCERTAIN']}
    metrics['maps_with_obstacle_age_over_500ms']=sum(r['no_obstacle_within_500ms'] for r in gate_maps)
    write_json(output/'run_metrics.json', metrics)
    return metrics


def analyze_maps(cap, output, cfg, files=None, slow_flags=None):
    files = sorted(cap.glob('costmap_[0-9]*.npz')) if files is None else files
    if not files:
        return dict(snapshot_count=0)
    maps = []
    for f in files:
        with np.load(f) as d:
            maps.append({k:d[k].copy() for k in d.files})
    od = np.genfromtxt(cap/'odometry.csv', delimiter=',', names=True, dtype=None, encoding='utf-8')
    ot = np.asarray(od['stamp_s']*1e9, np.int64)
    positions = np.column_stack([od[k] for k in ['x','y','z']])
    quats = np.column_stack([od[k] for k in ['qx','qy','qz','qw']])
    report = []
    margin = cfg['consistency']['grid_margin_cells']
    for i in range(1,len(maps)):
        a,b = maps[i-1],maps[i]
        x,y = overlap(a,b,margin)
        if x.size==0:
            continue
        known = (x!=255)&(y!=255)
        occx,occy = x==254,y==254
        union = ((occx|occy)&known).sum()
        pair = nearest_stamps(ot, [int(a['stamp_ns']),int(b['stamp_ns'])])
        j,k = pair
        dt = (int(b['stamp_ns'])-int(a['stamp_ns']))/1e9
        speed = np.linalg.norm(positions[k]-positions[j])/dt
        rot = np.degrees((Rotation.from_quat(quats[j]).inv()*Rotation.from_quat(quats[k])).magnitude())/dt
        slow = speed<cfg['consistency']['slow_translation_mps'] and rot<cfg['consistency']['slow_rotation_degps']
        if slow_flags is not None:
            slow = bool(slow_flags[i-1])
        r = dict(stamp_ns=int(b['stamp_ns']), interval_ms=dt*1000, overlap_cells=x.size,
            known_both_cells=int(known.sum()), lethal_cells=int((b['grid']==254).sum()),
            unknown_fraction=float(np.mean(b['grid']==255)), free_fraction=float(np.mean(b['grid']==0)),
            state_change_fraction=float(np.mean(x!=y)),
            lethal_toggle_fraction=float(np.mean(occx!=occy)),
            lethal_retention_fraction=float(np.sum(occx&occy)/occx.sum()) if occx.any() else None,
            temporal_lethal_jaccard_known=float(np.sum(occx&occy&known)/union) if union else None,
            lethal_to_free_cells=int(np.sum(occx&(y==0))),
            lethal_to_unknown_cells=int(np.sum(occx&(y==255))),
            unknown_to_free_cells=int(np.sum((x==255)&(y==0))),
            translation_speed_mps=float(speed), rotation_speed_degps=float(rot), slow_sensor=int(slow),
            aba_lethal_flicker_cells=0, aba_overlap_cells=0, slow_sensor_triplet=0)
        if i>=2:
            p,a,b = maps[i-2],maps[i-1],maps[i]
            origins = np.array([z['origin'] for z in [p,a,b]])
            res = float(p['resolution'])
            shifts = np.rint((origins-origins[0])/res).astype(int)
            lo = np.max(shifts+margin,axis=0)
            hi = np.min(shifts+np.array([[z['grid'].shape[1],z['grid'].shape[0]] for z in [p,a,b]])-margin,axis=0)
            if np.all(hi>lo):
                z = [m['grid'][lo[1]-s[1]:hi[1]-s[1], lo[0]-s[0]:hi[0]-s[0]] for m,s in zip([p,a,b],shifts)]
                r['aba_lethal_flicker_cells'] = int(np.sum((z[0]==254)&(z[1]!=254)&(z[2]==254)))
                r['aba_overlap_cells'] = z[0].size
                r['slow_sensor_triplet'] = int(slow and report[-1]['slow_sensor']) if report else 0
        report.append(r)
    write_csv(output/'map_transitions.csv', report)
    summary = dict(snapshot_count=len(maps), pairs=len(report), slow_sensor_pairs=sum(r['slow_sensor'] for r in report),
                   all_route={}, slow_sensor={}, confirmed_static_environment=False,
                   anomaly_clearing_truth=None, dynamic_object_precision_recall_iou=None)
    for group, sel in [('all_route',report),('slow_sensor',[r for r in report if r['slow_sensor']])]:
        for field in ['unknown_fraction','state_change_fraction','lethal_toggle_fraction',
                      'lethal_retention_fraction','temporal_lethal_jaccard_known','lethal_to_free_cells',
                      'lethal_to_unknown_cells','unknown_to_free_cells','aba_lethal_flicker_cells']:
            sample=[r[field] for r in sel if r[field] is not None]
            if field=='aba_lethal_flicker_cells':
                sample=[r[field] for r in sel if r['aba_overlap_cells']>0 and (group=='all_route' or r['slow_sensor_triplet'])]
            summary[group][field] = describe(sample)
    fig, axes = plt.subplots(3,1,figsize=(10,8),sharex=True)
    t = np.array([r['stamp_ns'] for r in report],dtype=np.int64)
    t = (t-t[0])/1e9
    axes[0].plot(t,[r['lethal_cells'] for r in report],label='Lethal cells')
    axes[0].legend()
    axes[1].plot(t,[r['unknown_fraction'] for r in report],label='Unknown fraction')
    axes[1].legend()
    axes[2].plot(t,[r['lethal_toggle_fraction'] for r in report],label='World-cell lethal toggles')
    axes[2].scatter(t[[r['slow_sensor']==1 for r in report]],
        np.array([r['lethal_toggle_fraction'] for r in report])[[r['slow_sensor']==1 for r in report]],s=5,label='Slow sensor')
    axes[2].legend(); axes[2].set_xlabel('Replay time since first map pair (s)')
    fig.tight_layout();fig.savefig(output/'map_stability.png',dpi=160);plt.close(fig)
    pick = np.argsort([r['lethal_to_free_cells'] for r in report])[-3:]
    fig, axes = plt.subplots(2,3,figsize=(12,8))
    for col, idx in enumerate(pick):
        for row,m in enumerate(maps[idx:idx+2]):
            # 固定世界坐标显示，避免把滚动原点造成的图像位移误判为闪烁
            o=np.asarray(m['origin']);h,w=m['grid'].shape;res=float(m['resolution'])
            colors=np.zeros((h,w,3));g=m['grid']
            colors[:]=.85;colors[g==0]=1;colors[g==254]=[.85,.1,.1];colors[g==255]=[.25,.25,.25]
            colors[(g>0)&(g<254)]=[1,.7,.2]
            axes[row,col].imshow(colors,origin='lower',extent=[o[0],o[0]+w*res,o[1],o[1]+h*res])
            axes[row,col].set_title(f"{int(m['stamp_ns'])/1e9:.3f}; clear={report[idx]['lethal_to_free_cells']}")
    fig.tight_layout();fig.savefig(output/'map_clearing_cases.png',dpi=150);plt.close(fig)
    return summary


if __name__ == '__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--config',type=Path,default=Path(__file__).with_name('quality.yaml'))
    p.add_argument('--run',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    result=analyze_run(a.run,a.output,yaml.safe_load(a.config.read_text()))
    print(json.dumps({k:result[k] for k in ['perception','map_update_time_ms'] if k in result},indent=2))
