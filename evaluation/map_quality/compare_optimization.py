#!/usr/bin/env python3
"""Comparable cadence + real global maps + timing; no precision/recall claims."""
import argparse
import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml
from analyze_quality import analyze_maps, nearest_stamps
from common import describe, sha256, write_csv, write_json


def rows(file):
    return list(csv.DictReader(file.open()))


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--root',type=Path,required=True)
    p.add_argument('--config',type=Path,default=Path(__file__).with_name('quality.yaml'))
    p.add_argument('--name',default='comparison')
    p.add_argument('--profiles', nargs='+')
    p.add_argument('--cadence-s', type=float, default=0.6)
    a=p.parse_args();out=a.root/a.name;out.mkdir(exist_ok=False)
    cfg=yaml.safe_load(a.config.read_text())
    names=a.profiles or [n for n in ['baseline','fast_final' if (a.root/'fast_final/run_status.json').exists() else 'fast','ground'] if (a.root/n/'run_status.json').exists()]
    all_files={n: sorted((a.root/n/'capture').glob('costmap_[0-9]*.npz')) for n in names}
    stamps={n: np.array([int(f.stem.split('_')[1]) for f in files],dtype=np.int64) for n,files in all_files.items()}
    reference=stamps['baseline']
    # 截取三组共有区间，在同一约 0.6 s 时间网格比较，避免高频自身美化闪烁统计
    reference=reference[(reference>=max(v[0] for v in stamps.values())+int(2e9)) &
                        (reference<=min(v[-1] for v in stamps.values())-int(1e9))]
    if a.profiles:
        indices=[]
        for i,stamp in enumerate(reference):
            if not indices or stamp-reference[indices[-1]] >= int(a.cadence_s*1e9)-int(2e7):
                indices.append(i)
        reference=reference[indices]
    selected={n: nearest_stamps(stamps[n],reference) for n in names}
    valid=np.ones(len(reference),bool)
    for n in names:valid &= abs(stamps[n][selected[n]]-reference)<=int(7e7)
    common_slow=None
    matched_files={}
    for n in names:
        temporary=out/(n+'_cadence_selection');temporary.mkdir()
        matched_files[n]=[all_files[n][i] for i in selected[n][valid]]
        analyze_maps(a.root/n/'capture',temporary,cfg,matched_files[n])
        flags=np.array([int(r['slow_sensor']) for r in rows(temporary/'map_transitions.csv')],dtype=bool)
        common_slow=flags if common_slow is None else common_slow & flags
    result={};table=[];inputs={}
    for n in names:
        directory=out/n;directory.mkdir()
        files=[all_files[n][i] for i in selected[n][valid]]
        stable=analyze_maps(a.root/n/'capture',directory,cfg,files,common_slow)
        write_json(directory/'matched_snapshots.json',dict(reference_stamps_ns=reference[valid].tolist(),
                   actual_stamps_ns=stamps[n][selected[n][valid]].tolist(),max_matching_error_ms=70,
                   files=[str(f) for f in files]))
        run=json.loads((a.root/'analysis'/n/'run_metrics.json').read_text())
        diag=rows(a.root/n/'capture/diagnostics.csv')
        timing={}
        for key in ['queue_tf_ready_ms','callback_to_publish_ms']:
            values=[float(json.loads(d['values_json'])[key]) for d in diag if key in json.loads(d['values_json'])]
            timing[key]=describe(values)
        reasons={}
        for d in diag:
            v=json.loads(d['values_json'])
            if 'reference_status' in v:reasons[v['reference_status']]=reasons.get(v['reference_status'],0)+1
        result[n]=dict(run_metrics=run,matched_cadence_stability=stable,perception_ros_timing=timing,
                       reference_reasons=reasons,run_status=json.loads((a.root/n/'run_status.json').read_text()))
        maps=sorted((a.root/n/'capture').glob('global_map_*.npz'))
        if maps:
            last=np.load(maps[-1]);g=last['grid'];res=float(last['resolution']);origin=last['origin']
            integration=[float(json.loads(d['values_json'])['integration_ms']) for d in diag if d['name']=='lidar_mapping/integration']
            project=[float(json.loads(d['values_json'])['projection_publish_ms']) for d in diag if d['name']=='lidar_mapping']
            state=[json.loads(d['values_json']) for d in diag if d['name']=='lidar_mapping'][-1]
            result[n]['global_map']=dict(grid_shape=list(g.shape),resolution=res,origin=origin.tolist(),
                width_m=g.shape[1]*res,height_m=g.shape[0]*res,free_cells=int((g==0).sum()),
                occupied_cells=int((g==100).sum()),unknown_fraction=float((g<0).mean()),
                integration_ms=describe(integration),projection_publish_ms=describe(project),final_diagnostics=state,
                truth_accuracy=None,not_cross_session_localized=True)
            fig,ax=plt.subplots(figsize=(8,9));colors=np.full((*g.shape,3),.5);colors[g==0]=1;colors[g==100]=[.8,.1,.1]
            ax.imshow(colors,origin='lower',extent=[origin[0],origin[0]+g.shape[1]*res,origin[1],origin[1]+g.shape[0]*res])
            od=np.genfromtxt(a.root/n/'capture/odometry.csv',delimiter=',',names=True,dtype=None,encoding='utf-8')
            tf=next(r for r in rows(a.root/n/'capture/tf.csv') if r['parent']=='odom' and r['child']=='camera_init')
            from scipy.spatial.transform import Rotation
            xyz=np.column_stack([od[k] for k in ['x','y','z']]);R=Rotation.from_quat([float(tf[k]) for k in ['qx','qy','qz','qw']]).as_matrix()
            xyz=xyz@R.T+np.array([float(tf[k]) for k in ['x','y','z']]);ax.plot(xyz[:,0],xyz[:,1],c='#0088ff',lw=1,label='Estimated trajectory')
            ax.set(title='Session map: red occupied / white ground-supported / gray unknown',xlabel='odom x (m)',ylabel='odom y (m)');ax.legend();fig.tight_layout();fig.savefig(directory/'global_map.png',dpi=150);plt.close(fig)
        table.append(dict(profile=n,gate_valid_percent=100*run['perception']['gate_valid_fraction'],
            obstacle_hz=run['topics']['obstacles']['wall_arrival']['hz'],
            map_publish_hz=run['topics']['costmap']['wall_arrival']['hz'],
            map_interval_p95_ms=run['topics']['costmap']['wall_arrival']['interval_ms']['p95'],
            perception_p95_ms=run['perception']['processing_ms']['p95'],
            update_p95_ms=run['map_update_time_ms']['p95'],
            matched_pairs=stable['pairs'],slow_pairs=stable['slow_sensor_pairs'],
            slow_toggle_percent=100*stable['slow_sensor']['lethal_toggle_fraction']['mean'],
            slow_retention_percent=100*stable['slow_sensor']['lethal_retention_fraction']['mean'],
            slow_aba_mean=stable['slow_sensor']['aba_lethal_flicker_cells']['mean']))
        inputs[n]={str(f.relative_to(a.root)):sha256(f) for f in files}
    write_json(out/'comparison.json',result);write_csv(out/'comparison.csv',table)
    write_json(out/'manifest.json',dict(inputs=inputs,cadence='baseline ~600 ms, common ROS times +/-70 ms',
        common_slow_flags=common_slow.tolist(),no_static_environment_ground_truth=True,analyzer_sha256=sha256(Path(__file__).with_name('analyze_quality.py'))))
    fig,axes=plt.subplots(1,3,figsize=(12,3.5))
    for ax,key,title in zip(axes,['gate_valid_percent','map_publish_hz','map_interval_p95_ms'],['Ground reference valid (%)','Local map output (Hz)','Output interval P95 (ms)']):
        ax.bar([r['profile'] for r in table],[r[key] for r in table],color=['#667c99','#319a91','#a07cba'][:len(table)]);ax.set_title(title)
    fig.tight_layout();fig.savefig(out/'quality_comparison.png',dpi=160);plt.close(fig)
    print(json.dumps(table,indent=2))


if __name__=='__main__':main()
