#!/usr/bin/env python3
"""Recalculate existing evidence; never rerun old planning or trajectory experiments."""
import argparse
import importlib.util
import json
from pathlib import Path

import numpy as np
import yaml

from common import describe, sha256, write_csv, write_json


def main(args):
    cfg=yaml.safe_load(args.config.read_text())
    ws=Path(cfg['workspace']);out=args.output;out.mkdir(parents=True,exist_ok=True)
    prior=cfg['prior']
    navigation=ws/prior['navigation'];official=ws/prior['official']
    plans=json.loads((navigation/'planning/paired_planning.json').read_text())
    checks=json.loads((official/'official_ispathvalid.json').read_text())
    index={(r['attempt'],r['layer']):r for r in checks}
    plan_rows=[]
    for record in plans:
        for label,layer in record['layers'].items():
            check=index[(record['attempt'],label)]
            with np.load(navigation/'planning'/layer['costmap']) as d:
                path=np.asarray(layer.get('path',[])).reshape(-1,2)
                samples=[]
                if len(path):
                    for a,b in zip(path,path[1:]):
                        steps=max(1,int(np.ceil(np.linalg.norm(b-a)/.02)))
                        samples.extend(a+(b-a)*np.arange(steps)[:,None]/steps)
                    samples.append(path[-1])
                if samples:
                    cell=np.floor((np.array(samples)-d['origin'])/float(d['resolution'])).astype(int)
                    valid=(cell[:,0]>=0)&(cell[:,0]<d['grid'].shape[1])&(cell[:,1]>=0)&(cell[:,1]<d['grid'].shape[0])
                    cost=d['grid'][cell[valid,1],cell[valid,0]]
                    unknown=int(np.sum(cost==255));inscribed=int(np.sum(cost==253))
                else:
                    unknown=inscribed=0
            rejected=check.get('rejected_single_pose_queries',[])
            plan_rows.append(dict(attempt=record['attempt']+1,layer=label,
                action_success=int(layer.get('status')==4 and layer.get('error_code')==0),
                error_code=layer.get('error_code'),path_poses=len(path),
                official_valid=int(check['path']['is_valid']),
                snapshot_verified=int(check['raw_grid_verified_before'] and check['raw_grid_verified_after']),
                unknown_centerline_samples=unknown,inscribed_centerline_samples=inscribed,
                rejected_pose_count=len(rejected),request_latency_ms=layer.get('request_latency_ms'),
                failure_reason='NO_VALID_PATH' if layer.get('error_code')==208 else
                    ('official_saved_snapshot_rejects_inscribed_end_poses' if rejected else ''),
                root_cause_confirmed=False))
    write_csv(out/'prior_planning_audit.csv',plan_rows)
    plan_summary={}
    for label in ['voxel','ground']:
        group=[r for r in plan_rows if r['layer']==label];success=[r for r in group if r['action_success']]
        plan_summary[label]=dict(requests=len(group),action_success=len(success),
            official_valid_among_success=sum(r['official_valid'] for r in success),
            errors={str(code):sum(r['error_code']==code for r in group) for code in sorted({r['error_code'] for r in group})},
            successful_paths_touching_unknown=sum(r['unknown_centerline_samples']>0 for r in success),
            request_to_result_latency_ms=describe([r['request_latency_ms'] for r in group]))
    spec=importlib.util.spec_from_file_location('old_summary',ws/'evaluation/summarize_odometry.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    trajectories={}
    for case in ['indoor_static','indoor_loop','outdoor_loop']:
        path=Path(prior['trajectories'])/case/'odometry.csv'
        trajectories[case]=dict(source=str(path),sha256=sha256(path),**module.summarize(path))
    write_json(out/'prior_trajectories_rechecked.json',trajectories)
    evidence=[]
    for root in [ws/'evaluation',ws/'src/lidar_perception',ws/'src/lidar_nav2_bringup',ws/'src/FAST_LIO']:
        for f in sorted(root.rglob('*')):
            if not f.is_file() or f.suffix not in ['.py','.cpp','.hpp','.h','.yaml','.md','.sh']:
                continue
            if any(k in f.parts for k in ['results','map_quality','.git','vendor','third_party','Log']):
                continue
            evidence.append(dict(path=str(f.relative_to(ws)),sha256=sha256(f),bytes=f.stat().st_size))
    write_csv(out/'source_inventory.csv',evidence)
    write_json(out/'prior_summary.json',dict(planning=plan_summary,trajectories=trajectories,
        official_control_results={label:dict(accepted=sum(r['probes'][label]['is_valid'] for r in checks if label in r['probes']),
                                             queries=sum(label in r['probes'] for r in checks))
                                 for label in ['free','inscribed','lethal','unknown','empty','outside']},
        verified_snapshots=sum(r['snapshot_verified'] for r in plan_rows),
        planning_source=str(navigation),official_source=str(official),
        old_report_discrepancy='Final paired CSV dropped counter is 2; early height_verified report states 0 for a different run. Do not merge runs.',
        artifact_hashes={str(f):sha256(f) for f in [navigation/'planning/paired_planning.json',official/'official_ispathvalid.json']}))
    print(json.dumps(plan_summary,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--config',type=Path,default=Path(__file__).with_name('quality.yaml'))
    p.add_argument('--output',type=Path,required=True)
    main(p.parse_args())
