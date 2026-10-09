"""Resource, phase and pairing evidence; no semantic accuracy or causal latency claims."""
import argparse
import csv
import json
from collections import Counter
from pathlib import Path
import numpy as np
from scipy.ndimage import label
from common import describe, write_json, write_csv


def rows(file):
    return list(csv.DictReader(file.open()))


def connectivity(grid):
    labels,n=label(grid==0)
    sizes=np.bincount(labels.ravel())[1:]
    occupied=np.argwhere(grid==100)
    return dict(free_cells=int((grid==0).sum()),occupied_cells=int((grid==100).sum()),
        unknown_fraction=float((grid<0).mean()),free_components=int(n),
        largest_free_component_cells=int(sizes.max()) if sizes.size else 0,
        occupied_index_bounds=[occupied.min(axis=0).tolist(),occupied.max(axis=0).tolist()] if len(occupied) else None)


def online(run):
    diag=rows(run/'capture/diagnostics.csv')
    groups={}
    for r in diag:
        v=json.loads(r['values_json'])
        for k,x in v.items():
            if k.endswith('_ms'):
                groups.setdefault(r['name']+'/'+k,[]).append(float(x))
    timings={k:describe(v) for k,v in groups.items()}
    events=rows(run/'capture/events.csv')
    topic_stamps={k:{int(r['stamp_ns']) for r in events if r['topic']==k} for k in ['ground','obstacles','clearing']}
    frames=set.union(*topic_stamps.values());masks=Counter();missing=[]
    perception={int(v['scan_stamp_ns']):r['state'] for r in diag if r['name']=='lidar_perception.pipeline'
                for v in [json.loads(r['values_json'])] if 'scan_stamp_ns' in v}
    for stamp in sorted(frames):
        absent=[k for k,v in topic_stamps.items() if stamp not in v]
        if absent:
            mask='+'.join(absent);masks[mask]+=1
            missing.append(dict(stamp_ns=stamp,missing=mask,perception_state=perception.get(stamp,'NOT_OBSERVED')))
    write_csv(run/'pairing_observer.csv',missing)
    mapper=[json.loads(r['values_json']) for r in diag if r['name']=='lidar_mapping']
    node_pairs=[dict(**json.loads(r['values_json']),perception_state=perception.get(int(json.loads(r['values_json'])['scan_stamp_ns']),'NOT_OBSERVED')) for r in diag if r['name']=='lidar_mapping/pairing']
    if node_pairs:write_csv(run/'pairing_mapper.csv',node_pairs)
    return dict(mapper_pairing_samples=len(node_pairs),mapper_pairing_gate_correlated=sum(r['perception_state']=='GROUND_UNCERTAIN' for r in node_pairs),
        mapper_pairing_non_gate=[r for r in node_pairs if r['perception_state']!='GROUND_UNCERTAIN'],
        timings=timings,resources=json.loads((run/'resources.json').read_text()),
        observer_missing_topic_masks=dict(masks),observer_gate_correlated_missing=sum(r['perception_state']=='GROUND_UNCERTAIN' for r in missing),
        observer_unexplained_missing=[r for r in missing if r['perception_state']!='GROUND_UNCERTAIN'],
        mapper_final=mapper[-1],gate_frames=Counter(perception.values()),
        pairing_note='Observer and mapper have independent QoS/queues; missing masks diagnose observed associations, not network loss')


def core(root):
    profiles=[p for p in sorted(root.iterdir()) if (p/'metrics.csv').exists()]
    baseline=root/'full';base=rows(baseline/'metrics.csv');out={}
    for profile in profiles:
        metrics=rows(profile/'metrics.csv');changes=[];exact=0;geo=None;previous=None;toggle=[]
        for r,b in zip(metrics,base):
            g=np.fromfile(profile/(r['stamp_ns']+'.grid'),dtype=np.int8).reshape(int(r['height']),int(r['width']))
            ref=np.fromfile(baseline/(b['stamp_ns']+'.grid'),dtype=np.int8).reshape(int(b['height']),int(b['width']))
            # 基准与候选图幅可不同，固定世界坐标并集对齐，未覆盖仍未知
            origins=np.rint(np.array([[float(r['origin_x']),float(r['origin_y'])],[float(b['origin_x']),float(b['origin_y'])]])/.1).astype(int)
            lo=origins.min(axis=0);hi=np.maximum(origins[0]+[g.shape[1],g.shape[0]],origins[1]+[ref.shape[1],ref.shape[0]])
            grids=[]
            for origin,raw in zip(origins,[g,ref]):
                dest=np.full((hi[1]-lo[1],hi[0]-lo[0]),-1,np.int8);xy=origin-lo
                dest[xy[1]:xy[1]+raw.shape[0],xy[0]:xy[0]+raw.shape[1]]=raw;grids.append(dest)
            a,z=grids;equal=int(np.array_equal(a,z));exact+=equal
            changes.append(dict(stamp_ns=r['stamp_ns'],different_cells=int((a!=z).sum()),
                occupied_to_free=int(((z==100)&(a==0)).sum()),occupied_to_unknown=int(((z==100)&(a<0)).sum()),
                free_to_occupied=int(((z==0)&(a==100)).sum()),unknown_to_free=int(((z<0)&(a==0)).sum()),
                occupied_iou=float(((a==100)&(z==100)).sum()/((a==100)|(z==100)).sum()) if ((a==100)|(z==100)).any() else None))
            geo=connectivity(g)
        write_csv(profile/'quality_vs_full.csv',changes)
        stages={k:describe([float(r[k]) for r in metrics if int(r['integrated']) or k=='project_ms']) for k in ['integration_ms','project_ms','ray_keys_ms','tree_update_ms','inner_occupancy_ms','projection_maintenance_ms','prune_ms']}
        out[profile.name]=dict(frames=len(metrics),integrated=sum(int(r['integrated']) for r in metrics),exact_grid_frames=exact,
            performance=stages,resources=json.loads((profile/'resources.json').read_text()),final_nodes=int(metrics[-1]['nodes']),
            final_geometry=geo,final_vs_full=changes[-1],mean_different_cells=float(np.mean([r['different_cells'] for r in changes])))
    return out


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);a=p.parse_args()
    result={name:online(a.root/name) for name in ['baseline','optimized','optimized_final'] if (a.root/name/'run_status.json').exists()}
    if (a.root/'mapping_core').exists():result['mapping_core']=core(a.root/'mapping_core')
    write_json(a.root/'performance.json',result)
    write_csv(a.root/'phase_performance.csv',[dict(profile=name,phase=phase,**stats) for name,r in result.items() if name!='mapping_core' for phase,stats in r['timings'].items()])
    print(json.dumps({n:dict(resources=r['resources'],mapper_final=r['mapper_final'],observer_missing_topic_masks=r['observer_missing_topic_masks']) for n,r in result.items() if n!='mapping_core'},indent=2))
