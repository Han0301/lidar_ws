#!/usr/bin/env python3
"""Planar geometry and repeat observations without an external reference map."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from common import describe, fit_plane, read_pcd, residual_metrics, rotation, write_csv, write_json


def read_scans(capture):
    od = np.genfromtxt(capture/'odometry.csv',delimiter=',',names=True)
    times = od['stamp_s']
    xyz = np.column_stack([od[k] for k in ['x','y','z']])
    q = np.column_stack([od[k] for k in ['qx','qy','qz','qw']])
    scans = []
    unmatched = 0
    files = sorted(capture.glob('body_*.npz'))
    legacy = not files
    if legacy:
        files = sorted(capture.glob('scan_*.npz'))
    for f in files:
        with np.load(f) as d:
            t = float(d['stamp']) if legacy else int(d['stamp_ns'])/1e9
            i = int(np.argmin(abs(times-t)))
            if abs(times[i]-t)>.002:
                unmatched += 1
                continue
            p = d['points'][:,:3].astype(float)
            p = p[np.isfinite(p).all(axis=1)]
            scans.append(dict(stamp=t,points=p@rotation(q[i]).T+xyz[i],source=f.name))
    return scans, unmatched


def select_roi(p, roi):
    u = p @ np.asarray(roi['basis']).T
    return ((u[:,0]>=roi['bounds'][0][0])&(u[:,0]<=roi['bounds'][0][1])&
            (u[:,1]>=roi['bounds'][1][0])&(u[:,1]<=roi['bounds'][1][1])&
            (np.abs(p@np.asarray(roi['normal'])+roi['d'])<=roi['normal_band_m']))


def discover_rois(points, cfg):
    pcfg = cfg['plane']
    _, ix = np.unique(np.floor(points/pcfg['discovery_voxel_m']).astype(np.int64),axis=0,return_index=True)
    reduced = points[ix]
    rois = []
    jobs=[]
    near=reduced[(np.linalg.norm(reduced[:,:2],axis=1)<8)&(reduced[:,2]<.25)&(reduced[:,2]>-3.5)]
    near_above=reduced[(np.linalg.norm(reduced[:,:2],axis=1)<12)&(reduced[:,2]>-.2)&(reduced[:,2]<3.5)]
    for label, initial, count, kind in [('near_start_horizontal',near,1,'horizontal'),
                                       ('near_start_vertical',near_above,3,'vertical'),
                                       ('horizontal',reduced,pcfg['max_planes'],'horizontal'),
                                       ('vertical',reduced,4,'vertical')]:
      remaining=initial.copy()
      for i in range(count):
        settings=dict(pcfg,normal_kind=kind)
        fit = fit_plane(remaining,settings,pcfg['seed']+i)
        if fit is None:
            break
        n,d,signed,mask = fit
        inliers = remaining[mask]
        axis = np.eye(3)[np.argmin(np.abs(n))]
        u = np.cross(n,axis);u/=np.linalg.norm(u)
        v = np.cross(n,u)
        basis = np.array([u,v])
        coords = inliers@basis.T
        lo,hi = np.quantile(coords,pcfg['roi_tangent_quantiles'],axis=0)
        # 选择空间支撑最多的固定 4 m 切平面网格，而非残差最小的位置
        extent=pcfg['max_patch_extent_m']
        tiles=np.floor(coords/extent).astype(np.int64)
        keys,counts=np.unique(tiles,axis=0,return_counts=True)
        center=(keys[np.argmax(counts)]+.5)*extent
        half=pcfg['max_patch_extent_m']/2
        lo=np.maximum(lo,center-half);hi=np.minimum(hi,center+half)
        roi=dict(id=f'{label}_{i:02d}',normal=n.tolist(),d=d,basis=basis.tolist(),
                 bounds=np.column_stack([lo,hi]).tolist(),normal_band_m=pcfg['roi_normal_band_m'],
                 automatic_geometric_candidate=True, semantic_truth=None,
                 discovery_inliers=len(inliers))
        if min(hi-lo)>.5:
            rois.append(roi)
        remaining=remaining[~mask]
    return rois


def obstacle_roi_metrics(capture, rois, cfg, output):
    # perception_sensor → odom 来自采集时同一扫描 TF；odom → camera_init 是固定重力旋转
    tf_rows = list(csv.DictReader((capture/'tf.csv').open()))
    static = [r for r in tf_rows if r['parent']=='odom' and r['child']=='camera_init' and r['static']=='1']
    if not static:
        return dict(reason='No recorded fixed odom/camera_init transform')
    level=rotation([float(static[-1][k]) for k in ['qx','qy','qz','qw']])
    records=[]
    # 只选择法向近垂直的结构候选；没有身份或静态标注，不把每帧簇编号当跟踪 ID
    candidates=[r for r in rois if abs(r['normal'][2])<.35]
    for f in sorted(capture.glob('obstacles_*.npz')):
        with np.load(f) as d:
            p=(d['points']@d['rotation'].T+d['translation'])@level
            t=int(d['stamp_ns'])/1e9
            for roi in candidates:
                s=p[select_roi(p,roi)]
                if len(s)<cfg['consistency']['min_roi_points']:
                    continue
                signed=s@np.asarray(roi['normal'])+roi['d']
                uv=s@np.asarray(roi['basis']).T
                lo,hi=np.quantile(uv,[.05,.95],axis=0)
                records.append(dict(roi=roi['id'],stamp=t,points=len(s),
                    center_x=float(s[:,0].mean()),center_y=float(s[:,1].mean()),center_z=float(s[:,2].mean()),
                    signed_distance_median_m=float(np.median(signed)),
                    span_u_m=float(hi[0]-lo[0]),span_v_m=float(hi[1]-lo[1])))
    write_csv(output/'obstacle_roi_frames.csv',records)
    summary={}
    pairs=[]
    for roi in candidates:
        group=[r for r in records if r['roi']==roi['id']]
        if not group:
            continue
        for a,b in zip(group,group[1:]):
            if b['stamp']-a['stamp']>cfg['consistency']['max_adjacent_gap_s']:
                continue
            pairs.append(dict(roi=roi['id'],stamp=b['stamp'],dt_s=b['stamp']-a['stamp'],
                centroid_delta_m=float(np.linalg.norm([b[k]-a[k] for k in ['center_x','center_y','center_z']])),
                normal_median_delta_m=abs(b['signed_distance_median_m']-a['signed_distance_median_m']),
                span_u_delta_m=abs(b['span_u_m']-a['span_u_m']),span_v_delta_m=abs(b['span_v_m']-a['span_v_m'])))
        g=[r for r in pairs if r['roi']==roi['id']]
        summary[roi['id']]=dict(frames=len(group),adjacent_pairs=len(g),confirmed_static_object=False,
            metrics={k:describe([r[k] for r in g]) for k in ['centroid_delta_m','normal_median_delta_m','span_u_delta_m','span_v_delta_m']})
    write_csv(output/'obstacle_roi_pairs.csv',pairs)
    return summary


def analyze(capture, output, cfg, map_file=None, roi_file=None):
    output.mkdir(parents=True,exist_ok=True)
    scans,unmatched=read_scans(capture) if capture else ([],0)
    if map_file:
        if map_file.suffix=='.pcd':
            points=read_pcd(map_file)
        else:
            with np.load(map_file) as d:
                points=d['points'].astype(float)
        source=str(map_file)
    elif scans:
        points=np.concatenate([s['points'] for s in scans])
        source='0.5 s sampled deskewed scan accumulation in camera_init; not the internal ikd-tree map'
    else:
        raise ValueError('Map or paired scans required')
    rois=json.loads(roi_file.read_text()) if roi_file else discover_rois(points,cfg)
    write_json(output/'rois.json',rois)
    table=[];frame_rows=[];repeat_rows=[];plot_data=[];skipped=[]
    for roi in rois:
        selected=points[select_roi(points,roi)]
        settings=dict(cfg['plane'],normal_kind='vertical' if abs(roi['normal'][2])<.35 else 'horizontal')
        fit=fit_plane(selected,settings)
        if fit is None:
            skipped.append(dict(roi=roi['id'],points=len(selected),reason='insufficient points, insufficient 2-D support or unstable/refit orientation'))
            continue
        n,d,signed,mask=fit
        for label,res in [('all_roi',signed),('ransac_inliers',signed[mask])]:
            table.append(dict(roi=roi['id'],subset=label,**residual_metrics(res,cfg['plane']['threshold_m'])))
        plot_data.append((roi,selected,signed))
        # 按固定地图拟合面投影，避免每帧各自拟合掩盖重影
        visits=[];visit=[];previous=None
        for scan in scans:
            s=scan['points'][select_roi(scan['points'],roi)]
            if len(s)<cfg['consistency']['min_roi_points']:
                continue
            residual=s@n+d
            frame_rows.append(dict(roi=roi['id'],stamp=scan['stamp'],points=len(s),
                median_signed_m=float(np.median(residual)),rms_to_fixed_plane_m=float(np.sqrt(np.mean(residual**2))),
                p95_to_fixed_plane_m=float(np.percentile(abs(residual),95))))
            if previous is not None and scan['stamp']-previous>cfg['consistency']['min_visit_gap_s']:
                visits.append(visit);visit=[]
            visit.append((scan['stamp'],s));previous=scan['stamp']
        if visit:
            visits.append(visit)
        for j in range(1,len(visits)):
            a=np.concatenate([p for _,p in visits[j-1]])
            b=np.concatenate([p for _,p in visits[j]])
            # 双向近邻距离包含点密度/视角差，固定 LIO 坐标下不执行 ICP 后对齐
            nearest=np.r_[cKDTree(a).query(b,workers=1)[0],cKDTree(b).query(a,workers=1)[0]]
            basis=np.asarray(roi['basis'])
            qa=np.floor((a@basis.T)/.2).astype(np.int64)
            qb=np.floor((b@basis.T)/.2).astype(np.int64)
            common=set(map(tuple,qa))&set(map(tuple,qb))
            ac=a[np.array([tuple(x) in common for x in qa])]
            bc=b[np.array([tuple(x) in common for x in qb])]
            shared_nn=np.r_[cKDTree(ac).query(bc,workers=1)[0],cKDTree(bc).query(ac,workers=1)[0]] if len(common)>=10 else np.array([])
            plane_a=fit_plane(a,cfg['plane'])
            plane_b=fit_plane(b,cfg['plane'])
            angle=None
            if plane_a and plane_b:
                angle=float(np.degrees(np.arccos(np.clip(abs(plane_a[0]@plane_b[0]),0,1))))
            repeat_rows.append(dict(roi=roi['id'],visit_a=j-1,visit_b=j,
                start_a=visits[j-1][0][0],end_a=visits[j-1][-1][0],start_b=visits[j][0][0],end_b=visits[j][-1][0],
                points_a=len(a),points_b=len(b),median_plane_offset_change_m=float(abs(np.median(a@n+d)-np.median(b@n+d))),
                fitted_normal_angle_deg=angle,nn_mean_m=float(nearest.mean()),nn_p50_m=float(np.percentile(nearest,50)),
                nn_p95_m=float(np.percentile(nearest,95)),nn_max_m=float(nearest.max()),
                common_tangent_cells=len(common),common_points_a=len(ac),common_points_b=len(bc),
                shared_nn_mean_m=float(shared_nn.mean()) if len(shared_nn) else None,
                shared_nn_p50_m=float(np.percentile(shared_nn,50)) if len(shared_nn) else None,
                shared_nn_p95_m=float(np.percentile(shared_nn,95)) if len(shared_nn) else None,
                shared_nn_max_m=float(shared_nn.max()) if len(shared_nn) else None))
    write_csv(output/'plane_metrics.csv',table)
    write_csv(output/'plane_frame_metrics.csv',frame_rows)
    write_csv(output/'repeat_observation_metrics.csv',repeat_rows)
    summary=dict(source=source,map_points=len(points),paired_scans=len(scans),unmatched_scans=unmatched,
        plane_rois=len(plot_data),skipped_rois=skipped,roi_selection='deterministic orientation-constrained sequential RANSAC; most-supported 4 m tangent tile clipped by 10%-90% bounds; 0.30 m normal slab; near-start lower horizontal/above-origin vertical candidates plus all-route horizontal/vertical candidates; save all ROIs',
        plane_metrics=table,repeat_observations=repeat_rows,absolute_mapping_accuracy=None,
        note='Self-consistency; fitted inlier residual is threshold-conditioned. All-ROI residual and outlier fraction retained. No external map, semantics or surveyed planes.')
    if capture and (capture/'tf.csv').exists():
        summary['obstacle_structural_roi_consistency']=obstacle_roi_metrics(capture,rois,cfg,output)
    write_json(output/'geometry_metrics.json',summary)
    if plot_data:
        fig,axes=plt.subplots(len(plot_data),2,figsize=(10,3*len(plot_data)),squeeze=False)
        for ax,(roi,s,signed) in zip(axes,plot_data):
            uv=s@np.asarray(roi['basis']).T
            step=max(1,len(s)//20000)
            h=ax[0].scatter(uv[::step,0],uv[::step,1],c=signed[::step]*100,s=2,cmap='coolwarm',vmin=-20,vmax=20)
            ax[0].set_title(roi['id']+' tangent patch; residual cm');ax[0].set_aspect('equal');fig.colorbar(h,ax=ax[0])
            ax[1].hist(signed*100,bins=80);ax[1].set_xlabel('Signed plane residual (cm)');ax[1].set_ylabel('Points')
        fig.tight_layout();fig.savefig(output/'plane_residuals.png',dpi=140);plt.close(fig)
    if frame_rows:
        fig,ax=plt.subplots(figsize=(10,4))
        anchor=min(r['stamp'] for r in frame_rows)
        for roi in rois:
            g=[r for r in frame_rows if r['roi']==roi['id']]
            ax.plot([r['stamp']-anchor for r in g],[r['median_signed_m']*100 for r in g],'.',label=roi['id'])
        ax.set_xlabel('Scan time (s)');ax.set_ylabel('Median signed residual to fixed plane (cm)');ax.legend()
        fig.tight_layout();fig.savefig(output/'repeat_plane_offsets.png',dpi=160);plt.close(fig)
    return summary


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--config',type=Path,default=Path(__file__).with_name('quality.yaml'))
    p.add_argument('--capture',type=Path)
    p.add_argument('--map',type=Path)
    p.add_argument('--rois',type=Path)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    result=analyze(a.capture,a.output,yaml.safe_load(a.config.read_text()),a.map,a.rois)
    print(json.dumps({k:result[k] for k in ['map_points','paired_scans','plane_rois','unmatched_scans']},indent=2))
