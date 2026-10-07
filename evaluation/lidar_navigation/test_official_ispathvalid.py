#!/usr/bin/env python3
"""Check saved XY paths with the installed official IsPathValid service.

Circular footprint mode makes orientation irrelevant. This is not a polygon
footprint audit. The fixture freezes the raw uint8 costmaps before each query.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.srv import GetCostmap, IsPathValid
from std_srvs.srv import Trigger
import yaml

parser=argparse.ArgumentParser()
parser.add_argument('--reference',type=Path,required=True)
parser.add_argument('--fixture',type=Path,required=True)
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args();args.fixture=args.fixture.resolve();args.output=args.output.resolve();args.reference=args.reference.resolve();args.output.mkdir(parents=True,exist_ok=True)
root=args.output/'scenes';root.mkdir(exist_ok=True)
records=json.loads((args.reference/'paired_planning.json').read_text())
scenes=[]
for record in records:
    for name,layer in record['layers'].items():
        data=np.load(args.reference/layer['costmap']);grid=data['grid'];origin=data['origin'];resolution=float(data['resolution'])
        scene=f'{name}_{record["attempt"]:02d}';directory=root/scene;directory.mkdir(exist_ok=True)
        (directory/'metadata.txt').write_text(' '.join(map(str,[grid.shape[1],grid.shape[0],resolution,*origin,*record['start']])),encoding='utf-8')
        (directory/'grid.bin').write_bytes(grid.astype(np.uint8).tobytes())
        scenes.append((scene,record,layer,grid,origin,resolution))
(root/'active_scene.txt').write_text(scenes[0][0],encoding='utf-8')
params={'official_snapshot':{'planner_server':{'ros__parameters':{'use_sim_time':False,'bond_heartbeat_period':0.0,'planner_plugins':['GridBased'],'GridBased':{'plugin':'nav2_navfn_planner::NavfnPlanner','allow_unknown':False}}},'global_costmap':{'global_costmap':{'ros__parameters':{'use_sim_time':False,'global_frame':'odom','robot_base_frame':'nav_base','rolling_window':False,'width':12,'height':12,'resolution':.1,'robot_radius':.25,'footprint_padding':.01,'plugins':['frozen'],'frozen':{'plugin':'nav2_costmap_2d::InflationLayer','enabled':False},'track_unknown_space':True,'footprint_clearing_enabled':False,'update_frequency':5.0,'publish_frequency':0.0}}}}}
parameter_file=args.output/'fixture.yaml';parameter_file.write_text(yaml.safe_dump(params),encoding='utf-8')
log=(args.output/'fixture.log').open('w')
process=subprocess.Popen([str(args.fixture),str(root),'--ros-args','--params-file',str(parameter_file),'-r','__ns:=/official_snapshot'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
rclpy.init();node=rclpy.create_node('official_ispathvalid_audit')
loader=node.create_client(Trigger,'/official_snapshot/load_scene')
check=node.create_client(IsPathValid,'/official_snapshot/is_path_valid')
getmap=node.create_client(GetCostmap,'/official_snapshot/global_costmap/get_costmap')
results=[]

def finish(future):
    rclpy.spin_until_future_complete(node,future,timeout_sec=10)
    if not future.done():raise RuntimeError('Official service timed out')
    return future.result()

def query(points):
    request=IsPathValid.Request();request.path.header.frame_id='odom';request.path.header.stamp=node.get_clock().now().to_msg()
    for x,y in points:
        pose=PoseStamped();pose.header=request.path.header;pose.pose.position.x=float(x);pose.pose.position.y=float(y);pose.pose.orientation.w=1.0;request.path.poses.append(pose)
    began=time.perf_counter();response=finish(check.call_async(request))
    return {'is_valid':bool(response.is_valid),'invalid_pose_indices':list(getattr(response,'invalid_pose_indices',[])),'latency_ms':(time.perf_counter()-began)*1000,'poses':len(points)}

def verify_grid(expected,origin,resolution):
    actual=finish(getmap.call_async(GetCostmap.Request())).map
    metadata=actual.metadata
    grid=np.asarray(actual.data,dtype=np.uint8).reshape(metadata.size_y,metadata.size_x)
    if not np.array_equal(grid,expected) or not np.allclose([metadata.origin.position.x,metadata.origin.position.y],origin,atol=1e-9,rtol=0) or abs(metadata.resolution-resolution)>1e-9:
        raise RuntimeError('Official internal grid differs from the raw reference snapshot')

try:
    for client in [loader,check,getmap]:
        if not client.wait_for_service(timeout_sec=25):raise RuntimeError(f'Missing official service: {client.srv_name}')
    for scene,record,layer,grid,origin,resolution in scenes:
        (root/'active_scene.txt').write_text(scene,encoding='utf-8')
        loaded=finish(loader.call_async(Trigger.Request()))
        if not loaded.success:raise RuntimeError(loaded.message)
        verify_grid(grid,origin,resolution)
        row={'scene':scene,'attempt':record['attempt'],'layer':scene.split('_')[0],'original_error_code':layer.get('error_code'),'original_status':layer.get('status'),'start':record['start'],'goal':record['goal'],'grid_sha256':hashlib.sha256(grid.astype(np.uint8).tobytes()).hexdigest(),'raw_grid_verified_before':True,'path':query(layer.get('path',[])),'probes':{}}
        if layer.get('path') and not row['path']['is_valid']:
            row['rejected_single_pose_queries']=[]
            for index,point in enumerate(layer['path']):
                response=query([point])
                if not response['is_valid']:
                    cell=np.floor((np.asarray(point)-origin)/resolution).astype(int)
                    inside=0<=cell[0]<grid.shape[1] and 0<=cell[1]<grid.shape[0]
                    # Indices come from independent one-pose official calls,
                    # not the full-path response's invalid_pose_indices.
                    row['rejected_single_pose_queries'].append({'index':index,'xy':point,'snapshot_cost':int(grid[cell[1],cell[0]]) if inside else None,**response})
        for value,label in [(0,'free'),(253,'inscribed'),(254,'lethal'),(255,'unknown')]:
            locations=np.argwhere(grid==value)
            if len(locations):
                y,x=locations[len(locations)//2];point=origin+(np.array([x,y])+.5)*resolution
                row['probes'][label]={'source_cost':value,'xy':point.tolist(),**query([point])}
        row['probes']['empty']=query([])
        row['probes']['outside']=query([origin-np.array([1,1])])
        verify_grid(grid,origin,resolution);row['raw_grid_verified_after']=True
        results.append(row);(args.output/'official_ispathvalid.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
        print(scene,'original',row['original_error_code'],'official',row['path']['is_valid'],'indices',row['path']['invalid_pose_indices'],flush=True)
finally:
    node.destroy_node();rclpy.shutdown()
    if process.poll() is None:
        os.killpg(process.pid,signal.SIGINT)
        try:process.wait(timeout=10)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
    log.close()
print('Official checks completed',len(results),flush=True)
