#!/usr/bin/env python3
"""Real-bag short integration test of public launch, saved map and official Nav2 API."""
import argparse
import csv
import importlib.util
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid,Odometry
from nav2_msgs.msg import Costmap
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import IsPathValid, ManageLifecycleNodes
from geometry_msgs.msg import PoseStamped
from common import sha256,write_json


class Probe(Node):
    def __init__(self, output):
        self.output=output
        self.plan_index=0
        super().__init__('system_interface_probe')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
        self.map=None;self.global_map=None;self.local_map=None;self.odom=None
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid,'/map',lambda m:setattr(self,'map',m),qos)
        self.create_subscription(Costmap,'/global_costmap/costmap_raw',lambda m:setattr(self,'global_map',m),qos)
        self.create_subscription(Costmap,'/local_costmap/costmap_raw',lambda m:setattr(self,'local_map',m),qos)
        self.create_subscription(Odometry,'/nav/odom',lambda m:setattr(self,'odom',m),10)
        self.planner=ActionClient(self,ComputePathToPose,'/compute_path_to_pose')
        self.valid=self.create_client(IsPathValid,'/is_path_valid')

    def wait(self, condition, timeout=15):
        deadline=time.monotonic()+timeout
        while not condition():
            if time.monotonic()>deadline:raise RuntimeError('Integration wait timed out')
            rclpy.spin_once(self,timeout_sec=.1)

    def future(self, future):
        self.wait(future.done)
        return future.result()

    def pose(self, xy):
        pose=PoseStamped();pose.header.frame_id='odom';pose.header.stamp=self.get_clock().now().to_msg()
        pose.pose.position.x=float(xy[0]);pose.pose.position.y=float(xy[1]);pose.pose.orientation.w=1.
        return pose

    def plan(self,start,goal):
        request=ComputePathToPose.Goal();request.start=self.pose(start);request.goal=self.pose(goal)
        request.use_start=True;request.planner_id='GridBased'
        self.plan_index+=1
        began=time.monotonic();handle=self.future(self.planner.send_goal_async(request))
        if not handle.accepted:return dict(accepted=False)
        reply=self.future(handle.get_result_async());result=reply.result
        record=dict(start=list(map(float,start)),goal=list(map(float,goal)),status=int(reply.status),
                    error_code=int(result.error_code),error_msg=result.error_msg,
                    elapsed_wall_ms=(time.monotonic()-began)*1000,path_points=len(result.path.poses),official_valid=None)
        if result.path.poses:
            xy=np.array([[p.pose.position.x,p.pose.position.y] for p in result.path.poses])
            np.savetxt(self.output/('path_%02d.csv'%self.plan_index),xy,delimiter=',',header='x,y',comments='')
            record['path_length_m']=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum())
            self.wait(self.valid.service_is_ready)
            v=IsPathValid.Request();v.path=result.path
            record['official_valid']=bool(self.future(self.valid.call_async(v)).is_valid)
        return record


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--map-run',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--requests-from',type=Path)
    args=parser.parse_args()
    ws=Path('/home/h/lidar_ws');out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    spec=importlib.util.spec_from_file_location('prior',ws/'evaluation/run_playback.py');prior=importlib.util.module_from_spec(spec);spec.loader.exec_module(prior)
    if subprocess.run(['pgrep','-x','fastlio_mapping'],capture_output=True).returncode==0:raise RuntimeError('FAST-LIO already running')
    env=os.environ.copy();env.update(ROS_DOMAIN_ID='218',ROS_LOG_DIR=str(out/'ros_logs'),ROS_HOME=str(out/'ros_home'))
    os.environ.update(ROS_DOMAIN_ID='218',ROS_LOG_DIR=str(out/'ros_logs'),ROS_HOME=str(out/'ros_home'))
    original=ws/'src/FAST_LIO/Log';shutil.copytree(original,out/'log_before')
    lio=yaml.safe_load((ws/'src/FAST_LIO/config/mid360.yaml').read_text());lio['/**']['ros__parameters']['pcd_save']['pcd_save_en']=False
    (out/'mid360.yaml').write_text(yaml.safe_dump(lio))
    jobs={};handles=[];status={}
    def start(name,cmd):
        p,h=prior.start(cmd,name,out,env);jobs[name]=p;handles.append(h)
    probe=None
    try:
        start('lio',['ros2','launch','fast_lio','mapping.launch.py',f'config_path:={out}','config_file:=mid360.yaml','use_sim_time:=true','rviz:=false'])
        start('system',['ros2','launch','lidar_nav2_bringup','lidar.launch.py','mode:=planning','frontend:=false','rviz:=false',f'map:={args.map_run.resolve()}/global_map.yaml'])
        rclpy.init();probe=Probe(out);time.sleep(3)
        start('player',['ros2','bag','play',str(ws/'bags/室外闭环'),'--clock','100'])
        probe.wait(lambda:all(x is not None for x in [probe.map,probe.global_map,probe.local_map,probe.odom]),timeout=35)
        probe.wait(probe.planner.server_is_ready)
        m=probe.map;c=probe.global_map.metadata
        status['map']=dict(frame=m.header.frame_id,width=m.info.width,height=m.info.height,resolution=m.info.resolution,
            known_cells=sum(x>=0 for x in m.data),occupied_cells=sum(x==100 for x in m.data))
        np.savez_compressed(out/'loaded_map.npz',grid=np.asarray(m.data,dtype=np.int8).reshape(m.info.height,m.info.width),
                            origin=[m.info.origin.position.x,m.info.origin.position.y],resolution=m.info.resolution)
        status['global_costmap']=dict(width=c.size_x,height=c.size_y,resolution=c.resolution,frame=probe.global_map.header.frame_id)
        if (c.size_x,c.size_y)!=(m.info.width,m.info.height):raise RuntimeError('Global costmap did not use full saved-map extent')
        od=list(csv.DictReader((args.map_run/'capture/odometry.csv').open()))
        tf=next(r for r in csv.DictReader((args.map_run/'capture/tf.csv').open()) if r['parent']=='odom' and r['child']=='camera_init')
        R=Rotation.from_quat([float(tf[k]) for k in ['qx','qy','qz','qw']]).as_matrix()
        xyz=np.array([[float(r[k]) for k in ['x','y','z']] for r in od])@R.T
        xyz+=np.array([float(tf[k]) for k in ['x','y','z']])
        # 起终点来自真实估计轨迹，不用虚构地面真值；测试清晰记录未知区导致的拒绝
        status['planning']=[dict(source='recorded_estimated_trajectory', **probe.plan(xyz[i,:2],xyz[j,:2]))
                            for i,j in [(50,70),(50,300),(500,900),(1200,1500)]]
        from scipy.ndimage import label
        grid=np.asarray(probe.global_map.data,dtype=np.uint8).reshape(c.size_y,c.size_x)
        labels,count=label(grid<253)
        sizes=np.bincount(labels.ravel());sizes[0]=0
        if count and sizes.max()>1:
            cells=np.argwhere(labels==sizes.argmax())
            for axis in [0,1]:
                a=cells[np.argmin(cells[:,axis])];b=cells[np.argmax(cells[:,axis])]
                def world(cell):
                    return [c.origin.position.x+(float(cell[1])+.5)*c.resolution,
                            c.origin.position.y+(float(cell[0])+.5)*c.resolution]
                status['planning'].append(dict(source='observed_costmap_component_extrema',
                                               **probe.plan(world(a),world(b))))
        if args.requests_from:
            requests=json.loads(args.requests_from.read_text())['planning']
            status['fixed_baseline_planning']=[dict(source='fixed_baseline_request',**probe.plan(r['start'],r['goal'])) for r in requests]
        status['measurement_complete']=True
    except Exception as error:
        status['error']=str(error);status['measurement_complete']=False
    finally:
        if 'system' in jobs:
            for service in ['/lifecycle_manager_navigation/manage_nodes','/lifecycle_manager_map/manage_nodes']:
                with (out/(service.split('/')[1]+'_shutdown.log')).open('w') as log:
                    if probe is not None:
                        try:
                            client=probe.create_client(ManageLifecycleNodes,service)
                            if not client.wait_for_service(timeout_sec=5):raise RuntimeError('Lifecycle service unavailable')
                            request=ManageLifecycleNodes.Request();request.command=4
                            reply=probe.future(client.call_async(request))
                            status[service]=0 if reply.success else 1
                            log.write('Existing probe participant shutdown success: '+str(reply.success))
                        except Exception as error:
                            status[service]=1;log.write(str(error))
                    else:
                        status[service]=subprocess.run([sys.executable,str(ws/'evaluation/map_quality/lifecycle_command.py'),service,'4'],env=env,stdout=log,stderr=subprocess.STDOUT,timeout=20).returncode
        for name in ['player','system','lio']:
            if name in jobs:status[name+'_returncode']=prior.stop(jobs[name])
        if probe is not None:probe.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for h in handles:h.close()
        shutil.copytree(original,out/'log_generated');shutil.copytree(out/'log_before',original,dirs_exist_ok=True)
        status['shutdown_service_failures']={k:v for k,v in status.items() if k.startswith('/lifecycle_') and v!=0}
        write_json(out/'results.json',dict(mode='offline_real_bag_short_interface_test',**status,
            map_yaml_sha256=sha256(args.map_run/'global_map.yaml'),no_hardware_navigation=True))
    print(json.dumps(status,indent=2))
    return int('error' in status or status.get('system_returncode')!=0 or bool(status.get('shutdown_service_failures')))


if __name__=='__main__':sys.exit(main())
