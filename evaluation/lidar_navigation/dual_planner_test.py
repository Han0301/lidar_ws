"""Paired planner requests against one paused sensor stream; no robot commands."""
import argparse,copy,json,math,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import GetCostmap
from rosbag2_interfaces.srv import Pause,Resume
from lifecycle_msgs.srv import GetState

parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);parser.add_argument('--start-after',type=float,default=135);parser.add_argument('--attempts',type=int,default=12);args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
rclpy.init();node=rclpy.create_node('paired_planner_test');node.set_parameters([Parameter('use_sim_time',value=True)]);poses=[]
node.create_subscription(Odometry,'/nav/odom',poses.append,50)
pause=node.create_client(Pause,'/rosbag2_player/pause');resume=node.create_client(Resume,'/rosbag2_player/resume')
actions={name:ActionClient(node,ComputePathToPose,f'/{name}/compute_path_to_pose') for name in ['voxel','ground']}
services={name:node.create_client(GetCostmap,f'/{name}/global_costmap/get_costmap') for name in actions}
def finish(future,seconds=15):
 rclpy.spin_until_future_complete(node,future,timeout_sec=seconds)
 if not future.done():raise RuntimeError('Timed out waiting for ROS service/action')
 return future.result()
def spin_for(seconds):
 deadline=time.monotonic()+seconds
 while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
for client in [pause,resume,*services.values()]:
 if not client.wait_for_service(timeout_sec=40):raise RuntimeError(f'Missing service {client.srv_name}')
for client in actions.values():
 if not client.wait_for_server(timeout_sec=40):raise RuntimeError('Missing planner action')
# An action server can exist even when its costmap failed activation.
for name in actions:
 client=node.create_client(GetState,f'/{name}/global_costmap/global_costmap/get_state')
 if not client.wait_for_service(timeout_sec=20):raise RuntimeError(f'Missing {name} costmap lifecycle service')
 deadline=time.monotonic()+30
 while finish(client.call_async(GetState.Request())).current_state.id!=3:
  spin_for(.2)
  if time.monotonic()>deadline:raise RuntimeError(f'{name} costmap is not active; comparison is invalid')
while not poses:rclpy.spin_once(node,timeout_sec=.1)
anchor=stamp(poses[0]);results=[]
try:
 for attempt in range(args.attempts):
  target=args.start_after+4*(attempt+1);deadline=time.monotonic()+210
  while stamp(poses[-1])-anchor<target:
   rclpy.spin_once(node,timeout_sec=.1)
   if time.monotonic()>deadline:raise RuntimeError('Bag stopped before comparison point')
  finish(pause.call_async(Pause.Request()));spin_for(.8)
  current=copy.deepcopy(poses[-1]);old=None
  for previous in reversed(poses[:-1]):
   a=current.pose.pose.position;b=previous.pose.pose.position
   if math.hypot(a.x-b.x,a.y-b.y)>1.5:old=copy.deepcopy(previous);break
  if old is None:raise RuntimeError('No suitable past pose')
  record={'attempt':attempt,'source_stamp':stamp(current),'clock_stamp':node.get_clock().now().nanoseconds*1e-9,'start':[current.pose.pose.position.x,current.pose.pose.position.y],'goal':[old.pose.pose.position.x,old.pose.pose.position.y],'paused':True,'layers':{}}
  goals={};sent_at={};completed_at={}
  for name,client in actions.items():
   costmap=finish(services[name].call_async(GetCostmap.Request())).map;meta=costmap.metadata
   if meta.size_x<2 or meta.size_y<2:raise RuntimeError(f'{name} costmap has no usable grid')
   grid=np.asarray(costmap.data,dtype=np.uint8).reshape(meta.size_y,meta.size_x);filename=f'{name}_{attempt:02d}.npz'
   start_x=int(math.floor((record['start'][0]-meta.origin.position.x)/meta.resolution))
   start_y=int(math.floor((record['start'][1]-meta.origin.position.y)/meta.resolution))
   if not (0<=start_x<meta.size_x and 0<=start_y<meta.size_y):raise RuntimeError(f'{name} rolling map does not contain the current pose')
   np.savez_compressed(args.output/filename,grid=grid,resolution=meta.resolution,origin=[meta.origin.position.x,meta.origin.position.y],source_stamp=stamp(current),clock_stamp=record['clock_stamp'])
   record['layers'][name]={'costmap':filename,'lethal_cells':int((grid==254).sum()),'unknown_cells':int((grid==255).sum()),'start_cost':int(grid[start_y,start_x])}
   goal=ComputePathToPose.Goal();goal.planner_id='GridBased';goal.use_start=True;goal.start.header.frame_id='odom';goal.start.header.stamp=node.get_clock().now().to_msg();goal.start.pose=current.pose.pose;goal.goal.header=copy.deepcopy(goal.start.header);goal.goal.pose=old.pose.pose
   sent_at[name]=time.perf_counter();goals[name]=client.send_goal_async(goal)
  handles={name:finish(f) for name,f in goals.items()}
  futures={name:handle.get_result_async() for name,handle in handles.items() if handle.accepted}
  for name,future in futures.items():
   future.add_done_callback(lambda reply,name=name:completed_at.setdefault(name,time.perf_counter()))
  for name in actions:
   if name not in futures:record['layers'][name]['state']='rejected';continue
   reply=finish(futures[name])
   # Done callbacks may be queued on the executor after the future becomes done.
   deadline=time.monotonic()+2
   while name not in completed_at:
    rclpy.spin_once(node,timeout_sec=.01)
    if time.monotonic()>deadline:raise RuntimeError('Result timestamp callback did not run')
   record['layers'][name].update(status=reply.status,error_code=int(reply.result.error_code),planning_time_s=reply.result.planning_time.sec+reply.result.planning_time.nanosec*1e-9,request_latency_ms=(completed_at[name]-sent_at[name])*1000,path=[[p.pose.position.x,p.pose.position.y] for p in reply.result.path.poses])
  results.append(record);(args.output/'paired_planning.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
  print('pair',attempt,{n:r.get('error_code') for n,r in record['layers'].items()},flush=True)
  finish(resume.call_async(Resume.Request()))
finally:
 if resume.service_is_ready():finish(resume.call_async(Resume.Request()),seconds=3)
 node.destroy_node();rclpy.shutdown()
print('paired requests complete',len(results),flush=True)
