"""Compute paths on observed outdoor data; no commands are sent to a robot."""
import rclpy,time,json,argparse,math
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import numpy as np
from pathlib import Path
parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);parser.add_argument('--attempts',type=int,default=10);args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
rclpy.init();n=rclpy.create_node('recorded_path_validation');n.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)]);poses=[]; latest_map=None
n.create_subscription(Odometry,'/nav/odom',lambda m:poses.append(m),20)
def save_map(message):
 global latest_map
 latest_map=message
n.create_subscription(OccupancyGrid,'/global_costmap/costmap',save_map,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
client=ActionClient(n,ComputePathToPose,'/compute_path_to_pose')
if not client.wait_for_server(timeout_sec=30):raise RuntimeError('Planner action unavailable')
results=[]
for attempt in range(args.attempts):
 start=time.monotonic()
 while time.monotonic()-start<4:rclpy.spin_once(n,timeout_sec=.1)
 if len(poses)<5:continue
 current=poses[-1];selected=None
 for old in reversed(poses[:-1]):
  a=current.pose.pose.position;b=old.pose.pose.position
  if math.hypot(a.x-b.x,a.y-b.y)>1.5:selected=old;break
 if selected is None:continue
 evidence={'attempt':attempt,'stamp':n.get_clock().now().nanoseconds*1e-9,'start':[current.pose.pose.position.x,current.pose.pose.position.y],'goal':[selected.pose.pose.position.x,selected.pose.pose.position.y]}
 if latest_map is not None:
  m=latest_map
  filename=f'costmap_attempt_{attempt:02d}.npz'
  np.savez_compressed(args.output/filename,grid=np.array(m.data,dtype=np.int16).reshape(m.info.height,m.info.width),resolution=m.info.resolution,origin=[m.info.origin.position.x,m.info.origin.position.y],stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9)
  evidence['costmap']=filename
 goal=ComputePathToPose.Goal();goal.planner_id='GridBased';goal.use_start=False;goal.goal.header.frame_id='odom';goal.goal.header.stamp=n.get_clock().now().to_msg();goal.goal.pose=selected.pose.pose
 sent=client.send_goal_async(goal);rclpy.spin_until_future_complete(n,sent,timeout_sec=8)
 if not sent.done():results.append(dict(evidence,state='send_timeout'));continue
 handle=sent.result()
 if not handle.accepted:results.append(dict(evidence,state='rejected'));continue
 future=handle.get_result_async();rclpy.spin_until_future_complete(n,future,timeout_sec=8)
 if not future.done():results.append(dict(evidence,state='result_timeout'));continue
 result=future.result();path=result.result.path
 points=[[p.pose.position.x,p.pose.position.y] for p in path.poses]
 results.append(dict(evidence,action_status=result.status,error_code=int(result.result.error_code),poses=len(points),path=points))
(args.output/'planning.json').write_text(json.dumps(results,indent=2));print('attempts',len(results),'succeeded',sum(r.get('action_status')==4 for r in results),flush=True);n.destroy_node();rclpy.shutdown()
