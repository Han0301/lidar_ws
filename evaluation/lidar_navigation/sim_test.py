import rclpy,time,json,csv,math,sys
from pathlib import Path
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry,OccupancyGrid
from nav2_simple_commander.robot_navigator import BasicNavigator,TaskResult
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
import numpy as np
from scipy.ndimage import distance_transform_edt
rclpy.init();n=BasicNavigator(node_name='navigation_validation');n.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
out=Path(sys.argv[1]) if len(sys.argv)>1 else Path(__file__).resolve().parent / 'results/simulation';out.mkdir(parents=True,exist_ok=True)
trajectory=[];maps=[]
n.create_subscription(Odometry,'/odom',lambda m:trajectory.append([m.header.stamp.sec+m.header.stamp.nanosec*1e-9,m.pose.pose.position.x,m.pose.pose.position.y,m.twist.twist.linear.x,m.twist.twist.angular.z]),10)
n.create_subscription(OccupancyGrid,'/global_costmap/costmap',lambda m:maps.append(m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
for _ in range(15):rclpy.spin_once(n,timeout_sec=.1)
pose=PoseStamped();pose.header.frame_id='map';pose.header.stamp=n.get_clock().now().to_msg();pose.pose.position.x=2.;pose.pose.position.y=5.;pose.pose.orientation.w=1.
n.setInitialPose(pose)
n.waitUntilNav2Active(localizer='map_server')
goal=PoseStamped();goal.header.frame_id='map';goal.header.stamp=n.get_clock().now().to_msg();goal.pose.position.x=9.;goal.pose.position.y=5.;goal.pose.orientation.w=1.
start=time.monotonic();n.goToPose(goal)
while not n.isTaskComplete():
 if time.monotonic()-start>150:n.cancelTask();break
 time.sleep(.05)
result=n.getResult()
for _ in range(10):rclpy.spin_once(n,timeout_sec=.1)
with (out/'trajectory.csv').open('w') as f:w=csv.writer(f);w.writerow(['stamp','x_relative','y_relative','vx','wz']);w.writerows(trajectory)
# Loopback odometry is relative to initial pose, not map coordinates.
a=np.array(trajectory);xy=a[:,1:3]+[2.,5.];length=np.linalg.norm(np.diff(xy,axis=0),axis=1).sum() if len(a)>1 else 0.
summary=dict(result=str(result),elapsed_s=time.monotonic()-start,poses=len(a),path_length_m=float(length),final_goal_error_m=float(np.linalg.norm(xy[-1]-[9,5])) if len(a) else None,assumed_robot_radius_m=.25,model='Nav2 loopback ideal kinematics; no physics or hardware validation')
m=np.full((200,240),False);m[:5,:]=True;m[-5:,:]=True;m[:,:5]=True;m[:,-5:]=True;m[65:135,105:120]=True;edt=distance_transform_edt(~m)*.05
ij=np.floor(xy/.05).astype(int);valid=(ij[:,0]>=0)&(ij[:,0]<240)&(ij[:,1]>=0)&(ij[:,1]<200);clearance=edt[ij[valid,1],ij[valid,0]]-.25
summary['min_footprint_clearance_m']=float(clearance.min()) if len(clearance) else None
(out/'summary.json').write_text(json.dumps(summary,indent=2));np.save(out/'map_xy.npy',xy);print(summary);n.destroy_node();rclpy.shutdown()
