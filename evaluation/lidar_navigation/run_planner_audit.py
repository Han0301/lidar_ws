"""Wait for the bag clock, then test the planner over the same outdoor segment."""
import rclpy,time,subprocess,os,signal,sys
from nav_msgs.msg import Odometry
from pathlib import Path
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True)
rclpy.init();n=rclpy.create_node('planner_audit_gate');stamps=[]
n.create_subscription(Odometry,'/nav/odom',lambda m:stamps.append(m.header.stamp.sec+m.header.stamp.nanosec*1e-9),10)
start=time.monotonic()
while len(stamps)<3:
 rclpy.spin_once(n,timeout_sec=.1)
 if time.monotonic()-start>40:raise RuntimeError('No navigation poses')
anchor=stamps[0]
log=(out/'planner.log').open('w');p=subprocess.Popen(['ros2','launch','lidar_nav2_bringup','bag_planner.launch.py'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
try:
 while stamps[-1]-anchor<135:
  rclpy.spin_once(n,timeout_sec=.1)
  if time.monotonic()-start>210:raise RuntimeError('Bag clock stopped before planning segment')
 with (out/'test.log').open('w') as f:
  result=subprocess.run([sys.executable,str(Path(__file__).with_name('test_bag_planner.py')),'--output',str(out),'--attempts','12'],stdout=f,stderr=subprocess.STDOUT,timeout=90)
 print('planner_test_exit',result.returncode,flush=True)
finally:
 os.killpg(p.pid,signal.SIGINT)
 try:p.wait(timeout=12)
 except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
 log.close();n.destroy_node()
 if rclpy.ok():rclpy.shutdown()

sys.exit(result.returncode)
