"""Launch two planners consuming one identical outdoor perception stream."""
import sys,subprocess,signal,os,time
import rclpy
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True)
share=Path(get_package_share_directory('lidar_nav2_bringup'));processes=[];files=[]
rclpy.init()
gate=rclpy.create_node('paired_planner_start_gate')
gate.set_parameters([Parameter('use_sim_time',value=True)])
poses=[]
gate.create_subscription(Odometry,'/nav/odom',poses.append,10)
buffer=Buffer();listener=TransformListener(buffer,gate)
deadline=time.monotonic()+60
try:
    while len(poses)<3 or gate.get_clock().now().nanoseconds==0 or not buffer.can_transform('odom','nav_base',rclpy.time.Time()):
        rclpy.spin_once(gate,timeout_sec=.1)
        if time.monotonic()>deadline:
            raise RuntimeError('Clock, navigation poses or odom/nav_base transform not ready')
    print('paired start gate ready',gate.get_clock().now().nanoseconds*1e-9,flush=True)
finally:
    gate.destroy_node();rclpy.shutdown()
try:
    for namespace,config in [('voxel','bag_planner.yaml'),('ground','bag_planner_ground.yaml')]:
        file=(out/f'{namespace}.log').open('w');files.append(file)
        process=subprocess.Popen(['ros2','launch','lidar_nav2_bringup','bag_planner.launch.py',f'namespace:={namespace}',f'params:={share}/config/{config}'],stdout=file,stderr=subprocess.STDOUT,start_new_session=True);processes.append(process)
    with (out/'paired_test.log').open('w') as file:
        result=subprocess.run([sys.executable,str(Path(__file__).with_name('dual_planner_test.py')),'--output',str(out),'--attempts','12'],stdout=file,stderr=subprocess.STDOUT,timeout=600)
    print('paired_test_exit',result.returncode,flush=True)
finally:
    for process in reversed(processes):
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=12)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
    for file in files:file.close()
sys.exit(result.returncode)
