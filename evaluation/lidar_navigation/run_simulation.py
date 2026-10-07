"""Navigation evaluation; owns and cleans up its isolated processes."""
import subprocess,os,signal,time,argparse
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
here=Path(__file__).resolve().parent
parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,default=here/'results/simulation_cloud');args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
nav=Path(get_package_share_directory('lidar_nav2_bringup'));perception=Path(get_package_share_directory('lidar_perception'))
processes=[];handles=[]
def start(command,name):
 f=(args.output/(name+'.log')).open('w');handles.append(f);p=subprocess.Popen(command,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);processes.append(p);return p
try:
 start(['ros2','launch','lidar_nav2_bringup','simulation.launch.py',f'params:={nav}/config/nav2_sim_cloud.yaml'],'navigation')
 start(['ros2','run','lidar_perception','perception_node','--ros-args','--params-file',str(perception/'config/perception.yaml'),'-p','input_topic:=/simulation/cloud_body','-p','output_frame:=nav_sensor'],'perception')
 start(['python3',str(here/'scan_to_cloud.py')],'ideal_cloud')
 time.sleep(5)
 client=start(['python3',str(here/'sim_test.py'),str(args.output)],'test')
 result=client.wait(timeout=200)
 print('navigation_test_exit',result,flush=True)
finally:
 for p in reversed(processes):
  if p.poll() is None:
   os.killpg(p.pid,signal.SIGINT)
   try:p.wait(timeout=12)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
 for f in handles:f.close()
