#!/usr/bin/env python3
"""Synthetic empty-map lifecycle regression, separate from real-bag quality."""
import argparse
import json
import os
import signal
import subprocess
import time
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=False)
env = os.environ.copy()
env.update(ROS_DOMAIN_ID='219', ROS_LOG_DIR=str(args.output/'ros_logs'), ROS_HOME=str(args.output/'ros_home'))
root = Path('/home/h/lidar_ws')
results = []
for label, transitions, sig in [('active_sigint', ['configure','activate'], signal.SIGINT),
                               ('active_sigterm', ['configure','activate'], signal.SIGTERM),
                               ('finalized_sigint', ['configure','activate','deactivate','cleanup','shutdown'], signal.SIGINT)]:
    with (args.output/(label+'.log')).open('w') as log:
        tf = subprocess.Popen(['ros2','run','tf2_ros','static_transform_publisher','--frame-id','odom','--child-frame-id','nav_base'],
                              env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        node = subprocess.Popen([str(root/'install/lib/lidar_nav2_bringup/local_costmap_server'), '--ros-args',
            '-r','__node:=local_costmap','-r','__ns:=/local_costmap','--params-file',str(root/'src/lidar_nav2_bringup/config/local_costmap_fast.yaml'),
            '-p','use_sim_time:=false'], env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        codes = []
        try:
            time.sleep(2)
            for transition in transitions:
                r = subprocess.run(['ros2','lifecycle','set','--no-daemon','/local_costmap/local_costmap',transition], env=env,
                                   stdout=log,stderr=subprocess.STDOUT,timeout=15)
                codes.append(r.returncode)
            time.sleep(.5)
            os.killpg(node.pid,sig)
            exit_code = node.wait(timeout=15)
            results.append(dict(case=label, transitions=codes, exit_code=exit_code))
            print(label,exit_code,flush=True)
        finally:
            if node.poll() is None:os.killpg(node.pid,signal.SIGKILL);node.wait()
            os.killpg(tf.pid,signal.SIGINT);tf.wait(timeout=10)
(args.output/'results.json').write_text(json.dumps(dict(mode='synthetic_empty_map_lifecycle',cases=results),indent=2))
if any(r['exit_code'] != 0 or any(r['transitions']) for r in results):raise SystemExit(1)
