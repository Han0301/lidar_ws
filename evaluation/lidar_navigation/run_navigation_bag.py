#!/usr/bin/env python3
"""Run one recorded bag through FAST-LIO and the MID-360 monitor."""

import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
WORKSPACE = Path(os.environ.get("LIDAR_WS", SCRIPT_DIR.parents[1])).resolve()
RESULTS = Path(os.environ.get("LIDAR_EVAL_RESULTS", SCRIPT_DIR / "results")).resolve()
CONFIG = WORKSPACE / "src/FAST_LIO/config/mid360.yaml"


def start(command, label, directory, env):
    output = open(directory / f"{label}.log", "w", encoding="utf-8")
    process = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT,
                               env=env, start_new_session=True, text=True)
    return process, output


def stop(process, timeout=12):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
    return process.returncode


def run(bag, case, domain):
    directory = RESULTS / case
    directory.mkdir(parents=True, exist_ok=True)
    source_log = WORKSPACE / "src/FAST_LIO/Log"
    saved_log = directory / "fast_lio_log_before"
    if subprocess.run(["pgrep", "-x", "fastlio_mapping"], capture_output=True).returncode == 0:
        raise RuntimeError("A FAST-LIO process is already running; stop it before the isolated test")
    if source_log.is_dir():
        if saved_log.exists():
            shutil.rmtree(saved_log)
        shutil.copytree(source_log, saved_log)
    config = CONFIG.read_text(encoding="utf-8")
    config = re.sub(r'(?m)^(\s*map_file_path:\s*).+$',
                    lambda match: f'{match.group(1)}"{directory / "map.pcd"}"', config)
    config = re.sub(r'(?m)^(\s*pcd_save_en:\s*)true\b', r'\1false', config)
    (directory / "mid360.yaml").write_text(config, encoding="utf-8")
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(domain)
    env["ROS_HOME"] = str(directory / "ros_home")
    env["ROS_LOG_DIR"] = str(directory / "ros_logs")
    env["LD_LIBRARY_PATH"] = f"{WORKSPACE / 'sdk_install/lib'}:{env.get('LD_LIBRARY_PATH', '')}"
    env["RCUTILS_COLORIZED_OUTPUT"] = "0"
    processes = {}
    handles = []
    began = time.monotonic()
    status = {}
    try:
        launches = {
            "fast_lio": ["ros2", "launch", "fast_lio", "mapping.launch.py",
                         f"config_path:={directory}", "config_file:=mid360.yaml",
                         "use_sim_time:=true", "rviz:=false"],
            "monitor": ["ros2", "launch", "mid360_monitor", "mid360_monitor.launch.py"],
            "planner_audit": [sys.executable, str(SCRIPT_DIR / "run_planner_audit.py"), str(directory / "planning")],
            "perception": ["ros2", "launch", "lidar_nav2_bringup", "perception.launch.py"],
            "costmap": ["ros2", "launch", "lidar_nav2_bringup", "bag_costmap.launch.py"],
            "perception_recorder": [sys.executable, str(SCRIPT_DIR / "record_perception.py"), str(directory / "perception")],
            "odometry": [sys.executable, str(SCRIPT_DIR.parent / "record_odometry.py"), str(directory / "odometry.csv")],
        }
        for label, command in launches.items():
            process, handle = start(command, label, directory, env)
            processes[label] = process
            handles.append(handle)
        time.sleep(5)
        for label, process in processes.items():
            if process.poll() is not None:
                raise RuntimeError(f"{label} exited before playback: {process.returncode}")
        player, handle = start(["ros2", "bag", "play", str(bag), "--clock", "100", "-r", "1.0"],
                               "player", directory, env)
        gate, gate_handle = start([sys.executable, str(SCRIPT_DIR / "activate_costmap.py")], "activation", directory, env)
        processes["activation"] = gate
        handles.append(gate_handle)
        processes["player"] = player
        handles.append(handle)
        print(f"START {case} {bag}", flush=True)
        try:
            status["player_returncode"] = player.wait(timeout=720)
        except subprocess.TimeoutExpired:
            status["player_timeout"] = True
            status["player_returncode"] = stop(player)
        print(f"PLAYBACK_DONE {case} code={status['player_returncode']} elapsed_s={time.monotonic()-began:.1f}", flush=True)
        for label in ("fast_lio", "monitor", "odometry", "perception", "costmap", "perception_recorder"):
            if processes[label].poll() is not None:
                raise RuntimeError(f"{label} exited during playback: {processes[label].returncode}")
        time.sleep(4)
    except KeyboardInterrupt:
        status["error"] = "Interrupted; this run is incomplete"
    except Exception as error:
        status["error"] = str(error)
        print(f"ERROR {case} {error}", flush=True)
    finally:
        for label in ("player", "planner_audit", "activation", "perception_recorder", "costmap", "perception", "odometry", "monitor", "fast_lio"):
            if label in processes:
                status[f"{label}_returncode"] = stop(processes[label])
        for handle in handles:
            handle.close()
        if saved_log.is_dir():
            shutil.copytree(saved_log, source_log, dirs_exist_ok=True)
        if not status.get("error"):
            if status.get("activation_returncode") != 0:
                status["error"] = "Nav2 lifecycle activation failed"
            if status.get("planner_audit_returncode") != 0:
                status["error"] = "Planner audit did not finish"
            recorded = directory / "perception/status.json"
            evidence = json.loads(recorded.read_text()) if recorded.exists() else {}
            if evidence.get("diagnostics", 0) < 100 or evidence.get("costmaps", 0) < 20:
                status["error"] = "Insufficient perception/costmap output"
        status["elapsed_s"] = time.monotonic() - began
        (directory / "run_status.json").write_text(json.dumps(status, indent=2), encoding="utf-8")
        print(f"END {case} {status}", flush=True)
    return 0 if status.get("player_returncode") == 0 and not status.get("error") else 1


if __name__ == "__main__":
    sys.exit(run(Path(sys.argv[1]), sys.argv[2], int(sys.argv[3])))
