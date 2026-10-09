#!/usr/bin/env python3
"""Isolated real-bag replay using existing nodes and existing lifecycle gate."""
import argparse
import importlib.util
import json
import os
import platform
import shutil
import subprocess
import sys
import time
from pathlib import Path

import yaml

from common import sha256, write_json
from resource_profile import ResourceProfile


def run(args):
    cfg = yaml.safe_load(args.config.read_text())
    ws = Path(cfg['workspace'])
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    spec = importlib.util.spec_from_file_location('prior_runner', ws/'evaluation/run_playback.py')
    prior = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(prior)
    if subprocess.run(['pgrep', '-x', 'fastlio_mapping'], capture_output=True).returncode == 0:
        raise RuntimeError('FAST-LIO is running; isolated replay refused')
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID=str(cfg['domain']), ROS_HOME=str(out/'ros_home'),
               ROS_LOG_DIR=str(out/'ros_logs'), RCUTILS_COLORIZED_OUTPUT='0',
               RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}] [{time}] [{name}]: {message}')
    source = ws/'src/FAST_LIO/config/mid360.yaml'
    lio = yaml.safe_load(source.read_text())
    params = lio['/**']['ros__parameters']
    params['map_file_path'] = str(out/'map.pcd')
    # 与此前离线基线一致，仅关闭磁盘 PCD 保存；处理算法参数完全保持
    params['pcd_save']['pcd_save_en'] = False
    (out/'mid360.yaml').write_text(yaml.safe_dump(lio), encoding='utf-8')
    for name, file in [('perception', ws/'install/share/lidar_perception/config/perception.yaml'),
                       ('bridge', ws/'install/share/lidar_nav2_bringup/config/bridge.yaml'),
                       ('costmap', ws/'install/share/lidar_nav2_bringup/config/bag_costmap.yaml')]:
        shutil.copy2(Path(cfg.get(name+'_config', file)), out/(name+'.yaml'))
    if cfg.get('mapping_enabled', False):
        mapping = yaml.safe_load(Path(cfg['mapping_config']).read_text())
        mapping['lidar_mapping']['ros__parameters'].update(use_sim_time=True, output_prefix=str(out/'global_map'))
        (out/'mapping.yaml').write_text(yaml.safe_dump(mapping), encoding='utf-8')
    shutil.copy2(args.config, out/'quality.yaml')
    source_log = ws/'src/FAST_LIO/Log'
    before = out/'fast_lio_log_before'
    shutil.copytree(source_log, before)
    config_manifest = {str(p): sha256(p) for p in [source, out/'mid360.yaml', out/'perception.yaml',
                                                   out/'bridge.yaml', out/'costmap.yaml', args.config]}
    write_json(out/'manifest.json', dict(mode='offline_rosbag_replay', bag=str(args.bag.resolve()),
        bag_sha256={f.name: sha256(f) for f in args.bag.glob('*.mcap')}, configs=config_manifest,
        git_head=subprocess.check_output(['git', '-C', str(ws), 'rev-parse', 'HEAD'], text=True).strip(),
        git_status=subprocess.check_output(['git', '-C', str(ws), 'status', '--short'], text=True),
        host=platform.platform(), rate=cfg['playback_rate'], observer='read_only',
        experiment_profile=cfg.get('experiment_profile', 'original'),
        production_parameter_changes=([cfg.get('perception_config'), cfg.get('costmap_config')]
                                      if 'experiment_profile' in cfg else []), output_only_changes=['pcd_save_en=false', 'map_file_path'],
        mapping_config_sha256=sha256(out/'mapping.yaml') if (out/'mapping.yaml').exists() else None,
        source_sha256={str(f.relative_to(ws)):sha256(f) for package in ['lidar_perception','lidar_mapping','lidar_nav2_bringup']
                       for f in (ws/'src'/package).rglob('*') if f.is_file() and f.suffix in ['.cpp','.hpp','.py','.yaml','.xml','.txt']},
        nav2_debug_timing=True, started_unix_s=time.time()))
    jobs = {}
    handles = []
    status = {}
    began = time.monotonic()
    profiler = ResourceProfile(out, jobs)
    profiler.start()

    def start(label, cmd):
        process, handle = prior.start(cmd, label, out, env)
        jobs[label] = process
        handles.append(handle)

    try:
        here = Path(__file__).resolve().parent
        start('observer', [sys.executable, str(here/'record_quality.py'), '--config', str(args.config),
                           '--output', str(out/'capture')])
        start('fast_lio', ['ros2', 'launch', 'fast_lio', 'mapping.launch.py', f'config_path:={out}',
                           'config_file:=mid360.yaml', 'use_sim_time:=true', 'rviz:=false'])
        if args.navigation:
            start('bridge', ['ros2', 'run', 'lidar_nav2_bringup', 'lio_nav_bridge', '--ros-args',
                             '--params-file', str(out/'bridge.yaml')])
            start('perception', ['ros2', 'run', 'lidar_perception', 'perception_node', '--ros-args',
                                '-r', '__node:=lidar_perception', '--params-file', str(out/'perception.yaml')])
            start('costmap', ['ros2', 'run', cfg.get('costmap_package', 'nav2_costmap_2d'),
                             cfg.get('costmap_executable', 'nav2_costmap_2d'), '--ros-args',
                             '-r', '__node:=local_costmap', '-r', '__ns:=/local_costmap',
                             '--params-file', str(out/'costmap.yaml'), '--log-level',
                             'local_costmap.local_costmap:=debug'])
            start('manager', ['ros2', 'run', 'nav2_lifecycle_manager', 'lifecycle_manager', '--ros-args',
                             '-r', '__node:=lifecycle_manager_costmap', '-p', 'use_sim_time:=true',
                             '-p', 'autostart:=false', '-p', 'bond_timeout:=0.0',
                             '-p', 'node_names:=[local_costmap/local_costmap]'])
        if args.navigation and cfg.get('mapping_enabled', False):
            start('mapping', ['ros2', 'run', 'lidar_mapping', 'mapping_node', '--ros-args',
                              '--params-file', str(out/'mapping.yaml')])
        time.sleep(4)
        for name, job in jobs.items():
            if job.poll() is not None:
                raise RuntimeError(f'{name} exited before replay: {job.returncode}')
        if args.navigation:
            start('activation', [sys.executable, str(ws/'evaluation/lidar_navigation/activate_costmap.py')])
        start('player', ['ros2', 'bag', 'play', str(args.bag.resolve()), '--clock', '100', '-r', str(cfg['playback_rate'])])
        print('Replay started:', args.bag, flush=True)
        status['player_returncode'] = jobs['player'].wait(timeout=720)
        for name, job in jobs.items():
            if name not in ['activation', 'player'] and job.poll() is not None:
                raise RuntimeError(f'{name} exited during replay: {job.returncode}')
        time.sleep(2)
        print('Replay complete; collecting final output', flush=True)
    except BaseException as e:
        status['error'] = f'{type(e).__name__}: {e}'
    finally:
        if args.navigation and 'manager' in jobs and jobs['manager'].poll() is None:
            try:
                with (out/'lifecycle_shutdown.log').open('w') as log:
                    stopped = subprocess.run([sys.executable, str(here/'lifecycle_command.py'),
                        '/lifecycle_manager_costmap/manage_nodes', '4'], env=env,
                        stdout=log, stderr=subprocess.STDOUT, timeout=20)
                    status['lifecycle_shutdown_returncode'] = stopped.returncode
            except Exception as e:
                status['lifecycle_shutdown_error'] = str(e)
        for name in ['player', 'activation', 'costmap', 'manager', 'mapping', 'perception', 'bridge', 'fast_lio', 'observer']:
            if name in jobs:
                status[name+'_returncode'] = prior.stop(jobs[name])
        profiler.stop()
        for handle in handles:
            handle.close()
        shutil.copytree(source_log, out/'fast_lio_log_generated')
        shutil.copytree(before, source_log, dirs_exist_ok=True)
        status['elapsed_wall_s'] = time.monotonic()-began
        try:
            observed = json.loads((out/'capture/status.json').read_text())
            if observed['counts'].get('body', 0) < 100 or observed['worker_errors']:
                status['error'] = 'Insufficient clouds or observer write errors'
            if args.navigation and (status.get('activation_returncode') != 0 or observed['counts'].get('costmap', 0) < 20):
                status['error'] = 'Missing active costmap output'
        except Exception as e:
            status['error'] = 'Observer output unavailable: '+str(e)
        if status.get('player_returncode') != 0:
            status['error'] = status.get('error', 'Replay did not complete')
        status['measurement_complete'] = 'error' not in status
        abnormal={name:status.get(name+'_returncode') for name in jobs
                  if name not in ['player','activation'] and status.get(name+'_returncode') not in [0,-2,130]}
        if abnormal:
            status['shutdown_error'] = abnormal
        write_json(out/'run_status.json', status)
    print(json.dumps(status), flush=True)
    return int('error' in status or 'shutdown_error' in status)


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--config', type=Path, default=Path(__file__).with_name('quality.yaml'))
    p.add_argument('--bag', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--navigation', action='store_true')
    sys.exit(run(p.parse_args()))
