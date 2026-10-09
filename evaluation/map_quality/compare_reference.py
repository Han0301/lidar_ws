#!/usr/bin/env python3
"""Paired core comparison on saved real deskewed scans; no ground-truth accuracy."""
import argparse
import csv
import json
import subprocess
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from common import describe, sha256, write_json


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    source = args.capture
    odometry = list(csv.DictReader((source/'odometry.csv').open()))
    times = np.array([float(r['stamp_s']) for r in odometry])
    transforms = list(csv.DictReader((source/'tf.csv').open()))
    fixed = next(r for r in transforms if r['parent'] == 'odom' and r['child'] == 'camera_init')
    gravity = Rotation.from_quat([float(fixed[k]) for k in ['qx','qy','qz','qw']]).as_matrix()
    inputs = args.output/'input'
    inputs.mkdir()
    manifest = {}
    for file in sorted(source.glob('body_*.npz')):
        scan = np.load(file)
        stamp = int(scan['stamp_ns']) / 1e9
        idx = int(np.argmin(abs(times-stamp)))
        if abs(times[idx]-stamp) > 1e-6:
            raise ValueError('No matching scan odometry: '+file.name)
        row = odometry[idx]
        world = gravity @ Rotation.from_quat([float(row[k]) for k in ['qx','qy','qz','qw']]).as_matrix()
        yaw = np.arctan2(world[1,0], world[0,0])
        level = Rotation.from_euler('z', -yaw).as_matrix() @ world
        xyz = np.asarray(scan['points'], dtype=np.float32)
        points = np.column_stack([xyz, np.zeros(len(xyz), dtype=np.float32)])
        with (inputs/(file.stem+'.bin')).open('wb') as out:
            out.write(level.astype('<f4').tobytes())
            out.write(points.astype('<f4').tobytes())
        manifest[file.name] = sha256(file)
    benchmark = Path('/home/h/lidar_ws/evaluation/lidar_navigation/results/tools/benchmark')
    results = {}
    for name, reference in [('baseline', '0.10'), ('dense_reference', '0.05')]:
        directory = args.output/name
        with (args.output/(name+'.log')).open('w') as log:
            subprocess.run([str(benchmark), str(inputs), str(directory), '0.10', 'patchwork', 'none', reference],
                           stdout=log, stderr=subprocess.STDOUT, check=True)
        rows = list(csv.DictReader((directory/'metrics.csv').open()))
        reasons = {}
        for row in rows:
            key = row['reference_status']
            reasons[key] = reasons.get(key,0)+1
        results[name] = dict(frames=len(rows), valid=sum(int(r['ground_valid']) for r in rows),
                            reasons=reasons, processing_ms=describe([float(r['ms']) for r in rows]))
    write_json(args.output/'comparison.json', results)
    write_json(args.output/'manifest.json', dict(mode='saved_real_scan_core_benchmark',
        source=str(source.resolve()), inputs=manifest, benchmark_sha256=sha256(benchmark),
        fixed_gravity_transform=fixed, classifier_voxel=0.1, references=[0.1,0.05],
        accuracy_ground_truth=False, intensity_replaced_with_zero=True))
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
