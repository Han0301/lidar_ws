#!/usr/bin/env python3
"""Experimental LiDAR-only scan rotation vs IMU gyro lag check for recorded bags.

This is a diagnostic, not a timestamp calibration: 100 ms rolling scans, geometry,
ICP failures, and LiDAR-IMU extrinsics can bias the estimate.
"""

import json
import sys
from pathlib import Path

import numpy as np
import rosbag2_py
from livox_ros_driver2.msg import CustomMsg
from rclpy.serialization import deserialize_message
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Imu


def stamp_s(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def scan_points(msg, stride=10):
    pts = np.asarray([(p.x, p.y, p.z) for p in msg.points[::stride]], dtype=np.float64)
    dist = np.linalg.norm(pts, axis=1)
    return pts[np.isfinite(dist) & (dist >= 0.7) & (dist <= 35.0)]


def rigid_fit(src, dst):
    src_mean = src.mean(axis=0)
    dst_mean = dst.mean(axis=0)
    u, _, vt = np.linalg.svd((src - src_mean).T @ (dst - dst_mean))
    fix = np.diag([1.0, 1.0, np.linalg.det(u @ vt)])
    rot = vt.T @ fix @ u.T
    trans = dst_mean - rot @ src_mean
    return rot, trans


def icp_rotation(current, previous, iterations=5):
    if len(current) < 300 or len(previous) < 300:
        return None
    tree = cKDTree(previous)
    rot = np.eye(3)
    trans = np.zeros(3)
    count = 0
    median_distance = float("inf")
    for _ in range(iterations):
        moved = current @ rot.T + trans
        distances, matches = tree.query(moved, k=1)
        usable = distances < 1.0
        if usable.sum() < 300:
            return None
        cutoff = min(0.65, float(np.quantile(distances[usable], 0.7)))
        good = usable & (distances <= cutoff)
        if good.sum() < 250:
            return None
        delta_rot, delta_trans = rigid_fit(moved[good], previous[matches[good]])
        rot = delta_rot @ rot
        trans = delta_rot @ trans + delta_trans
        count = int(good.sum())
        median_distance = float(np.median(distances[good]))

    _, neighborhoods = tree.query(previous, k=12)
    neighbors = previous[neighborhoods]
    centered = neighbors - neighbors.mean(axis=1, keepdims=True)
    covariance = np.einsum("nki,nkj->nij", centered, centered) / centered.shape[1]
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    normals = eigenvectors[:, :, 0]
    planar = eigenvalues[:, 0] / np.maximum(eigenvalues.sum(axis=1), 1e-12) < 0.08
    for _ in range(6):
        moved = current @ rot.T + trans
        distances, matches = tree.query(moved, k=1)
        n = normals[matches]
        residuals = np.sum(n * (moved - previous[matches]), axis=1)
        good = (distances < 0.7) & planar[matches] & (np.abs(residuals) < 0.25)
        if good.sum() < 250:
            return None
        jacobian = np.column_stack((np.cross(moved[good], n[good]), n[good]))
        delta, _, _, _ = np.linalg.lstsq(jacobian, -residuals[good], rcond=None)
        if np.linalg.norm(delta[:3]) > 0.3 or np.linalg.norm(delta[3:]) > 0.5:
            return None
        delta_rot = Rotation.from_rotvec(delta[:3]).as_matrix()
        rot = delta_rot @ rot
        trans = delta_rot @ trans + delta[3:]
        count = int(good.sum())
        median_distance = float(np.median(np.abs(residuals[good])))
    return rot, count, median_distance


def load_bag(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    clouds = []
    gyro_time = []
    gyro = []
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic == "/livox/lidar":
            msg = deserialize_message(raw, CustomMsg)
            center = stamp_s(msg.header) + max((p.offset_time for p in msg.points), default=0) * 0.5e-9
            clouds.append((center, scan_points(msg)))
        elif topic == "/livox/imu":
            msg = deserialize_message(raw, Imu)
            gyro_time.append(stamp_s(msg.header))
            gyro.append((msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z))
    return clouds, np.asarray(gyro_time), np.asarray(gyro)


def fit_gyro_to_lidar(gyro_rate, lidar_rate):
    a = gyro_rate - gyro_rate.mean(axis=0)
    options = []
    for sign in (1, -1):
        b = sign * (lidar_rate - lidar_rate.mean(axis=0))
        cov = a.T @ b
        u, _, vt = np.linalg.svd(cov)
        fix = np.diag([1.0, 1.0, np.linalg.det(u @ vt)])
        rot = vt.T @ fix @ u.T
        predicted = a @ rot.T
        scale = float(np.sum(predicted * b) / np.sum(predicted ** 2))
        error = np.linalg.norm(b - scale * predicted, axis=1)
        rms = float(np.sqrt(np.mean(error ** 2)))
        explained = float(1 - np.sum(error ** 2) / np.sum(np.linalg.norm(b, axis=1) ** 2))
        options.append((explained, rms, sign, scale))
    return max(options)


def lag_scan(pairs, gyro_time, gyro):
    gyro_prefix = np.vstack((np.zeros((1, 3)), np.cumsum(gyro, axis=0)))
    begin = np.asarray([x[0] for x in pairs])
    end = np.asarray([x[1] for x in pairs])
    lidar_rate = np.asarray([x[2] for x in pairs])
    results = []
    for lag_ms in range(-100, 101, 5):
        delta = lag_ms / 1000
        left = np.searchsorted(gyro_time, begin + delta, side="left")
        right = np.searchsorted(gyro_time, end + delta, side="right")
        count = right - left
        valid = count >= 8
        if valid.sum() < 30:
            continue
        gyro_rate = (gyro_prefix[right[valid]] - gyro_prefix[left[valid]]) / count[valid, None]
        explained, rms, sign, scale = fit_gyro_to_lidar(gyro_rate, lidar_rate[valid])
        results.append({"lag_ms": lag_ms, "pairs": int(valid.sum()), "rms_rad_s": rms,
                        "explained_fraction": explained, "icp_rotation_sign": sign,
                        "icp_to_gyro_scale": scale})
    return results


def analyze(bag):
    clouds, gyro_time, gyro = load_bag(bag)
    pairs = []
    quality = []
    for index in range(1, len(clouds)):
        before, previous = clouds[index - 1]
        after, current = clouds[index]
        duration = after - before
        if not 0.08 <= duration <= 0.12:
            continue
        result = icp_rotation(current, previous)
        if result is None:
            continue
        rot, inliers, residual = result
        angular_rate = Rotation.from_matrix(rot).as_rotvec() / duration
        if np.linalg.norm(angular_rate) > 4.0 or residual > 0.5:
            continue
        pairs.append((before, after, angular_rate))
        quality.append((inliers, residual))
        if index % 100 == 0:
            print(f"scans={index}/{len(clouds)} valid_pairs={len(pairs)}", flush=True)
    if len(pairs) < 50:
        raise RuntimeError(f"Only {len(pairs)} LiDAR-only scan pairs passed ICP quality checks")
    full = lag_scan(pairs, gyro_time, gyro)
    midpoint = (pairs[0][0] + pairs[-1][1]) / 2
    first = lag_scan([p for p in pairs if p[1] <= midpoint], gyro_time, gyro)
    second = lag_scan([p for p in pairs if p[0] >= midpoint], gyro_time, gyro)
    def best(items):
        return max(items, key=lambda item: item["explained_fraction"]) if items else None
    return {
        "bag": bag.name,
        "icp_method": "point-to-point initial, point-to-plane refinement",
        "clouds": len(clouds),
        "valid_scan_pairs": len(pairs),
        "median_icp_inliers": float(np.median([x[0] for x in quality])),
        "median_icp_residual_m": float(np.median([x[1] for x in quality])),
        "best_full": best(full),
        "best_first_half": best(first),
        "best_second_half": best(second),
        "scan": full,
        "note": "Experimental correlation only; not an independently calibrated physical time offset.",
    }


if __name__ == "__main__":
    result = analyze(Path(sys.argv[1]))
    output = Path(sys.argv[2])
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({key: result[key] for key in (
        "bag", "valid_scan_pairs", "median_icp_residual_m",
        "best_full", "best_first_half", "best_second_half")}, ensure_ascii=False), flush=True)
