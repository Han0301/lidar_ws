#!/usr/bin/env python3
"""Read recorded MID-360 messages and report input timing and IMU statistics."""

import bisect
import json
import math
import statistics
import sys
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import Imu


def summary(values):
    if not values:
        return None
    return {
        "count": len(values),
        "min": min(values),
        "mean": statistics.fmean(values),
        "max": max(values),
        "std": statistics.pstdev(values),
    }


def stamps_to_seconds(header):
    return header.stamp.sec + header.stamp.nanosec / 1e9


def analyze(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    lidar = []
    imu = []
    point_counts = []
    scan_ms = []
    acceleration = [[], [], []]
    angular_velocity = [[], [], []]
    acceleration_norm = []
    angular_velocity_norm = []
    lidar_record_delay_ms = []
    imu_record_delay_ms = []
    mismatch = 0
    invalid_scan = 0
    message_count = 0
    while reader.has_next():
        topic, raw, recorded_ns = reader.read_next()
        message_count += 1
        if topic == "/livox/imu":
            msg = deserialize_message(raw, Imu)
            stamp = stamps_to_seconds(msg.header)
            imu.append(stamp)
            imu_record_delay_ms.append((recorded_ns / 1e9 - stamp) * 1000)
            a = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
            w = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
            for axis in range(3):
                acceleration[axis].append(a[axis])
                angular_velocity[axis].append(w[axis])
            acceleration_norm.append(math.sqrt(sum(x * x for x in a)))
            angular_velocity_norm.append(math.sqrt(sum(x * x for x in w)))
        elif topic == "/livox/lidar":
            msg = deserialize_message(raw, CustomMsg)
            stamp = stamps_to_seconds(msg.header)
            lidar.append(stamp)
            lidar_record_delay_ms.append((recorded_ns / 1e9 - stamp) * 1000)
            point_counts.append(msg.point_num)
            if msg.point_num != len(msg.points):
                mismatch += 1
            if abs(stamp - msg.timebase / 1e9) > 1e-6:
                mismatch += 1
            if msg.points:
                scan_ms.append(max(point.offset_time for point in msg.points) / 1e6)
            else:
                invalid_scan += 1

    lidar_dt_ms = [(b - a) * 1000 for a, b in zip(lidar, lidar[1:])]
    imu_dt_ms = [(b - a) * 1000 for a, b in zip(imu, imu[1:])]
    lidar_gaps = [
        {"after_index": index, "relative_time_s": lidar[index] - lidar[0], "gap_ms": value}
        for index, value in enumerate(lidar_dt_ms) if value > 150
    ]
    imu_gaps = [
        {"after_index": index, "relative_time_s": imu[index] - imu[0], "gap_ms": value}
        for index, value in enumerate(imu_dt_ms) if value > 15
    ]
    cover = {"complete": 0, "start_unknown": 0, "missing_end": 0,
             "no_imu_inside": 0, "gap_over_15ms": 0, "scan_duration_outside_50_150ms": 0}
    imu_per_scan = []
    for begin, duration_ms in zip(lidar, scan_ms):
        end = begin + duration_ms / 1000
        left = bisect.bisect_left(imu, begin)
        right = bisect.bisect_right(imu, end)
        start_unknown = left == 0
        missing_end = right == len(imu)
        inside = right - left
        nearby = imu[max(left - 1, 0):min(right + 1, len(imu))]
        gap = max((b - a for a, b in zip(nearby, nearby[1:])), default=0) > 0.015
        invalid_duration = not 50 <= duration_ms <= 150
        imu_per_scan.append(inside)
        cover["start_unknown"] += start_unknown
        cover["missing_end"] += missing_end
        cover["no_imu_inside"] += inside == 0
        cover["gap_over_15ms"] += gap
        cover["scan_duration_outside_50_150ms"] += invalid_duration
        cover["complete"] += not (start_unknown or missing_end or inside == 0 or gap or invalid_duration)

    return {
        "bag": path.name,
        "message_count": message_count,
        "lidar_count": len(lidar),
        "imu_count": len(imu),
        "sensor_duration_s": max(lidar[-1], imu[-1]) - min(lidar[0], imu[0]),
        "lidar_dt_ms": summary(lidar_dt_ms),
        "imu_dt_ms": summary(imu_dt_ms),
        "lidar_nonmonotonic": sum(value <= 0 for value in lidar_dt_ms),
        "imu_nonmonotonic": sum(value <= 0 for value in imu_dt_ms),
        "imu_gaps_over_15ms": sum(value > 15 for value in imu_dt_ms),
        "lidar_gaps_over_150ms": sum(value > 150 for value in lidar_dt_ms),
        "lidar_gap_details": lidar_gaps,
        "imu_gap_details": imu_gaps,
        "point_counts": summary(point_counts),
        "scan_duration_ms": summary(scan_ms),
        "imu_per_scan": summary(imu_per_scan),
        "record_delay_ms": {"lidar": summary(lidar_record_delay_ms),
                            "imu": summary(imu_record_delay_ms)},
        "point_or_timebase_mismatch": mismatch,
        "invalid_scan": invalid_scan,
        "coverage": cover,
        "imu_accel_xyz": [summary(axis) for axis in acceleration],
        "imu_gyro_xyz": [summary(axis) for axis in angular_velocity],
        "imu_accel_norm": summary(acceleration_norm),
        "imu_gyro_norm": summary(angular_velocity_norm),
    }


if __name__ == "__main__":
    bag = Path(sys.argv[1])
    output = Path(sys.argv[2])
    result = analyze(bag)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({key: result[key] for key in ("bag", "lidar_count", "imu_count", "coverage")}, ensure_ascii=False), flush=True)
