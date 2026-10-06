#!/usr/bin/env python3
"""Audit recorded MID-360 LiDAR/IMU timestamp consistency, without inferring calibration."""

import bisect
import json
import sys
from pathlib import Path

import rosbag2_py
from livox_ros_driver2.msg import CustomMsg
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def describe(values):
    if not values:
        return None
    values = sorted(values)

    def percentile(p):
        index = (len(values) - 1) * p
        low = int(index)
        high = min(low + 1, len(values) - 1)
        return values[low] + (values[high] - values[low]) * (index - low)

    return {
        "count": len(values),
        "min": values[0],
        "median": percentile(0.5),
        "p95": percentile(0.95),
        "max": values[-1],
    }


def audit(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    lidar = []
    imu = []
    lidar_delay_ms = []
    imu_delay_ms = []
    mismatch = 0
    nonzero_first_offset = 0
    imu_nonmonotonic = 0
    lidar_nonmonotonic = 0
    last_imu_stamp = None
    last_lidar_stamp = None
    while reader.has_next():
        topic, raw, recorded_ns = reader.read_next()
        if topic == "/livox/imu":
            msg = deserialize_message(raw, Imu)
            timestamp = stamp_ns(msg.header)
            imu_nonmonotonic += last_imu_stamp is not None and timestamp <= last_imu_stamp
            last_imu_stamp = timestamp
            imu.append(timestamp)
            imu_delay_ms.append((recorded_ns - timestamp) / 1e6)
        elif topic == "/livox/lidar":
            msg = deserialize_message(raw, CustomMsg)
            timestamp = stamp_ns(msg.header)
            lidar_nonmonotonic += last_lidar_stamp is not None and timestamp <= last_lidar_stamp
            last_lidar_stamp = timestamp
            offsets = [point.offset_time for point in msg.points]
            mismatch += timestamp != msg.timebase
            mismatch += msg.point_num != len(offsets)
            nonzero_first_offset += bool(offsets and min(offsets) != 0)
            lidar.append((timestamp, timestamp + max(offsets, default=0)))
            lidar_delay_ms.append((recorded_ns - timestamp) / 1e6)

    imu.sort()
    before_begin_ms = []
    after_end_ms = []
    missing_begin = 0
    missing_end = 0
    imu_in_scan = []
    for begin, end in lidar:
        first_inside = bisect.bisect_left(imu, begin)
        first_after = bisect.bisect_left(imu, end)
        missing_begin += first_inside == 0
        missing_end += first_after == len(imu)
        if first_inside > 0:
            before_begin_ms.append((begin - imu[first_inside - 1]) / 1e6)
        if first_after < len(imu):
            after_end_ms.append((imu[first_after] - end) / 1e6)
        imu_in_scan.append(bisect.bisect_right(imu, end) - first_inside)

    lidar_stamps = [begin for begin, _ in lidar]
    lidar_intervals_ms = [(b - a) / 1e6 for a, b in zip(lidar_stamps, lidar_stamps[1:])]
    imu_intervals_ms = [(b - a) / 1e6 for a, b in zip(imu, imu[1:])]
    gaps = [
        {"after_scan_index_zero_based": i, "seconds_from_first_scan": (lidar_stamps[i] - lidar_stamps[0]) / 1e9,
         "gap_ms": gap}
        for i, gap in enumerate(lidar_intervals_ms) if gap > 150
    ]
    return {
        "bag": bag.name,
        "lidar_messages": len(lidar),
        "imu_messages": len(imu),
        "stamp_epoch": "unix-like" if min(imu[0], lidar_stamps[0]) > 1_000_000_000_000_000_000 else "other",
        "header_timebase_or_point_count_mismatch": mismatch,
        "lidar_nonmonotonic": lidar_nonmonotonic,
        "imu_nonmonotonic": imu_nonmonotonic,
        "scans_with_nonzero_min_point_offset": nonzero_first_offset,
        "lidar_period_ms": describe(lidar_intervals_ms),
        "imu_period_ms": describe(imu_intervals_ms),
        "lidar_gaps_over_150ms": gaps,
        "imu_gaps_over_15ms": sum(gap > 15 for gap in imu_intervals_ms),
        "imu_before_scan_start_ms": describe(before_begin_ms),
        "imu_after_scan_end_ms": describe(after_end_ms),
        "imu_samples_in_scan": describe(imu_in_scan),
        "scans_without_prior_imu_in_bag": missing_begin,
        "scans_without_later_imu_in_bag": missing_end,
        "bag_record_time_minus_lidar_stamp_ms": describe(lidar_delay_ms),
        "bag_record_time_minus_imu_stamp_ms": describe(imu_delay_ms),
        "physical_lidar_imu_offset_ms": None,
        "physical_offset_note": "Message stamps and coverage cannot establish physical LiDAR-IMU latency or packet time_type.",
    }


if __name__ == "__main__":
    result = audit(Path(sys.argv[1]))
    output = Path(sys.argv[2])
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps({key: result[key] for key in (
        "bag", "lidar_messages", "imu_messages", "scans_without_prior_imu_in_bag",
        "scans_without_later_imu_in_bag", "lidar_gaps_over_150ms")}, ensure_ascii=False), flush=True)
