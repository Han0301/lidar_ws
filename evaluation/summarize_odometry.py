#!/usr/bin/env python3
"""Summarize one FAST-LIO odometry recording without claiming ground-truth accuracy."""

import csv
import json
import math
import statistics
import sys
from pathlib import Path


def distance(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def summarize(path):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        return {"messages": 0}
    stamp = [float(row["stamp_s"]) for row in rows]
    xyz = [tuple(float(row[axis]) for axis in ("x", "y", "z")) for row in rows]
    quaternion = [tuple(float(row[axis]) for axis in ("qx", "qy", "qz", "qw")) for row in rows]
    qdot = abs(sum(x * y for x, y in zip(quaternion[0], quaternion[-1])))
    qdot /= math.sqrt(sum(x * x for x in quaternion[0]) * sum(x * x for x in quaternion[-1]))
    qdot = min(1.0, qdot)
    intervals = [b - a for a, b in zip(stamp, stamp[1:])]
    start = xyz[0]
    end = xyz[-1]
    return {
        "messages": len(rows),
        "frame_ids": sorted({row["frame_id"] for row in rows}),
        "child_frame_ids": sorted({row["child_frame_id"] for row in rows}),
        "duration_s": stamp[-1] - stamp[0],
        "start_xyz_m": start,
        "end_xyz_m": end,
        "end_minus_start_xyz_m": tuple(y - x for x, y in zip(start, end)),
        "closure_3d_m": distance(start, end),
        "closure_xy_m": math.hypot(end[0] - start[0], end[1] - start[1]),
        "start_end_orientation_angle_deg": math.degrees(2 * math.acos(qdot)),
        "trajectory_length_m": sum(distance(a, b) for a, b in zip(xyz, xyz[1:])),
        "max_distance_from_start_m": max(distance(start, point) for point in xyz),
        "xyz_range_m": tuple(max(v) - min(v) for v in zip(*xyz)),
        "xyz_std_m": tuple(statistics.pstdev(v) for v in zip(*xyz)),
        "nonmonotonic_stamp": sum(value <= 0 for value in intervals),
        "max_odom_interval_s": max(intervals, default=None),
        "odom_intervals_over_0_15s": sum(value > 0.15 for value in intervals),
    }


if __name__ == "__main__":
    path = Path(sys.argv[1])
    result = summarize(path)
    output = Path(sys.argv[2])
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if result["messages"] == 0:
        sys.exit("No odometry messages were recorded")
