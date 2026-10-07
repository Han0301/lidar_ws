#!/usr/bin/env python3
"""Capture deskewed scans for offline annotation; not a production ROS node."""
import csv
import json
from pathlib import Path
import signal
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry

class Capture(Node):
    def __init__(self, output):
        super().__init__('outdoor_deskewed_capture')
        self.output = output
        self.output.mkdir(parents=True, exist_ok=True)
        self.start = None
        self.last_save = -10.
        self.scans = 0
        self.rows = []
        self.create_subscription(PointCloud2, '/cloud_registered_body', self.cloud, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/Odometry', self.odom, 50)
    def cloud(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.scans += 1
        if self.start is None:
            self.start = stamp
        elapsed = stamp-self.start
        if elapsed-self.last_save < 0.95:
            return
        names = [f.name for f in msg.fields]
        fields = ['x','y','z']+(['intensity'] if 'intensity' in names else [])
        points = point_cloud2.read_points_numpy(msg, field_names=fields, skip_nans=True)
        np.savez_compressed(self.output/f'scan_{elapsed:07.2f}.npz', points=points,
                            stamp=stamp, elapsed=elapsed, frame=msg.header.frame_id)
        self.last_save = elapsed
    def odom(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.rows.append([stamp,p.x,p.y,p.z,q.x,q.y,q.z,q.w])
    def save(self):
        with (self.output/'odometry.csv').open('w') as f:
            writer=csv.writer(f);writer.writerow(['stamp_s','x','y','z','qx','qy','qz','qw']);writer.writerows(self.rows)
        (self.output/'status.json').write_text(json.dumps({'clouds_received':self.scans,'odometry':len(self.rows),'saved_scans':len(list(self.output.glob('scan_*.npz')))},indent=2))

rclpy.init()
node=Capture(Path(sys.argv[1]))
try:
    rclpy.spin(node)
except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):
    pass
finally:
    node.save();node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
