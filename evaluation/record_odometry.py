#!/usr/bin/env python3
"""Collect FAST-LIO odometry from one isolated offline playback."""

import csv
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class Recorder(Node):
    def __init__(self, output):
        super().__init__("lidar_bag_eval_odom_recorder")
        self.file = open(output, "w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)
        self.writer.writerow(["stamp_s", "frame_id", "child_frame_id", "x", "y", "z", "qx", "qy", "qz", "qw"])
        self.count = 0
        self.subscription = self.create_subscription(Odometry, "/Odometry", self.on_odom, qos_profile_sensor_data)

    def on_odom(self, message):
        pose = message.pose.pose
        stamp = message.header.stamp
        self.writer.writerow([
            stamp.sec + stamp.nanosec / 1e9,
            message.header.frame_id,
            message.child_frame_id,
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
        ])
        self.count += 1
        if self.count % 100 == 0:
            self.file.flush()

    def close(self):
        self.file.flush()
        self.file.close()


if __name__ == "__main__":
    rclpy.init()
    recorder = Recorder(sys.argv[1])
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        print(f"odometry messages: {recorder.count}", flush=True)
        recorder.close()
        recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
