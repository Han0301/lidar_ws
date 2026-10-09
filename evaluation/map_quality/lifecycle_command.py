#!/usr/bin/env python3
"""Bounded explicit lifecycle command for reproducible clean replay shutdown."""
import argparse
import rclpy
from nav2_msgs.srv import ManageLifecycleNodes

parser = argparse.ArgumentParser()
parser.add_argument('service')
parser.add_argument('command', type=int, choices=range(7))
args = parser.parse_args()
rclpy.init()
node = rclpy.create_node('evaluation_lifecycle_command')
try:
    client = node.create_client(ManageLifecycleNodes, args.service)
    if not client.wait_for_service(timeout_sec=5):
        raise RuntimeError('Lifecycle service unavailable')
    request = ManageLifecycleNodes.Request()
    request.command = args.command
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10)
    if not future.done() or not future.result().success:
        raise RuntimeError('Lifecycle command failed')
finally:
    node.destroy_node()
    rclpy.shutdown()
