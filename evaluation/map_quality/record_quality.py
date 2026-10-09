#!/usr/bin/env python3
"""Read-only observer: timestamps, TF, clouds, actual update footprint, raw grids."""
import argparse
import csv
import json
import queue
import signal
import threading
import time
from collections import Counter
from pathlib import Path

import numpy as np
import rclpy
import yaml
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PolygonStamped
from livox_ros_driver2.msg import CustomMsg
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection3DArray

from common import rotation, write_json


def stamp_ns(header):
    return header.stamp.sec * 1000000000 + header.stamp.nanosec


class Recorder(Node):

    def __init__(self, args):
        super().__init__('map_quality_observer')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.out = args.output
        self.cfg = yaml.safe_load(args.config.read_text())
        self.out.mkdir(parents=True, exist_ok=False)
        self.counts = Counter()
        self.stats = Counter()
        self.last_saved = {}
        self.pending = []
        self.files = []
        self.writers = {}
        self.saved = Counter()
        self.work = queue.Queue(maxsize=128)
        self.worker_errors = []
        self.worker = threading.Thread(target=self.save_worker, daemon=True)
        self.worker.start()
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.last_map = None
        self.open_csv('events', ['topic', 'stamp_ns', 'arrival_mono_ns', 'ros_now_ns', 'points', 'end_ns'])
        self.open_csv('diagnostics', ['arrival_mono_ns', 'stamp_ns', 'name', 'state', 'values_json'])
        self.open_csv('odometry', ['stamp_s', 'frame_id', 'child_frame_id', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        self.open_csv('objects', ['stamp_ns', 'index', 'x', 'y', 'z', 'sx', 'sy', 'sz'])
        self.open_csv('tf', ['stamp_ns', 'parent', 'child', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'static'])
        self.create_subscription(CustomMsg, '/livox/lidar', self.raw, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/Odometry', self.odom, QoSProfile(depth=100))
        self.create_subscription(PolygonStamped, '/local_costmap/published_footprint',
                                 lambda m: self.event(m, 'update_footprint'), QoSProfile(depth=100))
        self.create_subscription(Detection3DArray, '/perception/objects', self.objects, QoSProfile(depth=50))
        for label, topic in [('body', '/cloud_registered_body'), ('ground', '/perception/ground'),
                             ('obstacles', '/perception/obstacles'), ('clearing', '/perception/clearing')]:
            self.create_subscription(PointCloud2, topic, lambda m, label=label: self.cloud(m, label),
                                     QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(PointCloud2, '/Laser_map', self.map_cloud,
                                 QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Costmap, '/local_costmap/costmap_raw', self.costmap,
                                 QoSProfile(depth=20, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(OccupancyGrid, '/map', self.global_map,
            QoSProfile(depth=5, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        for topic in ['/perception/diagnostics', '/nav/diagnostics', '/mapping/diagnostics']:
            self.create_subscription(DiagnosticArray, topic, self.diagnostic, QoSProfile(depth=100))
        self.create_subscription(TFMessage, '/tf', lambda m: self.tf(m, False), QoSProfile(depth=100))
        self.create_subscription(TFMessage, '/tf_static', lambda m: self.tf(m, True),
                                 QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(.01, self.resolve_pending, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def open_csv(self, name, fields):
        f = (self.out / (name+'.csv')).open('w', encoding='utf-8', newline='')
        self.files.append(f)
        self.writers[name] = csv.writer(f)
        self.writers[name].writerow(fields)

    def event(self, msg, topic, points=0, end_ns=0):
        arrival = time.monotonic_ns()
        self.counts[topic] += 1
        self.writers['events'].writerow([topic, stamp_ns(msg.header), arrival,
                                        self.get_clock().now().nanoseconds, points, end_ns])
        return arrival

    def raw(self, msg):
        end = msg.timebase + max((p.offset_time for p in msg.points), default=0)
        self.event(msg, 'raw_lidar', msg.point_num, end)

    def odom(self, msg):
        self.event(msg, 'odometry')
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.writers['odometry'].writerow([stamp_ns(msg.header)/1e9, msg.header.frame_id,
                                          msg.child_frame_id, p.x, p.y, p.z, q.x, q.y, q.z, q.w])

    def tf(self, msg, static):
        for m in msg.transforms:
            p, q = m.transform.translation, m.transform.rotation
            self.writers['tf'].writerow([stamp_ns(m.header), m.header.frame_id, m.child_frame_id,
                                        p.x, p.y, p.z, q.x, q.y, q.z, q.w, int(static)])

    def diagnostic(self, msg):
        for s in msg.status:
            self.writers['diagnostics'].writerow([time.monotonic_ns(), stamp_ns(msg.header),
                s.name, s.message, json.dumps({v.key: v.value for v in s.values})])

    def objects(self, msg):
        self.event(msg, 'objects', len(msg.detections))
        for i, d in enumerate(msg.detections):
            p, s = d.bbox.center.position, d.bbox.size
            self.writers['objects'].writerow([stamp_ns(msg.header), i, p.x, p.y, p.z, s.x, s.y, s.z])

    def cloud(self, msg, label):
        self.event(msg, label, msg.width*msg.height)
        s = stamp_ns(msg.header)
        full = self.cfg.get('capture_mapping_inputs', False)
        if label == 'clearing' and not full:
            return
        period = 0 if full or label == 'obstacles' else self.cfg['scan_sample_period_s']
        if (s-self.last_saved.get(label, -10**20))/1e9 < period-1e-6:
            return
        self.last_saved[label] = s
        fields = ['x', 'y', 'z', 'intensity'] if label == 'body' and full else ['x', 'y', 'z']
        values = point_cloud2.read_points_numpy(msg, field_names=fields, skip_nans=False).reshape(-1, len(fields))
        payload = dict(points=values[:, :3], stamp_ns=s, frame=msg.header.frame_id)
        if len(fields) == 4:
            payload['intensity'] = values[:, 3]
        if label == 'body':
            self.submit(label, s, payload)
        else:
            self.pending.append((label, s, payload, time.monotonic()))

    def resolve_pending(self):
        remain = []
        for label, s, data, began in self.pending:
            try:
                t = self.buffer.lookup_transform('odom', str(data['frame']), Time(nanoseconds=s))
                p, q = t.transform.translation, t.transform.rotation
                data['rotation'] = rotation([q.x, q.y, q.z, q.w])
                data['translation'] = np.array([p.x, p.y, p.z])
                self.submit(label, s, data)
            except Exception:
                if time.monotonic()-began < 1:
                    remain.append((label, s, data, began))
                else:
                    self.stats['observer_tf_unresolved'] += 1
        self.pending = remain

    def costmap(self, msg):
        self.event(msg, 'costmap')
        m = msg.metadata
        self.submit('costmap', stamp_ns(msg.header),
                    dict(grid=np.asarray(msg.data, dtype=np.uint8).reshape(m.size_y, m.size_x),
                         origin=[m.origin.position.x, m.origin.position.y], resolution=m.resolution,
                         stamp_ns=stamp_ns(msg.header), frame=msg.header.frame_id))

    def global_map(self, msg):
        self.event(msg, 'global_map')
        m = msg.info
        self.submit('global_map', stamp_ns(msg.header),
                    dict(grid=np.asarray(msg.data, dtype=np.int8).reshape(m.height, m.width),
                         origin=[m.origin.position.x, m.origin.position.y], resolution=m.resolution,
                         stamp_ns=stamp_ns(msg.header), frame=msg.header.frame_id))

    def map_cloud(self, msg):
        self.event(msg, 'published_map', msg.width*msg.height)
        self.last_map = msg

    def submit(self, label, stamp, payload):
        try:
            self.work.put_nowait((label, stamp, payload))
        except queue.Full:
            self.stats['observer_save_queue_dropped'] += 1

    def save_worker(self):
        while True:
            task = self.work.get()
            try:
                if task is None:
                    return
                label, stamp, payload = task
                np.savez_compressed(self.out / f'{label}_{stamp}.npz', **payload)
                self.saved[label] += 1
            except Exception as e:
                self.worker_errors.append(str(e))
            finally:
                self.work.task_done()

    def close(self):
        self.resolve_pending()
        self.stats['observer_pending_at_shutdown'] = len(self.pending)
        self.work.join()
        self.work.put(None)
        self.worker.join()
        if self.last_map is not None:
            m = self.last_map
            xyz = point_cloud2.read_points_numpy(m, field_names=['x', 'y', 'z'], skip_nans=True).reshape(-1, 3)
            np.savez_compressed(self.out/'published_map.npz', points=xyz, stamp_ns=stamp_ns(m.header), frame=m.header.frame_id)
        for f in self.files:
            f.close()
        write_json(self.out/'status.json', dict(counts=dict(self.counts), saved=dict(self.saved),
                                               observer_stats=dict(self.stats), worker_errors=self.worker_errors))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--config', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    rclpy.init()
    n = Recorder(args)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        n.close()
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
