#!/usr/bin/env python3
"""Replay a read-only SQLite bag fragment through the installed ROS2 node.

Run in a sourced Humble environment. This checks transport and finite outputs,
not trajectory accuracy. Override extrinsics/configuration for your own bag.
"""
import argparse
from collections import Counter
import json
import math
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry, Path as PathMsg
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Float64MultiArray
from tf2_msgs.msg import TFMessage
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path, help='SQLite rosbag .db3 file')
    parser.add_argument('--duration', type=float, default=12.0)
    parser.add_argument('--rate', type=float, default=1.0)
    parser.add_argument('--imu-hz', type=int, choices=[100, 200, 400], default=200)
    parser.add_argument('--no-network', action='store_true')
    parser.add_argument('--best-effort', action='store_true')
    parser.add_argument('--config', default='geode/velodyne_alive.yaml')
    parser.add_argument('--lidar-topic', default='/velodyne_points')
    parser.add_argument('--imu-topic', default='/imu/data')
    args = parser.parse_args()
    if args.rate <= 0 or args.duration <= 0:
        parser.error('duration and rate must be positive')

    # Avoid publishing test data into another ROS domain by default.
    os.environ.setdefault('ROS_DOMAIN_ID', '73')
    rclpy.init()
    node = rclpy.create_node('alive_lio_replay_smoke')
    counts = Counter()
    stamps = []
    failures = []
    frames = set()

    def odom_cb(msg):
        counts['odometry'] += 1
        stamps.append(msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec)
        values = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
                  msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                  msg.pose.pose.orientation.z, msg.pose.pose.orientation.w, *msg.pose.covariance]
        if not all(math.isfinite(v) for v in values):
            failures.append('nonfinite odometry')
        if msg.header.frame_id != 'camera_init' or msg.child_frame_id != 'body':
            failures.append('unexpected odometry frame')

    def tf_cb(msg):
        counts['tf'] += len(msg.transforms)
        frames.update((t.header.frame_id, t.child_frame_id) for t in msg.transforms)

    def network_cb(msg):
        counts['network'] += 1
        if len(msg.data) != 8 or not all(math.isfinite(v) for v in msg.data):
            failures.append('invalid neural velocity output')

    subscriptions = [
        node.create_subscription(Odometry, '/Odometry', odom_cb, 100),
        node.create_subscription(PointCloud2, '/cloud_registered', lambda msg: counts.update(clouds=1), 100),
        node.create_subscription(PathMsg, '/path', lambda msg: counts.update(paths=1), 100),
        node.create_subscription(TFMessage, '/tf', tf_cb, 100),
        node.create_subscription(Float64MultiArray, '/NetworkOuput', network_cb, 100),
    ]
    qos = QoSProfile(depth=1000, reliability=(ReliabilityPolicy.BEST_EFFORT if args.best_effort else ReliabilityPolicy.RELIABLE))
    publishers = {
        args.lidar_topic: node.create_publisher(PointCloud2, args.lidar_topic, qos),
        args.imu_topic: node.create_publisher(Imu, args.imu_topic, qos),
    }
    types = {args.lidar_topic: PointCloud2, args.imu_topic: Imu}
    share = Path(get_package_share_directory('alive_lio'))
    config_path = Path(args.config)
    if not config_path.is_absolute():
        config_path = share / 'config' / config_path
    config = yaml.safe_load(config_path.read_text())
    params = config['/**']['ros__parameters']
    params['common'].update(lid_topic=args.lidar_topic, imu_topic=args.imu_topic)
    params['localizability'].update(en_network=not args.no_network, use_gpu=False,
                                    model_name=f'model_vehicle{args.imu_hz}',
                                    buffer_len=args.imu_hz*5, num_threads=1)
    params['publish']['console_status_en'] = False
    executable = Path(get_package_prefix('alive_lio')) / 'lib/alive_lio/alive_lio_node'
    try:
        with tempfile.TemporaryDirectory(prefix='alive-lio-smoke-') as temp:
            temp = Path(temp)
            params['output_directory'] = str(temp / 'output')
            settings = temp / 'params.yaml'
            settings.write_text(yaml.safe_dump(config))
            with (temp / 'node.log').open('w+') as logfile:
                process = subprocess.Popen([str(executable), '--ros-args', '--params-file', str(settings)],
                                           stdout=logfile, stderr=subprocess.STDOUT)
                try:
                    deadline = time.monotonic() + 20
                    while time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=0.05)
                        if process.poll() is not None:
                            raise RuntimeError(f'node exited during startup: {process.returncode}')
                        if all(pub.get_subscription_count() > 0 for pub in publishers.values()):
                            break
                    else:
                        raise RuntimeError('sensor subscriptions were not discovered')
                    # SQLite URI mode=ro leaves the original recording untouched.
                    connection = sqlite3.connect(args.bag.resolve().as_uri() + '?mode=ro', uri=True)
                    topic_names = dict(connection.execute('SELECT id, name FROM topics'))
                    ids = [i for i, name in topic_names.items() if name in publishers]
                    if len(ids) != 2:
                        raise RuntimeError('bag must contain the configured PointCloud2 and IMU topics')
                    placeholders = ','.join('?' for _ in ids)
                    start = connection.execute(f'SELECT MIN(timestamp) FROM messages WHERE topic_id IN ({placeholders})', ids).fetchone()[0]
                    end = start + int(args.duration * 1e9)
                    rows = connection.execute(f'SELECT topic_id,timestamp,data FROM messages WHERE topic_id IN ({placeholders}) AND timestamp <= ? ORDER BY timestamp,id', [*ids, end])
                    wall_start = time.monotonic()
                    for topic_id, stamp, data in rows:
                        target = wall_start + (stamp-start)/1e9/args.rate
                        while time.monotonic() < target:
                            rclpy.spin_once(node, timeout_sec=min(0.01, target-time.monotonic()))
                        topic = topic_names[topic_id]
                        publishers[topic].publish(deserialize_message(data, types[topic]))
                        counts['input_lidar' if topic == args.lidar_topic else 'input_imu'] += 1
                        rclpy.spin_once(node, timeout_sec=0)
                        if process.poll() is not None:
                            raise RuntimeError(f'node exited during replay: {process.returncode}')
                    connection.close()
                    deadline = time.monotonic() + 3
                    while time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=0.05)
                    assert counts['odometry'] > 10, dict(counts)
                    assert counts['clouds'] > 10 and counts['paths'] > 10, dict(counts)
                    assert ('camera_init', 'body') in frames and ('world', 'camera_init') in frames, frames
                    assert all(a < b for a,b in zip(stamps,stamps[1:])), 'odometry timestamps are not strictly increasing'
                    if not args.no_network:
                        assert counts['network'] > 0, 'no neural inference output'
                    assert not failures, failures[:5]
                finally:
                    if process.poll() is None:
                        process.send_signal(signal.SIGINT)
                        try:
                            process.wait(timeout=10)
                        except subprocess.TimeoutExpired:
                            process.kill()
                            process.wait()
                    logfile.seek(0)
                    log = logfile.read()
                    print(log[-3500:])
                assert process.returncode == 0, f'unclean shutdown: {process.returncode}'
                print(json.dumps({'result':'PASS','network':not args.no_network,
                                  'best_effort':args.best_effort,'counts':dict(counts)}, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
