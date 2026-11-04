#!/usr/bin/env python3
"""Check both installed ROS2 converters, field values and topic remapping."""
import os
os.environ.setdefault('ROS_DOMAIN_ID', '75')
import math
from pathlib import Path
import signal
import struct
import subprocess
import time
from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


def main():
    rclpy.init()
    node = rclpy.create_node('alive_converter_smoke')
    try:
        prefix = Path(get_package_prefix('alive_lio')) / 'lib/alive_lio'
        for converter in ['pandar_to_velodyne', 'rs_to_velodyne']:
            output = []
            subscriber = node.create_subscription(PointCloud2, '/converter_test_output', output.append, 10)
            publisher = node.create_publisher(PointCloud2, '/converter_test_input', qos_profile_sensor_data)
            input_topic = '/hesai/pandar' if converter == 'pandar_to_velodyne' else '/rslidar_points'
            process = subprocess.Popen([str(prefix/converter), 'XYZIRT', 'XYZIRT', '--ros-args',
                                        '-r', f'{input_topic}:=/converter_test_input',
                                        '-r', '/velodyne_points:=/converter_test_output'])
            try:
                deadline = time.monotonic() + 10
                while time.monotonic() < deadline and publisher.get_subscription_count() == 0:
                    rclpy.spin_once(node, timeout_sec=0.05)
                assert publisher.get_subscription_count(), 'input subscription not discovered'
                cloud = PointCloud2(height=1, width=2, is_dense=True, point_step=32, row_step=64)
                cloud.header.stamp.sec = 100
                cloud.header.frame_id = 'input_lidar'
                intensity_type = PointField.FLOAT32 if converter == 'pandar_to_velodyne' else PointField.UINT8
                cloud.fields = [PointField(name=n, offset=o, datatype=t, count=1) for n,o,t in [
                    ('x',0,PointField.FLOAT32), ('y',4,PointField.FLOAT32), ('z',8,PointField.FLOAT32),
                    ('intensity',12,intensity_type), ('ring',16,PointField.UINT16), ('timestamp',24,PointField.FLOAT64)]]
                data = bytearray(64)
                for i in range(2):
                    struct.pack_into('<fff',data,i*32,1.+i,2.,3.)
                    struct.pack_into('<f' if intensity_type == PointField.FLOAT32 else '<B',data,i*32+12,42)
                    struct.pack_into('<H',data,i*32+16,3+i)
                    struct.pack_into('<d',data,i*32+24,100.+0.01*i)
                cloud.data = bytes(data)
                deadline = time.monotonic() + 5
                while time.monotonic() < deadline and not output:
                    publisher.publish(cloud)
                    rclpy.spin_once(node, timeout_sec=0.1)
                assert output, f'{converter}: no output'
                result = output[-1]
                assert result.width*result.height == 2
                assert result.header.frame_id == 'velodyne'
                assert result.header.stamp.sec == 100
                offsets = {field.name:field.offset for field in result.fields}
                assert set(['x','y','z','intensity','ring','time']) <= offsets.keys()
                for i in range(2):
                    base = i*result.point_step
                    assert math.isclose(struct.unpack_from('<f',result.data,base+offsets['x'])[0],1.+i)
                    assert struct.unpack_from('<H',result.data,base+offsets['ring'])[0] == 3+i
                    assert math.isclose(struct.unpack_from('<f',result.data,base+offsets['time'])[0],0.01*i,abs_tol=1e-6)
                print(converter + ': PASS',flush=True)
            finally:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                node.destroy_subscription(subscriber)
                node.destroy_publisher(publisher)
            assert process.returncode == 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
