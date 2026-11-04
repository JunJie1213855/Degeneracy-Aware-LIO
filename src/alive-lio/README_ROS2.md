# ALIVE-LIO on ROS 2 Humble

This checkout is a native ROS 2 / ament port. The voxel map, iterated ESKF,
degeneracy detection and learned inertial velocity fusion remain in C++.
No ROS 1 installation or ros1_bridge is needed.

## Build

Use Ubuntu 22.04 with ROS 2 Humble. Required dependencies are ament_cmake,
rclcpp, ament_index_cpp, sensor_msgs, nav_msgs, geometry_msgs, std_msgs,
visualization_msgs, builtin_interfaces, tf2_ros, pcl_conversions, PCL, Eigen,
OpenMP, modern header-only Sophus, and ONNX Runtime's C/C++ distribution.
Install RViz2 for visualization. Sophus must provide Sophus::Sophus and
sophus/so3.hpp; the old libSophus.so alone is insufficient.

The tested CPU runtime is [ONNX Runtime 1.18.0](https://github.com/microsoft/onnxruntime/releases/tag/v1.18.0).
Extract onnxruntime-linux-x64-1.18.0.tgz to ~/.local/opt.
Its SHA-256 is fa4d11b3fa1b2bf1c3b2efa8f958634bc34edc95e351ac2a0408c6ad5c5504f0.

```bash
source /opt/ros/humble/setup.bash
# Optional: source the Livox driver overlay before building for CustomMsg support.
source /home/ros/rosws/livox_ros_ws/install/setup.bash
cd /home/ros/rosws/bievr_lio_ws
MAKEFLAGS=-j3 colcon build --packages-select alive_lio --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DONNXRUNTIME_ROOT=/home/ros/.local/opt/onnxruntime-linux-x64-1.18.0
source install/setup.bash
```

ONNXRUNTIME_ROOT also accepts an environment variable. The runtime library path
is recorded in the installed executable's RPATH; keep the distribution there.
All four pretrained models are installed in the package share directory.

livox_ros_driver2 is optional at build time. Standard PointCloud2 inputs work
without it; selecting AVIA CustomMsg then reports a clear startup error.

## Run

On the tested WSL host, Fast DDS discovered peers but did not deliver messages
between processes. Cyclone DDS passed the replay and converter tests. It is
already installed on this host; in every estimator, driver and bag terminal use:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

On another Humble host, install ros-humble-rmw-cyclonedds-cpp if needed. No global
shell settings or other projects are modified by this port.

```bash
source /home/ros/rosws/bievr_lio_ws/install/setup.bash
ros2 launch alive_lio mapping.launch.py config:=geode/velodyne_alive.yaml rviz:=true
```

config accepts a path relative to the installed config directory or an absolute
file path. All four original launch names are available with a .py suffix,
for example mapping_avia_geode_alive.launch.py.

For ROS 2 bags, start the estimator and replay in another sourced terminal:

```bash
ros2 launch alive_lio mapping.launch.py \
  config:=/absolute/path/to/sensor.yaml use_sim_time:=true rviz:=true
# Other terminal:
ros2 bag play /path/to/rosbag2 --clock
```

ROS 1 .bag recordings need conversion before ROS 2 playback.
Configurations use the ROS 2 wildcard node key and ros__parameters. Parameter
names are dotted, e.g. common.lid_topic and localizability.en_network.
Direct execution and overrides are supported:

```bash
ros2 run alive_lio alive_lio_node --ros-args \
  --params-file /absolute/path/to/sensor.yaml \
  -p common.lid_topic:=/velodyne_points_a -p common.imu_topic:=/imu_raw_a
```

Verify topics, LiDAR type, per-point time/ring fields, calibration and IMU rate.
Extrinsic translation has three doubles and rotation has nine row-major doubles.
Keep the sensor still during initialization. GEODE's Velodyne config uses the
100 Hz vehicle model. For a 200 Hz IMU select model_vehicle200 and buffer_len:
1000 (5 seconds). The handheld model uses 200 Hz / 1000 samples; the 400 Hz
vehicle model uses 2000 samples. Rate and platform matching matter for accuracy.

All configurations default to CPU inference. For CUDA, build against a GPU
ONNX Runtime distribution with its required CUDA/cuDNN dependencies, then set
localizability.use_gpu to true. Unavailable CUDA providers produce a startup
error. Disabling localizability.en_network skips model loading entirely.

## Interfaces

Input subscriptions use SensorDataQoS: best effort, depth 1000. Both reliable
and best-effort sensor publishers are supported. Outputs are reliable, depth 10.

- /Odometry: nav_msgs/msg/Odometry, parent camera_init, child body.
- /cloud_registered and /cloud_registered_body: sensor_msgs/msg/PointCloud2.
- /path: nav_msgs/msg/Path.
- /planes: visualization_msgs/msg/MarkerArray, when enabled.
- /ConditionNumber, /NetworkOuput and other diagnostics:
  std_msgs/msg/Float64MultiArray. Original topic spelling is preserved.
- TF: world -> camera_init -> body.

Debug output defaults to /tmp/alive_lio/Log. Override output_directory to keep
it elsewhere. Algorithm parameters are read at startup; restart to apply changes.
Both converter executables retain their positional arguments and support ROS 2
topic remapping:

```bash
ros2 run alive_lio pandar_to_velodyne XYZIRT XYZIRT
ros2 run alive_lio rs_to_velodyne XYZIRT XYZIRT true
```

## Runtime verification

The integration test opens SQLite recordings read-only, starts the installed
node, and checks finite odometry, increasing timestamps, clouds, paths, TF,
neural output and clean SIGINT shutdown. It uses ROS domain 73 by default.

```bash
python3 src/alive-lio/tests/replay_smoke.py /path/to/recording.db3 \
  --duration 12 --imu-hz 200
python3 src/alive-lio/tests/replay_smoke.py /path/to/recording.db3 \
  --duration 4 --no-network --best-effort
```

These checks verify execution and transport, not trajectory accuracy. Evaluate
drift and degeneracy robustness with calibrated data and ground truth.

Validated on Ubuntu 22.04 / ROS 2 Humble in WSL with Cyclone DDS and CPU inference:

| Check | Result |
| --- | --- |
| Release build, main node and both converters | Passed |
| Four dataset launch files / all four pretrained models | Startup and shutdown passed |
| 12-second Velodyne / 200 Hz IMU replay | 112 odometry/cloud/path outputs, 224 TF transforms, 63 neural outputs |
| 4-second replay, network disabled, best-effort input | 32 odometry/cloud/path outputs, 64 TF transforms |
| Pandar and RoboSense converters | Field values, timestamps and ROS 2 remapping passed |

The recorded fragment was read from big_loop.db3 without modifying it. The
vehicle200 model and a 1000-sample window were selected for its 200 Hz IMU.
GPU inference and a full-sequence accuracy evaluation have not been tested.
