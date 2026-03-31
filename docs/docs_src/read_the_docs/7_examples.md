# Examples

We provide several examples for using the holistic fusion library in ROS1 and ROS2 workspaces.

## ROS1 Examples

The majority of the development over the past years has been done in ROS1.
We provide several examples for using the holistic fusion library in ROS1 workspaces, which can be found in the [ROS1 examples](https://github.com/leggedrobotics/holistic_fusion/tree/main/ros).

### ANYmal Example

This is the ANYmal example from the holisitic fusion paper.
The corresponding example is called `anymal_estimator_graph`.

### Excavator Example

This is the excavator example from the holisitic fusion paper.
The corresponding example is called `excavator_holistic_graph`.

### SuperMegaBot Example

This is the SuperMegaBot example as used during the ETH Robotics Summer School 2023 and 2024.
The corresponding example is called `smb_estimator_graph`.

### Ground Generation Example

This is an example for generating ground truth trajectories from a Leica Total Station and GNSS measurements.
Note that in this example two non-drifting measurement sources are used, which is not the case in the other examples.
Holistic fusion is used to align the two reference frames, fuse the measurements and generate a ground truth trajectory.
The corresponding example is called `atn_position3_fuser`.

### Other Examples

Some other more simplified examples are:
* `imu_pose3_fuser`: Simple example for fusing IMU and pose3 measurements.
* `pure_imu_integration`: Simple example for integrating IMU measurements without any fusion.

## ROS2 Examples

### SuperMegaBot ROS2 Example

The ROS2 port of the SMB estimator is available in the
[smb_estimator_graph_ros2](https://github.com/leggedrobotics/holistic_fusion/tree/main/ros2/examples/smb_estimator_graph_ros2) package.
It demonstrates fusing IMU, LiDAR odometry, wheel odometry, and optionally VIO for the Super Mega Bot.

#### Fused Measurements

The example supports the following measurement sources (individually toggleable via parameters):

| Measurement | Type | ROS Topic | Factor Type |
|-------------|------|-----------|-------------|
| IMU | Core | `/imu/data_raw` | Preintegrated IMU factor |
| LiDAR Odometry | Absolute (6D) | `/open3d/scan2map_odometry` | Unary Pose3 Absolute |
| Wheel Odometry | Binary (6D) | `/wheel_odometry` | Binary Pose3 Between |
| Wheel Velocities | Local (3D) | `/wheel_velocities` | Unary Velocity3 Local |
| VIO Odometry | Binary (6D) | `/tracking_camera/odom/sample` | Binary Pose3 Between |

#### Running

Launch the estimator with:

```bash
ros2 launch smb_estimator_graph_ros2 smb_estimator_graph.launch.py
```

Topic names can be remapped via launch arguments:

```bash
ros2 launch smb_estimator_graph_ros2 smb_estimator_graph.launch.py \
    imu_topic_name:=/my_imu/data \
    lidar_odometry_topic_name:=/my_lidar/odom
```

Additional launch files are provided for replay and simulation:

- `smb_estimator_graph_replay.launch.py` — for playing back rosbags.
- `smb_estimator_graph_sim.launch.py` — for simulation environments.

#### Configuration

Parameters are split into two layers:

- **Core parameters** (`config/core/`): General graph settings shared across applications.
    - `core_graph_config.yaml` — high-level graph configuration.
    - `core_graph_params.yaml` — IMU noise parameters and graph tuning.
    - `core_extrinsic_params.yaml` — reference frame names.
- **SMB-specific parameters** (`config/smb_specific/`): Sensor noise and extrinsics specific to SMB.
    - `smb_graph_params.yaml` — sensor noise values and feature flags (e.g., `useLioOdometry`, `useWheelOdometryBetween`).
    - `smb_extrinsic_params.yaml` — sensor frame extrinsic calibrations.

#### Differences from the ROS1 Version

The ROS2 SMB example is architecturally equivalent to the ROS1 version but uses:

- `rclcpp::Node` and `ament_cmake` build system.
- Python-based launch files instead of XML.
- Dedicated worker threads for publishing to minimize latency.
- Layered YAML parameter files loaded in the launch description.