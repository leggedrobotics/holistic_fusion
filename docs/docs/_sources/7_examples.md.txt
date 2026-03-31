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

Coming soon.