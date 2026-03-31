# System Overview

Holistic Fusion (HF) is structured in a layered architecture that separates the core sensor fusion logic from the
communication layer (ROS1 / ROS2). This allows the core library to be used independently of any middleware.

## Architecture

The system consists of three main layers:

1. **Core Library (`graph_msf`)**: Pure C++ library depending on Eigen and GTSAM. Contains the factor graph, state
   management, and all measurement handling logic.
2. **Communication Wrappers (`graph_msf_ros` / `graph_msf_ros2`)**: ROS1 and ROS2 wrapper classes that handle
   subscribers, publishers, TF broadcasting, and parameter loading.
3. **Application Examples**: Robot-specific implementations (e.g., ANYmal, SMB, Excavator) that inherit from the
   respective communication wrapper.

## Core Library

### Interface Layer

The primary entry point is the `GraphMsf` base class, which provides the public API for:

- **IMU integration**: `addCoreImuMeasurementAndGetState()` — adds an IMU measurement, triggers preintegration, and
  returns the current state estimate at IMU rate.
- **Graph initialization**: `initYawAndPositionInWorld()`, `initYawAndPosition()` — bootstrap the graph with an initial
  pose from an absolute sensor.
- **Offline optimization**: `optimizeSlowBatchSmoother()` — trigger post-mission batch optimization.
- **State logging**: `logRealTimeNavStates()` — log estimated trajectories for evaluation.

Two concrete flavors are provided:

- **`GraphMsfClassic`**: Traditional factor graph approach.
- **`GraphMsfHolistic`**: The holistic approach using GTSAM expression factors, enabling setup-agnostic fusion with
  automatic reference frame alignment.

### Measurement Types

HF supports three general categories of measurements, each with dedicated classes:

| Category | Class | Description | Example Sensors |
|----------|-------|-------------|-----------------|
| **Absolute** | `UnaryMeasurementXDAbsolute` | Non-drifting measurements in a fixed reference frame | GNSS, absolute LiDAR localization |
| **Landmark** | `UnaryMeasurementXDLandmark` | Measurements to landmarks without systematic drift | Fiducial markers, beacon-based localization |
| **Binary (Relative)** | `BinaryMeasurementXD` | Relative constraints between consecutive states | Wheel odometry, scan-to-scan matching, VIO |

Each category supports different dimensionalities:

- **Pose3 (6D)**: Full SE(3) pose — `addUnaryPose3AbsoluteMeasurement()`, `addBinaryPose3Measurement()`
- **Position3 (3D)**: R3 position — `addUnaryPosition3AbsoluteMeasurement()`, `addUnaryPosition3LandmarkMeasurement()`
- **Velocity3 (3D)**: R3 velocity — `addUnaryVelocity3AbsoluteMeasurement()`, `addUnaryVelocity3LocalMeasurement()`
- **Orientation (1D)**: Individual roll, pitch, or yaw — `addUnaryRollAbsoluteMeasurement()`, etc.
- **Bearing3 (3D)**: Bearing to landmarks — `addUnaryBearing3LandmarkMeasurement()`

Additional convenience factors are available:

- **Zero motion**: `addZeroMotionFactor()` — constrains consecutive states to be identical.
- **Zero velocity**: `addZeroVelocityFactor()` — constrains the velocity to zero.

### Graph Manager

The `GraphManager` is the backend engine responsible for:

- Managing the GTSAM factor graph and variable index.
- Running incremental fixed-lag smoothing in real time (iSAM2-based).
- Supporting offline batch optimization for post-mission trajectory refinement.
- Tracking dynamic reference frame transformations (random walk modeling for coordinate frame alignment).

### Static Transforms

Extrinsic calibrations between sensor frames are managed via the `StaticTransforms` class. Each application provides
its own subclass (e.g., `SmbStaticTransforms`) that reads the relevant transforms from TF or configuration files.

## Communication Wrappers

### ROS1 (`graph_msf_ros`)

- Inherits from `GraphMsfHolistic` (or `GraphMsfClassic`).
- Provides standard ROS1 subscribers (IMU) and publishers (Odometry, Path, TF).
- Parameters are loaded from YAML files via the ROS parameter server.

### ROS2 (`graph_msf_ros2`)

- Inherits from `GraphMsfHolistic` (or `GraphMsfClassic`) and `rclcpp::Node`.
- Uses dedicated worker threads (`nonTimeCriticalThread`, `imuOdomThread`) for state publishing to minimize latency.
- Parameters are loaded from layered YAML configuration files.
- Uses `tf2_ros` for transform broadcasting.

## Data Flow

The typical data flow in an HF application is:

1. **IMU callback** receives IMU data at high rate (~200-400 Hz), triggers preintegration, and publishes the
   preintegrated state estimate.
2. **Sensor callbacks** (LiDAR, GNSS, wheel odometry, etc.) add unary or binary factors to the graph at their
   respective rates.
3. The **GraphManager** periodically optimizes the factor graph (fixed-lag smoothing) and provides updated state
   estimates.
4. The **communication wrapper** publishes odometry messages, paths, and TF transforms.