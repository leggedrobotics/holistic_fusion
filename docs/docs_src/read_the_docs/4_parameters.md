# Parameters

The parameters of the `graph_msf` library are split in two parts: the parameters of the core library and the parameters of the ROS/ROS2 wrapper packages / downstream applications.

All parameters are provided as YAML files. An example for a ROS1 application is provided in each of the example packages in the [ROS1 examples](https://github.com/leggedrobotics/holistic_fusion/tree/main/ros).

## Core Library Parameters

The core libary parameters are given in the following 3 yaml file-types:
- `core_extrinsic_params.yaml`: This file contains the names of the reference frames.
- `core_graph_config.yaml`: This contains some high-level configurations.
- `core_graph_params.yaml`: This file contains the parameters for the graph such as the main tuning parameters that are ALWAYS needed for any holistiic fusion application.

**Note: `core_graph_params.yaml` only contains the noise parameters of the central IMU, as these are always needed for any holistic fusion application. The noise parameters of the other sensors are provided in the sensor-specific parameter files.**

### Core Extrinsic Parameters
Coming soon.

### Core Graph Config
Coming soon.

### Core Graph Parameters
Coming soon. The IMU noise parameters follow the GTSAM conventions (continuous-time noise amplitude spectral densities).

#### Earth rotation compensation

GTSAM (>= 4.3) can model the rotation of the navigation frame in the IMU preintegration exactly (Coriolis and centrifugal
accelerations, Earth-rate compensation of the gyroscope integration). This requires the angular velocity of the world frame
w.r.t. the inertial frame, expressed in the world frame. Holistic Fusion computes it from three parameters in `noise_params`:

| Parameter | Meaning |
|---|---|
| `earthRotationCompensation` | `true` enables the rotating-frame model. `false` uses the plain inertial model. |
| `latitudeDeg` | Geodetic latitude in degrees, positive on the northern hemisphere (Zurich: 47.4). |
| `worldFrameNorthAligned` | `true` only if the y-axis of the (gravity-aligned) world frame points north, i.e. the world frame is ENU. Then the horizontal Earth-rate component Ω·cos(lat) is used in addition to the vertical component Ω·sin(lat). With an arbitrary yaw of the world frame (the usual case, e.g. LiDAR-initialized), only the vertical component is used, which is exact for the Coriolis effect on horizontal motion. |

With Ω = 7.2921159e-5 rad/s the resulting vector is `[0, Ω·cos(lat) (if north aligned), Ω·sin(lat)]`, e.g. `[0, 4.93e-5, 5.37e-5]` rad/s
for an ENU world frame in Zurich. The effect is small for slow robots (Coriolis acceleration 2·Ω·v ≈ 1.5e-4 m/s² at 1 m/s) but
the Earth rate of 15 deg/h is not negligible for good gyroscopes and long runs.

## Applications Specific Parameters

Application specific parameters are provided separately.
In theory this can fully be done by the user, but we provide some examples for ROS1 and ROS2 applications in the respective example packages.

An example for the anymal robot can be found in the [ROS1 examples](https://github.com/leggedrobotics/holistic_fusion/tree/main/ros).
In the `anymal_specific` directory all the extrinsic parameters and noise parameters specific to the ANYmal example are provided.