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
Coming soon.

## Applications Specific Parameters

Application specific parameters are provided separately.
In theory this can fully be done by the user, but we provide some examples for ROS1 and ROS2 applications in the respective example packages.

An example for the anymal robot can be found in the [ROS1 examples](https://github.com/leggedrobotics/holistic_fusion/tree/main/ros).
In the `anymal_specific` directory all the extrinsic parameters and noise parameters specific to the ANYmal example are provided.