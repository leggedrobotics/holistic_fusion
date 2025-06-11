# Installation

## Requirements

### Docker

We recommend using Docker to run the code. This way you can avoid dependency issues and have a clean environment.
The instructions for building and running the Docker image can be found in the [Docker section](6_docker.md).

### Custom Build

**Note: This is only needed if you are not using Docker.**

The two main dependencies of the `graph_msf` library are Eigen3 and GTSAM.
For running the ROS and ROS2 examples, you also need to have ROS Noetic and/or ROS2 Humble installed.

#### Graph MSF Core

##### Eigen3

Make sure you have the Eigen3-headers in your include path. This is usually automatically the case if you have ROS installed.

##### GTSAM

We install GTSAM from source.
You can also install it locally by adding the `-DCMAKE_INSTALL_PREFIX` option to the CMake command as done in the following.

* Get the source code and compile it:

```bash
 git clone https://github.com/borglab/gtsam.git \
    && mkdir -p ./gtsam/build \
    && cd ./gtsam/build \
    && git checkout 4.2 \
    && cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
        -DGTSAM_WITH_TBB=OFF \
        -DGTSAM_POSE3_EXPMAP=ON \
        -DGTSAM_ROT3_EXPMAP=ON \
        -DGTSAM_USE_QUATERNIONS=ON \
        -DGTSAM_USE_SYSTEM_EIGEN=ON \
        -DCMAKE_INSTALL_PREFIX=$HOME/.local \
        .. \
    && make install -j$(nproc)
 ```

* Environment variables (e.g. add to your .bashrc-file):

```bash
export CMAKE_PREFIX_PATH=$HOME/.local/:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=$HOME/.local/lib/:$LD_LIBRARY_PATH
export LIBRARY_PATH=${LIBRARY_PATH}:${LD_LIBRARY_PATH}
```

This is usually only needed if you you choose a non-standard (local) install directory (as specified with `-DCMAKE_INSTALL_PREFIX=$HOME/.local \` before).

## Core Library

The core library can be compile without ROS or ROS2. It is a pure C++ library that can be used in any C++ project with CMake support.
When using the library with ROS or ROS2, we provide packages called `graph_msf_catkin` and `graph_msf_ament` that are wrapper projects exposing the functionalities to the workspace, which can be used to compile the library with the respective build system.

To build the core library (standalone) you can simply do this using cmake:

**Note: This is only needed if you are not building it with catkin or ament/colcon.**

```bash
git clone https://github.com/leggedrobotics/holistic_fusion.git
cd holistic_fusion/graph_msf
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

## Catkin Workspace

To use the `graph_msf` library in a ROS workspace, we provide a package called `graph_msf_catkin`.
To install all dependencies and compile the library, you can follow these steps:

1. Setting up the workspace:

```bash
mkdir catkin_ws
mkdir src
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
```

2. Cloning the requirements

```bash
wget -qO - https://raw.githubusercontent.com/leggedrobotics/holistic_fusion/refs/heads/main/catkin_workspace.vcs | vcs import src
```

3. Compiling the workspace:

```bash
catkin build graph_msf_ros_examples
```

This should build 14 packages, including all ROS1 examples, the core library, the `graph_msf_catkin` package, the `graph_msf_ros` package (commodity package for the ROS examples), and the `graph_msf_ros_examples` package.

## Colcon Workspace

Still under construction.
