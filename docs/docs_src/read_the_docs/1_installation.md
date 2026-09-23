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

We install GTSAM (version 4.3.0) from source. GTSAM 4.3 requires C++17, CMake >= 3.20, Eigen >= 3.4 and a compiler newer than gcc 9.
Ubuntu 22.04 (ROS2 Humble) fulfils all of this out of the box. On Ubuntu 20.04 (ROS Noetic) you additionally need to

* install a newer CMake, e.g. `pip3 install "cmake>=3.20,<4"`,
* install the Eigen 3.4.0 headers into `/usr/local` (found before the system Eigen 3.3.7),
* use gcc-10 (`apt install gcc-10 g++-10` and select it via `update-alternatives`), and
* build GTSAM with `-DGTSAM_WITH_TBB=OFF` (20.04 ships TBB 2020, GTSAM 4.3 needs oneTBB).

See [`docker/submodules/gtsam.sh`](https://github.com/leggedrobotics/holistic_fusion/blob/main/docker/submodules/gtsam.sh) for the exact commands used in our Docker images.
You can also install it locally by adding the `-DCMAKE_INSTALL_PREFIX` option to the CMake command as done in the following.

* Get the source code and compile it:

```bash
 git clone https://github.com/borglab/gtsam.git \
    && mkdir -p ./gtsam/build \
    && cd ./gtsam/build \
    && git checkout 4.3.0 \
    && cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
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

2. Cloning the repository and (vcs-managed) third-party dependencies

```bash
cd src
git clone https://github.com/leggedrobotics/holistic_fusion.git
wget -qO - https://raw.githubusercontent.com/leggedrobotics/holistic_fusion/refs/heads/main/catkin_workspace.vcs | vcs import .
cd ..
```

3. Compiling the workspace:

```bash
catkin build graph_msf_ros_examples
```

This should build 14 packages, including all ROS1 examples, the core library, the `graph_msf_catkin` package, the `graph_msf_ros` package (commodity package for the ROS examples), and the `graph_msf_ros_examples` package.

## Colcon Workspace

To use the `graph_msf` library in a ROS2 workspace, we provide a package called `graph_msf_ros2`.
To install all dependencies and compile the library, you can follow these steps:

1. Setting up the workspace:

```bash
mkdir -p ros2_ws/src
cd ros2_ws
```

2. Cloning the repository and (vcs-managed) third-party dependencies

```bash
cd src
git clone https://github.com/leggedrobotics/holistic_fusion.git
wget -qO - https://raw.githubusercontent.com/leggedrobotics/holistic_fusion/refs/heads/main/colcon_workspace.vcs | vcs import .
cd ..
```

3. Compiling the workspace:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release --packages-up-to smb_estimator_graph_ros2
```

This should build the core library, the `graph_msf_ros2` package, the `graph_msf_ros2_msgs` package, and the `smb_estimator_graph_ros2` example package.
