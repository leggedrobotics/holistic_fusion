#!/bin/bash

# Add ROS 2 apt repository
apt update && apt install -y \
  software-properties-common \
  curl \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
 && apt update && apt install -y \
  python3-pip \
  python3-rosdep \
  python3-colcon-common-extensions \
  python3-vcstool \
  ros-${ROS}-desktop \
  ros-${ROS}-velodyne-pointcloud \
  ros-${ROS}-joy \
  ros-${ROS}-grid-map \
 && rm -rf /var/lib/apt/lists/* \
 && ([ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || rosdep init) && rosdep update \
 && apt update && apt install -y \
  libblas-dev \
  xutils-dev \
  gfortran \
  libf2c2-dev \
  libgmock-dev \
  libgoogle-glog-dev \
  libboost-all-dev \
  libeigen3-dev \
  libglpk-dev \
  liburdfdom-dev \
  liboctomap-dev \
  libassimp-dev \
  python3-pytest \
  python3-lxml \
  ros-${ROS}-ompl \
  ros-${ROS}-octomap-msgs \
  doxygen-latex \
  usbutils \
 && rm -rf /var/lib/apt/lists/* \
 && sudo ln -sf /usr/include/eigen3 /usr/local/include/eigen3
