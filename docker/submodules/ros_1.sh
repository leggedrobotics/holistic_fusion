#!/bin/bash

sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
 && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - \
 && apt update && apt install -y \
  python3-pip \
  python3-rosdep \
  python3-rosclean \
  python3-rosparam \
  python3-progressbar \
  python3-catkin-tools \
  python3-osrf-pycommon \
  python3-virtualenvwrapper \
  ros-${ROS}-desktop-full \
  ros-${ROS}-velodyne-pointcloud \
  ros-${ROS}-joy \
  ros-${ROS}-grid-map-core \
  ros-${ROS}-grid-map \
 && rm -rf /var/lib/apt/lists/* \
 && rosdep init && rosdep update \
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
  python3-catkin-tools \
  python3-pytest \
  python3-lxml \
  ros-${ROS}-ompl \
  ros-${ROS}-octomap-msgs \
  ros-${ROS}-pybind11-catkin \
  doxygen-latex \
  usbutils \
  python3-vcstool \
 && rm -rf /var/lib/apt/lists/* \
 && sudo ln -s /usr/include/eigen3 /usr/local/include/