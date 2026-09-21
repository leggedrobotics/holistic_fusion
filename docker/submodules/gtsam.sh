#!/bin/bash

# Exit if a single command fails
set -e

# GTSAM version to install
GTSAM_VERSION=4.3.0

# ----------------------------------------------------------------------------------------------------------------------
# Toolchain requirements of GTSAM 4.3 that Ubuntu 20.04 (ROS Noetic) does not fulfil out of the box:
#   * CMake >= 3.20 (GTSAM's CMake code uses list(REMOVE_ITEM ...) with possibly empty lists; 20.04 ships 3.16)
#   * Eigen >= 3.4 (GTSAM's headers use Eigen 3.4 initializer-list constructors; 20.04 ships 3.3.7)
#   * A compiler newer than gcc 9 (gcc 9 does not compile gtsam/constrained/QpCost.h; 20.04 ships gcc 9.4, gcc 10 is available)
#   * oneTBB (2021+) if TBB is enabled; 20.04 ships TBB 2020, so TBB is disabled there.
# Ubuntu 22.04 (ROS 2 Humble) ships CMake 3.22, Eigen 3.4.0, gcc 11 and oneTBB 2021 and needs none of this.
# ----------------------------------------------------------------------------------------------------------------------
GTSAM_EXTRA_CMAKE_ARGS=""
. /etc/os-release
if [ "${VERSION_ID}" = "20.04" ]; then
  echo "Ubuntu 20.04 detected: installing newer CMake, Eigen 3.4.0 and gcc-10 for GTSAM ${GTSAM_VERSION}."

  # CMake (pinned to < 4, as CMake 4 refuses packages with cmake_minimum_required < 3.5, which is common in ROS 1)
  pip3 install "cmake>=3.20,<4"
  hash -r
  cmake --version

  # Eigen 3.4.0 headers into /usr/local (found before the system Eigen 3.3.7 in /usr by CMake)
  mkdir -p /software && cd /software \
   && curl -sL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz -o eigen-3.4.0.tar.gz \
   && tar xzf eigen-3.4.0.tar.gz && rm eigen-3.4.0.tar.gz \
   && mkdir -p eigen-3.4.0/build && cd eigen-3.4.0/build \
   && cmake -DCMAKE_INSTALL_PREFIX=/usr/local .. > /dev/null \
   && make install > /dev/null \
   && cd /software && rm -rf eigen-3.4.0

  # gcc-10 as default compiler (also used for the ROS workspace afterwards)
  apt-get update && apt-get install -y gcc-10 g++-10 && rm -rf /var/lib/apt/lists/*
  update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 100 --slave /usr/bin/g++ g++ /usr/bin/g++-10 --slave /usr/bin/gcov gcov /usr/bin/gcov-10
  update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 90 --slave /usr/bin/g++ g++ /usr/bin/g++-9 --slave /usr/bin/gcov gcov /usr/bin/gcov-9
  update-alternatives --set gcc /usr/bin/gcc-10
  g++ --version

  # No oneTBB on 20.04
  GTSAM_EXTRA_CMAKE_ARGS="-DGTSAM_WITH_TBB=OFF"
fi

# Install GTSAM
mkdir -p /software \
 && cd /software \
 && git clone https://github.com/borglab/gtsam.git \
 && mkdir -p /software/gtsam/build \
 && cd /software/gtsam/build \
 && git checkout ${GTSAM_VERSION} \
 && cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
    -DGTSAM_POSE3_EXPMAP=ON \
    -DGTSAM_ROT3_EXPMAP=ON \
    -DGTSAM_USE_QUATERNIONS=ON \
    -DGTSAM_USE_SYSTEM_EIGEN=ON \
    ${GTSAM_EXTRA_CMAKE_ARGS} \
    /software/gtsam \
 && make install -j$(nproc)

 # Update library cache
 ldconfig
