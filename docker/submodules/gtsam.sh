#!/bin/bash

mkdir -p /software \
 && cd /software \
 && git clone https://github.com/borglab/gtsam.git \
 && mkdir -p /software/gtsam/build \
 && cd /software/gtsam/build \
 && git checkout 4.2 \
 && cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
    -DGTSAM_POSE3_EXPMAP=ON \
    -DGTSAM_ROT3_EXPMAP=ON \
    -DGTSAM_USE_QUATERNIONS=ON \
    -DGTSAM_USE_SYSTEM_EIGEN=ON \
    /software/gtsam \
 && make install -j$(nproc)