/*
Copyright 2024 by Julian Nubert & Timon Mathis, Robotic Systems Lab & Autonomous Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "atn_position3_fuser/Position3Estimator.h"

// GraphMSF ROS
#include "graph_msf_ros/ros/read_ros_params.h"

namespace position3_se {

void Position3Estimator::readParams(const ros::NodeHandle& privateNode) {
  // Set frames ----------------------------
  /// Position Measurment frame (Prism)
  std::string frame = graph_msf::tryGetParam<std::string>("extrinsics/positionMeasFrame", privateNode);
  dynamic_cast<Position3StaticTransforms*>(staticTransformsPtr_.get())->setPositionMeasFrame(frame);

  // Sensor Parameters ----------------------------
  positionRate_ = graph_msf::tryGetParam<double>("sensor_params/positionRate", privateNode);

  /// Noise Parameters ----
  /// Position measurement unary noise
  positionMeasUnaryNoise_ = graph_msf::tryGetParam<double>("noise_params/PositionMeasUnaryNoise", privateNode);
}

}  // namespace position3_se
