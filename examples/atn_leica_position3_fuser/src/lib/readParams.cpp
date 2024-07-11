/*
Copyright 2024 by Julian Nubert & Timon Mathis, Robotic Systems Lab & Autonomous Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "atn_leica_position3_fuser/Position3Estimator.h"

// GraphMSF ROS
#include "graph_msf_ros/ros/read_ros_params.h"

namespace position3_se {

void Position3Estimator::readParams_(const ros::NodeHandle& privateNode) {
  // Variables for parameter fetching
  double dParam;
  int iParam;
  bool bParam;
  std::string sParam;

  // Call super method
  graph_msf::GraphMsfRos::readParams_(privateNode);

  // Set frames ----------------------------
  /// Realsense frame
  std::string frame = graph_msf::tryGetParam<std::string>("extrinsics/realsenseFrame", privateNode);
  dynamic_cast<Position3StaticTransforms*>(staticTransformsPtr_.get())->setRealsenseOdometryFrame(frame);

  /// Body frame
  frame = graph_msf::tryGetParam<std::string>("extrinsics/BodyFrame", privateNode);
  dynamic_cast<Position3StaticTransforms*>(staticTransformsPtr_.get())->setBodyFrame(frame);

  /// Position Measurment frame (Prism)
  frame = graph_msf::tryGetParam<std::string>("extrinsics/positionMeasFrame", privateNode);
  dynamic_cast<Position3StaticTransforms*>(staticTransformsPtr_.get())->setPositionMeasFrame(frame);

  // Sensor Parameters ----------------------------
  positionRate_ = graph_msf::tryGetParam<double>("sensor_params/positionRate", privateNode);

  /// Noise Parameters ----
  /// Position measurement unary noise
  positionMeasUnaryNoise_ = graph_msf::tryGetParam<double>("noise_params/PositionMeasUnaryNoise", privateNode);

  /// Realsense Odometry unary noise
  const auto RealsenseOdomPoseUnaryNoise =
      graph_msf::tryGetParam<std::vector<double>>("noise_params/RealsenseOdomPoseUnaryNoise", privateNode);  // roll,pitch,yaw,x,y,z
  RealsenseOdomPoseUnaryNoise_ << RealsenseOdomPoseUnaryNoise[0], RealsenseOdomPoseUnaryNoise[1], RealsenseOdomPoseUnaryNoise[2],
      RealsenseOdomPoseUnaryNoise[3], RealsenseOdomPoseUnaryNoise[4], RealsenseOdomPoseUnaryNoise[5];
}

}  // namespace position3_se
