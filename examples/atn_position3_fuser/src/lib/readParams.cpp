/*
Copyright 2024 by Julian Nubert & Timon Mathis, Robotic Systems Lab & Autonomous Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "atn_position3_fuser/Position3Estimator.h"

// Workspace
#include "atn_position3_fuser/Position3StaticTransforms.h"
#include "graph_msf_ros/ros/read_ros_params.h"

namespace position3_se {

void Position3Estimator::readParams(const ros::NodeHandle& privateNode) {
  // Set frames ----------------------------
  /// Position Measurment frame (Prism)
  std::string frame = graph_msf::tryGetParam<std::string>("extrinsics/prismPositionMeasFrame", privateNode);
  dynamic_cast<Position3StaticTransforms*>(staticTransformsPtr_.get())->setPrismPositionMeasFrame(frame);

  /// Position Measurment frame (GNSS)
  frame = graph_msf::tryGetParam<std::string>("extrinsics/gnssPositionMeasFrame", privateNode);
  dynamic_cast<Position3StaticTransforms*>(staticTransformsPtr_.get())->setGnssPositionMeasFrame(frame);

  /// Position Measurment frame (GNSS Offline Pose)
  frame = graph_msf::tryGetParam<std::string>("extrinsics/gnssOfflinePoseMeasFrame", privateNode);
  dynamic_cast<Position3StaticTransforms*>(staticTransformsPtr_.get())->setGnssOfflinePoseMeasFrame(frame);

  // Sensor Parameters ----------------------------
  prismPositionRate_ = graph_msf::tryGetParam<double>("sensor_params/prismPositionRate", privateNode);
  gnssPositionRate_ = graph_msf::tryGetParam<double>("sensor_params/gnssPositionRate", privateNode);
  gnssOfflinePoseRate_ = graph_msf::tryGetParam<double>("sensor_params/gnssOfflinePoseRate", privateNode);

  // Alignment Parameters ----------------------------
  // Initial SE3 Alignment Noise
  const auto poseAlignmentNoise =
      graph_msf::tryGetParam<std::vector<double>>("alignment_params/initialSe3AlignmentNoiseDensity", privateNode);
  initialSe3AlignmentNoise_ << poseAlignmentNoise[0], poseAlignmentNoise[1], poseAlignmentNoise[2], poseAlignmentNoise[3],
      poseAlignmentNoise[4], poseAlignmentNoise[5];
  // Random Walk
  auto gnssSe3AlignmentRandomWalk = graph_msf::tryGetParam<std::vector<double>>("alignment_params/gnssSe3AlignmentRandomWalk", privateNode);
  gnssSe3AlignmentRandomWalk_ << gnssSe3AlignmentRandomWalk[0], gnssSe3AlignmentRandomWalk[1], gnssSe3AlignmentRandomWalk[2],
      gnssSe3AlignmentRandomWalk[3], gnssSe3AlignmentRandomWalk[4], gnssSe3AlignmentRandomWalk[5];

  /// Noise Parameters ----
  /// Position measurement unary noise
  prismPositionMeasUnaryNoise_ = graph_msf::tryGetParam<double>("noise_params/prismPositionMeasUnaryNoise", privateNode);
  gnssPositionMeasUnaryNoise_ = graph_msf::tryGetParam<double>("noise_params/gnssPositionMeasUnaryNoise", privateNode);
  const auto gnssOfflinePoseMeasUnaryNoise =
      graph_msf::tryGetParam<std::vector<double>>("noise_params/gnssOfflinePoseMeasUnaryNoise", privateNode);
  gnssOfflinePoseMeasUnaryNoise_ << gnssOfflinePoseMeasUnaryNoise[0], gnssOfflinePoseMeasUnaryNoise[1], gnssOfflinePoseMeasUnaryNoise[2],
      gnssOfflinePoseMeasUnaryNoise[3], gnssOfflinePoseMeasUnaryNoise[4], gnssOfflinePoseMeasUnaryNoise[5];
}

}  // namespace position3_se
