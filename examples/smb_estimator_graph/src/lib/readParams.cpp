/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "smb_estimator_graph/SmbEstimator.h"

// Project
#include "smb_estimator_graph/SmbStaticTransforms.h"

namespace smb_se {

void SmbEstimator::readParams_(const ros::NodeHandle& privateNode) {
  // Check
  if (!graphConfigPtr_) {
    throw std::runtime_error("SmbEstimator: graphConfigPtr must be initialized.");
  }

  // Flags
  useLidarUnaryFactorFlag_ = graph_msf::tryGetParam<bool>("sensor_params/useLidarUnaryFactor", privateNode);
  useWheelOdometryFlag_ = graph_msf::tryGetParam<bool>("sensor_params/useWheelOdometry", privateNode);
  useVioOdometryFlag_ = graph_msf::tryGetParam<bool>("sensor_params/useVioOdometry", privateNode);

  // Sensor Params
  lidarOdometryRate_ = graph_msf::tryGetParam<double>("sensor_params/lidarOdometryRate", privateNode);
  wheelOdometryRate_ = graph_msf::tryGetParam<double>("sensor_params/wheelOdometryRate", privateNode);
  vioOdometryRate_ = graph_msf::tryGetParam<double>("sensor_params/vioOdometryRate", privateNode);

  // Noise Parameters
  /// LiDAR Odometry
  const auto poseBetweenNoise =
      graph_msf::tryGetParam<std::vector<double>>("noise_params/lidarPoseBetweenNoise", privateNode);  // roll,pitch,yaw,x,y,z
  lidarPoseBetweenNoise_ << poseBetweenNoise[0], poseBetweenNoise[1], poseBetweenNoise[2], poseBetweenNoise[3], poseBetweenNoise[4],
      poseBetweenNoise[5];
  const auto poseUnaryNoise =
      graph_msf::tryGetParam<std::vector<double>>("noise_params/lidarPoseUnaryNoise", privateNode);  // roll,pitch,yaw,x,y,z
  lidarPoseUnaryNoise_ << poseUnaryNoise[0], poseUnaryNoise[1], poseUnaryNoise[2], poseUnaryNoise[3], poseUnaryNoise[4], poseUnaryNoise[5];
  /// Wheel Odometry
  const auto wheelPoseBetweenNoise =
      graph_msf::tryGetParam<std::vector<double>>("noise_params/wheelPoseBetweenNoise", privateNode);  // roll,pitch,yaw,x,y,z
  wheelPoseBetweenNoise_ << wheelPoseBetweenNoise[0], wheelPoseBetweenNoise[1], wheelPoseBetweenNoise[2], wheelPoseBetweenNoise[3],
      wheelPoseBetweenNoise[4], wheelPoseBetweenNoise[5];
  /// VIO Odometry
  const auto vioPoseBetweenNoise =
      graph_msf::tryGetParam<std::vector<double>>("noise_params/vioPoseBetweenNoise", privateNode);  // roll,pitch,yaw,x,y,z
  vioPoseBetweenNoise_ << vioPoseBetweenNoise[0], vioPoseBetweenNoise[1], vioPoseBetweenNoise[2], vioPoseBetweenNoise[3],
      vioPoseBetweenNoise[4], vioPoseBetweenNoise[5];

  // Set frames
  /// LiDAR odometry frame
  dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())
      ->setLidarOdometryFrame(graph_msf::tryGetParam<std::string>("extrinsics/lidarOdometryFrame", privateNode));
  /// Wheel Odometry frame
  dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())
      ->setWheelOdometryFrame(graph_msf::tryGetParam<std::string>("extrinsics/wheelOdometryFrame", privateNode));
  /// VIO Odometry frame
  dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())
      ->setVioOdometryFrame(graph_msf::tryGetParam<std::string>("extrinsics/vioOdometryFrame", privateNode));
}

}  // namespace smb_se
