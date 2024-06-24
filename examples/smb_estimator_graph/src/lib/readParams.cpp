/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "smb_estimator_graph/SmbEstimator.h"

// GraphMSF ROS
#include "graph_msf_ros/ros/read_ros_params.h"

// Project
#include "smb_estimator_graph/SmbStaticTransforms.h"

namespace smb_se {

void SmbEstimator::readParams_(const ros::NodeHandle& privateNode) {
  // Check
  if (!graphConfigPtr_) {
    throw std::runtime_error("SmbEstimator: graphConfigPtr must be initialized.");
  }

  // Flags
  useLioOdometryFlag_ = graph_msf::tryGetParam<bool>("sensor_params/useLioOdometry", privateNode);
  useWheelOdometryFlag_ = graph_msf::tryGetParam<bool>("sensor_params/useWheelOdometry", privateNode);
  useVioOdometryFlag_ = graph_msf::tryGetParam<bool>("sensor_params/useVioOdometry", privateNode);

  // Sensor Params
  lioOdometryRate_ = graph_msf::tryGetParam<double>("sensor_params/lioOdometryRate", privateNode);
  wheelOdometryRate_ = graph_msf::tryGetParam<double>("sensor_params/wheelOdometryRate", privateNode);
  vioOdometryRate_ = graph_msf::tryGetParam<double>("sensor_params/vioOdometryRate", privateNode);

  // Alignment Parameters
  const auto poseAlignmentNoise =
      graph_msf::tryGetParam<std::vector<double>>("alignment_params/initialSe3AlignmentNoiseDensity", privateNode);
  initialSe3AlignmentNoise_ << poseAlignmentNoise[0], poseAlignmentNoise[1], poseAlignmentNoise[2], poseAlignmentNoise[3],
      poseAlignmentNoise[4], poseAlignmentNoise[5];

  // Noise Parameters
  /// LiDAR Odometry
  const auto poseUnaryNoise =
      graph_msf::tryGetParam<std::vector<double>>("noise_params/lioPoseUnaryNoiseDensity", privateNode);  // roll,pitch,yaw,x,y,z
  lioPoseUnaryNoise_ << poseUnaryNoise[0], poseUnaryNoise[1], poseUnaryNoise[2], poseUnaryNoise[3], poseUnaryNoise[4], poseUnaryNoise[5];
  /// Wheel Odometry
  const auto wheelPoseBetweenNoise =
      graph_msf::tryGetParam<std::vector<double>>("noise_params/wheelPoseBetweenNoiseDensity", privateNode);  // roll,pitch,yaw,x,y,z
  wheelPoseBetweenNoise_ << wheelPoseBetweenNoise[0], wheelPoseBetweenNoise[1], wheelPoseBetweenNoise[2], wheelPoseBetweenNoise[3],
      wheelPoseBetweenNoise[4], wheelPoseBetweenNoise[5];
  /// VIO Odometry
  const auto vioPoseBetweenNoise =
      graph_msf::tryGetParam<std::vector<double>>("noise_params/vioPoseBetweenNoiseDensity", privateNode);  // roll,pitch,yaw,x,y,z
  vioPoseBetweenNoise_ << vioPoseBetweenNoise[0], vioPoseBetweenNoise[1], vioPoseBetweenNoise[2], vioPoseBetweenNoise[3],
      vioPoseBetweenNoise[4], vioPoseBetweenNoise[5];

  // Set frames
  /// LiDAR odometry frame
  dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())
      ->setLioOdometryFrame(graph_msf::tryGetParam<std::string>("extrinsics/lidarOdometryFrame", privateNode));
  /// Wheel Odometry frame
  dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())
      ->setWheelOdometryFrame(graph_msf::tryGetParam<std::string>("extrinsics/wheelOdometryFrame", privateNode));
  /// VIO Odometry frame
  dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())
      ->setVioOdometryFrame(graph_msf::tryGetParam<std::string>("extrinsics/vioOdometryFrame", privateNode));
}

}  // namespace smb_se
