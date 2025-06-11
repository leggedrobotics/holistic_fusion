/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "smb_estimator_graph_ros2/SmbEstimator.h"

// Project
#include "smb_estimator_graph_ros2/SmbStaticTransforms.h"
#include "smb_estimator_graph_ros2/constants.h"

// GraphMSF ROS2
#include "graph_msf_ros2/ros/read_ros_params.h"

namespace smb_se {

void SmbEstimator::readParams() {
  // Check
  if (!graphConfigPtr_) {
    throw std::runtime_error("SmbEstimator: graphConfigPtr must be initialized.");
  }

  // Flags
  useLioOdometryFlag_ = graph_msf::tryGetParam<bool>(node_, "sensor_params.useLioOdometry");
  useWheelOdometryBetweenFlag_ = graph_msf::tryGetParam<bool>(node_, "sensor_params.useWheelOdometryBetween");
  useWheelLinearVelocitiesFlag_ = graph_msf::tryGetParam<bool>(node_, "sensor_params.useWheelLinearVelocities");
  useVioOdometryFlag_ = graph_msf::tryGetParam<bool>(node_, "sensor_params.useVioOdometry");

  // Sensor Params
  lioOdometryRate_ = graph_msf::tryGetParam<int>(node_, "sensor_params.lioOdometryRate");
  wheelOdometryBetweenRate_ = graph_msf::tryGetParam<int>(node_, "sensor_params.wheelOdometryBetweenRate");
  wheelLinearVelocitiesRate_ = graph_msf::tryGetParam<int>(node_, "sensor_params.wheelLinearVelocitiesRate");
  vioOdometryRate_ = graph_msf::tryGetParam<int>(node_, "sensor_params.vioOdometryRate");

  // Alignment Parameters
  const auto initialSe3AlignmentNoiseDensity =
      graph_msf::tryGetParam<std::vector<double>>(node_, "alignment_params.initialSe3AlignmentNoiseDensity");
  initialSe3AlignmentNoise_ << initialSe3AlignmentNoiseDensity[0], initialSe3AlignmentNoiseDensity[1], initialSe3AlignmentNoiseDensity[2],
      initialSe3AlignmentNoiseDensity[3], initialSe3AlignmentNoiseDensity[4], initialSe3AlignmentNoiseDensity[5];
  const auto lioSe3AlignmentRandomWalk =
      graph_msf::tryGetParam<std::vector<double>>(node_, "alignment_params.lioSe3AlignmentRandomWalk");
  lioSe3AlignmentRandomWalk_ << lioSe3AlignmentRandomWalk[0], lioSe3AlignmentRandomWalk[1], lioSe3AlignmentRandomWalk[2],
      lioSe3AlignmentRandomWalk[3], lioSe3AlignmentRandomWalk[4], lioSe3AlignmentRandomWalk[5];

  // Noise Parameters
  /// LiDAR Odometry
  const auto poseUnaryNoise =
      graph_msf::tryGetParam<std::vector<double>>(node_, "noise_params.lioPoseUnaryNoiseDensity");  // roll,pitch,yaw,x,y,z
  lioPoseUnaryNoise_ << poseUnaryNoise[0], poseUnaryNoise[1], poseUnaryNoise[2], poseUnaryNoise[3], poseUnaryNoise[4], poseUnaryNoise[5];
  /// Wheel Odometry
  /// Between
  const auto wheelPoseBetweenNoise =
      graph_msf::tryGetParam<std::vector<double>>(node_, "noise_params.wheelPoseBetweenNoiseDensity");  // roll,pitch,yaw,x,y,z
  wheelPoseBetweenNoise_ << wheelPoseBetweenNoise[0], wheelPoseBetweenNoise[1], wheelPoseBetweenNoise[2], wheelPoseBetweenNoise[3],
      wheelPoseBetweenNoise[4], wheelPoseBetweenNoise[5];
  /// Linear Velocities
  const auto wheelLinearVelocitiesNoise =
      graph_msf::tryGetParam<std::vector<double>>(node_, "noise_params.wheelLinearVelocitiesNoiseDensity");  // left,right
  wheelLinearVelocitiesNoise_ << wheelLinearVelocitiesNoise[0], wheelLinearVelocitiesNoise[1], wheelLinearVelocitiesNoise[2];
  /// VIO Odometry
  const auto vioPoseBetweenNoise =
      graph_msf::tryGetParam<std::vector<double>>(node_, "noise_params.vioPoseBetweenNoiseDensity");  // roll,pitch,yaw,x,y,z
  vioPoseBetweenNoise_ << vioPoseBetweenNoise[0], vioPoseBetweenNoise[1], vioPoseBetweenNoise[2], vioPoseBetweenNoise[3],
      vioPoseBetweenNoise[4], vioPoseBetweenNoise[5];

  // Set frames
  /// LiDAR odometry frame
  dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())
      ->setLioOdometryFrame(graph_msf::tryGetParam<std::string>(node_, "extrinsics.lidarOdometryFrame"));
  /// Wheel Odometry frame
  dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())
      ->setWheelOdometryBetweenFrame(graph_msf::tryGetParam<std::string>(node_, "extrinsics.wheelOdometryBetweenFrame"));
  /// Whel Linear Velocities frames
  /// Left
  dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())
      ->setWheelLinearVelocityLeftFrame(graph_msf::tryGetParam<std::string>(node_, "extrinsics.wheelLinearVelocityLeftFrame"));
  /// Right
  dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())
      ->setWheelLinearVelocityRightFrame(graph_msf::tryGetParam<std::string>(node_, "extrinsics.wheelLinearVelocityRightFrame"));

  /// VIO Odometry frame
  dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())
      ->setVioOdometryFrame(graph_msf::tryGetParam<std::string>(node_, "extrinsics.vioOdometryFrame"));

  // Wheel Radius
  wheelRadiusMeter_ = graph_msf::tryGetParam<double>(node_, "sensor_params.wheelRadius");
}

}  // namespace smb_se
