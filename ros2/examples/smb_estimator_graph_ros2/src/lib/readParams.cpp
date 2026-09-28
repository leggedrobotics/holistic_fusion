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

// HolisticFusion ROS2
#include "holistic_fusion_ros2/ros/read_ros_params.h"

namespace smb_se {

void SmbEstimator::readParams() {
  // Check
  if (!graphConfigPtr_) {
    throw std::runtime_error("SmbEstimator: graphConfigPtr must be initialized.");
  }

  // Flags
  useLioOdometryFlag_ = holistic_fusion::tryGetParam<bool>(this, "sensor_params.useLioOdometry");
  dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())->setUseLioOdometryFlag(useLioOdometryFlag_);
  useWheelOdometryBetweenFlag_ = holistic_fusion::tryGetParam<bool>(this, "sensor_params.useWheelOdometryBetween");
  dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())->setUseWheelOdometryBetweenFlag(useWheelOdometryBetweenFlag_);
  useWheelLinearVelocitiesFlag_ = holistic_fusion::tryGetParam<bool>(this, "sensor_params.useWheelLinearVelocities");
  dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())->setUseWheelLinearVelocitiesFlag(useWheelLinearVelocitiesFlag_);
  useVioOdometryFlag_ = holistic_fusion::tryGetParam<bool>(this, "sensor_params.useVioOdometry");
  dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())->setUseVioOdometryFlag(useVioOdometryFlag_);

  // Sensor Params
  lioOdometryRate_ = holistic_fusion::tryGetParam<int>(this, "sensor_params.lioOdometryRate");
  wheelOdometryBetweenRate_ = holistic_fusion::tryGetParam<int>(this, "sensor_params.wheelOdometryBetweenRate");
  wheelLinearVelocitiesRate_ = holistic_fusion::tryGetParam<int>(this, "sensor_params.wheelLinearVelocitiesRate");
  vioOdometryRate_ = holistic_fusion::tryGetParam<int>(this, "sensor_params.vioOdometryRate");

  // Alignment Parameters
  const auto initialSe3AlignmentStdDev =
      holistic_fusion::tryGetParam<std::vector<double>>(this, "alignment_params.initialSe3AlignmentStdDev");
  initialSe3AlignmentNoise_ << initialSe3AlignmentStdDev[0], initialSe3AlignmentStdDev[1], initialSe3AlignmentStdDev[2],
      initialSe3AlignmentStdDev[3], initialSe3AlignmentStdDev[4], initialSe3AlignmentStdDev[5];
  const auto lioSe3AlignmentRandomWalk =
      holistic_fusion::tryGetParam<std::vector<double>>(this, "alignment_params.lioSe3AlignmentRandomWalk");
  lioSe3AlignmentRandomWalk_ << lioSe3AlignmentRandomWalk[0], lioSe3AlignmentRandomWalk[1], lioSe3AlignmentRandomWalk[2],
      lioSe3AlignmentRandomWalk[3], lioSe3AlignmentRandomWalk[4], lioSe3AlignmentRandomWalk[5];

  // Noise Parameters
  /// LiDAR Odometry
  const auto poseUnaryNoise =
            holistic_fusion::tryGetParam<std::vector<double>>(this, "noise_params.lioPoseUnaryStdDev");  // roll,pitch,yaw,x,y,z
  lioPoseUnaryNoise_ << poseUnaryNoise[0], poseUnaryNoise[1], poseUnaryNoise[2], poseUnaryNoise[3], poseUnaryNoise[4], poseUnaryNoise[5];
  /// Wheel Odometry
  /// Between
  const auto wheelPoseBetweenNoise =
      holistic_fusion::tryGetParam<std::vector<double>>(this, "noise_params.wheelPoseBetweenNoiseDensity");  // roll,pitch,yaw,x,y,z
  wheelPoseBetweenNoise_ << wheelPoseBetweenNoise[0], wheelPoseBetweenNoise[1], wheelPoseBetweenNoise[2], wheelPoseBetweenNoise[3],
      wheelPoseBetweenNoise[4], wheelPoseBetweenNoise[5];
  /// Linear Velocities
  const auto wheelLinearVelocitiesNoise =
      holistic_fusion::tryGetParam<std::vector<double>>(this, "noise_params.wheelLinearVelocitiesNoiseDensity");  // left,right
  wheelLinearVelocitiesNoise_ << wheelLinearVelocitiesNoise[0], wheelLinearVelocitiesNoise[1], wheelLinearVelocitiesNoise[2];
  /// VIO Odometry
  const auto vioPoseBetweenNoise =
      holistic_fusion::tryGetParam<std::vector<double>>(this, "noise_params.vioPoseBetweenNoiseDensity");  // roll,pitch,yaw,x,y,z
  vioPoseBetweenNoise_ << vioPoseBetweenNoise[0], vioPoseBetweenNoise[1], vioPoseBetweenNoise[2], vioPoseBetweenNoise[3],
      vioPoseBetweenNoise[4], vioPoseBetweenNoise[5];

  // Set frames
  /// LiDAR odometry frame
  dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())
      ->setLioOdometryFrame(holistic_fusion::tryGetParam<std::string>(this, "extrinsics.lidarOdometryFrame"));
  /// Wheel Odometry frame
  dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())
      ->setWheelOdometryBetweenFrame(holistic_fusion::tryGetParam<std::string>(this, "extrinsics.wheelOdometryBetweenFrame"));
  /// Whel Linear Velocities frames
  /// Left
  dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())
      ->setWheelLinearVelocityLeftFrame(holistic_fusion::tryGetParam<std::string>(this, "extrinsics.wheelLinearVelocityLeftFrame"));
  /// Right
  dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())
      ->setWheelLinearVelocityRightFrame(holistic_fusion::tryGetParam<std::string>(this, "extrinsics.wheelLinearVelocityRightFrame"));

  /// VIO Odometry frame
  dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())
      ->setVioOdometryFrame(holistic_fusion::tryGetParam<std::string>(this, "extrinsics.vioOdometryFrame"));

  // Wheel Radius
  wheelRadiusMeter_ = holistic_fusion::tryGetParam<double>(this, "sensor_params.wheelRadius");
}

}  // namespace smb_se
