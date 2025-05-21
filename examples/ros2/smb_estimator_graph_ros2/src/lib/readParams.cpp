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

// GraphMSF ROS
#include "graph_msf_ros2/ros/read_ros_params.h"

namespace smb_se {

void SmbEstimator::readParams(const ros::NodeHandle& privateNode) {
  // Check
  if (!graphConfigPtr_) {
    throw std::runtime_error("SmbEstimator: graphConfigPtr must be initialized.");
  }

  // Flags
  useLioOdometryFlag_ = graph_msf::tryGetParam<bool>("sensor_params/useLioOdometry", privateNode);
  useWheelOdometryBetweenFlag_ = graph_msf::tryGetParam<bool>("sensor_params/useWheelOdometryBetween", privateNode);
  useWheelLinearVelocitiesFlag_ = graph_msf::tryGetParam<bool>("sensor_params/useWheelLinearVelocities", privateNode);
  useVioOdometryFlag_ = graph_msf::tryGetParam<bool>("sensor_params/useVioOdometry", privateNode);

  // Sensor Params
  lioOdometryRate_ = graph_msf::tryGetParam<double>("sensor_params/lioOdometryRate", privateNode);
  wheelOdometryBetweenRate_ = graph_msf::tryGetParam<double>("sensor_params/wheelOdometryBetweenRate", privateNode);
  wheelLinearVelocitiesRate_ = graph_msf::tryGetParam<double>("sensor_params/wheelLinearVelocitiesRate", privateNode);
  vioOdometryRate_ = graph_msf::tryGetParam<double>("sensor_params/vioOdometryRate", privateNode);

  // Alignment Parameters
  const auto initialSe3AlignmentNoiseDensity =
      graph_msf::tryGetParam<std::vector<double>>("alignment_params/initialSe3AlignmentNoiseDensity", privateNode);
  initialSe3AlignmentNoise_ << initialSe3AlignmentNoiseDensity[0], initialSe3AlignmentNoiseDensity[1], initialSe3AlignmentNoiseDensity[2],
      initialSe3AlignmentNoiseDensity[3], initialSe3AlignmentNoiseDensity[4], initialSe3AlignmentNoiseDensity[5];
  const auto lioSe3AlignmentRandomWalk =
      graph_msf::tryGetParam<std::vector<double>>("alignment_params/lioSe3AlignmentRandomWalk", privateNode);
  lioSe3AlignmentRandomWalk_ << lioSe3AlignmentRandomWalk[0], lioSe3AlignmentRandomWalk[1], lioSe3AlignmentRandomWalk[2],
      lioSe3AlignmentRandomWalk[3], lioSe3AlignmentRandomWalk[4], lioSe3AlignmentRandomWalk[5];

  // Noise Parameters
  /// LiDAR Odometry
  const auto poseUnaryNoise =
      graph_msf::tryGetParam<std::vector<double>>("noise_params/lioPoseUnaryNoiseDensity", privateNode);  // roll,pitch,yaw,x,y,z
  lioPoseUnaryNoise_ << poseUnaryNoise[0], poseUnaryNoise[1], poseUnaryNoise[2], poseUnaryNoise[3], poseUnaryNoise[4], poseUnaryNoise[5];
  /// Wheel Odometry
  /// Between
  const auto wheelPoseBetweenNoise =
      graph_msf::tryGetParam<std::vector<double>>("noise_params/wheelPoseBetweenNoiseDensity", privateNode);  // roll,pitch,yaw,x,y,z
  wheelPoseBetweenNoise_ << wheelPoseBetweenNoise[0], wheelPoseBetweenNoise[1], wheelPoseBetweenNoise[2], wheelPoseBetweenNoise[3],
      wheelPoseBetweenNoise[4], wheelPoseBetweenNoise[5];
  /// Linear Velocities
  const auto wheelLinearVelocitiesNoise =
      graph_msf::tryGetParam<std::vector<double>>("noise_params/wheelLinearVelocitiesNoiseDensity", privateNode);  // left,right
  wheelLinearVelocitiesNoise_ << wheelLinearVelocitiesNoise[0], wheelLinearVelocitiesNoise[1], wheelLinearVelocitiesNoise[2];
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
      ->setWheelOdometryBetweenFrame(graph_msf::tryGetParam<std::string>("extrinsics/wheelOdometryBetweenFrame", privateNode));
  /// Whel Linear Velocities frames
  /// Left
  dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())
      ->setWheelLinearVelocityLeftFrame(graph_msf::tryGetParam<std::string>("extrinsics/wheelLinearVelocityLeftFrame", privateNode));
  /// Right
  dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())
      ->setWheelLinearVelocityRightFrame(graph_msf::tryGetParam<std::string>("extrinsics/wheelLinearVelocityRightFrame", privateNode));

  /// VIO Odometry frame
  dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())
      ->setVioOdometryFrame(graph_msf::tryGetParam<std::string>("extrinsics/vioOdometryFrame", privateNode));

  // Wheel Radius
  wheelRadiusMeter_ = graph_msf::tryGetParam<double>("sensor_params/wheelRadius", privateNode);
}

}  // namespace smb_se
