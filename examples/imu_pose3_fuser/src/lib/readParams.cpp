/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "imu_pose3_fuser/ImuPose3Fuser.h"

// GraphMSF ROS
#include "graph_msf_ros/ros/read_ros_params.h"

namespace imu_pose3_fuser {

void ImuPose3Fuser::readParams_(const ros::NodeHandle& privateNode) {
  // Check
  if (!graphConfigPtr_) {
    throw std::runtime_error("SmbEstimator: graphConfigPtr must be initialized.");
  }

  // Sensor Params
  pose3OdometryRate_ = graph_msf::tryGetParam<double>("sensor_params/pose3OdometryRate", privateNode);

  // Noise Parameters
  /// Pose3 Odometry
  const auto poseUnaryNoise =
      graph_msf::tryGetParam<std::vector<double>>("noise_params/pose3UnaryNoiseDensity", privateNode);  // roll,pitch,yaw,x,y,z
  pose3UnaryNoise_ << poseUnaryNoise[0], poseUnaryNoise[1], poseUnaryNoise[2], poseUnaryNoise[3], poseUnaryNoise[4], poseUnaryNoise[5];
}

}  // namespace imu_pose3_fuser
