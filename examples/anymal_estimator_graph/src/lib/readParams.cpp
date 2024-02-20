/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "anymal_estimator_graph/AnymalEstimator.h"

// GraphMSF ROS
#include "graph_msf_ros/ros/read_ros_params.h"

// Project
#include "anymal_estimator_graph/AnymalStaticTransforms.h"

namespace anymal_se {

void AnymalEstimator::readParams_(const ros::NodeHandle& privateNode) {
  // Check
  if (!graphConfigPtr_) {
    throw std::runtime_error("AnymalEstimator: graphConfigPtr must be initialized.");
  }

  // Sensor Params
  lioOdometryRate_ = graph_msf::tryGetParam<double>("sensor_params/lioOdometryRate", privateNode);
  leggedOdometryRate_ = graph_msf::tryGetParam<double>("sensor_params/leggedOdometryRate", privateNode);
  gnssRate_ = graph_msf::tryGetParam<double>("sensor_params/gnssRate", privateNode);

  // Noise Parameters
  /// LiDAR Odometry
  const auto poseUnaryNoise =
      graph_msf::tryGetParam<std::vector<double>>("noise_params/lioPoseUnaryNoise", privateNode);  // roll,pitch,yaw,x,y,z
  lioPoseUnaryNoise_ << poseUnaryNoise[0], poseUnaryNoise[1], poseUnaryNoise[2], poseUnaryNoise[3], poseUnaryNoise[4], poseUnaryNoise[5];

  /// LiDAR Odometry
  const auto legPoseBetweenNoise =
      graph_msf::tryGetParam<std::vector<double>>("noise_params/legPoseBetweenNoise", privateNode);  // roll,pitch,yaw,x,y,z
  legPoseBetweenNoise_ << legPoseBetweenNoise[0], legPoseBetweenNoise[1], legPoseBetweenNoise[2], legPoseBetweenNoise[3],
      legPoseBetweenNoise[4], legPoseBetweenNoise[5];

  /// Gnss
  gnssPositionUnaryNoise_ = graph_msf::tryGetParam<double>("noise_params/gnssPositionUnaryNoise", privateNode);
  gnssHeadingUnaryNoise_ = graph_msf::tryGetParam<double>("noise_params/gnssHeadingUnaryNoise", privateNode);

  // GNSS
  useGnssFlag_ = graph_msf::tryGetParam<bool>("launch/usingGnss", privateNode);

  // Legged Odometry
  useLeggedOdometryFlag_ = graph_msf::tryGetParam<bool>("launch/usingLeggedOdometry", privateNode);

  // Gnss parameters
  if (useGnssFlag_) {
    gnssHandlerPtr_->usingGnssReferenceFlag = graph_msf::tryGetParam<bool>("gnss/useGnssReference", privateNode);
    gnssHandlerPtr_->setGnssReferenceLatitude(graph_msf::tryGetParam<double>("gnss/referenceLatitude", privateNode));
    gnssHandlerPtr_->setGnssReferenceLongitude(graph_msf::tryGetParam<double>("gnss/referenceLongitude", privateNode));
    gnssHandlerPtr_->setGnssReferenceAltitude(graph_msf::tryGetParam<double>("gnss/referenceAltitude", privateNode));
    gnssHandlerPtr_->setGnssReferenceHeading(graph_msf::tryGetParam<double>("gnss/referenceHeading", privateNode));
  }

  // Set frames
  /// LiDAR frame
  dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())
      ->setLioOdometryFrame(graph_msf::tryGetParam<std::string>("extrinsics/lioOdometryFrame", privateNode));

  /// Legged Odometry frame
  dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())
      ->setLeggedOdometryFrame(graph_msf::tryGetParam<std::string>("extrinsics/leggedOdometryFrame", privateNode));

  /// Gnss frame
  dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())
      ->setGnssFrame(graph_msf::tryGetParam<std::string>("extrinsics/gnssFrame", privateNode));
}

}  // namespace anymal_se
