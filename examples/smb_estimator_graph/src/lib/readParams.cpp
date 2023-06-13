/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "smb_estimator_graph/SmbEstimator.h"

// Project
#include "smb_estimator_graph/SmbStaticTransforms.h"

namespace anymal_se {

void AnymalEstimator::readParams_(const ros::NodeHandle& privateNode) {
  // Check
  if (!graphConfigPtr_) {
    throw std::runtime_error("AnymalEstimator: graphConfigPtr must be initialized.");
  }

  // Sensor Params
  lidarRate_ = graph_msf::tryGetParam<double>("sensor_params/lidarOdometryRate", privateNode);
  gnssRate_ = graph_msf::tryGetParam<double>("sensor_params/gnssRate", privateNode);

  // Noise Parameters
  /// LiDAR Odometry
  const auto poseBetweenNoise =
      graph_msf::tryGetParam<std::vector<double>>("noise_params/poseBetweenNoise", privateNode);  // roll,pitch,yaw,x,y,z
  poseBetweenNoise_ << poseBetweenNoise[0], poseBetweenNoise[1], poseBetweenNoise[2], poseBetweenNoise[3], poseBetweenNoise[4],
      poseBetweenNoise[5];
  const auto poseUnaryNoise =
      graph_msf::tryGetParam<std::vector<double>>("noise_params/poseUnaryNoise", privateNode);  // roll,pitch,yaw,x,y,z
  poseUnaryNoise_ << poseUnaryNoise[0], poseUnaryNoise[1], poseUnaryNoise[2], poseUnaryNoise[3], poseUnaryNoise[4], poseUnaryNoise[5];
  /// Gnss
  gnssPositionUnaryNoise_ = graph_msf::tryGetParam<double>("noise_params/gnssPositionUnaryNoise", privateNode);
  gnssHeadingUnaryNoise_ = graph_msf::tryGetParam<double>("noise_params/gnssHeadingUnaryNoise", privateNode);

  // Gnss parameters
  if (graphConfigPtr_->usingGnssFlag) {
    gnssHandlerPtr_->usingGnssReferenceFlag = graph_msf::tryGetParam<bool>("gnss/useGnssReference", privateNode);
    gnssHandlerPtr_->setGnssReferenceLatitude(graph_msf::tryGetParam<double>("gnss/referenceLatitude", privateNode));
    gnssHandlerPtr_->setGnssReferenceLongitude(graph_msf::tryGetParam<double>("gnss/referenceLongitude", privateNode));
    gnssHandlerPtr_->setGnssReferenceAltitude(graph_msf::tryGetParam<double>("gnss/referenceAltitude", privateNode));
    gnssHandlerPtr_->setGnssReferenceHeading(graph_msf::tryGetParam<double>("gnss/referenceHeading", privateNode));
  }

  // Set frames
  /// LiDAR frame
  dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())
      ->setLidarFrame(graph_msf::tryGetParam<std::string>("extrinsics/lidarFrame", privateNode));
  /// Gnss frame
  dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())
      ->setGnssFrame(graph_msf::tryGetParam<std::string>("extrinsics/gnssFrame", privateNode));
}

}  // namespace anymal_se
