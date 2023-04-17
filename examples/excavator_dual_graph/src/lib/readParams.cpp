/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#include "excavator_dual_graph/ExcavatorEstimator.h"

namespace excavator_se {

void ExcavatorEstimator::readParams_(const ros::NodeHandle& privateNode) {
  // Variables for parameter fetching
  double dParam;
  int iParam;
  bool bParam;
  std::string sParam;

  // Call super method
  graph_msf::GraphMsfRos::readParams_(privateNode);

  // Set frames
  /// LiDAR frame
  std::string frame = graph_msf::tryGetParam<std::string>("extrinsics/lidarFrame", privateNode);
  dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->setLidarFrame(frame);
  /// Cabin frame
  frame = graph_msf::tryGetParam<std::string>("extrinsics/cabinFrame", privateNode);
  dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->setCabinFrame(frame);
  /// Left Gnss frame
  frame = graph_msf::tryGetParam<std::string>("extrinsics/gnssFrame1", privateNode);
  dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->setLeftGnssFrame(frame);
  /// Right Gnss frame
  frame = graph_msf::tryGetParam<std::string>("extrinsics/gnssFrame2", privateNode);
  dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->setRightGnssFrame(frame);

  // Sensor Parameters
  lidarRate_ = graph_msf::tryGetParam<double>("sensor_params/lidarOdometryRate", privateNode);
  gnssLeftRate_ = graph_msf::tryGetParam<double>("sensor_params/gnssRate", privateNode);
  gnssRightRate_ = graph_msf::tryGetParam<double>("sensor_params/gnssRate", privateNode);

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

  // GNSS Parameters
  if (graphConfigPtr_->usingGnssFlag) {
    // Gnss parameters
    gnssHandlerPtr_->usingGnssReferenceFlag = graph_msf::tryGetParam<bool>("gnss/useGnssReference", privateNode);
    gnssHandlerPtr_->setGnssReferenceLatitude(graph_msf::tryGetParam<double>("gnss/referenceLatitude", privateNode));
    gnssHandlerPtr_->setGnssReferenceLongitude(graph_msf::tryGetParam<double>("gnss/referenceLongitude", privateNode));
    gnssHandlerPtr_->setGnssReferenceAltitude(graph_msf::tryGetParam<double>("gnss/referenceAltitude", privateNode));
    gnssHandlerPtr_->setGnssReferenceHeading(graph_msf::tryGetParam<double>("gnss/referenceHeading", privateNode));
  }
}

}  // namespace excavator_se
