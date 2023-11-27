/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#include "excavator_holistic_graph/ExcavatorEstimator.h"

namespace excavator_se {

void ExcavatorEstimator::readParams_(const ros::NodeHandle& privateNode) {
  // Variables for parameter fetching
  double dParam;
  int iParam;
  bool bParam;
  std::string sParam;

  // Call super method
  graph_msf::GraphMsfRos::readParams_(privateNode);

  // Set frames ----------------------------
  /// LiDAR frame
  std::string frame = graph_msf::tryGetParam<std::string>("extrinsics/lidarFrame", privateNode);
  dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->setLioOdometryFrame(frame);
  /// Cabin frame
  frame = graph_msf::tryGetParam<std::string>("extrinsics/cabinFrame", privateNode);
  dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->setCabinFrame(frame);
  /// Left Gnss frame
  frame = graph_msf::tryGetParam<std::string>("extrinsics/gnssFrame1", privateNode);
  dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->setLeftGnssFrame(frame);
  /// Right Gnss frame
  frame = graph_msf::tryGetParam<std::string>("extrinsics/gnssFrame2", privateNode);
  dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->setRightGnssFrame(frame);

  // Sensor Parameters ----------------------------
  lioOdometryRate_ = graph_msf::tryGetParam<double>("sensor_params/lioOdometryRate", privateNode);
  gnssRate_ = graph_msf::tryGetParam<double>("sensor_params/gnssRate", privateNode);

  /// Noise Parameters ----
  /// LiDAR Odometry
  const auto lioPoseUnaryNoise =
      graph_msf::tryGetParam<std::vector<double>>("noise_params/lioPoseUnaryNoise", privateNode);  // roll,pitch,yaw,x,y,z
  lioPoseUnaryNoise_ << lioPoseUnaryNoise[0], lioPoseUnaryNoise[1], lioPoseUnaryNoise[2], lioPoseUnaryNoise[3], lioPoseUnaryNoise[4],
      lioPoseUnaryNoise[5];
  /// Gnss
  gnssPositionUnaryNoise_ = graph_msf::tryGetParam<double>("noise_params/gnssPositionUnaryNoise", privateNode);
  gnssHeadingUnaryNoise_ = graph_msf::tryGetParam<double>("noise_params/gnssHeadingUnaryNoise", privateNode);

  // Launch Parameters
  /// LiDAR Odometry
  useLioOdometryFlag_ = graph_msf::tryGetParam<bool>("launch/useLioOdometry", privateNode);
  useLeftGnssFlag_ = graph_msf::tryGetParam<bool>("launch/useLeftGnss", privateNode);
  useRightGnssFlag_ = graph_msf::tryGetParam<bool>("launch/useRightGnss", privateNode);

  // GNSS Parameters
  if (useLeftGnssFlag_ || useRightGnssFlag_) {
    // Gnss parameters
    gnssHandlerPtr_->usingGnssReferenceFlag = graph_msf::tryGetParam<bool>("gnss/useGnssReference", privateNode);
    gnssHandlerPtr_->setGnssReferenceLatitude(graph_msf::tryGetParam<double>("gnss/referenceLatitude", privateNode));
    gnssHandlerPtr_->setGnssReferenceLongitude(graph_msf::tryGetParam<double>("gnss/referenceLongitude", privateNode));
    gnssHandlerPtr_->setGnssReferenceAltitude(graph_msf::tryGetParam<double>("gnss/referenceAltitude", privateNode));
    gnssHandlerPtr_->setGnssReferenceHeading(graph_msf::tryGetParam<double>("gnss/referenceHeading", privateNode));
  }
}

}  // namespace excavator_se
