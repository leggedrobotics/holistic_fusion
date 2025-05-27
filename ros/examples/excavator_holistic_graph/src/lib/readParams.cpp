/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "excavator_holistic_graph/ExcavatorEstimator.h"

// Workspace
#include "graph_msf_ros/ros/read_ros_params.h"

namespace excavator_se {

void ExcavatorEstimator::readParams(const ros::NodeHandle& privateNode) {
  // Variables for parameter fetching
  double dParam;
  int iParam;
  bool bParam;
  std::string sParam;

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

  // Alignment Parameters ----------------------------
  /// Initial Se3 Alignment
  auto initialSe3AlignmentNoiseDensity =
      graph_msf::tryGetParam<std::vector<double>>("alignment_params/initialSe3AlignmentNoiseDensity", privateNode);
  initialSe3AlignmentNoiseDensity_ << initialSe3AlignmentNoiseDensity[0], initialSe3AlignmentNoiseDensity[1],
      initialSe3AlignmentNoiseDensity[2], initialSe3AlignmentNoiseDensity[3], initialSe3AlignmentNoiseDensity[4],
      initialSe3AlignmentNoiseDensity[5];
  ///
  auto lioSe3AlignmentRandomWalk = graph_msf::tryGetParam<std::vector<double>>("alignment_params/lioSe3AlignmentRandomWalk", privateNode);
  lioSe3AlignmentRandomWalk_ << lioSe3AlignmentRandomWalk[0], lioSe3AlignmentRandomWalk[1], lioSe3AlignmentRandomWalk[2],
      lioSe3AlignmentRandomWalk[3], lioSe3AlignmentRandomWalk[4], lioSe3AlignmentRandomWalk[5];

  /// Noise Parameters ----
  /// LiDAR Odometry
  const auto lioPoseUnaryNoise =
      graph_msf::tryGetParam<std::vector<double>>("noise_params/lioPoseUnaryNoiseDensity", privateNode);  // roll,pitch,yaw,x,y,z
  lioPoseUnaryNoise_ << lioPoseUnaryNoise[0], lioPoseUnaryNoise[1], lioPoseUnaryNoise[2], lioPoseUnaryNoise[3], lioPoseUnaryNoise[4],
      lioPoseUnaryNoise[5];
  /// Gnss
  gnssPositionUnaryNoise_ = graph_msf::tryGetParam<double>("noise_params/gnssPositionUnaryNoiseDensity", privateNode);
  gnssHeadingUnaryNoise_ = graph_msf::tryGetParam<double>("noise_params/gnssHeadingUnaryNoiseDensity", privateNode);

  // Launch Parameters
  /// LiDAR Odometry
  useLioOdometryFlag_ = graph_msf::tryGetParam<bool>("launch/useLioOdometry", privateNode);
  useLeftGnssFlag_ = graph_msf::tryGetParam<bool>("launch/useLeftGnss", privateNode);
  useRightGnssFlag_ = graph_msf::tryGetParam<bool>("launch/useRightGnss", privateNode);

  // GNSS Parameters
  if (useLeftGnssFlag_ || useRightGnssFlag_) {
    // Gnss parameters
    gnssHandlerPtr_->setUseGnssReferenceFlag(graph_msf::tryGetParam<bool>("gnss/useGnssReference", privateNode));
    gnssHandlerPtr_->setGnssReferenceLatitude(graph_msf::tryGetParam<double>("gnss/referenceLatitude", privateNode));
    gnssHandlerPtr_->setGnssReferenceLongitude(graph_msf::tryGetParam<double>("gnss/referenceLongitude", privateNode));
    gnssHandlerPtr_->setGnssReferenceAltitude(graph_msf::tryGetParam<double>("gnss/referenceAltitude", privateNode));
    gnssHandlerPtr_->setGnssReferenceHeading(graph_msf::tryGetParam<double>("gnss/referenceHeading", privateNode));
  }
}

}  // namespace excavator_se
