/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
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
#include "anymal_estimator_graph/constants.h"

namespace anymal_se {

void AnymalEstimator::readParams(const ros::NodeHandle& privateNode) {
  graph_msf::GraphMsfRos::readParams(privateNode);

  const auto requireVectorParam = [&privateNode](const std::string& key, const std::size_t expectedSize) {
    const auto values = graph_msf::tryGetParam<std::vector<double>>(key, privateNode);
    if (values.size() != expectedSize) {
      throw std::runtime_error("AnymalEstimator: parameter '" + key + "' must contain " + std::to_string(expectedSize) +
                               " entries, but got " + std::to_string(values.size()) + ".");
    }
    return values;
  };

  // Check
  if (!graphConfigPtr_) {
    throw std::runtime_error("AnymalEstimator: graphConfigPtr must be initialized.");
  }
  if (!anymalStaticTransformsPtr_) {
    throw std::runtime_error("AnymalEstimator: typed static transforms must be initialized.");
  }

  // Sensor Params ---------------------------------------------------
  // GNSS
  gnssRate_ = graph_msf::tryGetParam<double>("sensor_params/gnssRate", privateNode);
  // LiDAR unary / absolute odometry.
  lioOdometryRate_ =
      graph_msf::tryGetParamWithAliases<double>({"sensor_params/lioOdometryRate", "sensor_params/lidarOdometryRate"}, privateNode);
  // Keep a dedicated LiDAR-between rate so the between sensor can mirror the ROS 2 reference contract when configured.
  lioBetweenOdometryRate_ =
      graph_msf::tryGetParamWithAliases<double>({"sensor_params/lioBetweenOdometryRate", "sensor_params/lioBetweenRate",
                                                 "sensor_params/lioOdometryRate", "sensor_params/lidarOdometryRate"},
                                                privateNode);
  // Legged Between
  leggedOdometryBetweenRate_ = graph_msf::tryGetParam<double>("sensor_params/leggedOdometryBetweenRate", privateNode);
  leggedOdometryPoseDownsampleFactor_ = graph_msf::tryGetParam<int>("sensor_params/leggedOdometryPoseDownsampleFactor", privateNode);
  // Legged Velocity
  leggedOdometryVelocityRate_ = graph_msf::tryGetParam<double>("sensor_params/leggedOdometryVelocityRate", privateNode);
  leggedOdometryVelocityDownsampleFactor_ =
      graph_msf::tryGetParam<int>("sensor_params/leggedOdometryVelocityDownsampleFactor", privateNode);
  // Legged Kinematics
  leggedKinematicsRate_ = graph_msf::tryGetParam<double>("sensor_params/leggedKinematicsRate", privateNode);
  leggedKinematicsDownsampleFactor_ = graph_msf::tryGetParam<int>("sensor_params/leggedKinematicsDownsampleFactor", privateNode);
  // VIO
  vioOdometryRate_ = graph_msf::tryGetParam<double>("sensor_params/vioOdometryRate", privateNode);
  vioOdometryBetweenRate_ = graph_msf::tryGetParam<double>("sensor_params/vioOdometryBetweenRate", privateNode);

  // Alignment Parameters
  const auto initialSe3AlignmentNoise = requireVectorParam("alignment_params/initialSe3AlignmentNoiseDensity", 6);
  initialSe3AlignmentNoise_ << initialSe3AlignmentNoise[0], initialSe3AlignmentNoise[1], initialSe3AlignmentNoise[2],
      initialSe3AlignmentNoise[3], initialSe3AlignmentNoise[4], initialSe3AlignmentNoise[5];
  const auto lioSe3AlignmentRandomWalk = requireVectorParam("alignment_params/lioSe3AlignmentRandomWalk", 6);
  lioSe3AlignmentRandomWalk_ << lioSe3AlignmentRandomWalk[0], lioSe3AlignmentRandomWalk[1], lioSe3AlignmentRandomWalk[2],
      lioSe3AlignmentRandomWalk[3], lioSe3AlignmentRandomWalk[4], lioSe3AlignmentRandomWalk[5];
  const auto vioSe3AlignmentRandomWalk = requireVectorParam("alignment_params/vioSe3AlignmentRandomWalk", 6);
  vioSe3AlignmentRandomWalk_ << vioSe3AlignmentRandomWalk[0], vioSe3AlignmentRandomWalk[1], vioSe3AlignmentRandomWalk[2],
      vioSe3AlignmentRandomWalk[3], vioSe3AlignmentRandomWalk[4], vioSe3AlignmentRandomWalk[5];

  // Noise Parameters ---------------------------------------------------
  /// LiDAR Odometry
  const auto poseUnaryNoise = requireVectorParam("noise_params/lioPoseUnaryNoiseDensity", 6);  // roll,pitch,yaw,x,y,z
  lioPoseUnaryNoise_ << poseUnaryNoise[0], poseUnaryNoise[1], poseUnaryNoise[2], poseUnaryNoise[3], poseUnaryNoise[4], poseUnaryNoise[5];

  /// LiDAR Odometry as Between
  const auto lioPoseBetweenNoise = requireVectorParam("noise_params/lioPoseBetweenNoiseDensity", 6);  // roll,pitch,yaw,x,y,z
  lioPoseBetweenNoise_ << lioPoseBetweenNoise[0], lioPoseBetweenNoise[1], lioPoseBetweenNoise[2], lioPoseBetweenNoise[3],
      lioPoseBetweenNoise[4], lioPoseBetweenNoise[5];

  /// VIO Odometry Unary
  const auto vioPoseUnaryNoise = requireVectorParam("noise_params/vioPoseUnaryNoiseDensity", 6);  // roll,pitch,yaw,x,y,z
  vioPoseUnaryNoise_ << vioPoseUnaryNoise[0], vioPoseUnaryNoise[1], vioPoseUnaryNoise[2], vioPoseUnaryNoise[3], vioPoseUnaryNoise[4],
      vioPoseUnaryNoise[5];

  /// VIO Odometry Between
  const auto vioPoseBetweenNoise = requireVectorParam("noise_params/vioPoseBetweenNoiseDensity", 6);  // roll,pitch,yaw,x,y,z
  vioPoseBetweenNoise_ << vioPoseBetweenNoise[0], vioPoseBetweenNoise[1], vioPoseBetweenNoise[2], vioPoseBetweenNoise[3],
      vioPoseBetweenNoise[4], vioPoseBetweenNoise[5];

  /// Legged Odometry
  const auto legPoseBetweenNoise = requireVectorParam("noise_params/legPoseBetweenNoiseDensity", 6);  // roll,pitch,yaw,x,y,z
  legPoseBetweenNoise_ << legPoseBetweenNoise[0], legPoseBetweenNoise[1], legPoseBetweenNoise[2], legPoseBetweenNoise[3],
      legPoseBetweenNoise[4], legPoseBetweenNoise[5];

  /// Legged Velocity Unary
  const auto legVelocityUnaryNoise = requireVectorParam("noise_params/legVelocityUnaryNoiseDensity", 3);  // vx,vy,vz
  legVelocityUnaryNoise_ << legVelocityUnaryNoise[0], legVelocityUnaryNoise[1], legVelocityUnaryNoise[2];

  /// Legged Kinematics Foot Position Unary
  const auto legKinematicsFootPositionUnaryNoise =
      requireVectorParam("noise_params/legKinematicsFootPositionUnaryNoiseDensity", 3);  // x,y,z
  legKinematicsFootPositionUnaryNoise_ << legKinematicsFootPositionUnaryNoise[0], legKinematicsFootPositionUnaryNoise[1],
      legKinematicsFootPositionUnaryNoise[2];
  gnssPositionOutlierThreshold_ = graph_msf::tryGetParam<double>("noise_params/gnssPositionOutlierThreshold", privateNode);

  // Flags ---------------------------------------------------
  // Read estimator enable flags from YAML sensor_params only.
  // Keep the supported sensor_params aliases, but do not fall back to launch/ parameters.
  useGnssUnaryFlag_ = graph_msf::tryGetParamWithAliases<bool>({"sensor_params/useGnss", "sensor_params/useGnssUnary"}, privateNode);
  useLioUnaryFlag_ = graph_msf::tryGetParamWithAliases<bool>({"sensor_params/useLioOdometry", "sensor_params/useLioUnary"}, privateNode);
  useLioBetweenFlag_ =
      graph_msf::tryGetParamWithAliases<bool>({"sensor_params/useLioBetweenOdometry", "sensor_params/useLioBetween"}, privateNode);
  useVioOdometryFlag_ = graph_msf::tryGetParamWithAliases<bool>({"sensor_params/useVioOdometry"}, privateNode);
  useVioOdometryBetweenFlag_ = graph_msf::tryGetParamWithAliases<bool>({"sensor_params/useVioOdometryBetween"}, privateNode);
  // Legged Between Odometry
  useLeggedBetweenFlag_ =
      graph_msf::tryGetParamWithAliases<bool>({"sensor_params/useLeggedOdometryBetween", "sensor_params/useLeggedBetween"}, privateNode);
  // Legged Velocity Unary
  useLeggedVelocityUnaryFlag_ = graph_msf::tryGetParamWithAliases<bool>({"sensor_params/useLeggedVelocityUnary"}, privateNode);
  // Legged Kinematics
  useLeggedKinematicsFlag_ = graph_msf::tryGetParamWithAliases<bool>({"sensor_params/useLeggedKinematics"}, privateNode);

  graphConfigPtr_->useAdaptiveAdditionalOptimizationIterationsFlag_ =
      graph_msf::tryGetParam<bool>("graph_params/useAdaptiveAdditionalOptimizationIterations", privateNode);
  graphConfigPtr_->adaptiveAdditionalOptimizationMinRelativeErrorImprovement_ =
      graph_msf::tryGetParam<double>("graph_params/adaptiveAdditionalOptimizationMinRelativeErrorImprovement", privateNode);
  graphConfigPtr_->printAdditionalOptimizationDiagnosticsFlag_ =
      graph_msf::tryGetParam<bool>("graph_params/printAdditionalOptimizationDiagnostics", privateNode);
  graphConfigPtr_->deferFutureUnaryMeasurementsFlag_ =
      graph_msf::tryGetParam<bool>("graph_params/deferFutureUnaryMeasurements", privateNode);
  graphConfigPtr_->maxDeferredUnaryFutureLeadSeconds_ =
      graph_msf::tryGetParam<double>("graph_params/maxDeferredUnaryFutureLeadSeconds", privateNode);
  graphConfigPtr_->maxDeferredUnaryMeasurementsInQueue_ =
      graph_msf::tryGetParam<int>("graph_params/maxDeferredUnaryMeasurementsInQueue", privateNode);

  // Gnss parameters ---------------------------------------------------
  gnssHandlerPtr_ = std::make_shared<graph_msf::GnssHandler>();

  const bool useYawInitialGuessFromFile =
      graph_msf::tryGetParamWithAliases<bool>({"gnss/useYawInitialGuessFromFile", "gnss_params/useYawInitialGuessFromFile"}, privateNode);
  const bool useYawInitialGuessFromAlignment = graph_msf::tryGetParamWithAliases<bool>(
      {"gnss/yawInitialGuessFromAlignment", "gnss_params/yawInitialGuessFromAlignment"}, privateNode);
  const double initYawDegFromFile = graph_msf::tryGetParamWithAliases<double>({"gnss/initYaw", "gnss_params/initYaw"}, privateNode);
  const bool useGnssReference =
      graph_msf::tryGetParamWithAliases<bool>({"gnss/useGnssReference", "gnss_params/useGnssReference"}, privateNode);
  const double referenceLatitude =
      graph_msf::tryGetParamWithAliases<double>({"gnss/referenceLatitude", "gnss_params/referenceLatitude"}, privateNode);
  const double referenceLongitude =
      graph_msf::tryGetParamWithAliases<double>({"gnss/referenceLongitude", "gnss_params/referenceLongitude"}, privateNode);
  const double referenceAltitude =
      graph_msf::tryGetParamWithAliases<double>({"gnss/referenceAltitude", "gnss_params/referenceAltitude"}, privateNode);
  const double referenceHeading =
      graph_msf::tryGetParamWithAliases<double>({"gnss/referenceHeading", "gnss_params/referenceHeading"}, privateNode);

  gnssHandlerPtr_->setUseYawInitialGuessFromFile(useYawInitialGuessFromFile);
  gnssHandlerPtr_->setUseYawInitialGuessFromAlignment(useYawInitialGuessFromAlignment);
  gnssHandlerPtr_->setGlobalYawDegFromFile(initYawDegFromFile);
  gnssHandlerPtr_->setUseGnssReferenceFlag(useGnssReference);
  gnssHandlerPtr_->setGnssReferenceLatitude(referenceLatitude);
  gnssHandlerPtr_->setGnssReferenceLongitude(referenceLongitude);
  gnssHandlerPtr_->setGnssReferenceAltitude(referenceAltitude);
  gnssHandlerPtr_->setGnssReferenceHeading(referenceHeading);

  const double trajectoryAlignmentLidarRate = graph_msf::tryGetParam<double>("trajectoryAlignment/lidarRate", privateNode);
  const double trajectoryAlignmentGnssRate = graph_msf::tryGetParam<double>("trajectoryAlignment/gnssRate", privateNode);
  const double trajectoryAlignmentMinDistanceHeadingInit =
      graph_msf::tryGetParam<double>("trajectoryAlignment/minimumDistanceHeadingInit", privateNode);
  const double trajectoryAlignmentMinimumSpatialSpread =
      graph_msf::tryGetParam<double>("trajectoryAlignment/minimumSpatialSpread", privateNode);
  const double trajectoryAlignmentNoMovementDistance =
      graph_msf::tryGetParam<double>("trajectoryAlignment/noMovementDistance", privateNode);
  const double trajectoryAlignmentNoMovementTime = graph_msf::tryGetParam<double>("trajectoryAlignment/noMovementTime", privateNode);
  initSyncMaxDt_ = graph_msf::tryGetParam<double>("trajectoryAlignment/initSyncMaxDt", privateNode);

  if (gnssHandlerPtr_->getUseYawInitialGuessFromAlignment()) {
    gnssHandlerPtr_->setUseYawInitialGuessFromFile(false);
    trajectoryAlignmentHandler_ = std::make_shared<graph_msf::TrajectoryAlignmentHandler>();
    trajectoryAlignmentHandler_->setSe3Rate(trajectoryAlignmentLidarRate);
    trajectoryAlignmentHandler_->setR3Rate(trajectoryAlignmentGnssRate);
    trajectoryAlignmentHandler_->setMinDistanceHeadingInit(trajectoryAlignmentMinDistanceHeadingInit);
    trajectoryAlignmentHandler_->setMinimumSpatialSpread(trajectoryAlignmentMinimumSpatialSpread);
    trajectoryAlignmentHandler_->setNoMovementDistance(trajectoryAlignmentNoMovementDistance);
    trajectoryAlignmentHandler_->setNoMovementTime(trajectoryAlignmentNoMovementTime);
  }

  if (gnssHandlerPtr_->getUseGnssReferenceFlag()) {
    REGULAR_COUT << GREEN_START << " Using GNSS reference from parameters." << COLOR_END << std::endl;
  } else {
    REGULAR_COUT << GREEN_START << " Will wait for GNSS measurements to initialize reference coordinates." << COLOR_END << std::endl;
  }

  // Coordinate Frames ---------------------------------------------------
  /// LiDAR frame
  anymalStaticTransformsPtr_->setLioOdometryFrame(
      graph_msf::tryGetParamWithAliases<std::string>({"extrinsics/lioOdometryFrame", "extrinsics/lidarOdometryFrame"}, privateNode));

  /// Legged Odometry frame
  anymalStaticTransformsPtr_->setLeggedOdometryFrame(graph_msf::tryGetParam<std::string>("extrinsics/leggedOdometryFrame", privateNode));

  /// Gnss frame
  anymalStaticTransformsPtr_->setGnssFrame(graph_msf::tryGetParam<std::string>("extrinsics/gnssFrame", privateNode));

  /// VIO frame
  const std::string vioOdometryFrame =
      graph_msf::tryGetParamWithAliases<std::string>({"extrinsics/vioOdometryFrame", "extrinsics/vioOdometryBetweenFrame"}, privateNode);
  // Keep the B2W-compatible frame value in the parameter files, but only require the TF lookup
  // when one of the VIO streams is actually enabled in this ROS 1 wrapper.
  if (useVioOdometryFlag_ || useVioOdometryBetweenFlag_) {
    anymalStaticTransformsPtr_->setVioOdometryFrame(vioOdometryFrame);
  } else {
    anymalStaticTransformsPtr_->setVioOdometryFrame(std::string(""));
  }
}

}  // namespace anymal_se
