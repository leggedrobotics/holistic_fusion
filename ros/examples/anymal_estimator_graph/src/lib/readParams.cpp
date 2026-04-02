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
  const auto initialSe3AlignmentNoise =
      graph_msf::tryGetParam<std::vector<double>>("alignment_params/initialSe3AlignmentNoiseDensity", privateNode);
  initialSe3AlignmentNoise_ << initialSe3AlignmentNoise[0], initialSe3AlignmentNoise[1], initialSe3AlignmentNoise[2],
      initialSe3AlignmentNoise[3], initialSe3AlignmentNoise[4], initialSe3AlignmentNoise[5];
  const auto lioSe3AlignmentRandomWalk =
      graph_msf::tryGetParam<std::vector<double>>("alignment_params/lioSe3AlignmentRandomWalk", privateNode);
  lioSe3AlignmentRandomWalk_ << lioSe3AlignmentRandomWalk[0], lioSe3AlignmentRandomWalk[1], lioSe3AlignmentRandomWalk[2],
      lioSe3AlignmentRandomWalk[3], lioSe3AlignmentRandomWalk[4], lioSe3AlignmentRandomWalk[5];
  const auto vioSe3AlignmentRandomWalk =
      graph_msf::tryGetParam<std::vector<double>>("alignment_params/vioSe3AlignmentRandomWalk", privateNode);
  vioSe3AlignmentRandomWalk_ << vioSe3AlignmentRandomWalk[0], vioSe3AlignmentRandomWalk[1], vioSe3AlignmentRandomWalk[2],
      vioSe3AlignmentRandomWalk[3], vioSe3AlignmentRandomWalk[4], vioSe3AlignmentRandomWalk[5];

  // Noise Parameters ---------------------------------------------------
  /// LiDAR Odometry
  const auto poseUnaryNoise =
      graph_msf::tryGetParam<std::vector<double>>("noise_params/lioPoseUnaryNoiseDensity", privateNode);  // roll,pitch,yaw,x,y,z
  lioPoseUnaryNoise_ << poseUnaryNoise[0], poseUnaryNoise[1], poseUnaryNoise[2], poseUnaryNoise[3], poseUnaryNoise[4], poseUnaryNoise[5];

  /// LiDAR Odometry as Between
  const auto lioPoseBetweenNoise =
      graph_msf::tryGetParam<std::vector<double>>("noise_params/lioPoseBetweenNoiseDensity", privateNode);  // roll,pitch,yaw,x,y,z
  lioPoseBetweenNoise_ << lioPoseBetweenNoise[0], lioPoseBetweenNoise[1], lioPoseBetweenNoise[2], lioPoseBetweenNoise[3],
      lioPoseBetweenNoise[4], lioPoseBetweenNoise[5];

  /// VIO Odometry Unary
  const auto vioPoseUnaryNoise =
      graph_msf::tryGetParam<std::vector<double>>("noise_params/vioPoseUnaryNoiseDensity", privateNode);  // roll,pitch,yaw,x,y,z
  vioPoseUnaryNoise_ << vioPoseUnaryNoise[0], vioPoseUnaryNoise[1], vioPoseUnaryNoise[2], vioPoseUnaryNoise[3], vioPoseUnaryNoise[4],
      vioPoseUnaryNoise[5];

  /// VIO Odometry Between
  const auto vioPoseBetweenNoise =
      graph_msf::tryGetParam<std::vector<double>>("noise_params/vioPoseBetweenNoiseDensity", privateNode);  // roll,pitch,yaw,x,y,z
  vioPoseBetweenNoise_ << vioPoseBetweenNoise[0], vioPoseBetweenNoise[1], vioPoseBetweenNoise[2], vioPoseBetweenNoise[3],
      vioPoseBetweenNoise[4], vioPoseBetweenNoise[5];

  /// Legged Odometry
  const auto legPoseBetweenNoise =
      graph_msf::tryGetParam<std::vector<double>>("noise_params/legPoseBetweenNoiseDensity", privateNode);  // roll,pitch,yaw,x,y,z
  legPoseBetweenNoise_ << legPoseBetweenNoise[0], legPoseBetweenNoise[1], legPoseBetweenNoise[2], legPoseBetweenNoise[3],
      legPoseBetweenNoise[4], legPoseBetweenNoise[5];

  /// Legged Velocity Unary
  const auto legVelocityUnaryNoise =
      graph_msf::tryGetParam<std::vector<double>>("noise_params/legVelocityUnaryNoiseDensity", privateNode);  // vx,vy,vz
  legVelocityUnaryNoise_ << legVelocityUnaryNoise[0], legVelocityUnaryNoise[1], legVelocityUnaryNoise[2];

  /// Legged Kinematics Foot Position Unary
  const auto legKinematicsFootPositionUnaryNoise =
      graph_msf::tryGetParam<std::vector<double>>("noise_params/legKinematicsFootPositionUnaryNoiseDensity", privateNode);  // x,y,z
  legKinematicsFootPositionUnaryNoise_ << legKinematicsFootPositionUnaryNoise[0], legKinematicsFootPositionUnaryNoise[1],
      legKinematicsFootPositionUnaryNoise[2];

  // Flags ---------------------------------------------------
  // Accept both the legacy ROS 1 launch flags and the B2W-style sensor flags so existing launch files
  // keep working while the estimator can also be driven with the newer ROS 2-like parameter layout.
  useGnssUnaryFlag_ = graph_msf::tryGetParamWithAliases<bool>(
      {"launch/usingGnssUnary", "sensor_params/useGnss", "sensor_params/useGnssUnary"}, privateNode);
  useLioUnaryFlag_ = graph_msf::tryGetParamWithAliases<bool>(
      {"launch/usingLioUnary", "sensor_params/useLioOdometry", "sensor_params/useLioUnary"}, privateNode);
  useLioBetweenFlag_ = graph_msf::tryGetParamWithAliases<bool>(
      {"launch/usingLioBetween", "sensor_params/useLioBetweenOdometry", "sensor_params/useLioBetween"}, privateNode);
  useVioOdometryFlag_ = graph_msf::tryGetParamWithAliases<bool>({"launch/usingVioOdometry", "sensor_params/useVioOdometry"}, privateNode);
  useVioOdometryBetweenFlag_ =
      graph_msf::tryGetParamWithAliases<bool>({"launch/usingVioOdometryBetween", "sensor_params/useVioOdometryBetween"}, privateNode);
  // Legged Between Odometry
  useLeggedBetweenFlag_ = graph_msf::tryGetParam<bool>("launch/usingLeggedBetween", privateNode);
  // Legged Velocity Unary
  useLeggedVelocityUnaryFlag_ = graph_msf::tryGetParam<bool>("launch/usingLeggedVelocityUnary", privateNode);
  // Legged Kinematics
  useLeggedKinematicsFlag_ = graph_msf::tryGetParam<bool>("launch/usingLeggedKinematics", privateNode);

  // Gnss parameters ---------------------------------------------------
  if (useGnssUnaryFlag_) {
    // GNSS Handler
    gnssHandlerPtr_ = std::make_shared<graph_msf::GnssHandler>();

    // Read Yaw initial guess options
    gnssHandlerPtr_->setUseYawInitialGuessFromFile(graph_msf::tryGetParamWithAliases<bool>(
        {"gnss/useYawInitialGuessFromFile", "gnss_params/useYawInitialGuessFromFile"}, privateNode));
    gnssHandlerPtr_->setUseYawInitialGuessFromAlignment(graph_msf::tryGetParamWithAliases<bool>(
        {"gnss/yawInitialGuessFromAlignment", "gnss_params/yawInitialGuessFromAlignment"}, privateNode));

    // Alignment options.
    if (gnssHandlerPtr_->getUseYawInitialGuessFromAlignment()) {
      // Make sure no dual true
      gnssHandlerPtr_->setUseYawInitialGuessFromFile(false);
      trajectoryAlignmentHandler_ = std::make_shared<graph_msf::TrajectoryAlignmentHandler>();

      trajectoryAlignmentHandler_->setSe3Rate(graph_msf::tryGetParam<double>("trajectoryAlignment/lidarRate", privateNode));
      trajectoryAlignmentHandler_->setR3Rate(graph_msf::tryGetParam<double>("trajectoryAlignment/gnssRate", privateNode));

      trajectoryAlignmentHandler_->setMinDistanceHeadingInit(
          graph_msf::tryGetParam<double>("trajectoryAlignment/minimumDistanceHeadingInit", privateNode));
      trajectoryAlignmentHandler_->setMinimumSpatialSpread(
          graph_msf::tryGetParam<double>("trajectoryAlignment/minimumSpatialSpread", privateNode));
      trajectoryAlignmentHandler_->setNoMovementDistance(
          graph_msf::tryGetParam<double>("trajectoryAlignment/noMovementDistance", privateNode));
      trajectoryAlignmentHandler_->setNoMovementTime(graph_msf::tryGetParam<double>("trajectoryAlignment/noMovementTime", privateNode));

    } else if (!gnssHandlerPtr_->getUseYawInitialGuessFromAlignment() && gnssHandlerPtr_->getUseYawInitialGuessFromFile()) {
      gnssHandlerPtr_->setGlobalYawDegFromFile(
          graph_msf::tryGetParamWithAliases<double>({"gnss/initYaw", "gnss_params/initYaw"}, privateNode));
    }

    // GNSS Reference
    gnssHandlerPtr_->setUseGnssReferenceFlag(
        graph_msf::tryGetParamWithAliases<bool>({"gnss/useGnssReference", "gnss_params/useGnssReference"}, privateNode));

    if (gnssHandlerPtr_->getUseGnssReferenceFlag()) {
      REGULAR_COUT << GREEN_START << " Using GNSS reference from parameters." << COLOR_END << std::endl;
      gnssHandlerPtr_->setGnssReferenceLatitude(
          graph_msf::tryGetParamWithAliases<double>({"gnss/referenceLatitude", "gnss_params/referenceLatitude"}, privateNode));
      gnssHandlerPtr_->setGnssReferenceLongitude(
          graph_msf::tryGetParamWithAliases<double>({"gnss/referenceLongitude", "gnss_params/referenceLongitude"}, privateNode));
      gnssHandlerPtr_->setGnssReferenceAltitude(
          graph_msf::tryGetParamWithAliases<double>({"gnss/referenceAltitude", "gnss_params/referenceAltitude"}, privateNode));
      gnssHandlerPtr_->setGnssReferenceHeading(
          graph_msf::tryGetParamWithAliases<double>({"gnss/referenceHeading", "gnss_params/referenceHeading"}, privateNode));
    } else {
      REGULAR_COUT << GREEN_START << " Will wait for GNSS measurements to initialize reference coordinates." << COLOR_END << std::endl;
    }

    // GNSS Outlier Threshold
    gnssPositionOutlierThreshold_ = graph_msf::tryGetParam<double>("noise_params/gnssPositionOutlierThreshold", privateNode);
  }  // End GNSS Unary

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
