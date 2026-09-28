/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "anymal_estimator_graph/AnymalEstimator.h"

// HolisticFusion ROS
#include "holistic_fusion_ros/ros/read_ros_params.h"

// Project
#include "anymal_estimator_graph/AnymalStaticTransforms.h"
#include "anymal_estimator_graph/constants.h"

namespace anymal_se {

void AnymalEstimator::readParams(const ros::NodeHandle& privateNode) {
  // Check
  if (!graphConfigPtr_) {
    throw std::runtime_error("AnymalEstimator: graphConfigPtr must be initialized.");
  }

  // Sensor Params
  // GNSS
  gnssRate_ = holistic_fusion::tryGetParam<double>("sensor_params/gnssRate", privateNode);
  // LIO
  lioOdometryRate_ = holistic_fusion::tryGetParam<double>("sensor_params/lioOdometryRate", privateNode);
  // Legged Between
  leggedOdometryBetweenRate_ = holistic_fusion::tryGetParam<double>("sensor_params/leggedOdometryBetweenRate", privateNode);
  leggedOdometryPoseDownsampleFactor_ = holistic_fusion::tryGetParam<int>("sensor_params/leggedOdometryPoseDownsampleFactor", privateNode);
  // Legged Velocity
  leggedOdometryVelocityRate_ = holistic_fusion::tryGetParam<double>("sensor_params/leggedOdometryVelocityRate", privateNode);
  leggedOdometryVelocityDownsampleFactor_ =
      holistic_fusion::tryGetParam<int>("sensor_params/leggedOdometryVelocityDownsampleFactor", privateNode);
  // Legged Kinematics
  leggedKinematicsRate_ = holistic_fusion::tryGetParam<double>("sensor_params/leggedKinematicsRate", privateNode);
  leggedKinematicsDownsampleFactor_ = holistic_fusion::tryGetParam<int>("sensor_params/leggedKinematicsDownsampleFactor", privateNode);

  // Alignment Parameters
  const auto initialSe3AlignmentStdDev =
      holistic_fusion::tryGetParam<std::vector<double>>("alignment_params/initialSe3AlignmentStdDev", privateNode);
  initialSe3AlignmentNoise_ << initialSe3AlignmentStdDev[0], initialSe3AlignmentStdDev[1], initialSe3AlignmentStdDev[2],
      initialSe3AlignmentStdDev[3], initialSe3AlignmentStdDev[4], initialSe3AlignmentStdDev[5];
  const auto lioSe3AlignmentRandomWalk =
      holistic_fusion::tryGetParam<std::vector<double>>("alignment_params/lioSe3AlignmentRandomWalk", privateNode);
  lioSe3AlignmentRandomWalk_ << lioSe3AlignmentRandomWalk[0], lioSe3AlignmentRandomWalk[1], lioSe3AlignmentRandomWalk[2],
      lioSe3AlignmentRandomWalk[3], lioSe3AlignmentRandomWalk[4], lioSe3AlignmentRandomWalk[5];

  // Noise Parameters ---------------------------------------------------
  /// LiDAR Odometry
  const auto poseUnaryNoise =
            holistic_fusion::tryGetParam<std::vector<double>>("noise_params/lioPoseUnaryStdDev", privateNode);  // roll,pitch,yaw,x,y,z
  lioPoseUnaryNoise_ << poseUnaryNoise[0], poseUnaryNoise[1], poseUnaryNoise[2], poseUnaryNoise[3], poseUnaryNoise[4], poseUnaryNoise[5];

  /// LiDAR Odometry as Between
  const auto lioPoseBetweenNoise =
      holistic_fusion::tryGetParam<std::vector<double>>("noise_params/lioPoseBetweenNoiseDensity", privateNode);  // roll,pitch,yaw,x,y,z
  lioPoseBetweenNoise_ << lioPoseBetweenNoise[0], lioPoseBetweenNoise[1], lioPoseBetweenNoise[2], lioPoseBetweenNoise[3],
      lioPoseBetweenNoise[4], lioPoseBetweenNoise[5];

  /// Legged Odometry
  const auto legPoseBetweenNoise =
      holistic_fusion::tryGetParam<std::vector<double>>("noise_params/legPoseBetweenNoiseDensity", privateNode);  // roll,pitch,yaw,x,y,z
  legPoseBetweenNoise_ << legPoseBetweenNoise[0], legPoseBetweenNoise[1], legPoseBetweenNoise[2], legPoseBetweenNoise[3],
      legPoseBetweenNoise[4], legPoseBetweenNoise[5];

  /// Legged Velocity Unary
  const auto legVelocityUnaryNoise =
      holistic_fusion::tryGetParam<std::vector<double>>("noise_params/legVelocityUnaryNoiseDensity", privateNode);  // vx,vy,vz
  legVelocityUnaryNoise_ << legVelocityUnaryNoise[0], legVelocityUnaryNoise[1], legVelocityUnaryNoise[2];

  /// Legged Kinematics Foot Position Unary
  const auto legKinematicsFootPositionUnaryNoise =
      holistic_fusion::tryGetParam<std::vector<double>>("noise_params/legKinematicsFootPositionUnaryNoiseDensity", privateNode);  // x,y,z
  legKinematicsFootPositionUnaryNoise_ << legKinematicsFootPositionUnaryNoise[0], legKinematicsFootPositionUnaryNoise[1],
      legKinematicsFootPositionUnaryNoise[2];

  // Flags ---------------------------------------------------
  // GNSS Unary
  useGnssUnaryFlag_ = holistic_fusion::tryGetParam<bool>("launch/usingGnssUnary", privateNode);
  // LIO Unary
  useLioUnaryFlag_ = holistic_fusion::tryGetParam<bool>("launch/usingLioUnary", privateNode);
  // LIO Between
  useLioBetweenFlag_ = holistic_fusion::tryGetParam<bool>("launch/usingLioBetween", privateNode);
  // Legged Between Odometry
  useLeggedBetweenFlag_ = holistic_fusion::tryGetParam<bool>("launch/usingLeggedBetween", privateNode);
  // Legged Velocity Unary
  useLeggedVelocityUnaryFlag_ = holistic_fusion::tryGetParam<bool>("launch/usingLeggedVelocityUnary", privateNode);
  // Legged Kinematics
  useLeggedKinematicsFlag_ = holistic_fusion::tryGetParam<bool>("launch/usingLeggedKinematics", privateNode);

  // Gnss parameters ---------------------------------------------------
  if (useGnssUnaryFlag_) {
    // GNSS Handler
    gnssHandlerPtr_ = std::make_shared<holistic_fusion::GnssHandler>();

    // Read Yaw initial guess options
    gnssHandlerPtr_->setUseYawInitialGuessFromFile(holistic_fusion::tryGetParam<bool>("gnss/useYawInitialGuessFromFile", privateNode));
    gnssHandlerPtr_->setUseYawInitialGuessFromAlignment(holistic_fusion::tryGetParam<bool>("gnss/yawInitialGuessFromAlignment", privateNode));

    // Alignment options.
    if (gnssHandlerPtr_->getUseYawInitialGuessFromAlignment()) {
      // Make sure no dual true
      gnssHandlerPtr_->setUseYawInitialGuessFromFile(false);
      trajectoryAlignmentHandler_ = std::make_shared<holistic_fusion::TrajectoryAlignmentHandler>();

      trajectoryAlignmentHandler_->setSe3Rate(holistic_fusion::tryGetParam<double>("trajectoryAlignment/lidarRate", privateNode));
      trajectoryAlignmentHandler_->setR3Rate(holistic_fusion::tryGetParam<double>("trajectoryAlignment/gnssRate", privateNode));

      trajectoryAlignmentHandler_->setMinDistanceHeadingInit(
          holistic_fusion::tryGetParam<double>("trajectoryAlignment/minimumDistanceHeadingInit", privateNode));
      trajectoryAlignmentHandler_->setNoMovementDistance(
          holistic_fusion::tryGetParam<double>("trajectoryAlignment/noMovementDistance", privateNode));
      trajectoryAlignmentHandler_->setNoMovementTime(holistic_fusion::tryGetParam<double>("trajectoryAlignment/noMovementTime", privateNode));

    } else if (!gnssHandlerPtr_->getUseYawInitialGuessFromAlignment() && gnssHandlerPtr_->getUseYawInitialGuessFromFile()) {
      gnssHandlerPtr_->setGlobalYawDegFromFile(holistic_fusion::tryGetParam<double>("gnss/initYaw", privateNode));
    }

    // GNSS Reference
    gnssHandlerPtr_->setUseGnssReferenceFlag(holistic_fusion::tryGetParam<bool>("gnss/useGnssReference", privateNode));

    if (gnssHandlerPtr_->getUseGnssReferenceFlag()) {
      REGULAR_COUT << GREEN_START << " Using GNSS reference from parameters." << COLOR_END << std::endl;
      gnssHandlerPtr_->setGnssReferenceLatitude(holistic_fusion::tryGetParam<double>("gnss/referenceLatitude", privateNode));
      gnssHandlerPtr_->setGnssReferenceLongitude(holistic_fusion::tryGetParam<double>("gnss/referenceLongitude", privateNode));
      gnssHandlerPtr_->setGnssReferenceAltitude(holistic_fusion::tryGetParam<double>("gnss/referenceAltitude", privateNode));
      gnssHandlerPtr_->setGnssReferenceHeading(holistic_fusion::tryGetParam<double>("gnss/referenceHeading", privateNode));
    } else {
      REGULAR_COUT << GREEN_START << " Will wait for GNSS measurements to initialize reference coordinates." << COLOR_END << std::endl;
    }

    // GNSS Outlier Threshold
    gnssPositionOutlierThreshold_ = holistic_fusion::tryGetParam<double>("noise_params/gnssPositionOutlierThreshold", privateNode);
  }  // End GNSS Unary

  // Coordinate Frames ---------------------------------------------------
  /// LiDAR frame
  dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())
      ->setLioOdometryFrame(holistic_fusion::tryGetParam<std::string>("extrinsics/lioOdometryFrame", privateNode));

  /// Legged Odometry frame
  dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())
      ->setLeggedOdometryFrame(holistic_fusion::tryGetParam<std::string>("extrinsics/leggedOdometryFrame", privateNode));

  /// Gnss frame
  dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())
      ->setGnssFrame(holistic_fusion::tryGetParam<std::string>("extrinsics/gnssFrame", privateNode));
}

}  // namespace anymal_se
