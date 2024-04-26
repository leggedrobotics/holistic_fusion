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

void AnymalEstimator::readParams_(const ros::NodeHandle& privateNode) {
  // Check
  if (!graphConfigPtr_) {
    throw std::runtime_error("AnymalEstimator: graphConfigPtr must be initialized.");
  }

  // Sensor Params
  lioOdometryRate_ = graph_msf::tryGetParam<double>("sensor_params/lioOdometryRate", privateNode);
  leggedOdometryRate_ = graph_msf::tryGetParam<double>("sensor_params/leggedOdometryRate", privateNode);
  gnssRate_ = graph_msf::tryGetParam<double>("sensor_params/gnssRate", privateNode);

  // Noise Parameters ---------------------------------------------------
  /// Gnss
  gnssPositionUnaryNoise_ = graph_msf::tryGetParam<double>("noise_params/gnssPositionUnaryNoise", privateNode);

  /// LiDAR Odometry
  const auto poseUnaryNoise =
      graph_msf::tryGetParam<std::vector<double>>("noise_params/lioPoseUnaryNoise", privateNode);  // roll,pitch,yaw,x,y,z
  lioPoseUnaryNoise_ << poseUnaryNoise[0], poseUnaryNoise[1], poseUnaryNoise[2], poseUnaryNoise[3], poseUnaryNoise[4], poseUnaryNoise[5];

  /// LiDAR Odometry as Between
  const auto lioPoseBetweenNoise =
      graph_msf::tryGetParam<std::vector<double>>("noise_params/lioPoseBetweenNoise", privateNode);  // roll,pitch,yaw,x,y,z
  lioPoseBetweenNoise_ << lioPoseBetweenNoise[0], lioPoseBetweenNoise[1], lioPoseBetweenNoise[2], lioPoseBetweenNoise[3],
      lioPoseBetweenNoise[4], lioPoseBetweenNoise[5];

  /// Legged Odometry
  const auto legPoseBetweenNoise =
      graph_msf::tryGetParam<std::vector<double>>("noise_params/legPoseBetweenNoise", privateNode);  // roll,pitch,yaw,x,y,z
  legPoseBetweenNoise_ << legPoseBetweenNoise[0], legPoseBetweenNoise[1], legPoseBetweenNoise[2], legPoseBetweenNoise[3],
      legPoseBetweenNoise[4], legPoseBetweenNoise[5];

  /// Legged Velocity Unary
  const auto legVelocityUnaryNoise =
      graph_msf::tryGetParam<std::vector<double>>("noise_params/legVelocityUnaryNoise", privateNode);  // vx,vy,vz

  // Flags ---------------------------------------------------
  // GNSS Unary
  useGnssUnaryFlag_ = graph_msf::tryGetParam<bool>("launch/usingGnssUnary", privateNode);
  // LIO Unary
  useLioUnaryFlag_ = graph_msf::tryGetParam<bool>("launch/usingLioUnary", privateNode);
  // LIO Between
  useLioBetweenFlag_ = graph_msf::tryGetParam<bool>("launch/usingLioBetween", privateNode);
  // Legged Between Odometry
  useLeggedBetweenFlag_ = graph_msf::tryGetParam<bool>("launch/usingLeggedBetween", privateNode);
  // Legged Velocity Unary
  useLeggedVelocityUnaryFlag_ = graph_msf::tryGetParam<bool>("launch/usingLeggedVelocityUnary", privateNode);

  // Gnss parameters ---------------------------------------------------
  if (useGnssUnaryFlag_) {
    // GNSS Handler
    gnssHandlerPtr_ = std::make_shared<graph_msf::GnssHandler>();

    // Read Yaw initial guess options
    gnssHandlerPtr_->useYawInitialGuessFromFile_ = graph_msf::tryGetParam<bool>("gnss/useYawInitialGuessFromFile", privateNode);
    gnssHandlerPtr_->yawInitialGuessFromAlignment_ = graph_msf::tryGetParam<bool>("gnss/yawInitialGuessFromAlignment", privateNode);

    // Alignment options.
    if (gnssHandlerPtr_->yawInitialGuessFromAlignment_) {
      // Make sure no dual true.
      gnssHandlerPtr_->useYawInitialGuessFromFile_ = false;
      trajectoryAlignmentHandler_ = std::make_shared<graph_msf::TrajectoryAlignmentHandler>();

      trajectoryAlignmentHandler_->setLidarRate(graph_msf::tryGetParam<double>("trajectoryAlignment/lidarRate", privateNode));
      trajectoryAlignmentHandler_->setGnssRate(graph_msf::tryGetParam<double>("trajectoryAlignment/gnssRate", privateNode));

      trajectoryAlignmentHandler_->setMinDistanceHeadingInit(
          graph_msf::tryGetParam<double>("trajectoryAlignment/minimumDistanceHeadingInit", privateNode));
      trajectoryAlignmentHandler_->setNoMovementDistance(
          graph_msf::tryGetParam<double>("trajectoryAlignment/noMovementDistance", privateNode));
      trajectoryAlignmentHandler_->setNoMovementTime(graph_msf::tryGetParam<double>("trajectoryAlignment/noMovementTime", privateNode));

    } else if (!(gnssHandlerPtr_->yawInitialGuessFromAlignment_) && (gnssHandlerPtr_->useYawInitialGuessFromFile_)) {
      gnssHandlerPtr_->globalYawDegFromFile_ = graph_msf::tryGetParam<double>("gnss/initYaw", privateNode);
    }

    // GNSS Reference
    gnssHandlerPtr_->usingGnssReferenceFlag = graph_msf::tryGetParam<bool>("gnss/useGnssReference", privateNode);

    if (gnssHandlerPtr_->usingGnssReferenceFlag) {
      REGULAR_COUT << GREEN_START << " Using GNSS reference from parameters." << COLOR_END << std::endl;
      gnssHandlerPtr_->setGnssReferenceLatitude(graph_msf::tryGetParam<double>("gnss/referenceLatitude", privateNode));
      gnssHandlerPtr_->setGnssReferenceLongitude(graph_msf::tryGetParam<double>("gnss/referenceLongitude", privateNode));
      gnssHandlerPtr_->setGnssReferenceAltitude(graph_msf::tryGetParam<double>("gnss/referenceAltitude", privateNode));
      gnssHandlerPtr_->setGnssReferenceHeading(graph_msf::tryGetParam<double>("gnss/referenceHeading", privateNode));
    } else {
      REGULAR_COUT << GREEN_START << " Will wait for GNSS measurements to initialize reference coordinates." << COLOR_END << std::endl;
    }
  }

  // Coordinate Frames ---------------------------------------------------
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
