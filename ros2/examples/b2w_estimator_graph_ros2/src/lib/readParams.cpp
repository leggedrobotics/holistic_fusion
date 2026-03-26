/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "b2w_estimator_graph_ros2/B2WEstimator.h"

// Project
#include "b2w_estimator_graph_ros2/B2WStaticTransforms.h"
#include "b2w_estimator_graph_ros2/constants.h"

// GraphMSF ROS2
#include "graph_msf_ros2/ros/read_ros_params.h"

namespace b2w_se {

void B2WEstimator::readParams() {
  // Check
  if (!graphConfigPtr_) {
    throw std::runtime_error("B2WEstimator: graphConfigPtr must be initialized.");
  }

  // Flags
  useGnssFlag_ = graph_msf::tryGetParam<bool>(this, "sensor_params.useGnss");
  dynamic_cast<B2WStaticTransforms*>(staticTransformsPtr_.get())->setUseGnssFlag(useGnssFlag_);

  useLioOdometryFlag_ = graph_msf::tryGetParam<bool>(this, "sensor_params.useLioOdometry");
  dynamic_cast<B2WStaticTransforms*>(staticTransformsPtr_.get())->setUseLioOdometryFlag(useLioOdometryFlag_);

  useLioBetweenOdometryFlag_ = graph_msf::tryGetParam<bool>(this, "sensor_params.useLioBetweenOdometry");
  dynamic_cast<B2WStaticTransforms*>(staticTransformsPtr_.get())->setUseLioBetweenOdometryFlag(useLioBetweenOdometryFlag_);

  useWheelOdometryBetweenFlag_ = graph_msf::tryGetParam<bool>(this, "sensor_params.useWheelOdometryBetween");
  dynamic_cast<B2WStaticTransforms*>(staticTransformsPtr_.get())->setUseWheelOdometryBetweenFlag(useWheelOdometryBetweenFlag_);

  useWheelLinearVelocitiesFlag_ = graph_msf::tryGetParam<bool>(this, "sensor_params.useWheelLinearVelocities");
  dynamic_cast<B2WStaticTransforms*>(staticTransformsPtr_.get())->setUseWheelLinearVelocitiesFlag(useWheelLinearVelocitiesFlag_);

  useVioOdometryFlag_ = graph_msf::tryGetParam<bool>(this, "sensor_params.useVioOdometry");
  dynamic_cast<B2WStaticTransforms*>(staticTransformsPtr_.get())->setUseVioOdometryFlag(useVioOdometryFlag_);

  useVioOdometryBetweenFlag_ = graph_msf::tryGetParam<bool>(this, "sensor_params.useVioOdometryBetween");
  dynamic_cast<B2WStaticTransforms*>(staticTransformsPtr_.get())->setUseVioOdometryBetweenFlag(useVioOdometryBetweenFlag_);


    // Print sensor flags in bright green
    RCLCPP_INFO(this->get_logger(), "\033[92m=== B2W Estimator Configuration ===\033[0m");
    RCLCPP_INFO(this->get_logger(), "\033[92m- Use GNSS: %s\033[0m", useGnssFlag_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "\033[92m- Use LIO Odometry: %s\033[0m", useLioOdometryFlag_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "\033[92m- Use LIO Between Odometry: %s\033[0m", useLioBetweenOdometryFlag_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "\033[92m- Use Wheel Odometry Between: %s\033[0m", useWheelOdometryBetweenFlag_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "\033[92m- Use Wheel Linear Velocities: %s\033[0m", useWheelLinearVelocitiesFlag_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "\033[92m- Use VIO Odometry: %s\033[0m", useVioOdometryFlag_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "\033[92m- Use VIO Odometry Between: %s\033[0m", useVioOdometryBetweenFlag_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "\033[92m===============================\033[0m");

  // Sensor Param as
  lioOdometryRate_ = graph_msf::tryGetParam<int>(this, "sensor_params.lioOdometryRate");
  gnssRate_ = graph_msf::tryGetParam<int>(this, "sensor_params.gnssRate");
  lioBetweenOdometryRate_ = graph_msf::tryGetParam<int>(this, "sensor_params.lioBetweenOdometryRate");
  wheelOdometryBetweenRate_ = graph_msf::tryGetParam<int>(this, "sensor_params.wheelOdometryBetweenRate");
  wheelLinearVelocitiesRate_ = graph_msf::tryGetParam<int>(this, "sensor_params.wheelLinearVelocitiesRate");
  vioOdometryRate_ = graph_msf::tryGetParam<int>(this, "sensor_params.vioOdometryRate");
vioOdometryBetweenRate_ = graph_msf::tryGetParam<int>(this, "sensor_params.vioOdometryBetweenRate");

  // Alignment Parameters
  const auto initialSe3AlignmentNoiseDensity =
      graph_msf::tryGetParam<std::vector<double>>(this, "alignment_params.initialSe3AlignmentNoiseDensity");
  initialSe3AlignmentNoise_ << initialSe3AlignmentNoiseDensity[0], initialSe3AlignmentNoiseDensity[1], initialSe3AlignmentNoiseDensity[2],
      initialSe3AlignmentNoiseDensity[3], initialSe3AlignmentNoiseDensity[4], initialSe3AlignmentNoiseDensity[5];
  const auto lioSe3AlignmentRandomWalk =
      graph_msf::tryGetParam<std::vector<double>>(this, "alignment_params.lioSe3AlignmentRandomWalk");
  lioSe3AlignmentRandomWalk_ << lioSe3AlignmentRandomWalk[0], lioSe3AlignmentRandomWalk[1], lioSe3AlignmentRandomWalk[2],
      lioSe3AlignmentRandomWalk[3], lioSe3AlignmentRandomWalk[4], lioSe3AlignmentRandomWalk[5];

  const auto vioSe3AlignmentRandomWalk =
      graph_msf::tryGetParam<std::vector<double>>(this, "alignment_params.vioSe3AlignmentRandomWalk");
  vioSe3AlignmentRandomWalk_ << vioSe3AlignmentRandomWalk[0], vioSe3AlignmentRandomWalk[1], vioSe3AlignmentRandomWalk[2],
      vioSe3AlignmentRandomWalk[3], vioSe3AlignmentRandomWalk[4], vioSe3AlignmentRandomWalk[5];

  // Noise Parameters
  /// LiDAR Odometry
  const auto lidarUnaryNoise =
      graph_msf::tryGetParam<std::vector<double>>(this, "noise_params.lioPoseUnaryNoiseDensity");  // roll,pitch,yaw,x,y,z
  lioPoseUnaryNoise_ << lidarUnaryNoise[0], lidarUnaryNoise[1], lidarUnaryNoise[2], lidarUnaryNoise[3], lidarUnaryNoise[4], lidarUnaryNoise[5];

  const auto lidarBetweenNoise =
      graph_msf::tryGetParam<std::vector<double>>(this, "noise_params.lioPoseBetweenNoiseDensity");  // roll,pitch,yaw,x,y,z
  lioBetweenNoise_ << lidarBetweenNoise[0], lidarBetweenNoise[1], lidarBetweenNoise[2], lidarBetweenNoise[3], lidarBetweenNoise[4], lidarBetweenNoise[5];

  /// Wheel Odometry
  /// Between
  const auto wheelPoseBetweenNoise =
      graph_msf::tryGetParam<std::vector<double>>(this, "noise_params.wheelPoseBetweenNoiseDensity");  // roll,pitch,yaw,x,y,z
  wheelPoseBetweenNoise_ << wheelPoseBetweenNoise[0], wheelPoseBetweenNoise[1], wheelPoseBetweenNoise[2], wheelPoseBetweenNoise[3],
      wheelPoseBetweenNoise[4], wheelPoseBetweenNoise[5];
  /// Linear Velocities
  const auto wheelLinearVelocitiesNoise =
      graph_msf::tryGetParam<std::vector<double>>(this, "noise_params.wheelLinearVelocitiesNoiseDensity");  // left,right
  wheelLinearVelocitiesNoise_ << wheelLinearVelocitiesNoise[0], wheelLinearVelocitiesNoise[1], wheelLinearVelocitiesNoise[2];
  /// VIO Odometry Between
  const auto vioPoseBetweenNoise =
      graph_msf::tryGetParam<std::vector<double>>(this, "noise_params.vioPoseBetweenNoiseDensity");  // roll,pitch,yaw,x,y,z
  vioPoseBetweenNoise_ << vioPoseBetweenNoise[0], vioPoseBetweenNoise[1], vioPoseBetweenNoise[2], vioPoseBetweenNoise[3],
      vioPoseBetweenNoise[4], vioPoseBetweenNoise[5];

    /// VIO Odometry Unary
    const auto vioPoseUnaryNoise =
            graph_msf::tryGetParam<std::vector<double>>(this, "noise_params.vioPoseUnaryNoiseDensity");  // roll,pitch,yaw,x,y,z
    vioPoseUnaryNoise_ << vioPoseUnaryNoise[0], vioPoseUnaryNoise[1], vioPoseUnaryNoise[2], vioPoseUnaryNoise[3],
            vioPoseUnaryNoise[4], vioPoseUnaryNoise[5];

// GNSS parameters
if (useGnssFlag_) {
    // GNSS Handler
    gnssHandlerPtr_ = std::make_shared<graph_msf::GnssHandler>();

    // Read Yaw initial guess options
    gnssHandlerPtr_->setUseYawInitialGuessFromFile(graph_msf::tryGetParam<bool>(this, "gnss_params.useYawInitialGuessFromFile"));
    gnssHandlerPtr_->setUseYawInitialGuessFromAlignment(graph_msf::tryGetParam<bool>(this, "gnss_params.yawInitialGuessFromAlignment"));

    // Alignment options
    if (gnssHandlerPtr_->getUseYawInitialGuessFromAlignment()) {
    // Make sure no dual true
    gnssHandlerPtr_->setUseYawInitialGuessFromFile(false);
    trajectoryAlignmentHandler_ = std::make_shared<graph_msf::TrajectoryAlignmentHandler>();

    trajectoryAlignmentHandler_->setSe3Rate(graph_msf::tryGetParam<double>(this, "trajectoryAlignment.lidarRate"));
    trajectoryAlignmentHandler_->setR3Rate(graph_msf::tryGetParam<double>(this, "trajectoryAlignment.gnssRate"));

    trajectoryAlignmentHandler_->setMinDistanceHeadingInit(
        graph_msf::tryGetParam<double>(this, "trajectoryAlignment.minimumDistanceHeadingInit"));
    trajectoryAlignmentHandler_->setMinimumSpatialSpread(
        graph_msf::tryGetParam<double>(this, "trajectoryAlignment.minimumSpatialSpread"));
    trajectoryAlignmentHandler_->setNoMovementDistance(
        graph_msf::tryGetParam<double>(this, "trajectoryAlignment.noMovementDistance"));
    trajectoryAlignmentHandler_->setNoMovementTime(graph_msf::tryGetParam<double>(this, "trajectoryAlignment.noMovementTime"));
    } else if (!gnssHandlerPtr_->getUseYawInitialGuessFromAlignment() && gnssHandlerPtr_->getUseYawInitialGuessFromFile()) {
    gnssHandlerPtr_->setGlobalYawDegFromFile(graph_msf::tryGetParam<double>(this, "gnss_params.initYaw"));
    }

    // GNSS Reference
    gnssHandlerPtr_->setUseGnssReferenceFlag(graph_msf::tryGetParam<bool>(this, "gnss_params.useGnssReference"));

    if (gnssHandlerPtr_->getUseGnssReferenceFlag()) {
    RCLCPP_INFO(this->get_logger(), "Using GNSS reference from parameters.");
    gnssHandlerPtr_->setGnssReferenceLatitude(graph_msf::tryGetParam<double>(this, "gnss_params.referenceLatitude"));
    gnssHandlerPtr_->setGnssReferenceLongitude(graph_msf::tryGetParam<double>(this, "gnss_params.referenceLongitude"));
    gnssHandlerPtr_->setGnssReferenceAltitude(graph_msf::tryGetParam<double>(this, "gnss_params.referenceAltitude"));
    gnssHandlerPtr_->setGnssReferenceHeading(graph_msf::tryGetParam<double>(this, "gnss_params.referenceHeading"));
    } else {
    RCLCPP_INFO(this->get_logger(), "Will wait for GNSS measurements to initialize reference coordinates.");
    }

    // GNSS Outlier Threshold
    gnssPositionOutlierThreshold_ = graph_msf::tryGetParam<double>(this, "noise_params.gnssPositionOutlierThreshold");
}

  // Set frames

  dynamic_cast<B2WStaticTransforms*>(staticTransformsPtr_.get())
      ->setGnssFrame(graph_msf::tryGetParam<std::string>(this, "extrinsics.gnssFrame"));

  /// LiDAR odometry frame
  dynamic_cast<B2WStaticTransforms*>(staticTransformsPtr_.get())
      ->setLioOdometryFrame(graph_msf::tryGetParam<std::string>(this, "extrinsics.lidarOdometryFrame"));
      
  dynamic_cast<B2WStaticTransforms*>(staticTransformsPtr_.get())
      ->setWheelOdometryBetweenFrame(graph_msf::tryGetParam<std::string>(this, "extrinsics.betweenLidarOdometryFrame"));

  /// Wheel Odometry frame
  dynamic_cast<B2WStaticTransforms*>(staticTransformsPtr_.get())
      ->setWheelOdometryBetweenFrame(graph_msf::tryGetParam<std::string>(this, "extrinsics.wheelOdometryBetweenFrame"));
  /// Whel Linear Velocities frames
  /// Left
  dynamic_cast<B2WStaticTransforms*>(staticTransformsPtr_.get())
      ->setWheelLinearVelocityLeftFrame(graph_msf::tryGetParam<std::string>(this, "extrinsics.wheelLinearVelocityLeftFrame"));
  /// Right
  dynamic_cast<B2WStaticTransforms*>(staticTransformsPtr_.get())
      ->setWheelLinearVelocityRightFrame(graph_msf::tryGetParam<std::string>(this, "extrinsics.wheelLinearVelocityRightFrame"));

  // Wheel Radius
  wheelRadiusMeter_ = graph_msf::tryGetParam<double>(this, "sensor_params.wheelRadius");

  /// VIO Odometry frame
  dynamic_cast<B2WStaticTransforms*>(staticTransformsPtr_.get())
      ->setVioOdometryFrame(graph_msf::tryGetParam<std::string>(this, "extrinsics.vioOdometryFrame"));

 // VIO Odometry frame Between    
  dynamic_cast<B2WStaticTransforms*>(staticTransformsPtr_.get())
  ->setVioOdometryBetweenFrame(graph_msf::tryGetParam<std::string>(this, "extrinsics.vioOdometryBetweenFrame"));

}

}  // namespace b2w_se
