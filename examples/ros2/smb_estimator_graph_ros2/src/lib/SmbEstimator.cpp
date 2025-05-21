/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "smb_estimator_graph_ros2/SmbEstimator.h"

// Project
#include "smb_estimator_graph_ros2/SmbStaticTransforms.h"

// Workspace
#include "graph_msf_ros2/measurements/BinaryMeasurementXD.h"
#include "graph_msf_ros2/measurements/UnaryMeasurementXD.h"
#include "graph_msf_ros2/util/conversions.h"
#include "smb_estimator_graph_ros2/constants.h"

namespace smb_se {

SmbEstimator::SmbEstimator(std::shared_ptr<ros::NodeHandle> privateNodePtr) : graph_msf::GraphMsfRos(privateNodePtr) {
  REGULAR_COUT << GREEN_START << " SmbEstimator-Constructor called." << COLOR_END << std::endl;

  // Configurations ----------------------------
  // Static transforms
  staticTransformsPtr_ = std::make_shared<SmbStaticTransforms>(privateNodePtr);

  // Set up
  SmbEstimator::setup();
}

void SmbEstimator::setup() {
  REGULAR_COUT << GREEN_START << " SmbEstimator-Setup called." << COLOR_END << std::endl;

  // Read parameters ----------------------------
  SmbEstimator::readParams(privateNode);

  // Super class
  GraphMsfRos::setup(staticTransformsPtr_);

  // Publishers ----------------------------
  SmbEstimator::initializePublishers(privateNode);

  // Subscribers ----------------------------
  SmbEstimator::initializeSubscribers(privateNode);

  // Messages ----------------------------
  SmbEstimator::initializeMessages(privateNode);

  // Services ----------------------------
  SmbEstimator::initializeServices(privateNode);

  // Transforms ----------------------------
  staticTransformsPtr_->findTransformations();

  // Wrap up ----------------------------
  REGULAR_COUT << GREEN_START << " Set up successfully." << COLOR_END << std::endl;
}

void SmbEstimator::initializePublishers(ros::NodeHandle& privateNode) {
  // Paths
  pubMeasMapLioPath_ = privateNode.advertise<nav_msgs::Path>("/graph_msf/measLiDAR_path_map_imu", ROS_QUEUE_SIZE);
  pubMeasMapVioPath_ = privateNode.advertise<nav_msgs::Path>("/graph_msf/measVIO_path_map_imu", ROS_QUEUE_SIZE);
}

void SmbEstimator::initializeSubscribers(ros::NodeHandle& privateNode) {
  // LiDAR Odometry
  if (useLioOdometryFlag_) {
    subLioOdometry_ = privateNode.subscribe<nav_msgs::Odometry>(
        "/lidar_odometry_topic", ROS_QUEUE_SIZE, &SmbEstimator::lidarOdometryCallback_, this, ros::TransportHints().tcpNoDelay());
    REGULAR_COUT << COLOR_END << " Initialized LiDAR Odometry subscriber with topic: " << subLioOdometry_.getTopic() << std::endl;
  }

  // Wheel Odometry Between
  if (useWheelOdometryBetweenFlag_) {
    subWheelOdometryBetween_ = privateNode.subscribe<nav_msgs::Odometry>(
        "/wheel_odometry_topic", ROS_QUEUE_SIZE, &SmbEstimator::wheelOdometryPoseCallback_, this, ros::TransportHints().tcpNoDelay());
    REGULAR_COUT << COLOR_END << " Initialized Wheel Odometry subscriber with topic: " << subWheelOdometryBetween_.getTopic() << std::endl;
  }

  // Wheel Linear Velocities
  if (useWheelLinearVelocitiesFlag_) {
    subWheelLinearVelocities_ = privateNode.subscribe<std_msgs::Float64MultiArray>(
        "/wheel_velocities_topic", ROS_QUEUE_SIZE, &SmbEstimator::wheelLinearVelocitiesCallback_, this, ros::TransportHints().tcpNoDelay());
    REGULAR_COUT << COLOR_END << " Initialized Wheel Linear Velocities subscriber with topic: " << subWheelLinearVelocities_.getTopic()
                 << std::endl;
  }

  // VIO
  if (useVioOdometryFlag_) {
    subVioOdometry_ = privateNode.subscribe<nav_msgs::Odometry>("/vio_odometry_topic", ROS_QUEUE_SIZE, &SmbEstimator::vioOdometryCallback_,
                                                                this, ros::TransportHints().tcpNoDelay());
    REGULAR_COUT << COLOR_END << " Initialized VIO Odometry subscriber with topic: " << subVioOdometry_.getTopic() << std::endl;
  }
}

void SmbEstimator::initializeMessages(ros::NodeHandle& privateNode) {
  // Path
  measLio_mapImuPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  measVio_mapImuPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
}

void SmbEstimator::initializeServices(ros::NodeHandle& privateNode) {
  // Nothing for now
}

void SmbEstimator::imuCallback(const sensor_msgs::Imu::ConstPtr& imuPtr) {
  // Check whether any of the measurements is available, otherwise do pure imu integration
  if (graph_msf::GraphMsf::areRollAndPitchInited() && !graph_msf::GraphMsf::areYawAndPositionInited() && !useLioOdometryFlag_ &&
      !useWheelOdometryBetweenFlag_ && !useWheelLinearVelocitiesFlag_ && !useVioOdometryFlag_) {
    // Initialization
    REGULAR_COUT << RED_START << " IMU callback is setting global yaw and position, as no other odometry is available. Initializing..."
                 << COLOR_END << std::endl;
    // Create dummy measurement for initialization
    graph_msf::UnaryMeasurementXD<Eigen::Isometry3d, 6> unary6DMeasurement(
        "IMU_init_6D", int(graphConfigPtr_->imuRate_), staticTransformsPtr_->getImuFrame(),
        staticTransformsPtr_->getImuFrame() + sensorFrameCorrectedNameId, graph_msf::RobustNorm::None(), imuPtr->header.stamp.toSec(), 1.0,
        Eigen::Isometry3d::Identity(), Eigen::MatrixXd::Identity(6, 1));
    // Initialize
    graph_msf::GraphMsf::initYawAndPosition(unary6DMeasurement);
    REGULAR_COUT << RED_START << " ...initialized yaw and position to identity." << COLOR_END << std::endl;
    // Pretend that we received first measurement --> In order to allow for optimization
    graph_msf::GraphMsf::pretendFirstMeasurementReceived();
  }

  // Super class
  graph_msf::GraphMsfRos::imuCallback(imuPtr);
}

void SmbEstimator::lidarOdometryCallback_(const nav_msgs::Odometry::ConstPtr& odomLidarPtr) {
  // Static members
  static int lidarOdometryCallbackCounter__ = -1;

  // Counter
  ++lidarOdometryCallbackCounter__;

  Eigen::Isometry3d lio_T_M_Lk;
  graph_msf::odomMsgToEigen(*odomLidarPtr, lio_T_M_Lk.matrix());

  // Transform to IMU frame
  double lidarOdometryTimeK = odomLidarPtr->header.stamp.toSec();

  // Frame Name
  const std::string& lioOdometryFrame = dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())->getLioOdometryFrame();  // alias

  // Measurement
  graph_msf::UnaryMeasurementXDAbsolute<Eigen::Isometry3d, 6> unary6DMeasurement(
      "Lidar_unary_6D", int(lioOdometryRate_), lioOdometryFrame, lioOdometryFrame + sensorFrameCorrectedNameId,
      graph_msf::RobustNorm::None(), lidarOdometryTimeK, 1.0, lio_T_M_Lk, lioPoseUnaryNoise_, odomLidarPtr->header.frame_id,
      staticTransformsPtr_->getWorldFrame(), initialSe3AlignmentNoise_, lioSe3AlignmentRandomWalk_);

  // Add measurement or initialize
  if (lidarOdometryCallbackCounter__ <= 2) {
    return;
  } else if (areYawAndPositionInited()) {  // Already initialized --> unary factor
    this->addUnaryPose3AbsoluteMeasurement(unary6DMeasurement);
  } else {  // Initializing
    REGULAR_COUT << GREEN_START << " LiDAR odometry callback is setting global yaw, as it was not set so far." << COLOR_END << std::endl;
    this->initYawAndPosition(unary6DMeasurement);
  }

  // Visualization ----------------------------
  // Add to path message
  addToPathMsg(measLio_mapImuPathPtr_, odomLidarPtr->header.frame_id, odomLidarPtr->header.stamp,
               (lio_T_M_Lk * staticTransformsPtr_
                                 ->rv_T_frame1_frame2(dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())->getLioOdometryFrame(),
                                                      staticTransformsPtr_->getImuFrame())
                                 .matrix())
                   .block<3, 1>(0, 3),
               graphConfigPtr_->imuBufferLength_ * 4);

  // Publish Path
  pubMeasMapLioPath_.publish(measLio_mapImuPathPtr_);
}

void SmbEstimator::wheelOdometryPoseCallback_(const nav_msgs::Odometry::ConstPtr& wheelOdometryKPtr) {
  if (!areRollAndPitchInited()) {
    return;
  }

  // Counter
  ++wheelOdometryCallbackCounter_;

  // Eigen Type
  Eigen::Isometry3d T_O_Bw_k;
  graph_msf::odomMsgToEigen(*wheelOdometryKPtr, T_O_Bw_k.matrix());
  double wheelOdometryTimeK = wheelOdometryKPtr->header.stamp.toSec();

  // At start
  if (wheelOdometryCallbackCounter_ == 0) {
    T_O_Bw_km1_ = T_O_Bw_k;
    wheelOdometryTimeKm1_ = wheelOdometryTimeK;
    return;
  }

  // Frame name
  const std::string& wheelOdometryFrame =
      dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())->getWheelOdometryBetweenFrame();  // alias

  // State Machine
  if (!areYawAndPositionInited()) {
    // If no LIO, then initialize the graph here
    if (!useLioOdometryFlag_) {
      REGULAR_COUT << GREEN_START << " Wheel odometry callback is setting global yaw and position, as lio is all set to false." << COLOR_END
                   << std::endl;
      graph_msf::UnaryMeasurementXD<Eigen::Isometry3d, 6> unary6DMeasurement(
          "Lidar_unary_6D", int(wheelOdometryBetweenRate_), wheelOdometryFrame, wheelOdometryFrame + sensorFrameCorrectedNameId,
          graph_msf::RobustNorm::None(), wheelOdometryTimeK, 1.0, Eigen::Isometry3d::Identity(), Eigen::MatrixXd::Identity(6, 1));
      graph_msf::GraphMsf::initYawAndPosition(unary6DMeasurement);
      REGULAR_COUT << " Initialized yaw and position to identity in the wheel odometry callback, as lio and vio are all set to false."
                   << std::endl;
    }
  } else {
    // Only add every 5th measurement
    int measurementRate = static_cast<int>(wheelOdometryBetweenRate_ / 5);
    // Check
    if (wheelOdometryCallbackCounter_ % 5 == 0 && wheelOdometryCallbackCounter_ > 0) {
      // Compute Delta
      Eigen::Isometry3d T_Bkm1_Bk = T_O_Bw_km1_.inverse() * T_O_Bw_k;
      // Create measurement
      graph_msf::BinaryMeasurementXD<Eigen::Isometry3d, 6> delta6DMeasurement(
          "Wheel_odometry_6D", measurementRate, wheelOdometryFrame, wheelOdometryFrame + sensorFrameCorrectedNameId,
          graph_msf::RobustNorm::Tukey(1.0), wheelOdometryTimeKm1_, wheelOdometryTimeK, T_Bkm1_Bk, wheelPoseBetweenNoise_);
      // Add to graph
      this->addBinaryPose3Measurement(delta6DMeasurement);

      // Prepare for next iteration
      T_O_Bw_km1_ = T_O_Bw_k;
      wheelOdometryTimeKm1_ = wheelOdometryTimeK;
    }
  }
}

void SmbEstimator::wheelLinearVelocitiesCallback_(const std_msgs::Float64MultiArray::ConstPtr& wheelsSpeedsPtr) {
  if (!areRollAndPitchInited()) {
    return;
  }

  // Get the 3 values
  const double timeK = wheelsSpeedsPtr->data[0];
  const double leftWheelSpeedRps = wheelsSpeedsPtr->data[1];
  const double rightWheelSpeedRps = wheelsSpeedsPtr->data[2];
  const double leftWheelSpeedMs = leftWheelSpeedRps * wheelRadiusMeter_;
  const double rightWheelSpeedMs = rightWheelSpeedRps * wheelRadiusMeter_;

  // Print
  if (graphConfigPtr_->verboseLevel_ > 1) {
    REGULAR_COUT << " Wheel linear velocities callback: " << std::setprecision(14) << timeK << " " << leftWheelSpeedMs << " [m/s] "
                 << rightWheelSpeedMs << " [m/s]" << std::endl;
  }

  // Frame name
  const std::string& wheelLinearVelocityLeftFrame =
      dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())->getWheelLinearVelocityLeftFrame();  // alias
  const std::string& wheelLinearVelocityRightFrame =
      dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())->getWheelLinearVelocityRightFrame();  // alias

  // State Machine
  if (!areYawAndPositionInited()) {
    // If no LIO, then initialize the graph here
    if (!useLioOdometryFlag_ && !useWheelOdometryBetweenFlag_) {
      REGULAR_COUT << GREEN_START << " Wheel linear velocities callback is setting global yaw and position, as lio is all set to false."
                   << COLOR_END << std::endl;
      graph_msf::UnaryMeasurementXD<Eigen::Isometry3d, 6> unary6DMeasurement(
          "Lidar_unary_6D", int(wheelLinearVelocitiesRate_), wheelLinearVelocityLeftFrame,
          wheelLinearVelocityLeftFrame + sensorFrameCorrectedNameId, graph_msf::RobustNorm::None(), timeK, 1.0,
          Eigen::Isometry3d::Identity(), Eigen::MatrixXd::Identity(6, 1));
      graph_msf::GraphMsf::initYawAndPosition(unary6DMeasurement);
      REGULAR_COUT
          << " Initialized yaw and position to identity in the wheel linear velocities callback, as lio and vio are all set to false."
          << std::endl;
    }
  } else {
    // Left Wheel
    graph_msf::UnaryMeasurementXD<Eigen::Vector3d, 3> leftWheelLinearVelocityMeasurement(
        "Wheel_linear_velocity_left", int(wheelLinearVelocitiesRate_), wheelLinearVelocityLeftFrame,
        wheelLinearVelocityLeftFrame + sensorFrameCorrectedNameId, graph_msf::RobustNorm::Tukey(1.0), timeK, 1.0,
        Eigen::Vector3d(leftWheelSpeedMs, 0.0, 0.0), wheelLinearVelocitiesNoise_);
    this->addUnaryVelocity3LocalMeasurement(leftWheelLinearVelocityMeasurement);

    // Right Wheel
    graph_msf::UnaryMeasurementXD<Eigen::Vector3d, 3> rightWheelLinearVelocityMeasurement(
        "Wheel_linear_velocity_right", int(wheelLinearVelocitiesRate_), wheelLinearVelocityRightFrame,
        wheelLinearVelocityRightFrame + sensorFrameCorrectedNameId, graph_msf::RobustNorm::Tukey(1.0), timeK, 1.0,
        Eigen::Vector3d(rightWheelSpeedMs, 0.0, 0.0), wheelLinearVelocitiesNoise_);
    this->addUnaryVelocity3LocalMeasurement(rightWheelLinearVelocityMeasurement);
  }
}

void SmbEstimator::vioOdometryCallback_(const nav_msgs::Odometry::ConstPtr& vioOdomPtr) {
  std::cout << "VIO odometry not yet stable enough for usage, disable flag." << std::endl;

  // Extract
  Eigen::Isometry3d vio_T_M_Ck;
  graph_msf::odomMsgToEigen(*vioOdomPtr, vio_T_M_Ck.matrix());

  // Visualization ----------------------------
  // Add to path message
  addToPathMsg(measVio_mapImuPathPtr_, vioOdomPtr->header.frame_id, vioOdomPtr->header.stamp,
               (vio_T_M_Ck * staticTransformsPtr_
                                 ->rv_T_frame1_frame2(dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())->getVioOdometryFrame(),
                                                      staticTransformsPtr_->getImuFrame())
                                 .matrix())
                   .block<3, 1>(0, 3),
               graphConfigPtr_->imuBufferLength_ * 4 * 10);

  // Publish Path
  pubMeasMapVioPath_.publish(measVio_mapImuPathPtr_);
}

}  // namespace smb_se