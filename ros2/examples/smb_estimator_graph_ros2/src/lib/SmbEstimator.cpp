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
#include "graph_msf/measurements/BinaryMeasurementXD.h"
#include "graph_msf/measurements/UnaryMeasurementXD.h"
#include "graph_msf_ros2/util/conversions.h"
#include "smb_estimator_graph_ros2/constants.h"

namespace smb_se {

SmbEstimator::SmbEstimator(std::shared_ptr<rclcpp::Node>& node) : graph_msf::GraphMsfRos2(node) {
  REGULAR_COUT << GREEN_START << " SmbEstimator-Constructor called." << COLOR_END << std::endl;

  staticTransformsPtr_ = std::make_shared<SmbStaticTransforms>(node_);
  // Call setup after declaring parameters
  SmbEstimator::setup();
}

void SmbEstimator::setup() {
  REGULAR_COUT << GREEN_START << " SmbEstimator-Setup called." << COLOR_END << std::endl;

  // Boolean flags
  node_->declare_parameter("sensor_params.useLioOdometry", false);
  node_->declare_parameter("sensor_params.useWheelOdometryBetween", false);
  node_->declare_parameter("sensor_params.useWheelLinearVelocities", false);
  node_->declare_parameter("sensor_params.useVioOdometry", false);

  // Sensor parameters (int)
  node_->declare_parameter("sensor_params.lioOdometryRate", 0);
  node_->declare_parameter("sensor_params.wheelOdometryBetweenRate", 0);
  node_->declare_parameter("sensor_params.wheelLinearVelocitiesRate", 0);
  node_->declare_parameter("sensor_params.vioOdometryRate", 0);

  // Alignment parameters (vector of double)
  node_->declare_parameter("alignment_params.initialSe3AlignmentNoiseDensity", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  node_->declare_parameter("alignment_params.lioSe3AlignmentRandomWalk", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

  // Noise parameters (vectors of double)
  node_->declare_parameter("noise_params.lioPoseUnaryNoiseDensity", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  node_->declare_parameter("noise_params.wheelPoseBetweenNoiseDensity", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  node_->declare_parameter("noise_params.wheelLinearVelocitiesNoiseDensity", std::vector<double>{0.0, 0.0, 0.0});
  node_->declare_parameter("noise_params.vioPoseBetweenNoiseDensity", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

  // Extrinsic frames (string)
  node_->declare_parameter("extrinsics.lidarOdometryFrame", std::string(""));
  node_->declare_parameter("extrinsics.wheelOdometryBetweenFrame", std::string(""));
  node_->declare_parameter("extrinsics.wheelLinearVelocityLeftFrame", std::string(""));
  node_->declare_parameter("extrinsics.wheelLinearVelocityRightFrame", std::string(""));
  node_->declare_parameter("extrinsics.vioOdometryFrame", std::string(""));

  // Wheel Radius (double)
  node_->declare_parameter("sensor_params.wheelRadius", 0.0);

  SmbEstimator::readParams();

  // Initialize ROS 2 publishers and subscribers
  SmbEstimator::initializePublishers();
  SmbEstimator::initializeSubscribers();
  SmbEstimator::initializeMessages();
  SmbEstimator::initializeServices();

  GraphMsfRos2::setup(staticTransformsPtr_);

  // Transforms
  staticTransformsPtr_->findTransformations();

  REGULAR_COUT << GREEN_START << " Set up successfully." << COLOR_END << std::endl;
}

void SmbEstimator::initializePublishers() {
  pubMeasMapLioPath_ = node_->create_publisher<nav_msgs::msg::Path>("/graph_msf/measLiDAR_path_map_imu", ROS_QUEUE_SIZE);
  pubMeasMapVioPath_ = node_->create_publisher<nav_msgs::msg::Path>("/graph_msf/measVIO_path_map_imu", ROS_QUEUE_SIZE);
}

void SmbEstimator::initializeSubscribers() {
  if (useLioOdometryFlag_) {
    subLioOdometry_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/lidar_odometry_topic", ROS_QUEUE_SIZE, std::bind(&SmbEstimator::lidarOdometryCallback_, this, std::placeholders::_1));
    REGULAR_COUT << COLOR_END << " Initialized LiDAR Odometry subscriber with topic: /lidar_odometry_topic" << std::endl;
  }

  if (useWheelOdometryBetweenFlag_) {
    subWheelOdometryBetween_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/wheel_odometry_topic", ROS_QUEUE_SIZE, std::bind(&SmbEstimator::wheelOdometryPoseCallback_, this, std::placeholders::_1));
    REGULAR_COUT << COLOR_END << " Initialized Wheel Odometry subscriber with topic: /wheel_odometry_topic" << std::endl;
  }

  if (useWheelLinearVelocitiesFlag_) {
    subWheelLinearVelocities_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/wheel_velocities_topic", ROS_QUEUE_SIZE, std::bind(&SmbEstimator::wheelLinearVelocitiesCallback_, this, std::placeholders::_1));
    REGULAR_COUT << COLOR_END << " Initialized Wheel Linear Velocities subscriber with topic: /wheel_velocities_topic" << std::endl;
  }

  if (useVioOdometryFlag_) {
    subVioOdometry_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/vio_odometry_topic", ROS_QUEUE_SIZE, std::bind(&SmbEstimator::vioOdometryCallback_, this, std::placeholders::_1));
    REGULAR_COUT << COLOR_END << " Initialized VIO Odometry subscriber with topic: /vio_odometry_topic" << std::endl;
  }
}

void SmbEstimator::initializeMessages() {
  measLio_mapImuPathPtr_ = std::make_shared<nav_msgs::msg::Path>();
  measVio_mapImuPathPtr_ = std::make_shared<nav_msgs::msg::Path>();
}

void SmbEstimator::initializeServices() {
  // Nothing for now
}

void SmbEstimator::imuCallback(const sensor_msgs::msg::Imu::SharedPtr imuPtr) {
  if (graph_msf::GraphMsf::areRollAndPitchInited() && !graph_msf::GraphMsf::areYawAndPositionInited() && !useLioOdometryFlag_ &&
      !useWheelOdometryBetweenFlag_ && !useWheelLinearVelocitiesFlag_ && !useVioOdometryFlag_) {
    REGULAR_COUT << RED_START << " IMU callback is setting global yaw and position, as no other odometry is available. Initializing..."
                 << COLOR_END << std::endl;

    graph_msf::UnaryMeasurementXD<Eigen::Isometry3d, 6> unary6DMeasurement(
        "IMU_init_6D", int(graphConfigPtr_->imuRate_), staticTransformsPtr_->getImuFrame(),
        staticTransformsPtr_->getImuFrame() + sensorFrameCorrectedNameId, graph_msf::RobustNorm::None(),
        imuPtr->header.stamp.sec + imuPtr->header.stamp.nanosec * 1e-9, 1.0, Eigen::Isometry3d::Identity(),
        Eigen::MatrixXd::Identity(6, 1));

    graph_msf::GraphMsf::initYawAndPosition(unary6DMeasurement);
    graph_msf::GraphMsf::pretendFirstMeasurementReceived();
  }

  graph_msf::GraphMsfRos2::imuCallback(imuPtr);
}

void SmbEstimator::lidarOdometryCallback_(const nav_msgs::msg::Odometry::ConstSharedPtr& odomLidarPtr) {
  static int lidarOdometryCallbackCounter__ = -1;
  ++lidarOdometryCallbackCounter__;

  Eigen::Isometry3d lio_T_M_Lk;
  graph_msf::odomMsgToEigen(*odomLidarPtr, lio_T_M_Lk.matrix());
  double lidarOdometryTimeK = odomLidarPtr->header.stamp.sec + odomLidarPtr->header.stamp.nanosec * 1e-9;

  const std::string& lioOdometryFrame = dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())->getLioOdometryFrame();

  graph_msf::UnaryMeasurementXDAbsolute<Eigen::Isometry3d, 6> unary6DMeasurement(
      "Lidar_unary_6D", int(lioOdometryRate_), lioOdometryFrame, lioOdometryFrame + sensorFrameCorrectedNameId,
      graph_msf::RobustNorm::None(), lidarOdometryTimeK, 1.0, lio_T_M_Lk, lioPoseUnaryNoise_, odomLidarPtr->header.frame_id,
      staticTransformsPtr_->getWorldFrame(), initialSe3AlignmentNoise_, lioSe3AlignmentRandomWalk_);

  if (lidarOdometryCallbackCounter__ <= 2) {
    return;
  } else if (areYawAndPositionInited()) {
    this->addUnaryPose3AbsoluteMeasurement(unary6DMeasurement);
  } else {
    this->initYawAndPosition(unary6DMeasurement);
  }

  addToPathMsg(measLio_mapImuPathPtr_, odomLidarPtr->header.frame_id, odomLidarPtr->header.stamp,
               (lio_T_M_Lk * staticTransformsPtr_->rv_T_frame1_frame2(lioOdometryFrame, staticTransformsPtr_->getImuFrame()).matrix())
                   .block<3, 1>(0, 3),
               graphConfigPtr_->imuBufferLength_ * 4);

  pubMeasMapLioPath_->publish(*measLio_mapImuPathPtr_);
}

void SmbEstimator::wheelOdometryPoseCallback_(const nav_msgs::msg::Odometry::ConstSharedPtr& wheelOdometryKPtr) {
  if (!areRollAndPitchInited()) {
    return;
  }

  ++wheelOdometryCallbackCounter_;

  Eigen::Isometry3d T_O_Bw_k;
  graph_msf::odomMsgToEigen(*wheelOdometryKPtr, T_O_Bw_k.matrix());
  double wheelOdometryTimeK = wheelOdometryKPtr->header.stamp.sec + wheelOdometryKPtr->header.stamp.nanosec * 1e-9;

  if (wheelOdometryCallbackCounter_ == 0) {
    T_O_Bw_km1_ = T_O_Bw_k;
    wheelOdometryTimeKm1_ = wheelOdometryTimeK;
    return;
  }

  const std::string& wheelOdometryFrame = dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())->getWheelOdometryBetweenFrame();

  if (!areYawAndPositionInited()) {
    if (!useLioOdometryFlag_) {
      graph_msf::UnaryMeasurementXD<Eigen::Isometry3d, 6> unary6DMeasurement(
          "Lidar_unary_6D", int(wheelOdometryBetweenRate_), wheelOdometryFrame, wheelOdometryFrame + sensorFrameCorrectedNameId,
          graph_msf::RobustNorm::None(), wheelOdometryTimeK, 1.0, Eigen::Isometry3d::Identity(), Eigen::MatrixXd::Identity(6, 1));
      graph_msf::GraphMsf::initYawAndPosition(unary6DMeasurement);
    }
  } else if (wheelOdometryCallbackCounter_ % 5 == 0 && wheelOdometryCallbackCounter_ > 0) {
    Eigen::Isometry3d T_Bkm1_Bk = T_O_Bw_km1_.inverse() * T_O_Bw_k;
    graph_msf::BinaryMeasurementXD<Eigen::Isometry3d, 6> delta6DMeasurement(
        "Wheel_odometry_6D", int(wheelOdometryBetweenRate_ / 5), wheelOdometryFrame, wheelOdometryFrame + sensorFrameCorrectedNameId,
        graph_msf::RobustNorm::Tukey(1.0), wheelOdometryTimeKm1_, wheelOdometryTimeK, T_Bkm1_Bk, wheelPoseBetweenNoise_);
    this->addBinaryPose3Measurement(delta6DMeasurement);

    T_O_Bw_km1_ = T_O_Bw_k;
    wheelOdometryTimeKm1_ = wheelOdometryTimeK;
  }
}

void SmbEstimator::wheelLinearVelocitiesCallback_(const std_msgs::msg::Float64MultiArray::ConstSharedPtr& wheelsSpeedsPtr) {
  if (!areRollAndPitchInited()) {
    return;
  }

  const double timeK = wheelsSpeedsPtr->data[0];
  const double leftWheelSpeedRps = wheelsSpeedsPtr->data[1];
  const double rightWheelSpeedRps = wheelsSpeedsPtr->data[2];
  const double leftWheelSpeedMs = leftWheelSpeedRps * wheelRadiusMeter_;
  const double rightWheelSpeedMs = rightWheelSpeedRps * wheelRadiusMeter_;

  const std::string& wheelLinearVelocityLeftFrame =
      dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())->getWheelLinearVelocityLeftFrame();
  const std::string& wheelLinearVelocityRightFrame =
      dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())->getWheelLinearVelocityRightFrame();

  if (!areYawAndPositionInited()) {
    if (!useLioOdometryFlag_ && !useWheelOdometryBetweenFlag_) {
      graph_msf::UnaryMeasurementXD<Eigen::Isometry3d, 6> unary6DMeasurement(
          "Lidar_unary_6D", int(wheelLinearVelocitiesRate_), wheelLinearVelocityLeftFrame,
          wheelLinearVelocityLeftFrame + sensorFrameCorrectedNameId, graph_msf::RobustNorm::None(), timeK, 1.0,
          Eigen::Isometry3d::Identity(), Eigen::MatrixXd::Identity(6, 1));
      graph_msf::GraphMsf::initYawAndPosition(unary6DMeasurement);
    }
  } else {
    graph_msf::UnaryMeasurementXD<Eigen::Vector3d, 3> leftWheelLinearVelocityMeasurement(
        "Wheel_linear_velocity_left", int(wheelLinearVelocitiesRate_), wheelLinearVelocityLeftFrame,
        wheelLinearVelocityLeftFrame + sensorFrameCorrectedNameId, graph_msf::RobustNorm::Tukey(1.0), timeK, 1.0,
        Eigen::Vector3d(leftWheelSpeedMs, 0.0, 0.0), wheelLinearVelocitiesNoise_);
    this->addUnaryVelocity3LocalMeasurement(leftWheelLinearVelocityMeasurement);

    graph_msf::UnaryMeasurementXD<Eigen::Vector3d, 3> rightWheelLinearVelocityMeasurement(
        "Wheel_linear_velocity_right", int(wheelLinearVelocitiesRate_), wheelLinearVelocityRightFrame,
        wheelLinearVelocityRightFrame + sensorFrameCorrectedNameId, graph_msf::RobustNorm::Tukey(1.0), timeK, 1.0,
        Eigen::Vector3d(rightWheelSpeedMs, 0.0, 0.0), wheelLinearVelocitiesNoise_);
    this->addUnaryVelocity3LocalMeasurement(rightWheelLinearVelocityMeasurement);
  }
}

void SmbEstimator::vioOdometryCallback_(const nav_msgs::msg::Odometry::ConstSharedPtr& vioOdomPtr) {
  std::cout << "VIO odometry not yet stable enough for usage, disable flag." << std::endl;

  Eigen::Isometry3d vio_T_M_Ck;
  graph_msf::odomMsgToEigen(*vioOdomPtr, vio_T_M_Ck.matrix());

  addToPathMsg(measVio_mapImuPathPtr_, vioOdomPtr->header.frame_id, vioOdomPtr->header.stamp,
               (vio_T_M_Ck * staticTransformsPtr_
                                 ->rv_T_frame1_frame2(dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())->getVioOdometryFrame(),
                                                      staticTransformsPtr_->getImuFrame())
                                 .matrix())
                   .block<3, 1>(0, 3),
               graphConfigPtr_->imuBufferLength_ * 4 * 10);

  pubMeasMapVioPath_->publish(*measVio_mapImuPathPtr_);
}

}  // namespace smb_se