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
#include "holistic_fusion/measurements/BinaryMeasurementXD.h"
#include "holistic_fusion/measurements/UnaryMeasurementXD.h"
#include "holistic_fusion_ros2/util/conversions.h"
#include "smb_estimator_graph_ros2/constants.h"

namespace smb_se {

SmbEstimator::SmbEstimator(const std::string& nodeName, const rclcpp::NodeOptions& options) : holistic_fusion::HolisticFusionRos2(nodeName, options) {
  REGULAR_COUT << GREEN_START << " SmbEstimator-Constructor called." << COLOR_END << std::endl;
}

void SmbEstimator::setup() {
  REGULAR_COUT << GREEN_START << " SmbEstimator-Setup called." << COLOR_END << std::endl;

  // Boolean flags
  this->declare_parameter("sensor_params.useLioOdometry", false);
  this->declare_parameter("sensor_params.useWheelOdometryBetween", false);
  this->declare_parameter("sensor_params.useWheelLinearVelocities", false);
  this->declare_parameter("sensor_params.useVioOdometry", false);

  // Sensor parameters (int)
  this->declare_parameter("sensor_params.lioOdometryRate", 0);
  this->declare_parameter("sensor_params.wheelOdometryBetweenRate", 0);
  this->declare_parameter("sensor_params.wheelLinearVelocitiesRate", 0);
  this->declare_parameter("sensor_params.vioOdometryRate", 0);

  // Alignment parameters (vector of double)
  this->declare_parameter("alignment_params.initialSe3AlignmentStdDev", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  this->declare_parameter("alignment_params.lioSe3AlignmentRandomWalk", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

  // Noise parameters (vectors of double)
  this->declare_parameter("noise_params.lioPoseUnaryStdDev", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  this->declare_parameter("noise_params.wheelPoseBetweenNoiseDensity", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  this->declare_parameter("noise_params.wheelLinearVelocitiesNoiseDensity", std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter("noise_params.vioPoseBetweenNoiseDensity", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

  // Extrinsic frames (string)
  this->declare_parameter("extrinsics.lidarOdometryFrame", std::string(""));
  this->declare_parameter("extrinsics.wheelOdometryBetweenFrame", std::string(""));
  this->declare_parameter("extrinsics.wheelLinearVelocityLeftFrame", std::string(""));
  this->declare_parameter("extrinsics.wheelLinearVelocityRightFrame", std::string(""));
  this->declare_parameter("extrinsics.vioOdometryFrame", std::string(""));

  // Wheel Radius (double)
  this->declare_parameter("sensor_params.wheelRadius", 0.0);

  // Create SmbStaticTransforms
  staticTransformsPtr_ = std::make_shared<SmbStaticTransforms>(shared_from_this());

  SmbEstimator::readParams();

  // Initialize ROS 2 publishers and subscribers
  SmbEstimator::initializePublishers();
  SmbEstimator::initializeSubscribers();
  SmbEstimator::initializeMessages();
  SmbEstimator::initializeServices();

  HolisticFusionRos2::setup(staticTransformsPtr_);

  // Transforms --> query until returns true
  bool foundTransforms = false;
  while (!foundTransforms) {
    foundTransforms = staticTransformsPtr_->findTransformations();
    // Sleep for 0.1 seconds to avoid busy waiting
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  REGULAR_COUT << GREEN_START << " Set up successfully." << COLOR_END << std::endl;
}

void SmbEstimator::initializePublishers() {
  pubMeasMapLioPath_ = this->create_publisher<nav_msgs::msg::Path>("/holistic_fusion/measLiDAR_path_map_imu", ROS_QUEUE_SIZE);
  pubMeasMapVioPath_ = this->create_publisher<nav_msgs::msg::Path>("/holistic_fusion/measVIO_path_map_imu", ROS_QUEUE_SIZE);
}

void SmbEstimator::initializeSubscribers() {
  if (useLioOdometryFlag_) {
    subLioOdometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/lidar_odometry_topic", ROS_QUEUE_SIZE, std::bind(&SmbEstimator::lidarOdometryCallback_, this, std::placeholders::_1));
    REGULAR_COUT << COLOR_END << " Initialized LiDAR Odometry subscriber with topic: /lidar_odometry_topic" << std::endl;
  }

  if (useWheelOdometryBetweenFlag_) {
    subWheelOdometryBetween_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/wheel_odometry_topic", ROS_QUEUE_SIZE, std::bind(&SmbEstimator::wheelOdometryPoseCallback_, this, std::placeholders::_1));
    REGULAR_COUT << COLOR_END << " Initialized Wheel Odometry subscriber with topic: /wheel_odometry_topic" << std::endl;
  }

  if (useWheelLinearVelocitiesFlag_) {
    subWheelLinearVelocities_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/wheel_velocities_topic", ROS_QUEUE_SIZE, std::bind(&SmbEstimator::wheelLinearVelocitiesCallback_, this, std::placeholders::_1));
    REGULAR_COUT << COLOR_END << " Initialized Wheel Linear Velocities subscriber with topic: /wheel_velocities_topic" << std::endl;
  }

  if (useVioOdometryFlag_) {
    subVioOdometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
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
  const rclcpp::Time new_imu_timestamp{imuPtr->header.stamp};

  if (holistic_fusion::HolisticFusion::areRollAndPitchInited() && !holistic_fusion::HolisticFusion::areYawAndPositionInited() && !useLioOdometryFlag_ &&
      !useWheelOdometryBetweenFlag_ && !useWheelLinearVelocitiesFlag_ && !useVioOdometryFlag_) {
    REGULAR_COUT << RED_START << " IMU callback is setting global yaw and position, as no other odometry is available. Initializing..."
                 << COLOR_END << std::endl;

    holistic_fusion::HolisticFusion::initHeadingAndPositionAtStart();
    holistic_fusion::HolisticFusion::pretendFirstMeasurementReceived();
  }
  // Remove if norm is larger than 100
  const double angular_velocity_norm = std::sqrt(imuPtr->angular_velocity.x * imuPtr->angular_velocity.x +
                imuPtr->angular_velocity.y * imuPtr->angular_velocity.y +
                imuPtr->angular_velocity.z * imuPtr->angular_velocity.z);
  const double linear_acceleration_norm = std::sqrt(imuPtr->linear_acceleration.x * imuPtr->linear_acceleration.x +
                imuPtr->linear_acceleration.y * imuPtr->linear_acceleration.y +
                imuPtr->linear_acceleration.z * imuPtr->linear_acceleration.z);
  if (angular_velocity_norm > 10) {
    ++num_imu_errors_;
    REGULAR_COUT << RED_START << " IMU angular velocity is larger than 10 rad/s, skipping this measurement. Total error count = " << num_imu_errors_ << COLOR_END << std::endl;
    return;
  } else if (linear_acceleration_norm > 100.0) {
    ++num_imu_errors_;
    REGULAR_COUT << RED_START << " IMU linear acceleration norm is larger than 100 m/s^2, skipping this measurement. Total error count = " << num_imu_errors_ << COLOR_END << std::endl;
    return;
  }
  // Check timestamps strictly increase
  if (new_imu_timestamp == last_imu_timestamp_) {
    ++num_imu_errors_;
    REGULAR_COUT << RED_START << " IMU timestamp " << new_imu_timestamp.seconds() << " was duplicated, skipping this measurement. Total error count = " << num_imu_errors_ << COLOR_END << std::endl;
    return;
  } else if (new_imu_timestamp < last_imu_timestamp_) {
    ++num_imu_errors_;
    REGULAR_COUT << RED_START << " IMU timestamp " << new_imu_timestamp.seconds() << " was before last included IMU measurement "
        " at time" << last_imu_timestamp_.seconds() << ", skipping this measurement. Total error count = " << num_imu_errors_ << COLOR_END << std::endl;
    return;
  }
  last_imu_timestamp_ = new_imu_timestamp;

  holistic_fusion::HolisticFusionRos2::imuCallback(imuPtr);
}

void SmbEstimator::lidarOdometryCallback_(const nav_msgs::msg::Odometry::ConstSharedPtr& odomLidarPtr) {
  static int lidarOdometryCallbackCounter__ = -1;
  static double lastLidarOdometryTimeK_ = 0.0;
  static constexpr double lioOdometryRate_ = 10.0;  // Hz

  // Timestamp
  double lidarOdometryTimeK = odomLidarPtr->header.stamp.sec + odomLidarPtr->header.stamp.nanosec * 1e-9;

  // Check whether the callback rate is not exceeded
  if (lidarOdometryCallbackCounter__ >= 0 && lidarOdometryTimeK - lastLidarOdometryTimeK_ < (1.0 / lioOdometryRate_)) {
    return;  // Skip this callback if the rate is exceeded
  } else {
    lastLidarOdometryTimeK_ = lidarOdometryTimeK;  // Update the last timestamp
  }

  // Update the callback counter
  ++lidarOdometryCallbackCounter__;

  Eigen::Isometry3d lio_T_M_Lk;
  holistic_fusion::odomMsgToEigen(*odomLidarPtr, lio_T_M_Lk.matrix());
  

  const std::string& lioOdometryFrame = dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())->getLioOdometryFrame();

  holistic_fusion::UnaryMeasurementXDAbsolute<Eigen::Isometry3d, 6> unary6DMeasurement(
      "Lidar_unary_6D", int(lioOdometryRate_), lioOdometryFrame, lioOdometryFrame + sensorFrameCorrectedNameId,
      holistic_fusion::RobustNorm::Huber(3.0), lidarOdometryTimeK, 1.0, lio_T_M_Lk, lioPoseUnaryNoise_, odomLidarPtr->header.frame_id,
      staticTransformsPtr_->getWorldFrame(), initialSe3AlignmentNoise_, lioSe3AlignmentRandomWalk_);

  if (lidarOdometryCallbackCounter__ <= 2) {
    return;
  } else if (areYawAndPositionInited()) {
    this->addUnaryPose3AbsoluteMeasurement(unary6DMeasurement);
  } else {
    this->initHeadingAndPosition(unary6DMeasurement);
  }

  addToPathMsg(measLio_mapImuPathPtr_, odomLidarPtr->header.frame_id  + referenceFrameAlignedNameId, odomLidarPtr->header.stamp,
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
  holistic_fusion::odomMsgToEigen(*wheelOdometryKPtr, T_O_Bw_k.matrix());
  double wheelOdometryTimeK = wheelOdometryKPtr->header.stamp.sec + wheelOdometryKPtr->header.stamp.nanosec * 1e-9;

  if (wheelOdometryCallbackCounter_ == 0) {
    T_O_Bw_km1_ = T_O_Bw_k;
    wheelOdometryTimeKm1_ = wheelOdometryTimeK;
    return;
  }

  const std::string& wheelOdometryFrame = dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())->getWheelOdometryBetweenFrame();

  if (!areYawAndPositionInited()) {
    if (!useLioOdometryFlag_) {
      holistic_fusion::HolisticFusion::initHeadingAndPositionAtStart();
    }
  } else if (wheelOdometryCallbackCounter_ % 5 == 0 && wheelOdometryCallbackCounter_ > 0) {
    Eigen::Isometry3d T_Bkm1_Bk = T_O_Bw_km1_.inverse() * T_O_Bw_k;
    holistic_fusion::BinaryMeasurementXD<Eigen::Isometry3d, 6> delta6DMeasurement(
        "Wheel_odometry_6D", int(wheelOdometryBetweenRate_ / 5), wheelOdometryFrame, wheelOdometryFrame + sensorFrameCorrectedNameId,
        holistic_fusion::RobustNorm::Tukey(1.0), wheelOdometryTimeKm1_, wheelOdometryTimeK, T_Bkm1_Bk, wheelPoseBetweenNoise_);
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
      holistic_fusion::HolisticFusion::initHeadingAndPositionAtStart();
    }
  } else {
    holistic_fusion::UnaryMeasurementXD<Eigen::Vector3d, 3> leftWheelLinearVelocityMeasurement(
        "Wheel_linear_velocity_left", int(wheelLinearVelocitiesRate_), wheelLinearVelocityLeftFrame,
        wheelLinearVelocityLeftFrame + sensorFrameCorrectedNameId, holistic_fusion::RobustNorm::None(), timeK, 1.0,
        Eigen::Vector3d(leftWheelSpeedMs, 0.0, 0.0), wheelLinearVelocitiesNoise_);
    this->addUnaryVelocity3LocalMeasurement(leftWheelLinearVelocityMeasurement);

    holistic_fusion::UnaryMeasurementXD<Eigen::Vector3d, 3> rightWheelLinearVelocityMeasurement(
        "Wheel_linear_velocity_right", int(wheelLinearVelocitiesRate_), wheelLinearVelocityRightFrame,
        wheelLinearVelocityRightFrame + sensorFrameCorrectedNameId, holistic_fusion::RobustNorm::None(), timeK, 1.0,
        Eigen::Vector3d(rightWheelSpeedMs, 0.0, 0.0), wheelLinearVelocitiesNoise_);
    this->addUnaryVelocity3LocalMeasurement(rightWheelLinearVelocityMeasurement);
  }
}

void SmbEstimator::vioOdometryCallback_(const nav_msgs::msg::Odometry::ConstSharedPtr& vioOdomPtr) {
  std::cout << "VIO odometry not yet stable enough for usage, disable flag." << std::endl;

  Eigen::Isometry3d vio_T_M_Ck;
  holistic_fusion::odomMsgToEigen(*vioOdomPtr, vio_T_M_Ck.matrix());

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
