/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "smb_estimator_graph/SmbEstimator.h"

// Project
#include "smb_estimator_graph/SmbStaticTransforms.h"

// Workspace
#include "graph_msf/measurements/BinaryMeasurementXD.h"
#include "graph_msf/measurements/UnaryMeasurementXD.h"
#include "graph_msf_ros/util/conversions.h"
#include "smb_estimator_graph/constants.h"

namespace smb_se {

SmbEstimator::SmbEstimator(std::shared_ptr<ros::NodeHandle> privateNodePtr) : graph_msf::GraphMsfRos(privateNodePtr) {
  REGULAR_COUT << GREEN_START << " Initializing..." << COLOR_END << std::endl;

  // Configurations ----------------------------
  // Static transforms
  staticTransformsPtr_ = std::make_shared<SmbStaticTransforms>(privateNodePtr);

  // Set up
  if (!SmbEstimator::setup()) {
    REGULAR_COUT << RED_START << " Failed to set up." << COLOR_END << std::endl;
    std::runtime_error("SmbEstimator failed to set up.");
  }

  REGULAR_COUT << GREEN_START << " Set up successfully." << COLOR_END << std::endl;
}

bool SmbEstimator::setup() {
  REGULAR_COUT << GREEN_START << " Setting up." << COLOR_END << std::endl;

  // Super class
  if (not graph_msf::GraphMsfRos::setup()) {
    throw std::runtime_error("GraphMsfRos could not be initialized");
  }

  // Read parameters ----------------------------
  SmbEstimator::readParams_(privateNode_);

  // Transforms ----------------------------
  staticTransformsPtr_->findTransformations();

  // Publishers ----------------------------
  SmbEstimator::initializePublishers_(privateNode_);

  // Subscribers ----------------------------
  SmbEstimator::initializeSubscribers_(privateNode_);

  // Messages ----------------------------
  SmbEstimator::initializeMessages_(privateNode_);

  // Services ----------------------------
  SmbEstimator::initializeServices_(privateNode_);

  // Wrap up ----------------------------
  REGULAR_COUT << GREEN_START << " Set up successfully." << COLOR_END << std::endl;

  return true;
}

void SmbEstimator::initializePublishers_(ros::NodeHandle& privateNode) {
  // Paths
  pubMeasMapLioPath_ = privateNode_.advertise<nav_msgs::Path>("/graph_msf/measLiDAR_path_map_imu", ROS_QUEUE_SIZE);
  pubMeasMapVioPath_ = privateNode_.advertise<nav_msgs::Path>("/graph_msf/measVIO_path_map_imu", ROS_QUEUE_SIZE);
}

void SmbEstimator::initializeSubscribers_(ros::NodeHandle& privateNode) {
  // LiDAR Odometry
  if (useLioOdometryFlag_) {
    subLioOdometry_ = privateNode_.subscribe<nav_msgs::Odometry>(
        "/lidar_odometry_topic", ROS_QUEUE_SIZE, &SmbEstimator::lidarOdometryCallback_, this, ros::TransportHints().tcpNoDelay());
    REGULAR_COUT << COLOR_END << " Initialized LiDAR Odometry subscriber with topic: " << subLioOdometry_.getTopic() << std::endl;
  }

  // Wheel Odometry
  if (useWheelOdometryFlag_) {
    subWheelOdometry_ = privateNode_.subscribe<nav_msgs::Odometry>(
        "/wheel_odometry_topic", ROS_QUEUE_SIZE, &SmbEstimator::wheelOdometryCallback_, this, ros::TransportHints().tcpNoDelay());
    REGULAR_COUT << COLOR_END << " Initialized Wheel Odometry subscriber with topic: " << subWheelOdometry_.getTopic() << std::endl;
  }

  // VIO
  if (useVioOdometryFlag_) {
    subVioOdometry_ = privateNode_.subscribe<nav_msgs::Odometry>("/vio_odometry_topic", ROS_QUEUE_SIZE, &SmbEstimator::vioOdometryCallback_,
                                                                 this, ros::TransportHints().tcpNoDelay());
    REGULAR_COUT << COLOR_END << " Initialized VIO Odometry subscriber with topic: " << subVioOdometry_.getTopic() << std::endl;
  }
}

void SmbEstimator::initializeMessages_(ros::NodeHandle& privateNode) {
  // Path
  measLio_mapImuPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  measVio_mapImuPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
}

void SmbEstimator::initializeServices_(ros::NodeHandle& privateNode) {
  // Nothing for now
}

void SmbEstimator::imuCallback_(const sensor_msgs::Imu::ConstPtr& imuPtr) {
  // Check whether any of the measurements is available, otherwise do pure imu integration
  if (graph_msf::GraphMsf::areRollAndPitchInited() && !graph_msf::GraphMsf::areYawAndPositionInited() && !useLioOdometryFlag_ &&
      !useWheelOdometryFlag_ && !useVioOdometryFlag_) {
    // Initialization
    REGULAR_COUT << RED_START << " IMU callback is setting global yaw and position, as no other odometry is available. Initializing..."
                 << COLOR_END << std::endl;
    // Create dummy measurement for initialization
    graph_msf::UnaryMeasurementXD<Eigen::Isometry3d, 6> unary6DMeasurement(
        "IMU_init_6D", int(graphConfigPtr_->imuRate_), staticTransformsPtr_->getImuFrame(),
        staticTransformsPtr_->getImuFrame() + sensorFrameCorrectedNameId_, graph_msf::RobustNormEnum::None, 1.345,
        imuPtr->header.stamp.toSec(), staticTransformsPtr_->getWorldFrame(), 1.0, initialSe3AlignmentNoise_, Eigen::Isometry3d::Identity(),
        Eigen::MatrixXd::Identity(6, 1));
    // Initialize
    graph_msf::GraphMsf::initYawAndPosition(unary6DMeasurement);
    REGULAR_COUT << RED_START << " ...initialized yaw and position to identity." << COLOR_END << std::endl;
    // Pretend that we received first measurement --> In order to allow for optimization
    graph_msf::GraphMsf::pretendFirstMeasurementReceived();
  }

  // Super class
  graph_msf::GraphMsfRos::imuCallback_(imuPtr);
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
  graph_msf::UnaryMeasurementXD<Eigen::Isometry3d, 6> unary6DMeasurement(
      "Lidar_unary_6D", int(lioOdometryRate_), lioOdometryFrame, lioOdometryFrame + sensorFrameCorrectedNameId_,
      graph_msf::RobustNormEnum::None, 1.345, lidarOdometryTimeK, odomLidarPtr->header.frame_id, 1.0, initialSe3AlignmentNoise_, lio_T_M_Lk,
      lioPoseUnaryNoise_);

  // Add measurement or initialize
  if (lidarOdometryCallbackCounter__ <= 2) {
    return;
  } else if (areYawAndPositionInited()) {  // Already initialized --> unary factor
    this->addUnaryPose3Measurement(unary6DMeasurement);
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

void SmbEstimator::wheelOdometryCallback_(const nav_msgs::Odometry::ConstPtr& wheelOdometryKPtr) {
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
  const std::string& wheelOdometryFrame = dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())->getWheelOdometryFrame();  // alias

  // State Machine
  if (!areYawAndPositionInited()) {
    // If no LIO, then initialize the graph here
    if (!useLioOdometryFlag_) {
      REGULAR_COUT << GREEN_START << " Wheel odometry callback is setting global yaw and position, as lio is all set to false." << COLOR_END
                   << std::endl;
      graph_msf::UnaryMeasurementXD<Eigen::Isometry3d, 6> unary6DMeasurement(
          "Lidar_unary_6D", int(wheelOdometryRate_), wheelOdometryFrame, wheelOdometryFrame + sensorFrameCorrectedNameId_,
          graph_msf::RobustNormEnum::None, 1.345, wheelOdometryTimeK, staticTransformsPtr_->getWorldFrame(), 1.0, initialSe3AlignmentNoise_,
          Eigen::Isometry3d::Identity(), Eigen::MatrixXd::Identity(6, 1));
      graph_msf::GraphMsf::initYawAndPosition(unary6DMeasurement);
      REGULAR_COUT << " Initialized yaw and position to identity in the wheel odometry callback, as lio and vio are all set to false."
                   << std::endl;
    }
  } else {
    // Only add every 5th measurement
    int measurementRate = static_cast<int>(wheelOdometryRate_ / 5);
    // Check
    if (wheelOdometryCallbackCounter_ % 5 == 0 && wheelOdometryCallbackCounter_ > 0) {
      // Compute Delta
      Eigen::Isometry3d T_Bkm1_Bk = T_O_Bw_km1_.inverse() * T_O_Bw_k;
      // Create measurement
      graph_msf::BinaryMeasurementXD<Eigen::Isometry3d, 6> delta6DMeasurement(
          "Wheel_odometry_6D", measurementRate, wheelOdometryFrame, wheelOdometryFrame + sensorFrameCorrectedNameId_,
          graph_msf::RobustNormEnum::Tukey, 1.0, wheelOdometryTimeKm1_, wheelOdometryTimeK, T_Bkm1_Bk, wheelPoseBetweenNoise_);
      // Add to graph
      this->addBinaryPoseMeasurement(delta6DMeasurement);

      // Prepare for next iteration
      T_O_Bw_km1_ = T_O_Bw_k;
      wheelOdometryTimeKm1_ = wheelOdometryTimeK;
    }
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