/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "smb_estimator_graph/SmbEstimator.h"

// Project
#include "smb_estimator_graph/SmbStaticTransforms.h"

// Workspace
#include "graph_msf/measurements/BinaryMeasurement6D.h"
#include "graph_msf/measurements/UnaryMeasurement1D.h"
#include "graph_msf/measurements/UnaryMeasurement3D.h"
#include "graph_msf/measurements/UnaryMeasurement6D.h"
#include "graph_msf_ros/util/conversions.h"
#include "smb_estimator_graph/constants.h"

namespace smb_se {

SmbEstimator::SmbEstimator(std::shared_ptr<ros::NodeHandle> privateNodePtr) : graph_msf::GraphMsfRos(privateNodePtr) {
  REGULAR_COUT << GREEN_START << " Initializing..." << COLOR_END << std::endl;

  // Configurations ----------------------------
  // Static transforms
  staticTransformsPtr_ = std::make_shared<SmbStaticTransforms>(privateNodePtr);
  // GNSS
  gnssHandlerPtr_ = std::make_shared<graph_msf::GnssHandler>();

  // Set up
  if (!SmbEstimator::setup()) {
    REGULAR_COUT << RED_START << "M545EstimatorGraph" << COLOR_END << " Failed to set up." << std::endl;
    std::runtime_error("M545EstimatorGraph failed to set up.");
  }

  REGULAR_COUT << GREEN_START << " Set up successfully." << COLOR_END << std::endl;
}

bool SmbEstimator::setup() {
  REGULAR_COUT << GREEN_START << " Setting up." << COLOR_END << std::endl;

  // Super class
  if (not graph_msf::GraphMsfRos::setup()) {
    throw std::runtime_error("GraphMsfRos could not be initialized");
  }

  // Publishers ----------------------------
  SmbEstimator::initializePublishers_(privateNode_);

  // Subscribers ----------------------------
  SmbEstimator::initializeSubscribers_(privateNode_);

  // Messages ----------------------------
  SmbEstimator::initializeMessages_(privateNode_);

  // Read parameters ----------------------------
  SmbEstimator::readParams_(privateNode_);
  staticTransformsPtr_->findTransformations();

  // Wrap up ----------------------------
  REGULAR_COUT << GREEN_START << " Set up successfully." << COLOR_END << std::endl;

  return true;
}

void SmbEstimator::initializePublishers_(ros::NodeHandle& privateNode) {
  // Super
  graph_msf::GraphMsfRos::initializePublishers_(privateNode);
  // Paths
  pubMeasWorldLidarPath_ = privateNode_.advertise<nav_msgs::Path>("/graph_msf/measLiDAR_path_map_imu", ROS_QUEUE_SIZE);
}

void SmbEstimator::initializeSubscribers_(ros::NodeHandle& privateNode) {
  // LiDAR Odometry
  subLidarOdometry_ = privateNode_.subscribe<nav_msgs::Odometry>(
      "/lidar_odometry_topic", ROS_QUEUE_SIZE, &SmbEstimator::lidarOdometryCallback_, this, ros::TransportHints().tcpNoDelay());
  REGULAR_COUT << COLOR_END << " Initialized LiDAR Odometry subscriber with topic: " << subLidarOdometry_.getTopic() << std::endl;

  // Wheel Odometry
  subWheelOdometry_ = privateNode_.subscribe<nav_msgs::Odometry>(
      "/wheel_odometry_topic", ROS_QUEUE_SIZE, &SmbEstimator::wheelOdometryCallback_, this, ros::TransportHints().tcpNoDelay());
}

void SmbEstimator::initializeMessages_(ros::NodeHandle& privateNode) {
  // Super
  graph_msf::GraphMsfRos::initializeMessages_(privateNode);
  // Path
  measLidar_worldImuPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
}

void SmbEstimator::lidarOdometryCallback_(const nav_msgs::Odometry::ConstPtr& odomLidarPtr) {
  // Static members
  static int lidarOdometryCallbackCounter__ = -1;
  static std::shared_ptr<graph_msf::UnaryMeasurement6D> lidarOdometryKm1Ptr__;

  // Counter
  ++lidarOdometryCallbackCounter__;

  Eigen::Matrix4d compslam_T_Wl_Lk;
  graph_msf::odomMsgToEigen(*odomLidarPtr, compslam_T_Wl_Lk);

  // Transform to IMU frame
  double timeK = odomLidarPtr->header.stamp.toSec();

  // Measurement
  std::shared_ptr<graph_msf::UnaryMeasurement6D> lidarOdometryKPtr = std::make_unique<graph_msf::UnaryMeasurement6D>(
      "Lidar_6D", dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())->getLidarOdometryFrame(), int(lidarOdometryRate_), timeK,
      compslam_T_Wl_Lk, poseUnaryNoise_);

  if (lidarOdometryCallbackCounter__ <= 0) {
    return;
  } else if (areYawAndPositionInited()) {  // Already initialized --> unary factor
    this->addUnaryPoseMeasurement(*lidarOdometryKPtr);
  } else if (!graphConfigPtr_->usingGnssFlag || secondsSinceStart_() > 15) {  // Initializing
    REGULAR_COUT << GREEN_START
                 << " LiDAR odometry callback is setting global yaw, as it was "
                    "not set so far."
                 << COLOR_END << std::endl;
    this->initYawAndPosition(compslam_T_Wl_Lk, dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())->getLidarOdometryFrame());
  }

  // Wrap up iteration
  lidarOdometryKm1Ptr__ = lidarOdometryKPtr;

  // Visualization ----------------------------
  // Add to path message
  addToPathMsg(
      measLidar_worldImuPathPtr_, staticTransformsPtr_->getWorldFrame(), odomLidarPtr->header.stamp,
      (compslam_T_Wl_Lk * staticTransformsPtr_
                              ->rv_T_frame1_frame2(dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())->getLidarOdometryFrame(),
                                                   staticTransformsPtr_->getImuFrame())
                              .matrix())
          .block<3, 1>(0, 3),
      graphConfigPtr_->imuBufferLength * 4);

  // Publish Path
  pubMeasWorldLidarPath_.publish(measLidar_worldImuPathPtr_);
}

void SmbEstimator::wheelOdometryCallback_(const nav_msgs::Odometry::ConstPtr& wheelOdometryKPtr) {
  // Static members
  static int wheelOdometryCallbackCounter__ = -1;
  static Eigen::Isometry3d T_O_Wkm1__;
  static double timeKm1__ = 0.0;

  // Counter
  ++wheelOdometryCallbackCounter__;

  // Eigen Type
  Eigen::Isometry3d T_O_Wk;
  graph_msf::odomMsgToEigen(*wheelOdometryKPtr, T_O_Wk.matrix());
  double timeK = wheelOdometryKPtr->header.stamp.toSec();

  if (wheelOdometryCallbackCounter__ > 0) {
    // Compute Delta
    Eigen::Isometry3d T_Wkm1_Wk = T_O_Wkm1__.inverse() * T_O_Wk;
    // Create measurement
    graph_msf::BinaryMeasurement6D deltaMeasurement("Wheel_odometry_6D",
                                                    dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())->getWheelOdometryFrame(),
                                                    wheelOdometryRate_, timeKm1__, timeK, T_Wkm1_Wk, poseBetweenNoise_);
    // Add to graph
    graph_msf::GraphMsf::addOdometryMeasurement(deltaMeasurement);
  }

  // Provide next iteration
  T_O_Wkm1__ = T_O_Wk;
  timeKm1__ = wheelOdometryKPtr->header.stamp.toSec();
}

}  // namespace smb_se