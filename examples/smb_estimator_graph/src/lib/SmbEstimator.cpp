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
}

void SmbEstimator::initializeMessages_(ros::NodeHandle& privateNode) {
  // Super
  graph_msf::GraphMsfRos::initializeMessages_(privateNode);
  // Path
  measLidar_worldImuPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
}

bool SmbEstimator::gnssCoordinatesToENUCallback_(graph_msf_ros_msgs::GetPathInEnu::Request& req,
                                                 graph_msf_ros_msgs::GetPathInEnu::Response& res) {
  nav_msgs::PathPtr enuPathPtr = nav_msgs::PathPtr(new nav_msgs::Path);
  for (auto& coordinate : req.wgs84Coordinates) {
    Eigen::Vector3d enuCoordinate;
    Eigen::Vector3d gnssCoordinate = Eigen::Vector3d(coordinate.latitude, coordinate.longitude, coordinate.altitude);
    gnssHandlerPtr_->convertNavSatToPosition(gnssCoordinate, enuCoordinate);

    addToPathMsg(enuPathPtr, dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())->getMapFrame(), ros::Time::now(), enuCoordinate,
                 std::numeric_limits<int>::max());
  }
  res.gnssEnuPath = *enuPathPtr;

  return true;
}

void SmbEstimator::lidarOdometryCallback_(const nav_msgs::Odometry::ConstPtr& odomLidarPtr) {
  // Static members
  static int odometryCallbackCounter__ = -1;
  static std::shared_ptr<graph_msf::UnaryMeasurement6D> odometryKm1Ptr__;

  // Counter
  ++odometryCallbackCounter__;

  Eigen::Matrix4d compslam_T_Wl_Lk;
  graph_msf::odomMsgToEigen(*odomLidarPtr, compslam_T_Wl_Lk);

  // Transform to IMU frame
  double timeK = odomLidarPtr->header.stamp.toSec();

  // Measurement
  std::shared_ptr<graph_msf::UnaryMeasurement6D> odometryKPtr;
  // Create pseudo unary factors
  if (graphConfigPtr_->usingGnssFlag) {
    odometryKPtr = std::make_unique<graph_msf::UnaryMeasurement6D>(
        "Lidar 6D", dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())->getLidarFrame(), int(lidarRate_), timeK,
        compslam_T_Wl_Lk, poseUnaryNoise_);
    if (odometryCallbackCounter__ > 0) {
      this->addDualOdometryMeasurementAndReturnNavState(*odometryKm1Ptr__, *odometryKPtr, poseBetweenNoise_);
    }
  } else {  // real unary factors
    odometryKPtr = std::make_unique<graph_msf::UnaryMeasurement6D>(
        "Lidar 6D", dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())->getLidarFrame(), int(lidarRate_), timeK,
        compslam_T_Wl_Lk, poseUnaryNoise_);
    this->addUnaryPoseMeasurement(*odometryKPtr);
  }

  if (!areYawAndPositionInited() && (!graphConfigPtr_->usingGnssFlag || secondsSinceStart_() > 15)) {
    REGULAR_COUT << GREEN_START
                 << " LiDAR odometry callback is setting global yaw, as it was "
                    "not set so far."
                 << COLOR_END << std::endl;
    this->initYawAndPosition(compslam_T_Wl_Lk, dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())->getLidarFrame());
  }

  // Wrap up iteration
  odometryKm1Ptr__ = odometryKPtr;

  // Add to path message
  addToPathMsg(measLidar_worldImuPathPtr_, staticTransformsPtr_->getWorldFrame(), odomLidarPtr->header.stamp,
               (compslam_T_Wl_Lk * staticTransformsPtr_
                                       ->rv_T_frame1_frame2(dynamic_cast<SmbStaticTransforms*>(staticTransformsPtr_.get())->getLidarFrame(),
                                                            staticTransformsPtr_->getImuFrame())
                                       .matrix())
                   .block<3, 1>(0, 3),
               graphConfigPtr_->imuBufferLength * 4);

  // Publish Path
  pubMeasWorldLidarPath_.publish(measLidar_worldImuPathPtr_);
}

}  // namespace smb_se