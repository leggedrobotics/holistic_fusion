/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "anymal_dual_graph/AnymalEstimator.h"

// Project
#include "anymal_dual_graph/AnymalStaticTransforms.h"

// Workspace
#include "graph_msf/measurements/BinaryMeasurement6D.h"
#include "graph_msf/measurements/UnaryMeasurement1D.h"
#include "graph_msf/measurements/UnaryMeasurement3D.h"
#include "graph_msf/measurements/UnaryMeasurement6D.h"
#include "graph_msf_ros/util/conversions.h"

namespace anymal_se {

AnymalEstimator::AnymalEstimator(std::shared_ptr<ros::NodeHandle> privateNodePtr) : graph_msf::GraphMsfRos(privateNodePtr) {
  std::cout << YELLOW_START << "AnymalEstimator" << GREEN_START << " Setting up." << COLOR_END << std::endl;

  // Configurations ----------------------------
  // Extend static transforms
  staticTransformsPtr_ = std::make_shared<AnymalStaticTransforms>(privateNodePtr, *staticTransformsPtr_);
  // Implementation specific
  gnssHandlerPtr_ = std::make_shared<graph_msf::GnssHandler>();
  trajectoryAlignmentHandlerPtr_ = std::make_shared<graph_msf::TrajectoryAlignmentHandler>();

  // Get ROS params and set extrinsics
  readParams_(privateNode_);
  staticTransformsPtr_->findTransformations();

  // Publishers ----------------------------
  initializePublishers_(privateNodePtr);

  // Subscribers ----------------------------
  initializeSubscribers_(privateNodePtr);

  // Messages ----------------------------
  initializeMessages_(privateNodePtr);

  // Server ----------------------------
  initializeServers_(privateNodePtr);

  trajectoryAlignmentHandlerPtr_->initHandler();

  std::cout << YELLOW_START << "AnymalEstimatorGraph" << GREEN_START << " Set up successfully." << COLOR_END << std::endl;
}

void AnymalEstimator::initializePublishers_(std::shared_ptr<ros::NodeHandle>& privateNodePtr) {
  // Super
  graph_msf::GraphMsfRos::initializePublishers_(privateNodePtr);
  // Paths
  pubMeasWorldGnssPath_ = privateNode_.advertise<nav_msgs::Path>("/graph_msf/measGnss_path_map_gnss", ROS_QUEUE_SIZE);
  pubMeasWorldLidarPath_ = privateNode_.advertise<nav_msgs::Path>("/graph_msf/measLiDAR_path_map_imu", ROS_QUEUE_SIZE);
}

void AnymalEstimator::initializeSubscribers_(std::shared_ptr<ros::NodeHandle>& privateNodePtr) {
  // LiDAR Odometry
  subLidarOdometry_ = privateNode_.subscribe<nav_msgs::Odometry>(
      "/lidar_odometry_topic", ROS_QUEUE_SIZE, &AnymalEstimator::lidarOdometryCallback_, this, ros::TransportHints().tcpNoDelay());
  std::cout << YELLOW_START << "CompslamEstimator" << COLOR_END
            << " Initialized LiDAR Odometry subscriber with topic: " << subLidarOdometry_.getTopic() << std::endl;

  // GNSS
  if (graphConfigPtr_->usingGnssFlag) {
    subGnss_ = privateNode_.subscribe<sensor_msgs::NavSatFix>("/gnss_topic", ROS_QUEUE_SIZE, &AnymalEstimator::gnssCallback_, this,
                                                              ros::TransportHints().tcpNoDelay());
    std::cout << YELLOW_START << "FactorGraphFiltering" << COLOR_END << " Initialized Gnss subscriber with topic: " << subGnss_.getTopic()
              << std::endl;
  } else {
    std::cout << YELLOW_START << "CompslamEstimator" << GREEN_START
              << " Gnss usage is set to false. Hence, lidar unary factors will be activated after graph initialization." << COLOR_END
              << std::endl;
  }
}

void AnymalEstimator::initializeMessages_(std::shared_ptr<ros::NodeHandle>& privateNodePtr) {
  // Super
  graph_msf::GraphMsfRos::initializeMessages_(privateNodePtr);
  // Path
  measGnss_worldGnssPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  measLidar_worldImuPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
}

void AnymalEstimator::initializeServers_(std::shared_ptr<ros::NodeHandle>& privateNodePtr) {
  serverTransformGnssToEnu_ =
      privateNode_.advertiseService("/gnss_coordinates_to_enu_topic", &AnymalEstimator::gnssCoordinatesToENUCallback_, this);
}

bool AnymalEstimator::gnssCoordinatesToENUCallback_(graph_msf_msgs::GetPathInEnu::Request& req,
                                                    graph_msf_msgs::GetPathInEnu::Response& res) {
  nav_msgs::PathPtr enuPathPtr = nav_msgs::PathPtr(new nav_msgs::Path);
  for (auto& coordinate : req.wgs84Coordinates) {
    Eigen::Vector3d enuCoordinate;
    Eigen::Vector3d gnssCoordinate = Eigen::Vector3d(coordinate.latitude, coordinate.longitude, coordinate.altitude);
    gnssHandlerPtr_->convertNavSatToPosition(gnssCoordinate, enuCoordinate);

    addToPathMsg(enuPathPtr, dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getMapFrame(), ros::Time::now(),
                 enuCoordinate, std::numeric_limits<int>::max());
  }
  res.gnssEnuPath = *enuPathPtr;

  return true;
}

void AnymalEstimator::lidarOdometryCallback_(const nav_msgs::Odometry::ConstPtr& odomLidarPtr) {
  // Static members
  static int odometryCallbackCounter__ = -1;
  static std::shared_ptr<graph_msf::UnaryMeasurement6D> odometryKm1Ptr__;

  // Counter
  ++odometryCallbackCounter__;

  Eigen::Matrix4d compslam_T_Wl_Lk;
  graph_msf::odomMsgToEigen(*odomLidarPtr, compslam_T_Wl_Lk);

  // Trajectory alignment ----------------------------------------------
  if (!initialized_ && graphConfigPtr_->usingGnssFlag) {
    trajectoryAlignmentHandlerPtr_->addLidarPose(compslam_T_Wl_Lk.block<3, 1>(0, 3), odomLidarPtr->header.stamp.toSec());
    --odometryCallbackCounter__;
    return;
  }

  // Transform to IMU frame
  double timeK = odomLidarPtr->header.stamp.toSec();

  // Measurement
  std::shared_ptr<graph_msf::UnaryMeasurement6D> odometryKPtr;
  // Create pseudo unary factors
  if (graphConfigPtr_->usingGnssFlag) {
    odometryKPtr = std::make_unique<graph_msf::UnaryMeasurement6D>(
        "Lidar 6D", dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getLidarFrame(), int(lidarRate_), timeK,
        compslam_T_Wl_Lk, poseUnaryNoise_);
    if (odometryCallbackCounter__ > 0) {
      this->addDualOdometryMeasurementAndReturnNavState(*odometryKm1Ptr__, *odometryKPtr, poseBetweenNoise_);
    }
  } else {  // real unary factors
    odometryKPtr = std::make_unique<graph_msf::UnaryMeasurement6D>(
        "Lidar 6D", dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getLidarFrame(), int(lidarRate_), timeK,
        compslam_T_Wl_Lk, poseUnaryNoise_);
    this->addUnaryPoseMeasurement(*odometryKPtr);
  }

  if (!areYawAndPositionInited() && (!graphConfigPtr_->usingGnssFlag || secondsSinceStart_() > 15)) {
    std::cout << YELLOW_START << "AnymalEstimator" << GREEN_START
              << " LiDAR odometry callback is setting global yaw, as it was not set so far." << COLOR_END << std::endl;
    this->initYawAndPosition(compslam_T_Wl_Lk, dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getLidarFrame());
  }

  // Wrap up iteration
  odometryKm1Ptr__ = odometryKPtr;

  // Add to path message
  addToPathMsg(
      measLidar_worldImuPathPtr_, staticTransformsPtr_->getWorldFrame(), odomLidarPtr->header.stamp,
      (compslam_T_Wl_Lk * staticTransformsPtr_
                              ->rv_T_frame1_frame2(dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getLidarFrame(),
                                                   staticTransformsPtr_->getImuFrame())
                              .matrix())
          .block<3, 1>(0, 3),
      graphConfigPtr_->imuBufferLength * 4);

  // Publish Path
  pubMeasWorldLidarPath_.publish(measLidar_worldImuPathPtr_);
}

void AnymalEstimator::gnssCallback_(const sensor_msgs::NavSatFix::ConstPtr& gnssMsgPtr) {
  // Static method variables
  static Eigen::Vector3d accumulatedCoordinates__(0.0, 0.0, 0.0);
  static Eigen::Vector3d W_t_W_Gnss_km1__;
  static int gnssCallbackCounter__ = 0;
  Eigen::Vector3d W_t_W_Gnss;

  // Start
  ++gnssCallbackCounter__;
  Eigen::Vector3d gnssCoord = Eigen::Vector3d(gnssMsgPtr->latitude, gnssMsgPtr->longitude, gnssMsgPtr->altitude);
  Eigen::Vector3d estCovarianceXYZ(gnssMsgPtr->position_covariance[0], gnssMsgPtr->position_covariance[4],
                                   gnssMsgPtr->position_covariance[8]);

  if (!graphConfigPtr_->usingGnssFlag) {
    ROS_WARN("Received Gnss message, but usage is set to false.");
    this->activateFallbackGraph();
    return;
  } else if (not initialized_) {
    if (gnssCallbackCounter__ < NUM_GNSS_CALLBACKS_UNTIL_START + 1) {
      // Wait until measurements got accumulated
      accumulatedCoordinates__ += gnssCoord;
      if (!(gnssCallbackCounter__ % 10)) {
        std::cout << YELLOW_START << "AnymalEstimator" << COLOR_END << " NOT ENOUGH Gnss MESSAGES ARRIVED!" << std::endl;
      }
      return;
    } else if (gnssCallbackCounter__ == NUM_GNSS_CALLBACKS_UNTIL_START + 1) {
      gnssHandlerPtr_->initHandler(accumulatedCoordinates__ / NUM_GNSS_CALLBACKS_UNTIL_START);
      ++gnssCallbackCounter__;
    }
    // Convert to cartesian coordinates
    gnssHandlerPtr_->convertNavSatToPosition(gnssCoord, W_t_W_Gnss);
    trajectoryAlignmentHandlerPtr_->addGnssPose(W_t_W_Gnss, gnssMsgPtr->header.stamp.toSec());

    double initYawEnuLidar;
    if (!trajectoryAlignmentHandlerPtr_->alignTrajectories(initYawEnuLidar)) {
      --gnssCallbackCounter__;
      return;
    }
    double initYaw = initYawEnuLidar;
    gnssHandlerPtr_->setInitYaw(initYaw);
    initialized_ = true;

    gnssHandlerPtr_->initHandler(gnssHandlerPtr_->getInitYaw());
  }

  // Convert to cartesian coordinates
  gnssHandlerPtr_->convertNavSatToPosition(gnssCoord, W_t_W_Gnss);

  // Initialization
  if (gnssCallbackCounter__ == NUM_GNSS_CALLBACKS_UNTIL_START + 2) {
    if (not this->initYawAndPosition(gnssHandlerPtr_->getInitYaw(),
                                     dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getLidarFrame(), W_t_W_Gnss,
                                     dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getGnssFrame())) {
      // Decrease counter if not successfully initialized
      --gnssCallbackCounter__;
    }
  } else {
    graph_msf::UnaryMeasurement3D meas_W_t_W_Gnss(
        "Gnss", dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getGnssFrame(), gnssRate_,
        gnssMsgPtr->header.stamp.toSec(), W_t_W_Gnss,
        Eigen::Vector3d(gnssPositionUnaryNoise_, gnssPositionUnaryNoise_, gnssPositionUnaryNoise_));
    // graph_msf::GraphMsfInterface::addGnssPositionMeasurement_(meas_W_t_W_Gnss);
    this->addDualGnssPositionMeasurement(meas_W_t_W_Gnss, W_t_W_Gnss_km1__, estCovarianceXYZ, true, true);
  }
  W_t_W_Gnss_km1__ = W_t_W_Gnss;

  /// Add GNSS to Path
  addToPathMsg(measGnss_worldGnssPathPtr_, dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getMapFrame(),
               gnssMsgPtr->header.stamp, W_t_W_Gnss, graphConfigPtr_->imuBufferLength * 4);
  /// Publish path
  pubMeasWorldGnssPath_.publish(measGnss_worldGnssPathPtr_);
}

}  // namespace anymal_se