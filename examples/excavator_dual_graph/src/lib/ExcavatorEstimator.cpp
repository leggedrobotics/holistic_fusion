/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "excavator_dual_graph/ExcavatorEstimator.h"

// Project
#include "excavator_dual_graph/ExcavatorStaticTransforms.h"

// Workspace
#include "graph_msf/measurements/BinaryMeasurementXD.h"
#include "graph_msf/measurements/UnaryMeasurement1D_.h"
#include "graph_msf/measurements/UnaryMeasurement3D_.h"
#include "graph_msf/measurements/UnaryMeasurementXD.h"
#include "graph_msf_ros/util/conversions.h"

namespace excavator_se {

ExcavatorEstimator::ExcavatorEstimator(std::shared_ptr<ros::NodeHandle> privateNodePtr) : privateNode_(*privateNodePtr) {
  std::cout << YELLOW_START << "ExcavatorEstimator" << GREEN_START << " Setting up." << COLOR_END << std::endl;

  // Configurations ----------------------------
  graphConfigPtr_ = std::make_shared<graph_msf::GraphConfig>();
  staticTransformsPtr_ = std::make_shared<ExcavatorStaticTransforms>(privateNodePtr);
  gnssHandlerPtr_ = std::make_shared<graph_msf::GnssHandler>();

  // Get ROS params and set extrinsics
  readParams_(privateNode_);
  staticTransformsPtr_->findTransformations();

  if (not graph_msf::GraphMsf::setup()) {
    throw std::runtime_error("GraphMsf could not be initialized");
  }

  // Publishers ----------------------------
  initializePublishers_(privateNodePtr);

  // Subscribers ----------------------------
  initializeSubscribers_(privateNodePtr);

  // Messages ----------------------------
  initializeMessages_(privateNodePtr);

  std::cout << YELLOW_START << "ExcavatorEstimator" << GREEN_START << " Set up successfully." << COLOR_END << std::endl;
}

void ExcavatorEstimator::initializePublishers_(std::shared_ptr<ros::NodeHandle>& privateNodePtr) {
  // Super Class
  graph_msf::GraphMsfRos::initializePublishers_(privateNodePtr);
  // Paths
  pubMeasWorldGnssLPath_ = privateNode_.advertise<nav_msgs::Path>("/graph_msf/meas_path_world_gnssL", ROS_QUEUE_SIZE);
  pubMeasWorldGnssRPath_ = privateNode_.advertise<nav_msgs::Path>("/graph_msf/meas_path_world_gnssR", ROS_QUEUE_SIZE);
  pubMeasWorldLidarPath_ = privateNode_.advertise<nav_msgs::Path>("/graph_msf/meas_path_world_Lidar", ROS_QUEUE_SIZE);
}

void ExcavatorEstimator::initializeMessages_(std::shared_ptr<ros::NodeHandle>& privateNodePtr) {
  // Super
  graph_msf::GraphMsfRos::initializeMessages_(privateNodePtr);
  // Path
  measWorldLeftGnssPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  measWorldRightGnssPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  measWorldLidarPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
}

void ExcavatorEstimator::initializeSubscribers_(std::shared_ptr<ros::NodeHandle>& privateNodePtr) {
  // Imu
  subImu_ = privateNode_.subscribe<sensor_msgs::Imu>("/imu_topic", ROS_QUEUE_SIZE, &ExcavatorEstimator::imuCallback_, this,
                                                     ros::TransportHints().tcpNoDelay());
  std::cout << YELLOW_START << "ExcavatorEstimator" << COLOR_END << " Initialized IMU cabin subscriber." << std::endl;

  // LiDAR Odometry
  subLidarOdometry_ = privateNode_.subscribe<nav_msgs::Odometry>(
      "/lidar_odometry_topic", ROS_QUEUE_SIZE, &ExcavatorEstimator::lidarOdometryCallback_, this, ros::TransportHints().tcpNoDelay());
  std::cout << YELLOW_START << "ExcavatorEstimator" << COLOR_END << " Initialized LiDAR Odometry subscriber." << std::endl;

  // GNSS
  if (graphConfigPtr_->usingGnssFlag) {
    subGnssLeft_.subscribe(*privateNodePtr, "/gnss_topic_1", ROS_QUEUE_SIZE);
    subGnssRight_.subscribe(*privateNodePtr, "/gnss_topic_2", ROS_QUEUE_SIZE);
    gnssExactSyncPtr_.reset(
        new message_filters::Synchronizer<GnssExactSyncPolicy>(GnssExactSyncPolicy(ROS_QUEUE_SIZE), subGnssLeft_, subGnssRight_));
    gnssExactSyncPtr_->registerCallback(boost::bind(&ExcavatorEstimator::gnssCallback_, this, _1, _2));
    std::cout << YELLOW_START << "FactorGraphFiltering" << COLOR_END << " Initialized Gnss subscriber (for both Gnss topics)." << std::endl;
  } else {
    std::cout << YELLOW_START << "ExcavatorEstimator" << GREEN_START
              << " Gnss usage is set to false. Hence, lidar unary factors will be activated after graph initialization." << COLOR_END
              << std::endl;
  }
}

void ExcavatorEstimator::imuCallback_(const sensor_msgs::Imu::ConstPtr& imuMsgPtr) {
  static bool firstCallbackFlag__ = true;
  if (firstCallbackFlag__) {
    firstCallbackFlag__ = false;
  }
  // Convert to Eigen
  Eigen::Vector3d linearAcc(imuMsgPtr->linear_acceleration.x, imuMsgPtr->linear_acceleration.y, imuMsgPtr->linear_acceleration.z);
  Eigen::Vector3d angularVel(imuMsgPtr->angular_velocity.x, imuMsgPtr->angular_velocity.y, imuMsgPtr->angular_velocity.z);
  // Create Pointer for Carrying State
  std::shared_ptr<graph_msf::SafeNavState> preIntegratedNavStatePtr;
  std::shared_ptr<graph_msf::SafeNavStateWithCovarianceAndBias> optimizedStateWithCovarianceAndBiasPtr;
  // Add Measurement and Get State
  if (graph_msf::GraphMsf::addImuMeasurementAndGetState(linearAcc, angularVel, imuMsgPtr->header.stamp.toSec(), preIntegratedNavStatePtr,
                                                        optimizedStateWithCovarianceAndBiasPtr)) {
    // Encountered delay
    if (ros::Time::now() - ros::Time(preIntegratedNavStatePtr->getTimeK()) > ros::Duration(0.5)) {
      std::cout << RED_START << "ExcavatorEstimator" << COLOR_END << " Encountered delay of "
                << (ros::Time::now() - ros::Time(preIntegratedNavStatePtr->getTimeK())).toSec() << " seconds." << std::endl;
    }
    // Publish Odometry
    this->publishState_(preIntegratedNavStatePtr, optimizedStateWithCovarianceAndBiasPtr);
  }
}

void ExcavatorEstimator::lidarOdometryCallback_(const nav_msgs::Odometry::ConstPtr& odomLidarPtr) {
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
        "Lidar 6D", dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getLidarFrame(), int(lidarRate_), timeK,
        compslam_T_Wl_Lk, poseUnaryNoise_);
    if (odometryCallbackCounter__ > 0) {
      this->addDualOdometryMeasurementAndReturnNavState(*odometryKm1Ptr__, *odometryKPtr, poseBetweenNoise_);
    }
  } else {  // real unary factors
    odometryKPtr = std::make_unique<graph_msf::UnaryMeasurement6D>(
        "Lidar 6D", dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getLidarFrame(), int(lidarRate_), timeK,
        compslam_T_Wl_Lk, poseUnaryNoise_);
    this->addUnaryPoseMeasurement(*odometryKPtr);
  }

  if (!this->areYawAndPositionInited() && (!graphConfigPtr_->usingGnssFlag || secondsSinceStart_() > 15)) {
    std::cout << YELLOW_START << "ExcavatorEstimator" << GREEN_START
              << " LiDAR odometry callback is setting global cabin yaw to 0, as it was not set so far." << COLOR_END << std::endl;
    this->initYawAndPosition(compslam_T_Wl_Lk, dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getLidarFrame());
  }

  // Wrap up iteration
  odometryKm1Ptr__ = odometryKPtr;

  // Add to path message
  addToPathMsg(measWorldLidarPathPtr_, dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getWorldFrame(),
               odomLidarPtr->header.stamp, compslam_T_Wl_Lk.block<3, 1>(0, 3), graphConfigPtr_->imuBufferLength * 4);

  // Publish Path
  pubMeasWorldLidarPath_.publish(measWorldLidarPathPtr_);
}

void ExcavatorEstimator::gnssCallback_(const sensor_msgs::NavSatFix::ConstPtr& leftGnssMsgPtr,
                                       const sensor_msgs::NavSatFix::ConstPtr& rightGnssMsgPtr) {
  // Static method variables
  static Eigen::Vector3d accumulatedLeftCoordinates__(0.0, 0.0, 0.0);
  static Eigen::Vector3d accumulatedRightCoordinates__(0.0, 0.0, 0.0);
  static Eigen::Vector3d W_t_W_GnssL_km1__, W_t_W_GnssR_km1__;
  static int gnssCallbackCounter__ = 0;

  // Start
  ++gnssCallbackCounter__;
  Eigen::Vector3d leftGnssCoord = Eigen::Vector3d(leftGnssMsgPtr->latitude, leftGnssMsgPtr->longitude, leftGnssMsgPtr->altitude);
  Eigen::Vector3d rightGnssCoord = Eigen::Vector3d(rightGnssMsgPtr->latitude, rightGnssMsgPtr->longitude, rightGnssMsgPtr->altitude);
  Eigen::Vector3d estCovarianceXYZ(leftGnssMsgPtr->position_covariance[0], leftGnssMsgPtr->position_covariance[4],
                                   leftGnssMsgPtr->position_covariance[8]);
  if (!graphConfigPtr_->usingGnssFlag) {
    ROS_WARN("Received Gnss message, but usage is set to false.");
    this->activateFallbackGraph();
    return;
  } else if (gnssCallbackCounter__ < NUM_GNSS_CALLBACKS_UNTIL_START + 1) {
    // Wait until measurements got accumulated
    accumulatedLeftCoordinates__ += leftGnssCoord;
    accumulatedRightCoordinates__ += rightGnssCoord;
    if (!(gnssCallbackCounter__ % 10)) {
      std::cout << YELLOW_START << "ExcavatorEstimator" << COLOR_END << " NOT ENOUGH Gnss MESSAGES ARRIVED!" << std::endl;
    }
    return;
  } else if (gnssCallbackCounter__ == NUM_GNSS_CALLBACKS_UNTIL_START + 1) {
    gnssHandlerPtr_->initHandler(accumulatedLeftCoordinates__ / NUM_GNSS_CALLBACKS_UNTIL_START,
                                 accumulatedRightCoordinates__ / NUM_GNSS_CALLBACKS_UNTIL_START);
  }

  // Convert to cartesian coordinates
  Eigen::Vector3d W_t_W_GnssL, W_t_W_GnssR;
  gnssHandlerPtr_->convertNavSatToPositions(leftGnssCoord, rightGnssCoord, W_t_W_GnssL, W_t_W_GnssR);
  double yaw_W_C = gnssHandlerPtr_->computeYaw(W_t_W_GnssL, W_t_W_GnssR);

  // Initialization
  if (gnssCallbackCounter__ == NUM_GNSS_CALLBACKS_UNTIL_START + 1) {
    if (!this->initYawAndPosition(yaw_W_C, dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getCabinFrame(),
                                  W_t_W_GnssL, dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getLeftGnssFrame())) {
      // Decrease counter if not successfully initialized
      --gnssCallbackCounter__;
    }
  } else {
    graph_msf::UnaryMeasurement1D meas_yaw_W_C("Gnss yaw",
                                               dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getCabinFrame(),
                                               int(gnssLeftRate_), leftGnssMsgPtr->header.stamp.toSec(), yaw_W_C, gnssHeadingUnaryNoise_);
    this->addGnssHeadingMeasurement(meas_yaw_W_C);
    graph_msf::UnaryMeasurement3D meas_W_t_W_GnssL(
        "Gnss left", dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getLeftGnssFrame(), gnssLeftRate_,
        leftGnssMsgPtr->header.stamp.toSec(), W_t_W_GnssL,
        Eigen::Vector3d(gnssPositionUnaryNoise_, gnssPositionUnaryNoise_, gnssPositionUnaryNoise_));
    this->addDualGnssPositionMeasurement(meas_W_t_W_GnssL, W_t_W_GnssL_km1__, estCovarianceXYZ, true, true);
  }
  W_t_W_GnssL_km1__ = W_t_W_GnssL;
  W_t_W_GnssR_km1__ = W_t_W_GnssR;

  // Left GNSS
  /// Add to Path
  addToPathMsg(measWorldLeftGnssPathPtr_, dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getWorldFrame(),
               leftGnssMsgPtr->header.stamp, W_t_W_GnssL, graphConfigPtr_->imuBufferLength * 4);
  /// Publish path
  pubMeasWorldGnssLPath_.publish(measWorldLeftGnssPathPtr_);
  // Right GNSS
  /// Add to path
  addToPathMsg(measWorldRightGnssPathPtr_, dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getWorldFrame(),
               rightGnssMsgPtr->header.stamp, W_t_W_GnssR, graphConfigPtr_->imuBufferLength * 4);
  /// Publish path
  pubMeasWorldGnssRPath_.publish(measWorldRightGnssPathPtr_);
}

void ExcavatorEstimator::publishState_(
    const std::shared_ptr<graph_msf::SafeNavState>& navStatePtr,
    const std::shared_ptr<graph_msf::SafeNavStateWithCovarianceAndBias>& optimizedStateWithCovarianceAndBiasPtr) {
  // Lookup I->B, also influenced by rotation of cabin
  static tf::StampedTransform transform_I_B;
  tfListener_.waitForTransform(staticTransformsPtr_->getImuFrame(),
                               dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getBaseLinkFrame(), ros::Time(0),
                               ros::Duration(0.1));
  listener_.lookupTransform(staticTransformsPtr_->getImuFrame(),
                            dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getBaseLinkFrame(), ros::Time(0),
                            transform_I_B);
  // Update Imu->Base transformation
  graph_msf::tfToIsometry3(transform_I_B, staticTransformsPtr_->lv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(),
                                                                                   staticTransformsPtr_->getBaseLinkFrame()));
  // Updaate Base->Imu transformation
  staticTransformsPtr_->lv_T_frame1_frame2(staticTransformsPtr_->getBaseLinkFrame(), staticTransformsPtr_->getImuFrame()) =
      staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(), staticTransformsPtr_->getBaseLinkFrame()).inverse();

  // Publish state
  graph_msf::GraphMsfRos::publishState_(navStatePtr, optimizedStateWithCovarianceAndBiasPtr);
}

}  // namespace excavator_se