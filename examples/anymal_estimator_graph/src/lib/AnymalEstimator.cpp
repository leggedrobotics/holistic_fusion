/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "anymal_estimator_graph/AnymalEstimator.h"

// Project
#include "anymal_estimator_graph/AnymalStaticTransforms.h"

// Workspace
#include "anymal_estimator_graph/constants.h"
#include "graph_msf/measurements/BinaryMeasurementXD.h"
#include "graph_msf/measurements/UnaryMeasurementXD.h"
#include "graph_msf_ros/util/conversions.h"

namespace anymal_se {

AnymalEstimator::AnymalEstimator(const std::shared_ptr<ros::NodeHandle>& privateNodePtr) : graph_msf::GraphMsfRos(privateNodePtr) {
  REGULAR_COUT << GREEN_START << " Initializing..." << COLOR_END << std::endl;

  // Configurations ----------------------------
  // Static transforms
  staticTransformsPtr_ = std::make_shared<AnymalStaticTransforms>(privateNodePtr);

  // Set up
  if (!AnymalEstimator::setup()) {
    REGULAR_COUT << COLOR_END << " Failed to set up." << std::endl;
    throw std::runtime_error("ANYmalEstimatorGraph failed to set up.");
  }

  REGULAR_COUT << GREEN_START << " Set up successfully." << COLOR_END << std::endl;
}

bool AnymalEstimator::setup() {
  REGULAR_COUT << GREEN_START << " Setting up." << COLOR_END << std::endl;

  // Super class
  if (not GraphMsfRos::setup()) {
    throw std::runtime_error("GraphMsfRos could not be initialized");
  }

  // Read parameters ----------------------------
  AnymalEstimator::readParams_(privateNode_);

  // Wait for static transforms ----------------------------
  staticTransformsPtr_->findTransformations();

  // Publishers ----------------------------
  AnymalEstimator::initializePublishers_(privateNode_);

  // Subscribers ----------------------------
  AnymalEstimator::initializeSubscribers_(privateNode_);

  // Messages ----------------------------
  AnymalEstimator::initializeMessages_(privateNode_);

  // Server ----------------------------
  AnymalEstimator::initializeServices_(privateNode_);

  // Wrap up ----------------------------
  REGULAR_COUT << GREEN_START << " Set up successfully." << COLOR_END << std::endl;

  return true;
}

void AnymalEstimator::initializePublishers_(ros::NodeHandle& privateNode) {
  // Paths
  pubMeasMapLioPath_ = privateNode_.advertise<nav_msgs::Path>("/graph_msf/measLiDAR_path_map_imu", ROS_QUEUE_SIZE);
  pubMeasWorldGnssPath_ = privateNode_.advertise<nav_msgs::Path>("/graph_msf/measGnss_path_world_gnss", ROS_QUEUE_SIZE);
}

void AnymalEstimator::initializeSubscribers_(ros::NodeHandle& privateNode) {
  // GNSS

  // LiDAR Odometry
  // Unary
  if (useLioUnaryFlag_) {
    subLioUnary_ = privateNode_.subscribe<nav_msgs::Odometry>(
        "/lidar_odometry_topic", ROS_QUEUE_SIZE, &AnymalEstimator::lidarUnaryCallback_, this, ros::TransportHints().tcpNoDelay());

    REGULAR_COUT << COLOR_END << " Initialized LiDAR Unary Factor Odometry subscriber with topic: " << subLioUnary_.getTopic() << std::endl;
  }
  // Between
  if (useLioBetweenFlag_) {
    subLioBetween_ = privateNode_.subscribe<nav_msgs::Odometry>(
        "/lidar_odometry_topic", ROS_QUEUE_SIZE, &AnymalEstimator::lidarBetweenCallback_, this, ros::TransportHints().tcpNoDelay());
    REGULAR_COUT << COLOR_END << " Initialized LiDAR Odometry subscriber with topic: " << subLioBetween_.getTopic() << std::endl;
  }

  // GNSS
  if (useGnssUnaryFlag_) {
    subGnssUnary_ = privateNode_.subscribe<sensor_msgs::NavSatFix>("/gnss_topic", ROS_QUEUE_SIZE, &AnymalEstimator::gnssUnaryCallback_, this,
                                                              ros::TransportHints().tcpNoDelay());
    REGULAR_COUT << " Initialized Gnss subscriber with topic: " << subGnssUnary_.getTopic() << std::endl;
  }

  // Legged Odometry
  if (useLeggedBetweenFlag_) {
    subLeggedBetween_ = privateNode_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
        "/legged_odometry_topic", ROS_QUEUE_SIZE, &AnymalEstimator::leggedBetweenCallback_, this, ros::TransportHints().tcpNoDelay());
    REGULAR_COUT << COLOR_END << " Initialized Legged Odometry subscriber with topic: " << subLeggedBetween_.getTopic() << std::endl;
  }
}

void AnymalEstimator::initializeMessages_(ros::NodeHandle& privateNode) {
  // Path
  measLio_mapImuPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  measGnss_worldGnssPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
}

void AnymalEstimator::initializeServices_(ros::NodeHandle& privateNode) {
  // Nothing
  // TODO: add soft reset of the graph for on-the-go re-init.
}

void AnymalEstimator::leggedBetweenCallback_(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& leggedOdometryKPtr) {
  // Counter
  ++leggedOdometryCallbackCounter__;

  // areYawAndPositionInited() ?
  if (!areRollAndPitchInited()) {
    return;
  }

  // TODO parametrize this?
  int sampleRate = int(leggedOdometryRate_) / 20;
  if ((leggedOdometryCallbackCounter__ % 20) == 0) {
    // Eigen Type
    Eigen::Isometry3d T_O_Leg_k = Eigen::Isometry3d::Identity();

    graph_msf::geometryPoseToEigen(*leggedOdometryKPtr, T_O_Leg_k.matrix());
    double legOdometryTimeK = leggedOdometryKPtr->header.stamp.toSec();

    if (leggedOdometryCallbackCounter__ == 0) {
      T_O_Leg_km1__ = T_O_Leg_k;
      legOdometryTimeKm1__ = legOdometryTimeK;
    }

    // Compute Delta
    const Eigen::Isometry3d T_Wkm1_Wk = T_O_Leg_km1__.inverse() * T_O_Leg_k;
    // Create measurement
    graph_msf::BinaryMeasurementXD<Eigen::Isometry3d, 6> delta6DMeasurement(
        "Leg_odometry_6D", int(sampleRate), dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getLeggedOdometryFrame(),
        graph_msf::RobustNormEnum::Cauchy, 1.0, legOdometryTimeKm1__, legOdometryTimeK, T_Wkm1_Wk, legPoseBetweenNoise_);
    // Add to graph
    graph_msf::GraphMsf::addBinaryPoseMeasurement(delta6DMeasurement);

    // Provide next iteration
    T_O_Leg_km1__ = T_O_Leg_k;
    legOdometryTimeKm1__ = legOdometryTimeK;
    leggedOdometryCallbackCounter__ = 0;
  }
}

void AnymalEstimator::lidarBetweenCallback_(const nav_msgs::Odometry::ConstPtr& odomLidarPtr) {
  if (!areRollAndPitchInited()) {
    return;
  }

  // Counter
  ++lidarBetweenCallbackCounter_;

  // Convert
  Eigen::Isometry3d lio_T_M_Lk = Eigen::Isometry3d::Identity();
  graph_msf::odomMsgToEigen(*odomLidarPtr, lio_T_M_Lk.matrix());
  // Get the time
  double lidarBetweenTimeK = odomLidarPtr->header.stamp.toSec();

  if (lidarBetweenCallbackCounter_ == 0) {
    lio_T_M_Lkm1_ = lio_T_M_Lk;
    lidarBetweenTimeKm1_ = lidarBetweenTimeK;
  }

  // Add to trajectory aligner if needed.
  if (gnssHandlerPtr_->yawInitialGuessFromAlignment_) {
    trajectoryAlignmentHandler_->addLidarPose(lio_T_M_Lk.translation(), lidarBetweenTimeK);
  }

  if (lidarBetweenCallbackCounter_ <= 2) {
    return;
  } else if (!areYawAndPositionInited()) {  // Initializing
    if (!useGnssUnaryFlag_ && !useLioUnaryFlag_) {
      // Measurement
      graph_msf::UnaryMeasurementXD<Eigen::Isometry3d, 6> unary6DMeasurement(
          "Lidar_unary_6D", int(lioOdometryRate_), dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getLioOdometryFrame(),
          graph_msf::RobustNormEnum::Cauchy, 1.0, lidarBetweenTimeK, odomLidarPtr->header.frame_id, 1.0, lio_T_M_Lk, lioPoseUnaryNoise_);

      REGULAR_COUT << GREEN_START << " LiDAR odometry callback is setting global yaw, as it was not set so far." << COLOR_END << std::endl;
      this->initYawAndPosition(unary6DMeasurement);
    }
  } else {  // Already initialized --> Between factor

    // Compute Delta
    const Eigen::Isometry3d T_Lkm1_Lk = lio_T_M_Lkm1_.inverse() * lio_T_M_Lk;
    // Create measurement
    graph_msf::BinaryMeasurementXD<Eigen::Isometry3d, 6> delta6DMeasurement(
        "Lidar_between_6D", int(lioOdometryRate_), dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getLioOdometryFrame(),
        graph_msf::RobustNormEnum::Cauchy, 1.0, lidarBetweenTimeKm1_, lidarBetweenTimeK, T_Lkm1_Lk, lioPoseUnaryNoise_);
    // Add to graph
    graph_msf::GraphMsf::addBinaryPoseMeasurement(delta6DMeasurement);
  }
  // Provide for next iteration
  lio_T_M_Lkm1_ = lio_T_M_Lk;
  lidarBetweenTimeKm1_ = lidarBetweenTimeK;

  // Visualization ----------------------------
  // Add to path message
  addToPathMsg(
      measLio_mapImuPathPtr_, odomLidarPtr->header.frame_id, odomLidarPtr->header.stamp,
      (lio_T_M_Lk * staticTransformsPtr_
                        ->rv_T_frame1_frame2(dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getLioOdometryFrame(),
                                             staticTransformsPtr_->getImuFrame())
                        .matrix())
          .block<3, 1>(0, 3),
      graphConfigPtr_->imuBufferLength_ * 4);

  // Publish Path
  pubMeasMapLioPath_.publish(measLio_mapImuPathPtr_);
}

void AnymalEstimator::lidarUnaryCallback_(const nav_msgs::Odometry::ConstPtr& odomLidarPtr) {
  // Counter
  ++lidarUnaryCallbackCounter_;

  Eigen::Isometry3d lio_T_M_Lk = Eigen::Isometry3d::Identity();
  graph_msf::odomMsgToEigen(*odomLidarPtr, lio_T_M_Lk.matrix());

  // Transform to IMU frame
  double lidarUnaryTimeK = odomLidarPtr->header.stamp.toSec();

  if (gnssHandlerPtr_->yawInitialGuessFromAlignment_) {
    trajectoryAlignmentHandler_->addLidarPose(lio_T_M_Lk.translation(), lidarUnaryTimeK);
  }

  // Measurement
  graph_msf::UnaryMeasurementXD<Eigen::Isometry3d, 6> unary6DMeasurement(
      "Lidar_unary_6D", int(lioOdometryRate_), dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getLioOdometryFrame(),
      graph_msf::RobustNormEnum::None, 1.0, lidarUnaryTimeK, odomLidarPtr->header.frame_id, 1.0, lio_T_M_Lk, lioPoseUnaryNoise_);

  if (lidarUnaryCallbackCounter_ <= 2) {
    return;
  } else if (!areYawAndPositionInited()) {  // Initializing
    if (!useGnssUnaryFlag_ || (lidarUnaryCallbackCounter_ > NUM_GNSS_CALLBACKS_UNTIL_START)) {
      REGULAR_COUT << GREEN_START << " LiDAR odometry callback is setting global yaw, as it was not set so far." << COLOR_END << std::endl;
      this->initYawAndPosition(unary6DMeasurement);
    }
  } else {  // Already initialized --> unary factor
    this->addUnaryPoseMeasurement(unary6DMeasurement);
  }

  // Visualization ----------------------------
  // Add to path message
  addToPathMsg(
      measLio_mapImuPathPtr_, odomLidarPtr->header.frame_id, odomLidarPtr->header.stamp,
      (lio_T_M_Lk * staticTransformsPtr_
                        ->rv_T_frame1_frame2(dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getLioOdometryFrame(),
                                             staticTransformsPtr_->getImuFrame())
                        .matrix())
          .block<3, 1>(0, 3),
      graphConfigPtr_->imuBufferLength_ * 4);

  // Publish Path
  pubMeasMapLioPath_.publish(measLio_mapImuPathPtr_);
}

void AnymalEstimator::gnssUnaryCallback_(const sensor_msgs::NavSatFix::ConstPtr& gnssMsgPtr) {

  // Counter
  ++gnssCallbackCounter_;

  // Convert to Eigen
  Eigen::Vector3d gnssCoord = Eigen::Vector3d(gnssMsgPtr->latitude, gnssMsgPtr->longitude, gnssMsgPtr->altitude);
  Eigen::Vector3d estStdDevXYZ(sqrt(gnssMsgPtr->position_covariance[0]), sqrt(gnssMsgPtr->position_covariance[4]),
                               sqrt(gnssMsgPtr->position_covariance[8]));

  // Initialize GNSS Handler
  if (gnssCallbackCounter_ < NUM_GNSS_CALLBACKS_UNTIL_START) {  // Accumulate measurements
    // Wait until measurements got accumulated
    accumulatedGnssCoordinates_ += gnssCoord;
    if (!(gnssCallbackCounter_ % 10)) {
      std::cout << YELLOW_START << "AnymalEstimator" << COLOR_END << " NOT ENOUGH GNSS MESSAGES ARRIVED!" << std::endl;
    }
    return;
  } else if (gnssCallbackCounter_ == NUM_GNSS_CALLBACKS_UNTIL_START) {  // Initialize GNSS Handler
    gnssHandlerPtr_->initHandler(accumulatedGnssCoordinates_ / NUM_GNSS_CALLBACKS_UNTIL_START);
    std::cout << YELLOW_START << "AnymalEstimator" << COLOR_END << " GNSS Handler initialized." << std::endl;
    return;
  }

  // Convert to cartesian coordinates
  Eigen::Vector3d W_t_W_Gnss;
  gnssHandlerPtr_->convertNavSatToPosition(gnssCoord, W_t_W_Gnss);

  // Inital world yaw initialization options.
  if (!areYawAndPositionInited()) {  // 1: Initialization
    // 0: Default
    double initYaw_W_Base{0.0}; // Default is 0 yaw
    // 1: From file
    if (gnssHandlerPtr_->useYawInitialGuessFromFile_) {
      initYaw_W_Base = gnssHandlerPtr_->globalYawDegFromFile_ / 180.0 * M_PI;
    } else if (gnssHandlerPtr_->yawInitialGuessFromAlignment_) { // 2: From alignment
      // Adding the GNSS measurement
      trajectoryAlignmentHandler_->addGnssPose(W_t_W_Gnss, gnssMsgPtr->header.stamp.toSec());
      // In radians.
      if (!(trajectoryAlignmentHandler_->alignTrajectories(initYaw_W_Base))) {
        return;
      }
      std::cout << GREEN_START << "Trajectory Alignment Successful. Obtained Yaw Value : " << COLOR_END << initYaw_W_Base << std::endl;
    }

    // Initialization
    if (not this->initYawAndPosition(initYaw_W_Base, W_t_W_Gnss, staticTransformsPtr_->getWorldFrame(),
                                     dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getBaseLinkFrame(),
                                     dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getGnssFrame())) {
      // Make clear that this was not successful
      REGULAR_COUT << RED_START << " GNSS initialization of yaw and position failed." << std::endl;
    } else {
      REGULAR_COUT << GREEN_START << " GNSS initialization of yaw and position successful." << std::endl;
    }
  } else {  // 2: Unary factor
    graph_msf::UnaryMeasurementXD<Eigen::Vector3d, 3> meas_W_t_W_Gnss(
        "GnssPosition", int(gnssRate_), dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getGnssFrame(),
        graph_msf::RobustNormEnum::None, 1.0, gnssMsgPtr->header.stamp.toSec(), staticTransformsPtr_->getWorldFrame(), 1.0, W_t_W_Gnss,
        estStdDevXYZ);
    // graph_msf::GraphMsfInterface::addGnssPositionMeasurement_(meas_W_t_W_Gnss);
    this->addUnaryPosition3Measurement(meas_W_t_W_Gnss);
  }


  /// Add GNSS to Path
  addToPathMsg(measGnss_worldGnssPathPtr_, staticTransformsPtr_->getWorldFrame(), gnssMsgPtr->header.stamp, W_t_W_Gnss,
               graphConfigPtr_->imuBufferLength_ * 4);
  /// Publish path
  pubMeasWorldGnssPath_.publish(measGnss_worldGnssPathPtr_);
}

}  // namespace anymal_se