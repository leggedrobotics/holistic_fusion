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
  // LiDAR Odometry
  if (useLioFlag_) {
    if (enforceLIOasBetweenMeasurement_) {
      subLidarOdometry_ = privateNode_.subscribe<nav_msgs::Odometry>("/lidar_odometry_topic", ROS_QUEUE_SIZE,
                                                                     &AnymalEstimator::lidarBetweenOdometryCallback_, this,
                                                                     ros::TransportHints().tcpNoDelay());

    } else {
      subLidarOdometry_ = privateNode_.subscribe<nav_msgs::Odometry>(
          "/lidar_odometry_topic", ROS_QUEUE_SIZE, &AnymalEstimator::lidarOdometryCallback_, this, ros::TransportHints().tcpNoDelay());
    }

    REGULAR_COUT << COLOR_END << " Initialized LiDAR Odometry subscriber with topic: " << subLidarOdometry_.getTopic() << std::endl;
  }

  // GNSS
  if (useGnssFlag_) {
    subGnss_ = privateNode_.subscribe<sensor_msgs::NavSatFix>("/gnss_topic", ROS_QUEUE_SIZE, &AnymalEstimator::gnssCallback_, this,
                                                              ros::TransportHints().tcpNoDelay());
    REGULAR_COUT << " Initialized Gnss subscriber with topic: " << subGnss_.getTopic() << std::endl;
  }

  // Legged Odometry
  if (useLeggedOdometryFlag_) {
    leggedOdometry_ = privateNode_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
        "/legged_odometry_topic", ROS_QUEUE_SIZE, &AnymalEstimator::leggedOdometryCallback_, this, ros::TransportHints().tcpNoDelay());
    REGULAR_COUT << COLOR_END << " Initialized Legged Odometry subscriber with topic: " << leggedOdometry_.getTopic() << std::endl;
  }
}

void AnymalEstimator::initializeMessages_(ros::NodeHandle& privateNode) {
  // Path
  measLio_mapImuPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  measGnss_worldGnssPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
}

void AnymalEstimator::initializeServices_(ros::NodeHandle& privateNode) {
  // Nothing
  // TODO add soft reset of the graph for on-the-go re-init.
}

void AnymalEstimator::leggedOdometryCallback_(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& leggedOdometryKPtr) {
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

void AnymalEstimator::lidarBetweenOdometryCallback_(const nav_msgs::Odometry::ConstPtr& odomLidarPtr) {
  if (!areRollAndPitchInited()) {
    return;
  }

  // Counter
  ++lidarOdometryCallbackCounter__;

  // Convert
  Eigen::Isometry3d lio_T_M_Lk = Eigen::Isometry3d::Identity();
  graph_msf::odomMsgToEigen(*odomLidarPtr, lio_T_M_Lk.matrix());
  // Get the time
  double lidarOdometryTimeK = odomLidarPtr->header.stamp.toSec();

  if (lidarOdometryCallbackCounter__ == 0) {
    lio_T_M_Lkm1__ = lio_T_M_Lk;
    lidarOdometryTimeKm1__ = lidarOdometryTimeK;
  }

  // Add to trajectory aligner if needed.
  if (gnssHandlerPtr_->yawInitialGuessFromAlignment_) {
    trajectoryAlignmentHandler_->addLidarPose(lio_T_M_Lk.translation(), lidarOdometryTimeK);
  }

  if (lidarOdometryCallbackCounter__ <= 2) {
    return;
  } else if (!areYawAndPositionInited()) {  // Initializing
    if (!useGnssFlag_) {
      // Measurement
      graph_msf::UnaryMeasurementXD<Eigen::Isometry3d, 6> unary6DMeasurement(
          "Lidar_unary_6D", int(lioOdometryRate_), dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getLioOdometryFrame(),
          graph_msf::RobustNormEnum::Cauchy, 1.0, lidarOdometryTimeK, odomLidarPtr->header.frame_id, 1.0, lio_T_M_Lk, lioPoseUnaryNoise_);

      REGULAR_COUT << GREEN_START << " LiDAR odometry callback is setting global yaw, as it was not set so far." << COLOR_END << std::endl;
      this->initYawAndPosition(unary6DMeasurement);
    }
  } else {  // Already initialized --> Between factor

    // Compute Delta
    const Eigen::Isometry3d T_Wkm1_Wk = lio_T_M_Lkm1__.inverse() * lio_T_M_Lk;
    // Create measurement
    graph_msf::BinaryMeasurementXD<Eigen::Isometry3d, 6> delta6DMeasurement(
        "Lidar_between_6D", int(lioOdometryRate_), dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getLioOdometryFrame(),
        graph_msf::RobustNormEnum::Cauchy, 1.0, lidarOdometryTimeKm1__, lidarOdometryTimeK, T_Wkm1_Wk, lioPoseUnaryNoise_);
    // Add to graph
    graph_msf::GraphMsf::addBinaryPoseMeasurement(delta6DMeasurement);
  }
  // Provide next iteration
  lio_T_M_Lkm1__ = lio_T_M_Lk;
  lidarOdometryTimeKm1__ = lidarOdometryTimeK;
  // leggedOdometryCallbackCounter__ = 0;

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

void AnymalEstimator::lidarOdometryCallback_(const nav_msgs::Odometry::ConstPtr& odomLidarPtr) {
  // Counter
  ++lidarOdometryCallbackCounter__;

  Eigen::Isometry3d lio_T_M_Lk = Eigen::Isometry3d::Identity();
  graph_msf::odomMsgToEigen(*odomLidarPtr, lio_T_M_Lk.matrix());

  // Transform to IMU frame
  double lidarOdometryTimeK = odomLidarPtr->header.stamp.toSec();

  if (gnssHandlerPtr_->yawInitialGuessFromAlignment_) {
    trajectoryAlignmentHandler_->addLidarPose(lio_T_M_Lk.translation(), lidarOdometryTimeK);
  }

  // Measurement
  graph_msf::UnaryMeasurementXD<Eigen::Isometry3d, 6> unary6DMeasurement(
      "Lidar_unary_6D", int(lioOdometryRate_), dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getLioOdometryFrame(),
      graph_msf::RobustNormEnum::Cauchy, 1.0, lidarOdometryTimeK, odomLidarPtr->header.frame_id, 1.0, lio_T_M_Lk, lioPoseUnaryNoise_);

  // Expression factors not implemented yet.
  /*graph_msf::UnaryMeasurementXD<Eigen::Isometry3d, 6> unary6DMeasurement(
      "Lidar_unary_6D", int(lioOdometryRate_), dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getLioOdometryFrame(),
      graph_msf::RobustNormEnum::Cauchy, 1.0, lidarOdometryTimeK, odomLidarPtr->header.frame_id, 1.0, lio_T_M_Lk,
      Eigen::Vector3d(lioPoseUnaryNoise_, lioPoseUnaryNoise_, lioPoseUnaryNoise_));*/

  if (lidarOdometryCallbackCounter__ <= 2) {
    return;
  } else if (!areYawAndPositionInited()) {  // Initializing
    if (!useGnssFlag_ || lidarOdometryCallbackCounter__ > NUM_GNSS_CALLBACKS_UNTIL_START) {
      REGULAR_COUT << GREEN_START << " LiDAR odometry callback is setting global yaw, as it was not set so far." << COLOR_END << std::endl;
      this->initYawAndPosition(unary6DMeasurement);
    }
  } else {  // Already initialized --> unary factor
    this->addUnaryPoseMeasurement(unary6DMeasurement);
    // Expression factors not implemented yet.
    // this->addPositionMeasurement(unary6DMeasurement);
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

void AnymalEstimator::gnssCallback_(const sensor_msgs::NavSatFix::ConstPtr& gnssMsgPtr) {
  // Static method variables
  static Eigen::Vector3d accumulatedCoordinates__(0.0, 0.0, 0.0);
  static int gnssCallbackCounter__ = 0;
  static Eigen::Vector3d last_W_t_W_Gnss__;

  // Counter
  ++gnssCallbackCounter__;

  // Convert to Eigen
  Eigen::Vector3d gnssCoord = Eigen::Vector3d(gnssMsgPtr->latitude, gnssMsgPtr->longitude, gnssMsgPtr->altitude);
  Eigen::Vector3d estStdDevXYZ(sqrt(gnssMsgPtr->position_covariance[0]), sqrt(gnssMsgPtr->position_covariance[4]),
                               sqrt(gnssMsgPtr->position_covariance[8]));

  // Initialize GNSS Handler
  if (gnssCallbackCounter__ <= NUM_GNSS_CALLBACKS_UNTIL_START) {  // Accumulate measurements
    // Wait until measurements got accumulated
    accumulatedCoordinates__ += gnssCoord;
    if (!(gnssCallbackCounter__ % 10)) {
      std::cout << YELLOW_START << "AnymalEstimator" << COLOR_END << " NOT ENOUGH Gnss MESSAGES ARRIVED!" << std::endl;
    }
    return;
  } else if (gnssCallbackCounter__ == NUM_GNSS_CALLBACKS_UNTIL_START + 1) {  // Initialize GNSS Handler
    gnssHandlerPtr_->initHandler(accumulatedCoordinates__ / NUM_GNSS_CALLBACKS_UNTIL_START);
    std::cout << YELLOW_START << "AnymalEstimator" << COLOR_END << " GNSS Handler initialized." << std::endl;
    return;
  }

  // Convert to cartesian coordinates
  Eigen::Vector3d W_t_W_Gnss;
  gnssHandlerPtr_->convertNavSatToPosition(gnssCoord, W_t_W_Gnss);

  // Inital world yaw initialization options.
  if (!areYawAndPositionInited()) {  // 1: Initialization

    double initYaw_W_Base{0.0};

    if (gnssHandlerPtr_->useYawInitialGuessFromFile_) {
      initYaw_W_Base = gnssHandlerPtr_->globalAttitudeYawFromFile_ / 180.0 * M_PI;

    } else if (gnssHandlerPtr_->yawInitialGuessFromAlignment_) {
      // Adding the GNSS measurement
      trajectoryAlignmentHandler_->addGnssPose(W_t_W_Gnss, gnssMsgPtr->header.stamp.toSec());

      if (!useLioFlag_) {
        std::cout << RED_START << "LIO odometry is off. But alignment is on, doesnt make sense. " << COLOR_END << std::endl;
        return;
      }

      // In radians.
      if (!(trajectoryAlignmentHandler_->alignTrajectories(initYaw_W_Base))) {
        return;
      }
      std::cout << GREEN_START << "Trajectory Alignment successfull Yaw Value : " << COLOR_END << initYaw_W_Base << std::endl;
    }

    // External yaw test.
    /*if ((gnssCallbackCounter__ % 150) == 0) {
      initYaw_W_Base = -90.0;
      this->initYawAndPosition(initYaw_W_Base, staticTransformsPtr_->getWorldFrame(),
                              dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getBaseLinkFrame());
    }*/

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
        graph_msf::RobustNormEnum::Cauchy, 1.0, gnssMsgPtr->header.stamp.toSec(), staticTransformsPtr_->getWorldFrame(), 1.0, W_t_W_Gnss,
        estStdDevXYZ);
    // graph_msf::GraphMsfInterface::addGnssPositionMeasurement_(meas_W_t_W_Gnss);
    this->addUnaryPosition3Measurement(meas_W_t_W_Gnss);
  }

  // Last GNSS measurement
  last_W_t_W_Gnss__ = W_t_W_Gnss;

  /// Add GNSS to Path
  addToPathMsg(measGnss_worldGnssPathPtr_, staticTransformsPtr_->getWorldFrame(), gnssMsgPtr->header.stamp, W_t_W_Gnss,
               graphConfigPtr_->imuBufferLength_ * 4);
  /// Publish path
  pubMeasWorldGnssPath_.publish(measGnss_worldGnssPathPtr_);
}

}  // namespace anymal_se