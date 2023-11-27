/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "excavator_holistic_graph/ExcavatorEstimator.h"

// Project
#include "excavator_holistic_graph/ExcavatorStaticTransforms.h"
#include "excavator_holistic_graph/constants.h"

// Workspace
#include "graph_msf/measurements/BinaryMeasurementXD.h"
#include "graph_msf/measurements/UnaryMeasurementXD.h"
#include "graph_msf_ros/util/conversions.h"

namespace excavator_se {

ExcavatorEstimator::ExcavatorEstimator(std::shared_ptr<ros::NodeHandle> privateNodePtr) : graph_msf::GraphMsfRos(privateNodePtr) {
  std::cout << YELLOW_START << "ExcavatorEstimator" << GREEN_START << " Setting up." << COLOR_END << std::endl;

  // Configurations ----------------------------
  // Static Transforms
  staticTransformsPtr_ = std::make_shared<ExcavatorStaticTransforms>(privateNodePtr);

  // GNSS Handler
  gnssHandlerPtr_ = std::make_shared<graph_msf::GnssHandler>();

  // Setup
  if (not ExcavatorEstimator::setup()) {
    REGULAR_COUT << RED_START << " Failed to set up." << COLOR_END << std::endl;
    throw std::runtime_error("ExcavatorEstimator could not be initialized");
  }

  std::cout << YELLOW_START << "ExcavatorEstimator" << GREEN_START << " Set up successfully." << COLOR_END << std::endl;
}

bool ExcavatorEstimator::setup() {
  REGULAR_COUT << GREEN_START << " Setting up." << COLOR_END << std::endl;

  // Read parameters ----------------------------
  ExcavatorEstimator::readParams_(privateNode_);

  // Super class
  if (not graph_msf::GraphMsfRos::setup()) {
    throw std::runtime_error("GraphMsfRos could not be initialized");
  }

  // Publishers ----------------------------
  ExcavatorEstimator::initializePublishers_(privateNode_);

  // Subscribers ----------------------------
  ExcavatorEstimator::initializeSubscribers_(privateNode_);

  // Messages ----------------------------
  ExcavatorEstimator::initializeMessages_(privateNode_);

  // Static Transforms
  staticTransformsPtr_->findTransformations();

  // Wrap up ----------------------------
  REGULAR_COUT << GREEN_START << " Set up successfully." << COLOR_END << std::endl;

  return true;
}

void ExcavatorEstimator::initializePublishers_(ros::NodeHandle& privateNode) {
  // Status
  REGULAR_COUT << GREEN_START << " Initializing Publishers..." << COLOR_END << std::endl;

  // Paths
  pubMeasWorldGnssLPath_ = privateNode.advertise<nav_msgs::Path>("/graph_msf/measGnssL_path_world_gnssL", ROS_QUEUE_SIZE);
  pubMeasWorldGnssRPath_ = privateNode.advertise<nav_msgs::Path>("/graph_msf/measGnssR_path_world_gnssR", ROS_QUEUE_SIZE);
  pubMeasMapLioPath_ = privateNode.advertise<nav_msgs::Path>("/graph_msf/measLiDAR_path_map_imu", ROS_QUEUE_SIZE);
}

void ExcavatorEstimator::initializeSubscribers_(ros::NodeHandle& privateNode) {
  // LiDAR Odometry
  if (useLioOdometryFlag_) {
    subLidarOdometry_ = privateNode.subscribe<nav_msgs::Odometry>(
        "/lidar_odometry_topic", ROS_QUEUE_SIZE, &ExcavatorEstimator::lidarOdometryCallback_, this, ros::TransportHints().tcpNoDelay());
    std::cout << YELLOW_START << "ExcavatorEstimator" << COLOR_END << " Initialized LiDAR Odometry subscriber." << std::endl;
  }

  // GNSS
  if (useLeftGnssFlag_ || useRightGnssFlag_) {
    subGnssLeft_.subscribe(privateNode, "/gnss_topic_1", ROS_QUEUE_SIZE);
    subGnssRight_.subscribe(privateNode, "/gnss_topic_2", ROS_QUEUE_SIZE);
    gnssExactSyncPtr_.reset(
        new message_filters::Synchronizer<GnssExactSyncPolicy>(GnssExactSyncPolicy(ROS_QUEUE_SIZE), subGnssLeft_, subGnssRight_));
    gnssExactSyncPtr_->registerCallback(boost::bind(&ExcavatorEstimator::gnssCallback_, this, _1, _2));
    std::cout << YELLOW_START << "FactorGraphFiltering" << COLOR_END << " Initialized Gnss subscriber (for both Gnss topics)." << std::endl;
  }
}

void ExcavatorEstimator::initializeMessages_(ros::NodeHandle& privateNode) {
  // Status
  REGULAR_COUT << GREEN_START << " Initializing Messages..." << COLOR_END << std::endl;

  // Paths
  measGnss_worldGnssLPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  measGnss_worldGnssRPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  measLio_mapImuPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);  // Preintegrated Estimate of LiDAR Frame
}

void ExcavatorEstimator::lidarOdometryCallback_(const nav_msgs::Odometry::ConstPtr& odomLidarPtr) {
  // Static members
  static int lidarOdometryCallbackCounter__ = -1;
  static Eigen::Isometry3d lio_T_M_LKm1__;
  static double lidarOdometryTimeKm1__ = 0;

  // Counter
  ++lidarOdometryCallbackCounter__;

  Eigen::Isometry3d lio_T_M_Lk;
  graph_msf::odomMsgToEigen(*odomLidarPtr, lio_T_M_Lk.matrix());

  // Transform to IMU frame
  double lidarOdometryTimeK = odomLidarPtr->header.stamp.toSec();

  if (lidarOdometryCallbackCounter__ <= 2) {
    return;
  } else if (areYawAndPositionInited()) {  // Already initialized --> unary factor
    // Measurement
    graph_msf::UnaryMeasurementXD<Eigen::Isometry3d, 6> unary6DMeasurement(
        "Lidar_unary_6D", int(lioOdometryRate_), lidarOdometryTimeK, odomLidarPtr->header.frame_id,
        dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getLioOdometryFrame(), lio_T_M_Lk, lioPoseUnaryNoise_);
    this->addUnaryPoseMeasurement(unary6DMeasurement);
  } else if (!(useLeftGnssFlag_ || useRightGnssFlag_) || secondsSinceStart_() > 15) {  // Initializing
    REGULAR_COUT << GREEN_START << " LiDAR odometry callback is setting global yaw, as it was not set so far." << COLOR_END << std::endl;
    graph_msf::UnaryMeasurementXD<Eigen::Isometry3d, 6> unary6DMeasurement(
        "Lidar_unary_6D", int(lioOdometryRate_), lidarOdometryTimeK, odomLidarPtr->header.frame_id,
        dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getLioOdometryFrame(), lio_T_M_Lk, lioPoseUnaryNoise_);
    this->initYawAndPosition(unary6DMeasurement);
  }

  // Wrap up iteration
  lio_T_M_LKm1__ = lio_T_M_Lk;
  lidarOdometryTimeKm1__ = lidarOdometryTimeK;

  // Visualization ----------------------------
  // Add to path message
  addToPathMsg(
      measLio_mapImuPathPtr_, odomLidarPtr->header.frame_id, odomLidarPtr->header.stamp,
      (lio_T_M_Lk * staticTransformsPtr_
                        ->rv_T_frame1_frame2(dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getLioOdometryFrame(),
                                             staticTransformsPtr_->getImuFrame())
                        .matrix())
          .block<3, 1>(0, 3),
      graphConfigPtr_->imuBufferLength * 4);

  // Publish Path
  pubMeasMapLioPath_.publish(measLio_mapImuPathPtr_);
}

void ExcavatorEstimator::gnssCallback_(const sensor_msgs::NavSatFix::ConstPtr& leftGnssMsgPtr,
                                       const sensor_msgs::NavSatFix::ConstPtr& rightGnssMsgPtr) {
  // Static method variables
  static Eigen::Vector3d accumulatedLeftCoordinates__(0.0, 0.0, 0.0);
  static Eigen::Vector3d accumulatedRightCoordinates__(0.0, 0.0, 0.0);
  static bool gnssHeadingHealthyFlag__ = true;
  static bool leftGnssPositionHealthyFlag__ = true;
  static bool rightGnssPositionHealthyFlag__ = true;
  // static Eigen::Vector3d W_t_W_GnssL_km1__, W_t_W_GnssR_km1__;
  static int gnssCallbackCounter__ = 0;

  // Start
  ++gnssCallbackCounter__;
  Eigen::Vector3d leftGnssCoord = Eigen::Vector3d(leftGnssMsgPtr->latitude, leftGnssMsgPtr->longitude, leftGnssMsgPtr->altitude);
  Eigen::Vector3d rightGnssCoord = Eigen::Vector3d(rightGnssMsgPtr->latitude, rightGnssMsgPtr->longitude, rightGnssMsgPtr->altitude);
  Eigen::Vector3d leftGnssCovarianceXYZ(leftGnssMsgPtr->position_covariance[0], leftGnssMsgPtr->position_covariance[4],
                                        leftGnssMsgPtr->position_covariance[8]);
  Eigen::Vector3d rightGnssCovarianceXYZ(rightGnssMsgPtr->position_covariance[0], rightGnssMsgPtr->position_covariance[4],
                                         rightGnssMsgPtr->position_covariance[8]);

  // Workflow
  if (gnssCallbackCounter__ <= NUM_GNSS_CALLBACKS_UNTIL_START) {  // Accumulate measurements at the beginning
    // Wait until measurements got accumulated
    accumulatedLeftCoordinates__ += leftGnssCoord;
    accumulatedRightCoordinates__ += rightGnssCoord;
    if (!(gnssCallbackCounter__ % 10)) {
      REGULAR_COUT << " NOT ENOUGH Gnss MESSAGES ARRIVED!" << std::endl;
    }
    return;
  } else if (gnssCallbackCounter__ ==
             NUM_GNSS_CALLBACKS_UNTIL_START + 1) {  // Initialize the GNSS handler with the initial position and initialize the graph
    Eigen::Vector3d meanLeftCoordinates = accumulatedLeftCoordinates__ / NUM_GNSS_CALLBACKS_UNTIL_START;
    Eigen::Vector3d meanRightCoordinates = accumulatedRightCoordinates__ / NUM_GNSS_CALLBACKS_UNTIL_START;
    gnssHandlerPtr_->initHandler(meanLeftCoordinates, meanRightCoordinates);
  }

  // Convert to cartesian coordinates --> Needed for every ramaining step
  Eigen::Vector3d W_t_W_GnssL, W_t_W_GnssR;
  gnssHandlerPtr_->convertNavSatToPositions(leftGnssCoord, rightGnssCoord, W_t_W_GnssL, W_t_W_GnssR);
  double yaw_W_C = gnssHandlerPtr_->computeYaw(W_t_W_GnssL, W_t_W_GnssR);

  // State Machine
  if (!areYawAndPositionInited() && areRollAndPitchInited()) {  // Try to initialize yaw and position if not done already
    if (this->initYawAndPosition(yaw_W_C, W_t_W_GnssL, staticTransformsPtr_->getWorldFrame(),
                                 dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getCabinFrame(),
                                 dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getLeftGnssFrame())) {
      REGULAR_COUT << " Set yaw and position successfully." << std::endl;
    }
  } else {  // Already initialized --> add to graph
    // Measurement type 1: GNSS Yaw ----------------------------------------------------
    if (useGnssYawFlag_) {
      // Create yaw measurement and add it
      Eigen::Matrix<double, 1, 1> gnssHeadingUnaryNoise(std::max(5.0 * leftGnssCovarianceXYZ.maxCoeff(), gnssHeadingUnaryNoise_));
      graph_msf::UnaryMeasurementXD<double, 1> meas_yaw_W_C(
          "GnssYaw", int(gnssRate_), leftGnssMsgPtr->header.stamp.toSec(), staticTransformsPtr_->getWorldFrame() + "_ENU",
          dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getCabinFrame(), yaw_W_C, gnssHeadingUnaryNoise,
          GNSS_HEADING_COVARIANCE_VIOLATION_THRESHOLD);
      if (!this->addHeadingMeasurement(meas_yaw_W_C)) {
        if (gnssHeadingHealthyFlag__) {
          REGULAR_COUT << RED_START << " Gnss heading measurement not added due to large covariance." << COLOR_END << std::endl;
          gnssHeadingHealthyFlag__ = false;
        }
      } else if (!gnssHeadingHealthyFlag__) {
        REGULAR_COUT << GREEN_START << " Gnss heading measurement returned." << COLOR_END << std::endl;
        gnssHeadingHealthyFlag__ = true;
      }
    }
    // Measurement type 2: GNSS Left Position ----------------------------------------------------
    if (useLeftGnssFlag_) {
      leftGnssCovarianceXYZ = Eigen::Vector3d(std::max(leftGnssCovarianceXYZ(0), gnssPositionUnaryNoise_),
                                              std::max(leftGnssCovarianceXYZ(1), gnssPositionUnaryNoise_),
                                              std::max(leftGnssCovarianceXYZ(2), gnssPositionUnaryNoise_));
      graph_msf::UnaryMeasurementXD<Eigen::Vector3d, 3> meas_W_t_W_GnssL(
          "GnssLeftPosition", int(gnssRate_), leftGnssMsgPtr->header.stamp.toSec(), staticTransformsPtr_->getWorldFrame() + "_ENU",
          dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getLeftGnssFrame(), W_t_W_GnssL, leftGnssCovarianceXYZ,
          GNSS_POSITION_COVARIANCE_VIOLATION_THRESHOLD);
      if (!this->addPositionMeasurement(meas_W_t_W_GnssL)) {
        if (leftGnssPositionHealthyFlag__) {
          REGULAR_COUT << RED_START << " Gnss left position measurement not added." << COLOR_END << std::endl;
          leftGnssPositionHealthyFlag__ = false;
        }
      } else if (!leftGnssPositionHealthyFlag__) {
        REGULAR_COUT << GREEN_START << " Gnss left position measurement returned." << COLOR_END << std::endl;
        leftGnssPositionHealthyFlag__ = true;
      }
    }
    // Measurement type 3: GNSS Right Position ----------------------------------------------------
    if (useRightGnssFlag_) {
      rightGnssCovarianceXYZ = Eigen::Vector3d(std::max(rightGnssCovarianceXYZ(0), gnssPositionUnaryNoise_),
                                               std::max(rightGnssCovarianceXYZ(1), gnssPositionUnaryNoise_),
                                               std::max(rightGnssCovarianceXYZ(2), gnssPositionUnaryNoise_));
      graph_msf::UnaryMeasurementXD<Eigen::Vector3d, 3> meas_W_t_W_GnssR(
          "GnssRightPosition", int(gnssRate_), rightGnssMsgPtr->header.stamp.toSec(), staticTransformsPtr_->getWorldFrame() + "_ENU",
          dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getRightGnssFrame(), W_t_W_GnssR, rightGnssCovarianceXYZ,
          GNSS_POSITION_COVARIANCE_VIOLATION_THRESHOLD);
      if (!this->addPositionMeasurement(meas_W_t_W_GnssR)) {
        if (rightGnssPositionHealthyFlag__) {
          REGULAR_COUT << RED_START << " Gnss right position measurement not added." << COLOR_END << std::endl;
          rightGnssPositionHealthyFlag__ = false;
        }
      } else if (!rightGnssPositionHealthyFlag__) {
        REGULAR_COUT << GREEN_START << " Gnss right position measurement returned." << COLOR_END << std::endl;
        rightGnssPositionHealthyFlag__ = true;
      }
    }
    // Visualizations ------------------------------------------------------------
    // Left GNSS
    if (useLeftGnssFlag_) {
      addToPathMsg(measGnss_worldGnssLPathPtr_, staticTransformsPtr_->getWorldFrame(), leftGnssMsgPtr->header.stamp, W_t_W_GnssL,
                   graphConfigPtr_->imuBufferLength * 4);
      pubMeasWorldGnssLPath_.publish(measGnss_worldGnssLPathPtr_);
    }
    // Right GNSS
    if (useRightGnssFlag_) {
      addToPathMsg(measGnss_worldGnssRPathPtr_, staticTransformsPtr_->getWorldFrame(), rightGnssMsgPtr->header.stamp, W_t_W_GnssR,
                   graphConfigPtr_->imuBufferLength * 4);
      pubMeasWorldGnssRPath_.publish(measGnss_worldGnssRPathPtr_);
    }
  }
}

void ExcavatorEstimator::publishState_(
    const std::shared_ptr<graph_msf::SafeIntegratedNavState>& preIntegratedNavStatePtr,
    const std::shared_ptr<graph_msf::SafeNavStateWithCovarianceAndBias>& optimizedStateWithCovarianceAndBiasPtr) {
  // Lookup I->B, also influenced by rotation of cabin
  static tf::StampedTransform transform_I_B;
  tfListener_.waitForTransform(staticTransformsPtr_->getImuFrame(),
                               dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getBaseLinkFrame(), ros::Time(0),
                               ros::Duration(0.1));
  tfListener_.lookupTransform(staticTransformsPtr_->getImuFrame(),
                              dynamic_cast<ExcavatorStaticTransforms*>(staticTransformsPtr_.get())->getBaseLinkFrame(), ros::Time(0),
                              transform_I_B);
  // Update Imu->Base transformation
  graph_msf::tfToIsometry3(transform_I_B, staticTransformsPtr_->lv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(),
                                                                                   staticTransformsPtr_->getBaseLinkFrame()));
  // Updaate Base->Imu transformation
  staticTransformsPtr_->lv_T_frame1_frame2(staticTransformsPtr_->getBaseLinkFrame(), staticTransformsPtr_->getImuFrame()) =
      staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(), staticTransformsPtr_->getBaseLinkFrame()).inverse();

  // Publish state
  graph_msf::GraphMsfRos::publishState_(preIntegratedNavStatePtr, optimizedStateWithCovarianceAndBiasPtr);
}

}  // namespace excavator_se