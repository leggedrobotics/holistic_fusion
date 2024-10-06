/*
Copyright 2024 by Julian Nubert & Timon Mathis, Robotic Systems Lab & Autonomous Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "atn_position3_fuser/Position3Estimator.h"

// Project
#include "atn_position3_fuser/Position3StaticTransforms.h"
#include "atn_position3_fuser/constants.h"

// Workspace
#include "graph_msf/measurements/BinaryMeasurementXD.h"
#include "graph_msf/measurements/UnaryMeasurementXD.h"
#include "graph_msf_ros/util/conversions.h"

namespace position3_se {

Position3Estimator::Position3Estimator(std::shared_ptr<ros::NodeHandle> privateNodePtr) : graph_msf::GraphMsfRos(privateNodePtr) {
  REGULAR_COUT << GREEN_START << " Position3Estimator-Constructor called." << COLOR_END << std::endl;

  // Configurations ----------------------------
  // Static Transforms
  staticTransformsPtr_ = std::make_shared<Position3StaticTransforms>(privateNodePtr);

  // GNSS Handler
  gnssHandlerPtr_ = std::make_shared<graph_msf::GnssHandler>();

  // Setup
  Position3Estimator::setup();
}

//---------------------------------------------------------------
void Position3Estimator::setup() {
  REGULAR_COUT << GREEN_START << " Position3Estimator-Setup called." << COLOR_END << std::endl;

  // Read parameters ----------------------------
  Position3Estimator::readParams(privateNode);

  // Super class
  GraphMsfRos::setup(staticTransformsPtr_);

  // Publishers ----------------------------
  Position3Estimator::initializePublishers(privateNode);

  // Subscribers ----------------------------
  Position3Estimator::initializeSubscribers(privateNode);

  // Messages ----------------------------
  Position3Estimator::initializeMessages(privateNode);

  // Services ----------------------------
  Position3Estimator::initializeServices(privateNode);

  // Static Transforms
  staticTransformsPtr_->findTransformations();

  // Wrap up ----------------------------
  REGULAR_COUT << GREEN_START << " Set up successfully." << COLOR_END << std::endl;
}

//---------------------------------------------------------------
void Position3Estimator::initializePublishers(ros::NodeHandle& privateNode) {
  // Status
  REGULAR_COUT << GREEN_START << " Initializing Publishers..." << COLOR_END << std::endl;

  // Paths
  if constexpr (constexprUsePrismPositionUnaryFlag_) {
    pubMeasWorldPrismPositionPath_ = privateNode.advertise<nav_msgs::Path>("/graph_msf/measPosition_path_world_prism", ROS_QUEUE_SIZE);
  }
  if constexpr (constexprUseGnssPositionUnaryFlag_) {
    pubMeasWorldGnssPositionPath_ = privateNode.advertise<nav_msgs::Path>("/graph_msf/measPosition_path_world_gnss", ROS_QUEUE_SIZE);
  }
  if constexpr (constexprUseGnssOfflinePoseUnaryFlag_) {
    pubMeasWorldGnssOfflinePosePath_ =
        privateNode.advertise<nav_msgs::Path>("/graph_msf/measPosition_path_world_gnss_offline_pose", ROS_QUEUE_SIZE);
  }
}

void Position3Estimator::initializeSubscribers(ros::NodeHandle& privateNode) {
  // Prism Position
  if constexpr (constexprUsePrismPositionUnaryFlag_) {
    subPrismPosition_ = privateNode.subscribe<geometry_msgs::PointStamped>(
        "/prism_position_topic", ROS_QUEUE_SIZE, &Position3Estimator::prismPositionCallback_, this, ros::TransportHints().tcpNoDelay());
  }
  // GNSS
  if constexpr (constexprUseGnssPositionUnaryFlag_) {
    subGnssPosition_ = privateNode.subscribe<sensor_msgs::NavSatFix>(
        "/gnss_position_topic", ROS_QUEUE_SIZE, &Position3Estimator::gnssPositionCallback_, this, ros::TransportHints().tcpNoDelay());
  }
  // GNSS Offline Pose
  if constexpr (constexprUseGnssOfflinePoseUnaryFlag_) {
    subGnssOfflinePose_ =
        privateNode.subscribe<nav_msgs::Odometry>("/gnss_offline_pose_topic", ROS_QUEUE_SIZE, &Position3Estimator::gnssOfflinePoseCallback_,
                                                  this, ros::TransportHints().tcpNoDelay());
  }

  // Log
  std::cout << YELLOW_START << "FactorGraphFiltering" << COLOR_END << " Initialized Position subscriber (on position_topic)." << std::endl;
}

//---------------------------------------------------------------
void Position3Estimator::initializeMessages(ros::NodeHandle& privateNode) {
  // Status
  REGULAR_COUT << GREEN_START << " Initializing Messages..." << COLOR_END << std::endl;

  // Paths
  measPosition_worldPrismPositionPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  measPosition_worldGnssPositionPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  measPosition_worldGnssOfflinePosePathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
}

// --------------------------------------------------------------
void Position3Estimator::initializeServices(ros::NodeHandle& privateNode) {
  // Trigger Prism Unary
  if constexpr (constexprUsePrismPositionUnaryFlag_) {
    srvTogglePrismPositionUnary_ =
        privateNode.advertiseService("/atn_position3_fuser_node/togglePrismUnary", &Position3Estimator::srvTogglePrismUnaryCallback_, this);
  }
  // Trigger GNSS Unary
  if constexpr (constexprUseGnssPositionUnaryFlag_) {
    srvToggleGnssPositionUnary_ =
        privateNode.advertiseService("/atn_position3_fuser_node/toggleGnssUnary", &Position3Estimator::srvToggleGnssUnaryCallback_, this);
  }
  // Trigger GNSS Offline Pose Unary
  if constexpr (constexprUseGnssOfflinePoseUnaryFlag_) {
    srvToggleGnssOfflinePoseUnary_ = privateNode.advertiseService("/atn_position3_fuser_node/toggleGnssOfflinePoseUnary",
                                                                  &Position3Estimator::srvToggleGnssOfflinePoseUnaryCallback_, this);
  }
}

//---------------------------------------------------------------
void Position3Estimator::prismPositionCallback_(const geometry_msgs::PointStamped::ConstPtr& leicaPositionPtr) {
  // Counter
  prismPositionCallbackCounter_++;

  // Translate to Eigen
  Eigen::Vector3d positionMeas = Eigen::Vector3d(leicaPositionPtr->point.x, leicaPositionPtr->point.y, leicaPositionPtr->point.z);
  Eigen::Vector3d positionCovarianceXYZ(prismPositionMeasUnaryNoise_, prismPositionMeasUnaryNoise_,
                                        prismPositionMeasUnaryNoise_);  // TODO: Set proper values

  // State Machine
  if (!areYawAndPositionInited() && areRollAndPitchInited()) {
    // Try to initialize yaw and position if not done already
    if (this->initYawAndPositionInWorld(
            0.0, positionMeas, staticTransformsPtr_->getBaseLinkFrame(),
            dynamic_cast<Position3StaticTransforms*>(staticTransformsPtr_.get())->getPrismPositionMeasFrame())) {
      REGULAR_COUT << " Set yaw and position successfully." << std::endl;
    } else {
      REGULAR_COUT << " Could not set yaw and position." << std::endl;
    }
  }
  // Else if active
  else if (usePrismPositionUnaryFlag_) {
    const std::string& positionMeasFrame =
        dynamic_cast<Position3StaticTransforms*>(staticTransformsPtr_.get())->getPrismPositionMeasFrame();
    const std::string& fixedFrame = staticTransformsPtr_->getWorldFrame();
    // Already initialized --> add position measurement to graph
    graph_msf::UnaryMeasurementXDAbsolute<Eigen::Vector3d, 3> meas_W_t_W_P(
        "LeicaPosition", int(prismPositionRate_), positionMeasFrame, positionMeasFrame + sensorFrameCorrectedNameId,
        graph_msf::RobustNorm::None(), leicaPositionPtr->header.stamp.toSec(), POS_COVARIANCE_VIOLATION_THRESHOLD, positionMeas,
        positionCovarianceXYZ, fixedFrame, initialSe3AlignmentNoise_);
    this->addUnaryPosition3AbsoluteMeasurement(meas_W_t_W_P);
  }
  // Else if not active
  else if ((prismPositionCallbackCounter_ % 10) == 0) {
    REGULAR_COUT << " Prism unary measurement is turned off." << std::endl;
  }

  // Visualizations
  addToPathMsg(measPosition_worldPrismPositionPathPtr_, staticTransformsPtr_->getWorldFrame(), leicaPositionPtr->header.stamp, positionMeas,
               graphConfigPtr_->imuBufferLength_ * 4);
  pubMeasWorldPrismPositionPath_.publish(measPosition_worldPrismPositionPathPtr_);
}

//---------------------------------------------------------------
void Position3Estimator::gnssPositionCallback_(const sensor_msgs::NavSatFix::ConstPtr& gnssPositionPtr) {
  // Counter
  gnssPositionCallbackCounter_++;

  // Translate to Eigen
  Eigen::Vector3d gnssCoord = Eigen::Vector3d(gnssPositionPtr->latitude, gnssPositionPtr->longitude, gnssPositionPtr->altitude);
  Eigen::Vector3d estStdDevXYZ(sqrt(gnssPositionPtr->position_covariance[0]), sqrt(gnssPositionPtr->position_covariance[4]),
                               sqrt(gnssPositionPtr->position_covariance[8]));

  // Initialize GNSS Handler
  if (gnssPositionCallbackCounter_ < NUM_GNSS_CALLBACKS_UNTIL_START) {  // Accumulate measurements
    // Wait until measurements got accumulated
    accumulatedGnssCoordinates_ += gnssCoord;
    if ((gnssPositionCallbackCounter_ % 10) == 0) {
      REGULAR_COUT << " NOT ENOUGH GNSS MESSAGES ARRIVED!" << std::endl;
    }
    return;
  } else if (gnssPositionCallbackCounter_ == NUM_GNSS_CALLBACKS_UNTIL_START) {  // Initialize GNSS Handler
    gnssHandlerPtr_->initHandler(accumulatedGnssCoordinates_ / NUM_GNSS_CALLBACKS_UNTIL_START);
    REGULAR_COUT << " GNSS Handler initialized." << std::endl;
    return;
  }

  // Convert to Cartesian Coordinates
  Eigen::Vector3d W_t_W_Gnss;
  gnssHandlerPtr_->convertNavSatToPosition(gnssCoord, W_t_W_Gnss);
  std::string fixedFrame = staticTransformsPtr_->getWorldFrame();

  // Regular Operation
  if (!areYawAndPositionInited() && areRollAndPitchInited()) {
    // Set yaw (only if no prism is used)
    if constexpr (!constexprUsePrismPositionUnaryFlag_) {
      // Try to initialize yaw and position if not done already
      if (this->initYawAndPositionInWorld(
              0.0, W_t_W_Gnss, staticTransformsPtr_->getBaseLinkFrame(),
              dynamic_cast<Position3StaticTransforms*>(staticTransformsPtr_.get())->getGnssPositionMeasFrame())) {
        REGULAR_COUT << " GNSS set yaw and position successfully, as there is no prism." << std::endl;
      } else {
        REGULAR_COUT << " Could not set yaw and position." << std::endl;
      }
    }
  }
  // Else if active
  else if (areRollAndPitchInited() && useGnssPositionUnaryFlag_) {
    const std::string& positionMeasFrame = dynamic_cast<Position3StaticTransforms*>(staticTransformsPtr_.get())->getGnssPositionMeasFrame();
    // Already initialized --> add position measurement to graph
    graph_msf::UnaryMeasurementXDAbsolute<Eigen::Vector3d, 3> meas_W_t_W_P(
        "GnssPosition", int(gnssPositionRate_), positionMeasFrame, positionMeasFrame + sensorFrameCorrectedNameId,
        graph_msf::RobustNorm::None(), gnssPositionPtr->header.stamp.toSec(), POS_COVARIANCE_VIOLATION_THRESHOLD, W_t_W_Gnss, estStdDevXYZ,
        fixedFrame, initialSe3AlignmentNoise_);
    this->addUnaryPosition3AbsoluteMeasurement(meas_W_t_W_P);
  } else if ((gnssPositionCallbackCounter_ % 10) == 0) {
    REGULAR_COUT << " GNSS unary measurement is turned off." << std::endl;
  }

  // Adjust frame name
  if (fixedFrame != staticTransformsPtr_->getWorldFrame()) {
    fixedFrame += referenceFrameAlignedNameId;
  }

  // Visualizations
  addToPathMsg(measPosition_worldGnssPositionPathPtr_, fixedFrame, gnssPositionPtr->header.stamp, W_t_W_Gnss,
               graphConfigPtr_->imuBufferLength_ * 4);
  // Publish Path
  pubMeasWorldGnssPositionPath_.publish(measPosition_worldGnssPositionPathPtr_);
}

//---------------------------------------------------------------
void Position3Estimator::gnssOfflinePoseCallback_(const nav_msgs::Odometry::ConstPtr& gnssOfflinePosePtr) {
  // Counter
  gnssOfflinePoseCallbackCounter_++;

  // Prepare Data
  Eigen::Isometry3d T_ENU_Gk = Eigen::Isometry3d::Identity();
  graph_msf::odomMsgToEigen(*gnssOfflinePosePtr, T_ENU_Gk.matrix());
  double gnssUnaryTimeK = gnssOfflinePosePtr->header.stamp.toSec();

  // Fixed Frame
  std::string fixedFrame = staticTransformsPtr_->getWorldFrame();
  // fixedFrame = gnssOfflinePosePtr->header.frame_id;

  // Measurement
  const std::string& gnssFrameName =
      dynamic_cast<Position3StaticTransforms*>(staticTransformsPtr_.get())->getGnssOfflinePoseMeasFrame();  // alias
  graph_msf::UnaryMeasurementXDAbsolute<Eigen::Isometry3d, 6> unary6DMeasurement(
      "Lidar_unary_6D", int(gnssOfflinePoseRate_), gnssFrameName, gnssFrameName + sensorFrameCorrectedNameId, graph_msf::RobustNorm::None(),
      gnssUnaryTimeK, 1.0, T_ENU_Gk, gnssOfflinePoseMeasUnaryNoise_, fixedFrame, initialSe3AlignmentNoise_);

  if (gnssOfflinePoseCallbackCounter_ <= 2) {
    return;
  } else if (!areYawAndPositionInited()) {  // Initializing if no GNSS
    if (areRollAndPitchInited() && !usePrismPositionUnaryFlag_) {
      REGULAR_COUT << GREEN_START << " GNSS offline odometry callback is setting global yaw, as it was not set so far." << COLOR_END
                   << std::endl;
      this->initYawAndPosition(unary6DMeasurement);
    }
  } else {  // Already initialized --> unary factor
    this->addUnaryPose3AbsoluteMeasurement(unary6DMeasurement);
  }

  // Visualization ----------------------------
  // Add to path message
  addToPathMsg(measPosition_worldGnssOfflinePosePathPtr_, fixedFrame, gnssOfflinePosePtr->header.stamp, T_ENU_Gk.translation(),
               graphConfigPtr_->imuBufferLength_ * 4);

  // Publish Path
  pubMeasWorldGnssOfflinePosePath_.publish(measPosition_worldGnssOfflinePosePathPtr_);
}

//---------------------------------------------------------------
bool Position3Estimator::srvTogglePrismUnaryCallback_(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  // Toggle Flag
  usePrismPositionUnaryFlag_ = !usePrismPositionUnaryFlag_;

  // Response
  res.success = true;
  res.message = "Prism unary measurement toggled " + std::string(usePrismPositionUnaryFlag_ ? "on" : "off") + ".";

  return true;
}

//---------------------------------------------------------------
bool Position3Estimator::srvToggleGnssUnaryCallback_(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  // Toggle Flag
  useGnssPositionUnaryFlag_ = !useGnssPositionUnaryFlag_;

  // Response
  res.success = true;
  res.message = "GNSS unary measurement toggled " + std::string(useGnssPositionUnaryFlag_ ? "on" : "off") + ".";

  return true;
}

//---------------------------------------------------------------
bool Position3Estimator::srvToggleGnssOfflinePoseUnaryCallback_(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  // Toggle Flag
  useGnssOfflinePoseUnaryFlag_ = !useGnssOfflinePoseUnaryFlag_;

  // Response
  res.success = true;
  res.message = "GNSS offline pose unary measurement toggled " + std::string(useGnssOfflinePoseUnaryFlag_ ? "on" : "off") + ".";

  return true;
}

}  // namespace position3_se