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
  pubMeasWorldPrismPositionPath_ = privateNode.advertise<nav_msgs::Path>("/graph_msf/measPosition_path_world_prism", ROS_QUEUE_SIZE);
  pubMeasWorldGnssPositionPath_ = privateNode.advertise<nav_msgs::Path>("/graph_msf/measPosition_path_world_gnss", ROS_QUEUE_SIZE);
}

void Position3Estimator::initializeSubscribers(ros::NodeHandle& privateNode) {
  // Prism Position
  subPrismPosition_ = privateNode.subscribe<geometry_msgs::PointStamped>(
      "/prism_position_topic", ROS_QUEUE_SIZE, &Position3Estimator::prismPositionCallback_, this, ros::TransportHints().tcpNoDelay());
  // GNSS
  subGnssPosition_ = privateNode.subscribe<sensor_msgs::NavSatFix>(
      "/gnss_position_topic", ROS_QUEUE_SIZE, &Position3Estimator::gnssPositionCallback_, this, ros::TransportHints().tcpNoDelay());

  // Log
  std::cout << YELLOW_START << "FactorGraphFiltering" << COLOR_END << " Initialized Position subscriber (on position_topic)." << std::endl;
  return;
}

//---------------------------------------------------------------
void Position3Estimator::initializeMessages(ros::NodeHandle& privateNode) {
  // Status
  REGULAR_COUT << GREEN_START << " Initializing Messages..." << COLOR_END << std::endl;

  // Paths
  measPosition_worldPrismPositionPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  measPosition_worldGnssPositionPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
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
  } else {
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

  // Visualizations
  addToPathMsg(measPosition_worldPrismPositionPathPtr_, staticTransformsPtr_->getWorldFrame(), leicaPositionPtr->header.stamp, positionMeas,
               graphConfigPtr_->imuBufferLength_ * 4);
  pubMeasWorldPrismPositionPath_.publish(measPosition_worldPrismPositionPathPtr_);

  // Log
  std::cout << YELLOW_START << "Position3Estimator" << COLOR_END << " Prism Position Callback." << std::endl;
}

//---------------------------------------------------------------
void Position3Estimator::gnssPositionCallback_(const sensor_msgs::NavSatFix::ConstPtr& gnssPositionPtr) {
  // Log
  // std::cout << YELLOW_START << "Position3Estimator" << COLOR_END << " GNSS Position Callback." << std::endl;
  std::cout << YELLOW_START << "Position3Estimator" << COLOR_END << " GNSS Position Callback, Uncertainty: " << std::endl;
  std::cout << gnssPositionPtr->position_covariance[0] << std::endl;
  std::cout << gnssPositionPtr->position_covariance[4] << std::endl;
  std::cout << gnssPositionPtr->position_covariance[8] << std::endl;
}

}  // namespace position3_se