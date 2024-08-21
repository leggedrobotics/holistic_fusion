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
  Position3Estimator::readParams_(privateNode_);

  // Super class
  GraphMsfRos::setup(staticTransformsPtr_);

  // Publishers ----------------------------
  Position3Estimator::initializePublishers_(privateNode_);

  // Subscribers ----------------------------
  Position3Estimator::initializeSubscribers_(privateNode_);

  // Messages ----------------------------
  Position3Estimator::initializeMessages_(privateNode_);

  // Static Transforms
  staticTransformsPtr_->findTransformations();

  // Wrap up ----------------------------
  REGULAR_COUT << GREEN_START << " Set up successfully." << COLOR_END << std::endl;
}

//---------------------------------------------------------------
void Position3Estimator::initializePublishers_(ros::NodeHandle& privateNode) {
  // Status
  REGULAR_COUT << GREEN_START << " Initializing Publishers..." << COLOR_END << std::endl;

  // Paths
  pubMeasWorldPositionPath_ = privateNode.advertise<nav_msgs::Path>("/graph_msf/measPosition_path_world_prism", ROS_QUEUE_SIZE);
}

void Position3Estimator::initializeSubscribers_(ros::NodeHandle& privateNode) {
  subPosition_ = privateNode.subscribe<geometry_msgs::PointStamped>(
      "/position_topic", ROS_QUEUE_SIZE, &Position3Estimator::positionCallback_, this, ros::TransportHints().tcpNoDelay());

  std::cout << YELLOW_START << "FactorGraphFiltering" << COLOR_END << " Initialized Position subscriber (on position_topic)." << std::endl;
  return;
}

//---------------------------------------------------------------
void Position3Estimator::initializeMessages_(ros::NodeHandle& privateNode) {
  // Status
  REGULAR_COUT << GREEN_START << " Initializing Messages..." << COLOR_END << std::endl;

  // Paths
  measPosition_worldPositionPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
}

//---------------------------------------------------------------
void Position3Estimator::positionCallback_(const geometry_msgs::PointStamped::ConstPtr& leicaPositionPtr) {
  // Counter
  positionCallbackCounter_++;

  // Translate to Eigen
  Eigen::Vector3d positionMeas = Eigen::Vector3d(leicaPositionPtr->point.x, leicaPositionPtr->point.y, leicaPositionPtr->point.z);
  Eigen::Vector3d positionCovarianceXYZ(positionMeasUnaryNoise_, positionMeasUnaryNoise_,
                                        positionMeasUnaryNoise_);  // TODO: Set proper values

  // State Machine
  if (!areYawAndPositionInited() && areRollAndPitchInited()) {
    // Try to initialize yaw and position (WITH ZERO POSITION) if not done already
    if (this->initYawAndPositionInWorld(0.0, positionMeas, staticTransformsPtr_->getBaseLinkFrame(),
                                        dynamic_cast<Position3StaticTransforms*>(staticTransformsPtr_.get())->getPositionMeasFrame())) {
      REGULAR_COUT << " Set yaw and position successfully." << std::endl;
    } else {
      REGULAR_COUT << " Could not set yaw and position." << std::endl;
    }
  } else {
    const std::string& positionMeasFrame = dynamic_cast<Position3StaticTransforms*>(staticTransformsPtr_.get())->getPositionMeasFrame();
    const std::string& fixedFrame = staticTransformsPtr_->getWorldFrame();
    // Already initialized --> add position measurement to graph
    graph_msf::UnaryMeasurementXDAbsolute<Eigen::Vector3d, 3> meas_W_t_W_P(
        "LeicaPosition", int(positionRate_), positionMeasFrame, positionMeasFrame + sensorFrameCorrectedNameId_,
        graph_msf::RobustNorm::None(), leicaPositionPtr->header.stamp.toSec(), POS_COVARIANCE_VIOLATION_THRESHOLD, positionMeas,
        positionCovarianceXYZ, fixedFrame, initialSe3AlignmentNoise_);
    this->addUnaryPosition3AbsoluteMeasurement(meas_W_t_W_P);
  }

  // Visualizations
  addToPathMsg(measPosition_worldPositionPathPtr_, staticTransformsPtr_->getWorldFrame(), leicaPositionPtr->header.stamp, positionMeas,
               graphConfigPtr_->imuBufferLength_ * 4);
  pubMeasWorldPositionPath_.publish(measPosition_worldPositionPathPtr_);
}

}  // namespace position3_se