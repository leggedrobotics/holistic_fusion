/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "imu_pose3_fuser/ImuPose3Fuser.h"

// Workspace
#include "graph_msf/measurements/BinaryMeasurementXD.h"
#include "graph_msf/measurements/UnaryMeasurementXD.h"
#include "graph_msf_ros/util/conversions.h"
#include "graph_msf/config/StaticTransforms.h"
#include "imu_pose3_fuser/constants.h"

namespace imu_pose3_fuser {

ImuPose3Fuser::ImuPose3Fuser(std::shared_ptr<ros::NodeHandle> privateNodePtr) : graph_msf::GraphMsfRos(privateNodePtr) {
  REGULAR_COUT << GREEN_START << " Initializing..." << COLOR_END << std::endl;

  // Configurations ----------------------------
  // Static transforms
  staticTransformsPtr_ = std::make_shared<graph_msf::StaticTransforms>();

  // Set up
  if (!ImuPose3Fuser::setup()) {
    REGULAR_COUT << RED_START << " Failed to set up." << COLOR_END << std::endl;
    std::runtime_error("SmbEstimator failed to set up.");
  }

  REGULAR_COUT << GREEN_START << " Set up successfully." << COLOR_END << std::endl;
}

bool ImuPose3Fuser::setup() {
  REGULAR_COUT << GREEN_START << " Setting up." << COLOR_END << std::endl;

    // Super class
    if (not graph_msf::GraphMsfRos::setup()) {
        throw std::runtime_error("GraphMsfRos could not be initialized");
    }

  // Read parameters ----------------------------
  ImuPose3Fuser::readParams_(privateNode_);
    staticTransformsPtr_->findTransformations();

  // Publishers ----------------------------
  ImuPose3Fuser::initializePublishers_(privateNode_);

  // Subscribers ----------------------------
  ImuPose3Fuser::initializeSubscribers_(privateNode_);

  // Messages ----------------------------
  ImuPose3Fuser::initializeMessages_(privateNode_);

    // Server ----------------------------
    ImuPose3Fuser::initializeServices_(privateNode_);

  // Wrap up ----------------------------
  REGULAR_COUT << GREEN_START << " Set up successfully." << COLOR_END << std::endl;

  return true;
}

void ImuPose3Fuser::initializePublishers_(ros::NodeHandle& privateNode) {
  // Paths
  pubMeasPose3Path_ = privateNode_.advertise<nav_msgs::Path>("/graph_msf/measPose3_path_world_imu", ROS_QUEUE_SIZE);
}

void ImuPose3Fuser::initializeSubscribers_(ros::NodeHandle& privateNode) {
  // Pose3 Odometry
    subPose3Odometry_ = privateNode_.subscribe<nav_msgs::Odometry>(
        "/pose3_odometry_topic", ROS_QUEUE_SIZE, &ImuPose3Fuser::pose3Callback_, this, ros::TransportHints().tcpNoDelay());
    REGULAR_COUT << COLOR_END << " Initialized Pose3 Odometry subscriber with topic: " << subPose3Odometry_.getTopic() << std::endl;
}

void ImuPose3Fuser::initializeMessages_(ros::NodeHandle& privateNode) {
  // Path
  measPose3_worldImuPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
}

void ImuPose3Fuser::initializeServices_(ros::NodeHandle& privateNode) {
  // Nothing
}

void ImuPose3Fuser::pose3Callback_(const nav_msgs::Odometry::ConstPtr& odomPtr) {
  // Static members
  static int odometryCallbackCounter__ = -1;


  // Counter
  ++odometryCallbackCounter__;

  Eigen::Isometry3d T_W_Ik;
  graph_msf::odomMsgToEigen(*odomPtr, T_W_Ik.matrix());

  // Transform to IMU frame
  double odometryTimeK = odomPtr->header.stamp.toSec();

  // Measurement
  graph_msf::UnaryMeasurementXD<Eigen::Isometry3d, 6> unary6DMeasurement(
            "pose3_unary_6D", int(pose3OdometryRate_), staticTransformsPtr_->getImuFrame(),
            graph_msf::RobustNormEnum::None, 1.345, odometryTimeK, staticTransformsPtr_->getWorldFrame(), 1.0, T_W_Ik, pose3UnaryNoise_);

  // Only add measurement once every second in beginning
  bool addMeasurementFlag = false;
  if (odometryCallbackCounter__ < 4000) {
      if ((odometryCallbackCounter__ % 200) == 0) {
          addMeasurementFlag = true;
      }
  } else { // And more rarely later
      if ((odometryCallbackCounter__ % 2000) == 0) {
          addMeasurementFlag = true;
      }
  }

    // Add measurement or initialize
  if (odometryCallbackCounter__ <= 2) {
    return;
  } else if (!areYawAndPositionInited()) {  // Initializing
      REGULAR_COUT << GREEN_START << " Odometry callback is setting global yaw, as it was not set so far." << COLOR_END << std::endl;
      this->initYawAndPosition(unary6DMeasurement);
  } else if (addMeasurementFlag) {  // Already initialized --> unary factor
    this->addUnaryPoseMeasurement(unary6DMeasurement);
  }

  // Visualization ----------------------------
  // Add to path message
  addToPathMsg(measPose3_worldImuPathPtr_, staticTransformsPtr_->getWorldFrame(), odomPtr->header.stamp, T_W_Ik.matrix().block<3, 1>(0, 3),
               graphConfigPtr_->imuBufferLength * 4);

  // Publish Path
  pubMeasPose3Path_.publish(measPose3_worldImuPathPtr_);
}

}  // namespace imu_pose3_fuser