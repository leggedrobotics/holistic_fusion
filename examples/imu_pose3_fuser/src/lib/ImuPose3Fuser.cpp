/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "imu_pose3_fuser/ImuPose3Fuser.h"

// Workspace
#include "graph_msf/config/StaticTransforms.h"
#include "graph_msf/measurements/BinaryMeasurementXD.h"
#include "graph_msf/measurements/UnaryMeasurementXD.h"
#include "graph_msf_ros/util/conversions.h"
#include "imu_pose3_fuser/constants.h"

namespace imu_pose3_fuser {

ImuPose3Fuser::ImuPose3Fuser(std::shared_ptr<ros::NodeHandle> privateNodePtr) : graph_msf::GraphMsfRos(privateNodePtr) {
  REGULAR_COUT << GREEN_START << " ImuPose3Fuser-Constructor called." << COLOR_END << std::endl;

  // Configurations ----------------------------
  // Static transforms
  staticTransformsPtr_ = std::make_shared<graph_msf::StaticTransforms>();

  // Set up
  ImuPose3Fuser::setup();
}

void ImuPose3Fuser::setup() {
  REGULAR_COUT << GREEN_START << " ImuPose3Fuser-Setup called." << COLOR_END << std::endl;

  // Read parameters
  ImuPose3Fuser::readParams(privateNode);

  // Super class
  GraphMsfRos::setup(staticTransformsPtr_);

  // Find transformations
  staticTransformsPtr_->findTransformations();

  // Publishers ----------------------------
  ImuPose3Fuser::initializePublishers(privateNode);

  // Subscribers ----------------------------
  ImuPose3Fuser::initializeSubscribers(privateNode);

  // Messages ----------------------------
  ImuPose3Fuser::initializeMessages(privateNode);

  // Server ----------------------------
  ImuPose3Fuser::initializeServices(privateNode);

  // Wrap up ----------------------------
  REGULAR_COUT << GREEN_START << " Set up successfully." << COLOR_END << std::endl;
}

void ImuPose3Fuser::initializePublishers(ros::NodeHandle& privateNode) {
  // Paths
  pubMeasPose3Path_ = privateNode.advertise<nav_msgs::Path>("/graph_msf/measPose3_path_world_imu", ROS_QUEUE_SIZE);
}

void ImuPose3Fuser::initializeSubscribers(ros::NodeHandle& privateNode) {
  // Pose3 Odometry
  subPose3Odometry_ = privateNode.subscribe<nav_msgs::Odometry>("/pose3_odometry_topic", ROS_QUEUE_SIZE, &ImuPose3Fuser::pose3Callback_,
                                                                 this, ros::TransportHints().tcpNoDelay());
  REGULAR_COUT << COLOR_END << " Initialized Pose3 Odometry subscriber with topic: " << subPose3Odometry_.getTopic() << std::endl;
}

void ImuPose3Fuser::initializeMessages(ros::NodeHandle& privateNode) {
  // Path
  measPose3_worldImuPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
}

void ImuPose3Fuser::initializeServices(ros::NodeHandle& privateNode) {
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
  graph_msf::UnaryMeasurementXDAbsolute<Eigen::Isometry3d, 6> unary6DMeasurement(
      "Pose3Unary6D", int(pose3OdometryRate_), unaryPose3Frame_, unaryPose3Frame_, graph_msf::RobustNorm::None(), odometryTimeK, 1.0,
      T_W_Ik, pose3UnaryNoise_, staticTransformsPtr_->getWorldFrame(), initialSe3AlignmentNoise_);

  // Only add measurement once every second in beginning
  bool addMeasurementFlag = false;
  if (odometryCallbackCounter__ < 4000) {
    if ((odometryCallbackCounter__ % 200) == 0) {
      addMeasurementFlag = true;
    }
  } else {  // And more rarely later
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
    this->addUnaryPose3AbsoluteMeasurement(unary6DMeasurement);
  }

  // Visualization ----------------------------
  // Add to path message
  addToPathMsg(measPose3_worldImuPathPtr_, staticTransformsPtr_->getWorldFrame(), odomPtr->header.stamp, T_W_Ik.matrix().block<3, 1>(0, 3),
               graphConfigPtr_->imuBufferLength_ * 4);

  // Publish Path
  pubMeasPose3Path_.publish(measPose3_worldImuPathPtr_);
}

}  // namespace imu_pose3_fuser