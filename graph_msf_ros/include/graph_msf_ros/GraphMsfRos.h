/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef CompslamSeRos_H
#define CompslamSeRos_H

// std
#include <chrono>

// ROS
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
// Workspace
#include "graph_msf/frontend/GraphMsf.h"
#include "graph_msf_ros/ros/read_ros_params.h"

namespace graph_msf {

class GraphMsfRos : public GraphMsf {
 public:
  GraphMsfRos() {}

 protected:
  // Functions that need implementation
  virtual void initializePublishers_(std::shared_ptr<ros::NodeHandle>& privateNodePtr);
  virtual void initializeMessages_(std::shared_ptr<ros::NodeHandle>& privateNodePtr);
  virtual void initializeSubscribers_(std::shared_ptr<ros::NodeHandle>& privateNodePtr) = 0;
  // Commodity Functions to be shared
  static void addToPathMsg(nav_msgs::PathPtr pathPtr, const std::string& frameName, const ros::Time& stamp, const Eigen::Vector3d& t,
                           const int maxBufferLength);
  static void addToOdometryMsg(nav_msgs::OdometryPtr msgPtr, const std::string& fixedFrame, const std::string& movingFrame,
                               const ros::Time& stamp, const Eigen::Isometry3d& T, const Eigen::Vector3d& W_v_W_F,
                               const Eigen::Vector3d& W_w_W_F,
                               const Eigen::Matrix<double, 6, 6>& poseCovariance = Eigen::Matrix<double, 6, 6>::Zero(),
                               const Eigen::Matrix<double, 6, 6>& twistCovariance = Eigen::Matrix<double, 6, 6>::Zero());
  long secondsSinceStart_();

  // Commodity functions
  virtual void readParams_(const ros::NodeHandle& privateNode);

  // Publish State
  virtual void publishState_(const std::shared_ptr<graph_msf::SafeNavState>& preIntegratedNavStatePtr,
                             const std::shared_ptr<graph_msf::SafeNavStateWithCovarianceAndBias>& optimizedStateWithCovarianceAndBiasPtr);

  // Time
  std::chrono::time_point<std::chrono::high_resolution_clock> startTime_;
  std::chrono::time_point<std::chrono::high_resolution_clock> currentTime_;

 private:
  // Publishers
  // Odometry
  ros::Publisher pubEstOdomImu_;
  ros::Publisher pubEstWorldImu_;
  ros::Publisher pubOptWorldImu_;
  // Path
  ros::Publisher pubEstOdomImuPath_;
  ros::Publisher pubEstWorldImuPath_;
  ros::Publisher pubOptWorldImuPath_;
  ros::Publisher pubMeasWorldGnssLPath_;
  ros::Publisher pubMeasWorldGnssRPath_;
  ros::Publisher pubMeasWorldLidarPath_;
  // TF
  tf::TransformBroadcaster tfBroadcaster_;

  // Messages
  // Odometry
  nav_msgs::OdometryPtr estOdomImuMsgPtr_;
  nav_msgs::OdometryPtr estWorldImuMsgPtr_;
  nav_msgs::OdometryPtr optWorldImuMsgPtr_;
  // Path
  nav_msgs::PathPtr estOdomImuPathPtr_;
  nav_msgs::PathPtr estWorldImuPathPtr_;
  nav_msgs::PathPtr optWorldImuPathPtr_;
  nav_msgs::PathPtr measWorldLeftGnssPathPtr_;
  nav_msgs::PathPtr measWorldRightGnssPathPtr_;
  nav_msgs::PathPtr measWorldLidarPathPtr_;

  // Last Optimized State Timestamp
  double lastOptimizedStateTimestamp_ = 0.0;
};
}  // namespace graph_msf
#endif  // end M545ESTIMATORGRAPH_H
