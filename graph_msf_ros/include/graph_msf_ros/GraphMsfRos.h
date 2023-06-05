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
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
// Workspace
#include "graph_msf/frontend/GraphMsf.h"
#include "graph_msf_ros/ros/read_ros_params.h"

namespace graph_msf {

class GraphMsfRos : public GraphMsf {
 public:
  GraphMsfRos(std::shared_ptr<ros::NodeHandle> privateNodePtr);
  // Destructor
  ~GraphMsfRos() = default;
  // Setup
  virtual bool setup() override;

 protected:
  // Functions that need implementation
  virtual void initializePublishers_(ros::NodeHandle& privateNode);
  virtual void initializeMessages_(ros::NodeHandle& privateNode);
  virtual void initializeSubscribers_(ros::NodeHandle& privateNode);

  // Commodity Functions to be shared -----------------------------------
  // Static
  static void addToPathMsg(nav_msgs::PathPtr pathPtr, const std::string& frameName, const ros::Time& stamp, const Eigen::Vector3d& t,
                           const int maxBufferLength);
  static void addToOdometryMsg(nav_msgs::OdometryPtr msgPtr, const std::string& fixedFrame, const std::string& movingFrame,
                               const ros::Time& stamp, const Eigen::Isometry3d& T, const Eigen::Vector3d& W_v_W_F,
                               const Eigen::Vector3d& W_w_W_F,
                               const Eigen::Matrix<double, 6, 6>& poseCovariance = Eigen::Matrix<double, 6, 6>::Zero(),
                               const Eigen::Matrix<double, 6, 6>& twistCovariance = Eigen::Matrix<double, 6, 6>::Zero());
  static void extractCovariancesFromOptimizedState(
      Eigen::Matrix<double, 6, 6>& poseCovarianceRos, Eigen::Matrix<double, 6, 6>& twistCovarianceRos,
      const std::shared_ptr<graph_msf::SafeNavStateWithCovarianceAndBias>& optimizedStateWithCovarianceAndBiasPtr);

  // Parameter Loading -----------------------------------
  virtual void readParams_(const ros::NodeHandle& privateNode);

  // Callbacks
  virtual void imuCallback_(const sensor_msgs::Imu::ConstPtr& imuPtr);

  // Publishing -----------------------------------
  void publishOptimizedStateAndBias_(
      const std::shared_ptr<graph_msf::SafeNavStateWithCovarianceAndBias>& optimizedStateWithCovarianceAndBiasPtr,
      const Eigen::Matrix<double, 6, 6>& poseCovarianceRos, const Eigen::Matrix<double, 6, 6>& twistCovarianceRos);
  virtual void publishState_(const std::shared_ptr<graph_msf::SafeNavState>& preIntegratedNavStatePtr,
                             const std::shared_ptr<graph_msf::SafeNavStateWithCovarianceAndBias>& optimizedStateWithCovarianceAndBiasPtr);
  // Publish Transform to TF
  void publishTransform_(const std::string& frameName, const std::string& childFrameName, const double timeStamp,
                         const Eigen::Isometry3d& T_frame_childFrame);

  // Publish IMU Odometries
  void publishImuOdom_(const std::shared_ptr<graph_msf::SafeNavState>& preIntegratedNavStatePtr,
                       const Eigen::Matrix<double, 6, 6>& poseCovarianceRos, const Eigen::Matrix<double, 6, 6>& twistCovarianceRos);

  // Node
  ros::NodeHandle privateNode_;

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
  // Imu Bias
  ros::Publisher pubAccelBias_;
  ros::Publisher pubGyroBias_;

  // Subscribers
  ros::Subscriber subImu_;

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
  // Imu Bias
  geometry_msgs::Vector3StampedPtr accelBiasMsgPtr_;
  geometry_msgs::Vector3StampedPtr gyroBiasMsgPtr_;

  // Last Optimized State Timestamp
  double lastOptimizedStateTimestamp_ = 0.0;
};
}  // namespace graph_msf
#endif  // end M545ESTIMATORGRAPH_H
