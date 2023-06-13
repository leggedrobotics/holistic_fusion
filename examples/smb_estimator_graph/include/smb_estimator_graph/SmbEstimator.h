/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef Smb_Estimator_H
#define Smb_Estimator_H

// std
#include <chrono>

// ROS
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// Workspace
#include "graph_msf/gnss/GnssHandler.h"
#include "graph_msf/measurements/UnaryMeasurement6D.h"
#include "graph_msf/trajectory_alignment/TrajectoryAlignmentHandler.h"
#include "graph_msf_ros/GraphMsfRos.h"
#include "graph_msf_ros_msgs/GetPathInEnu.h"
#include "graph_msf_ros_msgs/GetPathInEnuRequest.h"
#include "graph_msf_ros_msgs/GetPathInEnuResponse.h"

// Defined Macros
#define ROS_QUEUE_SIZE 100
#define NUM_GNSS_CALLBACKS_UNTIL_START 20  // 0

namespace smb_se {

class SmbEstimator : public graph_msf::GraphMsfRos {
 public:
  SmbEstimator(std::shared_ptr<ros::NodeHandle> privateNodePtr);
  // Destructor
  ~SmbEstimator() = default;
  // Setup
  virtual bool setup() override;

 private:
  // Virtual Functions
  virtual void initializePublishers_(ros::NodeHandle& privateNode) override;
  virtual void initializeMessages_(ros::NodeHandle& privateNodePtr) override;
  virtual void initializeSubscribers_(ros::NodeHandle& privateNodePtr) override;
  virtual void readParams_(const ros::NodeHandle& privateNode) override;

  // GNSS Handler
  std::shared_ptr<graph_msf::GnssHandler> gnssHandlerPtr_;

  // Time
  std::chrono::time_point<std::chrono::high_resolution_clock> startTime_;
  std::chrono::time_point<std::chrono::high_resolution_clock> currentTime_;

  // Config -------------------------------------

  // Rates
  double lidarRate_ = 5.0;

  // Noise
  Eigen::Matrix<double, 6, 1> poseBetweenNoise_;
  Eigen::Matrix<double, 6, 1> poseUnaryNoise_;

  // ROS Related stuff ----------------------------

  // Callbacks
  void lidarOdometryCallback_(const nav_msgs::Odometry::ConstPtr& lidar_odom_ptr);
  bool gnssCoordinatesToENUCallback_(graph_msf_ros_msgs::GetPathInEnu::Request& req, graph_msf_ros_msgs::GetPathInEnu::Response& res);

  // Subscribers
  // Instances
  ros::Subscriber subLidarOdometry_;
  tf::TransformListener tfListener_;

  // Publishers
  // Path
  ros::Publisher pubMeasWorldLidarPath_;

  // Messages
  nav_msgs::PathPtr measLidar_worldImuPathPtr_;

  // Initialization
  bool initialized_ = false;
};
}  // namespace smb_se
#endif  // end Smb_Estimator_H
