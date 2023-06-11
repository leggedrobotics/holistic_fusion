/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef AnymalESTIMATOR_H
#define AnymalESTIMATOR_H

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
#include "graph_msf_msgs/GetPathInEnu.h"
#include "graph_msf_msgs/GetPathInEnuRequest.h"
#include "graph_msf_msgs/GetPathInEnuResponse.h"
#include "graph_msf_ros/GraphMsfRos.h"

// Defined Macros
#define ROS_QUEUE_SIZE 100
#define NUM_GNSS_CALLBACKS_UNTIL_START 20  // 0

namespace anymal_se {

class AnymalEstimator : public graph_msf::GraphMsfRos {
 public:
  AnymalEstimator(std::shared_ptr<ros::NodeHandle> privateNodePtr);
  // Destructor
  ~AnymalEstimator() = default;
  // Setup
  virtual bool setup() override;

 private:
  // Virtual Functions
  virtual void initializePublishers_(ros::NodeHandle& privateNode) override;
  virtual void initializeMessages_(ros::NodeHandle& privateNodePtr) override;
  virtual void initializeSubscribers_(ros::NodeHandle& privateNodePtr) override;
  virtual void readParams_(const ros::NodeHandle& privateNode) override;

  // Other
  void initializeServices_(ros::NodeHandle& privateNode);

  // GNSS Handler
  std::shared_ptr<graph_msf::GnssHandler> gnssHandlerPtr_;

  // Trajectory Alignment
  std::shared_ptr<graph_msf::TrajectoryAlignmentHandler> trajectoryAlignmentHandlerPtr_;

  // Time
  std::chrono::time_point<std::chrono::high_resolution_clock> startTime_;
  std::chrono::time_point<std::chrono::high_resolution_clock> currentTime_;

  // Config -------------------------------------

  // Rates
  double lidarRate_ = 5.0;
  double gnssRate_ = 10.0;

  // Noise
  Eigen::Matrix<double, 6, 1> poseBetweenNoise_;
  Eigen::Matrix<double, 6, 1> poseUnaryNoise_;
  double gnssPositionUnaryNoise_ = 1.0;  // in [m]
  double gnssHeadingUnaryNoise_ = 1.0;   // in [rad]

  // ROS Related stuff ----------------------------

  // Callbacks
  void lidarOdometryCallback_(const nav_msgs::Odometry::ConstPtr& lidar_odom_ptr);
  void gnssCallback_(const sensor_msgs::NavSatFix::ConstPtr& gnssPtr);
  bool gnssCoordinatesToENUCallback_(graph_msf_msgs::GetPathInEnu::Request& req, graph_msf_msgs::GetPathInEnu::Response& res);

  // Subscribers
  // Instances
  ros::Subscriber subLidarOdometry_;
  ros::Subscriber subGnss_;
  tf::TransformListener tfListener_;

  // Publishers
  // Path
  ros::Publisher pubMeasWorldGnssPath_;
  ros::Publisher pubMeasWorldLidarPath_;

  // Messages
  nav_msgs::PathPtr measGnss_worldGnssPathPtr_;
  nav_msgs::PathPtr measLidar_worldImuPathPtr_;

  // Servers
  ros::ServiceServer serverTransformGnssToEnu_;

  // Initialization
  bool initialized_ = false;
};
}  // namespace anymal_se
#endif  // end M545ESTIMATORGRAPH_H
