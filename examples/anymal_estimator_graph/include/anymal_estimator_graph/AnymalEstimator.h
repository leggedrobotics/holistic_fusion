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
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
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
#include "graph_msf/measurements/UnaryMeasurementXD.h"
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
  virtual void readParams_(const ros::NodeHandle& privateNode);

  // Callbacks
  void lidarOdometryCallback_(const nav_msgs::Odometry::ConstPtr& lidar_odom_ptr);
  void gnssCallback_(const sensor_msgs::NavSatFix::ConstPtr& gnssPtr);
  void leggedOdometryCallback_(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& leggedOdometryKPtr);

  // Other
  void initializeServices_(ros::NodeHandle& privateNode);

  // GNSS Handler
  std::shared_ptr<graph_msf::GnssHandler> gnssHandlerPtr_;

  // Time
  std::chrono::time_point<std::chrono::high_resolution_clock> startTime_;
  std::chrono::time_point<std::chrono::high_resolution_clock> currentTime_;

  // Config -------------------------------------
  // Rates
  double lioOdometryRate_ = 5.0;
  double leggedOdometryRate_ = 400.0;
  double gnssRate_ = 10.0;

  // Flags
  bool useGnssFlag_ = false;
  bool useLioFlag_ = true;
  bool useLeggedOdometryFlag_ = true;

  // Noise
  Eigen::Matrix<double, 6, 1> lioPoseUnaryNoise_;
  Eigen::Matrix<double, 6, 1> legPoseBetweenNoise_;
  double gnssPositionUnaryNoise_ = 1.0;  // in [m]

  // Initialization Params
  double initialBaseYawDeg_;

  // Outlier Params
  double outlierJumpingThreshold_;

  // ROS Objects ----------------------------

  // Subscribers
  // Instances
  ros::Subscriber subLidarOdometry_;
  ros::Subscriber subGnss_;
  ros::Subscriber leggedOdometry_;
  tf::TransformListener tfListener_;

  // Publishers
  // Path
  ros::Publisher pubMeasMapLioPath_;
  ros::Publisher pubMeasWorldGnssPath_;

  // Messages
  nav_msgs::PathPtr measLio_mapImuPathPtr_;
  nav_msgs::PathPtr measGnss_worldGnssPathPtr_;

  // Servers
  ros::ServiceServer serverTransformGnssToEnu_;
};
}  // namespace anymal_se
#endif  // end M545ESTIMATORGRAPH_H
