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
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_listener.h>

// Workspace
#include "graph_msf_ros/GraphMsfRos.h"

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

  // Time
  std::chrono::time_point<std::chrono::high_resolution_clock> startTime_;
  std::chrono::time_point<std::chrono::high_resolution_clock> currentTime_;

  // Config -------------------------------------

  // Rates
  double lidarOdometryRate_ = 5.0;
  double wheelOdometryRate_ = 50.0;
  double vioOdometryRate_ = 50.0;

  // Noise
  Eigen::Matrix<double, 6, 1> lidarPoseBetweenNoise_;
  Eigen::Matrix<double, 6, 1> lidarPoseUnaryNoise_;
  Eigen::Matrix<double, 6, 1> wheelPoseBetweenNoise_;
  Eigen::Matrix<double, 6, 1> vioPoseBetweenNoise_;

  // ROS Related stuff ----------------------------

  // Callbacks
  void lidarOdometryCallback_(const nav_msgs::Odometry::ConstPtr& lidarOdomPtr);
  void wheelOdometryCallback_(const nav_msgs::Odometry::ConstPtr& wheelOdomPtr);
  void vioOdometryCallback_(const nav_msgs::Odometry::ConstPtr& vioOdomPtr);

  // Subscribers
  // Instances
  ros::Subscriber subLioOdometry_;
  ros::Subscriber subWheelOdometry_;
  ros::Subscriber subVioOdometry_;
  tf::TransformListener tfListener_;

  // Publishers
  // Path
  ros::Publisher pubMeasMapLioPath_;
  ros::Publisher pubMeasMapVioPath_;

  // Messages
  nav_msgs::PathPtr measLio_mapImuPathPtr_;
  nav_msgs::PathPtr measVio_mapImuPathPtr_;

  // Flags
  bool useLidarUnaryFactorFlag_ = false;
  bool useWheelOdometryFlag_ = false;
  bool useVioOdometryFlag_ = false;
};
}  // namespace smb_se
#endif  // end Smb_Estimator_H
