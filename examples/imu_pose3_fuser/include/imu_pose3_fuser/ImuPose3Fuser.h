/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
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

namespace imu_pose3_fuser {

class ImuPose3Fuser : public graph_msf::GraphMsfRos {
 public:
  ImuPose3Fuser(std::shared_ptr<ros::NodeHandle> privateNodePtr);
  // Destructor
  ~ImuPose3Fuser() = default;
  // Setup
  virtual bool setup() override;

 private:
  // Virtual Functions
  virtual void initializePublishers_(ros::NodeHandle& privateNode) override;
  virtual void initializeSubscribers_(ros::NodeHandle& privateNode) override;
  virtual void initializeMessages_(ros::NodeHandle& privateNode) override;
  virtual void initializeServices_(ros::NodeHandle& privateNode) override;
  virtual void readParams_(const ros::NodeHandle& privateNode) override;

  // Time
  std::chrono::time_point<std::chrono::high_resolution_clock> startTime_;
  std::chrono::time_point<std::chrono::high_resolution_clock> currentTime_;

  // Config -------------------------------------

  // Rates
  double pose3OdometryRate_ = 5.0;

  // Noise
  Eigen::Matrix<double, 6, 1> pose3UnaryNoise_;

  // ROS Related stuff ----------------------------

  // Callbacks
  void pose3Callback_(const nav_msgs::Odometry::ConstPtr& lidarOdomPtr);

  // Subscribers
  // Instances
  ros::Subscriber subPose3Odometry_;

  // Publishers
  // Path
  ros::Publisher pubMeasPose3Path_;

  // Messages
  nav_msgs::PathPtr measPose3_worldImuPathPtr_;
};
}  // namespace imu_pose3_fuser
#endif  // end Smb_Estimator_H
