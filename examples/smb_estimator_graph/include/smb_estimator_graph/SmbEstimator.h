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
  void readParams_(const ros::NodeHandle& privateNode) override;
  void initializePublishers_(ros::NodeHandle& privateNode) override;
  void initializeSubscribers_(ros::NodeHandle& privateNode) override;
  void initializeMessages_(ros::NodeHandle& privateNode) override;
  void initializeServices_(ros::NodeHandle& privateNode) override;
  void imuCallback_(const sensor_msgs::Imu::ConstPtr& imuPtr) override;

  // Time
  std::chrono::time_point<std::chrono::high_resolution_clock> startTime_;
  std::chrono::time_point<std::chrono::high_resolution_clock> currentTime_;

  // Config -------------------------------------

  // Rates
  double lioOdometryRate_ = 5.0;
  double wheelOdometryRate_ = 50.0;
  double vioOdometryRate_ = 50.0;

  // Alignment Parameters
  Eigen::Matrix<double, 6, 1> initialSe3AlignmentNoise_;

  // Noise
  Eigen::Matrix<double, 6, 1> lioPoseUnaryNoise_;
  Eigen::Matrix<double, 6, 1> wheelPoseBetweenNoise_;
  Eigen::Matrix<double, 6, 1> vioPoseBetweenNoise_;

  // ROS Related stuff ----------------------------

  // Callbacks
  void lidarOdometryCallback_(const nav_msgs::Odometry::ConstPtr& lidarOdomPtr);
  void wheelOdometryCallback_(const nav_msgs::Odometry::ConstPtr& wheelOdomPtr);
  void vioOdometryCallback_(const nav_msgs::Odometry::ConstPtr& vioOdomPtr);

  // Callback Members
  int wheelOdometryCallbackCounter_ = -1;
  Eigen::Isometry3d T_O_Bw_km1_;
  double wheelOdometryTimeKm1_ = 0.0;

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
  bool useLioOdometryFlag_ = true;
  bool useWheelOdometryFlag_ = false;
  bool useVioOdometryFlag_ = false;
};
}  // namespace smb_se
#endif  // end Smb_Estimator_H
