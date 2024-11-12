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
#include <std_msgs/Float64MultiArray.h>
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
  void setup();

 protected:
  // Virtual Functions
  void readParams(const ros::NodeHandle& privateNode) override;
  void initializePublishers(ros::NodeHandle& privateNode) override;
  void initializeSubscribers(ros::NodeHandle& privateNode) override;
  void initializeMessages(ros::NodeHandle& privateNode) override;
  void initializeServices(ros::NodeHandle& privateNode) override;
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imuPtr) override;

 private:
  // Time
  std::chrono::time_point<std::chrono::high_resolution_clock> startTime_;
  std::chrono::time_point<std::chrono::high_resolution_clock> currentTime_;

  // Config -------------------------------------

  // Rates
  double lioOdometryRate_ = 5.0;
  double wheelOdometryBetweenRate_ = 50.0;
  double wheelLinearVelocitiesRate_ = 50.0;
  double vioOdometryRate_ = 50.0;

  // Alignment Parameters
  Eigen::Matrix<double, 6, 1> initialSe3AlignmentNoise_ = 1.0 * Eigen::Matrix<double, 6, 1>::Ones();
  Eigen::Matrix<double, 6, 1> lioSe3AlignmentRandomWalk_ = 0.0 * Eigen::Matrix<double, 6, 1>::Ones();

  // Noise
  // LiDAR Odometry
  Eigen::Matrix<double, 6, 1> lioPoseUnaryNoise_;
  // Wheel Odometry
  // Between
  Eigen::Matrix<double, 6, 1> wheelPoseBetweenNoise_;
  // Linear Velocities
  Eigen::Matrix<double, 3, 1> wheelLinearVelocitiesNoise_;
  // VIO Odometry
  Eigen::Matrix<double, 6, 1> vioPoseBetweenNoise_;

  // ROS Related stuff ----------------------------

  // Callbacks
  // LiDAR
  void lidarOdometryCallback_(const nav_msgs::Odometry::ConstPtr& lidarOdomPtr);
  // Wheel Between
  void wheelOdometryPoseCallback_(const nav_msgs::Odometry::ConstPtr& wheelOdomPtr);
  // Wheel Linear Velocities
  void wheelLinearVelocitiesCallback_(const std_msgs::Float64MultiArray::ConstPtr& wheelsSpeedsPtr);
  // VIO
  void vioOdometryCallback_(const nav_msgs::Odometry::ConstPtr& vioOdomPtr);

  // Callback Members
  int wheelOdometryCallbackCounter_ = -1;
  Eigen::Isometry3d T_O_Bw_km1_;
  double wheelOdometryTimeKm1_ = 0.0;

  // Subscribers
  // Instances
  // LIO
  ros::Subscriber subLioOdometry_;
  // Wheel Between
  ros::Subscriber subWheelOdometryBetween_;
  // Wheel Linear Velocities
  ros::Subscriber subWheelLinearVelocities_;
  // VIO
  ros::Subscriber subVioOdometry_;
  // TF Listener
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
  bool useWheelOdometryBetweenFlag_ = false;
  bool useWheelLinearVelocitiesFlag_ = false;
  bool useVioOdometryFlag_ = false;

  // Wheel Radius
  double wheelRadiusMeter_ = 0.195;
};
}  // namespace smb_se
#endif  // end Smb_Estimator_H
