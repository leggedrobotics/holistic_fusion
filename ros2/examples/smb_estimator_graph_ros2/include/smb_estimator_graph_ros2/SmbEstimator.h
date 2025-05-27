/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#pragma once

// std
#include <chrono>
#include <memory>

// ROS 2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

// Workspace
#include "graph_msf_ros2/GraphMsfRos2.h"

// Defined Macros
#define ROS_QUEUE_SIZE 100
#define NUM_GNSS_CALLBACKS_UNTIL_START 20  // 0

namespace smb_se {

class SmbEstimator : public graph_msf::GraphMsfRos2 {
 public:
  explicit SmbEstimator(std::shared_ptr<rclcpp::Node>& node);

  ~SmbEstimator() override = default;

  void setup();

 protected:
  // Virtual Functions
  void readParams();
  void initializePublishers();
  void initializeSubscribers();
  void initializeMessages();
  void initializeServices();
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imuPtr);

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
  void lidarOdometryCallback_(const nav_msgs::msg::Odometry::ConstSharedPtr& lidarOdomPtr);
  // Wheel Between
  void wheelOdometryPoseCallback_(const nav_msgs::msg::Odometry::ConstSharedPtr& wheelOdomPtr);
  // Wheel Linear Velocities
  void wheelLinearVelocitiesCallback_(const std_msgs::msg::Float64MultiArray::ConstSharedPtr& wheelsSpeedsPtr);
  // VIO
  void vioOdometryCallback_(const nav_msgs::msg::Odometry::ConstSharedPtr& vioOdomPtr);

  // Callback Members
  int wheelOdometryCallbackCounter_ = -1;
  Eigen::Isometry3d T_O_Bw_km1_;
  double wheelOdometryTimeKm1_ = 0.0;

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLioOdometry_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subWheelOdometryBetween_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subWheelLinearVelocities_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subVioOdometry_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubMeasMapLioPath_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubMeasMapVioPath_;

  // Messages
  std::shared_ptr<nav_msgs::msg::Path> measLio_mapImuPathPtr_;
  std::shared_ptr<nav_msgs::msg::Path> measVio_mapImuPathPtr_;

  // Flags
  bool useLioOdometryFlag_ = true;
  bool useWheelOdometryBetweenFlag_ = false;
  bool useWheelLinearVelocitiesFlag_ = false;
  bool useVioOdometryFlag_ = false;

  // Wheel Radius
  double wheelRadiusMeter_ = 0.195;
};

}  // namespace smb_se
