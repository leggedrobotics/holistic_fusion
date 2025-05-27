/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "smb_estimator_graph_ros2/SmbStaticTransforms.h"

// ROS 2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/rclcpp.hpp>

// Workspace
#include "graph_msf_ros2/util/conversions.h"
#include "smb_estimator_graph_ros2/constants.h"

namespace smb_se {

SmbStaticTransforms::SmbStaticTransforms(const std::shared_ptr<rclcpp::Node>& nodePtr)
    : graph_msf::StaticTransformsTf(nodePtr, *this) {
  REGULAR_COUT << GREEN_START << " Initializing static transforms..." << COLOR_END << std::endl;
}

void SmbStaticTransforms::findTransformations() {
  // Super Method
  graph_msf::StaticTransformsTf::findTransformations();

  // Print to console
  REGULAR_COUT << COLOR_END << " Looking up transforms in TF-tree." << std::endl;
  REGULAR_COUT << COLOR_END << " Transforms between the following frames are required:" << std::endl;
  REGULAR_COUT << COLOR_END << " Waiting for up to 100 seconds until they arrive..." << std::endl;

  // Temporary variable
  geometry_msgs::msg::TransformStamped transform;

  // Imu to LiDAR Link ---
  REGULAR_COUT << COLOR_END << " Waiting for transform LO for 10 seconds." << std::endl;
  transform = tf_buffer_->lookupTransform(imuFrame_, lidarOdometryFrame_, tf2::TimePointZero, tf2::durationFromSec(1.0));
  Eigen::Isometry3d eigenTransform = tf2::transformToEigen(transform.transform);
  lv_T_frame1_frame2(imuFrame_, lidarOdometryFrame_) = eigenTransform;

  std::cout << YELLOW_START << "Smb-StaticTransforms" << COLOR_END << " Translation I_Lidar: " << imuFrame_ << " " << lidarOdometryFrame_
            << " " << rv_T_frame1_frame2(imuFrame_, lidarOdometryFrame_).translation() << std::endl;
  lv_T_frame1_frame2(lidarOdometryFrame_, imuFrame_) = rv_T_frame1_frame2(imuFrame_, lidarOdometryFrame_).inverse();

  // Imu to VIO Link ---
  REGULAR_COUT << COLOR_END << " Waiting for transform VIO for 10 seconds." << std::endl;
  transform = tf_buffer_->lookupTransform(imuFrame_, vioOdometryFrame_, tf2::TimePointZero, tf2::durationFromSec(1.0));
  eigenTransform = tf2::transformToEigen(transform.transform);
  lv_T_frame1_frame2(imuFrame_, vioOdometryFrame_) = eigenTransform;

  std::cout << YELLOW_START << "Smb-StaticTransforms" << COLOR_END << " Translation I_VIO: " << imuFrame_ << " " << vioOdometryFrame_ << " "
            << rv_T_frame1_frame2(imuFrame_, vioOdometryFrame_).translation() << std::endl;
  lv_T_frame1_frame2(vioOdometryFrame_, imuFrame_) = rv_T_frame1_frame2(imuFrame_, vioOdometryFrame_).inverse();

  // Wheel Frames ---
  REGULAR_COUT << RED_START
               << " As the Wheels are turning, we only use the position of the wheels frames and use the orientation of the baseLinkFrame_ frame."
               << COLOR_END << std::endl;

  // Left Wheel
  REGULAR_COUT << COLOR_END << " Waiting for transform for 10 seconds." << std::endl;
  transform = tf_buffer_->lookupTransform(imuFrame_, wheelLinearVelocityLeftFrame_, tf2::TimePointZero, tf2::durationFromSec(1.0));
  Eigen::Isometry3d T_I_WheelLeft = tf2::transformToEigen(transform.transform);
  T_I_WheelLeft.matrix().block<3, 3>(0, 0) = rv_T_frame1_frame2(imuFrame_, baseLinkFrame_).matrix().block<3, 3>(0, 0);
  lv_T_frame1_frame2(imuFrame_, wheelLinearVelocityLeftFrame_) = T_I_WheelLeft;

  std::cout << YELLOW_START << "Smb-StaticTransforms" << COLOR_END << " Translation I_WheelLeft: "
            << rv_T_frame1_frame2(imuFrame_, wheelLinearVelocityLeftFrame_).translation()
            << ", Rotation: " << T_I_WheelLeft.matrix().block<3, 3>(0, 0) << std::endl;
  lv_T_frame1_frame2(wheelLinearVelocityLeftFrame_, imuFrame_) = rv_T_frame1_frame2(imuFrame_, wheelLinearVelocityLeftFrame_).inverse();

  // Right Wheel
  REGULAR_COUT << COLOR_END << " Waiting for transform for 10 seconds." << std::endl;
  transform = tf_buffer_->lookupTransform(imuFrame_, wheelLinearVelocityRightFrame_, tf2::TimePointZero, tf2::durationFromSec(1.0));
  Eigen::Isometry3d T_I_WheelRight = tf2::transformToEigen(transform.transform);
  T_I_WheelRight.matrix().block<3, 3>(0, 0) = rv_T_frame1_frame2(imuFrame_, baseLinkFrame_).matrix().block<3, 3>(0, 0);
  lv_T_frame1_frame2(imuFrame_, wheelLinearVelocityRightFrame_) = T_I_WheelRight;

  std::cout << YELLOW_START << "Smb-StaticTransforms" << COLOR_END << " Translation I_WheelRight: "
            << rv_T_frame1_frame2(imuFrame_, wheelLinearVelocityRightFrame_).translation()
            << ", Rotation: " << T_I_WheelRight.matrix().block<3, 3>(0, 0) << std::endl;
  lv_T_frame1_frame2(wheelLinearVelocityRightFrame_, imuFrame_) = rv_T_frame1_frame2(imuFrame_, wheelLinearVelocityRightFrame_).inverse();

  REGULAR_COUT << GREEN_START << " Transforms looked up successfully." << COLOR_END << std::endl;
}

}  // namespace smb_se
