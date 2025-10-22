/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "b2w_estimator_graph_ros2/B2WStaticTransforms.h"

// ROS 2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/rclcpp.hpp>

// Workspace
#include "graph_msf_ros2/util/conversions.h"
#include "b2w_estimator_graph_ros2/constants.h"

namespace b2w_se {

B2WStaticTransforms::B2WStaticTransforms(const rclcpp::Node::SharedPtr& nodePtr) : graph_msf::StaticTransformsTf(nodePtr) {
  REGULAR_COUT << GREEN_START << " Initializing b2w static transforms..." << COLOR_END << std::endl;
}

bool B2WStaticTransforms::findTransformations() {
  // Super Method
  // Need to find the transformations in the TF-tree
  if (!graph_msf::StaticTransformsTf::findTransformations()) {
    REGULAR_COUT << RED_START << " Failed to find transformations in TF-tree." << COLOR_END << std::endl;
    return false;
  }

  // Print to console
  REGULAR_COUT << COLOR_END << " Looking up transforms in TF-tree." << std::endl;
  REGULAR_COUT << COLOR_END << " Waiting for up to 100 seconds until they arrive..." << std::endl;

  // Temporary variable
  geometry_msgs::msg::TransformStamped transform;
  Eigen::Isometry3d eigenTransform;

  // Imu to LiDAR Link ---
  if (useLioOdometryFlag_) {
    REGULAR_COUT << COLOR_END << " Waiting for transform between " << imuFrame_ << " and " << lidarOdometryFrame_ << " for 10 seconds." << std::endl;
    transform = tf_buffer_->lookupTransform(imuFrame_, lidarOdometryFrame_, tf2::TimePointZero, tf2::durationFromSec(1.0));
    eigenTransform = tf2::transformToEigen(transform.transform);
    lv_T_frame1_frame2(imuFrame_, lidarOdometryFrame_) = eigenTransform;

    std::cout << YELLOW_START << "B2W-StaticTransforms" << COLOR_END << " Translation I_Lidar: " << imuFrame_ << " " << lidarOdometryFrame_
              << " " << rv_T_frame1_frame2(imuFrame_, lidarOdometryFrame_).translation() << std::endl;
    lv_T_frame1_frame2(lidarOdometryFrame_, imuFrame_) = rv_T_frame1_frame2(imuFrame_, lidarOdometryFrame_).inverse();
  }
  
  // Imu to GNSS Link ---
  if (useGnssFlag_) {
    REGULAR_COUT << COLOR_END << " Waiting for transform between " << imuFrame_ << " and " << gnssFrame_ << " for 10 seconds." << std::endl;
    transform = tf_buffer_->lookupTransform(imuFrame_, gnssFrame_, tf2::TimePointZero, tf2::durationFromSec(1.0));
    eigenTransform = tf2::transformToEigen(transform.transform);
    lv_T_frame1_frame2(imuFrame_, gnssFrame_) = eigenTransform;

    std::cout << YELLOW_START << "B2W-StaticTransforms" << COLOR_END << " Translation I_GNSS: " << imuFrame_ << " " << gnssFrame_
              << " " << rv_T_frame1_frame2(imuFrame_, gnssFrame_).translation() << std::endl;
    lv_T_frame1_frame2(gnssFrame_, imuFrame_) = rv_T_frame1_frame2(imuFrame_, gnssFrame_).inverse();
  }

  if (useLioBetweenOdometryFlag_) {
    REGULAR_COUT << COLOR_END << " Waiting for transform between " << imuFrame_ << " and " << lidarBetweenFrame_ << " for 10 seconds." << std::endl;
    transform = tf_buffer_->lookupTransform(imuFrame_, lidarBetweenFrame_, tf2::TimePointZero, tf2::durationFromSec(1.0));
    eigenTransform = tf2::transformToEigen(transform.transform);
    lv_T_frame1_frame2(imuFrame_, lidarBetweenFrame_) = eigenTransform;

    std::cout << YELLOW_START << "B2W-StaticTransforms" << COLOR_END << " Translation I_Lidar: " << imuFrame_ << " " << lidarBetweenFrame_
              << " " << rv_T_frame1_frame2(imuFrame_, lidarBetweenFrame_).translation() << std::endl;
    lv_T_frame1_frame2(lidarBetweenFrame_, imuFrame_) = rv_T_frame1_frame2(imuFrame_, lidarBetweenFrame_).inverse();
  }


  // Imu to VIO Link ---
  if (useVioOdometryFlag_) {
    REGULAR_COUT << COLOR_END << " Waiting for transform between " << imuFrame_ << " and " << vioOdometryFrame_ << " for 10 seconds." << std::endl;
    transform = tf_buffer_->lookupTransform(imuFrame_, vioOdometryFrame_, tf2::TimePointZero, tf2::durationFromSec(1.0));
    eigenTransform = tf2::transformToEigen(transform.transform);
    lv_T_frame1_frame2(imuFrame_, vioOdometryFrame_) = eigenTransform;

    std::cout << YELLOW_START << "B2W-StaticTransforms" << COLOR_END << " Translation I_VIO: " << imuFrame_ << " " << vioOdometryFrame_
              << " " << rv_T_frame1_frame2(imuFrame_, vioOdometryFrame_).translation() << std::endl;
    lv_T_frame1_frame2(vioOdometryFrame_, imuFrame_) = rv_T_frame1_frame2(imuFrame_, vioOdometryFrame_).inverse();
  }

  // Wheel Frames ---
  if (useWheelOdometryBetweenFlag_ || useWheelLinearVelocitiesFlag_) {
    REGULAR_COUT
        << RED_START
        << " As the Wheels are turning, we only use the position of the wheels frames and use the orientation of the baseLinkFrame_ frame."
        << COLOR_END << std::endl;

    // Left Wheel
    REGULAR_COUT << COLOR_END << " Waiting for transform for 10 seconds." << std::endl;
    transform = tf_buffer_->lookupTransform(imuFrame_, wheelLinearVelocityLeftFrame_, tf2::TimePointZero, tf2::durationFromSec(1.0));
    Eigen::Isometry3d T_I_WheelLeft = tf2::transformToEigen(transform.transform);
    T_I_WheelLeft.matrix().block<3, 3>(0, 0) = rv_T_frame1_frame2(imuFrame_, baseLinkFrame_).matrix().block<3, 3>(0, 0);
    lv_T_frame1_frame2(imuFrame_, wheelLinearVelocityLeftFrame_) = T_I_WheelLeft;

    std::cout << YELLOW_START << "B2W-StaticTransforms" << COLOR_END
              << " Translation I_WheelLeft: " << rv_T_frame1_frame2(imuFrame_, wheelLinearVelocityLeftFrame_).translation()
              << ", Rotation: " << T_I_WheelLeft.matrix().block<3, 3>(0, 0) << std::endl;
    lv_T_frame1_frame2(wheelLinearVelocityLeftFrame_, imuFrame_) = rv_T_frame1_frame2(imuFrame_, wheelLinearVelocityLeftFrame_).inverse();

    // Right Wheel
    REGULAR_COUT << COLOR_END << " Waiting for transform for 10 seconds." << std::endl;
    transform = tf_buffer_->lookupTransform(imuFrame_, wheelLinearVelocityRightFrame_, tf2::TimePointZero, tf2::durationFromSec(1.0));
    Eigen::Isometry3d T_I_WheelRight = tf2::transformToEigen(transform.transform);
    T_I_WheelRight.matrix().block<3, 3>(0, 0) = rv_T_frame1_frame2(imuFrame_, baseLinkFrame_).matrix().block<3, 3>(0, 0);
    lv_T_frame1_frame2(imuFrame_, wheelLinearVelocityRightFrame_) = T_I_WheelRight;

    std::cout << YELLOW_START << "B2W-StaticTransforms" << COLOR_END
              << " Translation I_WheelRight: " << rv_T_frame1_frame2(imuFrame_, wheelLinearVelocityRightFrame_).translation()
              << ", Rotation: " << T_I_WheelRight.matrix().block<3, 3>(0, 0) << std::endl;
    lv_T_frame1_frame2(wheelLinearVelocityRightFrame_, imuFrame_) = rv_T_frame1_frame2(imuFrame_, wheelLinearVelocityRightFrame_).inverse();
  }

  REGULAR_COUT << GREEN_START << " Transforms looked up successfully." << COLOR_END << std::endl;
  return true;
}

}  // namespace b2w_se
