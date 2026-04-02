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

  // ---- MINIMAL ADDITION: also store LiDAR-odometry <-> GNSS (lever arm) if both are available ----
  if (useLioOdometryFlag_ && useGnssFlag_) {
    // T_LIO_I * T_I_G = T_LIO_G
    const Eigen::Isometry3d T_LIO_I = rv_T_frame1_frame2(lidarOdometryFrame_, imuFrame_);
    const Eigen::Isometry3d T_I_G   = rv_T_frame1_frame2(imuFrame_, gnssFrame_);
    const Eigen::Isometry3d T_LIO_G = T_LIO_I * T_I_G;

    lv_T_frame1_frame2(lidarOdometryFrame_, gnssFrame_) = T_LIO_G;
    lv_T_frame1_frame2(gnssFrame_, lidarOdometryFrame_) = T_LIO_G.inverse();

    std::cout << YELLOW_START << "B2W-StaticTransforms" << COLOR_END
              << " Translation LIO_GNSS: " << lidarOdometryFrame_ << " " << gnssFrame_
              << " " << rv_T_frame1_frame2(lidarOdometryFrame_, gnssFrame_).translation() << std::endl;
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

  // ---- MINIMAL ADDITION: also store LiDAR-between <-> GNSS if both are available ----
  if (useLioBetweenOdometryFlag_ && useGnssFlag_) {
    const Eigen::Isometry3d T_LB_I = rv_T_frame1_frame2(lidarBetweenFrame_, imuFrame_);
    const Eigen::Isometry3d T_I_G  = rv_T_frame1_frame2(imuFrame_, gnssFrame_);
    const Eigen::Isometry3d T_LB_G = T_LB_I * T_I_G;

    lv_T_frame1_frame2(lidarBetweenFrame_, gnssFrame_) = T_LB_G;
    lv_T_frame1_frame2(gnssFrame_, lidarBetweenFrame_) = T_LB_G.inverse();

    std::cout << YELLOW_START << "B2W-StaticTransforms" << COLOR_END
              << " Translation LIO_BETWEEN_GNSS: " << lidarBetweenFrame_ << " " << gnssFrame_
              << " " << rv_T_frame1_frame2(lidarBetweenFrame_, gnssFrame_).translation() << std::endl;
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

  // ---- MINIMAL ADDITION: also store VIO-odometry <-> GNSS if both are available ----
  if (useVioOdometryFlag_ && useGnssFlag_) {
    const Eigen::Isometry3d T_VIO_I = rv_T_frame1_frame2(vioOdometryFrame_, imuFrame_);
    const Eigen::Isometry3d T_I_G   = rv_T_frame1_frame2(imuFrame_, gnssFrame_);
    const Eigen::Isometry3d T_VIO_G = T_VIO_I * T_I_G;

    lv_T_frame1_frame2(vioOdometryFrame_, gnssFrame_) = T_VIO_G;
    lv_T_frame1_frame2(gnssFrame_, vioOdometryFrame_) = T_VIO_G.inverse();

    std::cout << YELLOW_START << "B2W-StaticTransforms" << COLOR_END
              << " Translation VIO_GNSS: " << vioOdometryFrame_ << " " << gnssFrame_
              << " " << rv_T_frame1_frame2(vioOdometryFrame_, gnssFrame_).translation() << std::endl;
  }

  // Imu to VIO Between Link ---
  if (useVioOdometryBetweenFlag_) {
    REGULAR_COUT << COLOR_END << " Waiting for transform between " << imuFrame_ << " and " << vioOdometryBetweenFrame_ << " for 10 seconds." << std::endl;
    transform = tf_buffer_->lookupTransform(imuFrame_, vioOdometryBetweenFrame_, tf2::TimePointZero, tf2::durationFromSec(1.0));
    eigenTransform = tf2::transformToEigen(transform.transform);
    lv_T_frame1_frame2(imuFrame_, vioOdometryBetweenFrame_) = eigenTransform;

    std::cout << YELLOW_START << "B2W-StaticTransforms" << COLOR_END << " Translation I_VIO_Between: " << imuFrame_ << " " << vioOdometryBetweenFrame_
              << " " << rv_T_frame1_frame2(imuFrame_, vioOdometryBetweenFrame_).translation() << std::endl;
    lv_T_frame1_frame2(vioOdometryBetweenFrame_, imuFrame_) = rv_T_frame1_frame2(imuFrame_, vioOdometryBetweenFrame_).inverse();
  }

  // ---- MINIMAL ADDITION: also store VIO-between <-> GNSS if both are available ----
  if (useVioOdometryBetweenFlag_ && useGnssFlag_) {
    const Eigen::Isometry3d T_VB_I = rv_T_frame1_frame2(vioOdometryBetweenFrame_, imuFrame_);
    const Eigen::Isometry3d T_I_G  = rv_T_frame1_frame2(imuFrame_, gnssFrame_);
    const Eigen::Isometry3d T_VB_G = T_VB_I * T_I_G;

    lv_T_frame1_frame2(vioOdometryBetweenFrame_, gnssFrame_) = T_VB_G;
    lv_T_frame1_frame2(gnssFrame_, vioOdometryBetweenFrame_) = T_VB_G.inverse();

    std::cout << YELLOW_START << "B2W-StaticTransforms" << COLOR_END
              << " Translation VIO_BETWEEN_GNSS: " << vioOdometryBetweenFrame_ << " " << gnssFrame_
              << " " << rv_T_frame1_frame2(vioOdometryBetweenFrame_, gnssFrame_).translation() << std::endl;
  }

  REGULAR_COUT << GREEN_START << " Transforms looked up successfully." << COLOR_END << std::endl;
  return true;
}

}  // namespace b2w_se
