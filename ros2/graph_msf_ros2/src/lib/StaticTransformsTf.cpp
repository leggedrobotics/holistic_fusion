/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "graph_msf_ros2/extrinsics/StaticTransformsTf.h"

// ROS
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/rclcpp.hpp>

// Workspace
#include "graph_msf_ros2/constants.h"
#include "graph_msf_ros2/util/conversions.h"

namespace graph_msf {

StaticTransformsTf::StaticTransformsTf(const rclcpp::Node::SharedPtr& node)
    : StaticTransforms(),
      tf_buffer_(std::make_shared<tf2_ros::Buffer>(node->get_clock())),
      tf_listener_(*tf_buffer_) {
  RCLCPP_INFO(rclcpp::get_logger("graph_msf"), "StaticTransformsTf - Initializing static transforms...");
}

bool StaticTransformsTf::findTransformations() {
  RCLCPP_INFO(rclcpp::get_logger("graph_msf"), "Looking up transforms in TF-tree.");
  RCLCPP_INFO(rclcpp::get_logger("graph_msf"), "Transforms between the following frames are required: %s, %s",
              imuFrame_.c_str(), baseLinkFrame_.c_str());
  RCLCPP_INFO(rclcpp::get_logger("graph_msf"), "Waiting for up to 100 seconds until they arrive...");

  // Temporary variable
  geometry_msgs::msg::TransformStamped transform_stamped;

  // Imu to Base ---
  RCLCPP_INFO(rclcpp::get_logger("graph_msf"), "Looking up transform from %s to %s", imuFrame_.c_str(),
              baseLinkFrame_.c_str());
  try {
    REGULAR_COUT << COLOR_END << " Waiting for transform between " << imuFrame_ << " and " << baseLinkFrame_ << " for 10 seconds." << std::endl;
    transform_stamped =
        tf_buffer_->lookupTransform(imuFrame_, baseLinkFrame_, rclcpp::Time(0), rclcpp::Duration::from_seconds(100.0));
    Eigen::Isometry3d eigenTransform = tf2::transformToEigen(transform_stamped.transform);
    lv_T_frame1_frame2(imuFrame_, baseLinkFrame_) = eigenTransform;
    std::cout << YELLOW_START << "Smb-StaticTransforms" << COLOR_END << " Translation I_B: " << imuFrame_ << " " << baseLinkFrame_
              << " " << rv_T_frame1_frame2(imuFrame_, baseLinkFrame_).translation() << std::endl;
    lv_T_frame1_frame2(baseLinkFrame_, imuFrame_) = rv_T_frame1_frame2(imuFrame_, baseLinkFrame_).inverse();
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(rclcpp::get_logger("graph_msf"), "Transform lookup failed: %s", ex.what());
    REGULAR_COUT << RED_START << " Transform lookup failed: " << ex.what() << COLOR_END << std::endl;
    return false;
  }
  // Wrap up
  RCLCPP_INFO(rclcpp::get_logger("graph_msf"), "Transforms looked up successfully.");
  return true;
}

}  // namespace graph_msf
