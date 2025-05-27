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

StaticTransformsTf::StaticTransformsTf(const rclcpp::Node::SharedPtr& node,
                                       const graph_msf::StaticTransforms& staticTransforms)
    : StaticTransforms(staticTransforms),
      tf_buffer_(std::make_shared<tf2_ros::Buffer>(node->get_clock())),
      tf_listener_(*tf_buffer_) {
  RCLCPP_INFO(rclcpp::get_logger("graph_msf"), "StaticTransformsTf - Initializing static transforms...");
}

void StaticTransformsTf::findTransformations() {
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
    transform_stamped =
        tf_buffer_->lookupTransform(imuFrame_, baseLinkFrame_, rclcpp::Time(0), rclcpp::Duration::from_seconds(100.0));
    tf2::Transform tf_transform;
    graph_msf::transformStampedToIsometry3(transform_stamped.transform, lv_T_frame1_frame2(imuFrame_, baseLinkFrame_));

    // graph_msf::tfToIsometry3(tf_transform, lv_T_frame1_frame2(imuFrame_, baseLinkFrame_));
    // graph_msf::tfToIsometry3(transform, lv_T_frame1_frame2(imuFrame_, baseLinkFrame_));
    // RCLCPP_INFO(rclcpp::get_logger("graph_msf"), "Translation I_Base: %s",
    //             lv_T_frame1_frame2(imuFrame_, baseLinkFrame_)
    //                 .translation()
    //                 .format(Eigen::IOFormat(Eigen::StreamPrecision))
    //                 .c_str());
    lv_T_frame1_frame2(baseLinkFrame_, imuFrame_) = lv_T_frame1_frame2(imuFrame_, baseLinkFrame_).inverse();
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(rclcpp::get_logger("graph_msf"), "Transform lookup failed: %s", ex.what());
  }

  RCLCPP_INFO(rclcpp::get_logger("graph_msf"), "Transforms looked up successfully.");
}

}  // namespace graph_msf
