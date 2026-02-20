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
      // Minimal robustness improvement: let the TF listener spin internally so transforms can arrive
      // even if the main executor isn't spinning yet.
      tf_listener_(*tf_buffer_, node, /*spin_thread=*/true) {
  RCLCPP_INFO(rclcpp::get_logger("graph_msf"), "StaticTransformsTf - Initializing static transforms...");
}

bool StaticTransformsTf::findTransformations() {
  RCLCPP_INFO(rclcpp::get_logger("graph_msf"), "Looking up transforms in TF-tree.");
  RCLCPP_INFO(rclcpp::get_logger("graph_msf"), "Transforms between the following frames are required: %s, %s",
              imuFrame_.c_str(), baseLinkFrame_.c_str());

  const double timeoutSeconds = 100.0;
  RCLCPP_INFO(rclcpp::get_logger("graph_msf"), "Waiting for up to %f seconds until they arrive...", timeoutSeconds);

  // Temporary variable
  geometry_msgs::msg::TransformStamped transform_stamped;

  // Imu to Base ---
  RCLCPP_INFO(rclcpp::get_logger("graph_msf"), "Looking up transform from %s to %s", imuFrame_.c_str(),
              baseLinkFrame_.c_str());

  // Robust wait: keep polling canTransform until overall timeout expires.
  const double pollSeconds = 0.1;
  rclcpp::Rate rate(1.0 / pollSeconds);

  // Use steady time for the timeout measurement (independent of sim/ros time).
  rclcpp::Clock steady_clock(RCL_STEADY_TIME);
  const rclcpp::Time start_time = steady_clock.now();
  rclcpp::Time last_log_time = start_time;

  std::string last_error;
  bool got_transform = false;

  REGULAR_COUT << COLOR_END << " Waiting for transform between " << imuFrame_ << " and " << baseLinkFrame_
               << " for " << timeoutSeconds << " seconds." << std::endl;

  while (rclcpp::ok() && (steady_clock.now() - start_time).seconds() < timeoutSeconds) {
    std::string err;

    // canTransform is non-throwing; it returns false until frames + connectivity are available.
    const bool can =
        tf_buffer_->canTransform(imuFrame_, baseLinkFrame_, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.0), &err);

    if (can) {
      try {
        // Once available, do the actual lookup (latest).
        transform_stamped = tf_buffer_->lookupTransform(imuFrame_, baseLinkFrame_, rclcpp::Time(0));
        got_transform = true;
        break;
      } catch (const tf2::TransformException& ex) {
        // Rare race: buffer changed between canTransform and lookup; keep waiting.
        last_error = ex.what();
      }
    } else {
      last_error = err;
    }

    // Throttle status output (every ~2s).
    if ((steady_clock.now() - last_log_time).seconds() >= 2.0) {
      RCLCPP_INFO(rclcpp::get_logger("graph_msf"),
                  "Still waiting for TF %s <- %s. Status: %s",
                  imuFrame_.c_str(), baseLinkFrame_.c_str(),
                  last_error.empty() ? "no data yet" : last_error.c_str());
      last_log_time = steady_clock.now();
    }

    rate.sleep();
  }

  if (!got_transform) {
    RCLCPP_ERROR(rclcpp::get_logger("graph_msf"),
                 "Transform lookup timed out after %.2f seconds. Last status: %s\nKnown frames:\n%s",
                 (steady_clock.now() - start_time).seconds(),
                 last_error.empty() ? "n/a" : last_error.c_str(),
                 tf_buffer_->allFramesAsString().c_str());
    REGULAR_COUT << RED_START << " Transform lookup timed out. Last status: "
                 << (last_error.empty() ? "n/a" : last_error) << COLOR_END << std::endl;
    return false;
  }

  // Convert + store
  Eigen::Isometry3d eigenTransform = tf2::transformToEigen(transform_stamped.transform);
  lv_T_frame1_frame2(imuFrame_, baseLinkFrame_) = eigenTransform;
  std::cout << YELLOW_START << "Ros2-StaticTransforms" << COLOR_END << " Translation I_B: " << imuFrame_ << " "
            << baseLinkFrame_ << " " << rv_T_frame1_frame2(imuFrame_, baseLinkFrame_).translation() << std::endl;
  std::cout << YELLOW_START << "Ros2-StaticTransforms" << COLOR_END << " Rotation I_B: " << imuFrame_ << " "
            << baseLinkFrame_ << " " << rv_T_frame1_frame2(imuFrame_, baseLinkFrame_).rotation() << std::endl;
  lv_T_frame1_frame2(baseLinkFrame_, imuFrame_) = rv_T_frame1_frame2(imuFrame_, baseLinkFrame_).inverse();

  // Wrap up
  RCLCPP_INFO(rclcpp::get_logger("graph_msf"), "Transforms looked up successfully.");
  return true;
}

}  // namespace graph_msf
