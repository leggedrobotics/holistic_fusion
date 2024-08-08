/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "graph_msf_ros/extrinsics/StaticTransformsTf.h"

// ROS
#include <ros/ros.h>

// Workspace
#include "graph_msf_ros/constants.h"
#include "graph_msf_ros/util/conversions.h"

namespace graph_msf {

StaticTransformsTf::StaticTransformsTf(const graph_msf::StaticTransforms& staticTransforms) : StaticTransforms(staticTransforms) {
  std::cout << YELLOW_START << "StaticTransformsTf" << GREEN_START << " Initializing static transforms..." << COLOR_END << std::endl;
}

void StaticTransformsTf::findTransformations() {
  // Print to console --------------------------
  REGULAR_COUT << " Looking up transforms in TF-tree." << std::endl;
  REGULAR_COUT << " Transforms between the following frames are required: " << imuFrame_ << ", " << baseLinkFrame_ << std::endl;
  REGULAR_COUT << " Waiting for up to 100 seconds until they arrive..." << std::endl;

  // Temporary variable
  static tf::StampedTransform transform;

  // Look up transforms ----------------------------
  // Sleep before subscribing, otherwise sometimes dying in the beginning of rosbag
  ros::Rate rosRate(10);
  rosRate.sleep();

  // Imu to Base ---
  REGULAR_COUT << " Looking up transform from " << imuFrame_ << " to " << baseLinkFrame_ << std::endl;
  listener_.waitForTransform(imuFrame_, baseLinkFrame_, ros::Time(0), ros::Duration(100.0));
  listener_.lookupTransform(imuFrame_, baseLinkFrame_, ros::Time(0), transform);
  // I_Base
  graph_msf::tfToIsometry3(tf::Transform(transform), lv_T_frame1_frame2(imuFrame_, baseLinkFrame_));
  REGULAR_COUT << " Translation I_Base: " << rv_T_frame1_frame2(imuFrame_, baseLinkFrame_).translation() << std::endl;
  // Base_I
  lv_T_frame1_frame2(baseLinkFrame_, imuFrame_) = rv_T_frame1_frame2(imuFrame_, baseLinkFrame_).inverse();

  // Wrapping up --------------------------
  REGULAR_COUT << " Transforms looked up successfully." << std::endl;
}

}  // namespace graph_msf
