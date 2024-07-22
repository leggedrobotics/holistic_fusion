/*
Copyright 2024 by Julian Nubert & Timon Mathis, Robotic Systems Lab & Autonomous Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "atn_position3_fuser/Position3StaticTransforms.h"

// ROS
#include <ros/ros.h>

// Workspace
#include "graph_msf_ros/util/conversions.h"

namespace position3_se {

Position3StaticTransforms::Position3StaticTransforms(const std::shared_ptr<ros::NodeHandle> privateNodePtr,
                                                     const graph_msf::StaticTransforms& staticTransforms)
    : graph_msf::StaticTransformsTf(staticTransforms) {
  std::cout << YELLOW_START << "StaticTransformsTf" << GREEN_START << " Initializing static transforms..." << COLOR_END << std::endl;
}

void Position3StaticTransforms::findTransformations() {
  // Print to console --------------------------
  std::cout << YELLOW_START << "StaticTransformsTf" << COLOR_END << " Looking up transforms in TF-tree." << std::endl;
  std::cout << YELLOW_START << "StaticTransformsTf" << COLOR_END << " Transforms between the following frames are required:" << std::endl;
  std::cout << YELLOW_START << "StaticTransformsTf" << COLOR_END << " " << positionMeasFrame_ << ", " << imuFrame_ << std::endl;
  std::cout << YELLOW_START << "StaticTransformsTf" << COLOR_END << " Waiting for up to 100 seconds until they arrive..." << std::endl;

  // Temporary variable
  static tf::StampedTransform transform;

  // Look up transforms ----------------------------
  // Sleep before subscribing, otherwise sometimes dying in the beginning of rosbag
  ros::Rate rosRate(10);
  rosRate.sleep();

  // Imu to Prism Link ---
  listener_.waitForTransform(imuFrame_, positionMeasFrame_, ros::Time(0), ros::Duration(1.0));
  listener_.lookupTransform(imuFrame_, positionMeasFrame_, ros::Time(0), transform);

  // I_Prism
  graph_msf::tfToIsometry3(tf::Transform(transform), lv_T_frame1_frame2(imuFrame_, positionMeasFrame_));
  std::cout << YELLOW_START << "PositionEstimator" << COLOR_END << " Translation I_Prism: " << std::endl
            << rv_T_frame1_frame2(imuFrame_, positionMeasFrame_).translation() << std::endl;
  std::cout << YELLOW_START << "PositionEstimator" << COLOR_END << " Rotation I_Prism: " << std::endl
            << rv_T_frame1_frame2(imuFrame_, positionMeasFrame_).rotation() << std::endl;
  // Prism_I
  lv_T_frame1_frame2(positionMeasFrame_, imuFrame_) = rv_T_frame1_frame2(imuFrame_, positionMeasFrame_).inverse();

  // Wrapping up --------------------------
  std::cout << YELLOW_START << "StaticTransformsTf" << GREEN_START << " Transforms looked up successfully." << COLOR_END << std::endl;

  // Call parent class
  graph_msf::StaticTransformsTf::findTransformations();
}

}  // namespace position3_se
