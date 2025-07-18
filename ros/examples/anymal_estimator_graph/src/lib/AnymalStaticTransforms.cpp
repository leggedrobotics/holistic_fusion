/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "anymal_estimator_graph/AnymalStaticTransforms.h"

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>

// Workspace
#include "anymal_estimator_graph/constants.h"
#include "graph_msf_ros/util/conversions.h"

// Macro Constant
#define TF_WAIT_TIME 30.0

namespace anymal_se {

AnymalStaticTransforms::AnymalStaticTransforms(const std::shared_ptr<ros::NodeHandle> privateNodePtr) : graph_msf::StaticTransformsTf() {
  REGULAR_COUT << GREEN_START << " Initializing static transforms..." << COLOR_END << std::endl;
}

void AnymalStaticTransforms::findTransformations() {
  // Print to console --------------------------
  REGULAR_COUT << " Looking up transforms in TF-tree." << std::endl;
  REGULAR_COUT << " Transforms between the following frames are required: " << lioOdometryFrame_ << ", " << gnssFrame_ << ", " << imuFrame_
               << ", " << baseLinkFrame_ << std::endl;
  REGULAR_COUT << " Waiting for up to " << TF_WAIT_TIME << " seconds until they arrive..." << std::endl;

  // Temporary variable
  static tf::StampedTransform transform;

  // Look up transforms ----------------------------
  // Sleep before subscribing, otherwise sometimes dying in the beginning of rosbag
  ros::Rate rosRate(10);
  rosRate.sleep();

  // Imu to LiDAR Link ---
  REGULAR_COUT << " Looking up transform from " << imuFrame_ << " to " << lioOdometryFrame_ << std::endl;
  listener_.waitForTransform(imuFrame_, lioOdometryFrame_, ros::Time(0), ros::Duration(TF_WAIT_TIME));
  listener_.lookupTransform(imuFrame_, lioOdometryFrame_, ros::Time(0), transform);
  // I_Lidar
  graph_msf::tfToIsometry3(tf::Transform(transform), lv_T_frame1_frame2(imuFrame_, lioOdometryFrame_));
  REGULAR_COUT << " Translation I_Lidar: " << rv_T_frame1_frame2(imuFrame_, lioOdometryFrame_).translation() << std::endl;
  // Lidar_I
  lv_T_frame1_frame2(lioOdometryFrame_, imuFrame_) = rv_T_frame1_frame2(imuFrame_, lioOdometryFrame_).inverse();

  // Imu to GNSS Link ---
  try {
    REGULAR_COUT << " Looking up transform from " << imuFrame_ << " to " << gnssFrame_ << std::endl;
    listener_.waitForTransform(imuFrame_, gnssFrame_, ros::Time(0), ros::Duration(TF_WAIT_TIME));
    listener_.lookupTransform(imuFrame_, gnssFrame_, ros::Time(0), transform);
    // I_Gnss
    graph_msf::tfToIsometry3(tf::Transform(transform), lv_T_frame1_frame2(imuFrame_, gnssFrame_));
    REGULAR_COUT << " Translation I_GnssL: " << rv_T_frame1_frame2(imuFrame_, gnssFrame_).translation() << std::endl;
    // GnssL_I
    lv_T_frame1_frame2(gnssFrame_, imuFrame_) = rv_T_frame1_frame2(imuFrame_, gnssFrame_).inverse();
  } catch (const tf2::LookupException& e) {
    REGULAR_COUT << RED_START << " Could not find transformations for GNSS in TF tree." << COLOR_END << std::endl;
  }

  REGULAR_COUT << GREEN_START << " Transforms looked up successfully." << COLOR_END << std::endl;

  // Call parent class
  graph_msf::StaticTransformsTf::findTransformations();
}

}  // namespace anymal_se
