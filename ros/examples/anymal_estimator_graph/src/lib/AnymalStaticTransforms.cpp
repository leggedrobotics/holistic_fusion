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

// std
#include <array>

// Workspace
#include "anymal_estimator_graph/constants.h"
#include "graph_msf_ros/defaults.h"
#include "graph_msf_ros/util/conversions.h"

namespace anymal_se {

AnymalStaticTransforms::AnymalStaticTransforms(const std::shared_ptr<ros::NodeHandle> privateNodePtr) : graph_msf::StaticTransformsTf() {
  REGULAR_COUT << GREEN_START << " Initializing static transforms..." << COLOR_END << std::endl;
}

bool AnymalStaticTransforms::findTransformations() {
  // Print to console --------------------------
  REGULAR_COUT << " Looking up transforms in TF-tree." << std::endl;
  REGULAR_COUT << " Transforms between the following frames are required: " << lioOdometryFrame_ << ", " << gnssFrame_ << ", "
               << vioOdometryFrame_ << ", " << imuFrame_ << ", " << baseLinkFrame_ << std::endl;
  REGULAR_COUT << " Waiting for up to " << graph_msf::kDefaultTfLookupTimeoutSeconds << " seconds until they arrive..." << std::endl;

  // Temporary variable
  static tf::StampedTransform transform;

  // Look up transforms ----------------------------
  // Sleep before subscribing, otherwise sometimes dying in the beginning of rosbag
  ros::Rate rosRate(10);
  rosRate.sleep();

  try {
    // Imu to LiDAR Link ---
    if (!lioOdometryFrame_.empty()) {
      REGULAR_COUT << " Looking up transform from " << imuFrame_ << " to " << lioOdometryFrame_ << std::endl;
      listener_.waitForTransform(imuFrame_, lioOdometryFrame_, ros::Time(0), ros::Duration(graph_msf::kDefaultTfLookupTimeoutSeconds));
      listener_.lookupTransform(imuFrame_, lioOdometryFrame_, ros::Time(0), transform);
      // I_Lidar
      graph_msf::tfToIsometry3(tf::Transform(transform), lv_T_frame1_frame2(imuFrame_, lioOdometryFrame_));
      REGULAR_COUT << " Translation I_Lidar: " << rv_T_frame1_frame2(imuFrame_, lioOdometryFrame_).translation() << std::endl;
      // Lidar_I
      lv_T_frame1_frame2(lioOdometryFrame_, imuFrame_) = rv_T_frame1_frame2(imuFrame_, lioOdometryFrame_).inverse();
    }

    // Imu to GNSS Link ---
    if (!gnssFrame_.empty()) {
      REGULAR_COUT << " Looking up transform from " << imuFrame_ << " to " << gnssFrame_ << std::endl;
      listener_.waitForTransform(imuFrame_, gnssFrame_, ros::Time(0), ros::Duration(graph_msf::kDefaultTfLookupTimeoutSeconds));
      listener_.lookupTransform(imuFrame_, gnssFrame_, ros::Time(0), transform);
      // I_Gnss
      graph_msf::tfToIsometry3(tf::Transform(transform), lv_T_frame1_frame2(imuFrame_, gnssFrame_));
      REGULAR_COUT << " Translation I_GnssL: " << rv_T_frame1_frame2(imuFrame_, gnssFrame_).translation() << std::endl;
      // GnssL_I
      lv_T_frame1_frame2(gnssFrame_, imuFrame_) = rv_T_frame1_frame2(imuFrame_, gnssFrame_).inverse();
    }

    // Imu to VIO Link ---
    if (!vioOdometryFrame_.empty()) {
      REGULAR_COUT << " Looking up transform from " << imuFrame_ << " to " << vioOdometryFrame_ << std::endl;
      listener_.waitForTransform(imuFrame_, vioOdometryFrame_, ros::Time(0), ros::Duration(graph_msf::kDefaultTfLookupTimeoutSeconds));
      listener_.lookupTransform(imuFrame_, vioOdometryFrame_, ros::Time(0), transform);
      // I_Vio
      graph_msf::tfToIsometry3(tf::Transform(transform), lv_T_frame1_frame2(imuFrame_, vioOdometryFrame_));
      REGULAR_COUT << " Translation I_Vio: " << rv_T_frame1_frame2(imuFrame_, vioOdometryFrame_).translation() << std::endl;
      // Vio_I
      lv_T_frame1_frame2(vioOdometryFrame_, imuFrame_) = rv_T_frame1_frame2(imuFrame_, vioOdometryFrame_).inverse();
    }
  } catch (const tf::TransformException& e) {
    REGULAR_COUT << RED_START << " Could not find Anymal-specific transforms in TF tree: " << e.what() << COLOR_END << std::endl;
    return false;
  }

  // Call parent class
  if (!graph_msf::StaticTransformsTf::findTransformations()) {
    return false;
  }

  const std::array<std::string, 5> knownFrames = {imuFrame_, baseLinkFrame_, lioOdometryFrame_, gnssFrame_, vioOdometryFrame_};
  for (const auto& frame1 : knownFrames) {
    if (frame1.empty()) {
      continue;
    }
    for (const auto& frame2 : knownFrames) {
      if (frame2.empty() || frame1 == frame2 || isFramePairInDictionary(frame1, frame2)) {
        continue;
      }
      if (!isFramePairInDictionary(frame1, imuFrame_) || !isFramePairInDictionary(imuFrame_, frame2)) {
        continue;
      }

      // The estimator frequently queries sensor-to-sensor pairs directly, while TF lookup above only
      // stores IMU-anchored transforms. Materialize the derived static pair here once at startup.
      set_T_frame1_frame2_andInverse(frame1, frame2, rv_T_frame1_frame2(frame1, imuFrame_) * rv_T_frame1_frame2(imuFrame_, frame2));
      REGULAR_COUT << " Cached derived transform from " << frame1 << " to " << frame2 << " via " << imuFrame_ << std::endl;
    }
  }

  REGULAR_COUT << GREEN_START << " Transforms looked up successfully." << COLOR_END << std::endl;
  return true;
}

}  // namespace anymal_se
