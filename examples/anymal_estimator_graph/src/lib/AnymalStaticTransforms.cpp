/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
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
#include "graph_msf_ros/util/conversions.h"

namespace anymal_se {

AnymalStaticTransforms::AnymalStaticTransforms(const std::shared_ptr<ros::NodeHandle> privateNodePtr,
                                               const graph_msf::StaticTransforms& staticTransforms)
    : graph_msf::StaticTransformsTf(staticTransforms) {
  std::cout << YELLOW_START << "AnymalStaticTransforms" << GREEN_START << " Initializing static transforms..." << COLOR_END << std::endl;
}

void AnymalStaticTransforms::findTransformations() {
  // Print to console --------------------------
  std::cout << YELLOW_START << "StaticTransformsTf" << COLOR_END << " Looking up transforms in TF-tree." << std::endl;
  std::cout << YELLOW_START << "StaticTransformsTf" << COLOR_END << " Transforms between the following frames are required:" << std::endl;
  std::cout << YELLOW_START << "StaticTransformsTf" << COLOR_END << " " << lidarFrame_ << ", " << gnssFrame_ << ", " << imuFrame_ << ", "
            << baseLinkFrame_ << std::endl;
  std::cout << YELLOW_START << "StaticTransformsTf" << COLOR_END << " Waiting for up to 100 seconds until they arrive..." << std::endl;

  // Temporary variable
  static tf::StampedTransform transform;

  // Look up transforms ----------------------------
  // Sleep before subscribing, otherwise sometimes dying in the beginning of rosbag
  ros::Rate rosRate(10);
  rosRate.sleep();

  // Imu to Base Link ---
  listener_.waitForTransform(imuFrame_, baseLinkFrame_, ros::Time(0), ros::Duration(100.0));
  listener_.lookupTransform(imuFrame_, baseLinkFrame_, ros::Time(0), transform);
  // I_Cabin
  graph_msf::tfToIsometry3(tf::Transform(transform), lv_T_frame1_frame2(imuFrame_, baseLinkFrame_));
  std::cout << YELLOW_START << "CompslamEstimator" << COLOR_END
            << " Translation I_Base: " << rv_T_frame1_frame2(imuFrame_, baseLinkFrame_).translation() << std::endl;
  // Cabin_I
  lv_T_frame1_frame2(baseLinkFrame_, imuFrame_) = rv_T_frame1_frame2(imuFrame_, baseLinkFrame_).inverse();

  // Imu to LiDAR Link ---
  std::cout << YELLOW_START << "StaticTransformsTf" << COLOR_END << " Waiting for transform for 10 seconds.";
  listener_.waitForTransform(imuFrame_, lidarFrame_, ros::Time(0), ros::Duration(1.0));
  listener_.lookupTransform(imuFrame_, lidarFrame_, ros::Time(0), transform);
  // I_Lidar
  graph_msf::tfToIsometry3(tf::Transform(transform), lv_T_frame1_frame2(imuFrame_, lidarFrame_));
  std::cout << YELLOW_START << "CompslamEstimator" << COLOR_END
            << " Translation I_Lidar: " << rv_T_frame1_frame2(imuFrame_, lidarFrame_).translation() << std::endl;
  // Lidar_I
  lv_T_frame1_frame2(lidarFrame_, imuFrame_) = rv_T_frame1_frame2(imuFrame_, lidarFrame_).inverse();

  // Imu to GNSS Link ---
  try {
    listener_.waitForTransform(imuFrame_, gnssFrame_, ros::Time(0), ros::Duration(1.0));
    listener_.lookupTransform(imuFrame_, gnssFrame_, ros::Time(0), transform);
    // I_Gnss
    graph_msf::tfToIsometry3(tf::Transform(transform), lv_T_frame1_frame2(imuFrame_, gnssFrame_));
    std::cout << YELLOW_START << "CompslamEstimator" << COLOR_END
              << " Translation I_GnssL: " << rv_T_frame1_frame2(imuFrame_, gnssFrame_).translation() << std::endl;
    // GnssL_I
    lv_T_frame1_frame2(gnssFrame_, imuFrame_) = rv_T_frame1_frame2(imuFrame_, gnssFrame_).inverse();
  } catch (const tf2::LookupException& e) {
    std::cout << YELLOW_START << "CompslamEstimator" << RED_START << " Could not find transformations for GNSS in TF tree." << COLOR_END
              << std::endl;
  }

  std::cout << YELLOW_START << "StaticTransformsTf" << GREEN_START << " Transforms looked up successfully." << COLOR_END << std::endl;
}

}  // namespace anymal_se
