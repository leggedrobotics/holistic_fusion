/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "smb_estimator_graph/SmbStaticTransforms.h"

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>

// Workspace
#include "graph_msf_ros/util/conversions.h"
#include "smb_estimator_graph/constants.h"

namespace smb_se {

SmbStaticTransforms::SmbStaticTransforms(const std::shared_ptr<ros::NodeHandle> privateNodePtr,
                                         const graph_msf::StaticTransforms& staticTransforms)
    : graph_msf::StaticTransformsTf(staticTransforms) {
  REGULAR_COUT << GREEN_START << " Initializing static transforms..." << COLOR_END << std::endl;
}

void SmbStaticTransforms::findTransformations() {
  // Print to console --------------------------
  REGULAR_COUT << COLOR_END << " Looking up transforms in TF-tree." << std::endl;
  REGULAR_COUT << COLOR_END << " Transforms between the following frames are required:" << std::endl;
  REGULAR_COUT << COLOR_END << " Waiting for up to 100 seconds until they arrive..." << std::endl;

  // Temporary variable
  static tf::StampedTransform transform;

  // Look up transforms ----------------------------
  // Sleep before subscribing, otherwise sometimes dying in the beginning of
  // rosbag
  ros::Rate rosRate(10);
  rosRate.sleep();

  // Imu to Base Link ---
  listener_.waitForTransform(imuFrame_, baseLinkFrame_, ros::Time(0), ros::Duration(100.0));
  listener_.lookupTransform(imuFrame_, baseLinkFrame_, ros::Time(0), transform);
  // I_Cabin
  graph_msf::tfToIsometry3(tf::Transform(transform), lv_T_frame1_frame2(imuFrame_, baseLinkFrame_));
  std::cout << YELLOW_START << "Smb-StaticTransforms" << COLOR_END
            << " Translation I_Base: " << rv_T_frame1_frame2(imuFrame_, baseLinkFrame_).translation() << std::endl;
  // Cabin_I
  lv_T_frame1_frame2(baseLinkFrame_, imuFrame_) = rv_T_frame1_frame2(imuFrame_, baseLinkFrame_).inverse();

  // Imu to LiDAR Link ---
  REGULAR_COUT << COLOR_END << " Waiting for transform for 10 seconds.";
  listener_.waitForTransform(imuFrame_, lidarOdometryFrame_, ros::Time(0), ros::Duration(1.0));
  listener_.lookupTransform(imuFrame_, lidarOdometryFrame_, ros::Time(0), transform);
  // I_Lidar
  graph_msf::tfToIsometry3(tf::Transform(transform), lv_T_frame1_frame2(imuFrame_, lidarOdometryFrame_));
  std::cout << YELLOW_START << "Smb-StaticTransforms" << COLOR_END
            << " Translation I_Lidar: " << rv_T_frame1_frame2(imuFrame_, lidarOdometryFrame_).translation() << std::endl;
  // Lidar_I
  lv_T_frame1_frame2(lidarOdometryFrame_, imuFrame_) = rv_T_frame1_frame2(imuFrame_, lidarOdometryFrame_).inverse();

  // Imu to VIO Link ---
  REGULAR_COUT << COLOR_END << " Waiting for transform for 10 seconds.";
  listener_.waitForTransform(imuFrame_, vioOdometryFrame_, ros::Time(0), ros::Duration(1.0));
  listener_.lookupTransform(imuFrame_, vioOdometryFrame_, ros::Time(0), transform);
  // I_VIO
  graph_msf::tfToIsometry3(tf::Transform(transform), lv_T_frame1_frame2(imuFrame_, vioOdometryFrame_));
  std::cout << YELLOW_START << "Smb-StaticTransforms" << COLOR_END
            << " Translation I_VIO: " << rv_T_frame1_frame2(imuFrame_, vioOdometryFrame_).translation() << std::endl;
  // VIO_I
  lv_T_frame1_frame2(vioOdometryFrame_, imuFrame_) = rv_T_frame1_frame2(imuFrame_, vioOdometryFrame_).inverse();

  REGULAR_COUT << GREEN_START << " Transforms looked up successfully." << COLOR_END << std::endl;
}

}  // namespace smb_se
