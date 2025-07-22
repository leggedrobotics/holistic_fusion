/*
Copyright 2024 by Julian Nubert & Timon Mathis, Robotic Systems Lab & Autonomous Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// ROS
#include <ros/ros.h>

// Debugging
#include <gflags/gflags.h>
#include <glog/logging.h>

// Local packages
#include "atn_position3_fuser/Position3Estimator.h"

// Main node entry point
int main(int argc, char** argv) {
  // Debugging
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  google::InstallFailureSignalHandler();

  // ROS related
  ros::init(argc, argv, "leica_position_graph_node");
  std::shared_ptr<ros::NodeHandle> privateNodePtr = std::make_shared<ros::NodeHandle>("~");
  /// Do multi-threaded spinner
  ros::MultiThreadedSpinner spinner(4);

  // Create Instance
  position3_se::Position3Estimator position3Estimator(privateNodePtr);
  spinner.spin();

  return 0;
}