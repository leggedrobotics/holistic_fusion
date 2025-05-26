/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
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
#include "anymal_estimator_graph/AnymalEstimator.h"

// Main node entry point
int main(int argc, char** argv) {
  // Debugging
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  google::InstallFailureSignalHandler();

  // ROS related
  ros::init(argc, argv, "anymal_estimator_graph");
  std::shared_ptr<ros::NodeHandle> privateNodePtr = std::make_shared<ros::NodeHandle>("~");
  /// Do multi-threaded spinner
  ros::MultiThreadedSpinner spinner(4);

  // Create Instance
  anymal_se::AnymalEstimator anymalEstimator(privateNodePtr);
  spinner.spin();

  return 0;
}