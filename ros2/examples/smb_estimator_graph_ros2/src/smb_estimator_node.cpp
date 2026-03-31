/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// ROS 2
#include <rclcpp/rclcpp.hpp>

// Debugging
// #include <gflags/gflags.h>

// Local packages
#include "smb_estimator_graph_ros2/SmbEstimator.h"

// Main node entry point
int main(int argc, char** argv) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Debugging
  // FLAGS_alsologtostderr = true;

  // Create Instance of SmbEstimator (which is itself a Node)
  auto smbEstimator = std::make_shared<smb_se::SmbEstimator>("smb_estimator_node");

  // Call setup after construction (needs shared_from_this)
  smbEstimator->setup();

  // Use Multi-Threaded Executor
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(smbEstimator);
  executor.spin();

  // Shutdown ROS 2
  rclcpp::shutdown();

  return 0;
}