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
#include "b2w_estimator_graph_ros2/B2WEstimator.h"

// Main node entry point
int main(int argc, char** argv) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Debugging
  // FLAGS_alsologtostderr = true;

  // Create Node
  auto privateNodePtr = std::make_shared<rclcpp::Node>("b2w_estimator_node");

  // Create Instance of B2WEstimator
  auto b2wEstimator = std::make_shared<b2w_se::B2WEstimator>(privateNodePtr);

  // Use Multi-Threaded Executor
  rclcpp::NodeOptions opts;
  opts.use_intra_process_comms(true);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), /*num_threads=*/4);

  executor.add_node(privateNodePtr);
  executor.spin();

  // Shutdown ROS 2
  rclcpp::shutdown();

  return 0;
}