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

  rclcpp::NodeOptions opts;
  opts.use_intra_process_comms(true);

  auto node = std::make_shared<b2w_se::B2WEstimator>("b2w_estimator_node", opts);
  node->setup(node);

  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), /*num_threads=*/4);
  exec.add_node(node);  // add the B2WEstimator itself
  exec.spin();

  // Shutdown ROS 2
  rclcpp::shutdown();

  return 0;
}