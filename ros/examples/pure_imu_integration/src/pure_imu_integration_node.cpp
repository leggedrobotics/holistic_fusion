/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// ROS
#include <ros/ros.h>
// Local packages
#include "pure_imu_integration/ImuIntegrator.h"

// Main node entry point
int main(int argc, char** argv) {
  // ROS related
  ros::init(argc, argv, "laserOdometry");
  std::shared_ptr<ros::NodeHandle> privateNodePtr = std::make_shared<ros::NodeHandle>("~");
  /// Do multi-threaded spinner
  ros::MultiThreadedSpinner spinner(4);

  // Create Instance
  imu_integrator::ImuIntegrator imuIntegrator(privateNodePtr);
  spinner.spin();

  return 0;
}