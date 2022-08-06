// C++
#include <string.h>

#include <boost/filesystem.hpp>
// ROS
#include <ros/ros.h>
// Local packages
#include "multi_lidar_imu/MultiOdomEstimator.h"

// Main node entry point
int main(int argc, char** argv) {
  // ROS related
  ros::init(argc, argv, "laserOdometry");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");
  /// Do multi-threaded spinner
  ros::MultiThreadedSpinner spinner(4);

  // Create Instance
  std::unique_ptr<compslam_se::CompslamEstimator> m545EstimatorGraphPtr =
      std::make_unique<compslam_se::MultiOdomEstimator>(node, privateNode);
  spinner.spin();

  return 0;
}