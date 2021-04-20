#include <ros/ros.h>

#include "fg_filtering/FactorGraphFiltering.h"

// Main node entry point
int main(int argc, char** argv) {
  // ROS related
  ros::init(argc, argv, "laserOdometry");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");
  /// Do multi-threaded spinner
  ros::MultiThreadedSpinner spinner(2);

  // Create Instance
  fg_filtering::FactorGraphFiltering fgFilter(0.1);
  // Setup Instance
  if (fgFilter.setup(node, privateNode)) ROS_INFO("Node is set up completely.");
  spinner.spin();

  return 0;
}
