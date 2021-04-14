#include <ros/ros.h>

#include "fg_filtering/FactorGraphFiltering.h"

// Main node entry point
int main(int argc, char** argv) {
  ros::init(argc, argv, "laserOdometry");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  // Create Instance
  fg_filtering::FactorGraphFiltering fgFilter(0.1);
  // Setup Instance
  if (fgFilter.setup(node, privateNode))
    ROS_INFO("Node is set up completely.");
    ROS_INFO("/imu_topic");
    ros::spin();

  return 0;
}
