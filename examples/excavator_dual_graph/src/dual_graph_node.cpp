// ROS
#include <ros/ros.h>
// Local packages
#include "excavator_dual_graph/ExcavatorEstimator.h"

// Main node entry point
int main(int argc, char** argv) {
  // ROS related
  ros::init(argc, argv, "laserOdometry");
  std::shared_ptr<ros::NodeHandle> privateNodePtr = std::make_shared<ros::NodeHandle>("~");
  /// Do multi-threaded spinner
  ros::MultiThreadedSpinner spinner(4);

  // Create Instance
  excavator_se::ExcavatorEstimator excavatorEstimator(privateNodePtr);
  spinner.spin();

  return 0;
}