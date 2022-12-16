// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>

// Workspace
#include "compslam_se_ros/extrinsics/StaticTransformsTf.h"
#include "compslam_se_ros/util/conversions.h"

namespace compslam_se {

StaticTransformsTf::StaticTransformsTf(ros::NodeHandle& privateNode) {
  std::cout << YELLOW_START << "StaticTransformsTf" << GREEN_START << " Initializing static transforms..." << COLOR_END << std::endl;
}

}  // namespace compslam_se
