// Implementation
#include "graph_msf_ros/extrinsics/StaticTransformsTf.h"

// ROS
#include <ros/ros.h>

// Workspace
#include "graph_msf_ros/util/conversions.h"

namespace graph_msf {

StaticTransformsTf::StaticTransformsTf(ros::NodeHandle& privateNode) {
  std::cout << YELLOW_START << "StaticTransformsTf" << GREEN_START << " Initializing static transforms..." << COLOR_END << std::endl;
}

}  // namespace graph_msf
