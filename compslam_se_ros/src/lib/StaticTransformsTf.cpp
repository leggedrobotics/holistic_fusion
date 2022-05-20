// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>

// Workspace
#include "compslam_se_ros/extrinsics/StaticTransformsTf.h"

namespace compslam_se {

StaticTransformsTf::StaticTransformsTf(ros::NodeHandle& privateNode) {
  std::cout << YELLOW_START << "StaticTransformsTf" << GREEN_START << " Initializing static transforms..." << COLOR_END << std::endl;
}

void StaticTransformsTf::findTransformations() {
  std::cout << YELLOW_START << "StaticTransformsTf" << COLOR_END << " Looking up transforms in TF-tree.";

  // Temporary variable
  tf::StampedTransform transform;
  // Lidar to imu link
  std::cout << YELLOW_START << "StaticTransformsTf" << COLOR_END << " Waiting for transform for 10 seconds.";
  listener_.waitForTransform(lidarFrame_, imuFrame_, ros::Time(0), ros::Duration(10.0));
  listener_.lookupTransform(lidarFrame_, imuFrame_, ros::Time(0), transform);
  tf_T_L_I_ = tf::Transform(transform);
  std::cout << YELLOW_START << "CompslamEstimator" << COLOR_END << " Transform L_I: " << tf_T_L_I_.getOrigin().getX() << ","
            << tf_T_L_I_.getOrigin().getY() << "," << tf_T_L_I_.getOrigin().getZ() << std::endl;

  std::cout << YELLOW_START << "StaticTransformsTf" << GREEN_START << " Transforms looked up successfully." << COLOR_END << std::endl;
}

}  // namespace compslam_se
