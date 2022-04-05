// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>

// Workspace
#include "compslam_ros/StaticTransforms.h"

namespace compslam_ros {

StaticTransforms::StaticTransforms(ros::NodeHandle& privateNode) {
  ROS_INFO("Initializing StaticTransforms");
}

void StaticTransforms::findTransformations() {
  ROS_WARN("Looking up transformations in TF-Tree...");

  // Temporary variable
  tf::StampedTransform transform;
  // Lidar to imu link
  listener_.waitForTransform(lidarFrame_, imuFrame_, ros::Time(0), ros::Duration(3.0));
  listener_.lookupTransform(lidarFrame_, imuFrame_, ros::Time(0), transform);
  tf_T_L_I_ = tf::Transform(transform);
  std::cout << YELLOW_START << "CompslamEstimator" << COLOR_END << "Transform L_I: " << tf_T_L_I_.getOrigin().getX() << ","
            << tf_T_L_I_.getOrigin().getY() << "," << tf_T_L_I_.getOrigin().getZ() << std::endl;
  //  // left gnss to imu link
  //  listener_.waitForTransform(leftGnssFrame_, imuFrame_, ros::Time(0), ros::Duration(3.0));
  //  listener_.lookupTransform(leftGnssFrame_, imuFrame_, ros::Time(0), transform);
  //  tf_T_GnssL_I_ = transform;
  //  std::cout << YELLOW_START << "CompslamEstimator" << COLOR_END << "Transform GnssL_I: " << tf_T_GnssL_I_.getOrigin().getX() << ","
  //            << tf_T_GnssL_I_.getOrigin().getY() << "," << tf_T_GnssL_I_.getOrigin().getZ() << std::endl;
  //  // right gnss to imu link
  //  listener_.waitForTransform(rightGnssFrame_, imuFrame_, ros::Time(0), ros::Duration(3.0));
  //  listener_.lookupTransform(rightGnssFrame_, imuFrame_, ros::Time(0), transform);
  //  tf_T_GnssR_I_ = transform;
  //  std::cout << YELLOW_START << "CompslamEstimator" << COLOR_END << "Transform GnssR_I: " << tf_T_GnssR_I_.getOrigin().getX() << ","
  //            << tf_T_GnssR_I_.getOrigin().getY() << "," << tf_T_GnssR_I_.getOrigin().getZ() << std::endl;
}

}  // namespace compslam_ros
