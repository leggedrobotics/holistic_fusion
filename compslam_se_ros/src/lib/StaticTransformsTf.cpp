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

void StaticTransformsTf::findTransformations() {
  std::cout << YELLOW_START << "StaticTransformsTf" << COLOR_END << " Looking up transforms in TF-tree.";

  // Temporary variable
  tf::StampedTransform transform;
  // Lidar to imu link
  std::cout << YELLOW_START << "StaticTransformsTf" << COLOR_END << " Waiting for transform for 10 seconds.";
  listener_.waitForTransform(lidarFrame_, imuFrame_, ros::Time(0), ros::Duration(10.0));
  listener_.lookupTransform(lidarFrame_, imuFrame_, ros::Time(0), transform);
  tfToMatrix4(tf::Transform(transform), lv_T_frame1_frame2(lidarFrame_, imuFrame_));
  std::cout << YELLOW_START << "CompslamEstimator" << COLOR_END
            << " Translation L_I: " << rv_T_frame1_frame2(lidarFrame_, imuFrame_).block<3, 1>(0, 3) << std::endl;

  std::cout << YELLOW_START << "StaticTransformsTf" << GREEN_START << " Transforms looked up successfully." << COLOR_END << std::endl;
}

}  // namespace compslam_se
