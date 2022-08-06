// Implementation
#include "multi_lidar_imu/MultiOdomEstimator.h"

namespace compslam_se {

MultiOdomEstimator::MultiOdomEstimator(ros::NodeHandle& node, ros::NodeHandle& privateNode) : CompslamEstimator(node, privateNode) {}

}  // namespace compslam_se