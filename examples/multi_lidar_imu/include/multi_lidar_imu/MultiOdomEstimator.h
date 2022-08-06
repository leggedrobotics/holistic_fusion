#ifndef MULTIODOMESTIMATOR_H
#define MULTIODOMESTIMATOR_H

// Workspace
#include "compslam_se_ros/CompslamEstimator.h"

// Defined Macros
#define ROS_QUEUE_SIZE 100
#define NUM_GNSS_CALLBACKS_UNTIL_START 20  // 0

namespace compslam_se {

class MultiOdomEstimator : public CompslamEstimator {
 public:
  MultiOdomEstimator(ros::NodeHandle& node, ros::NodeHandle& privateNode);
};

}  // namespace compslam_se

#endif  // end MULTIODOMESTIMATOR_H
