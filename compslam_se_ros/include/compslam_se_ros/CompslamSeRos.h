#ifndef CompslamSeRos_H
#define CompslamSeRos_H

// ROS
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

// Workspace
#include "compslam_se/CompslamSeInterface.h"

namespace compslam_se {

class CompslamSeRos : public CompslamSeInterface {
 public:
  CompslamSeRos() {}

 protected:
  // Functions that need implementation
  virtual void initializePublishers_(std::shared_ptr<ros::NodeHandle>& privateNodePtr) = 0;
  virtual void initializeSubscribers_(std::shared_ptr<ros::NodeHandle>& privateNodePtr) = 0;

  // Commodity Functions to be shared
  static void addToPathMsg(nav_msgs::PathPtr pathPtr, const std::string& frameName, const ros::Time& stamp, const Eigen::Vector3d& t,
                           const int maxBufferLength);
  static void addToOdometryMsg(nav_msgs::OdometryPtr msgPtr, const std::string& fixedFrame, const std::string& movingFrame,
                               const ros::Time& stamp, const Eigen::Matrix4d& T, const Eigen::Vector3d& W_v_W_F,
                               const Eigen::Vector3d& W_w_W_F);
};
}  // namespace compslam_se
#endif  // end M545ESTIMATORGRAPH_H
