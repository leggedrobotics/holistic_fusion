#ifndef M545ESTIMATORGRAPH_H
#define M545ESTIMATORGRAPH_H

// std
#include <chrono>

// ROS
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>

// Workspace
#include "compslam_se/CompslamSeInterface.h"
#include "compslam_se/measurements/UnaryMeasurement6D.h"
#include "compslam_se_ros/GnssHandler.h"
#include "compslam_se_ros/extrinsics/StaticTransformsTf.h"

// Defined Macros
#define ROS_QUEUE_SIZE 100
#define NUM_GNSS_CALLBACKS_UNTIL_START 20  // 0

namespace compslam_se {

class CompslamEstimator : public CompslamSeInterface {
 public:
  CompslamEstimator(ros::NodeHandle& node, ros::NodeHandle& privateNode);

 private:
  // Callbacks
  void imuCabinCallback_(const sensor_msgs::Imu::ConstPtr& imuPtr);
  void lidarOdometryCallback_(const nav_msgs::Odometry::ConstPtr& lidar_odom_ptr);

  // Publish State
  void publishState_(const double imuTimeK, const Eigen::Matrix4d& T_W_O, const Eigen::Matrix4d& T_O_Ik, const Eigen::Vector3d& Ic_v_W_Ic,
                     const Eigen::Vector3d& I_w_W_I) override;

  // Commodity functions
  void readParams_();

  // Time
  std::chrono::time_point<std::chrono::high_resolution_clock> startTime_;
  std::chrono::time_point<std::chrono::high_resolution_clock> currentTime_;
  long secondsSinceStart_();

  // Mutex
  std::mutex accessImuBaseMutex_;

  // Node
  ros::NodeHandle privateNode_;

  /// Subscribers
  ros::Subscriber subImu_;
  ros::Subscriber subLidarOdometry_;
  tf::TransformListener tfListener_;

  // Publisher
  ros::Publisher pubOdometryImu_;
  ros::Publisher pubOdometryLidar_;
  ros::Publisher pubWorldLidar_;
  ros::Publisher pubWorldImu_;
  ros::Publisher pubOdomPath_;

  // Messages
  nav_msgs::PathPtr odomPathPtr_;

  // Base IMU measurements
  Eigen::Vector3d latestImuBaseLinearAcc_;
  Eigen::Vector3d latestImuBaseAngularVel_;

  // Rates
  double lidarRate_ = 5.0;
  double gnssLeftRate_ = 20.0;
  double gnssRightRate_ = 20.0;
  // Thresholds
  double gnssOutlierThreshold_ = 1.0;

  // Noise
  Eigen::Matrix<double, 6, 1> poseBetweenNoise_;
  Eigen::Matrix<double, 6, 1> poseUnaryNoise_;
  double gnssPositionUnaryNoise_;
  double gnssHeadingUnaryNoise_;

  /// Gnss
  GnssHandler* gnssHandlerPtr_;
};
}  // namespace compslam_se
#endif  // end M545ESTIMATORGRAPH_H
