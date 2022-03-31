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
#include "m545_msgs/M545ActuatorStates.h"
#include "m545_msgs/M545Measurements.h"
#include "m545_msgs/M545State.h"

// Workspace
#include "compslam_ros/GnssHandler.h"
#include "compslam_ros/StaticTransforms.h"
#include "compslam_se/CompslamSeInterface.h"
#include "compslam_se/geometry/conversions.h"
#include "compslam_se/measurements/UnaryMeasurement6D.h"

// Defined Macros
#define ROS_QUEUE_SIZE 100
#define NUM_GNSS_CALLBACKS_UNTIL_START 20  // 0

namespace compslam_ros {

class CompslamEstimator : public compslam_se::CompslamSeInterface {
 public:
  CompslamEstimator(ros::NodeHandle& node, ros::NodeHandle& privateNode);

 private:
  // Callbacks
  void measurementsCallback_(const m545_msgs::M545Measurements::ConstPtr& measurementsMsgPtr);
  void imuCabinCallback_(const sensor_msgs::Imu::ConstPtr& imuPtr);
  void imuBaseCallback_(const sensor_msgs::Imu::ConstPtr& imuPtr);
  void lidarOdometryCallback_(const nav_msgs::Odometry::ConstPtr& lidar_odom_ptr);
  void gnssCallback_(const sensor_msgs::NavSatFix::ConstPtr& leftGnssPtr, const sensor_msgs::NavSatFix::ConstPtr& rightGnssPtr);

  /// Services
  ros::ServiceServer toggleGnssUsageService_;
  bool toggleGnssFlag_(std_srvs::Empty::Request& /*request*/, std_srvs::Empty::Response& /*response*/);

  // Publish State
  void publishState_(ros::Time imuTimeK, const Eigen::Matrix4d& T_W_O, const Eigen::Matrix4d& T_O_Ik, const Eigen::Vector3d& Ic_v_W_Ic,
                     const Eigen::Vector3d& I_w_W_I) override;

  // Commodity functions
  void readParams_(const ros::NodeHandle& privateNode);

  // Time
  std::chrono::time_point<std::chrono::high_resolution_clock> startTime_;
  std::chrono::time_point<std::chrono::high_resolution_clock> currentTime_;
  long secondsSinceStart_();

  // Mutex
  std::mutex accessImuBaseMutex_;

  // Exact sync for gnss
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::NavSatFix, sensor_msgs::NavSatFix> gnssExactSyncPolicy_;
  boost::shared_ptr<message_filters::Synchronizer<gnssExactSyncPolicy_>> gnssExactSyncPtr_;  // ROS Exact Sync Policy Message Filter

  /// Subscribers
  ros::Subscriber subImuCabin_;
  ros::Subscriber subImuBase_;
  ros::Subscriber subLidarOdometry_;
  tf::TransformListener tfListener_;
  message_filters::Subscriber<sensor_msgs::NavSatFix> subGnssLeft_;
  message_filters::Subscriber<sensor_msgs::NavSatFix> subGnssRight_;
  ros::Subscriber subMeasurements_;

  // Publisher
  ros::Publisher excavatorStatePublisher_;
  ros::Publisher pubOdometryCabin_;
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

  /// Flags
  bool usingGnssFlag_ = true;
  bool usingCompslamFlag_ = true;

  /// GNSS
  m545_estimator::GnssHandler* gnssHandlerPtr_;
};
}  // namespace compslam_ros
#endif  // end M545ESTIMATORGRAPH_H
