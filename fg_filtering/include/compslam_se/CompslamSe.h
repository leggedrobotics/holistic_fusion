#ifndef FG_FILTERING_H
#define FG_FILTERING_H

// C++
#include <mutex>
#include <stdexcept>
#include <string_view>
#include <thread>

// ROS
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/SetBool.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

// Package
#include "compslam_se/GraphManager.hpp"
#include "compslam_se/InterfacePrediction.h"
#include "compslam_se/SignalLogger.h"
#include "compslam_se/StaticTransforms.h"
#include "compslam_se/config/GraphConfig.h"
#include "compslam_se/geometry/eigen_conversions.h"
#include "compslam_se/geometry/math_utils.h"
#include "fg_filtering_log_msgs/ImuMultiplot.h"
#include "fg_filtering_log_msgs/LidarMultiplot.h"

// Workspace
#include "kindr/Core"

// Defined macros
#define ROS_QUEUE_SIZE 100
#define REQUIRED_GNSS_NUM_NOT_JUMPED 20
#define NUM_LIDAR_CALLBACKS_UNTIL_START 5
#define GNSS_COVARIANCE_VIOLATION_THRESHOLD 0.1  // 10000
#define GREEN_START "\033[92m"
#define YELLOW_START "\033[33m"
#define COLOR_END "\033[0m"

namespace compslam_se {

/** \brief Implementation of the factor graph based filtering.
 *
 */
class CompslamSe {
 public:
  // Constructor
  CompslamSe();
  // Destructor --> log signals
  ~CompslamSe() { signalLogger_.~SignalLogger(); };

  // Setup
  bool setup(ros::NodeHandle& node, ros::NodeHandle& privateNode, GraphConfig* graphConfigPtr, StaticTransforms* staticTransformsPtr);

  // Required Initialization
  bool initYawAndPosition(const double yaw, const Eigen::Vector3d& position);

  // Adderfunctions
  bool addImuMeasurement(const Eigen::Vector3d& linearAcc, const Eigen::Vector3d& angularVel, const ros::Time& imuTimeK,
                         InterfacePrediction*& predictionPtr);
  void addOdometryMeasurement(const Eigen::Matrix4d& T_O_Lk, const double rate, const std::vector<double>& poseBetweenNoise,
                              const ros::Time& odometryTimeK);
  void addGnssPositionMeasurement(const Eigen::Vector3d& position, const Eigen::Vector3d& lastPosition,
                                  const Eigen::Vector3d& covarianceXYZ, const ros::Time& gnssTimeK, const double rate,
                                  double positionUnaryNoise);

  // Getters
  bool getLogPlots() { return logPlots_; }

  // Log data
  void logSignals() { signalLogger_.~SignalLogger(); }

 protected:
  // Methods -------------
  /// Worker functions
  //// Set Imu Attitude
  bool alignImu_(const ros::Time& imuTimeK);
  //// Initialize the graph
  void initGraph_(const ros::Time& timeStamp_k);
  //// Updating the factor graph
  void optimizeGraph_();

  /// Utility functions
  //// Geometric transformation to IMU in world frame
  gtsam::Vector3 transformGnssPointToImuFrame_(const gtsam::Point3& gnssPosition, const tf::Quaternion& tf_q_W_I);
  //// Get the robot heading from the two GNSS positions
  static gtsam::Point3 getRobotHeading_(const Eigen::Vector3d& leftPosition, const Eigen::Vector3d& rightPosition);

  // Threads
  std::thread optimizeGraphThread_;  /// Thread 5: Update of the graph as soon as new lidar measurement has arrived

  // Mutex
  std::mutex initYawMutex_;
  std::mutex optimizeGraphMutex_;

  // Factor graph
  GraphManager* graphMgrPtr_ = NULL;
  StaticTransforms* staticTransformsPtr_ = NULL;

  // Graph Config
  GraphConfig* graphConfigPtr_ = NULL;

  /// Flags
  //// Configuration
  bool usingFallbackGraphFlag_ = true;

  //// Initialization
  bool alignedImuFlag_ = false;
  bool foundInitialYawAndPositionFlag_ = false;
  bool initedGraphFlag_ = false;
  //// During operation
  bool optimizeGraphFlag_ = false;
  bool gnssCovarianceViolatedFlag_ = false;

  /// Times
  ros::Time compslamTimeK_;
  ros::Time imuTimeKm1_;
  ros::Time imuTimeK_;
  double imuTimeOffset_ = 0.0;

  /// Transformations
  tf::Transform tf_compslam_T_I0_O_;
  tf::StampedTransform tf_T_W_Ik_;
  tf::Transform tf_T_W_I0_;  // Initial IMU pose (in graph)
  /// Attitudes
  double gravityConstant_ = 9.81;  // Will be overwritten
  double globalAttitudeYaw_W_C0_;
  Eigen::Vector3d globalPosition_W_I0_;
  double imuAttitudePitch_;
  double imuAttitudeRoll_;

  /// Publishers
  ros::Publisher pubLaserImuBias_;
  ros::Publisher pubOptimizationPath_;
  ros::Publisher pubCompslamPath_;
  ros::Publisher pubLeftGnssPath_;
  ros::Publisher pubRightGnssPath_;
  ros::Publisher imuMultiplotPublisher_;
  ros::Publisher lidarMultiplotPublisher_;

  /// Messages
  nav_msgs::PathPtr optimizationPathPtr_;
  nav_msgs::PathPtr compslamPathPtr_;
  nav_msgs::PathPtr leftGnssPathPtr_;
  nav_msgs::PathPtr rightGnssPathPtr_;

  // Signal Logger
  SignalLogger signalLogger_;
  SignalLoggerGnss signalLoggerGnss_;

  /// Counter
  long lidarCallbackCounter_ = 0;  // number of processed lidar frames
  long gnssCallbackCounter_ = 0;

  /// Logging
  bool logPlots_ = false;
};

}  // end namespace compslam_se

#endif  // FG_FILTERING_H
