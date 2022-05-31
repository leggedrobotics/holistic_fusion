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
#include "compslam_se/geometry/conversions.h"
#include "compslam_se/geometry/math_utils.h"
#include "compslam_se/measurements/DeltaMeasurement6D.h"
#include "compslam_se/measurements/UnaryMeasurement6D.h"

// Workspace
#include "kindr/Core"

// Defined macros
#define ROS_QUEUE_SIZE 100
#define REQUIRED_GNSS_NUM_NOT_JUMPED 20
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
  bool setup(ros::NodeHandle& node, GraphConfig* graphConfigPtr, StaticTransforms* staticTransformsPtr);

  // Required Initialization
  bool initYawAndPosition(const double yaw, const Eigen::Vector3d& position);
  bool initYawAndPosition(Eigen::Matrix4d T_O_I);
  bool areYawAndPositionInited();

  // Graph Manipulation
  void activateFallbackGraph();

  // Adderfunctions
  bool addImuMeasurement(const Eigen::Vector3d& linearAcc, const Eigen::Vector3d& angularVel, const ros::Time& imuTimeK,
                         std::shared_ptr<InterfacePrediction>& predictionPtr);
  void addOdometryMeasurement(const DeltaMeasurement6D& delta);
  void addUnaryPoseMeasurement(const UnaryMeasurement6D& unary);
  void addOdometryMeasurement(const UnaryMeasurement6D& odometryKm1, const UnaryMeasurement6D& odometryK,
                              const Eigen::Matrix<double, 6, 1>& poseBetweenNoise);
  void addGnssPositionMeasurement(const Eigen::Vector3d& position, const Eigen::Vector3d& lastPosition,
                                  const Eigen::Vector3d& covarianceXYZ, const ros::Time& gnssTimeK, const double rate,
                                  double positionUnaryNoise, double covarianceXYZ_violation_threshold);
  void addGnssHeadingMeasurement(const double yaw, const ros::Time& gnssTimeK, const double rate, double headingUnaryNoise);
  void addWheelOdometryMeasurement(const ros::Time& woTimeK, const double rate, const std::vector<double>& woSpeedNoise, 
                                   const gtsam::Vector3& linearVel, const gtsam::Vector3& angularVel);

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
  gtsam::Vector3 transformLeftGnssPointToImuFrame_(const gtsam::Point3& t_W_GnssL, const tf::Quaternion& tf_q_W_I);
  //// Get the robot heading from the two GNSS positions
  static gtsam::Point3 getRobotHeading_(const Eigen::Vector3d& leftPosition, const Eigen::Vector3d& rightPosition);

  // Threads
  std::thread optimizeGraphThread_;  /// Thread 5: Update of the graph as soon as new lidar measurement has arrived

  // Mutex
  std::mutex initYawAndPositionMutex_;
  std::mutex optimizeGraphMutex_;

  // Factor graph
  GraphManager* graphMgrPtr_ = NULL;

  // Graph Config
  GraphConfig* graphConfigPtr_ = NULL;
  StaticTransforms* staticTransformsPtr_ = NULL;

  /// Flags
  //// Configuration
  bool usingFallbackGraphFlag_ = true;

  //// Initialization
  bool alignedImuFlag_ = false;
  bool foundInitialYawAndPositionFlag_ = false;
  bool initedGraphFlag_ = false;
  bool receivedOdometryFlag_ = false;
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
  double globAttitude_W_I0_;
  Eigen::Vector3d globPosition_W_I0_;
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
  long gnssCallbackCounter_ = 0;
  int gnssNotJumpingCounter_ = 0;

  /// Logging
  bool logPlots_ = false;
};

}  // end namespace compslam_se

#endif  // FG_FILTERING_H
