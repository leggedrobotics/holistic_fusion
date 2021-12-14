#ifndef FG_FILTERING_H
#define FG_FILTERING_H

// C++
#include <mutex>
#include <string_view>
#include <thread>

// ROS
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
// ROS msgs
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

// Workspace
#include "fg_filtering/GraphManager.hpp"
#include "fg_filtering/SignalLogger.h"
#include "fg_filtering/StaticTransforms.h"
#include "fg_filtering/geometry/math_utils.h"
#include "kindr/Core"
#include "robot_utils/sensors/GNSS.hpp"

// Menzi
#include "fg_filtering_log_msgs/ImuMultiplot.h"
#include "fg_filtering_log_msgs/LidarMultiplot.h"

namespace compslam_se {

// Defined macros
#define ROS_QUEUE_SIZE 100
#define NUM_LIDAR_CALLBACKS_UNTIL_START 5
#define NUM_GNSS_CALLBACKS_UNTIL_YAW_INIT 20
#define REQUIRED_GNSS_NUM_NOT_JUMPED 20          // 0
#define GNSS_COVARIANCE_VIOLATION_THRESHOLD 0.1  // 10000
#define GREEN_START "\033[92m"
#define YELLOW_START "\033[33m"
#define COLOR_END "\033[0m"

/** \brief Implementation of the factor graph based filtering.
 *
 */
class FactorGraphFiltering {
 public:
  // Constructor
  FactorGraphFiltering();
  // Destructor --> log signals
  ~FactorGraphFiltering() { signalLogger_.~SignalLogger(); };

  // Setup
  bool setup(ros::NodeHandle& node, ros::NodeHandle& privateNode);

  /// Setters
  void setVerboseLevel(int verbose) { verboseLevel_ = verbose; }
  void setImuGravityDirection(std::string sParam) { imuGravityDirection_ = sParam; }

  // Getters
  bool getLogPlots() { return logPlots_; }

  // Log data
  void logSignals() { signalLogger_.~SignalLogger(); }

  // Access functions
  void addImuMeasurement(const Eigen::Vector3d& linearAcc, const Eigen::Vector3d& angularVel, const ros::Time& imuTimeK);
  void addOdometryMeasurement(const tf::Transform& tf_compslam_T_O_Lk, const ros::Time& odometryTimeK);
  void addGnssMeasurements(const gtsam::Point3& leftGnssCoord, const gtsam::Point3& rightGnssCoord, const Eigen::Vector3d& covarianceXYZ,
                           const ros::Time& gnssTimeK);

 protected:
  // Methods -------------
  /// Worker functions
  //// Set Imu Attitude
  bool alignImu_(const ros::Time& imuTimeK);
  //// Initialize GNSS pose
  void initGnss_(const gtsam::Point3& leftGnssCoordinates, const gtsam::Point3& rightGnssCoordinates);
  //// Initialize the graph
  void initGraph_(const ros::Time& timeStamp_k);
  //// Updating the factor graph
  void optimizeGraph_();
  //// Publish state in imu callback
  virtual void publishState_(ros::Time imuTimeK, const gtsam::Pose3& T_W_O, const gtsam::NavState& currentState,
                             const Eigen::Vector3d& I_w) = 0;

  /// Utility functions
  //// Convert GNSS readings to vectors
  // void convertNavSatToPositions(const sensor_msgs::NavSatFix::ConstPtr& leftGnssMsgPtr,
  //                              const sensor_msgs::NavSatFix::ConstPtr& rightGnssMsgPtr, gtsam::Point3& leftPosition,
  //                              gtsam::Point3& rightPosition);
  void convertNavSatToPositions(const gtsam::Point3& leftGnssCoordinate, const gtsam::Point3& rightGnssCoordinate,
                                gtsam::Point3& leftPosition, gtsam::Point3& rightPosition);
  //// Geometric transformation to IMU in world frame
  gtsam::Vector3 transformGnssPointToImuFrame_(const gtsam::Point3& gnssPosition, const tf::Quaternion& tf_q_W_I);
  //// Get the robot heading from the two GNSS positions
  static gtsam::Point3 getRobotHeading_(const Eigen::Vector3d& leftPosition, const Eigen::Vector3d& rightPosition);
  //// Compute yaw from the heading vector
  static double computeYawFromHeadingVector_(const gtsam::Point3& headingVector);
  //// Toggle GNSS Service Method
  bool toggleGnssFlag_(std_srvs::Empty::Request& /*request*/, std_srvs::Empty::Response& /*response*/);

  // Commodity functions
  void readParams_(const ros::NodeHandle& privateNode);

  // Threads
  /// Thread 1: Callback for compslam odometry
  /// Thread 2: Callback for IMU data, also publishes the state at exactly 100 Hz
  /// Thread 3: Dual callback for both GNSS topics
  /// Thread 4: Measurement callback
  std::thread optimizeGraphThread_;  /// Thread 5: Update of the graph as soon as new lidar measurement has arrived

  // Mutex
  std::mutex initGraphMutex_;
  std::mutex optimizeGraphMutex_;

  // Member variables -------------
  /// Geodetic Converter
  // geodetic_converter::GeodeticConverter geodeticConverterLeft_;
  robot_utils::GNSS gnssSensor_;

  /// Factor graph
  GraphManager graphMgr_;

  /// Flags
  //// Configuration
  bool usingGnssFlag_ = true;
  bool usingFallbackGraphFlag_ = true;
  bool usingCompslamFlag_ = true;
  bool usingGnssReferenceFlag_ = true;
  //// Initialization
  bool foundInitialYawFlag_ = false;
  bool initedGraphFlag_ = false;
  //// During operation
  bool optimizeGraphFlag_ = false;
  bool gnssCovarianceViolatedFlag_ = false;

  /// Thresholds
  double gnssOutlierThreshold_ = 1.0;
  // double graphUpdateThreshold_ = 0.2;

  /// Strings
  std::string imuGravityDirection_;

  /// Times
  ros::Time compslamTimeK_;
  ros::Time imuTimeKm1_;
  ros::Time imuTimeK_;
  double imuTimeOffset_ = 0.0;

  /// Rates
  int imuRate_ = 100;

  /// Containers
  //// Transformations
  ///// Compslam
  tf::Transform tf_compslam_T_I0_O_;
  tf::StampedTransform tf_T_W_Ik_;
  //// Attitude Parameters
  tf::Transform tf_T_W_I0_;        // Initial IMU pose (in graph)
  gtsam::Point3 W_t_W_GnssL0_;     // initial global position of left GNSS
  double gravityConstant_ = 9.81;  // Will be overwritten
  double globalAttitudeYaw_W_C0_;
  double imuAttitudePitch_;
  double imuAttitudeRoll_;

  /// Reference position
  double gnssReferenceLatitude_;
  double gnssReferenceLongitude_;
  double gnssReferenceAltitude_;
  double gnssReferenceHeading_;

  /// Static transforms
  StaticTransforms* staticTransformsPtr_ = NULL;

  /// Publishers
  ros::Publisher pubLaserImuBias_;
  ros::Publisher pubOptimizationPath_;
  ros::Publisher pubCompslamPath_;
  ros::Publisher pubLeftGnssPath_;
  ros::Publisher pubRightGnssPath_;
  ros::Publisher imuMultiplotPublisher_;
  ros::Publisher lidarMultiplotPublisher_;

  /// Services
  ros::ServiceServer toggleGnssUsageService_;

  /// Messages
  nav_msgs::PathPtr odomPathPtr_;
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

  /// Verbose
  int verboseLevel_ = 0;

  /// Logging
  bool logPlots_ = false;
};

}  // end namespace compslam_se

#endif  // FG_FILTERING_H
