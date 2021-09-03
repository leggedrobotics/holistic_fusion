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
#include <sensor_msgs/NavSatFix.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
// ROS msgs
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include "m545_msgs/M545ActuatorStates.h"
#include "m545_msgs/M545Measurements.h"
#include "m545_msgs/M545State.h"

// Workspace
#include "fg_filtering/GraphManager.hpp"
#include "fg_filtering/SignalLogger.h"
#include "fg_filtering/StaticTransforms.h"
#include "fg_filtering/geometry/math_utils.h"
//#include "geodetic_utils/geodetic_conv.hpp"
#include "kindr/Core"
#include "robot_utils/sensors/GNSS.hpp"

// Menzi
#include "excavator_model/ActuatorConversions.hpp"
#include "excavator_model/ConversionTraits.hpp"
#include "excavator_model/ExcavatorState.hpp"
#include "m545_description/M545Measurements.hpp"
#include "m545_description_ros/ConversionTraits.hpp"

namespace fg_filtering {

// Defined constants
#define ROS_QUEUE_SIZE 100
#define NUM_LIDAR_CALLBACKS_UNTIL_START 3
#define NUM_GNSS_CALLBACKS_UNTIL_START 3
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

 private:
  // Methods -------------

  /// Callbacks
  //// IMU Callback Functions for handling incoming IMU messages -------------
  void imuCabinCallback_(const sensor_msgs::Imu::ConstPtr& imuPtr);
  void imuBaseCallback_(const sensor_msgs::Imu::ConstPtr& imuPtr);
  //// LiDAR Odometry Callback
  void lidarOdometryCallback_(const nav_msgs::Odometry::ConstPtr& lidar_odom_ptr);
  //// GNSS Callback
  void gnssCallback_(const sensor_msgs::NavSatFix::ConstPtr& leftGnssPtr, const sensor_msgs::NavSatFix::ConstPtr& rightGnssPtr);
  //// Measurement Callback
  void measurementsCallback_(const m545_msgs::M545Measurements::ConstPtr& measurementsMsg);

  /// Worker functions
  //// Set Imu Attitude
  void alignImu_(const ros::Time& imuTimeK);
  //// Initialize GNSS pose
  void initGnss_(const sensor_msgs::NavSatFix::ConstPtr& leftGnssPtr, const sensor_msgs::NavSatFix::ConstPtr& rightGnssPtr);
  //// Initialize the graph
  void initGraph_(const ros::Time& timeStamp_k);
  //// Updating the factor graph
  void optimizeGraph_();
  //// Publish state in imu callback
  void publishState_(ros::Time imuTimeK, const gtsam::NavState& currentState, const Eigen::Vector3d& I_w);

  /// Utility functions
  //// Convert GNSS readings to vectors
  void convertNavSatToPositions(const sensor_msgs::NavSatFix::ConstPtr& leftGnssMsgPtr,
                                const sensor_msgs::NavSatFix::ConstPtr& rightGnssMsgPtr, gtsam::Point3& leftPosition,
                                gtsam::Point3& rightPosition);
  //// Geometric transformation to IMU in world frame
  gtsam::Vector3 transformGnssPointToImuFrame_(const gtsam::Point3& gnssPosition);
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
  std::mutex optimizeGraphMutex_;
  std::mutex accessImuBaseMutex_;
  std::mutex lidarUnaryMutex_;

  // Member variables -------------
  /// Geodetic Converter
  // geodetic_converter::GeodeticConverter geodeticConverterLeft_;
  robot_utils::GNSS gnssSensor_;

  /// Factor graph
  GraphManager graphMgr_;

  /// Flags
  bool imuAlignedFlag_ = false;
  bool initedGnssFlag_ = false;
  bool initedGraphFlag_ = false;
  bool optimizeGraphFlag_ = false;
  bool usingGnssFlag_ = true;
  bool usingCompslamFlag_ = true;
  bool addLidarUnaryFlag_ = false;
  bool usingLidarUnaryFlag_ = true;
  bool usingGnssReferenceFlag_ = true;

  /// Strings
  std::string imuGravityDirection_;

  /// Times
  ros::Time compslamTimeK_;
  ros::Time imuTimeKm1_;

  /// Containers
  //// Transformations
  ///// Compslam
  tf::Transform tf_compslam_T_I0_O_;
  ///// Current global transformation
  tf::StampedTransform tf_T_O_Ik_;
  //// Attitude Parameters
  gtsam::Rot3 initialImuAttitude_;
  gtsam::Pose3 initialImuPose_;
  double initialGlobalYaw_ = 0.0;
  gtsam::Point3 W_t_W_GnssL0_;     // initial global position of left GNSS
  double gravityConstant_ = 9.81;  // Will be overwritten
  tf::Transform tf_T_O_I0_;        // Initial IMU pose (in graph)
  //// Base IMU measurments
  Eigen::Vector3d latestImuBaseLinearAcc_;
  Eigen::Vector3d latestImuBaseAngularVel_;

  /// Reference position
  double gnssReferenceLatitude_;
  double gnssReferenceLongitude_;
  double gnssReferenceAltitude_;
  double gnssReferenceHeading_;

  /// Static transforms
  StaticTransforms* staticTransformsPtr_;

  /// Publishers
  ros::Publisher pubOdometry_;
  ros::Publisher pubLaserImuBias_;
  ros::Publisher pubOdomPath_;
  ros::Publisher pubOptimizationPath_;
  ros::Publisher pubCompslamPath_;
  ros::Publisher pubLeftGnssPath_;
  ros::Publisher pubRightGnssPath_;
  ros::Publisher excavatorStatePublisher_;

  /// Services
  ros::ServiceServer toggleGnssUsageService_;

  /// State to be published
  excavator_model::ExcavatorState estExcavatorState_;

  /// Messages
  nav_msgs::PathPtr odomPathPtr_;
  nav_msgs::PathPtr optimizationPathPtr_;
  nav_msgs::PathPtr compslamPathPtr_;
  nav_msgs::PathPtr leftGnssPathPtr_;
  nav_msgs::PathPtr rightGnssPathPtr_;

  /// Subscribers
  ros::Subscriber subImuCabin_;
  ros::Subscriber subImuBase_;
  ros::Subscriber subLidarOdometry_;
  tf::TransformListener tfListener_;
  message_filters::Subscriber<sensor_msgs::NavSatFix> subGnssLeft_;
  message_filters::Subscriber<sensor_msgs::NavSatFix> subGnssRight_;
  ros::Subscriber subMeasurements_;

  //// Exact sync for gnss
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::NavSatFix, sensor_msgs::NavSatFix> _gnssExactSyncPolicy;
  boost::shared_ptr<message_filters::Synchronizer<_gnssExactSyncPolicy>> gnssExactSyncPtr_;  // ROS Exact Sync Policy Message Filter

  /// Stored Messages
  m545_description::M545Measurements measurements_;

  // Message Converter
  m545_description_ros::ConversionTraits<m545_description::M545Measurements, m545_msgs::M545Measurements> measurementConverter_;
  excavator_model::ConversionTraits<excavator_model::ExcavatorState, m545_msgs::M545State> stateConverter_;

  // Signal Logger
  SignalLogger signalLogger_;

  /// Counter
  long lidarCallbackCounter_ = 0;  // number of processed lidar frames
  long gnssCallbackCounter_ = 0;

  /// Verbose
  int verboseLevel_ = 0;

  /// Logging
  bool logPlots_ = false;
};

}  // end namespace fg_filtering

#endif  // FG_FILTERING_H
