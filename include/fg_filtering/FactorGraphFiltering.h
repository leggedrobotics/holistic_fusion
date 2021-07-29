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
#include "geodetic_utils/geodetic_conv.hpp"
#include "kindr/Core"

// Menzi
#include "excavator_model/ActuatorConversions.hpp"
#include "excavator_model/ConversionTraits.hpp"
#include "excavator_model/ExcavatorState.hpp"
#include "m545_description/M545Measurements.hpp"
#include "m545_description_ros/ConversionTraits.hpp"

namespace fg_filtering {

#define NUM_LIDAR_CALLBACKS_UNTIL_PUBLISHING 5

/** \brief Implementation of the factor graph based filtering.
 *
 */
class FactorGraphFiltering {
 public:
  // Constructor
  explicit FactorGraphFiltering(float scanPeriod = 0.1);
  // Destructor --> log signals
  ~FactorGraphFiltering() { signalLogger_.~SignalLogger(); };

  // Setup ------------------------
  void setVerboseLevel(int verbose) { verboseLevel_ = verbose; }
  void setImuGravityDirection(std::string sParam) { imuGravityDirection_ = sParam; }
  /// Setup function
  bool setup(ros::NodeHandle& node, ros::NodeHandle& privateNode);

  // Log data
  void logSignals() { signalLogger_.~SignalLogger(); }

 private:
  // Functions -------------

  // Callbacks
  /// IMU Callback Function for handling incoming IMU messages -------------
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_ptr);
  /// LiDAR Odometry Callback
  void lidarOdometryCallback(const nav_msgs::Odometry::ConstPtr& lidar_odom_ptr);
  /// GNSS Callback
  void gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& leftGnssPtr, const sensor_msgs::NavSatFix::ConstPtr& rightGnssPtr);
  /// Measurement Callback
  void measurementsCallback(const m545_msgs::M545Measurements::ConstPtr& measurementsMsg);

  // Worker functions
  /// Set Imu Attitude
  void alignImu(const double imuTimeK);
  /// Initialize GNSS pose
  void initGNSS(const sensor_msgs::NavSatFix::ConstPtr& leftGnssPtr, const sensor_msgs::NavSatFix::ConstPtr& rightGnssPtr);
  /// Initialize the graph
  void initGraph(const ros::Time& timeStamp_k);
  /// Updating the factor graph
  void updateGraph();
  /// Publish state in imu callback
  void publishState(const gtsam::NavState& currentState, ros::Time imuTimeK);

  // Commodity

  // Threads
  /// Thread 1: Callback for compslam odometry
  /// Thread 2: Callback for IMU data, also publishes the state at exactly 100 Hz
  /// Thread 3: Dual callback for both GNSS topics
  /// Thread 4: Measurement callback
  std::thread optimizeGraphThread_;  /// Thread 5: Update of the graph as soon as new lidar measurement has arrived

  // Mutex
  std::mutex optimizeGraphMutex_;

  // Member variables -------------
  /// Geodetic Converter
  geodetic_converter::GeodeticConverter geodeticConverterLeft_;
  geodetic_converter::GeodeticConverter geodeticConverterRight_;

  /// Factor graph
  GraphManager graphMgr_;

  /// Flags
  bool imuAligned_ = false;
  bool graphInited_ = false;
  bool firstLidarOdomCallback_ = true;
  bool firstGnssCallback_ = true;
  bool optimizeGraph_ = false;
  std::string imuGravityDirection_;

  /// Times
  ros::Time compslamTimeK_;
  ros::Time imuTimeKm1_;

  /// Transformations
  //// Compslam
  tf::StampedTransform tf_T_OI_Compslam_km1_;
  tf::StampedTransform tf_T_OC_Compslam_;
  //// Transformed output of factor graph
  tf::StampedTransform tf_T_OC_;  // odometry transformation
  //// Transform of interest for state estimation
  tf::StampedTransform tf_T_OB_;
  //// Inverse initial compslam pose
  tf::Transform tf_T_OI_init_inv_;
  /// Attitude Parameters
  gtsam::Rot3 zeroYawImuAttitude_;
  double gravityConstant_;
  tf::Transform tf_initialImuPose_;

  /// ROS related
  ros::Time timeImuTrans_;  // time of current IMU transformation information
  ros::Time timeUpdate_;

  /// Static transforms
  StaticTransforms* staticTransformsPtr_;

  /// Publishers
  ros::Publisher pubOdometry_;
  ros::Publisher pubLaserImuBias_;
  ros::Publisher pubOdomPath_;
  ros::Publisher pubOdomLidarPath_;
  ros::Publisher pubCompslamPath_;
  ros::Publisher pubLeftGnssPath_;
  ros::Publisher pubRightGnssPath_;
  tf::TransformBroadcaster tfBroadcaster_;
  ros::Publisher excavatorStatePublisher_;

  /// State to be published
  excavator_model::ExcavatorState estExcavatorState_;

  /// Messages
  nav_msgs::PathPtr odomPathPtr_;
  nav_msgs::PathPtr odomLidarPathPtr_;
  nav_msgs::PathPtr compslamPathPtr_;
  nav_msgs::PathPtr leftGnssPathPtr_;
  nav_msgs::PathPtr rightGnssPathPtr_;
  //// Exact sync for gnss
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::NavSatFix, sensor_msgs::NavSatFix> _gnssExactSyncPolicy;
  boost::shared_ptr<message_filters::Synchronizer<_gnssExactSyncPolicy>> gnssExactSyncPtr_;  // ROS Exact Sync Policy Message Filter

  /// Subscribers
  ros::Subscriber subImu_;
  ros::Subscriber subLidarOdometry_;
  tf::TransformListener tfListener_;
  message_filters::Subscriber<sensor_msgs::NavSatFix> subGnssLeft_;
  message_filters::Subscriber<sensor_msgs::NavSatFix> subGnssRight_;
  ros::Subscriber subMeasurements_;

  /// Stored Messages
  m545_description::M545Measurements measurements_;

  // Message Converter
  m545_description_ros::ConversionTraits<m545_description::M545Measurements, m545_msgs::M545Measurements> measurementConverter_;
  excavator_model::ConversionTraits<excavator_model::ExcavatorState, m545_msgs::M545State> stateConverter_;

  // Signal Logger
  SignalLogger signalLogger_;

  /// Counter
  long lidarCallbackCounter_ = 0;  // number of processed lidar frames

  /// Verbose
  int verboseLevel_ = 0;
};

}  // end namespace fg_filtering

#endif  // FG_FILTERING_H
