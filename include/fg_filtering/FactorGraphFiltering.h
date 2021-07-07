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

  // Setup ------------------------
  void setVerboseLevel(int verbose) { _verboseLevel = verbose; }
  /// Setup function
  bool setup(ros::NodeHandle& node, ros::NodeHandle& privateNode);

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
  void alignImu(const double imuTime_k);
  /// Initialize GNSS pose
  void initGNSS(const sensor_msgs::NavSatFix::ConstPtr& leftGnssPtr, const sensor_msgs::NavSatFix::ConstPtr& rightGnssPtr);
  /// Initialize the graph
  void initGraph(const ros::Time& timeStamp_k);
  /// Updating the factor graph
  void updateGraph();
  /// Publish state in imu callback
  void publishState(const gtsam::NavState& currentState, ros::Time imuTime_k);
  // Commodity
  //  inline void print_map(IMUMap m) {
  //    for (auto const& pair : m) {
  //      std::cout << "{" << pair.first << ": " << pair.second << "}\n";
  //    }
  //  }

  // Threads
  /// Thread 1: Callback for compslam odometry
  /// thread 2: Callback for IMU data
  /// Thread 3: Callback for GNSS
  std::thread _publishOdometryAndTFThread;  // Thread 4: publishes the estimate at exactly 100 Hz
  std::thread _updateGraphThread;           // Thread 5: writes the IMU constraints to the graph

  // Mutex
  std::mutex _optimizeGraphMutex;

  // Member variables -------------
  /// Geodetic Converter
  geodetic_converter::GeodeticConverter _geodeticConverterLeft;
  geodetic_converter::GeodeticConverter _geodeticConverterRight;

  /// Factor graph
  GraphManager _graphMgr;

  /// Flags
  bool _imuAligned = false;
  bool _graphInited = false;
  bool _firstLidarOdomCallback = true;
  bool _firstScanCallback = true;
  bool _firstGnssCallback = true;
  bool _optimizeGraph = false;

  /// Times
  ros::Time _compslamTime_k;
  ros::Time _imuTime_km1;

  /// Transformations
  //// Compslam
  tf::StampedTransform _tf_T_OI_Compslam_km1;
  tf::StampedTransform _tf_T_OC_Compslam;
  //// Transformed output of factor graph
  tf::StampedTransform _tf_T_OC;  // odometry transformation
  //// Transform of interest for state estimation
  tf::StampedTransform _tf_T_OB;
  //// Inverse initial compslam pose
  tf::Transform _tf_T_OI_init_inv;
  /// Attitude Parameters
  gtsam::Rot3 _zeroYawIMUattitude;
  double _gravityConstant;
  tf::Transform _tf_initialImuPose;

  /// ROS related
  ros::Time _timeImuTrans;  // time of current IMU transformation information
  ros::Time _timeUpdate;

  /// Static transforms
  StaticTransforms* staticTransformsPtr_;

  /// Publishers
  ros::Publisher _pubOdometry;
  ros::Publisher _pubLaserImuBias;
  ros::Publisher _pubOdomPath;
  ros::Publisher _pubCompslamPath;
  ros::Publisher _pubLeftGnssPath;
  ros::Publisher _pubRightGnssPath;
  tf::TransformBroadcaster _tfBroadcaster;
  ros::Publisher _excavatorStatePublisher;

  /// State to be published
  excavator_model::ExcavatorState _estExcavatorState;

  /// Messages
  nav_msgs::PathPtr _odomPathPtr;
  nav_msgs::PathPtr _compslamPathPtr;
  nav_msgs::PathPtr _leftGnssPathPtr;
  nav_msgs::PathPtr _rightGnssPathPtr;
  //// Exact sync for gnss
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::NavSatFix, sensor_msgs::NavSatFix> _gnssExactSyncPolicy;
  boost::shared_ptr<message_filters::Synchronizer<_gnssExactSyncPolicy>> _gnssExactSyncPtr;  // ROS Exact Sync Policy Message Filter

  /// Subscribers
  ros::Subscriber _subImu;
  ros::Subscriber _subLidarOdometry;
  tf::TransformListener _tfListener;
  message_filters::Subscriber<sensor_msgs::NavSatFix> _subGnssLeft;
  message_filters::Subscriber<sensor_msgs::NavSatFix> _subGnssRight;
  ros::Subscriber _subMeasurements;

  /// Stored Messages
  m545_description::M545Measurements _measurements;

  // Message Converter
  m545_description_ros::ConversionTraits<m545_description::M545Measurements, m545_msgs::M545Measurements> _measurementConverter;
  excavator_model::ConversionTraits<excavator_model::ExcavatorState, m545_msgs::M545State> _stateConverter;

  // Signal Logger
  SignalLogger _signalLogger;

  /// Counter
  long lidarCallbackCounter_ = 0;  // number of processed lidar frames

  /// Verbose
  int _verboseLevel = 0;
};

}  // end namespace fg_filtering

#endif  // FG_FILTERING_H
