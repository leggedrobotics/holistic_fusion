#ifndef LOAM_LASERODOMETRY_H
#define LOAM_LASERODOMETRY_H

// CPP
#include <mutex>
#include <string_view>
#include <thread>

// ROS
#include <nav_msgs/Odometry.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
// ROS msgs
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

// loam
#include "loam/Angle.h"
#include "loam/GraphManager.hpp"
#include "loam/ImuManager.hpp"
#include "loam/Twist.h"
#include "loam/math_utils.h"

// Local
#include "LidarOdometryManager.hpp"
//#include "math_utils.h"

namespace fg_filtering {

/** \brief Implementation of the factor graph based filtering.
 *
 */
class FactorGraphFiltering {
 public:
  // Constructor
  explicit FactorGraphFiltering(float scanPeriod = 0.1);

  // Setup ------------------------
  /// Helper functions
  //// Frames
  void setMapFrame(const std::string& s) { _mapFrame = s; }
  void setOdomFrame(const std::string& s) { _odomFrame = s; }
  void setBaseLinkFrame(const std::string& s) { _baseLinkFrame = s; }
  void setImuFrame(const std::string& s) { _imuFrame = s; }
  void setLidarFrame(const std::string& s) { _lidarFrame = s; }
  //// Timing and Motions
  void setImuTimeOffset(const double d) { _imuTimeOffset = d; }
  void setZeroMotionDetection(const bool b) { _zeroMotionDetection = b; }
  void setVerboseLevel(int verbose) { _verboseLevel = verbose; }
  auto const& transformSum() { return _transformSum; }
  auto const& graphIMUBias() const { return _imuBiasMsg; }
  /// Setup function
  bool setup(ros::NodeHandle& node, ros::NodeHandle& privateNode);

  // Processing of buffered data ------------
  void process();

  // Callbacks
  /// IMU Callback Function for handling incoming IMU messages -------------
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_ptr);
  /// LiDAR Odometry Callback
  void lidarOdometryCallback(const nav_msgs::Odometry::ConstPtr& lidar_odom_ptr);

  void print_map(loam::IMUMap m) {
    for (auto const& pair : m) {
      std::cout << "{" << pair.first << ": " << pair.second << "}\n";
    }
  }

 private:
  // Functions -------------
  /// Publish the current result via the respective topics.
  void publishOdometryAndTF();
  /// Write IMU to grah
  void writeToGraph();
  /// Updating the factor graph
  bool updateFactorGraph();

  // Threads
  /// Thread 1: Callback for compslam odometry
  /// thread 2: Callback for IMU data
  /// Thread 3: Callback for GNSS
  std::thread _publishOdometryAndTFThread;  // Thread 4: publishes the estimate at exactly 100 Hz
  std::thread _writeToGraphThread;          // Thread 5: writes the IMU constraints to the graph

  // Mutex
  // std::mutex _factorGraphMutex;
  std::mutex _lidarDeltaPoseMutex;
  std::mutex _imuMeasurementMutex;

  // Member variables -------------
  /// IMU buffer
  loam::ImuManager _imuBuffer;

  /// Factor graph
  loam::GraphManager _graphMgr;

  /// Flags
  bool _systemInited = false;  // initialization flag
  bool _imuAligned = false;
  bool _graphInited = false;
  bool _firstScanCallback = true;

  /// Times
  ros::Time _lastImuTime;
  ros::Time _lastLidarTime;
  ros::Time _currentImuTime;
  ros::Time _currentLidarTime;

  /// Graph keys
  gtsam::Key _currentImuKey;
  gtsam::Key _currentLidarKey;

  /// Transformations
  tf::StampedTransform _tf_T_OI_CompslamLast;
  //// Output of factor graph
  tf::StampedTransform _tf_T_OI;  // odometry transformation
  //// Transform of interest for state estimation
  tf::StampedTransform _tf_T_OB;
  //// IMU init
  gtsam::Rot3 _zeroYawIMUattitude;

  /// Measurements
  //// IMU measurement, can then be found in the IMU buffer
  bool _newImuMeasurement = false;
  //// LiDAR Delta Pose
  gtsam::Pose3 _lidarDeltaPose;
  bool _newLidarDeltaPose = false;

  /// Twists
  loam::Twist _transform;     // optimized pose transformation //smk: also used as motion prior, also adjusted by IMU or
                              // VIO(if needed)
  loam::Twist _transformSum;  // accumulated optimized pose transformation

  /// ROS related
  ros::Time _timeImuTrans;  // time of current IMU transformation information
  ros::Time _timeUpdate;

  /// Frames
  std::string _mapFrame = "";
  std::string _odomFrame = "";
  std::string _baseLinkFrame = "";
  std::string _imuFrame = "";
  std::string _lidarFrame = "";

  /// Publishers
  ros::Publisher _pubOdometry;              // laser odometry publisher
  ros::Publisher _pubLaserImuBias;          // laser odometry imu bias publisher
  tf::TransformBroadcaster _tfBroadcaster;  // laser odometry transform broadcaster

  /// Subscribers
  ros::Subscriber _subImuTrans;       // IMU transformation information message subscriber
  ros::Subscriber _subImu;            /// IMU subscriber
  ros::Subscriber _subLidarOdometry;  // LiDAR Odometry subscriber
  tf::TransformListener _tfListener;

  /// Messages
  sensor_msgs::Imu _imuBiasMsg;  // IMU bias publishing ROS message

  /// Motion Parameters
  bool _gravityAttitudeInit = false;  // Flag if attitude from gravity were initialized
  tf::Transform _gravityAttitude;     // Attitude from initial gravity alignment
  bool _zeroMotionDetection = false;  // Detect and Add Zero Motion Factors(Zero delta Pose and Velocity)

  /// Timing
  double _imuTimeOffset =
      0.0;  // Offset between IMU and LiDAR Measurements - Depending on LiDAR timestamp first(+0.05) or last(-0.05)
  float _scanPeriod;  // time per scan
  uint16_t _ioRatio;  // ratio of input to output frames

  /// Counter
  long _frameCount;  // number of processed frames

  /// Verbose
  int _verboseLevel = 0;
};

}  // end namespace fg_filtering

#endif  // LOAM_LASERODOMETRY_H
