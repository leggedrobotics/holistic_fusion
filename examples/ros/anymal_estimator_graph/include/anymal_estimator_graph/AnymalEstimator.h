/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef ANYMAL_ESTIMATOR_H
#define ANYMAL_ESTIMATOR_H

// std
#include <chrono>

// ROS
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

// Custom Messages
#include "anymal_msgs/AnymalState.h"

// Workspace
#include "graph_msf/gnss/GnssHandler.h"
#include "graph_msf/measurements/UnaryMeasurementXD.h"
#include "graph_msf/trajectory_alignment/TrajectoryAlignmentHandler.h"
#include "graph_msf_ros/GraphMsfRos.h"

// Defined Macros
#define NUM_GNSS_CALLBACKS_UNTIL_START 20

namespace anymal_se {

class AnymalEstimator : public graph_msf::GraphMsfRos {
 public:
  AnymalEstimator(const std::shared_ptr<ros::NodeHandle>& privateNodePtr);
  // Destructor
  ~AnymalEstimator() = default;

  // Setup
  void setup();

 protected:
  // Virtual Functions
  void initializePublishers(ros::NodeHandle& privateNode) override;
  void initializeSubscribers(ros::NodeHandle& privateNode) override;
  void initializeMessages(ros::NodeHandle& privateNode) override;
  void initializeServices_(ros::NodeHandle& privateNode);

  // Parameter Loading
  void readParams(const ros::NodeHandle& privateNode) override;

  // Callbacks
  bool srvOfflineSmootherOptimizeCallback(graph_msf_ros_msgs::OfflineOptimizationTrigger::Request& req,
                                          graph_msf_ros_msgs::OfflineOptimizationTrigger::Response& res) override;

 private:
  // Callbacks
  // LIO
  void lidarUnaryCallback_(const nav_msgs::Odometry::ConstPtr& lidar_odom_ptr);
  void lidarBetweenCallback_(const nav_msgs::Odometry::ConstPtr& lidar_odom_ptr);
  // GNSS
  void gnssUnaryCallback_(const sensor_msgs::NavSatFix::ConstPtr& gnssPtr);
  // Legged
  void leggedBetweenCallback_(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& leggedOdometryPoseKPtr);
  void leggedVelocityUnaryCallback_(const nav_msgs::Odometry::ConstPtr& leggedOdometryKPtr);
  void leggedKinematicsCallback_(const anymal_msgs::AnymalState::ConstPtr& anymalStatePtr);

  // GNSS Handler
  std::shared_ptr<graph_msf::GnssHandler> gnssHandlerPtr_;

  // TrajectoryAlignment Handler
  std::shared_ptr<graph_msf::TrajectoryAlignmentHandler> trajectoryAlignmentHandler_;

  // Time
  std::chrono::time_point<std::chrono::high_resolution_clock> startTime_;
  std::chrono::time_point<std::chrono::high_resolution_clock> currentTime_;

  // Config -------------------------------------

  // Rates
  double lioOdometryRate_ = 5.0;
  double leggedOdometryBetweenRate_ = 400.0;
  int leggedOdometryPoseDownsampleFactor_ = 40;
  double leggedOdometryVelocityRate_ = 20.0;
  int leggedOdometryVelocityDownsampleFactor_ = 4;
  double leggedKinematicsRate_ = 400.0;
  int leggedKinematicsDownsampleFactor_ = 10;
  double gnssRate_ = 10.0;

  // Flags
  bool useGnssUnaryFlag_ = false;
  bool useLioUnaryFlag_ = true;
  bool useLioBetweenFlag_ = false;
  bool useLeggedBetweenFlag_ = false;
  bool useLeggedVelocityUnaryFlag_ = false;
  bool useLeggedKinematicsFlag_ = false;

  // Alignment Parameters
  Eigen::Matrix<double, 6, 1> initialSe3AlignmentNoise_ = 1.0 * Eigen::Matrix<double, 6, 1>::Ones();
  Eigen::Matrix<double, 6, 1> lioSe3AlignmentRandomWalk_ = 0.0 * Eigen::Matrix<double, 6, 1>::Ones();

  // Noise
  double gnssPositionOutlierThreshold_ = 1.0;
  Eigen::Matrix<double, 6, 1> lioPoseUnaryNoise_ = 1.0 * Eigen::Matrix<double, 6, 1>::Ones();
  Eigen::Matrix<double, 6, 1> lioPoseBetweenNoise_ = 1.0 * Eigen::Matrix<double, 6, 1>::Ones();
  Eigen::Matrix<double, 6, 1> legPoseBetweenNoise_ = 1.0 * Eigen::Matrix<double, 6, 1>::Ones();
  Eigen::Matrix<double, 3, 1> legVelocityUnaryNoise_ = 1.0 * Eigen::Matrix<double, 3, 1>::Ones();
  Eigen::Matrix<double, 3, 1> legKinematicsFootPositionUnaryNoise_ = 1.0 * Eigen::Matrix<double, 3, 1>::Ones();

  // Leg Odometry Handling
  static constexpr std::array<const char*, 4> legNames_ = {"LF", "RF", "LH", "RH"};
  std::array<long, legNames_.size()> legInContactForNSteps_ = {0, 0, 0, 0};
  std::array<long, legNames_.size()> legTotalContactsCounter_ = {0, 0, 0, 0};
  static constexpr long legInContactDebounceThreshold_ = 3;

  // Callback Members ----------------------------
  // GNSS
  Eigen::Vector3d accumulatedGnssCoordinates_{0.0, 0.0, 0.0};
  int gnssCallbackCounter_{-1};
  std::vector<Eigen::Vector3d> optionalGnssEnuCoordinatesTrajectory_;
  std::vector<double> optionalGnssEnuCoordinatesTimeStamps_;
  static constexpr bool logOptionalGnssEnuCoordinatesFlag_ = false;
  // LIO
  // Unary
  int lidarUnaryCallbackCounter_{-1};
  // Between
  int lidarBetweenCallbackCounter_{-1};
  double lidarBetweenTimeKm1_{0.0};
  Eigen::Isometry3d lio_T_M_Lkm1_ = Eigen::Isometry3d::Identity();
  // Legged
  int leggedOdometryPoseCallbackCounter_{-1};
  int leggedOdometryOdomCallbackCounter_{-1};
  int leggedKinematicsCallbackCounter_{-1};
  Eigen::Isometry3d T_O_Bl_km1_ = Eigen::Isometry3d::Identity();  // Odometry is in body frame
  double legOdometryTimeKm1_{0.0};

  // ROS Objects ----------------------------

  // Subscribers
  // Instances
  ros::Subscriber subGnssUnary_;
  ros::Subscriber subLioUnary_;
  ros::Subscriber subLioBetween_;
  ros::Subscriber subLeggedBetween_;
  ros::Subscriber subLeggedVelocityUnary_;
  ros::Subscriber subLeggedKinematics_;
  tf::TransformListener tfListener_;

  // Publishers
  // Path
  ros::Publisher pubMeasMapLioPath_;
  ros::Publisher pubMeasWorldGnssPath_;
  // Markers
  ros::Publisher pubFootContactMarkers_;

  // Messages
  nav_msgs::PathPtr measLio_mapLidarPathPtr_;
  nav_msgs::PathPtr measGnss_worldGnssPathPtr_;

  // Servers
  ros::ServiceServer serverTransformGnssToEnu_;
};

}  // namespace anymal_se

#endif  // end ANYMAL_ESTIMATOR_H
