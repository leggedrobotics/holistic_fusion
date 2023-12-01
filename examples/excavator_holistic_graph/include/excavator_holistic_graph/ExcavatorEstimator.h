/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef EXCAVATOR_ESTIMATOR_H
#define EXCAVATOR_ESTIMATOR_H

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
#include <tf/transform_listener.h>

// Workspace
#include "excavator_holistic_graph/ExcavatorStaticTransforms.h"
#include "graph_msf/gnss/GnssHandler.h"
#include "graph_msf/measurements/UnaryMeasurementXD.h"
#include "graph_msf_ros/GraphMsfRos.h"

// Defined Macros
#define NUM_GNSS_CALLBACKS_UNTIL_START 20
#define GNSS_POSITION_COVARIANCE_VIOLATION_THRESHOLD 0.2  // 10000
#define GNSS_HEADING_COVARIANCE_VIOLATION_THRESHOLD 1.0   // 10000

namespace excavator_se {

class ExcavatorEstimator : public graph_msf::GraphMsfRos {
 public:
  ExcavatorEstimator(std::shared_ptr<ros::NodeHandle> privateNodePtr);
  // Destructor
  ~ExcavatorEstimator() = default;
  // Setup
  virtual bool setup() override;

 private:
  // Methods ------------------------------------
  void initializePublishers_(ros::NodeHandle& privateNode) override;

  void initializeSubscribers_(ros::NodeHandle& privateNode) override;

  void initializeMessages_(ros::NodeHandle& privateNode) override;

  void readParams_(const ros::NodeHandle& privateNode) override;

  // Callbacks
  void lidarOdometryCallback_(const nav_msgs::Odometry::ConstPtr& lidar_odom_ptr);
  void gnssCallback_(const sensor_msgs::NavSatFix::ConstPtr& leftGnssPtr, const sensor_msgs::NavSatFix::ConstPtr& rightGnssPtr);

  // Publish State
  virtual void publishState_(
      const std::shared_ptr<graph_msf::SafeIntegratedNavState>& preIntegratedNavStatePtr,
      const std::shared_ptr<graph_msf::SafeNavStateWithCovarianceAndBias>& optimizedStateWithCovarianceAndBiasPtr) override;

  // Members ----------------------------------
  // Publishers
  // Path
  ros::Publisher pubMeasMapLioPath_;
  ros::Publisher pubMeasWorldGnssLPath_;
  ros::Publisher pubMeasWorldGnssRPath_;

  // Messages
  // Paths
  nav_msgs::PathPtr measLio_mapImuPathPtr_;
  nav_msgs::PathPtr measGnss_worldGnssLPathPtr_;
  nav_msgs::PathPtr measGnss_worldGnssRPathPtr_;

  // Subscribers
  ros::Subscriber subImu_;
  ros::Subscriber subLidarOdometry_;
  message_filters::Subscriber<sensor_msgs::NavSatFix> subGnssLeft_;
  message_filters::Subscriber<sensor_msgs::NavSatFix> subGnssRight_;
  tf::TransformListener tfListener_;

  // GNSS Sync Policy
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::NavSatFix, sensor_msgs::NavSatFix> GnssExactSyncPolicy;
  boost::shared_ptr<message_filters::Synchronizer<GnssExactSyncPolicy>> gnssExactSyncPtr_;  // ROS Exact Sync Policy Message Filter

  // Time
  std::chrono::time_point<std::chrono::high_resolution_clock> startTime_;
  std::chrono::time_point<std::chrono::high_resolution_clock> currentTime_;

  // GNSS Handler
  std::shared_ptr<graph_msf::GnssHandler> gnssHandlerPtr_;

  // Rates
  double lioOdometryRate_ = 5.0;
  double gnssRate_ = 20.0;

  // Noise
  Eigen::Matrix<double, 6, 1> lioPoseUnaryNoise_;
  double gnssPositionUnaryNoise_ = 1.0;  // in [m]
  double gnssHeadingUnaryNoise_ = 1.0;   // in [rad]

  /// Flags
  bool useLioOdometryFlag_ = true;
  bool useLeftGnssFlag_ = true;
  bool useRightGnssFlag_ = true;
  bool useGnssYawFlag_ = true;
};
}  // namespace excavator_se
#endif  // end EXCAVATOR_ESTIMATOR_H
