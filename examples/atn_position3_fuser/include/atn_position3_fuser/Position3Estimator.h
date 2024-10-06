/*
Copyright 2024 by Julian Nubert & Timon Mathis, Robotic Systems Lab & Autonomous Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef POSITION3_ESTIMATOR_H
#define POSITION3_ESTIMATOR_H

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

// Leica Position extension
#include <geometry_msgs/PointStamped.h>

// Workspace
#include "atn_position3_fuser/Position3StaticTransforms.h"
#include "graph_msf/gnss/GnssHandler.h"
#include "graph_msf/measurements/UnaryMeasurementXD.h"
#include "graph_msf_ros/GraphMsfRos.h"

// Defined Macros
#define POS_COVARIANCE_VIOLATION_THRESHOLD 0.2
#define NUM_GNSS_CALLBACKS_UNTIL_START 20

namespace position3_se {

class Position3Estimator : public graph_msf::GraphMsfRos {
 public:
  Position3Estimator(std::shared_ptr<ros::NodeHandle> privateNodePtr);
  // Destructor
  ~Position3Estimator() = default;
  // Setup
  void setup();

 protected:
  // Methods ------------------------------------
  void initializePublishers(ros::NodeHandle& privateNode) override;

  void initializeSubscribers(ros::NodeHandle& privateNode) override;

  void initializeMessages(ros::NodeHandle& privateNode) override;

  void initializeServices(ros::NodeHandle& privateNode) override;

  void readParams(const ros::NodeHandle& privateNode) override;

 private:
  // Callbacks
  void prismPositionCallback_(const geometry_msgs::PointStamped::ConstPtr& leicaPositionPtr);
  void gnssPositionCallback_(const sensor_msgs::NavSatFix::ConstPtr& gnssPositionPtr);
  void gnssOfflinePoseCallback_(const nav_msgs::Odometry::ConstPtr& gnssOfflinePosePtr);

  // Services
  bool srvTogglePrismUnaryCallback_(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool srvToggleGnssUnaryCallback_(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool srvToggleGnssOfflinePoseUnaryCallback_(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  // Members ----------------------------------
  // Publishers
  // Path
  ros::Publisher pubMeasWorldPrismPositionPath_;
  ros::Publisher pubMeasWorldGnssPositionPath_;
  ros::Publisher pubMeasWorldGnssOfflinePosePath_;

  // Messages
  // Paths
  nav_msgs::PathPtr measPosition_worldPrismPositionPathPtr_;
  nav_msgs::PathPtr measPosition_worldGnssPositionPathPtr_;
  nav_msgs::PathPtr measPosition_worldGnssOfflinePosePathPtr_;

  // Services
  // Trigger offline smoother optimization
  ros::ServiceServer srvTogglePrismPositionUnary_;
  ros::ServiceServer srvToggleGnssPositionUnary_;
  ros::ServiceServer srvToggleGnssOfflinePoseUnary_;

  // Subscribers
  ros::Subscriber subPrismPosition_;
  ros::Subscriber subGnssPosition_;
  ros::Subscriber subGnssOfflinePose_;
  // TF
  tf::TransformListener tfListener_;

  // Time
  std::chrono::time_point<std::chrono::high_resolution_clock> startTime_;
  std::chrono::time_point<std::chrono::high_resolution_clock> currentTime_;

  // GNSS Handler
  std::shared_ptr<graph_msf::GnssHandler> gnssHandlerPtr_;

  // Alignment Parameters
  Eigen::Matrix<double, 6, 1> initialSe3AlignmentNoise_ = 10 * Eigen::Matrix<double, 6, 1>::Ones();

  // Rates
  double prismPositionRate_ = 20.0;
  double gnssPositionRate_ = 10.0;
  double gnssOfflinePoseRate_ = 10.0;

  // Noise
  double prismPositionMeasUnaryNoise_ = 1.0;  // in [m]
  double gnssPositionMeasUnaryNoise_ = 1.0;   // in [m]
  Eigen::Matrix<double, 6, 1> gnssOfflinePoseMeasUnaryNoise_ = 1.0 * Eigen::Matrix<double, 6, 1>::Ones();

  // Variables
  Eigen::Vector3d accumulatedGnssCoordinates_{0.0, 0.0, 0.0};
  int prismPositionCallbackCounter_ = 0;
  int gnssPositionCallbackCounter_ = 0;
  int gnssOfflinePoseCallbackCounter_ = 0;

  // Flags
  static constexpr bool constexprUsePrismPositionUnaryFlag_ = false;
  static constexpr bool constexprUseGnssPositionUnaryFlag_ = false;
  static constexpr bool constexprUseGnssOfflinePoseUnaryFlag_ = true;
  bool usePrismPositionUnaryFlag_ = constexprUsePrismPositionUnaryFlag_;
  bool useGnssPositionUnaryFlag_ = constexprUseGnssPositionUnaryFlag_;
  bool useGnssOfflinePoseUnaryFlag_ = constexprUseGnssOfflinePoseUnaryFlag_;
};

}  // namespace position3_se

#endif  // end POSITION3_ESTIMATOR_H
