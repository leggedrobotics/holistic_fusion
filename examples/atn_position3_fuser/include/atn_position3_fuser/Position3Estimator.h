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

  void readParams(const ros::NodeHandle& privateNode) override;

 private:

  // Callbacks
  void positionCallback_(const geometry_msgs::PointStamped::ConstPtr& LeicaPositionPtr);

  // Members ----------------------------------
  // Publishers
  // Path
  ros::Publisher pubMeasWorldPositionPath_;

  // Messages
  // Paths
  nav_msgs::PathPtr measPosition_worldPositionPathPtr_;

  // Subscribers
  ros::Subscriber subImu_;
  ros::Subscriber subPosition_;

  tf::TransformListener tfListener_;

  // Time
  std::chrono::time_point<std::chrono::high_resolution_clock> startTime_;
  std::chrono::time_point<std::chrono::high_resolution_clock> currentTime_;

  // GNSS Handler
  std::shared_ptr<graph_msf::GnssHandler> gnssHandlerPtr_;

  // Alignment Parameters
  Eigen::Matrix<double, 6, 1> initialSe3AlignmentNoise_ = 10 * Eigen::Matrix<double, 6, 1>::Ones();

  // Rates
  double positionRate_ = 20.0;

  // Noise
  double positionMeasUnaryNoise_ = 1.0;  // in [m]

  // Variables
  int positionCallbackCounter_ = 0;
  bool measuredPositionHealthyFlag_ = true;
};

}  // namespace position3_se

#endif  // end POSITION3_ESTIMATOR_H
