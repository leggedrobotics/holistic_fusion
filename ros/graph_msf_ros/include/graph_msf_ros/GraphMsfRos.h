/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPHMSFROS_H
#define GRAPHMSFROS_H

// std
#include <chrono>

// ROS
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

// Workspace
#include "graph_msf/interface/GraphMsf.h"
#include "graph_msf/interface/GraphMsfClassic.h"
#include "graph_msf/interface/GraphMsfHolistic.h"
#include "graph_msf_ros_msgs/OfflineOptimizationTrigger.h"

// Macros
#define ROS_QUEUE_SIZE 1

namespace graph_msf {

class GraphMsfRos : public GraphMsfClassic, public GraphMsfHolistic {
 public:
  explicit GraphMsfRos(const std::shared_ptr<ros::NodeHandle>& privateNodePtr);
  // Destructor
  ~GraphMsfRos() override = default;

  // Setup
  void setup(const std::shared_ptr<StaticTransforms> staticTransformsPtr);

 protected:
  // Functions that need implementation
  virtual void initializePublishers(ros::NodeHandle& privateNode);
  virtual void initializeSubscribers(ros::NodeHandle& privateNode);
  virtual void initializeMessages(ros::NodeHandle& privateNode);
  virtual void initializeServices(ros::NodeHandle& privateNode);

  // Commodity Functions to be shared -----------------------------------
  // Static
  // Add to Topics
  static void addToPathMsg(const nav_msgs::PathPtr& pathPtr, const std::string& frameName, const ros::Time& stamp, const Eigen::Vector3d& t,
                           int maxBufferLength);
  static void addToOdometryMsg(const nav_msgs::OdometryPtr& msgPtr, const std::string& fixedFrame, const std::string& movingFrame,
                               const ros::Time& stamp, const Eigen::Isometry3d& T, const Eigen::Vector3d& W_v_W_F,
                               const Eigen::Vector3d& W_w_W_F,
                               const Eigen::Matrix<double, 6, 6>& poseCovariance = Eigen::Matrix<double, 6, 6>::Zero(),
                               const Eigen::Matrix<double, 6, 6>& twistCovariance = Eigen::Matrix<double, 6, 6>::Zero());
  static void addToPoseWithCovarianceStampedMsg(
      const geometry_msgs::PoseWithCovarianceStampedPtr& msgPtr, const std::string& frameName, const ros::Time& stamp,
      const Eigen::Isometry3d& T, const Eigen::Matrix<double, 6, 6>& transformCovariance = Eigen::Matrix<double, 6, 6>::Zero());
  // Extract from State
  static void extractCovariancesFromOptimizedState(
      Eigen::Matrix<double, 6, 6>& poseCovarianceRos, Eigen::Matrix<double, 6, 6>& twistCovarianceRos,
      const std::shared_ptr<graph_msf::SafeNavStateWithCovarianceAndBias>& optimizedStateWithCovarianceAndBiasPtr);
  // Markers
  static void createVelocityMarker(const std::string& referenceFrameName, const ros::Time& stamp, const Eigen::Vector3d& velocity,
                                   visualization_msgs::Marker& marker);
  void createContactMarker(const std::string& referenceFrameName, const ros::Time& stamp, const Eigen::Vector3d& position,
                           const std::string& nameSpace, const int id, visualization_msgs::Marker& marker);

  // Parameter Loading -----------------------------------
  virtual void readParams(const ros::NodeHandle& privateNode);

  // Callbacks
  virtual void imuCallback(const sensor_msgs::Imu::ConstPtr& imuPtr);

  // Services
  bool srvOfflineSmootherOptimizeCallback(graph_msf_ros_msgs::OfflineOptimizationTrigger::Request& req,
                                          graph_msf_ros_msgs::OfflineOptimizationTrigger::Response& res);
  bool srvLogRealTimeStatesCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  // Publishing -----------------------------------
  // Higher Level Functions
  virtual void publishState(const std::shared_ptr<graph_msf::SafeIntegratedNavState>& integratedNavStatePtr,
                            const std::shared_ptr<graph_msf::SafeNavStateWithCovarianceAndBias>& optimizedStateWithCovarianceAndBiasPtr);
  void publishNonTimeCriticalData(
      const Eigen::Matrix<double, 6, 6> poseCovarianceRos, const Eigen::Matrix<double, 6, 6> twistCovarianceRos,
      const Eigen::Vector3d positionVarianceRos, const Eigen::Vector3d orientationVarianceRos,
      const std::shared_ptr<const graph_msf::SafeIntegratedNavState> integratedNavStatePtr,
      const std::shared_ptr<const graph_msf::SafeNavStateWithCovarianceAndBias> optimizedStateWithCovarianceAndBiasPtr);
  void publishOptimizedStateAndBias(
      const std::shared_ptr<const graph_msf::SafeNavStateWithCovarianceAndBias> optimizedStateWithCovarianceAndBiasPtr,
      const Eigen::Matrix<double, 6, 6>& poseCovarianceRos, const Eigen::Matrix<double, 6, 6>& twistCovarianceRos);

  // Lower Level Functions
  void publishTfTreeTransform(const std::string& frameName, const std::string& childFrameName, double timeStamp,
                              const Eigen::Isometry3d& T_frame_childFrame);
  void publishImuOdoms(const std::shared_ptr<const graph_msf::SafeIntegratedNavState>& preIntegratedNavStatePtr,
                       const Eigen::Matrix<double, 6, 6>& poseCovarianceRos, const Eigen::Matrix<double, 6, 6>& twistCovarianceRos) const;
  void publishDiagVarianceVectors(const Eigen::Vector3d& posVarianceRos, const Eigen::Vector3d& rotVarianceRos,
                                  const double timeStamp) const;
  void publishVelocityMarkers(const std::shared_ptr<const graph_msf::SafeIntegratedNavState>& navStatePtr) const;
  void publishImuPaths(const std::shared_ptr<const graph_msf::SafeIntegratedNavState>& navStatePtr) const;
  void publishAddedImuMeas(const Eigen::Matrix<double, 6, 1>& addedImuMeas, const ros::Time& stamp) const;

  // Measure time
  long secondsSinceStart();

  // Node
  ros::NodeHandle privateNode;

  // Time
  std::chrono::time_point<std::chrono::high_resolution_clock> startTime;
  std::chrono::time_point<std::chrono::high_resolution_clock> currentTime;

  // Publishers
  // TF
  tf::TransformBroadcaster tfBroadcaster;

  // Members
  std::string referenceFrameAlignedNameId = "_graph_msf_aligned";
  std::string sensorFrameCorrectedNameId = "_graph_msf_corrected";
  std::string optimizationResultLoggingPath = "";

 private:
  // Publishers
  // Odometry
  ros::Publisher pubEstOdomImu_;
  ros::Publisher pubEstWorldImu_;
  ros::Publisher pubOptWorldImu_;
  // Vector3 Variances
  ros::Publisher pubEstWorldPosVariance_;
  ros::Publisher pubEstWorldRotVariance_;
  // Velocity Markers
  ros::Publisher pubVelocityMarker_;
  // Path
  ros::Publisher pubEstOdomImuPath_;
  ros::Publisher pubEstWorldImuPath_;
  ros::Publisher pubOptWorldImuPath_;
  ros::Publisher pubMeasWorldGnssLPath_;
  ros::Publisher pubMeasWorldGnssRPath_;
  ros::Publisher pubMeasWorldLidarPath_;
  // Imu Bias
  ros::Publisher pubAccelBias_;
  ros::Publisher pubGyroBias_;
  // Added Imu Measurements
  ros::Publisher pubAddedImuMeas_;

  // PoseStamped --> Needs to be dynamic as we do not know the number of sensors
  std::map<std::string, ros::Publisher> pubPoseStampedByTopicMap_ = {};

  // Subscribers
  ros::Subscriber subImu_;

  // Messages
  // Odometry
  nav_msgs::OdometryPtr estOdomImuMsgPtr_;
  nav_msgs::OdometryPtr estWorldImuMsgPtr_;
  nav_msgs::OdometryPtr optWorldImuMsgPtr_;
  // Vector3 Variances
  geometry_msgs::Vector3StampedPtr estWorldPosVarianceMsgPtr_;
  geometry_msgs::Vector3StampedPtr estWorldRotVarianceMsgPtr_;
  // Path
  // Estimated
  nav_msgs::PathPtr estOdomImuPathPtr_;
  nav_msgs::PathPtr estWorldImuPathPtr_;
  nav_msgs::PathPtr optWorldImuPathPtr_;
  // Measured
  nav_msgs::PathPtr measWorldLidarPathPtr_;
  // Imu Bias
  geometry_msgs::Vector3StampedPtr accelBiasMsgPtr_;
  geometry_msgs::Vector3StampedPtr gyroBiasMsgPtr_;

  // Services
  // Trigger offline smoother optimization
  ros::ServiceServer srvSmootherOptimize_;
  // Log Real-Time States
  ros::ServiceServer srvLogRealTimeStates_;

  // Last Optimized State Timestamp
  double lastOptimizedStateTimestamp_ = 0.0;

  // Mutex
  std::mutex rosPublisherMutex_;
};
}  // namespace graph_msf

#endif  // end GRAPHMSFROS_H
