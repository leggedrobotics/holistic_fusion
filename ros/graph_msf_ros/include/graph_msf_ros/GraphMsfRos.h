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

// Workspace
#include "graph_msf/interface/GraphMsf.h"
#include "graph_msf_ros/OfflineOptimizationTrigger.h"

// Macros
#define ROS_QUEUE_SIZE 1

namespace graph_msf {

class GraphMsfRos : public GraphMsf {
 public:
  explicit GraphMsfRos(const std::shared_ptr<ros::NodeHandle>& privateNodePtr);
  // Destructor
  ~GraphMsfRos() override = default;
  // Setup
  bool setup() override;

 protected:
  // Functions that need implementation
  virtual void initializePublishers_(ros::NodeHandle& privateNode);
  virtual void initializeSubscribers_(ros::NodeHandle& privateNode);
  virtual void initializeMessages_(ros::NodeHandle& privateNode);
  virtual void initializeServices_(ros::NodeHandle& privateNode);

  // Commodity Functions to be shared -----------------------------------
  // Static
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
  static void extractCovariancesFromOptimizedState(
      Eigen::Matrix<double, 6, 6>& poseCovarianceRos, Eigen::Matrix<double, 6, 6>& twistCovarianceRos,
      const std::shared_ptr<graph_msf::SafeNavStateWithCovarianceAndBias>& optimizedStateWithCovarianceAndBiasPtr);

  // Parameter Loading -----------------------------------
  virtual void readParams_(const ros::NodeHandle& privateNode);

  // Callbacks
  virtual void imuCallback_(const sensor_msgs::Imu::ConstPtr& imuPtr);

  // Services
  bool srvOfflineSmootherOptimizeCallback_(graph_msf_ros::OfflineOptimizationTrigger::Request& req,
                                           graph_msf_ros::OfflineOptimizationTrigger::Response& res);

  // Publishing -----------------------------------
  // Higher Level Functions
  virtual void publishState_(const std::shared_ptr<graph_msf::SafeIntegratedNavState>& integratedNavStatePtr,
                             const std::shared_ptr<graph_msf::SafeNavStateWithCovarianceAndBias>& optimizedStateWithCovarianceAndBiasPtr);
  void publishNonTimeCriticalData_(
      const Eigen::Isometry3d T_O_Ik, const double timeK, const Eigen::Matrix<double, 6, 6> poseCovarianceRos,
      const Eigen::Matrix<double, 6, 6> twistCovarianceRos, const Eigen::Vector3d positionVarianceRos,
      const Eigen::Vector3d orientationVarianceRos, const std::shared_ptr<const graph_msf::SafeIntegratedNavState> integratedNavStatePtr,
      const std::shared_ptr<const graph_msf::SafeNavStateWithCovarianceAndBias> optimizedStateWithCovarianceAndBiasPtr);
  void publishOptimizedStateAndBias_(
      const std::shared_ptr<const graph_msf::SafeNavStateWithCovarianceAndBias> optimizedStateWithCovarianceAndBiasPtr,
      const Eigen::Matrix<double, 6, 6>& poseCovarianceRos, const Eigen::Matrix<double, 6, 6>& twistCovarianceRos);

  // Lower Level Functions
  void publishTfTreeTransform_(const std::string& frameName, const std::string& childFrameName, double timeStamp,
                               const Eigen::Isometry3d& T_frame_childFrame);
  void publishImuOdoms_(const std::shared_ptr<const graph_msf::SafeIntegratedNavState>& preIntegratedNavStatePtr,
                        const Eigen::Matrix<double, 6, 6>& poseCovarianceRos, const Eigen::Matrix<double, 6, 6>& twistCovarianceRos) const;
  void publishDiagVarianceVectors(const Eigen::Vector3d& posVarianceRos, const Eigen::Vector3d& rotVarianceRos,
                                  const double timeStamp) const;
  void publishImuPaths_(const std::shared_ptr<const graph_msf::SafeIntegratedNavState>& navStatePtr) const;
  void publishAddedImuMeas_(const Eigen::Matrix<double, 6, 1>& addedImuMeas, const ros::Time& stamp) const;

  // Measure time
  long secondsSinceStart_();

  // Node
  ros::NodeHandle privateNode_;

  // Time
  std::chrono::time_point<std::chrono::high_resolution_clock> startTime_;
  std::chrono::time_point<std::chrono::high_resolution_clock> currentTime_;

  // Publishers
  // TF
  tf::TransformBroadcaster tfBroadcaster_;

  // Members
  std::string fixedFrameAlignedNameId_ = "_graph_msf_aligned";
  std::string sensorFrameCorrectedNameId_ = "_graph_msf_corrected";
  std::string optimizationResultLoggingPath_ = "";

 private:
  // Publishers
  // Odometry
  ros::Publisher pubEstOdomImu_;
  ros::Publisher pubEstMapImu_;
  ros::Publisher pubEstWorldImu_;
  ros::Publisher pubOptWorldImu_;
  // Vector3 Variances
  ros::Publisher pubEstWorldPosVariance_;
  ros::Publisher pubEstWorldRotVariance_;
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
  nav_msgs::OdometryPtr estMapImuMsgPtr_;
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

  // Last Optimized State Timestamp
  double lastOptimizedStateTimestamp_ = 0.0;

  // Mutex
  std::mutex rosPublisherMutex_;
};
}  // namespace graph_msf

#endif  // end GRAPHMSFROS_H
