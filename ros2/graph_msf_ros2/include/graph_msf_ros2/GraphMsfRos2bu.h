/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#pragma once
// std
#include <chrono>
#include <map>
#include <memory>
#include <mutex>

// ROS 2
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>

// Workspace
#include "graph_msf/interface/GraphMsf.h"
#include "graph_msf/interface/GraphMsfClassic.h"
#include "graph_msf/interface/GraphMsfHolistic.h"
// #include "graph_msf_ros2/srv/OfflineOptimizationTrigger.hpp"

// Macros
#define ROS_QUEUE_SIZE 1

namespace graph_msf {

class GraphMsfRos2 : public GraphMsfClassic, public GraphMsfHolistic {
 public:
  explicit GraphMsfRos2(const std::shared_ptr<rclcpp::Node>& node);
  ~GraphMsfRos2() override = default;

  // Setup
  void setup(const std::shared_ptr<StaticTransforms>& staticTransformsPtr);

 protected:
  // Functions that need implementation
  virtual void initializePublishers();
  virtual void initializeSubscribers();
  virtual void initializeMessages();
  //   virtual void initializeServices(rclcpp::Node& node);

  // Commodity Functions to be shared -----------------------------------
  static void addToPathMsg(const std::shared_ptr<nav_msgs::msg::Path>& pathPtr, const std::string& frameName,
                           const rclcpp::Time& stamp, const Eigen::Vector3d& t, int maxBufferLength);
  static void addToOdometryMsg(
      const std::shared_ptr<nav_msgs::msg::Odometry>& msgPtr, const std::string& fixedFrame,
      const std::string& movingFrame, const rclcpp::Time& stamp, const Eigen::Isometry3d& T,
      const Eigen::Vector3d& W_v_W_F, const Eigen::Vector3d& W_w_W_F,
      const Eigen::Matrix<double, 6, 6>& poseCovariance = Eigen::Matrix<double, 6, 6>::Zero(),
      const Eigen::Matrix<double, 6, 6>& twistCovariance = Eigen::Matrix<double, 6, 6>::Zero());
  static void addToPoseWithCovarianceStampedMsg(
      const std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped>& msgPtr, const std::string& frameName,
      const rclcpp::Time& stamp, const Eigen::Isometry3d& T,
      const Eigen::Matrix<double, 6, 6>& transformCovariance = Eigen::Matrix<double, 6, 6>::Zero());

  static void extractCovariancesFromOptimizedState(
      Eigen::Matrix<double, 6, 6>& poseCovarianceRos, Eigen::Matrix<double, 6, 6>& twistCovarianceRos,
      const std::shared_ptr<graph_msf::SafeNavStateWithCovarianceAndBias>& optimizedStateWithCovarianceAndBiasPtr);

  static void createVelocityMarker(const std::string& referenceFrameName, const rclcpp::Time& stamp,
                                   const Eigen::Vector3d& velocity, visualization_msgs::msg::Marker& marker);
  void createContactMarker(const std::string& referenceFrameName, const rclcpp::Time& stamp,
                           const Eigen::Vector3d& position, const std::string& nameSpace, const int id,
                           visualization_msgs::msg::Marker& marker);

  // Parameter Loading -----------------------------------
  virtual void readParams(const rclcpp::Node& node);

  // Callbacks
  virtual void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imuPtr);

  // Services
  //   bool srvOfflineSmootherOptimizeCallback(
  //       const std::shared_ptr<graph_msf_ros2::srv::OfflineOptimizationTrigger::Request> req,
  //       std::shared_ptr<graph_msf_ros2::srv::OfflineOptimizationTrigger::Response> res);

  // Publishing -----------------------------------
  virtual void publishState(
      const std::shared_ptr<graph_msf::SafeIntegratedNavState>& integratedNavStatePtr,
      const std::shared_ptr<graph_msf::SafeNavStateWithCovarianceAndBias>& optimizedStateWithCovarianceAndBiasPtr);
  void publishNonTimeCriticalData(
      const Eigen::Matrix<double, 6, 6> poseCovarianceRos, const Eigen::Matrix<double, 6, 6> twistCovarianceRos,
      const Eigen::Vector3d positionVarianceRos, const Eigen::Vector3d orientationVarianceRos,
      const std::shared_ptr<const graph_msf::SafeIntegratedNavState> integratedNavStatePtr,
      const std::shared_ptr<const graph_msf::SafeNavStateWithCovarianceAndBias> optimizedStateWithCovarianceAndBiasPtr);
  void publishOptimizedStateAndBias(
      const std::shared_ptr<const graph_msf::SafeNavStateWithCovarianceAndBias> optimizedStateWithCovarianceAndBiasPtr,
      const Eigen::Matrix<double, 6, 6>& poseCovarianceRos, const Eigen::Matrix<double, 6, 6>& twistCovarianceRos);

  void publishTfTreeTransform(const std::string& frameName, const std::string& childFrameName, double timeStamp,
                              const Eigen::Isometry3d& T_frame_childFrame);
  void publishImuOdoms(const std::shared_ptr<const graph_msf::SafeIntegratedNavState>& preIntegratedNavStatePtr,
                       const Eigen::Matrix<double, 6, 6>& poseCovarianceRos,
                       const Eigen::Matrix<double, 6, 6>& twistCovarianceRos) const;
  void publishDiagVarianceVectors(const Eigen::Vector3d& posVarianceRos, const Eigen::Vector3d& rotVarianceRos,
                                  const double timeStamp) const;
  void publishVelocityMarkers(const std::shared_ptr<const graph_msf::SafeIntegratedNavState>& navStatePtr) const;
  void publishImuPaths(const std::shared_ptr<const graph_msf::SafeIntegratedNavState>& navStatePtr) const;
  void publishAddedImuMeas(const Eigen::Matrix<double, 6, 1>& addedImuMeas, const rclcpp::Time& stamp) const;

  // Measure time
  long secondsSinceStart();

  // Node
  std::shared_ptr<rclcpp::Node> node_;

  // Time
  std::chrono::time_point<std::chrono::high_resolution_clock> startTime;
  std::chrono::time_point<std::chrono::high_resolution_clock> currentTime;

  // Publishers
  tf2_ros::TransformBroadcaster tfBroadcaster;

  // Members
  std::string referenceFrameAlignedNameId = "_graph_msf_aligned";
  std::string sensorFrameCorrectedNameId = "_graph_msf_corrected";
  std::string optimizationResultLoggingPath = "";

 private:
  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubEstOdomImu_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubEstWorldImu_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOptWorldImu_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pubEstWorldPosVariance_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pubEstWorldRotVariance_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pubVelocityMarker_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubEstOdomImuPath_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubEstWorldImuPath_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubOptWorldImuPath_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubMeasWorldGnssLPath_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubMeasWorldGnssRPath_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubMeasWorldLidarPath_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pubAccelBias_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pubGyroBias_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pubAddedImuMeas_;

  std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr>
      pubPoseStampedByTopicMap_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu_;

  // Messages
  std::shared_ptr<nav_msgs::msg::Odometry> estOdomImuMsgPtr_;
  std::shared_ptr<nav_msgs::msg::Odometry> estWorldImuMsgPtr_;
  std::shared_ptr<nav_msgs::msg::Odometry> optWorldImuMsgPtr_;
  std::shared_ptr<geometry_msgs::msg::Vector3Stamped> estWorldPosVarianceMsgPtr_;
  std::shared_ptr<geometry_msgs::msg::Vector3Stamped> estWorldRotVarianceMsgPtr_;
  std::shared_ptr<nav_msgs::msg::Path> estOdomImuPathPtr_;
  std::shared_ptr<nav_msgs::msg::Path> estWorldImuPathPtr_;
  std::shared_ptr<nav_msgs::msg::Path> optWorldImuPathPtr_;
  std::shared_ptr<nav_msgs::msg::Path> measWorldLidarPathPtr_;
  std::shared_ptr<geometry_msgs::msg::Vector3Stamped> accelBiasMsgPtr_;
  std::shared_ptr<geometry_msgs::msg::Vector3Stamped> gyroBiasMsgPtr_;

  // Service server
  //   rclcpp::Service<graph_msf_ros2::srv::OfflineOptimizationTrigger>::SharedPtr srvSmootherOptimize_;

  // Last Optimized State Timestamp
  double lastOptimizedStateTimestamp_ = 0.0;

  // Mutex
  std::mutex rosPublisherMutex_;
};

}  // namespace graph_msf
