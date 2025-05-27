// Implementation
#include "graph_msf_ros2/GraphMsfRos2.h"

// ROS 2
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Workspace
#include "graph_msf_ros2/constants.h"
#include "graph_msf_ros2/util/conversions.h"

namespace graph_msf {

GraphMsfRos2::GraphMsfRos2(const std::shared_ptr<rclcpp::Node>& nodePtr) : node_(nodePtr), tfBroadcaster(nodePtr) {
  RCLCPP_INFO(node_->get_logger(), "GraphMsfRos2-Constructor called.");
  graphConfigPtr_ = std::make_shared<GraphConfig>();
}

void GraphMsfRos2::setup(const std::shared_ptr<StaticTransforms>& staticTransformsPtr) {
  RCLCPP_INFO(node_->get_logger(), "GraphMsfRos2-Setup called.");

  if (staticTransformsPtr == nullptr) {
    throw std::runtime_error("Static transforms not set. They must be set.");
  }

  GraphMsfRos2::readParams(*node_);
  GraphMsf::setup(graphConfigPtr_, staticTransformsPtr);

  initializePublishers();
  initializeSubscribers();
  initializeMessages();
  // initializeServices();

  startTime = std::chrono::high_resolution_clock::now();
  RCLCPP_INFO(node_->get_logger(), "Set up successfully.");
}

void GraphMsfRos2::initializePublishers() {
  RCLCPP_INFO(node_->get_logger(), "Initializing publishers.");

  pubEstOdomImu_ = node_->create_publisher<nav_msgs::msg::Odometry>("/graph_msf/est_odometry_odom_imu", ROS_QUEUE_SIZE);
  pubEstWorldImu_ =
      node_->create_publisher<nav_msgs::msg::Odometry>("/graph_msf/est_odometry_world_imu", ROS_QUEUE_SIZE);
  pubOptWorldImu_ =
      node_->create_publisher<nav_msgs::msg::Odometry>("/graph_msf/opt_odometry_world_imu", ROS_QUEUE_SIZE);

  pubEstWorldPosVariance_ =
      node_->create_publisher<geometry_msgs::msg::Vector3Stamped>("/graph_msf/est_world_pos_variance", ROS_QUEUE_SIZE);
  pubEstWorldRotVariance_ =
      node_->create_publisher<geometry_msgs::msg::Vector3Stamped>("/graph_msf/est_world_rot_variance", ROS_QUEUE_SIZE);

  pubVelocityMarker_ =
      node_->create_publisher<visualization_msgs::msg::Marker>("/graph_msf/velocity_marker", ROS_QUEUE_SIZE);
  pubEstOdomImuPath_ = node_->create_publisher<nav_msgs::msg::Path>("/graph_msf/est_path_odom_imu", ROS_QUEUE_SIZE);
  pubEstWorldImuPath_ = node_->create_publisher<nav_msgs::msg::Path>("/graph_msf/est_path_world_imu", ROS_QUEUE_SIZE);
  pubOptWorldImuPath_ = node_->create_publisher<nav_msgs::msg::Path>("/graph_msf/opt_path_world_imu", ROS_QUEUE_SIZE);

  pubAccelBias_ = node_->create_publisher<geometry_msgs::msg::Vector3Stamped>("/graph_msf/accel_bias", ROS_QUEUE_SIZE);
  pubGyroBias_ = node_->create_publisher<geometry_msgs::msg::Vector3Stamped>("/graph_msf/gyro_bias", ROS_QUEUE_SIZE);
  pubAddedImuMeas_ = node_->create_publisher<sensor_msgs::msg::Imu>("/graph_msf/added_imu_meas", ROS_QUEUE_SIZE);
}

void GraphMsfRos2::initializeSubscribers() {
  RCLCPP_INFO(node_->get_logger(), "Initializing subscribers.");

  subImu_ = node_->create_subscription<sensor_msgs::msg::Imu>(
      "/imu_topic", ROS_QUEUE_SIZE, std::bind(&GraphMsfRos2::imuCallback, this, std::placeholders::_1));
  RCLCPP_INFO(node_->get_logger(), "Initialized main IMU subscriber with topic: /imu_topic");
}

void GraphMsfRos2::initializeMessages() {
  RCLCPP_INFO(node_->get_logger(), "Initializing messages.");

  estOdomImuMsgPtr_ = std::make_shared<nav_msgs::msg::Odometry>();
  estWorldImuMsgPtr_ = std::make_shared<nav_msgs::msg::Odometry>();
  optWorldImuMsgPtr_ = std::make_shared<nav_msgs::msg::Odometry>();

  estWorldPosVarianceMsgPtr_ = std::make_shared<geometry_msgs::msg::Vector3Stamped>();
  estWorldRotVarianceMsgPtr_ = std::make_shared<geometry_msgs::msg::Vector3Stamped>();

  estOdomImuPathPtr_ = std::make_shared<nav_msgs::msg::Path>();
  estWorldImuPathPtr_ = std::make_shared<nav_msgs::msg::Path>();
  optWorldImuPathPtr_ = std::make_shared<nav_msgs::msg::Path>();

  accelBiasMsgPtr_ = std::make_shared<geometry_msgs::msg::Vector3Stamped>();
  gyroBiasMsgPtr_ = std::make_shared<geometry_msgs::msg::Vector3Stamped>();
}

// void GraphMsfRos2::initializeServices() {
//   srvSmootherOptimize_ = node_->create_service<graph_msf_ros2::srv::OfflineOptimizationTrigger>(
//       "/graph_msf/trigger_offline_optimization",
//       std::bind(&GraphMsfRos2::srvOfflineSmootherOptimizeCallback, this, std::placeholders::_1,
//       std::placeholders::_2));
// }

// bool GraphMsfRos2::srvOfflineSmootherOptimizeCallback(
//     const std::shared_ptr<graph_msf_ros2::srv::OfflineOptimizationTrigger::Request> req,
//     std::shared_ptr<graph_msf_ros2::srv::OfflineOptimizationTrigger::Response> res) {
//   int maxIterations = req->max_optimization_iterations;

//   if (GraphMsf::optimizeSlowBatchSmoother(maxIterations, optimizationResultLoggingPath)) {
//     res->success = true;
//     res->message = "Optimization successful.";
//   } else {
//     res->success = false;
//     res->message = "Optimization failed.";
//   }
//   return true;
// }

// Main IMU Callback
void GraphMsfRos2::imuCallback(const sensor_msgs::msg::Imu::SharedPtr imuMsgPtr) {
  Eigen::Vector3d linearAcc(imuMsgPtr->linear_acceleration.x, imuMsgPtr->linear_acceleration.y,
                            imuMsgPtr->linear_acceleration.z);
  Eigen::Vector3d angularVel(imuMsgPtr->angular_velocity.x, imuMsgPtr->angular_velocity.y,
                             imuMsgPtr->angular_velocity.z);
  Eigen::Matrix<double, 6, 1> addedImuMeasurements;

  std::shared_ptr<SafeIntegratedNavState> preIntegratedNavStatePtr = nullptr;
  std::shared_ptr<SafeNavStateWithCovarianceAndBias> optimizedStateWithCovarianceAndBiasPtr = nullptr;

  if (GraphMsf::addCoreImuMeasurementAndGetState(
          linearAcc, angularVel, imuMsgPtr->header.stamp.sec + 1e-9 * imuMsgPtr->header.stamp.nanosec,
          preIntegratedNavStatePtr, optimizedStateWithCovarianceAndBiasPtr, addedImuMeasurements)) {
    if (node_->now().seconds() - preIntegratedNavStatePtr->getTimeK() > 0.5) {
      RCLCPP_WARN(node_->get_logger(), "Encountered delay of %f seconds.",
                  node_->now().seconds() - preIntegratedNavStatePtr->getTimeK());
    }

    this->publishState(preIntegratedNavStatePtr, optimizedStateWithCovarianceAndBiasPtr);
  } else if (GraphMsf::isGraphInited()) {
    RCLCPP_WARN(node_->get_logger(), "Could not add IMU measurement.");
  }
}

void GraphMsfRos2::publishState(
    const std::shared_ptr<graph_msf::SafeIntegratedNavState>& integratedNavStatePtr,
    const std::shared_ptr<graph_msf::SafeNavStateWithCovarianceAndBias>& optimizedStateWithCovarianceAndBiasPtr) {
  Eigen::Matrix<double, 6, 6> poseCovarianceRos, twistCovarianceRos;
  extractCovariancesFromOptimizedState(poseCovarianceRos, twistCovarianceRos, optimizedStateWithCovarianceAndBiasPtr);

  Eigen::Vector3d positionVarianceRos = poseCovarianceRos.block<3, 3>(0, 0).diagonal();
  Eigen::Vector3d orientationVarianceRos = poseCovarianceRos.block<3, 3>(3, 3).diagonal();

  std::thread publishNonTimeCriticalDataThread(&GraphMsfRos2::publishNonTimeCriticalData, this, poseCovarianceRos,
                                               twistCovarianceRos, positionVarianceRos, orientationVarianceRos,
                                               integratedNavStatePtr, optimizedStateWithCovarianceAndBiasPtr);
  publishNonTimeCriticalDataThread.detach();
}

void GraphMsfRos2::publishNonTimeCriticalData(
    const Eigen::Matrix<double, 6, 6> poseCovarianceRos, const Eigen::Matrix<double, 6, 6> twistCovarianceRos,
    const Eigen::Vector3d positionVarianceRos, const Eigen::Vector3d orientationVarianceRos,
    const std::shared_ptr<const graph_msf::SafeIntegratedNavState> integratedNavStatePtr,
    const std::shared_ptr<const graph_msf::SafeNavStateWithCovarianceAndBias> optimizedStateWithCovarianceAndBiasPtr) {
  std::lock_guard<std::mutex> lock(rosPublisherMutex_);

  double timeK = integratedNavStatePtr->getTimeK();

  publishImuOdoms(integratedNavStatePtr, poseCovarianceRos, twistCovarianceRos);
  publishDiagVarianceVectors(positionVarianceRos, orientationVarianceRos, timeK);
  publishVelocityMarkers(integratedNavStatePtr);
  publishImuPaths(integratedNavStatePtr);
  publishOptimizedStateAndBias(optimizedStateWithCovarianceAndBiasPtr, poseCovarianceRos, twistCovarianceRos);
}

}  // namespace graph_msf
