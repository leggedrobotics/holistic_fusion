// Implementation
#include "graph_msf_ros2/GraphMsfRos2.h"

// ROS2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_srvs/srv/trigger.hpp>

// Workspace
#include "graph_msf_ros2/util/conversions.h"

namespace graph_msf {

GraphMsfRos2::GraphMsfRos2(const std::string& nodeName, const rclcpp::NodeOptions& options) : Node(nodeName, options) {
  RCLCPP_INFO(this->get_logger(), "GraphMsfRos2-Constructor called.");

  // Configurations ----------------------------
  // Graph Config
  graphConfigPtr_ = std::make_shared<GraphConfig>();

  // Start non-time-critical thread for paths, variances, markers
  nonTimeCriticalThread_ = std::thread(&GraphMsfRos2::nonTimeCriticalThreadFunction, this);

  // Start IMU odometry thread for time-critical publishing
  imuOdomThread_ = std::thread(&GraphMsfRos2::imuOdomThreadFunction, this);

  // Start TF transforms thread for time-critical publishing
  tfTransformThread_ = std::thread(&GraphMsfRos2::tfTransformThreadFunction, this);
}

GraphMsfRos2::~GraphMsfRos2() {
  RCLCPP_INFO(this->get_logger(), "GraphMsfRos2-Destructor called.");

  // Signal all threads to shutdown
  shutdownRequested_ = true;
  nonTimeCriticalQueueCondition_.notify_all();
  imuOdomQueueCondition_.notify_all();
  tfTransformQueueCondition_.notify_all();

  // Wait for non-time-critical thread to finish
  if (nonTimeCriticalThread_.joinable()) {
    nonTimeCriticalThread_.join();
  }

  // Wait for IMU odometry thread to finish
  if (imuOdomThread_.joinable()) {
    imuOdomThread_.join();
  }

  // Wait for TF transform thread to finish
  if (tfTransformThread_.joinable()) {
    tfTransformThread_.join();
  }
}

void GraphMsfRos2::setup(std::shared_ptr<StaticTransforms> staticTransformsPtr) {
  RCLCPP_INFO(this->get_logger(), "GraphMsfRos2-Setup called.");

  // Check
  if (staticTransformsPtr == nullptr) {
    throw std::runtime_error("Static transforms not set. Has to be set.");
  }

  // Clock
  clock_ = this->get_clock();

  // Declare ROS parameters
  GraphMsfRos2::declareRosParams();

  // Read parameters ----------------------------
  GraphMsfRos2::readParams();

  // Call super class Setup ----------------------------
  GraphMsf::setup(graphConfigPtr_, staticTransformsPtr);

  // Publishers ----------------------------
  GraphMsfRos2::initializePublishers();

  // Subscribers ----------------------------
  GraphMsfRos2::initializeSubscribers();

  // Messages ----------------------------
  GraphMsfRos2::initializeMessages();

  tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // Services ----------------------------
  GraphMsfRos2::initializeServices(*this);

  // Time
  startTime = std::chrono::high_resolution_clock::now();

  // Wrap up
  RCLCPP_INFO(this->get_logger(), "Set up successfully.");
}

void GraphMsfRos2::initializePublishers() {
  RCLCPP_INFO(this->get_logger(), "Initializing publishers.");

  // Odometry
  pubEstOdomImu_ = this->create_publisher<nav_msgs::msg::Odometry>("/graph_msf/est_odometry_odom_imu", ROS_QUEUE_SIZE);
  pubEstWorldImu_ = this->create_publisher<nav_msgs::msg::Odometry>("/graph_msf/est_odometry_world_imu", ROS_QUEUE_SIZE);
  pubOptWorldImu_ = this->create_publisher<nav_msgs::msg::Odometry>("/graph_msf/opt_odometry_world_imu", ROS_QUEUE_SIZE);

  // Non-time-critical publishers with best-effort and larger queue
  auto best_effort_qos = rclcpp::QoS(100)
                             .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                             .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE)
                             .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

  // Vector3 Variances
  pubEstWorldPosVariance_ =
      this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/graph_msf/est_world_pos_variance", best_effort_qos);
  pubEstWorldRotVariance_ =
      this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/graph_msf/est_world_rot_variance", best_effort_qos);

  // Velocity Marker
  pubLinVelocityMarker_ = this->create_publisher<visualization_msgs::msg::Marker>("/graph_msf/lin_velocity_marker", best_effort_qos);
  pubAngularVelocityMarker_ = this->create_publisher<visualization_msgs::msg::Marker>("/graph_msf/angular_velocity_marker", best_effort_qos);

  // Paths
  pubEstOdomImuPath_ = this->create_publisher<nav_msgs::msg::Path>("/graph_msf/est_path_odom_imu", best_effort_qos);
  pubEstWorldImuPath_ = this->create_publisher<nav_msgs::msg::Path>("/graph_msf/est_path_world_imu", best_effort_qos);
  pubOptWorldImuPath_ = this->create_publisher<nav_msgs::msg::Path>("/graph_msf/opt_path_world_imu", best_effort_qos);

  // Imu Bias
  pubAccelBias_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/graph_msf/accel_bias", best_effort_qos);
  pubGyroBias_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/graph_msf/gyro_bias", best_effort_qos);
}

void GraphMsfRos2::initializeSubscribers() {
  RCLCPP_INFO(this->get_logger(), "Initializing subscribers.");

  // Imu
  subImu_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu_topic", rclcpp::QoS(ROS_QUEUE_SIZE).best_effort(),
                                                             std::bind(&GraphMsfRos2::imuCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "GraphMsfRos2 Initialized main IMU subscriber with topic: %s", subImu_->get_topic_name());
}

void GraphMsfRos2::initializeMessages() {
  RCLCPP_INFO(this->get_logger(), "Initializing messages.");

  // Odometry
  estOdomImuMsgPtr_ = std::make_shared<nav_msgs::msg::Odometry>();
  estWorldImuMsgPtr_ = std::make_shared<nav_msgs::msg::Odometry>();
  optWorldImuMsgPtr_ = std::make_shared<nav_msgs::msg::Odometry>();

  // Vector3 Variances
  estWorldPosVarianceMsgPtr_ = std::make_shared<geometry_msgs::msg::Vector3Stamped>();
  estWorldRotVarianceMsgPtr_ = std::make_shared<geometry_msgs::msg::Vector3Stamped>();

  // Path
  estOdomImuPathPtr_ = std::make_shared<nav_msgs::msg::Path>();
  estWorldImuPathPtr_ = std::make_shared<nav_msgs::msg::Path>();
  optWorldImuPathPtr_ = std::make_shared<nav_msgs::msg::Path>();

  // Imu Bias
  accelBiasMsgPtr_ = std::make_shared<geometry_msgs::msg::Vector3Stamped>();
  gyroBiasMsgPtr_ = std::make_shared<geometry_msgs::msg::Vector3Stamped>();
}

void GraphMsfRos2::initializeServices(rclcpp::Node& node) {
  RCLCPP_INFO(node.get_logger(), "Initializing services.");

  // Trigger offline smoother optimization
  srvSmootherOptimize_ = node.create_service<std_srvs::srv::Trigger>(
      "/graph_msf/trigger_offline_optimization",
      std::bind(&GraphMsfRos2::srvOfflineSmootherOptimizeCallback, this, std::placeholders::_1, std::placeholders::_2));
}

bool GraphMsfRos2::srvOfflineSmootherOptimizeCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                                      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
  // Hardcode max iterations (you can make this configurable via parameter if needed)
  int maxIterations = 100;

  // Trigger offline smoother optimization and create response
  if (optimizeSlowBatchSmoother(maxIterations, optimizationResultLoggingPath, false)) {
    res->success = true;
    res->message = "Optimization successful.";
  } else {
    res->success = false;
    res->message = "Optimization failed.";
  }
  return true;
}

void GraphMsfRos2::addToPathMsg(const nav_msgs::msg::Path::SharedPtr& pathPtr, const std::string& fixedFrameName, const rclcpp::Time& stamp,
                                const Eigen::Vector3d& t, const int maxBufferLength) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = fixedFrameName;
  pose.header.stamp = stamp;
  pose.pose.position.x = t(0);
  pose.pose.position.y = t(1);
  pose.pose.position.z = t(2);
  pathPtr->header.frame_id = fixedFrameName;
  pathPtr->header.stamp = stamp;
  pathPtr->poses.push_back(pose);
  if (pathPtr->poses.size() > maxBufferLength) {
    pathPtr->poses.erase(pathPtr->poses.begin());
  }
}

void GraphMsfRos2::addToOdometryMsg(const nav_msgs::msg::Odometry::SharedPtr& msgPtr, const std::string& fixedFrame,
                                    const std::string& movingFrame, const rclcpp::Time& stamp, const Eigen::Isometry3d& T,
                                    const Eigen::Vector3d& F_v_W_F, const Eigen::Vector3d& F_w_W_F,
                                    const Eigen::Matrix<double, 6, 6>& poseCovariance, const Eigen::Matrix<double, 6, 6>& twistCovariance) {
  msgPtr->header.frame_id = fixedFrame;
  msgPtr->child_frame_id = movingFrame;
  msgPtr->header.stamp = stamp;

  // Convert Eigen::Isometry3d to geometry_msgs::msg::Pose using tf2
  geometry_msgs::msg::Pose pose_msg;
  tf2::convert(T, pose_msg);
  msgPtr->pose.pose = pose_msg;

  msgPtr->twist.twist.linear.x = F_v_W_F(0);
  msgPtr->twist.twist.linear.y = F_v_W_F(1);
  msgPtr->twist.twist.linear.z = F_v_W_F(2);
  msgPtr->twist.twist.angular.x = F_w_W_F(0);
  msgPtr->twist.twist.angular.y = F_w_W_F(1);
  msgPtr->twist.twist.angular.z = F_w_W_F(2);

  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      msgPtr->pose.covariance[6 * i + j] = poseCovariance(i, j);
      msgPtr->twist.covariance[6 * i + j] = twistCovariance(i, j);
    }
  }
}

void GraphMsfRos2::extractCovariancesFromOptimizedState(
    Eigen::Matrix<double, 6, 6>& poseCovarianceRos, Eigen::Matrix<double, 6, 6>& twistCovarianceRos,
    const std::shared_ptr<graph_msf::SafeNavStateWithCovarianceAndBias>& optimizedStateWithCovarianceAndBiasPtr) {
  // Extract covariances from optimized state
  if (optimizedStateWithCovarianceAndBiasPtr != nullptr) {
    poseCovarianceRos =
        graph_msf::convertCovarianceGtsamConventionToRosConvention(optimizedStateWithCovarianceAndBiasPtr->getPoseCovariance());
    twistCovarianceRos.block<3, 3>(0, 0) = optimizedStateWithCovarianceAndBiasPtr->getVelocityCovariance();
  } else {
    poseCovarianceRos.setZero();
    twistCovarianceRos.setZero();
  }
}

// Markers
void GraphMsfRos2::createLinVelocityMarker(const std::string& referenceFrameName, const rclcpp::Time& stamp, const Eigen::Vector3d& velocity,
                                           visualization_msgs::msg::Marker& marker) {
  // Arrow
  marker.header.frame_id = referenceFrameName;
  marker.header.stamp = stamp;
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // Scale and Color
  marker.scale.x = 0.1;  // shaft diameter
  marker.scale.y = 0.2;  // head diameter
  marker.scale.z = 0.2;  // head length
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;

  // Define Arrow through start and end point
  geometry_msgs::msg::Point startPoint, endPoint;
  startPoint.x = 0.0;  // origin
  startPoint.y = 0.0;  // origin
  startPoint.z = 0.0;  // 0 meter above origin
  endPoint.x = startPoint.x + velocity(0);
  endPoint.y = startPoint.y + velocity(1);
  endPoint.z = startPoint.z + velocity(2);
  marker.points.push_back(startPoint);
  marker.points.push_back(endPoint);

  // Quaternion for orientation
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  marker.pose.orientation.x = q.x();
  marker.pose.orientation.y = q.y();
  marker.pose.orientation.z = q.z();
  marker.pose.orientation.w = q.w();
}

void GraphMsfRos2::createAngularVelocityMarker(const std::string& referenceFrameName, const rclcpp::Time& stamp, 
                                               const Eigen::Vector3d& angularVelocity, const Eigen::Isometry3d& currentPose,
                                               visualization_msgs::msg::Marker& marker) {
  // Cylinder to visualize angular velocity as a disc/ring oriented along rotation axis
  marker.header.frame_id = referenceFrameName;
  marker.header.stamp = stamp;
  marker.ns = "angular_velocity";
  marker.id = 1;
  marker.type = visualization_msgs::msg::Marker::CYLINDER;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // Angular velocity magnitude
  double angularMagnitude = angularVelocity.norm();
  
  if (angularMagnitude > 1e-6) {  // Only create marker if there's significant angular velocity
    // Scale based on angular velocity magnitude
    double baseRadius = std::min(std::max(angularMagnitude * 0.2, 0.1), 0.5);
    marker.scale.x = baseRadius * 2.0;  // diameter in x
    marker.scale.y = baseRadius * 2.0;  // diameter in y  
    marker.scale.z = 0.02;              // thin disc height
    
    // Color: blue for angular velocity with alpha based on magnitude
    marker.color.a = std::min(angularMagnitude * 0.5 + 0.3, 1.0);
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
  } else {
    // No significant angular velocity - make marker invisible
    marker.scale.x = 0.0;
    marker.scale.y = 0.0;
    marker.scale.z = 0.0;
    marker.color.a = 0.0;
  }

  // Set lifetime
  marker.lifetime = rclcpp::Duration::from_seconds(0.1);

  // Position at current pose position
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;

  // Orient the disc perpendicular to the angular velocity vector (rotation axis)
  if (angularMagnitude > 1e-6) {
    Eigen::Vector3d rotationAxis = angularVelocity.normalized();
    
    // Create a rotation that aligns the cylinder's z-axis with the rotation axis
    // Default cylinder orientation is along z-axis
    Eigen::Vector3d zAxis(0, 0, 1);
    
    // Calculate rotation to align z-axis with rotation axis
    Eigen::Quaterniond orientation;
    if (rotationAxis.dot(zAxis) > 0.9999) {
      // Already aligned
      orientation = Eigen::Quaterniond::Identity();
    } else if (rotationAxis.dot(zAxis) < -0.9999) {
      // Opposite direction - rotate 180 degrees around x-axis
      orientation = Eigen::Quaterniond(0, 1, 0, 0);
    } else {
      // General case - use cross product to find rotation axis
      Eigen::Vector3d rotAxis = zAxis.cross(rotationAxis).normalized();
      double angle = std::acos(std::max(-1.0, std::min(1.0, zAxis.dot(rotationAxis))));
      orientation = Eigen::Quaterniond(Eigen::AngleAxisd(angle, rotAxis));
    }
    
    marker.pose.orientation.x = orientation.x();
    marker.pose.orientation.y = orientation.y();
    marker.pose.orientation.z = orientation.z();
    marker.pose.orientation.w = orientation.w();
  } else {
    // Default orientation
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
  }
}

void GraphMsfRos2::createContactMarker(const std::string& referenceFrameName, const rclcpp::Time& stamp, const Eigen::Vector3d& position,
                                       const std::string& nameSpace, const int id, visualization_msgs::msg::Marker& marker) {
  // Sphere
  marker.header.frame_id = referenceFrameName;
  marker.header.stamp = stamp;
  marker.ns = nameSpace;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // Scale and Color
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  // Position
  marker.pose.position.x = position(0);
  marker.pose.position.y = position(1);
  marker.pose.position.z = position(2);

  // Orientation
  marker.pose.orientation.w = 1.0;
}

long GraphMsfRos2::secondsSinceStart() {
  currentTime = std::chrono::high_resolution_clock::now();
  return std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime).count();
}

// Main IMU Callback
void GraphMsfRos2::imuCallback(const sensor_msgs::msg::Imu::SharedPtr imuMsgPtr) {
  // Convert to Eigen
  Eigen::Vector3d linearAcc(imuMsgPtr->linear_acceleration.x, imuMsgPtr->linear_acceleration.y, imuMsgPtr->linear_acceleration.z);
  Eigen::Vector3d angularVel(imuMsgPtr->angular_velocity.x, imuMsgPtr->angular_velocity.y, imuMsgPtr->angular_velocity.z);
  Eigen::Matrix<double, 6, 1> addedImuMeasurements;  // accel, gyro

  // Create pointer for carrying state
  std::shared_ptr<SafeIntegratedNavState> preIntegratedNavStatePtr = nullptr;
  std::shared_ptr<SafeNavStateWithCovarianceAndBias> optimizedStateWithCovarianceAndBiasPtr = nullptr;

  // Add measurement and get state
  if (GraphMsf::addCoreImuMeasurementAndGetState(linearAcc, angularVel,
                                                 imuMsgPtr->header.stamp.sec + 1e-9 * imuMsgPtr->header.stamp.nanosec,
                                                 preIntegratedNavStatePtr, optimizedStateWithCovarianceAndBiasPtr, addedImuMeasurements)) {
    // Encountered Delay
    auto now = clock_->now();
    auto delay = now.seconds() - preIntegratedNavStatePtr->getTimeK();
    if (delay > 0.5) {
      RCLCPP_WARN(this->get_logger(), "Encountered delay of %.14f seconds.", delay);
      // Print now vs chrono time
      auto currentChronoTime = std::chrono::high_resolution_clock::now();
      auto chronoTimeSinceEpoch = std::chrono::duration_cast<std::chrono::duration<double>>(currentChronoTime.time_since_epoch()).count();
      RCLCPP_WARN(this->get_logger(), "Now: %.14f, Std chrono time: %.14f", now.seconds(), chronoTimeSinceEpoch);
    }

    // Publish Odometry
    this->publishState(preIntegratedNavStatePtr, optimizedStateWithCovarianceAndBiasPtr);
  } else if (GraphMsf::isGraphInited()) {
    RCLCPP_WARN(this->get_logger(), "Could not add IMU measurement.");
  }
}

// Publish state ---------------------------------------------------------------
// Higher Level Functions
void GraphMsfRos2::publishState(
    const std::shared_ptr<graph_msf::SafeIntegratedNavState>& integratedNavStatePtr,
    const std::shared_ptr<graph_msf::SafeNavStateWithCovarianceAndBias>& optimizedStateWithCovarianceAndBiasPtr) {
  // Covariances
  Eigen::Matrix<double, 6, 6> poseCovarianceRos, twistCovarianceRos;
  graph_msf::GraphMsfRos2::extractCovariancesFromOptimizedState(poseCovarianceRos, twistCovarianceRos,
                                                                optimizedStateWithCovarianceAndBiasPtr);

  // Time
  const double timeK = integratedNavStatePtr->getTimeK();

  // Queue IMU odometry data for separate thread processing
  {
    std::lock_guard<std::mutex> lock(imuOdomQueueMutex_);
    imuOdomQueue_.emplace(integratedNavStatePtr, poseCovarianceRos, twistCovarianceRos);
  }
  imuOdomQueueCondition_.notify_one();

  // Queue TF transforms data for separate thread processing
  {
    std::lock_guard<std::mutex> lock(tfTransformQueueMutex_);
    tfTransformQueue_.emplace(integratedNavStatePtr, optimizedStateWithCovarianceAndBiasPtr, timeK);
  }
  tfTransformQueueCondition_.notify_one();

  // Variances (only diagonal elements)
  Eigen::Vector3d positionVarianceRos = poseCovarianceRos.block<3, 3>(0, 0).diagonal();
  Eigen::Vector3d orientationVarianceRos = poseCovarianceRos.block<3, 3>(3, 3).diagonal();

  // Add path data at full rate (but don't publish yet)
  addToPathMsg(estOdomImuPathPtr_, staticTransformsPtr_->getOdomFrame(), rclcpp::Time(timeK * 1e9),
               integratedNavStatePtr->getT_O_Ik_gravityAligned().translation(), graphConfigPtr_->imuBufferLength_ * 4);
  addToPathMsg(estWorldImuPathPtr_, staticTransformsPtr_->getWorldFrame(), rclcpp::Time(timeK * 1e9),
               integratedNavStatePtr->getT_W_Ik().translation(), graphConfigPtr_->imuBufferLength_ * 4);

  // Add optimized path data at full rate if available
  if (optimizedStateWithCovarianceAndBiasPtr != nullptr &&
      optimizedStateWithCovarianceAndBiasPtr->getTimeK() - lastOptimizedStateTimestamp_ > 1e-03) {
    addToPathMsg(optWorldImuPathPtr_, staticTransformsPtr_->getWorldFrame(),
                 rclcpp::Time(optimizedStateWithCovarianceAndBiasPtr->getTimeK() * 1e9),
                 optimizedStateWithCovarianceAndBiasPtr->getT_W_Ik().translation(), graphConfigPtr_->imuBufferLength_ * 20);

    // Queue non-time critical data for non-time-critical thread processing
    {
      std::lock_guard<std::mutex> lock(nonTimeCriticalQueueMutex_);
      nonTimeCriticalQueue_.emplace(poseCovarianceRos, twistCovarianceRos, positionVarianceRos, orientationVarianceRos,
                                    optimizedStateWithCovarianceAndBiasPtr);
    }
    nonTimeCriticalQueueCondition_.notify_one();
    lastOptimizedStateTimestamp_ = optimizedStateWithCovarianceAndBiasPtr->getTimeK();
  }
}

// Publish to TF
void GraphMsfRos2::publishTfTransforms(const std::shared_ptr<graph_msf::SafeIntegratedNavState>& integratedNavStatePtr) {
  // Time
  const double& timeK = integratedNavStatePtr->getTimeK();  // Alias

  // B_O
  Eigen::Isometry3d T_B_Ok =
      staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getBaseLinkFrame(), staticTransformsPtr_->getImuFrame()) *
      integratedNavStatePtr->getT_O_Ik_gravityAligned().inverse();
  publishTfTreeTransform(staticTransformsPtr_->getBaseLinkFrame(), staticTransformsPtr_->getOdomFrame(), timeK, T_B_Ok);
  // O_W
  Eigen::Isometry3d T_O_W = integratedNavStatePtr->getT_W_O().inverse();
  publishTfTreeTransform(staticTransformsPtr_->getOdomFrame(), staticTransformsPtr_->getWorldFrame(), timeK, T_O_W);

  // Publish Velocity Markers
  publishVelocityMarkers(integratedNavStatePtr);
}

// Copy the arguments in order to be thread safe
void GraphMsfRos2::publishNonTimeCriticalData(
    const Eigen::Matrix<double, 6, 6> poseCovarianceRos, const Eigen::Matrix<double, 6, 6> twistCovarianceRos,
    const Eigen::Vector3d positionVarianceRos, const Eigen::Vector3d orientationVarianceRos,
    const std::shared_ptr<const graph_msf::SafeNavStateWithCovarianceAndBias> optimizedStateWithCovarianceAndBiasPtr) {
  // Mutex for not overloading ROS
  std::lock_guard<std::mutex> lock(rosPublisherMutex_);

  // Publish Variances
  publishDiagVarianceVectors(positionVarianceRos, orientationVarianceRos, optimizedStateWithCovarianceAndBiasPtr->getTimeK());

  // Publish paths (data was already added at full rate in publishState)
  publishImuPaths();

  // Publish optimized path (data was already added at full rate in publishState)
  if (pubOptWorldImuPath_->get_subscription_count() > 0) {
    pubOptWorldImuPath_->publish(*optWorldImuPathPtr_);
  }

  // Optimized estimate ----------------------
  publishOptimizedStateAndBias(optimizedStateWithCovarianceAndBiasPtr, poseCovarianceRos, twistCovarianceRos);
}

void GraphMsfRos2::publishOptimizedStateAndBias(
    const std::shared_ptr<const graph_msf::SafeNavStateWithCovarianceAndBias> optimizedStateWithCovarianceAndBiasPtr,
    const Eigen::Matrix<double, 6, 6>& poseCovarianceRos, const Eigen::Matrix<double, 6, 6>& twistCovarianceRos) {
  // A: Publish Odometry and IMU Biases at update rate
  if (optimizedStateWithCovarianceAndBiasPtr != nullptr) {
    // Odometry messages
    // world->imu
    if (pubOptWorldImu_->get_subscription_count() > 0) {
      addToOdometryMsg(optWorldImuMsgPtr_, staticTransformsPtr_->getWorldFrame(), staticTransformsPtr_->getImuFrame(),
                       rclcpp::Time(optimizedStateWithCovarianceAndBiasPtr->getTimeK() * 1e9),
                       optimizedStateWithCovarianceAndBiasPtr->getT_W_Ik(), optimizedStateWithCovarianceAndBiasPtr->getI_v_W_I(),
                       optimizedStateWithCovarianceAndBiasPtr->getI_w_W_I(), poseCovarianceRos, twistCovarianceRos);
      pubOptWorldImu_->publish(*optWorldImuMsgPtr_);
    }

    // Biases
    // Publish accel bias
    if (pubAccelBias_->get_subscription_count() > 0) {
      Eigen::Vector3d accelBias = optimizedStateWithCovarianceAndBiasPtr->getAccelerometerBias();
      accelBiasMsgPtr_->header.stamp = rclcpp::Time(optimizedStateWithCovarianceAndBiasPtr->getTimeK() * 1e9);
      accelBiasMsgPtr_->header.frame_id = staticTransformsPtr_->getImuFrame();
      accelBiasMsgPtr_->vector.x = accelBias(0);
      accelBiasMsgPtr_->vector.y = accelBias(1);
      accelBiasMsgPtr_->vector.z = accelBias(2);
      pubAccelBias_->publish(*accelBiasMsgPtr_);
    }
    // Publish gyro bias
    if (pubGyroBias_->get_subscription_count() > 0) {
      Eigen::Vector3d gyroBias = optimizedStateWithCovarianceAndBiasPtr->getGyroscopeBias();
      gyroBiasMsgPtr_->header.stamp = rclcpp::Time(optimizedStateWithCovarianceAndBiasPtr->getTimeK() * 1e9);
      gyroBiasMsgPtr_->header.frame_id = staticTransformsPtr_->getImuFrame();
      gyroBiasMsgPtr_->vector.x = gyroBias(0);
      gyroBiasMsgPtr_->vector.y = gyroBias(1);
      gyroBiasMsgPtr_->vector.z = gyroBias(2);
      pubGyroBias_->publish(*gyroBiasMsgPtr_);
    }
  }  // Publishing odometry and biases
}

// Publish TF transforms for optimized state at full rate
void GraphMsfRos2::publishOptimizedStateTfTransforms(
    const std::shared_ptr<const graph_msf::SafeNavStateWithCovarianceAndBias> optimizedStateWithCovarianceAndBiasPtr,
    const double timeStamp) {
  // Publish Transforms at imu rate
  if (optimizedStateWithCovarianceAndBiasPtr != nullptr && timeStamp - lastIntegratedStateTimestamp_ > 1e-03) {
    lastIntegratedStateTimestamp_ = timeStamp;
    // TFs in Optimized State
    for (const auto& framePairTransformMapIterator :
         optimizedStateWithCovarianceAndBiasPtr->getReferenceFrameTransforms().getTransformsMap()) {
      // Case 1: If includes world frame --> everything child of world
      if (framePairTransformMapIterator.first.first == staticTransformsPtr_->getWorldFrame() ||
          framePairTransformMapIterator.first.second == staticTransformsPtr_->getWorldFrame()) {
        // Get transform
        Eigen::Isometry3d T_W_M;
        std::string mapFrameName;
        const std::string& worldFrameName = staticTransformsPtr_->getWorldFrame();

        // If world is second, then map is first
        if (framePairTransformMapIterator.first.second == worldFrameName) {
          T_W_M = framePairTransformMapIterator.second.inverse();
          mapFrameName = framePairTransformMapIterator.first.first;
        }
        // If world is second, then map is first, this is the case for holistic alignment
        else {
          T_W_M = framePairTransformMapIterator.second;
          mapFrameName = framePairTransformMapIterator.first.second;
        }

        // Publish TF Tree only (removed PoseWithCovarianceStamped)
        publishTfTreeTransform(worldFrameName, mapFrameName + referenceFrameAlignedNameId, timeStamp, T_W_M);
      }
      // Case 2: Other transformations
      else {
        const std::string& sensorFrameName = framePairTransformMapIterator.first.first;
        const std::string& sensorFrameNameCorrected = framePairTransformMapIterator.first.second;
        const Eigen::Isometry3d& T_sensor_sensorCorrected = framePairTransformMapIterator.second;

        // Publish TF Tree only (removed PoseWithCovarianceStamped)
        publishTfTreeTransform(sensorFrameName, sensorFrameNameCorrected, timeStamp, T_sensor_sensorCorrected);
      }
    }  // for each frame pair transform
  }    // Publishing of Transforms
}

// Lower Level Functions
void GraphMsfRos2::publishTfTreeTransform(const std::string& parentFrameName, const std::string& childFrameName, const double timeStamp,
                                          const Eigen::Isometry3d& T_frame_childFrame) const {
  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.stamp = rclcpp::Time(timeStamp * 1e9);
  transformStamped.header.frame_id = parentFrameName;
  transformStamped.child_frame_id = childFrameName;
  transformStamped.transform.translation.x = T_frame_childFrame.translation().x();
  transformStamped.transform.translation.y = T_frame_childFrame.translation().y();
  transformStamped.transform.translation.z = T_frame_childFrame.translation().z();
  Eigen::Quaterniond q(T_frame_childFrame.rotation());
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  // Convert Eigen::Isometry3d to geometry_msgs::msg::Transform
  //   tf2::convert(T_frame_childFrame, transformStamped.transform);

  tfBroadcaster_->sendTransform(transformStamped);
}

void GraphMsfRos2::publishImuOdoms(const std::shared_ptr<const graph_msf::SafeIntegratedNavState>& preIntegratedNavStatePtr,
                                   const Eigen::Matrix<double, 6, 6>& poseCovarianceRos,
                                   const Eigen::Matrix<double, 6, 6>& twistCovarianceRos) const {
  // Odom->imu
  if (pubEstOdomImu_->get_subscription_count() > 0) {
    addToOdometryMsg(estOdomImuMsgPtr_, staticTransformsPtr_->getOdomFrame(), staticTransformsPtr_->getImuFrame(),
                     rclcpp::Time(preIntegratedNavStatePtr->getTimeK() * 1e9), preIntegratedNavStatePtr->getT_O_Ik_gravityAligned(),
                     preIntegratedNavStatePtr->getI_v_W_I(), preIntegratedNavStatePtr->getI_w_W_I(), poseCovarianceRos, twistCovarianceRos);
    pubEstOdomImu_->publish(*estOdomImuMsgPtr_);
  }
  // World->imu
  if (pubEstWorldImu_->get_subscription_count() > 0) {
    addToOdometryMsg(estWorldImuMsgPtr_, staticTransformsPtr_->getWorldFrame(), staticTransformsPtr_->getImuFrame(),
                     rclcpp::Time(preIntegratedNavStatePtr->getTimeK() * 1e9), preIntegratedNavStatePtr->getT_W_Ik(),
                     preIntegratedNavStatePtr->getI_v_W_I(), preIntegratedNavStatePtr->getI_w_W_I(), poseCovarianceRos, twistCovarianceRos);
    pubEstWorldImu_->publish(*estWorldImuMsgPtr_);
  }
}

void GraphMsfRos2::publishDiagVarianceVectors(const Eigen::Vector3d& posVarianceRos, const Eigen::Vector3d& rotVarianceRos,
                                              const double timeStamp) const {
  // World Position Variance
  if (pubEstWorldPosVariance_->get_subscription_count() > 0) {
    estWorldPosVarianceMsgPtr_->header.stamp = rclcpp::Time(timeStamp * 1e9);
    estWorldPosVarianceMsgPtr_->header.frame_id = staticTransformsPtr_->getWorldFrame();
    estWorldPosVarianceMsgPtr_->vector.x = posVarianceRos(0);
    estWorldPosVarianceMsgPtr_->vector.y = posVarianceRos(1);
    estWorldPosVarianceMsgPtr_->vector.z = posVarianceRos(2);
    pubEstWorldPosVariance_->publish(*estWorldPosVarianceMsgPtr_);
  }
  // World Rotation Variance
  if (pubEstWorldRotVariance_->get_subscription_count() > 0) {
    estWorldRotVarianceMsgPtr_->header.stamp = rclcpp::Time(timeStamp * 1e9);
    estWorldRotVarianceMsgPtr_->header.frame_id = staticTransformsPtr_->getWorldFrame();
    estWorldRotVarianceMsgPtr_->vector.x = rotVarianceRos(0);
    estWorldRotVarianceMsgPtr_->vector.y = rotVarianceRos(1);
    estWorldRotVarianceMsgPtr_->vector.z = rotVarianceRos(2);
    pubEstWorldRotVariance_->publish(*estWorldRotVarianceMsgPtr_);
  }
}

void GraphMsfRos2::publishVelocityMarkers(const std::shared_ptr<const graph_msf::SafeIntegratedNavState>& navStatePtr) const {
  // Linear Velocity Marker
  visualization_msgs::msg::Marker velocityMarker;
  createLinVelocityMarker(staticTransformsPtr_->getImuFrame(), rclcpp::Time(navStatePtr->getTimeK() * 1e9), navStatePtr->getI_v_W_I(),
                          velocityMarker);

  // Angular Velocity Marker
  visualization_msgs::msg::Marker angularVelocityMarker;
  createAngularVelocityMarker(staticTransformsPtr_->getImuFrame(), rclcpp::Time(navStatePtr->getTimeK() * 1e9), navStatePtr->getI_w_W_I(),
                              navStatePtr->getT_W_Ik(), angularVelocityMarker);

  // Publish both markers
  if (pubLinVelocityMarker_->get_subscription_count() > 0) {
    pubLinVelocityMarker_->publish(velocityMarker);
  }
  if (pubAngularVelocityMarker_->get_subscription_count() > 0) {
    pubAngularVelocityMarker_->publish(angularVelocityMarker);
  }
}

void GraphMsfRos2::publishImuPaths() const {
  // Only publish the accumulated path data (data was already added in publishState at full rate)
  // odom->imu
  if (pubEstOdomImuPath_->get_subscription_count() > 0) {
    pubEstOdomImuPath_->publish(*estOdomImuPathPtr_);
  }
  // world->imu
  if (pubEstWorldImuPath_->get_subscription_count() > 0) {
    pubEstWorldImuPath_->publish(*estWorldImuPathPtr_);
  }
}

void GraphMsfRos2::nonTimeCriticalThreadFunction() {
  // Rate limiting variables
  while (!shutdownRequested_) {
    std::unique_lock<std::mutex> lock(nonTimeCriticalQueueMutex_);

    // Wait for work or shutdown signal
    nonTimeCriticalQueueCondition_.wait(lock, [this] { return !nonTimeCriticalQueue_.empty() || shutdownRequested_; });

    // Process all queued work with rate limiting
    while (!nonTimeCriticalQueue_.empty() && !shutdownRequested_) {
      // Get the data from the queue
      NonTimeCriticalData data = std::move(nonTimeCriticalQueue_.front());
      nonTimeCriticalQueue_.pop();

      // Unlock while processing to avoid blocking the main thread
      lock.unlock();

      // Process the non-time-critical data
      publishNonTimeCriticalData(data.poseCovarianceRos, data.twistCovarianceRos, data.positionVarianceRos, data.orientationVarianceRos,
                                 data.optimizedStateWithCovarianceAndBiasPtr);

      // Re-lock for the next iteration
      lock.lock();
    }
  }
}

void GraphMsfRos2::imuOdomThreadFunction() {
  // Process IMU odometry publishing at high rate
  while (!shutdownRequested_) {
    std::unique_lock<std::mutex> lock(imuOdomQueueMutex_);

    // Wait for work or shutdown signal
    imuOdomQueueCondition_.wait(lock, [this] { return !imuOdomQueue_.empty() || shutdownRequested_; });

    // Process all queued IMU odometry data
    while (!imuOdomQueue_.empty() && !shutdownRequested_) {
      // Get the data from the queue
      ImuOdomData data = std::move(imuOdomQueue_.front());
      imuOdomQueue_.pop();

      // Unlock while processing to avoid blocking the main thread
      lock.unlock();

      // Publish IMU odometry messages
      publishImuOdoms(data.integratedNavStatePtr, data.poseCovarianceRos, data.twistCovarianceRos);

      // Re-lock for the next iteration
      lock.lock();
    }
  }
}

void GraphMsfRos2::tfTransformThreadFunction() {
  // Process TF transforms publishing at high rate
  while (!shutdownRequested_) {
    std::unique_lock<std::mutex> lock(tfTransformQueueMutex_);

    // Wait for work or shutdown signal
    tfTransformQueueCondition_.wait(lock, [this] { return !tfTransformQueue_.empty() || shutdownRequested_; });

    // Process all queued TF transform data
    while (!tfTransformQueue_.empty() && !shutdownRequested_) {
      // Get the data from the queue
      TfTransformData data = std::move(tfTransformQueue_.front());
      tfTransformQueue_.pop();

      // Unlock while processing to avoid blocking the main thread
      lock.unlock();

      // Publish TF transforms
      publishTfTransforms(data.integratedNavStatePtr);
      publishOptimizedStateTfTransforms(data.optimizedStateWithCovarianceAndBiasPtr, data.timeK);

      // Re-lock for the next iteration
      lock.lock();
    }
  }
}

}  // namespace graph_msf
