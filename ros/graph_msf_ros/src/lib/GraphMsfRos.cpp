/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "graph_msf_ros/GraphMsfRos.h"

// ROS
#include <tf/tf.h>

// Workspace
#include "graph_msf_ros/constants.h"
#include "graph_msf_ros/util/conversions.h"

namespace graph_msf {

GraphMsfRos::GraphMsfRos(const std::shared_ptr<ros::NodeHandle>& privateNodePtr) : GraphMsf(), privateNode_(*privateNodePtr) {
  REGULAR_COUT << GREEN_START << " Initializing." << COLOR_END << std::endl;

  // Configurations ----------------------------
  graphConfigPtr_ = std::make_shared<GraphConfig>();
  staticTransformsPtr_ = std::make_shared<StaticTransforms>();

  // Wrap up
  REGULAR_COUT << GREEN_START << " Initialized." << COLOR_END << std::endl;
}

bool GraphMsfRos::setup() {
  REGULAR_COUT << GREEN_START << " Setting up." << COLOR_END << std::endl;

  // Read parameters ----------------------------
  GraphMsfRos::readParams_(privateNode_);

  // Super class
  if (not graph_msf::GraphMsf::setup()) {
    throw std::runtime_error("GraphMsfRos could not be initialized");
  }

  // Publishers ----------------------------
  GraphMsfRos::initializePublishers_(privateNode_);

  // Subscribers ----------------------------
  GraphMsfRos::initializeSubscribers_(privateNode_);

  // Messages ----------------------------
  GraphMsfRos::initializeMessages_(privateNode_);

  // Services ----------------------------
  GraphMsfRos::initializeServices_(privateNode_);

  // Time
  startTime_ = std::chrono::high_resolution_clock::now();

  // Wrap up
  REGULAR_COUT << GREEN_START << " Set up successfully." << COLOR_END << std::endl;

  return true;
}

void GraphMsfRos::initializePublishers_(ros::NodeHandle& privateNode) {
  REGULAR_COUT << GREEN_START << " Initializing publishers." << COLOR_END << std::endl;
  // Odometry
  pubEstOdomImu_ = privateNode.advertise<nav_msgs::Odometry>("/graph_msf/est_odometry_odom_imu", ROS_QUEUE_SIZE);
  pubEstMapImu_ = privateNode.advertise<nav_msgs::Odometry>("/graph_msf/est_odometry_map_imu", ROS_QUEUE_SIZE);
  pubEstWorldImu_ = privateNode.advertise<nav_msgs::Odometry>("/graph_msf/est_odometry_world_imu", ROS_QUEUE_SIZE);
  pubOptWorldImu_ = privateNode.advertise<nav_msgs::Odometry>("/graph_msf/opt_odometry_world_imu", ROS_QUEUE_SIZE);
  // Paths
  pubEstOdomImuPath_ = privateNode.advertise<nav_msgs::Path>("/graph_msf/est_path_odom_imu", ROS_QUEUE_SIZE);
  pubEstWorldImuPath_ = privateNode.advertise<nav_msgs::Path>("/graph_msf/est_path_world_imu", ROS_QUEUE_SIZE);
  pubOptWorldImuPath_ = privateNode.advertise<nav_msgs::Path>("/graph_msf/opt_path_world_imu", ROS_QUEUE_SIZE);
  // Imu Bias
  pubAccelBias_ = privateNode.advertise<geometry_msgs::Vector3Stamped>("/graph_msf/accel_bias", ROS_QUEUE_SIZE);
  pubGyroBias_ = privateNode.advertise<geometry_msgs::Vector3Stamped>("/graph_msf/gyro_bias", ROS_QUEUE_SIZE);
  // Added Imu Measurements
  pubAddedImuMeas_ = privateNode.advertise<sensor_msgs::Imu>("/graph_msf/added_imu_meas", ROS_QUEUE_SIZE);
}

void GraphMsfRos::initializeSubscribers_(ros::NodeHandle& privateNode) {
  REGULAR_COUT << GREEN_START << " Initializing subscribers." << COLOR_END << std::endl;
  // Imu
  subImu_ = privateNode.subscribe<sensor_msgs::Imu>("/imu_topic", ROS_QUEUE_SIZE, &GraphMsfRos::imuCallback_, this,
                                                    ros::TransportHints().tcpNoDelay());
  std::cout << YELLOW_START << "GraphMsfRos" << COLOR_END << " Initialized IMU cabin subscriber with topic: " << subImu_.getTopic()
            << std::endl;
}

void GraphMsfRos::initializeMessages_(ros::NodeHandle& privateNode) {
  REGULAR_COUT << GREEN_START << " Initializing messages." << COLOR_END << std::endl;
  // Odometry
  estOdomImuMsgPtr_ = boost::make_shared<nav_msgs::Odometry>();
  estMapImuMsgPtr_ = boost::make_shared<nav_msgs::Odometry>();
  estWorldImuMsgPtr_ = boost::make_shared<nav_msgs::Odometry>();
  optWorldImuMsgPtr_ = boost::make_shared<nav_msgs::Odometry>();
  // Path
  estOdomImuPathPtr_ = boost::make_shared<nav_msgs::Path>();
  estWorldImuPathPtr_ = boost::make_shared<nav_msgs::Path>();
  optWorldImuPathPtr_ = boost::make_shared<nav_msgs::Path>();
  // Imu Bias
  accelBiasMsgPtr_ = boost::make_shared<geometry_msgs::Vector3Stamped>();
  gyroBiasMsgPtr_ = boost::make_shared<geometry_msgs::Vector3Stamped>();
}

void GraphMsfRos::initializeServices_(ros::NodeHandle& privateNode) {
  // Trigger offline smoother optimization
  srvSmootherOptimize_ =
      privateNode.advertiseService("/graph_msf/trigger_offline_optimization", &GraphMsfRos::srvOfflineSmootherOptimizeCallback_, this);
}

void GraphMsfRos::imuCallback_(const sensor_msgs::Imu::ConstPtr& imuMsgPtr) {
  // Convert to Eigen
  Eigen::Vector3d linearAcc(imuMsgPtr->linear_acceleration.x, imuMsgPtr->linear_acceleration.y, imuMsgPtr->linear_acceleration.z);
  Eigen::Vector3d angularVel(imuMsgPtr->angular_velocity.x, imuMsgPtr->angular_velocity.y, imuMsgPtr->angular_velocity.z);
  Eigen::Matrix<double, 6, 1> addedImuMeasurements;  // accel, gyro

  // Create pointer for carrying state
  std::shared_ptr<SafeIntegratedNavState> preIntegratedNavStatePtr = nullptr;
  std::shared_ptr<SafeNavStateWithCovarianceAndBias> optimizedStateWithCovarianceAndBiasPtr = nullptr;
  // Add measurement and get state
  if (this->addImuMeasurementAndGetState(linearAcc, angularVel, imuMsgPtr->header.stamp.toSec(), preIntegratedNavStatePtr,
                                         optimizedStateWithCovarianceAndBiasPtr, addedImuMeasurements)) {
    // Encountered delay
    if (ros::Time::now() - ros::Time(preIntegratedNavStatePtr->getTimeK()) > ros::Duration(0.5)) {
      std::cout << RED_START << "GraphMsfRos" << COLOR_END << " Encountered delay of "
                << (ros::Time::now() - ros::Time(preIntegratedNavStatePtr->getTimeK())).toSec() << " seconds." << std::endl;
    }
    // Publish Odometry
    this->publishState_(preIntegratedNavStatePtr, optimizedStateWithCovarianceAndBiasPtr);

    // Publish Filtered Imu Measurements
    this->publishAddedImuMeas_(addedImuMeasurements, imuMsgPtr->header.stamp);
  }
}

bool GraphMsfRos::srvOfflineSmootherOptimizeCallback_(graph_msf_ros::OfflineOptimizationTrigger::Request& req,
                                                      graph_msf_ros::OfflineOptimizationTrigger::Response& res) {
  // Max Iterations from service call
  int maxIterations = req.max_optimization_iterations;

  // Trigger offline smoother optimization and create response
  if (GraphMsf::optimizeSlowBatchSmoother(maxIterations)) {
    res.success = true;
    res.message = "Optimization successful.";
  } else {
    res.success = false;
    res.message = "Optimization failed.";
  }
  return true;
}

void GraphMsfRos::addToPathMsg(const nav_msgs::PathPtr& pathPtr, const std::string& fixedFrameName, const ros::Time& stamp,
                               const Eigen::Vector3d& t, const int maxBufferLength) {
  geometry_msgs::PoseStamped pose;
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

void GraphMsfRos::addToOdometryMsg(const nav_msgs::OdometryPtr& msgPtr, const std::string& fixedFrame, const std::string& movingFrame,
                                   const ros::Time& stamp, const Eigen::Isometry3d& T, const Eigen::Vector3d& F_v_W_F,
                                   const Eigen::Vector3d& F_w_W_F, const Eigen::Matrix<double, 6, 6>& poseCovariance,
                                   const Eigen::Matrix<double, 6, 6>& twistCovariance) {
  msgPtr->header.frame_id = fixedFrame;
  msgPtr->child_frame_id = movingFrame;
  msgPtr->header.stamp = stamp;
  tf::poseTFToMsg(isometry3ToTf(T), msgPtr->pose.pose);
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

void GraphMsfRos::extractCovariancesFromOptimizedState(
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

void GraphMsfRos::publishTransform_(const std::string& parentFrameName, const std::string& childFrameName, const double timeStamp,
                                    const Eigen::Isometry3d& T_frame_childFrame) {
  tf::Transform transform_frame_childFrame = isometry3ToTf(T_frame_childFrame);
  tfBroadcaster_.sendTransform(tf::StampedTransform(transform_frame_childFrame, ros::Time(timeStamp), parentFrameName, childFrameName));
}

void GraphMsfRos::publishImuOdoms_(const std::shared_ptr<graph_msf::SafeIntegratedNavState>& preIntegratedNavStatePtr,
                                   const Eigen::Matrix<double, 6, 6>& poseCovarianceRos,
                                   const Eigen::Matrix<double, 6, 6>& twistCovarianceRos) const {
  // Odom->imu with 100 Hz
  addToOdometryMsg(estOdomImuMsgPtr_, staticTransformsPtr_->getOdomFrame(), staticTransformsPtr_->getImuFrame(),
                   ros::Time(preIntegratedNavStatePtr->getTimeK()), preIntegratedNavStatePtr->getT_O_Ik_gravityAligned(),
                   preIntegratedNavStatePtr->getI_v_W_I(), preIntegratedNavStatePtr->getI_w_W_I(), poseCovarianceRos, twistCovarianceRos);
  pubEstOdomImu_.publish(estOdomImuMsgPtr_);
  // World->imu with 100 Hz
  addToOdometryMsg(estWorldImuMsgPtr_, staticTransformsPtr_->getWorldFrame(), staticTransformsPtr_->getImuFrame(),
                   ros::Time(preIntegratedNavStatePtr->getTimeK()), preIntegratedNavStatePtr->getT_W_Ik(),
                   preIntegratedNavStatePtr->getI_v_W_I(), preIntegratedNavStatePtr->getI_w_W_I(), poseCovarianceRos, twistCovarianceRos);
  pubEstWorldImu_.publish(estWorldImuMsgPtr_);
}

void GraphMsfRos::publishImuPaths_(const std::shared_ptr<graph_msf::SafeIntegratedNavState>& navStatePtr) const {
  // odom->imu
  addToPathMsg(estOdomImuPathPtr_, staticTransformsPtr_->getOdomFrame(), ros::Time(navStatePtr->getTimeK()),
               navStatePtr->getT_O_Ik_gravityAligned().translation(), graphConfigPtr_->imuBufferLength_ * 20);
  pubEstOdomImuPath_.publish(estOdomImuPathPtr_);
  // world->imu
  addToPathMsg(estWorldImuPathPtr_, staticTransformsPtr_->getWorldFrame(), ros::Time(navStatePtr->getTimeK()),
               (navStatePtr->getT_W_Ik()).translation(), graphConfigPtr_->imuBufferLength_ * 20);
  pubEstWorldImuPath_.publish(estWorldImuPathPtr_);
}

void GraphMsfRos::publishAddedImuMeas_(const Eigen::Matrix<double, 6, 1>& addedImuMeas, const ros::Time& stamp) const {
  // Publish added imu measurement
  sensor_msgs::Imu addedImuMeasMsg;
  addedImuMeasMsg.header.stamp = stamp;
  addedImuMeasMsg.header.frame_id = staticTransformsPtr_->getImuFrame();
  addedImuMeasMsg.linear_acceleration.x = addedImuMeas(0);
  addedImuMeasMsg.linear_acceleration.y = addedImuMeas(1);
  addedImuMeasMsg.linear_acceleration.z = addedImuMeas(2);
  addedImuMeasMsg.angular_velocity.x = addedImuMeas(3);
  addedImuMeasMsg.angular_velocity.y = addedImuMeas(4);
  addedImuMeasMsg.angular_velocity.z = addedImuMeas(5);
  pubAddedImuMeas_.publish(addedImuMeasMsg);
}

void GraphMsfRos::publishOptimizedStateAndBias_(
    const std::shared_ptr<graph_msf::SafeNavStateWithCovarianceAndBias>& optimizedStateWithCovarianceAndBiasPtr,
    const Eigen::Matrix<double, 6, 6>& poseCovarianceRos, const Eigen::Matrix<double, 6, 6>& twistCovarianceRos) {
  if (optimizedStateWithCovarianceAndBiasPtr != nullptr &&
      optimizedStateWithCovarianceAndBiasPtr->getTimeK() - lastOptimizedStateTimestamp_ > 1e-03) {
    // Time of this optimized state
    lastOptimizedStateTimestamp_ = optimizedStateWithCovarianceAndBiasPtr->getTimeK();

    // Odometry messages
    // world->imu
    addToOdometryMsg(optWorldImuMsgPtr_, staticTransformsPtr_->getWorldFrame(), staticTransformsPtr_->getImuFrame(),
                     ros::Time(optimizedStateWithCovarianceAndBiasPtr->getTimeK()), optimizedStateWithCovarianceAndBiasPtr->getT_W_Ik(),
                     optimizedStateWithCovarianceAndBiasPtr->getI_v_W_I(), optimizedStateWithCovarianceAndBiasPtr->getI_w_W_I(),
                     poseCovarianceRos, twistCovarianceRos);
    pubOptWorldImu_.publish(optWorldImuMsgPtr_);

    // Path
    // world->imu
    addToPathMsg(optWorldImuPathPtr_, staticTransformsPtr_->getWorldFrame(), ros::Time(optimizedStateWithCovarianceAndBiasPtr->getTimeK()),
                 optimizedStateWithCovarianceAndBiasPtr->getT_W_Ik().translation(), graphConfigPtr_->imuBufferLength_ * 20);
    pubOptWorldImuPath_.publish(optWorldImuPathPtr_);

    // Biases
    Eigen::Vector3d accelBias = optimizedStateWithCovarianceAndBiasPtr->getAccelerometerBias();
    Eigen::Vector3d gyroBias = optimizedStateWithCovarianceAndBiasPtr->getGyroscopeBias();
    // Publish accel bias
    accelBiasMsgPtr_->header.stamp = ros::Time(optimizedStateWithCovarianceAndBiasPtr->getTimeK());
    accelBiasMsgPtr_->header.frame_id = staticTransformsPtr_->getImuFrame();
    accelBiasMsgPtr_->vector.x = accelBias(0);
    accelBiasMsgPtr_->vector.y = accelBias(1);
    accelBiasMsgPtr_->vector.z = accelBias(2);
    pubAccelBias_.publish(accelBiasMsgPtr_);
    // Publish gyro bias
    gyroBiasMsgPtr_->header.stamp = ros::Time(optimizedStateWithCovarianceAndBiasPtr->getTimeK());
    gyroBiasMsgPtr_->header.frame_id = staticTransformsPtr_->getImuFrame();
    gyroBiasMsgPtr_->vector.x = gyroBias(0);
    gyroBiasMsgPtr_->vector.y = gyroBias(1);
    gyroBiasMsgPtr_->vector.z = gyroBias(2);
    pubGyroBias_.publish(gyroBiasMsgPtr_);

    // TFs in Optimized State
    for (const auto& transformIterator : optimizedStateWithCovarianceAndBiasPtr->getFixedFrameTransforms().getTransformsMap()) {
      // Get transform
      const Eigen::Isometry3d& T_frame1_frame2 = transformIterator.second;
      if (transformIterator.first.second == staticTransformsPtr_->getWorldFrame()) {
        const Eigen::Isometry3d T_M_Ik = T_frame1_frame2 * optimizedStateWithCovarianceAndBiasPtr->getT_W_Ik();
        const std::string& mapFrameName = transformIterator.first.first;
        const std::string& worldFrameName = transformIterator.first.second;
        if (graphConfigPtr_->verboseLevel_ >= 2) {
          std::cout << "Transformation from " << mapFrameName << " to " << worldFrameName << std::endl;
          std::cout << "Uncertainty: " << std::endl
                    << optimizedStateWithCovarianceAndBiasPtr->getFixedFrameTransformsCovariance().rv_T_frame1_frame2(
                           mapFrameName, transformIterator.first.second)
                    << std::endl;
        }
        // Map->imu
        addToOdometryMsg(estMapImuMsgPtr_, mapFrameName, staticTransformsPtr_->getImuFrame(),
                         ros::Time(optimizedStateWithCovarianceAndBiasPtr->getTimeK()), T_M_Ik,
                         optimizedStateWithCovarianceAndBiasPtr->getI_v_W_I(), optimizedStateWithCovarianceAndBiasPtr->getI_w_W_I(),
                         poseCovarianceRos, twistCovarianceRos);
        pubEstMapImu_.publish(estMapImuMsgPtr_);
        // Publish TF --> everything children of world
        publishTransform_(worldFrameName, mapFrameName + "_gmsf", optimizedStateWithCovarianceAndBiasPtr->getTimeK(),
                          T_frame1_frame2.inverse());

      } else {
        const std::string& worldFrameName = transformIterator.first.first;
        const std::string& mapFrameName = transformIterator.first.second;
        // Publish TF --> everything children of world
        publishTransform_(worldFrameName, mapFrameName, optimizedStateWithCovarianceAndBiasPtr->getTimeK(), T_frame1_frame2);
      }
    }
  }
}

long GraphMsfRos::secondsSinceStart_() {
  currentTime_ = std::chrono::high_resolution_clock::now();
  return std::chrono::duration_cast<std::chrono::seconds>(currentTime_ - startTime_).count();
}

// Publish state ---------------------------------------------------------------
void GraphMsfRos::publishState_(
    const std::shared_ptr<graph_msf::SafeIntegratedNavState>& integratedNavStatePtr,
    const std::shared_ptr<graph_msf::SafeNavStateWithCovarianceAndBias>& optimizedStateWithCovarianceAndBiasPtr) {
  // Covariances
  Eigen::Matrix<double, 6, 6> poseCovarianceRos, twistCovarianceRos;
  extractCovariancesFromOptimizedState(poseCovarianceRos, twistCovarianceRos, optimizedStateWithCovarianceAndBiasPtr);

  // Alias
  const Eigen::Isometry3d& T_O_Ik = integratedNavStatePtr->getT_O_Ik_gravityAligned();
  const double& timeK = integratedNavStatePtr->getTimeK();

  // Odometry messages
  publishImuOdoms_(integratedNavStatePtr, poseCovarianceRos, twistCovarianceRos);

  // Publish to TF
  // B_O
  Eigen::Isometry3d T_B_Ok =
      staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getBaseLinkFrame(), staticTransformsPtr_->getImuFrame()) *
      T_O_Ik.inverse();
  publishTransform_(staticTransformsPtr_->getBaseLinkFrame(), staticTransformsPtr_->getOdomFrame(), timeK, T_B_Ok);
  // O_W
  Eigen::Isometry3d T_O_W = integratedNavStatePtr->getT_W_O().inverse();
  publishTransform_(staticTransformsPtr_->getOdomFrame(), staticTransformsPtr_->getWorldFrame(), timeK, T_O_W);

  // Publish paths
  publishImuPaths_(integratedNavStatePtr);

  // Optimized estimate ----------------------
  publishOptimizedStateAndBias_(optimizedStateWithCovarianceAndBiasPtr, poseCovarianceRos, twistCovarianceRos);
}

}  // namespace graph_msf
