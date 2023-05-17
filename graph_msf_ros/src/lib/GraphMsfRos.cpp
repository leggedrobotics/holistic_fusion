/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "graph_msf_ros/GraphMsfRos.h"

// ROS
#include <tf/tf.h>

// Workspace
#include "graph_msf_ros/util/conversions.h"

namespace graph_msf {

GraphMsfRos::GraphMsfRos(std::shared_ptr<ros::NodeHandle> privateNodePtr) : privateNode_(*privateNodePtr) {
  std::cout << YELLOW_START << "GraphMsfRos" << GREEN_START << " Setting up." << COLOR_END << std::endl;

  // Configurations ----------------------------
  graphConfigPtr_ = std::make_shared<GraphConfig>();
  staticTransformsPtr_ = std::make_shared<StaticTransforms>();

  // Publishers ----------------------------
  initializePublishers_(privateNodePtr);

  // Subscribers ----------------------------
  initializeSubscribers_(privateNodePtr);

  // Messages ----------------------------
  initializeMessages_(privateNodePtr);

  // Read parameters ----------------------------
  readParams_(privateNode_);

  // Core class
  if (not this->setup()) {
    throw std::runtime_error("GraphMsfRos could not be initialized");
  }
}

void GraphMsfRos::initializePublishers_(std::shared_ptr<ros::NodeHandle>& privateNodePtr) {
  std::cout << YELLOW_START << "GraphMsfRos" << GREEN_START << " Initializing publishers." << COLOR_END << std::endl;
  // Odometry
  pubEstOdomImu_ = privateNodePtr->advertise<nav_msgs::Odometry>("/graph_msf/est_odometry_odom_imu", ROS_QUEUE_SIZE);
  pubEstWorldImu_ = privateNodePtr->advertise<nav_msgs::Odometry>("/graph_msf/est_odometry_world_imu", ROS_QUEUE_SIZE);
  pubOptWorldImu_ = privateNodePtr->advertise<nav_msgs::Odometry>("/graph_msf/opt_odometry_world_imu", ROS_QUEUE_SIZE);
  // Paths
  pubEstOdomImuPath_ = privateNodePtr->advertise<nav_msgs::Path>("/graph_msf/est_path_odom_imu", ROS_QUEUE_SIZE);
  pubEstWorldImuPath_ = privateNodePtr->advertise<nav_msgs::Path>("/graph_msf/est_path_world_imu", ROS_QUEUE_SIZE);
  pubOptWorldImuPath_ = privateNodePtr->advertise<nav_msgs::Path>("/graph_msf/opt_path_world_imu", ROS_QUEUE_SIZE);
  // Imu Bias
  pubAccelBias_ = privateNodePtr->advertise<geometry_msgs::Vector3Stamped>("/graph_msf/accel_bias", ROS_QUEUE_SIZE);
  pubGyroBias_ = privateNodePtr->advertise<geometry_msgs::Vector3Stamped>("/graph_msf/gyro_bias", ROS_QUEUE_SIZE);
}

void GraphMsfRos::initializeMessages_(std::shared_ptr<ros::NodeHandle>& privateNodePtr) {
  std::cout << YELLOW_START << "GraphMsfRos" << GREEN_START << " Initializing messages." << COLOR_END << std::endl;
  // Odometry
  estOdomImuMsgPtr_ = nav_msgs::OdometryPtr(new nav_msgs::Odometry);
  estWorldImuMsgPtr_ = nav_msgs::OdometryPtr(new nav_msgs::Odometry);
  optWorldImuMsgPtr_ = nav_msgs::OdometryPtr(new nav_msgs::Odometry);
  // Path
  estOdomImuPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  estWorldImuPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  optWorldImuPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  // Imu Bias
  accelBiasMsgPtr_ = geometry_msgs::Vector3StampedPtr(new geometry_msgs::Vector3Stamped);
  gyroBiasMsgPtr_ = geometry_msgs::Vector3StampedPtr(new geometry_msgs::Vector3Stamped);
}

void GraphMsfRos::initializeSubscribers_(std::shared_ptr<ros::NodeHandle>& privateNodePtr) {
  std::cout << YELLOW_START << "GraphMsfRos" << GREEN_START << " Initializing subscribers." << COLOR_END << std::endl;
  // Imu
  subImu_ = privateNodePtr->subscribe<sensor_msgs::Imu>("/imu_topic", ROS_QUEUE_SIZE, &GraphMsfRos::imuCallback_, this,
                                                        ros::TransportHints().tcpNoDelay());
  std::cout << YELLOW_START << "GraphMsfRos" << COLOR_END << " Initialized IMU cabin subscriber with topic: " << subImu_.getTopic()
            << std::endl;
}

void GraphMsfRos::imuCallback_(const sensor_msgs::Imu::ConstPtr& imuMsgPtr) {
  // Convert to Eigen
  Eigen::Vector3d linearAcc(imuMsgPtr->linear_acceleration.x, imuMsgPtr->linear_acceleration.y, imuMsgPtr->linear_acceleration.z);
  Eigen::Vector3d angularVel(imuMsgPtr->angular_velocity.x, imuMsgPtr->angular_velocity.y, imuMsgPtr->angular_velocity.z);
  // Create pointer for carrying state
  std::shared_ptr<graph_msf::SafeNavState> preIntegratedNavStatePtr;
  std::shared_ptr<graph_msf::SafeNavStateWithCovarianceAndBias> optimizedStateWithCovarianceAndBiasPtr;
  // Add measurement and get state
  if (this->addImuMeasurementAndGetState(linearAcc, angularVel, imuMsgPtr->header.stamp.toSec(), preIntegratedNavStatePtr,
                                         optimizedStateWithCovarianceAndBiasPtr)) {
    // Encountered delay
    if (ros::Time::now() - ros::Time(preIntegratedNavStatePtr->getTimeK()) > ros::Duration(0.5)) {
      std::cout << RED_START << "GraphMsfRos" << COLOR_END << " Encountered delay of "
                << (ros::Time::now() - ros::Time(preIntegratedNavStatePtr->getTimeK())).toSec() << " seconds." << std::endl;
    }
    // Publish Odometry
    this->publishState_(preIntegratedNavStatePtr, optimizedStateWithCovarianceAndBiasPtr);
  }
}

void GraphMsfRos::addToPathMsg(nav_msgs::PathPtr pathPtr, const std::string& frameName, const ros::Time& stamp, const Eigen::Vector3d& t,
                               const int maxBufferLength) {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = frameName;
  pose.header.stamp = stamp;
  pose.pose.position.x = t(0);
  pose.pose.position.y = t(1);
  pose.pose.position.z = t(2);
  pathPtr->header.frame_id = frameName;
  pathPtr->header.stamp = stamp;
  pathPtr->poses.push_back(pose);
  if (pathPtr->poses.size() > maxBufferLength) {
    pathPtr->poses.erase(pathPtr->poses.begin());
  }
}

void GraphMsfRos::addToOdometryMsg(nav_msgs::OdometryPtr msgPtr, const std::string& fixedFrame, const std::string& movingFrame,
                                   const ros::Time& stamp, const Eigen::Isometry3d& T, const Eigen::Vector3d& W_v_W_F,
                                   const Eigen::Vector3d& W_w_W_F, const Eigen::Matrix<double, 6, 6>& poseCovariance,
                                   const Eigen::Matrix<double, 6, 6>& twistCovariance) {
  msgPtr->header.frame_id = fixedFrame;
  msgPtr->child_frame_id = movingFrame;
  msgPtr->header.stamp = stamp;
  tf::poseTFToMsg(isometry3ToTf(T), msgPtr->pose.pose);
  msgPtr->twist.twist.linear.x = W_v_W_F(0);
  msgPtr->twist.twist.linear.y = W_v_W_F(1);
  msgPtr->twist.twist.linear.z = W_v_W_F(2);
  msgPtr->twist.twist.angular.x = W_w_W_F(0);
  msgPtr->twist.twist.angular.y = W_w_W_F(1);
  msgPtr->twist.twist.angular.z = W_w_W_F(2);

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

long GraphMsfRos::secondsSinceStart_() {
  return std::chrono::duration_cast<std::chrono::seconds>(currentTime_ - startTime_).count();
}

void GraphMsfRos::publishTransform_(const std::string& frameName, const std::string& childFrameName, const double timeStamp,
                                    const Eigen::Isometry3d& T_frame_childFrame) {
  static tf::Transform transform_frame_childFrame;
  transform_frame_childFrame.setOrigin(tf::Vector3(T_frame_childFrame(0, 3), T_frame_childFrame(1, 3), T_frame_childFrame(2, 3)));
  Eigen::Quaterniond q_frame_childFrame(T_frame_childFrame.rotation());
  transform_frame_childFrame.setRotation(
      tf::Quaternion(q_frame_childFrame.x(), q_frame_childFrame.y(), q_frame_childFrame.z(), q_frame_childFrame.w()));
  tfBroadcaster_.sendTransform(tf::StampedTransform(transform_frame_childFrame, ros::Time(timeStamp), frameName, childFrameName));
}

void GraphMsfRos::publishImuOdom_(const std::shared_ptr<graph_msf::SafeNavState>& navStatePtr,
                                  const Eigen::Matrix<double, 6, 6>& poseCovarianceRos,
                                  const Eigen::Matrix<double, 6, 6>& twistCovarianceRos) {
  // odom->imu with 100 Hz
  addToOdometryMsg(estOdomImuMsgPtr_, staticTransformsPtr_->getOdomFrame(), staticTransformsPtr_->getImuFrame(),
                   ros::Time(navStatePtr->getTimeK()), navStatePtr->getT_O_Ik_gravityAligned(), navStatePtr->getI_v_W_I(),
                   navStatePtr->getI_w_W_I(), poseCovarianceRos, twistCovarianceRos);
  pubEstOdomImu_.publish(estOdomImuMsgPtr_);
  // world->imu with 100 Hz
  addToOdometryMsg(estWorldImuMsgPtr_, staticTransformsPtr_->getWorldFrame(), staticTransformsPtr_->getImuFrame(),
                   ros::Time(navStatePtr->getTimeK()), navStatePtr->getT_W_Ik(), navStatePtr->getI_v_W_I(), navStatePtr->getI_w_W_I(),
                   poseCovarianceRos, twistCovarianceRos);
  pubEstWorldImu_.publish(estWorldImuMsgPtr_);
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
    addToOdometryMsg(optWorldImuMsgPtr_, staticTransformsPtr_->getMapFrame(), staticTransformsPtr_->getImuFrame(),
                     ros::Time(optimizedStateWithCovarianceAndBiasPtr->getTimeK()), optimizedStateWithCovarianceAndBiasPtr->getT_W_Ik(),
                     optimizedStateWithCovarianceAndBiasPtr->getI_v_W_I(), optimizedStateWithCovarianceAndBiasPtr->getI_w_W_I());
    pubOptWorldImu_.publish(optWorldImuMsgPtr_);

    // Path
    // world->imu
    addToPathMsg(optWorldImuPathPtr_, staticTransformsPtr_->getMapFrame(), ros::Time(optimizedStateWithCovarianceAndBiasPtr->getTimeK()),
                 optimizedStateWithCovarianceAndBiasPtr->getT_W_Ik().translation(), graphConfigPtr_->imuBufferLength * 20);
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
  }
}

// Publish state ---------------------------------------------------------------
void GraphMsfRos::publishState_(
    const std::shared_ptr<graph_msf::SafeNavState>& navStatePtr,
    const std::shared_ptr<graph_msf::SafeNavStateWithCovarianceAndBias>& optimizedStateWithCovarianceAndBiasPtr) {
  // Covariances
  Eigen::Matrix<double, 6, 6> poseCovarianceRos, twistCovarianceRos;
  extractCovariancesFromOptimizedState(poseCovarianceRos, twistCovarianceRos, optimizedStateWithCovarianceAndBiasPtr);

  // Alias
  const Eigen::Isometry3d& T_O_Ik = navStatePtr->getT_O_Ik_gravityAligned();
  const double& timeK = navStatePtr->getTimeK();

  // Odometry messages
  publishImuOdom_(navStatePtr, poseCovarianceRos, twistCovarianceRos);

  // Publish to TF
  // B_O
  Eigen::Isometry3d T_B_Ok =
      staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getBaseLinkFrame(), staticTransformsPtr_->getImuFrame()) *
      T_O_Ik.inverse();
  publishTransform_(staticTransformsPtr_->getBaseLinkFrame(), staticTransformsPtr_->getOdomFrame(), timeK, T_B_Ok);
  // O_M
  Eigen::Isometry3d T_O_M = navStatePtr->getT_M_O().inverse();
  publishTransform_(staticTransformsPtr_->getOdomFrame(), staticTransformsPtr_->getMapFrame(), timeK, T_O_M);
  // O_W
  Eigen::Isometry3d T_O_W = navStatePtr->getT_W_O().inverse();
  publishTransform_(staticTransformsPtr_->getOdomFrame(), staticTransformsPtr_->getWorldFrame(), timeK, T_O_W);

  // Publish paths
  // odom->imu
  addToPathMsg(estOdomImuPathPtr_, staticTransformsPtr_->getOdomFrame(), ros::Time(timeK), T_O_Ik.translation(),
               graphConfigPtr_->imuBufferLength * 20);
  pubEstOdomImuPath_.publish(estOdomImuPathPtr_);
  // world->imu
  addToPathMsg(estWorldImuPathPtr_, staticTransformsPtr_->getWorldFrame(), ros::Time(timeK), (T_O_W.inverse() * T_O_Ik).translation(),
               graphConfigPtr_->imuBufferLength * 20);
  pubEstWorldImuPath_.publish(estWorldImuPathPtr_);

  // Optimized estimate ----------------------
  publishOptimizedStateAndBias_(optimizedStateWithCovarianceAndBiasPtr, poseCovarianceRos, twistCovarianceRos);
}

}  // namespace graph_msf