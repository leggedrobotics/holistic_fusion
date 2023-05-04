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

void GraphMsfRos::initializePublishers_(std::shared_ptr<ros::NodeHandle>& privateNodePtr) {
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

long GraphMsfRos::secondsSinceStart_() {
  return std::chrono::duration_cast<std::chrono::seconds>(currentTime_ - startTime_).count();
}

void GraphMsfRos::publishState_(
    const std::shared_ptr<graph_msf::SafeNavState>& navStatePtr,
    const std::shared_ptr<graph_msf::SafeNavStateWithCovarianceAndBias>& optimizedStateWithCovarianceAndBiasPtr) {
  // Covariances
  Eigen::Matrix<double, 6, 6> poseCovarianceRos;
  Eigen::Matrix<double, 6, 6> twistCovarianceRos;
  twistCovarianceRos.setZero();
  if (optimizedStateWithCovarianceAndBiasPtr != nullptr) {
    poseCovarianceRos =
        graph_msf::convertCovarianceGtsamConventionToRosConvention(optimizedStateWithCovarianceAndBiasPtr->getPoseCovariance());
    twistCovarianceRos.block<3, 3>(0, 0) = optimizedStateWithCovarianceAndBiasPtr->getVelocityCovariance();
  } else {
    poseCovarianceRos.setZero();
  }

  // Alias
  const Eigen::Isometry3d& T_O_Ik = navStatePtr->getT_O_Ik_gravityAligned();
  const double& timeK = navStatePtr->getTimeK();

  // Odometry messages
  // odom->imu with 100 Hz
  addToOdometryMsg(estOdomImuMsgPtr_, staticTransformsPtr_->getOdomFrame(), staticTransformsPtr_->getImuFrame(), ros::Time(timeK), T_O_Ik,
                   navStatePtr->getI_v_W_I(), navStatePtr->getI_w_W_I(), poseCovarianceRos, twistCovarianceRos);
  pubEstOdomImu_.publish(estOdomImuMsgPtr_);
  // world->imu with 100 Hz
  addToOdometryMsg(estWorldImuMsgPtr_, staticTransformsPtr_->getWorldFrame(), staticTransformsPtr_->getImuFrame(), ros::Time(timeK),
                   navStatePtr->getT_W_Ik(), navStatePtr->getI_v_W_I(), navStatePtr->getI_w_W_I(), poseCovarianceRos, twistCovarianceRos);
  pubEstWorldImu_.publish(estWorldImuMsgPtr_);

  // Publish to TF
  // O_B
  static tf::Transform transform_B_O;
  Eigen::Isometry3d T_B_Ok =
      staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getBaseLinkFrame(), staticTransformsPtr_->getImuFrame()) *
      T_O_Ik.inverse();
  transform_B_O.setOrigin(tf::Vector3(T_B_Ok(0, 3), T_B_Ok(1, 3), T_B_Ok(2, 3)));
  Eigen::Quaterniond q_B_O(T_B_Ok.rotation());
  transform_B_O.setRotation(tf::Quaternion(q_B_O.x(), q_B_O.y(), q_B_O.z(), q_B_O.w()));
  tfBroadcaster_.sendTransform(tf::StampedTransform(transform_B_O, ros::Time(timeK), staticTransformsPtr_->getBaseLinkFrame(),
                                                    staticTransformsPtr_->getOdomFrame()));
  // O_W
  static tf::Transform transform_O_W;
  Eigen::Isometry3d T_O_W = navStatePtr->getT_W_O().inverse();
  transform_O_W.setOrigin(tf::Vector3(T_O_W(0, 3), T_O_W(1, 3), T_O_W(2, 3)));
  Eigen::Quaterniond q_O_W(T_O_W.rotation());
  transform_O_W.setRotation(tf::Quaternion(q_O_W.x(), q_O_W.y(), q_O_W.z(), q_O_W.w()));
  tfBroadcaster_.sendTransform(
      tf::StampedTransform(transform_O_W, ros::Time(timeK), staticTransformsPtr_->getOdomFrame(), staticTransformsPtr_->getWorldFrame()));

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
  if (optimizedStateWithCovarianceAndBiasPtr != nullptr &&
      optimizedStateWithCovarianceAndBiasPtr->getTimeK() - lastOptimizedStateTimestamp_ > 1e-03) {
    // Time of this optimized state
    lastOptimizedStateTimestamp_ = optimizedStateWithCovarianceAndBiasPtr->getTimeK();

    // Odometry messages
    // world->imu
    addToOdometryMsg(optWorldImuMsgPtr_, staticTransformsPtr_->getWorldFrame(), staticTransformsPtr_->getImuFrame(),
                     ros::Time(optimizedStateWithCovarianceAndBiasPtr->getTimeK()), optimizedStateWithCovarianceAndBiasPtr->getT_W_Ik(),
                     optimizedStateWithCovarianceAndBiasPtr->getI_v_W_I(), optimizedStateWithCovarianceAndBiasPtr->getI_w_W_I());
    pubOptWorldImu_.publish(optWorldImuMsgPtr_);

    // Path
    // world->imu
    addToPathMsg(optWorldImuPathPtr_, staticTransformsPtr_->getWorldFrame(), ros::Time(optimizedStateWithCovarianceAndBiasPtr->getTimeK()),
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

}  // namespace graph_msf