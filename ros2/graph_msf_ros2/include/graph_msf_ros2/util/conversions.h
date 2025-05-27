/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#pragma once

#include <tf2/LinearMath/Transform.h>
#include <Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace graph_msf {

Eigen::Matrix<double, 6, 6> convertCovarianceGtsamConventionToRosConvention(
    const Eigen::Matrix<double, 6, 6>& covGtsam);

void odomMsgToEigen(const nav_msgs::msg::Odometry& odomLidar, Eigen::Matrix4d& T);

void geometryPoseToEigen(const geometry_msgs::msg::Pose& pose, Eigen::Matrix4d& T);

void geometryPoseToEigen(const geometry_msgs::msg::PoseStamped& pose, Eigen::Matrix4d& T);

void geometryPoseToEigen(const geometry_msgs::msg::PoseWithCovarianceStamped& odomLidar, Eigen::Matrix4d& T);

void odomMsgToTf(const nav_msgs::msg::Odometry& odomLidar, tf2::Transform& T);

tf2::Transform matrix3ToTf(const Eigen::Matrix3d& R);

tf2::Transform matrix4ToTf(const Eigen::Matrix4d& T);

tf2::Transform isometry3ToTf(const Eigen::Isometry3d& T);

void tfToMatrix4(const tf2::Transform& tf_T, Eigen::Matrix4d& T);

void tfToIsometry3(const tf2::Transform& tf_T, Eigen::Isometry3d& T);

void transformStampedToIsometry3(const geometry_msgs::msg::Transform& transform, Eigen::Isometry3d& T);

tf2::Transform pose3ToTf(const Eigen::Matrix3d& T);

}  // namespace graph_msf
