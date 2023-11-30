/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GMSF_EIGEN_CONVERSIONS_H
#define GMSF_EIGEN_CONVERSIONS_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>

namespace graph_msf {

Eigen::Matrix<double, 6, 6> convertCovarianceGtsamConventionToRosConvention(const Eigen::Matrix<double, 6, 6>& covGtsam);

void odomMsgToEigen(const nav_msgs::Odometry& odomLidar, Eigen::Matrix4d& T);

void geometryPoseToEigen(const geometry_msgs::PoseWithCovarianceStamped& odomLidar, Eigen::Matrix4d& T);

void odomMsgToTf(const nav_msgs::Odometry& odomLidar, tf::Transform& T);

tf::Transform matrix3ToTf(const Eigen::Matrix3d& R);

tf::Transform matrix4ToTf(const Eigen::Matrix4d& T);

tf::Transform isometry3ToTf(const Eigen::Isometry3d& T);

void tfToMatrix4(const tf::Transform& tf_T, Eigen::Matrix4d& T);

void tfToIsometry3(const tf::Transform& tf_T, Eigen::Isometry3d& T);

tf::Transform pose3ToTf(const Eigen::Matrix3d& T);

}  // namespace graph_msf

#endif  // GMSF_EIGEN_CONVERSIONS_H
