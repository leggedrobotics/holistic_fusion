// Implementation
#include "graph_msf_ros2/util/conversions.h"
#include <tf2_eigen/tf2_eigen.hpp>

namespace graph_msf {

Eigen::Matrix<double, 6, 6> convertCovarianceGtsamConventionToRosConvention(
    const Eigen::Matrix<double, 6, 6>& covGtsam) {
  Eigen::Matrix<double, 6, 6> covRos;
  covRos.setZero();
  covRos.block<3, 3>(0, 0) = covGtsam.block<3, 3>(3, 3);
  covRos.block<3, 3>(3, 3) = covGtsam.block<3, 3>(0, 0);
  covRos.block<3, 3>(0, 3) = covGtsam.block<3, 3>(3, 0);
  covRos.block<3, 3>(3, 0) = covGtsam.block<3, 3>(0, 3);
  return covRos;
}

void geometryPoseToEigen(const geometry_msgs::msg::Pose& pose, Eigen::Matrix4d& T) {
  Eigen::Affine3d affine = Eigen::Affine3d::Identity();
  affine.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  affine.linear() = q.toRotationMatrix();
  T = affine.matrix();
}

void geometryPoseToEigen(const geometry_msgs::msg::PoseStamped& pose, Eigen::Matrix4d& T) {
  geometryPoseToEigen(pose.pose, T);
}

void geometryPoseToEigen(const geometry_msgs::msg::PoseWithCovarianceStamped& pose, Eigen::Matrix4d& T) {
  geometryPoseToEigen(pose.pose.pose, T);
}

void odomMsgToEigen(const nav_msgs::msg::Odometry& odomLidar, Eigen::Matrix4d& T) {
  Eigen::Affine3d affine = Eigen::Affine3d::Identity();

  affine.translation() =
      Eigen::Vector3d(odomLidar.pose.pose.position.x, odomLidar.pose.pose.position.y, odomLidar.pose.pose.position.z);
  Eigen::Quaterniond q(odomLidar.pose.pose.orientation.w, odomLidar.pose.pose.orientation.x,
                       odomLidar.pose.pose.orientation.y, odomLidar.pose.pose.orientation.z);
  affine.linear() = q.toRotationMatrix();
  T = affine.matrix();
}

void odomMsgToTf(const nav_msgs::msg::Odometry& odomLidar, tf2::Transform& T) {
  T = tf2::Transform(
      tf2::Quaternion(odomLidar.pose.pose.orientation.x, odomLidar.pose.pose.orientation.y,
                      odomLidar.pose.pose.orientation.z, odomLidar.pose.pose.orientation.w),
      tf2::Vector3(odomLidar.pose.pose.position.x, odomLidar.pose.pose.position.y, odomLidar.pose.pose.position.z));
}

tf2::Transform matrix3ToTf(const Eigen::Matrix3d& R) {
  Eigen::Quaterniond q(R);
  return tf2::Transform(tf2::Quaternion(q.x(), q.y(), q.z(), q.w()), tf2::Vector3(0.0, 0.0, 0.0));
}

tf2::Transform matrix4ToTf(const Eigen::Matrix4d& T) {
  Eigen::Quaterniond q(T.block<3, 3>(0, 0));
  return tf2::Transform(tf2::Quaternion(q.x(), q.y(), q.z(), q.w()), tf2::Vector3(T(0, 3), T(1, 3), T(2, 3)));
}

tf2::Transform isometry3ToTf(const Eigen::Isometry3d& T) {
  Eigen::Quaterniond q(T.rotation());
  return tf2::Transform(tf2::Quaternion(q.x(), q.y(), q.z(), q.w()),
                        tf2::Vector3(T.translation().x(), T.translation().y(), T.translation().z()));
}

#include <tf2/LinearMath/Transform.h>
#include <Eigen/Geometry>

void tfToMatrix4(const tf2::Transform& tf_T, Eigen::Matrix4d& T) {
  Eigen::Affine3d affine = Eigen::Affine3d::Identity();
  affine.translation() = Eigen::Vector3d(tf_T.getOrigin().x(), tf_T.getOrigin().y(), tf_T.getOrigin().z());
  Eigen::Quaterniond q(tf_T.getRotation().w(), tf_T.getRotation().x(), tf_T.getRotation().y(), tf_T.getRotation().z());
  affine.linear() = q.toRotationMatrix();
  T = affine.matrix();
}

void tfToIsometry3(const tf2::Transform& tf_T, Eigen::Isometry3d& T) {
  tf2::Vector3 origin = tf_T.getOrigin();
  T.translation() << origin.x(), origin.y(), origin.z();
  tf2::Quaternion tf_q = tf_T.getRotation();
  Eigen::Quaterniond eigen_q(tf_q.w(), tf_q.x(), tf_q.y(), tf_q.z());
  T.linear() = eigen_q.toRotationMatrix();
}

void transformStampedToIsometry3(const geometry_msgs::msg::Transform& transform, Eigen::Isometry3d& T) {
  T.translation() = Eigen::Vector3d(transform.translation.x, transform.translation.y, transform.translation.z);
  Eigen::Quaterniond q(transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z);
  T.linear() = q.toRotationMatrix();
}

tf2::Transform pose3ToTf(const Eigen::Matrix3d& T) {
  Eigen::Quaterniond q(T);
  return tf2::Transform(tf2::Quaternion(q.x(), q.y(), q.z(), q.w()), tf2::Vector3(0.0, 0.0, 0.0));
}

}  // namespace graph_msf
