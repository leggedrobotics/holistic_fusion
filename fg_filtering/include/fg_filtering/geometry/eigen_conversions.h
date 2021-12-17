#ifndef MENZI_SIM_WS_202111_EIGEN_CONVERSIONS_H
#define MENZI_SIM_WS_202111_EIGEN_CONVERSIONS_H

namespace compslam_se {

inline void odomMsgToEigen(const nav_msgs::Odometry& odomLidar, Eigen::Matrix4d& T) {
  tf::Quaternion tf_q;
  tf::quaternionMsgToTF(odomLidar.pose.pose.orientation, tf_q);
  Eigen::Vector3d t(odomLidar.pose.pose.position.x, odomLidar.pose.pose.position.y, odomLidar.pose.pose.position.z);
  Eigen::Quaternion<double> q(tf_q.getW(), tf_q.getX(), tf_q.getY(), tf_q.getZ());
  Eigen::Matrix3d R();
  T.setIdentity();
  T.block<3, 3>(0, 0) = q.matrix();
  T.block<3, 1>(0, 3) = t;
}

inline tf::Transform matrix3ToTf(const Eigen::Matrix3d& R) {
  Eigen::Quaterniond q(R);
  tf::Transform tf_R;
  tf_R.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
  tf_R.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  return tf_R;
}

inline tf::Transform matrix4ToTf(const Eigen::Matrix4d& T) {
  Eigen::Quaterniond q(T.block<3, 3>(0, 0));
  tf::Transform tf_T;
  tf_T.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
  tf_T.setOrigin(tf::Vector3(T(0, 3), T(1, 3), T(2, 3)));
  return tf_T;
}

}  // namespace compslam_se

#endif  // MENZI_SIM_WS_202111_EIGEN_CONVERSIONS_H
