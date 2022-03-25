#ifndef FG_FILTERING_MATH_UTILS_H
#define FG_FILTERING_MATH_UTILS_H

#include <cmath>

#include "compslam_se/geometry/Angle.h"
#include "compslam_se/geometry/Vector3.h"

namespace compslam_se {

/** \brief Convert the given radian angle to degrees.
 *
 * @param radians The radian angle to convert.
 * @return The angle in degrees.
 */
inline double rad2deg(double radians) {
  return radians * 180.0 / M_PI;
}

/** \brief Convert the given radian angle to degrees.
 *
 * @param radians The radian angle to convert.
 * @return The angle in degrees.
 */
inline float rad2deg(float radians) {
  return (float)(radians * 180.0 / M_PI);
}

/** \brief Convert the given degree angle to radian.
 *
 * @param degrees The degree angle to convert.
 * @return The radian angle.
 */
inline double deg2rad(double degrees) {
  return degrees * M_PI / 180.0;
}

/** \brief Convert the given degree angle to radian.
 *
 * @param degrees The degree angle to convert.
 * @return The radian angle.
 */
inline float deg2rad(float degrees) {
  return (float)(degrees * M_PI / 180.0);
}

/** \brief Calculate the squared difference of the given two points.
 *
 * @param a The first point.
 * @param b The second point.
 * @return The squared difference between point a and b.
 */
template <typename PointT>
inline float calcSquaredDiff(const PointT& a, const PointT& b) {
  float diffX = a.x - b.x;
  float diffY = a.y - b.y;
  float diffZ = a.z - b.z;

  return diffX * diffX + diffY * diffY + diffZ * diffZ;
}

/** \brief Calculate the squared difference of the given two points.
 *
 * @param a The first point.
 * @param b The second point.
 * @param wb The weighting factor for the SECOND point.
 * @return The squared difference between point a and b.
 */
template <typename PointT>
inline float calcSquaredDiff(const PointT& a, const PointT& b, const float& wb) {
  float diffX = a.x - b.x * wb;
  float diffY = a.y - b.y * wb;
  float diffZ = a.z - b.z * wb;

  return diffX * diffX + diffY * diffY + diffZ * diffZ;
}

/** \brief Calculate the absolute distance of the point to the origin.
 *
 * @param p The point.
 * @return The distance to the point.
 */
template <typename PointT>
inline float calcPointDistance(const PointT& p) {
  return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

/** \brief Calculate the squared distance of the point to the origin.
 *
 * @param p The point.
 * @return The squared distance to the point.
 */
template <typename PointT>
inline float calcSquaredPointDistance(const PointT& p) {
  return p.x * p.x + p.y * p.y + p.z * p.z;
}

/** \brief Rotate the given vector by the specified angle around the x-axis.
 *
 * @param v the vector to rotate
 * @param ang the rotation angle
 */
inline void rotX(Vector3& v, const Angle& ang) {
  float y = v.y();
  v.y() = ang.cos() * y - ang.sin() * v.z();
  v.z() = ang.sin() * y + ang.cos() * v.z();
}

/** \brief Rotate the given point by the specified angle around the x-axis.
 *
 * @param p the point to rotate
 * @param ang the rotation angle
 */
template <typename PointT>
inline void rotX(PointT& p, const Angle& ang) {
  float y = p.y;
  p.y = ang.cos() * y - ang.sin() * p.z;
  p.z = ang.sin() * y + ang.cos() * p.z;
}

/** \brief Rotate the given vector by the specified angle around the y-axis.
 *
 * @param v the vector to rotate
 * @param ang the rotation angle
 */
inline void rotY(Vector3& v, const Angle& ang) {
  float x = v.x();
  v.x() = ang.cos() * x + ang.sin() * v.z();
  v.z() = ang.cos() * v.z() - ang.sin() * x;
}

/** \brief Rotate the given point by the specified angle around the y-axis.
 *
 * @param p the point to rotate
 * @param ang the rotation angle
 */
template <typename PointT>
inline void rotY(PointT& p, const Angle& ang) {
  float x = p.x;
  p.x = ang.cos() * x + ang.sin() * p.z;
  p.z = ang.cos() * p.z - ang.sin() * x;
}

/** \brief Rotate the given vector by the specified angle around the z-axis.
 *
 * @param v the vector to rotate
 * @param ang the rotation angle
 */
inline void rotZ(Vector3& v, const Angle& ang) {
  float x = v.x();
  v.x() = ang.cos() * x - ang.sin() * v.y();
  v.y() = ang.sin() * x + ang.cos() * v.y();
}

/** \brief Rotate the given point by the specified angle around the z-axis.
 *
 * @param p the point to rotate
 * @param ang the rotation angle
 */
template <typename PointT>
inline void rotZ(PointT& p, const Angle& ang) {
  float x = p.x;
  p.x = ang.cos() * x - ang.sin() * p.y;
  p.y = ang.sin() * x + ang.cos() * p.y;
}

/** \brief Rotate the given vector by the specified angles around the z-, x- respectively y-axis.
 *
 * @param v the vector to rotate
 * @param angZ the rotation angle around the z-axis
 * @param angX the rotation angle around the x-axis
 * @param angY the rotation angle around the y-axis
 */
inline void rotateZXY(Vector3& v, const Angle& angZ, const Angle& angX, const Angle& angY) {
  rotZ(v, angZ);
  rotX(v, angX);
  rotY(v, angY);
}

/** \brief Rotate the given point by the specified angles around the z-, x- respectively y-axis.
 *
 * @param p the point to rotate
 * @param angZ the rotation angle around the z-axis
 * @param angX the rotation angle around the x-axis
 * @param angY the rotation angle around the y-axis
 */
template <typename PointT>
inline void rotateZXY(PointT& p, const Angle& angZ, const Angle& angX, const Angle& angY) {
  rotZ(p, angZ);
  rotX(p, angX);
  rotY(p, angY);
}

/** \brief Rotate the given vector by the specified angles around the y-, x- respectively z-axis.
 *
 * @param v the vector to rotate
 * @param angY the rotation angle around the y-axis
 * @param angX the rotation angle around the x-axis
 * @param angZ the rotation angle around the z-axis
 */
inline void rotateYXZ(Vector3& v, const Angle& angY, const Angle& angX, const Angle& angZ) {
  rotY(v, angY);
  rotX(v, angX);
  rotZ(v, angZ);
}

/** \brief Rotate the given point by the specified angles around the y-, x- respectively z-axis.
 *
 * @param p the point to rotate
 * @param angY the rotation angle around the y-axis
 * @param angX the rotation angle around the x-axis
 * @param angZ the rotation angle around the z-axis
 */
template <typename PointT>
inline void rotateYXZ(PointT& p, const Angle& angY, const Angle& angX, const Angle& angZ) {
  rotY(p, angY);
  rotX(p, angX);
  rotZ(p, angZ);
}

/** \brief Invert a homogenous transformation matrix
 *
 * @param m_in input 4x4 matrix
 * @param m_out inverted 4x4 matrix
 */
inline void invertHomogenousMatrix(const Eigen::Matrix4d& m_in, Eigen::Matrix4d& m_out) {
  m_out = Eigen::Matrix4d::Identity();
  Eigen::Matrix3d R = m_in.block<3, 3>(0, 0);
  Eigen::Vector3d t = m_in.block<3, 1>(0, 3);
  m_out.block<3, 3>(0, 0) = R.transpose();
  m_out.block<3, 1>(0, 3) = -R.transpose() * t;
}

// inline void odomMsgToTf(const nav_msgs::Odometry& odomLidar, tf::Transform& tf_T) {
//  tf::Quaternion tf_q;
//  tf::quaternionMsgToTF(odomLidar.pose.pose.orientation, tf_q);
//  tf::Vector3 tf_t = tf::Vector3(odomLidar.pose.pose.position.x, odomLidar.pose.pose.position.y, odomLidar.pose.pose.position.z);
//  tf_T.setRotation(tf_q);
//  tf_T.setOrigin(tf_t);
//}

inline tf::Transform pose3ToTf(const Eigen::Matrix4d& T) {
  Eigen::Quaterniond q(T.block<3, 3>(0, 0));
  tf::Transform tf_T;
  tf_T.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
  tf_T.setOrigin(tf::Vector3(T(0, 3), T(1, 3), T(2, 3)));
  return tf_T;
}

inline tf::Transform pose3ToTf(const gtsam::Pose3& T) {
  Eigen::Quaterniond q = T.rotation().toQuaternion();
  tf::Transform tf_T;
  tf_T.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
  tf_T.setOrigin(tf::Vector3(T.x(), T.y(), T.z()));
  return tf_T;
}

inline gtsam::Pose3 tfToPose3(const tf::Transform& tf_T) {
  return gtsam::Pose3(gtsam::Rot3(tf_T.getRotation().w(), tf_T.getRotation().x(), tf_T.getRotation().y(), tf_T.getRotation().z()),
                      gtsam::Vector3(tf_T.getOrigin().x(), tf_T.getOrigin().y(), tf_T.getOrigin().z()));
}

// Transformations need to be in same coordinate frame
inline gtsam::Pose3 computeDeltaPose(const tf::Transform& tf_T_km1, const tf::Transform& tf_T_k) {
  gtsam::Pose3 T_km1 = tfToPose3(tf_T_km1);
  gtsam::Pose3 T_k = tfToPose3(tf_T_k);
  gtsam::Pose3 T_km1_inv = T_km1.inverse();
  return T_km1_inv * T_k;
}

}  // end namespace compslam_se

#endif  // FG_FILTERING_MATH_UTILS_H
