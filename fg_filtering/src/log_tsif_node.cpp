// ROS
#include <ros/ros.h>
// Local packages
#include "fg_filtering/FactorGraphFiltering.h"
#include "m545_msgs/M545State.h"
#include "nav_msgs/Odometry.h"

#include "fg_filtering/SignalLoggerTsif.h"

fg_filtering::SignalLoggerTsif signalLogger;

tf::TransformListener* transformListenerPtr;
ros::Publisher* odomPubPtr;

std::string imuFrame = "IMU_CABIN_link";  // "imu_box_link"

void m545StateCallback(const m545_msgs::M545State::ConstPtr& m545StatePtr) {
  // Flag
  static bool firstCallback__ = true;
  static bool secondCallback__ = true;
  static gtsam::Pose3 T_W_lidar_init__;

  // Listen to tf to get estimate in imu frame
  tf::StampedTransform tf_T_chassis_lidar;
  tf::StampedTransform tf_T_lidar_imu;
  try {
    transformListenerPtr->lookupTransform("BASE", "os_lidar", ros::Time(0), tf_T_chassis_lidar);
    transformListenerPtr->lookupTransform("os_lidar", imuFrame, ros::Time(0), tf_T_lidar_imu);
  } catch (tf2::ConnectivityException) {
    std::cout << "Could not look up transform." << std::endl;
    return;
  }

  // Transform pose to lidar frame
  tf::Transform tf_T_W_chassis(
      tf::Quaternion(m545StatePtr->chassis_pose.orientation.x, m545StatePtr->chassis_pose.orientation.y,
                     m545StatePtr->chassis_pose.orientation.z, m545StatePtr->chassis_pose.orientation.w),
      tf::Vector3(m545StatePtr->chassis_pose.position.x, m545StatePtr->chassis_pose.position.y, m545StatePtr->chassis_pose.position.z));
  tf::StampedTransform tf_T_W_lidar;
  tf_T_W_lidar.setData(tf_T_W_chassis * tf_T_chassis_lidar);

  // Publish this message as odometry type
  nav_msgs::Odometry::Ptr odomMsgPtr(new nav_msgs::Odometry);
  odomMsgPtr->header.frame_id = "map";
  odomMsgPtr->child_frame_id = "os_lidar";
  odomMsgPtr->header.stamp = m545StatePtr->header.stamp;
  tf::poseTFToMsg(tf_T_W_lidar, odomMsgPtr->pose.pose);
  odomPubPtr->publish(odomMsgPtr);

  // Now log data in correct coordinate frame
  gtsam::Pose3 T_W_lidar(gtsam::Quaternion(tf_T_W_lidar.getRotation().w(), tf_T_W_lidar.getRotation().x(), tf_T_W_lidar.getRotation().y(),
                                           tf_T_W_lidar.getRotation().z()),
                         gtsam::Point3(tf_T_W_lidar.getOrigin().x(), tf_T_W_lidar.getOrigin().y(), tf_T_W_lidar.getOrigin().z()));
  if (firstCallback__) {
    firstCallback__ = false;
    return;
  } else if (secondCallback__) {
    T_W_lidar_init__ = T_W_lidar;
    secondCallback__ = false;
    return;
  }

  // Initial attitude and position
  gtsam::Matrix4 initialLidarMatrix;
  /// oberglatt
  // initialLidarMatrix << 0.409202, -0.894872, 0.178208, 8.97905, 0.909267, 0.416207, 0.00212302, 0.0377665, -0.0760714, 0.16117, 0.98399,
  //    5.32733, 0, 0, 0, 1;
  /// Arche
  initialLidarMatrix << -0.390712, -0.919848, 0.0349774, 1.29918, 0.920357, -0.389666, 0.033216, 0.887258, -0.0169242, 0.0451696, 0.998836,
      0.148529, 0, 0, 0, 1;
  gtsam::Pose3 initialLidarPose = gtsam::Pose3(initialLidarMatrix);
  gtsam::Pose3 T_W_lidar_corrected = initialLidarPose * T_W_lidar_init__.inverse() * T_W_lidar;  // T_W_lidar

  // Transform pose to imu frame
  gtsam::Pose3 T_lidar_imu(gtsam::Quaternion(tf_T_lidar_imu.getRotation().w(), tf_T_lidar_imu.getRotation().x(),
                                             tf_T_lidar_imu.getRotation().y(), tf_T_lidar_imu.getRotation().z()),
                           gtsam::Point3(tf_T_lidar_imu.getOrigin().x(), tf_T_lidar_imu.getOrigin().y(), tf_T_lidar_imu.getOrigin().z()));
  gtsam::Pose3 T_W_imu = T_W_lidar_corrected * T_lidar_imu;

  // Transform pose to imu frame
  // tf::StampedTransform tf_T_W_imu;
  // tf_T_W_imu.setData(tf_T_W_lidar * tf_T_chassis_imu);

  // Write to gtsam datatypes
  // gtsam::Point3 W_v_W_chassis(m545StatePtr->chassis_twist.linear.x, m545StatePtr->chassis_twist.linear.y,
  //                            m545StatePtr->chassis_twist.linear.z);
  // gtsam::Point3 imu_v_W_chassis = T_W_Imu.rotation() * W_v_W_chassis;

  signalLogger.publishLogger(m545StatePtr->header.stamp.sec, m545StatePtr->header.stamp.nsec, T_W_imu, gtsam::Point3());
}

void lidarOdometryCallback(const nav_msgs::Odometry::ConstPtr& odomPtr) {
  // Flag
  static bool firstCallback__ = true;
  static bool secondCallback__ = true;
  static gtsam::Pose3 T_W_lidar_init__;

  // Convert message to gtsam format
  gtsam::Pose3 T_W_lidar(gtsam::Quaternion(odomPtr->pose.pose.orientation.w, odomPtr->pose.pose.orientation.x,
                                           odomPtr->pose.pose.orientation.y, odomPtr->pose.pose.orientation.z),
                         gtsam::Point3(odomPtr->pose.pose.position.x, odomPtr->pose.pose.position.y, odomPtr->pose.pose.position.z));

  if (firstCallback__) {
    firstCallback__ = false;
    return;
  } else if (secondCallback__) {
    T_W_lidar_init__ = T_W_lidar;
    secondCallback__ = false;
    return;
  }

  // Initial attitude and position
  gtsam::Matrix4 initialLidarMatrix;
  /// oberglatt
  initialLidarMatrix << 0.409202, -0.894872, 0.178208, 8.97905, 0.909267, 0.416207, 0.00212302, 0.0377665, -0.0760714, 0.16117, 0.98399,
      5.32733, 0, 0, 0, 1;
  /// Arche
  // initialLidarMatrix << -0.390712, -0.919848, 0.0349774, 1.29918, 0.920357, -0.389666, 0.033216, 0.887258, -0.0169242, 0.0451696,
  // 0.998836,
  //    0.148529, 0, 0, 0, 1;
  gtsam::Pose3 initialLidarPose = gtsam::Pose3(initialLidarMatrix);
  gtsam::Pose3 T_W_lidar_corrected = initialLidarPose * T_W_lidar_init__.inverse() * T_W_lidar;

  // Listen to tf to get estimate in imu frame
  tf::StampedTransform tf_T_lidar_imu;
  try {
    transformListenerPtr->lookupTransform("os_lidar", imuFrame, ros::Time(0), tf_T_lidar_imu);
  } catch (tf2::ConnectivityException) {
    std::cout << "Could not look up transform." << std::endl;
    return;
  }
  gtsam::Pose3 T_lidar_imu(gtsam::Quaternion(tf_T_lidar_imu.getRotation().w(), tf_T_lidar_imu.getRotation().x(),
                                             tf_T_lidar_imu.getRotation().y(), tf_T_lidar_imu.getRotation().z()),
                           gtsam::Point3(tf_T_lidar_imu.getOrigin().x(), tf_T_lidar_imu.getOrigin().y(), tf_T_lidar_imu.getOrigin().z()));

  // Transform pose to imu frame
  gtsam::Pose3 T_W_imu = T_W_lidar_corrected * T_lidar_imu;

  // Write to singal logger
  signalLogger.publishLogger(odomPtr->header.stamp.sec, odomPtr->header.stamp.nsec, T_W_imu, gtsam::Point3());
}

void imuOdometryCallback(const nav_msgs::Odometry::ConstPtr& odomPtr) {
  // Flag
  static bool firstCallback__ = true;
  static bool secondCallback__ = true;
  static gtsam::Pose3 T_W_lidar_init__;

  // Convert messages to tf
  tf::Transform tf_T_W_imu(tf::Quaternion(odomPtr->pose.pose.orientation.x, odomPtr->pose.pose.orientation.y,
                                          odomPtr->pose.pose.orientation.z, odomPtr->pose.pose.orientation.w),
                           tf::Vector3(odomPtr->pose.pose.position.x, odomPtr->pose.pose.position.y, odomPtr->pose.pose.position.z));

  // Convert message to gtsam format
  gtsam::Pose3 T_W_imu(gtsam::Quaternion(odomPtr->pose.pose.orientation.w, odomPtr->pose.pose.orientation.x,
                                         odomPtr->pose.pose.orientation.y, odomPtr->pose.pose.orientation.z),
                       gtsam::Point3(odomPtr->pose.pose.position.x, odomPtr->pose.pose.position.y, odomPtr->pose.pose.position.z));

  // Listen to tf to get estimate in imu frame
  tf::StampedTransform tf_T_lidar_imu;
  try {
    transformListenerPtr->lookupTransform("os_lidar", imuFrame, ros::Time(0), tf_T_lidar_imu);
  } catch (tf2::ConnectivityException) {
    std::cout << "Could not look up transform." << std::endl;
    return;
  }
  tf::Transform tf_T_imu_lidar = tf_T_lidar_imu.inverse();

  // Move to gtsam
  gtsam::Pose3 T_lidar_imu(gtsam::Quaternion(tf_T_lidar_imu.getRotation().w(), tf_T_lidar_imu.getRotation().x(),
                                             tf_T_lidar_imu.getRotation().y(), tf_T_lidar_imu.getRotation().z()),
                           gtsam::Point3(tf_T_lidar_imu.getOrigin().x(), tf_T_lidar_imu.getOrigin().y(), tf_T_lidar_imu.getOrigin().z()));
  gtsam::Pose3 T_imu_lidar = T_lidar_imu.inverse();

  // Convert to lidar frame
  tf::Transform tf_T_W_lidar = tf_T_W_imu * tf_T_imu_lidar;
  gtsam::Pose3 T_W_lidar = T_W_imu * T_imu_lidar;

  // Publish in lidar frame
  nav_msgs::Odometry::Ptr odomMsgPtr(new nav_msgs::Odometry);
  odomMsgPtr->header.frame_id = "map";
  odomMsgPtr->child_frame_id = "os_lidar";
  odomMsgPtr->header.stamp = odomPtr->header.stamp;
  tf::poseTFToMsg(tf_T_W_lidar, odomMsgPtr->pose.pose);
  odomPubPtr->publish(odomMsgPtr);

  if (firstCallback__) {
    firstCallback__ = false;
    return;
  } else if (secondCallback__) {
    T_W_lidar_init__ = T_W_lidar;
    secondCallback__ = false;
    return;
  }

  // Initial attitude and position
  gtsam::Matrix4 initialLidarMatrix;
  /// oberglatt
  initialLidarMatrix << 0.409202, -0.894872, 0.178208, 8.97905, 0.909267, 0.416207, 0.00212302, 0.0377665, -0.0760714, 0.16117, 0.98399,
      5.32733, 0, 0, 0, 1;
  /// Arche
  // initialLidarMatrix << -0.390712, -0.919848, 0.0349774, 1.29918, 0.920357, -0.389666, 0.033216, 0.887258, -0.0169242, 0.0451696,
  // 0.998836,
  //    0.148529, 0, 0, 0, 1;
  gtsam::Pose3 initialLidarPose = gtsam::Pose3(initialLidarMatrix);
  gtsam::Pose3 T_W_lidar_corrected = initialLidarPose * T_W_lidar_init__.inverse() * T_W_lidar;

  // Transform pose to imu frame
  T_W_imu = T_W_lidar_corrected * T_lidar_imu;

  // Write to singal logger
  signalLogger.publishLogger(odomPtr->header.stamp.sec, odomPtr->header.stamp.nsec, T_W_imu, gtsam::Point3());
}

int main(int argc, char** argv) {
  std::cout << "CHECK IMU FRAME!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;

  // ROS related
  ros::init(argc, argv, "laserOdometry");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  // Initialize ros publishers and listeners
  transformListenerPtr = new (tf::TransformListener);
  odomPubPtr = new (ros::Publisher);
  // Set publisher
  *odomPubPtr = node.advertise<nav_msgs::Odometry>("/fg_filtering/transform_tsif_world_lidar", 10);
  //*odomPubPtr = node.advertise<nav_msgs::Odometry>("/fg_filtering/transform_msf_world_lidar", 10);

  // Initialize signal logger
  signalLogger.setup(node);

  // Initialize subscriber
  ros::Subscriber subscriber;
  subscriber = node.subscribe<m545_msgs::M545State>("/m545_state", 10, m545StateCallback, ros::TransportHints().tcpNoDelay());
  // subscriber =
  //    node.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init_CORRECTED", 10, lidarOdometryCallback, ros::TransportHints().tcpNoDelay());
  // subscriber = node.subscribe<nav_msgs::Odometry>("/msf_core/odometry", 10, imuOdometryCallback, ros::TransportHints().tcpNoDelay());

  // Spin
  ros::spin();

  // Do post-processing and signal logging
  std::cout << "\nFinished spinning...\n";
  signalLogger.~SignalLoggerTsif();

  std::cout << "\n--------------------------\nClosing down process..." << std::endl;
}
