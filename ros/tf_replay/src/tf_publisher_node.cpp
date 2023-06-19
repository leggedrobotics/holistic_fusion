#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
  transform.setRotation(tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,
                                       msg->pose.pose.orientation.w));
  br.sendTransform(tf::StampedTransform(transform.inverse(), msg->header.stamp, "imu_link", "odom_ct"));
  std::cout << "Publishing..." << std::endl;
}

int main(int argc, char** argv) {
  std::cout << "Initializing..." << std::endl;

  ros::init(argc, argv, "odom_to_tf");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/estimate_trajectory/odometry", 1, odometryCallback);

  std::cout << "Spinning..." << std::endl;

  ros::spin();

  return 0;
}
