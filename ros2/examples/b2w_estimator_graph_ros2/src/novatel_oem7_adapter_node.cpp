/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#include <cassert>
#include <cstring>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "b2w_estimator_graph_ros2/NovatelOem7AdapterConversions.h"

namespace b2w_se {
namespace {

bool isZeroStamp(const builtin_interfaces::msg::Time& stamp) {
  return stamp.sec == 0 && stamp.nanosec == 0;
}

}  // namespace

class NovatelOem7AdapterNode final : public rclcpp::Node {
 public:
  explicit NovatelOem7AdapterNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("novatel_oem7_adapter", options) {
    bestposTopic_ = declare_parameter<std::string>("bestpos_topic", "/novatel/oem7/bestpos");
    heading2Topic_ = declare_parameter<std::string>("heading2_topic", "/novatel/oem7/heading2");
    navSatFixTopic_ = declare_parameter<std::string>("navsatfix_topic", "/navsatfix");
    initialYawTopic_ = declare_parameter<std::string>("initial_yaw_topic", "/gnss/initial_yaw");
    frameId_ = declare_parameter<std::string>("frame_id", "gnss");
    useEllipsoidAltitude_ = declare_parameter<bool>("use_ellipsoid_altitude", true);
    headingYawOffsetDeg_ = declare_parameter<double>("heading_yaw_offset_deg", 0.0);

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    navSatFixPub_ = create_publisher<sensor_msgs::msg::NavSatFix>(navSatFixTopic_, qos);
    initialYawPub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(initialYawTopic_, qos);

    bestposSub_ = create_subscription<novatel_oem7_msgs::msg::BESTPOS>(
        bestposTopic_, qos, std::bind(&NovatelOem7AdapterNode::bestposCallback, this, std::placeholders::_1));
    heading2Sub_ = create_subscription<novatel_oem7_msgs::msg::HEADING2>(
        heading2Topic_, qos, std::bind(&NovatelOem7AdapterNode::heading2Callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "NovAtel adapter: %s -> %s, %s -> %s",
                bestposTopic_.c_str(), navSatFixTopic_.c_str(),
                heading2Topic_.c_str(), initialYawTopic_.c_str());
  }

 private:
  void bestposCallback(const novatel_oem7_msgs::msg::BESTPOS::ConstSharedPtr msg) {
    if (!novatel_oem7_adapter::hasValidBestpos(*msg)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Skipping BESTPOS: solution is not SOL_COMPUTED, position type is NONE, or numeric fields are invalid.");
      return;
    }

    sensor_msgs::msg::NavSatFix navSatFix;
    novatel_oem7_adapter::fillNavSatFixFromBestpos(*msg, useEllipsoidAltitude_, frameId_, navSatFix);
    if (isZeroStamp(navSatFix.header.stamp)) {
      navSatFix.header.stamp = now();
    }
    navSatFixPub_->publish(navSatFix);
  }

  void heading2Callback(const novatel_oem7_msgs::msg::HEADING2::ConstSharedPtr msg) {
    if (!novatel_oem7_adapter::hasValidHeading2(*msg)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Skipping HEADING2: solution is not SOL_COMPUTED, position type is NONE, or heading fields are invalid.");
      return;
    }

    geometry_msgs::msg::PoseWithCovarianceStamped initialYaw;
    novatel_oem7_adapter::fillInitialYawFromHeading2(*msg, headingYawOffsetDeg_, frameId_, initialYaw);
    if (isZeroStamp(initialYaw.header.stamp)) {
      initialYaw.header.stamp = now();
    }
    initialYawPub_->publish(initialYaw);
  }

  std::string bestposTopic_;
  std::string heading2Topic_;
  std::string navSatFixTopic_;
  std::string initialYawTopic_;
  std::string frameId_;
  bool useEllipsoidAltitude_ = true;
  double headingYawOffsetDeg_ = 0.0;

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navSatFixPub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialYawPub_;
  rclcpp::Subscription<novatel_oem7_msgs::msg::BESTPOS>::SharedPtr bestposSub_;
  rclcpp::Subscription<novatel_oem7_msgs::msg::HEADING2>::SharedPtr heading2Sub_;
};

}  // namespace b2w_se

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<b2w_se::NovatelOem7AdapterNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
