/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#include <functional>
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
    heading2Topic_ = declare_parameter<std::string>("heading2_topic", "/gt_box/cpt7/heading2");
    initialYawTopic_ = declare_parameter<std::string>("initial_yaw_topic", "/gnss/initial_yaw");
    sensorFrameId_ = declare_parameter<std::string>("sensor_frame_id", "cpt7_imu");
    headingYawOffsetDeg_ = declare_parameter<double>("heading_yaw_offset_deg", 0.0);

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    initialYawPub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(initialYawTopic_, qos);
    heading2Sub_ = create_subscription<novatel_oem7_msgs::msg::HEADING2>(
        heading2Topic_, qos, std::bind(&NovatelOem7AdapterNode::heading2Callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "NovAtel heading adapter frame_id=%s: %s -> %s",
                sensorFrameId_.c_str(), heading2Topic_.c_str(), initialYawTopic_.c_str());
  }

 private:
  void heading2Callback(const novatel_oem7_msgs::msg::HEADING2::ConstSharedPtr msg) {
    if (!novatel_oem7_adapter::hasValidHeading2(*msg)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Skipping HEADING2: solution is not SOL_COMPUTED, position type is NONE, or heading fields are invalid.");
      return;
    }

    geometry_msgs::msg::PoseWithCovarianceStamped initialYaw;
    novatel_oem7_adapter::fillInitialYawFromHeading2(*msg, headingYawOffsetDeg_, sensorFrameId_, initialYaw);
    if (isZeroStamp(initialYaw.header.stamp)) {
      initialYaw.header.stamp = now();
    }
    initialYawPub_->publish(initialYaw);
  }

  std::string heading2Topic_;
  std::string initialYawTopic_;
  std::string sensorFrameId_;
  double headingYawOffsetDeg_ = 0.0;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialYawPub_;
  rclcpp::Subscription<novatel_oem7_msgs::msg::HEADING2>::SharedPtr heading2Sub_;
};

}  // namespace b2w_se

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<b2w_se::NovatelOem7AdapterNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
