// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from holistic_fusion_ros2_msgs:srv/OfflineOptimizationTrigger.idl
// generated code does not contain a copyright notice

#ifndef HOLISTIC_FUSION_ROS2_MSGS__SRV__DETAIL__OFFLINE_OPTIMIZATION_TRIGGER__BUILDER_HPP_
#define HOLISTIC_FUSION_ROS2_MSGS__SRV__DETAIL__OFFLINE_OPTIMIZATION_TRIGGER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "holistic_fusion_ros2_msgs/srv/detail/offline_optimization_trigger__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace holistic_fusion_ros2_msgs
{

namespace srv
{

namespace builder
{

class Init_OfflineOptimizationTrigger_Request_max_optimization_iterations
{
public:
  Init_OfflineOptimizationTrigger_Request_max_optimization_iterations()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::holistic_fusion_ros2_msgs::srv::OfflineOptimizationTrigger_Request max_optimization_iterations(::holistic_fusion_ros2_msgs::srv::OfflineOptimizationTrigger_Request::_max_optimization_iterations_type arg)
  {
    msg_.max_optimization_iterations = std::move(arg);
    return std::move(msg_);
  }

private:
  ::holistic_fusion_ros2_msgs::srv::OfflineOptimizationTrigger_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::holistic_fusion_ros2_msgs::srv::OfflineOptimizationTrigger_Request>()
{
  return holistic_fusion_ros2_msgs::srv::builder::Init_OfflineOptimizationTrigger_Request_max_optimization_iterations();
}

}  // namespace holistic_fusion_ros2_msgs


namespace holistic_fusion_ros2_msgs
{

namespace srv
{

namespace builder
{

class Init_OfflineOptimizationTrigger_Response_message
{
public:
  explicit Init_OfflineOptimizationTrigger_Response_message(::holistic_fusion_ros2_msgs::srv::OfflineOptimizationTrigger_Response & msg)
  : msg_(msg)
  {}
  ::holistic_fusion_ros2_msgs::srv::OfflineOptimizationTrigger_Response message(::holistic_fusion_ros2_msgs::srv::OfflineOptimizationTrigger_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::holistic_fusion_ros2_msgs::srv::OfflineOptimizationTrigger_Response msg_;
};

class Init_OfflineOptimizationTrigger_Response_success
{
public:
  Init_OfflineOptimizationTrigger_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OfflineOptimizationTrigger_Response_message success(::holistic_fusion_ros2_msgs::srv::OfflineOptimizationTrigger_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_OfflineOptimizationTrigger_Response_message(msg_);
  }

private:
  ::holistic_fusion_ros2_msgs::srv::OfflineOptimizationTrigger_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::holistic_fusion_ros2_msgs::srv::OfflineOptimizationTrigger_Response>()
{
  return holistic_fusion_ros2_msgs::srv::builder::Init_OfflineOptimizationTrigger_Response_success();
}

}  // namespace holistic_fusion_ros2_msgs

#endif  // HOLISTIC_FUSION_ROS2_MSGS__SRV__DETAIL__OFFLINE_OPTIMIZATION_TRIGGER__BUILDER_HPP_
