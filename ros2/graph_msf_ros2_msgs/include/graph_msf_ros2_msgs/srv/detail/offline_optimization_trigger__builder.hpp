// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from graph_msf_ros2_msgs:srv/OfflineOptimizationTrigger.idl
// generated code does not contain a copyright notice

#ifndef GRAPH_MSF_ROS2_MSGS__SRV__DETAIL__OFFLINE_OPTIMIZATION_TRIGGER__BUILDER_HPP_
#define GRAPH_MSF_ROS2_MSGS__SRV__DETAIL__OFFLINE_OPTIMIZATION_TRIGGER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "graph_msf_ros2_msgs/srv/detail/offline_optimization_trigger__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace graph_msf_ros2_msgs
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
  ::graph_msf_ros2_msgs::srv::OfflineOptimizationTrigger_Request max_optimization_iterations(::graph_msf_ros2_msgs::srv::OfflineOptimizationTrigger_Request::_max_optimization_iterations_type arg)
  {
    msg_.max_optimization_iterations = std::move(arg);
    return std::move(msg_);
  }

private:
  ::graph_msf_ros2_msgs::srv::OfflineOptimizationTrigger_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::graph_msf_ros2_msgs::srv::OfflineOptimizationTrigger_Request>()
{
  return graph_msf_ros2_msgs::srv::builder::Init_OfflineOptimizationTrigger_Request_max_optimization_iterations();
}

}  // namespace graph_msf_ros2_msgs


namespace graph_msf_ros2_msgs
{

namespace srv
{

namespace builder
{

class Init_OfflineOptimizationTrigger_Response_message
{
public:
  explicit Init_OfflineOptimizationTrigger_Response_message(::graph_msf_ros2_msgs::srv::OfflineOptimizationTrigger_Response & msg)
  : msg_(msg)
  {}
  ::graph_msf_ros2_msgs::srv::OfflineOptimizationTrigger_Response message(::graph_msf_ros2_msgs::srv::OfflineOptimizationTrigger_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::graph_msf_ros2_msgs::srv::OfflineOptimizationTrigger_Response msg_;
};

class Init_OfflineOptimizationTrigger_Response_success
{
public:
  Init_OfflineOptimizationTrigger_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OfflineOptimizationTrigger_Response_message success(::graph_msf_ros2_msgs::srv::OfflineOptimizationTrigger_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_OfflineOptimizationTrigger_Response_message(msg_);
  }

private:
  ::graph_msf_ros2_msgs::srv::OfflineOptimizationTrigger_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::graph_msf_ros2_msgs::srv::OfflineOptimizationTrigger_Response>()
{
  return graph_msf_ros2_msgs::srv::builder::Init_OfflineOptimizationTrigger_Response_success();
}

}  // namespace graph_msf_ros2_msgs

#endif  // GRAPH_MSF_ROS2_MSGS__SRV__DETAIL__OFFLINE_OPTIMIZATION_TRIGGER__BUILDER_HPP_
