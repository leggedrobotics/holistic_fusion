// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from graph_msf_ros2_msgs:srv/OfflineOptimizationTrigger.idl
// generated code does not contain a copyright notice

#ifndef GRAPH_MSF_ROS2_MSGS__SRV__DETAIL__OFFLINE_OPTIMIZATION_TRIGGER__TRAITS_HPP_
#define GRAPH_MSF_ROS2_MSGS__SRV__DETAIL__OFFLINE_OPTIMIZATION_TRIGGER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "graph_msf_ros2_msgs/srv/detail/offline_optimization_trigger__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace graph_msf_ros2_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const OfflineOptimizationTrigger_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: max_optimization_iterations
  {
    out << "max_optimization_iterations: ";
    rosidl_generator_traits::value_to_yaml(msg.max_optimization_iterations, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const OfflineOptimizationTrigger_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: max_optimization_iterations
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_optimization_iterations: ";
    rosidl_generator_traits::value_to_yaml(msg.max_optimization_iterations, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const OfflineOptimizationTrigger_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace graph_msf_ros2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use graph_msf_ros2_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const graph_msf_ros2_msgs::srv::OfflineOptimizationTrigger_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  graph_msf_ros2_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use graph_msf_ros2_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const graph_msf_ros2_msgs::srv::OfflineOptimizationTrigger_Request & msg)
{
  return graph_msf_ros2_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<graph_msf_ros2_msgs::srv::OfflineOptimizationTrigger_Request>()
{
  return "graph_msf_ros2_msgs::srv::OfflineOptimizationTrigger_Request";
}

template<>
inline const char * name<graph_msf_ros2_msgs::srv::OfflineOptimizationTrigger_Request>()
{
  return "graph_msf_ros2_msgs/srv/OfflineOptimizationTrigger_Request";
}

template<>
struct has_fixed_size<graph_msf_ros2_msgs::srv::OfflineOptimizationTrigger_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<graph_msf_ros2_msgs::srv::OfflineOptimizationTrigger_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<graph_msf_ros2_msgs::srv::OfflineOptimizationTrigger_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace graph_msf_ros2_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const OfflineOptimizationTrigger_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const OfflineOptimizationTrigger_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const OfflineOptimizationTrigger_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace graph_msf_ros2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use graph_msf_ros2_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const graph_msf_ros2_msgs::srv::OfflineOptimizationTrigger_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  graph_msf_ros2_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use graph_msf_ros2_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const graph_msf_ros2_msgs::srv::OfflineOptimizationTrigger_Response & msg)
{
  return graph_msf_ros2_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<graph_msf_ros2_msgs::srv::OfflineOptimizationTrigger_Response>()
{
  return "graph_msf_ros2_msgs::srv::OfflineOptimizationTrigger_Response";
}

template<>
inline const char * name<graph_msf_ros2_msgs::srv::OfflineOptimizationTrigger_Response>()
{
  return "graph_msf_ros2_msgs/srv/OfflineOptimizationTrigger_Response";
}

template<>
struct has_fixed_size<graph_msf_ros2_msgs::srv::OfflineOptimizationTrigger_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<graph_msf_ros2_msgs::srv::OfflineOptimizationTrigger_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<graph_msf_ros2_msgs::srv::OfflineOptimizationTrigger_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<graph_msf_ros2_msgs::srv::OfflineOptimizationTrigger>()
{
  return "graph_msf_ros2_msgs::srv::OfflineOptimizationTrigger";
}

template<>
inline const char * name<graph_msf_ros2_msgs::srv::OfflineOptimizationTrigger>()
{
  return "graph_msf_ros2_msgs/srv/OfflineOptimizationTrigger";
}

template<>
struct has_fixed_size<graph_msf_ros2_msgs::srv::OfflineOptimizationTrigger>
  : std::integral_constant<
    bool,
    has_fixed_size<graph_msf_ros2_msgs::srv::OfflineOptimizationTrigger_Request>::value &&
    has_fixed_size<graph_msf_ros2_msgs::srv::OfflineOptimizationTrigger_Response>::value
  >
{
};

template<>
struct has_bounded_size<graph_msf_ros2_msgs::srv::OfflineOptimizationTrigger>
  : std::integral_constant<
    bool,
    has_bounded_size<graph_msf_ros2_msgs::srv::OfflineOptimizationTrigger_Request>::value &&
    has_bounded_size<graph_msf_ros2_msgs::srv::OfflineOptimizationTrigger_Response>::value
  >
{
};

template<>
struct is_service<graph_msf_ros2_msgs::srv::OfflineOptimizationTrigger>
  : std::true_type
{
};

template<>
struct is_service_request<graph_msf_ros2_msgs::srv::OfflineOptimizationTrigger_Request>
  : std::true_type
{
};

template<>
struct is_service_response<graph_msf_ros2_msgs::srv::OfflineOptimizationTrigger_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // GRAPH_MSF_ROS2_MSGS__SRV__DETAIL__OFFLINE_OPTIMIZATION_TRIGGER__TRAITS_HPP_
