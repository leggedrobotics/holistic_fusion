// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from holistic_fusion_ros2_msgs:srv/OfflineOptimizationTrigger.idl
// generated code does not contain a copyright notice

#ifndef HOLISTIC_FUSION_ROS2_MSGS__SRV__DETAIL__OFFLINE_OPTIMIZATION_TRIGGER__STRUCT_HPP_
#define HOLISTIC_FUSION_ROS2_MSGS__SRV__DETAIL__OFFLINE_OPTIMIZATION_TRIGGER__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__holistic_fusion_ros2_msgs__srv__OfflineOptimizationTrigger_Request __attribute__((deprecated))
#else
# define DEPRECATED__holistic_fusion_ros2_msgs__srv__OfflineOptimizationTrigger_Request __declspec(deprecated)
#endif

namespace holistic_fusion_ros2_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct OfflineOptimizationTrigger_Request_
{
  using Type = OfflineOptimizationTrigger_Request_<ContainerAllocator>;

  explicit OfflineOptimizationTrigger_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->max_optimization_iterations = 0ll;
    }
  }

  explicit OfflineOptimizationTrigger_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->max_optimization_iterations = 0ll;
    }
  }

  // field types and members
  using _max_optimization_iterations_type =
    int64_t;
  _max_optimization_iterations_type max_optimization_iterations;

  // setters for named parameter idiom
  Type & set__max_optimization_iterations(
    const int64_t & _arg)
  {
    this->max_optimization_iterations = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    holistic_fusion_ros2_msgs::srv::OfflineOptimizationTrigger_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const holistic_fusion_ros2_msgs::srv::OfflineOptimizationTrigger_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<holistic_fusion_ros2_msgs::srv::OfflineOptimizationTrigger_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<holistic_fusion_ros2_msgs::srv::OfflineOptimizationTrigger_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      holistic_fusion_ros2_msgs::srv::OfflineOptimizationTrigger_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<holistic_fusion_ros2_msgs::srv::OfflineOptimizationTrigger_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      holistic_fusion_ros2_msgs::srv::OfflineOptimizationTrigger_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<holistic_fusion_ros2_msgs::srv::OfflineOptimizationTrigger_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<holistic_fusion_ros2_msgs::srv::OfflineOptimizationTrigger_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<holistic_fusion_ros2_msgs::srv::OfflineOptimizationTrigger_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__holistic_fusion_ros2_msgs__srv__OfflineOptimizationTrigger_Request
    std::shared_ptr<holistic_fusion_ros2_msgs::srv::OfflineOptimizationTrigger_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__holistic_fusion_ros2_msgs__srv__OfflineOptimizationTrigger_Request
    std::shared_ptr<holistic_fusion_ros2_msgs::srv::OfflineOptimizationTrigger_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OfflineOptimizationTrigger_Request_ & other) const
  {
    if (this->max_optimization_iterations != other.max_optimization_iterations) {
      return false;
    }
    return true;
  }
  bool operator!=(const OfflineOptimizationTrigger_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OfflineOptimizationTrigger_Request_

// alias to use template instance with default allocator
using OfflineOptimizationTrigger_Request =
  holistic_fusion_ros2_msgs::srv::OfflineOptimizationTrigger_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace holistic_fusion_ros2_msgs


#ifndef _WIN32
# define DEPRECATED__holistic_fusion_ros2_msgs__srv__OfflineOptimizationTrigger_Response __attribute__((deprecated))
#else
# define DEPRECATED__holistic_fusion_ros2_msgs__srv__OfflineOptimizationTrigger_Response __declspec(deprecated)
#endif

namespace holistic_fusion_ros2_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct OfflineOptimizationTrigger_Response_
{
  using Type = OfflineOptimizationTrigger_Response_<ContainerAllocator>;

  explicit OfflineOptimizationTrigger_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit OfflineOptimizationTrigger_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    holistic_fusion_ros2_msgs::srv::OfflineOptimizationTrigger_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const holistic_fusion_ros2_msgs::srv::OfflineOptimizationTrigger_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<holistic_fusion_ros2_msgs::srv::OfflineOptimizationTrigger_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<holistic_fusion_ros2_msgs::srv::OfflineOptimizationTrigger_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      holistic_fusion_ros2_msgs::srv::OfflineOptimizationTrigger_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<holistic_fusion_ros2_msgs::srv::OfflineOptimizationTrigger_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      holistic_fusion_ros2_msgs::srv::OfflineOptimizationTrigger_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<holistic_fusion_ros2_msgs::srv::OfflineOptimizationTrigger_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<holistic_fusion_ros2_msgs::srv::OfflineOptimizationTrigger_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<holistic_fusion_ros2_msgs::srv::OfflineOptimizationTrigger_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__holistic_fusion_ros2_msgs__srv__OfflineOptimizationTrigger_Response
    std::shared_ptr<holistic_fusion_ros2_msgs::srv::OfflineOptimizationTrigger_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__holistic_fusion_ros2_msgs__srv__OfflineOptimizationTrigger_Response
    std::shared_ptr<holistic_fusion_ros2_msgs::srv::OfflineOptimizationTrigger_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OfflineOptimizationTrigger_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const OfflineOptimizationTrigger_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OfflineOptimizationTrigger_Response_

// alias to use template instance with default allocator
using OfflineOptimizationTrigger_Response =
  holistic_fusion_ros2_msgs::srv::OfflineOptimizationTrigger_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace holistic_fusion_ros2_msgs

namespace holistic_fusion_ros2_msgs
{

namespace srv
{

struct OfflineOptimizationTrigger
{
  using Request = holistic_fusion_ros2_msgs::srv::OfflineOptimizationTrigger_Request;
  using Response = holistic_fusion_ros2_msgs::srv::OfflineOptimizationTrigger_Response;
};

}  // namespace srv

}  // namespace holistic_fusion_ros2_msgs

#endif  // HOLISTIC_FUSION_ROS2_MSGS__SRV__DETAIL__OFFLINE_OPTIMIZATION_TRIGGER__STRUCT_HPP_
