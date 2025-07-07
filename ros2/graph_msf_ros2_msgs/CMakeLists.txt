cmake_minimum_required(VERSION 3.16)
project(graph_msf_ros2_msgs)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Message and Service Files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Wgs84Coordinate.msg"
  "srv/GetPathInEnu.srv"
  "srv/OfflineOptimizationTrigger.srv"
  DEPENDENCIES nav_msgs std_msgs
)

# Package export and installation
ament_export_dependencies(rosidl_default_runtime nav_msgs std_msgs)
ament_package()
