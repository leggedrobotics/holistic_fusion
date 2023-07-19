cmake_minimum_required(VERSION 3.16)
project(graph_msf_ros_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  nav_msgs
)

#add_action_files(
#  DIRECTORY action
#  FILES
#  Scan.action
#)

add_message_files(
  DIRECTORY msg
  FILES
  Wgs84Coordinate.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  GetPathInEnu.srv
)

generate_messages(
  DEPENDENCIES
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
