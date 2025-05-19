cmake_minimum_required(VERSION 3.16)
project(graph_msf_ros2)

# Compile as C++17
add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(graph_msf REQUIRED)

# Color (Optional)
if (NOT WIN32)
    string(ASCII 27 Esc)
    set(ColourReset "${Esc}[m")
    set(BoldMagenta "${Esc}[1;35m")
    set(Magenta "${Esc}[35m")
endif ()

# Include directories
include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
    ${graph_msf_INCLUDE_DIRS}
)

# Library
add_library(${PROJECT_NAME} SHARED
  src/lib/StaticTransformsTf.cpp
  src/lib/StaticTransformsUrdf.cpp
  src/lib/GraphMsfRos2.cpp
  src/lib/readParams.cpp
  src/lib/conversions.cpp
)

# Link dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  urdf
  kdl_parser
  tf2_eigen
  std_srvs
  visualization_msgs
  graph_msf
  rosidl_default_runtime
)

# Python Scripts ----------------------------------------------------------------
install(PROGRAMS
  graph_msf_ros2_py/replay/manual_pose_files_to_tf_and_odom_bag.py
  graph_msf_ros2_py/plotting/plot_latest_quantitites_in_folder.py
  graph_msf_ros2_py/bag_filter/remove_tf_from_bag.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers, config, and other files --------------------------------------
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export package
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  urdf
  kdl_parser
  tf2_eigen
  std_srvs
  visualization_msgs
  graph_msf
  Eigen3
  rosidl_default_runtime
)

ament_package()
