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
find_package(rosidl_default_generators REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(graph_msf_ros2_msgs REQUIRED
  COMPONENTS rosidl_generator_c rosidl_typesupport_cpp
)


# Check if the target from graph_msf is already defined
if(NOT TARGET graph_msf::graph_msf)
  find_package(graph_msf REQUIRED)
endif()

# Color (Optional)
if (NOT WIN32)
    string(ASCII 27 Esc)
    set(ColourReset "${Esc}[m")
    set(BoldMagenta "${Esc}[1;35m")
    set(Magenta "${Esc}[35m")
endif ()

###########
## Build ##
###########

# Include directories ---------------------------------------------------
include_directories(
    include
)

# Library ----------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/lib/StaticTransformsTf.cpp
  src/lib/GraphMsfRos2.cpp
  src/lib/readParams.cpp
  src/lib/conversions.cpp
)
# Exported alias target
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

# Link dependencies
ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    std_srvs
    visualization_msgs
    rosidl_default_runtime
    graph_msf_ros2_msgs
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    Eigen3::Eigen
    rclcpp::rclcpp
    tf2::tf2
    tf2_ros::tf2_ros
    tf2_eigen::tf2_eigen
    graph_msf::graph_msf
)

# Include directories for the library
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Print the include directories for debugging
get_target_property(INC_DIR graph_msf::graph_msf INTERFACE_INCLUDE_DIRECTORIES)
message("Include dir for graph_msf::graph_msf: ${INC_DIR}")

# Python Scripts ----------------------------------------------------------------
install(PROGRAMS
  ${PROJECT_NAME}_py/replay/manual_pose_files_to_tf_and_odom_bag.py
  ${PROJECT_NAME}_py/plotting/plot_latest_quantitites_in_folder.py
  ${PROJECT_NAME}_py/bag_filter/remove_tf_from_bag.py
  DESTINATION lib/${PROJECT_NAME}
)

#############
## Install ##
#############

# Export the include directories and linked libraries
install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets  # Export the target for downstream usage
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install the include directory
install(DIRECTORY include/
        DESTINATION include
)

# Install the export file for downstream projects
install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION share/${PROJECT_NAME}/cmake
)

# Version
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION 1.0.0
  COMPATIBILITY SameMajorVersion
)

# Configure
configure_package_config_file(
  "${PROJECT_NAME}Config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION share/${PROJECT_NAME}/cmake
)

# Files
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION share/${PROJECT_NAME}/cmake
)

# Export ament package
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME}Targets)
ament_export_dependencies(
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_eigen
  std_srvs
  visualization_msgs
  rosidl_default_runtime
  graph_msf_ros2_msgs
)

ament_package()
