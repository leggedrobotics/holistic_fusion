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

###########
## Build ##
###########

# # Create an INTERFACE library for the headers
# add_library(${PROJECT_NAME}_INTERFACE INTERFACE)
# # Exported alias target
# add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME}_INTERFACE)

# # Include directories for the library
# target_include_directories(${PROJECT_NAME}_INTERFACE
#   INTERFACE
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>
# )

# # Link dependencies (for plain CMake usage)
# target_link_libraries(${PROJECT_NAME}_INTERFACE
#   INTERFACE
#     nav_msgs::nav_msgs__rosidl_generator_cpp
#     std_msgs::std_msgs__rosidl_generator_cpp
#     rosidl_typesupport_cpp
# )

#############
## Install ##
#############

# # Export the include directories and linked libraries
# install(
#     TARGETS ${PROJECT_NAME}_INTERFACE
#     EXPORT ${PROJECT_NAME}Targets  # Export the target for downstream usage
#     ARCHIVE DESTINATION lib
#     LIBRARY DESTINATION lib
#     RUNTIME DESTINATION bin
# )

# Install the include directory
install(DIRECTORY include/
        DESTINATION include
)

# # Install the export file for downstream projects
# install(EXPORT ${PROJECT_NAME}Targets
#     FILE ${PROJECT_NAME}Targets.cmake
#     NAMESPACE ${PROJECT_NAME}::
#     DESTINATION share/${PROJECT_NAME}/cmake
# )

# # Version
# include(CMakePackageConfigHelpers)
# write_basic_package_version_file(
#   "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
#   VERSION 1.0.0
#   COMPATIBILITY SameMajorVersion
# )

# # Configure
# configure_package_config_file(
#   "${PROJECT_NAME}Config.cmake.in"
#   "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
#   INSTALL_DESTINATION share/${PROJECT_NAME}/cmake
# )

# # Files
# install(FILES
#   "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
#   "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
#   DESTINATION share/${PROJECT_NAME}/cmake
# )

# # Package export and installation
# ament_export_include_directories(include)
# ament_export_targets(${PROJECT_NAME}Targets)
ament_export_dependencies(rosidl_default_runtime nav_msgs std_msgs)
ament_package()
