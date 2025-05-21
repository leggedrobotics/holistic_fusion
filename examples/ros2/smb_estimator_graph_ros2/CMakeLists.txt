cmake_minimum_required(VERSION 3.16)
project(smb_estimator_graph_ros2)

## Compile as C++17
add_compile_options(-std=c++17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# For FindGlog.cmake and FindGflags.cmake
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}")

# Find Dependencies ---------------------------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(graph_msf_ros2 REQUIRED)
find_package(graph_msf REQUIRED)
find_package(Glog REQUIRED)

# Display Eigen version and path
message("Eigen Version: ${EIGEN3_VERSION_STRING}")
message("Eigen Path: ${Eigen3_DIR}")

# Color
if(NOT WIN32)
    string(ASCII 27 Esc)
    set(ColourReset "${Esc}[m")
    set(BoldMagenta "${Esc}[1;35m")
    set(Magenta "${Esc}[35m")
endif()

# Define include directories
include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
    ${GLOG_INCLUDE_DIRS}
    ${graph_msf_ros2_INCLUDE_DIRS}
    ${graph_msf_INCLUDE_DIRS}
)

# Library
add_library(${PROJECT_NAME}
    src/lib/SmbEstimator.cpp
    src/lib/readParams.cpp
    src/lib/SmbStaticTransforms.cpp
    include/smb_estimator_graph_ros2/constants.h
)

target_link_libraries(${PROJECT_NAME}
    ${gflags_LIBRARIES}
    ${GLOG_LIBRARIES}
    ${graph_msf_LIBRARIES}
    ${graph_msf_ros2_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    std_msgs
    sensor_msgs
    nav_msgs
    tf2
    tf2_ros
    graph_msf
    graph_msf_ros2
)

# Executable
add_executable(${PROJECT_NAME}_node src/smb_estimator_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
    ${gflags_LIBRARIES}
    ${GLOG_LIBRARIES}
    ${graph_msf_LIBRARIES}
    ${graph_msf_ros2_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}_node
    rclcpp
    std_msgs
    sensor_msgs
    nav_msgs
    tf2
    tf2_ros
    graph_msf
    graph_msf_ros2
)

# Add clang tooling
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND AND NOT DEFINED NO_CLANG_TOOLING)
    add_clang_tooling(
        TARGET ${PROJECT_NAME}
        SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
        CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
        CF_FIX
    )
endif()

# Install targets
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME}
)

# Mark other files for installation
install(DIRECTORY launch rviz config
    DESTINATION share/${PROJECT_NAME}
)

# Finalize ament package
ament_package()
