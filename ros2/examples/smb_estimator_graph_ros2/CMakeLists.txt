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
find_package(Glog REQUIRED)

# Check if the target from graph_msf is already defined
if(NOT TARGET graph_msf::graph_msf)
  find_package(graph_msf REQUIRED)
endif()
if(NOT TARGET graph_msf_ros2::graph_msf_ros2)
  find_package(graph_msf_ros2 REQUIRED)
endif()

# Non propagated ros dependencies
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

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
    ${std_srvs_INCLUDE_DIRS}
    ${visualization_msgs_INCLUDE_DIRS}
)

# Library
add_library(${PROJECT_NAME}
    src/lib/SmbEstimator.cpp
    src/lib/readParams.cpp
    src/lib/SmbStaticTransforms.cpp
)

# Link dependencies
target_link_libraries(${PROJECT_NAME}
    graph_msf_ros2::graph_msf_ros2
    ${GLOG_LIBRARIES}
    ${gflags_LIBRARIES}
)

# Print the include directories for debugging
get_target_property(INC_DIR graph_msf::graph_msf INTERFACE_INCLUDE_DIRECTORIES)
message("Include dir for graph_msf::graph_msf: ${INC_DIR}")
get_target_property(INC_DIR graph_msf_ros2::graph_msf_ros2 INTERFACE_INCLUDE_DIRECTORIES)
message("Include dir for graph_msf_ros2::graph_msf_ros2: ${INC_DIR}")

# Ament dependencies
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    std_msgs
    sensor_msgs
    nav_msgs
    tf2
    tf2_ros
    Glog
    gflags
)

# Executable
add_executable(${PROJECT_NAME}_node src/smb_estimator_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
)

ament_target_dependencies(${PROJECT_NAME}_node
    rclcpp
    std_msgs
    sensor_msgs
    nav_msgs
    tf2
    tf2_ros
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

# Export include directories
ament_export_include_directories(include)

# Finalize ament package
ament_package()
