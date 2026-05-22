cmake_minimum_required(VERSION 3.16)
project(b2w_estimator_graph_ros2)

## Compile as C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Keep default Release flags and append what you need
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG")
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE ON)

# Global extra flags (avoid repeating -std and -O3 here)
add_compile_options(-mtune=native -funroll-loops -fopenmp)

# For FindGlog.cmake and FindGflags.cmake
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}")

# Find Dependencies ---------------------------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(novatel_oem7_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Glog REQUIRED)
find_package(graph_msf_ros2)
find_package(graph_msf_ros2_msgs REQUIRED
  COMPONENTS rosidl_generator_c rosidl_typesupport_cpp
)

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
    src/lib/B2WEstimator.cpp
    src/lib/readParams.cpp
    src/lib/B2WStaticTransforms.cpp
)

add_library(${PROJECT_NAME}_novatel_adapter_conversions
    src/lib/NovatelOem7AdapterConversions.cpp
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
    geometry_msgs
    tf2
    tf2_ros
    Glog
    gflags
    graph_msf_ros2
    graph_msf_ros2_msgs
)

ament_target_dependencies(${PROJECT_NAME}_novatel_adapter_conversions
    geometry_msgs
    novatel_oem7_msgs
)

# Executable
add_executable(${PROJECT_NAME}_node src/b2w_estimator_node.cpp)
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

add_executable(novatel_oem7_adapter_node src/novatel_oem7_adapter_node.cpp)
target_link_libraries(novatel_oem7_adapter_node
    ${PROJECT_NAME}_novatel_adapter_conversions
)

ament_target_dependencies(novatel_oem7_adapter_node
    geometry_msgs
    novatel_oem7_msgs
    rclcpp
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
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_novatel_adapter_conversions
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}_node novatel_oem7_adapter_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME}
)

# Mark other files for installation
install(DIRECTORY launch rviz config
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_novatel_oem7_adapter_conversions
        test/test_novatel_oem7_adapter_conversions.cpp
    )
    if(TARGET test_novatel_oem7_adapter_conversions)
        target_link_libraries(test_novatel_oem7_adapter_conversions
            ${PROJECT_NAME}_novatel_adapter_conversions
        )
        ament_target_dependencies(test_novatel_oem7_adapter_conversions
            geometry_msgs
            novatel_oem7_msgs
        )
    endif()
endif()

# Export include directories
ament_export_include_directories(include)

# Finalize ament package
ament_package()
