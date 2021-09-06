cmake_minimum_required(VERSION 3.12)
project(fg_filtering)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  m545_msgs
  roscpp
  rospy
  std_msgs
  tf
  eigen_conversions
  tf_conversions
  message_generation
  std_srvs
  nodelet
  message_filters
  #geodetic_utils
  signal_logger
  excavator_model
  m545_description_ros
  robot_utils
  fg_filtering_log_msgs
)

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
message("Python Version:" ${Python3_VERSION})
  message("Python Path:" ${Python3_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
message("Eigen Version:" ${EIGEN3_VERSION_STRING})
message("Eigen Path:" ${Eigen3_DIR})

find_package(PCL REQUIRED COMPONENTS common io)
message("PCL Version:" ${PCL_VERSION})
message("PCL Path:" ${PCL_DIR})

find_package(GTSAM REQUIRED)
message("GTSAM version: ${GTSAM_VERSION}")
message("GTSAM path: ${GTSAM_DIR}")

include_directories(
  include
	${catkin_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs tf pcl_conversions eigen_conversions tf_conversions message_runtime nodelet message_filters
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

## Compile as C++14, supported in ROS Kinetic and newer
# set_property(TARGET invz_player PROPERTY CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Turn on if needed otherwise makes multiScanRegistration node crash
# if(NOT CMAKE_CROSSCOMPILING)
#   add_definitions(-march=native)
# endif()

add_subdirectory(src/lib)

add_executable(${PROJECT_NAME}_node src/fg_filtering_node.cpp include/fg_filtering/Datatypes.hpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

add_executable(leicamap_publisher_node src/pointcloud_publisher_node.cpp)
target_link_libraries(leicamap_publisher_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

# Add clang tooling
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND AND NOT DEFINED NO_CLANG_TOOLING)
  add_clang_tooling(
    TARGET ${PROJECT_NAME}
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_FIX
  )
endif(cmake_clang_tools_FOUND AND NOT DEFINED NO_CLANG_TOOLING)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)