cmake_minimum_required(VERSION 3.0.2)
project(fg_filtering)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  tf
  pcl_conversions
  eigen_conversions
  tf_conversions
  message_generation
  std_srvs
  nodelet
  message_filters
  loam
)

find_package(Eigen3 REQUIRED)
message("Eigen Version:" ${EIGEN3_VERSION_STRING})
message("Eigen Path:" ${Eigen3_DIR})

find_package(PCL REQUIRED)
message("PCL Version:" ${PCL_VERSION})
message("PCL Path:" ${PCL_DIR})

find_package(GTSAM REQUIRED)
message("GTSAM version: ${GTSAM_VERSION}")
message("GTSAM path: ${GTSAM_DIR}")

include_directories(
  include
  ${loam_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
	#${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

add_message_files(
  DIRECTORY msg
  FILES
  OptStatus.msg
)

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

add_executable(${PROJECT_NAME}_node src/fg_filtering_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)