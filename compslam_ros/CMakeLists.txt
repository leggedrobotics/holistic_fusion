cmake_minimum_required(VERSION 3.16)
project(compslam_ros)

set(CATKIN_PACKAGE_DEPENDENCIES
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  tf
  tf_conversions
  message_generation
  std_srvs
  nodelet
  message_filters
  signal_logger
  robot_utils
  compslam_se
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Eigen3 REQUIRED)
message("Eigen Version:" ${EIGEN3_VERSION_STRING})
message("Eigen Path:" ${Eigen3_DIR})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

catkin_package(
  CATKIN_DEPENDS ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${GTSAM_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        ${Python3_INCLUDE_DIRS}
)

# Library
add_library(${PROJECT_NAME}
        src/lib/CompslamEstimator.cpp
        src/lib/readParams.cpp
        src/lib/GnssHandler.cpp
        src/lib/StaticTransforms.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Executable
add_executable(${PROJECT_NAME}_node src/estimator_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

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

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)