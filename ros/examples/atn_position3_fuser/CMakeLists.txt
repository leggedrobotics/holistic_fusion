cmake_minimum_required(VERSION 3.16)
project(atn_position3_fuser)

## Compile as C++17, supported in ROS Noetic and newer
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-std=c++17)

#set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# For FindGlog.cmake and FindGflags.cmake
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}")

# Find Dependencies ---------------------------------------------------------------------------------------------------
find_package(Eigen3 REQUIRED)
message("Eigen Version:" ${EIGEN3_VERSION_STRING})
message("Eigen Path:" ${Eigen3_DIR})
find_package(Glog REQUIRED)

# Color
if (NOT WIN32)
    string(ASCII 27 Esc)
    set(ColourReset "${Esc}[m")
    set(BoldMagenta "${Esc}[1;35m")
    set(Magenta "${Esc}[35m")
endif ()

# Catkin dependencies -------------------------------------------------------------------------------------------------
set(CATKIN_PACKAGE_DEPENDENCIES
  graph_msf_catkin
  graph_msf_ros
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  std_msgs
  tf
  message_filters
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

# Python
catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${CATKIN_PACKAGE_DEPENDENCIES}
)

###########
## Build ##
###########
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${GLOG_INCLUDE_DIRS}
        ${gflags_INCLUDE_DIRS}
        ${Python3_INCLUDE_DIRS}
)

# Library
add_library(${PROJECT_NAME}
        src/lib/Position3Estimator.cpp
        src/lib/readParams.cpp
        src/lib/Position3StaticTransforms.cpp
        )
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${gflags_LIBRARIES}
        ${GLOG_LIBRARIES}
)

# Executable
add_executable(${PROJECT_NAME}_node src/graph_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
        ${PROJECT_NAME}
        ${gflags_LIBRARIES}
        ${GLOG_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS})

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

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Python
catkin_install_python(PROGRAMS
        position3_fuser_py/check_gnss_in_bag.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY position3_fuser_py
        DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
        FILES_MATCHING PATTERN "*.py"
)
