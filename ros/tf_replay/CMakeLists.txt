cmake_minimum_required(VERSION 3.16)
project(tf_replay)

## Compile as C++17, supported in ROS Noetic and newer
add_compile_options(-std=c++17)

# Color
if(NOT WIN32)
  string(ASCII 27 Esc)
  set(ColourReset "${Esc}[m")
  set(BoldMagenta "${Esc}[1;35m")
  set(Magenta     "${Esc}[35m")
endif()

# Catkin
set(CATKIN_PACKAGE_DEPENDENCIES
  roscpp
  nav_msgs
  tf
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

catkin_package(
  CATKIN_DEPENDS ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS EIGEN3
  INCLUDE_DIRS
  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${Python3_INCLUDE_DIRS}
)

# Library
add_executable(${PROJECT_NAME}_node
        src/tf_publisher_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

# Add clang tooling
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND AND NOT DEFINED NO_CLANG_TOOLING)
  add_clang_tooling(
    TARGET ${PROJECT_NAME}_node
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

install(DIRECTORY
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
