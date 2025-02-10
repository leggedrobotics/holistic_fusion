cmake_minimum_required(VERSION 3.16)
project(gtsam_tryout_examples)

## Compile as C++17, supported in ROS Noetic and newer
add_compile_options(-std=c++17)

# Find Dependencies ---------------------------------------------------------------------------------------------------
find_package(Eigen3 REQUIRED)
message("Eigen Version:" ${EIGEN3_VERSION_STRING})
message("Eigen Path:" ${Eigen3_DIR})

# Color
if (NOT WIN32)
    string(ASCII 27 Esc)
    set(ColourReset "${Esc}[m")
    set(BoldMagenta "${Esc}[1;35m")
    set(Magenta "${Esc}[35m")
endif ()

# March native check
#include(CheckCXXCompilerFlag)
#CHECK_CXX_COMPILER_FLAG("-march=native" COMPILER_SUPPORTS_MARCH_NATIVE)
#if(COMPILER_SUPPORTS_MARCH_NATIVE)
#  add_compile_options("-march=native")
#  message("${BoldMagenta}INFO: Using -march=native${ColourReset}")
#endif()

# Catkin dependencies -------------------------------------------------------------------------------------------------
set(CATKIN_PACKAGE_DEPENDENCIES
        gtsam_catkin
        graph_msf_ros
        nav_msgs
        sensor_msgs
        roscpp
        std_msgs
        tf
)

find_package(catkin REQUIRED COMPONENTS
        ${CATKIN_PACKAGE_DEPENDENCIES}
)

catkin_package(
        INCLUDE_DIRS
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS ${CATKIN_PACKAGE_DEPENDENCIES}
)

###########
## Build ##
###########
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

# Library
#add_library(${PROJECT_NAME}
#        src/lib/SmbEstimator.cpp
#        src/lib/readParams.cpp
#        src/lib/SmbStaticTransforms.cpp
#        include/smb_estimator_graph/constants.h)
#target_link_libraries(${PROJECT_NAME}
#        ${catkin_LIBRARIES})

# 1: Alignment Executable
add_executable(${PROJECT_NAME}_alignment_node src/alignment_tryout_main.cpp)
target_link_libraries(${PROJECT_NAME}_alignment_node ${catkin_LIBRARIES})

# 2: Fixed Lag Smoother Invalid Key
add_executable(${PROJECT_NAME}_fixed_lag_smoother_invalid_key_node src/fixed_lag_smoother_invalid_key_main.cpp)
target_link_libraries(${PROJECT_NAME}_fixed_lag_smoother_invalid_key_node ${catkin_LIBRARIES})

#add_dependencies(${PROJECT_NAME}
#        ${catkin_EXPORTED_TARGETS})

# Add clang tooling
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND AND NOT DEFINED NO_CLANG_TOOLING)
    add_clang_tooling(
            TARGET ${PROJECT_NAME}
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_FIX
    )
endif (cmake_clang_tools_FOUND AND NOT DEFINED NO_CLANG_TOOLING)

#############
## Install ##
#############

#install(TARGETS ${PROJECT_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY config launch rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
