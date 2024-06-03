cmake_minimum_required(VERSION 3.16)
project(graph_msf_ros)

## Compile as C++17, supported in ROS Noetic and newer
add_compile_options(-std=c++17)

# Find dependencies ------------------------------------------------------------
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

# Catkin
set(CATKIN_PACKAGE_DEPENDENCIES
        roscpp
        graph_msf
        kdl_parser
        nav_msgs
        geometry_msgs
        tf
)

find_package(catkin REQUIRED COMPONENTS
        ${CATKIN_PACKAGE_DEPENDENCIES}
        message_generation
)

add_service_files(
        FILES
        OfflineOptimizationTrigger.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
        CATKIN_DEPENDS ${CATKIN_PACKAGE_DEPENDENCIES}
        DEPENDS EIGEN3
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
        ${EIGEN3_INCLUDE_DIR}
        ${Python3_INCLUDE_DIRS}
)

# Library
add_library(${PROJECT_NAME}
        src/lib/StaticTransformsTf.cpp
        src/lib/StaticTransformsUrdf.cpp
        src/lib/GraphMsfRos.cpp
        src/lib/readParams.cpp
        src/lib/conversions.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

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

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY config rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
