cmake_minimum_required(VERSION 3.10)
project(graph_msf_catkin)

## Compile as C++17, supported in ROS Noetic and newer
add_compile_options(-std=c++17)

find_package(catkin REQUIRED)

# Color
if(NOT WIN32)
    string(ASCII 27 Esc)
    set(ColourReset "${Esc}[m")
    set(BoldMagenta "${Esc}[1;35m")
    set(Magenta "${Esc}[35m")
endif()

# Print CMAKE_BINARY_DIR
message("CMAKE_BINARY_DIR: ${CMAKE_BINARY_DIR}")
message("CMAKE_SOURCE_DIR: ${CMAKE_SOURCE_DIR}")
# Print CMAKE_DEVEL_PREFIX
message("CATKIN_DEVEL_PREFIX: ${CATKIN_DEVEL_PREFIX}")
# Print CATKIN_GLOBAL_INCLUDE_DESTINATION
message("CATKIN_GLOBAL_INCLUDE_DESTINATION: ${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
# Print CATKIN_GLOBAL_LIB_DESTINATION
message("CATKIN_GLOBAL_LIB_DESTINATION: ${CATKIN_GLOBAL_LIB_DESTINATION}")

# Ensure CATKIN_DEVEL_PREFIX is set, otherwise use a fallback
if(NOT CATKIN_DEVEL_PREFIX)
    set(CATKIN_DEVEL_PREFIX "${CMAKE_BINARY_DIR}/devel")
endif()

# Add the non-Catkin package ----------------------------------------------
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)
add_subdirectory(${CMAKE_SOURCE_DIR}/../../graph_msf
    ${CATKIN_DEVEL_PREFIX}/lib
)

# Move the include directory to the correct location
add_custom_target(copy_graph_msf ALL
    # Include
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CATKIN_DEVEL_PREFIX}/include
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_SOURCE_DIR}/../../graph_msf/include/graph_msf"
            "${CATKIN_DEVEL_PREFIX}/include/graph_msf"
)

# GTSAM --> Also expose
find_package(GTSAM REQUIRED)
find_package(GTSAM_UNSTABLE REQUIRED)

#############
## INSTALL ##
#############

# Install the include directory
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/.private/graph_msf_catkin/include
        DESTINATION ${CATKIN_DEVEL_PREFIX}/include/graph_msf
)

# Install the library
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/.private/graph_msf_catkin/lib
        DESTINATION ${CATKIN_DEVEL_PREFIX}/lib
        FILES_MATCHING PATTERN "*.so"
)

# Final catkin package --------------------------------------------------
catkin_package(
    INCLUDE_DIRS
        ${CATKIN_DEVEL_PREFIX}/include
        ${GTSAM_INCLUDE_DIR}
    LIBRARIES
        ${CATKIN_DEVEL_PREFIX}/lib/libgraph_msf.so
        ${CS_PROJECT_LIBRARIES}
        gtsam
        gtsam_unstable
        metis-gtsam
    CATKIN_DEPENDS
        ${${PROJECT_NAME}_CATKIN_RUN_DEPENDS}
        ${CATKIN_PACKAGE_DEPENDENCIES}
)

