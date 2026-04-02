cmake_minimum_required(VERSION 3.10)
project(graph_msf_catkin)

## Compile as C++17, supported in ROS Noetic and newer
add_compile_options(-std=c++17)

set(CATKIN_PACKAGE_DEPENDENCIES
    gtsam_catkin
)

find_package(catkin REQUIRED COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

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

# Force the wrapped native CMake project to use the GTSAM exported by gtsam_catkin.
if(DEFINED gtsam_catkin_PREFIX)
    set(_gtsam_catkin_gtsam_dir "${gtsam_catkin_PREFIX}/lib/cmake/GTSAM")
    set(_gtsam_catkin_gtsam_unstable_dir "${gtsam_catkin_PREFIX}/lib/cmake/GTSAM_UNSTABLE")

    if(NOT EXISTS "${_gtsam_catkin_gtsam_dir}/GTSAMConfig.cmake")
        message(FATAL_ERROR "gtsam_catkin was found, but ${_gtsam_catkin_gtsam_dir}/GTSAMConfig.cmake does not exist.")
    endif()

    if(NOT EXISTS "${_gtsam_catkin_gtsam_unstable_dir}/GTSAM_UNSTABLEConfig.cmake")
        message(FATAL_ERROR "gtsam_catkin was found, but ${_gtsam_catkin_gtsam_unstable_dir}/GTSAM_UNSTABLEConfig.cmake does not exist.")
    endif()

    set(GTSAM_DIR "${_gtsam_catkin_gtsam_dir}" CACHE PATH "GTSAM config directory exported by gtsam_catkin" FORCE)
    set(GTSAM_UNSTABLE_DIR "${_gtsam_catkin_gtsam_unstable_dir}" CACHE PATH "GTSAM unstable config directory exported by gtsam_catkin" FORCE)

    message(STATUS "Using GTSAM from gtsam_catkin: ${GTSAM_DIR}")
    message(STATUS "Using GTSAM_UNSTABLE from gtsam_catkin: ${GTSAM_UNSTABLE_DIR}")
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
    LIBRARIES
        ${CATKIN_DEVEL_PREFIX}/lib/libgraph_msf.so
        ${CS_PROJECT_LIBRARIES}
    CATKIN_DEPENDS
        ${CATKIN_PACKAGE_DEPENDENCIES}
)
