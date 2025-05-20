cmake_minimum_required(VERSION 3.10)
project(graph_msf_catkin)

find_package(catkin REQUIRED)

# Color
if (NOT WIN32)
    string(ASCII 27 Esc)
    set(ColourReset "${Esc}[m")
    set(BoldMagenta "${Esc}[1;35m")
    set(Magenta "${Esc}[35m")
endif ()

# Print CMAKE_BINARY_DIR
message("CMAKE_BINARY_DIR: ${CMAKE_BINARY_DIR}")
message("CMAKE_SOURCE_DIR: ${CMAKE_SOURCE_DIR}")
# Print CMAKE_DEVEL_PREFIX
message("CATKIN_DEVEL_PREFIX: ${CATKIN_DEVEL_PREFIX}")
# Print CATKIN_GLOBAL_INCLUDE_DESTINATION
message("CATKIN_GLOBAL_INCLUDE_DESTINATION: ${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
# Print CATKIN_GLOBAL_LIB_DESTINATION
message("CATKIN_GLOBAL_LIB_DESTINATION: ${CATKIN_GLOBAL_LIB_DESTINATION}")

# Graph MSF
include(ExternalProject)
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

ExternalProject_Add(graph_msf
  SOURCE_DIR ${CMAKE_SOURCE_DIR}/../../graph_msf
  PREFIX      "${CMAKE_SOURCE_DIR}"
  TMP_DIR     "${CMAKE_BINARY_DIR}/graph_msf/tmp"
  BINARY_DIR  "${CMAKE_BINARY_DIR}/graph_msf/build"
  INSTALL_DIR "${CATKIN_DEVEL_PREFIX}"
  CMAKE_CACHE_ARGS "-DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=true"
  CMAKE_ARGS
    -DCMAKE_BUILD_TYPE=Release
    -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}
  BUILD_ALWAYS 1
)

# Dependencies ---------------------------------------------------------
add_dependencies(graph_msf ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

#############
## INSTALL ##
#############

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        )

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/
        DESTINATION ${CATKIN_GLOBAL_LIB_DESTINATION}
        )

# Final catkin package --------------------------------------------------
catkin_package(
    INCLUDE_DIRS
        ${CATKIN_DEVEL_PREFIX}/include
    LIBRARIES
        ${CATKIN_DEVEL_PREFIX}/lib/libgraph_msf.so
        ${CS_PROJECT_LIBRARIES}
    CATKIN_DEPENDS
        ${${PROJECT_NAME}_CATKIN_RUN_DEPENDS}
        ${CATKIN_PACKAGE_DEPENDENCIES}
)
