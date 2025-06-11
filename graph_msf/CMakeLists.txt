cmake_minimum_required(VERSION 3.16)
project(graph_msf)

message("Building ${PROJECT_NAME} library -----------------------------")

## Compile as C++17, supported in ROS Noetic and newer
add_compile_options(-std=c++17)

# If build variables not set, set them to default values
if (NOT DEFINED CMAKE_BUILD_LIBDIR)
    set(CMAKE_BUILD_LIBDIR ${CMAKE_BINARY_DIR}/lib)
endif ()
if (NOT DEFINED CMAKE_BUILD_BINDIR)
    set(CMAKE_BUILD_BINDIR ${CMAKE_BINARY_DIR}/bin)
endif ()
if (NOT DEFINED CMAKE_BUILD_INCLUDE_DIR)
    set(CMAKE_BUILD_INCLUDE_DIR ${CMAKE_BINARY_DIR}/include)
endif ()
message("CMAKE_BUILD_LIBDIR: ${CMAKE_BUILD_LIBDIR}")
message("CMAKE_BUILD_BINDIR: ${CMAKE_BUILD_BINDIR}")
message("CMAKE_BUILD_INCLUDE_DIR: ${CMAKE_BUILD_INCLUDE_DIR}")

# Find dependencies ----------------------------------------------------------------------------------------------------
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(GTSAM_UNSTABLE REQUIRED)

message("Eigen Version:: ${EIGEN3_VERSION_STRING}")
message("Eigen Path:: ${EIGEN3_DIR}")
message("GTSAM Include Path:: ${GTSAM_INCLUDE_DIRS}")

# Color settings for terminal output
if(NOT WIN32)
    string(ASCII 27 Esc)
    set(ColourReset "${Esc}[m")
    set(BoldMagenta "${Esc}[1;35m")
    set(Magenta "${Esc}[35m")
endif ()

###########
## Build ##
###########

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIR}
        ${GTSAM_INCLUDE_DIR}
)

add_library(${PROJECT_NAME} SHARED
        src/lib/eigen_wrapped_gtsam_utils.cpp
        src/lib/FileLogger.cpp
        src/lib/GraphMsf.cpp
        src/lib/GraphMsfClassic.cpp
        src/lib/GraphMsfHolistic.cpp
        src/lib/GraphManager.cpp
        src/lib/ImuBuffer.cpp
        src/lib/Gnss.cpp
        src/lib/GnssHandler.cpp
        src/lib/TimeGraphKeyBuffer.cpp
        src/lib/TrajectoryAlignment.cpp
        src/lib/TrajectoryAlignmentHandler.cpp
        src/lib/NavState.cpp
)
# Exported alias target
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

# Link GTSAM and other dependencies to the graph_msf library
target_link_libraries(${PROJECT_NAME}
        gtsam
        gtsam_unstable
        metis-gtsam
)

# Include directories for the library
target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Add clang tooling
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND AND NOT DEFINED NO_CLANG_TOOLING)
    add_clang_tooling(
        TARGET ${PROJECT_NAME}
        SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
        CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
        CF_FIX
    )
endif()

#############
## Install ##
#############

# Export the include directories and linked libraries
install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets  # Export the target for downstream usage
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install the include directory
install(DIRECTORY include/
        DESTINATION include
)

# Install the export file for downstream projects
install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION share/${PROJECT_NAME}/cmake
)

# Version
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION 1.0.0
  COMPATIBILITY SameMajorVersion
)

# Configure
configure_package_config_file(
  "${PROJECT_NAME}Config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION share/${PROJECT_NAME}/cmake
)

# Files
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION share/${PROJECT_NAME}/cmake
)

message("Finished building ${PROJECT_NAME} library -----------------------------")
