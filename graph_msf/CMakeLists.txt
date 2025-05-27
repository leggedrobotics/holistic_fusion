cmake_minimum_required(VERSION 3.16)
project(graph_msf)

message("Building graph_msf library -----------------------------")

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
find_package(ament_cmake REQUIRED)
find_package(GTSAM REQUIRED)
find_package(GTSAM_UNSTABLE REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Development)
find_package(tf2_eigen REQUIRED)

message("Eigen Version:: ${EIGEN3_VERSION_STRING}")
message("Eigen Path:: ${EIGEN3_DIR}")
message("GTSAM Include Path:: ${GTSAM_INCLUDE_DIRS}")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Color settings for terminal output
if(NOT WIN32)
    string(ASCII 27 Esc)
    set(ColourReset "${Esc}[m")
    set(BoldMagenta "${Esc}[1;35m")
    set(Magenta "${Esc}[35m")
endif ()

# Eigen
find_package(Eigen3 REQUIRED COMPONENTS)
# GTSAM
find_package(GTSAM REQUIRED)
find_package(GTSAM_UNSTABLE REQUIRED)
message("GTSAM Version:" ${GTSAM_VERSION})
message("GTSAM Path:" ${GTSAM_DIR})

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

# Link GTSAM and other dependencies to the graph_msf library
target_link_libraries(${PROJECT_NAME}
        gtsam
        gtsam_unstable
        metis-gtsam
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

# Export the include directories and linked libraries
install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}-targets  # Export the target for downstream usage
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Install the export file for downstream projects
install(EXPORT ${PROJECT_NAME}-targets
        FILE ${PROJECT_NAME}Config.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

# Install the include directory
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

# Add GTSAM include directories to the exported interface
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}>
    ${EIGEN3_INCLUDE_DIR}
)

message("Finished building graph_msf library -----------------------------")
