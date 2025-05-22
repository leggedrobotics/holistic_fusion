cmake_minimum_required(VERSION 3.16)
project(graph_msf)

add_compile_options(-std=c++17)

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
    set(Magenta     "${Esc}[35m")
endif()

include_directories(
    include
    SYSTEM ${GTSAM_INCLUDE_DIR}
    SYSTEM ${GTSAM_UNSTABLE_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${Python3_INCLUDE_DIRS}
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

# Link dependencies explicitly
ament_target_dependencies(${PROJECT_NAME} Eigen3 GTSAM GTSAM_UNSTABLE tf2_eigen Python3)
target_link_libraries(${PROJECT_NAME} gtsam gtsam_unstable)

# Optional: Clang tooling
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND AND NOT DEFINED NO_CLANG_TOOLING)
    add_clang_tooling(
        TARGET ${PROJECT_NAME}
        SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
        CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
        CF_FIX
    )
endif()

# Install targets and directories --------------------------------------------------------------------------------------
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME}
)

# Export package settings for ament ------------------------------------------------------------------------------------
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(Eigen3 GTSAM GTSAM_UNSTABLE tf2_eigen Python3)

ament_package()
