# Usage

The core library is plain C++ and can be used in any C++ project with CMake support.

Moreover, we provide catkin and ament/colcon wrapper packages to conveniently use the library in ROS and ROS2 workspaces.

## ROS1 Workspace

For using `graph_msf` in a ROS1 workspace, we provide a package called `graph_msf_catkin`.
This package is exposing all `graph_msf` functionalities to the workspace, which can be used to compile the library with the catkin build system.
To use `graph_msf` in a ROS1 workspace, you can directly include the `graph_msf_catkin` in you `package.xml` and `CMakeLists.txt` files as follows:

Package.xml:
```xml
<build_depend>graph_msf_catkin</build_depend>
<exec_depend>graph_msf_catkin</exec_depend>
```

CMakeLists.txt:
```cmake
# Set the dependencies for your package
set(CATKIN_PACKAGE_DEPENDENCIES
        graph_msf_catkin
        # Add other dependencies here
)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
        ${CATKIN_PACKAGE_DEPENDENCIES}
        message_generation
)

# Catkin Package
catkin_package(
        CATKIN_DEPENDS ${CATKIN_PACKAGE_DEPENDENCIES}
        DEPENDS EIGEN3
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
)

# Includes
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        # Add other include directories here
)

# Link against graph_msf and its dependencies
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
```

## ROS2 Workspace
For using `graph_msf` in a ROS2 workspace, we provide a package called `graph_msf_ros2`.
This package is exposing all `graph_msf` functionalities to the workspace, which can be used to compile the library with the ament/colcon build system.
To use `graph_msf` in a ROS2 workspace, you can directly include `graph_msf_ros2` in your `package.xml` and `CMakeLists.txt` files as follows:

Package.xml:
```xml
<depend>graph_msf</depend>
<depend>graph_msf_ros2</depend>
```

CMakeLists.txt:
```cmake
# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(graph_msf_ros2 REQUIRED)

# Check if the target from graph_msf is already defined
if(NOT TARGET graph_msf::graph_msf)
  find_package(graph_msf REQUIRED)
endif()
if(NOT TARGET graph_msf_ros2::graph_msf_ros2)
  find_package(graph_msf_ros2 REQUIRED)
endif()

# Includes
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Library
add_library(${PROJECT_NAME}
  # Add your source files here
)

# Link against graph_msf and its dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  graph_msf_ros2
  # Add other dependencies here
)
target_link_libraries(${PROJECT_NAME}
  graph_msf::graph_msf
)

# Install
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```