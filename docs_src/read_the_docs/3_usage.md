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
Under construction.