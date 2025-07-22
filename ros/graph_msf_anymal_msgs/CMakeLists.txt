# Minimum cmake version
cmake_minimum_required(VERSION 3.5.1)

# Create project
project(graph_msf_anymal_msgs)

# Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    geometry_msgs
    std_msgs
)

# Declare the message and service files to be built
add_message_files(
  FILES
    AnymalState.msg
    Contact.msg
    ExtendedJointState.msg
)
generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
)

# Generate a package
catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    std_msgs
)
