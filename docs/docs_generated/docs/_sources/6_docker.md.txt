# Docker

We provide Dockerfiles for both ROS1 Noetic (Ubuntu 20.04) and ROS2 Humble (Ubuntu 22.04) with all dependencies installed.

## ROS1 Noetic

### Building the Docker Image

To build the docker image, simply run:

```bash
docker/bin/build_ros1_noetic_image.sh
```

### Running the Docker Container

To run the docker container, you can use the following command:

```bash
docker/bin/run_ros1_noetic_container.sh
```

This will start a new container from the `$USER` (not sudo) to avoid permission issues with the workspace.
The whole `$HOME` folder is shared with the container, so you can access and develop inside your workspace.
The repository is mounted into `/ros_ws/src/holistic_fusion`, and third-party dependencies are automatically imported on first startup via `vcs import`.
The catkin workspace is initialized with `cmake-args -DCMAKE_BUILD_TYPE=Release`.

## ROS2 Humble

### Building the Docker Image

To build the docker image, simply run:

```bash
docker/bin/build_ros2_humble_image.sh
```

### Running the Docker Container

To run the docker container, you can use the following command:

```bash
docker/bin/run_ros2_humble_container.sh
```

This will start a new container from the `$USER` (not sudo) to avoid permission issues with the workspace.
The whole `$HOME` folder is shared with the container, so you can access and develop inside your workspace.
The repository is mounted into `/ros2_ws/src/holistic_fusion`, and third-party dependencies are automatically imported on first startup via `vcs import`.

A convenience function `colcon_build_up_to` is available inside the container, e.g.:

```bash
colcon_build_up_to smb_estimator_graph_ros2
```

This wraps `colcon build --symlink-install --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=RelWithDebInfo --event-handlers console_direct+ --packages-up-to <package>`.
