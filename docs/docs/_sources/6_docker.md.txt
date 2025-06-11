# Docker

We provide a dockerfile with ROS Noetic and all the dependencies installed.

## Building the Docker Image

To build the docker image, simply run:

```bash
docker/bin/build_ros1_noetic_image.sh
```

## Running the Docker Container

To run the docker container, you can use the following command:

```bash
docker/bin/run_ros1_noetic_container.sh
```

This will start a new container from the `$USER` (not sudo) to avoid permission issues with the workspace.
The whole `$HOME` folder is shared with the container, so you can access and develop inside your workspace.
