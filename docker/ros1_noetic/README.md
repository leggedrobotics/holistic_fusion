# Docker Setup for Holistic Fusion

We provide docker images for running Holistic Fusion in a controlled environment.
The ROS1 Image is based on Ubuntu 20.04 and includes the necessary dependencies for running Holistic Fusion.
The ROS2 Image is based on Ubuntu 22.04 and will be released in the future.

## ROS1 Noetic Image

To build the ROS1 Noetic image, run the following command from the root of the repository:

```bash
docker/bin/build_ros1_noetic_image.sh
```