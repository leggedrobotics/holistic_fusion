#!/bin/bash

#=============================================================================
# Copyright (C) 2025, Robotic Systems Lab, ETH Zurich
# All rights reserved.
# http://www.rsl.ethz.ch
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================
# Authors: Julian Nubert, nubertj@ethz.ch
#          Lorenzo Terenzi, lterenzi@ethz.ch
#=============================================================================

# Exits if error occurs
set -e

# Image name
IMAGE="rslethz/holistic_fusion_ros1:latest"

# Entry point
ENTRYPOINT="/entrypoint.sh"

# Graphical output
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth

# Create symlinks to user configs within the build context.
mkdir -p .etc && cd .etc
ln -sf /etc/passwd .
ln -sf /etc/shadow .
ln -sf /etc/group .
cd ..

# Launch a container from the prebuilt image.
echo -e "[run.sh]: \e[1;32mRunning docker image '$IMAGE' with entrypoint $ENTRYPOINT.\e[0m"
echo "---------------------"
RUN_COMMAND="docker run \
  $ARGS \
  -v /lib/modules:/lib/modules \
  --volume=$XSOCK:$XSOCK:rw \
  --volume=$XAUTH:$XAUTH:rw \
  --env="QT_X11_NO_MITSHM=1" \
  --env="XAUTHORITY=$XAUTH" \
  --env="DISPLAY=$DISPLAY" \
  --cap-add=ALL \
  --privileged \
  --net=host \
  -eHOST_USERNAME=$(whoami) \
  -v$HOME:$HOME \
  -v$(pwd)/.etc/shadow:/etc/shadow \
  -v$(pwd)/.etc/passwd:/etc/passwd \
  -v$(pwd)/.etc/group:/etc/group \
  -v/etc/localtime:/etc/localtime:ro \
  --entrypoint=$ENTRYPOINT
  --shm-size=2gb
  -it $IMAGE"

# Final command
echo -e "[run.sh]: \e[1;32mThe final run command is\n\e[0;35m$RUN_COMMAND\e[0m."
$RUN_COMMAND

# EOF