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
# Authors: Vassilios Tsounis, tsounisv@ethz.ch
#          Julian Nubert, nubertj@ethz.ch
#=============================================================================
figlet holistic_fusion
#==
# Set up the catkin workspace
#==
# Create symlink to the repo and import third-party dependencies
if [ ! -d /ros_ws/src/kindr ]; then
    echo "[entrypoint.sh]: Importing third-party dependencies into /ros_ws/src..."
    cd /ros_ws/src && vcs import . < $REPO_DIR/catkin_workspace.vcs
fi
# Initialize catkin workspace if not yet done
if [ ! -f /ros_ws/.catkin_tools/profiles/default/config.yaml ]; then
    echo "[entrypoint.sh]: Initializing catkin workspace..."
    cd /ros_ws && source /opt/ros/noetic/setup.bash \
      && catkin init \
      && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
fi

# Ensure the host user owns the workspace
chown -R $HOST_USERNAME:$HOST_USERNAME /ros_ws

#==
# Log into the container as the host user
#==
# set home for host user
export HOME=/home/$HOST_USERNAME
export USER=$HOST_USERNAME
cd /ros_ws

# Enable sudo access without password
echo "root ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
echo "$HOST_USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Proceed as host user with superuser permissions
sudo -E -u $HOST_USERNAME bash --rcfile /etc/bash.bashrc

# EOF