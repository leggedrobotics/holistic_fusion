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

#==
# Foundation
#==
ARG UBUNTU_VERSION=20.04
FROM ubuntu:${UBUNTU_VERSION}

# Suppresses interactive calls to APT
ENV DEBIAN_FRONTEND="noninteractive"

# Updates
RUN apt update && apt upgrade -y

# ----------------------------------------------------------------------------

#==
# System APT base dependencies and utilities
#==

COPY docker/submodules/base.sh /home/base.sh
RUN chmod +x /home/base.sh
RUN /home/base.sh && rm /home/base.sh

#==
# ROS
#==

# Version
ARG ROS=noetic

COPY docker/submodules/ros_1.sh /home/ros_1.sh
RUN chmod +x /home/ros_1.sh
RUN /home/ros_1.sh && rm /home/ros_1.sh

#==
# GTSAM
#==

COPY docker/submodules/gtsam.sh /home/gtsam.sh
RUN chmod +x /home/gtsam.sh
RUN /home/gtsam.sh && rm /home/gtsam.sh

# ----------------------------------------------------------------------------

#==
# Permissions
#==
RUN chmod ugo+rwx /software

#==
# Environment
#==
COPY docker/ros1_noetic/bashrc /etc/bash.bashrc
RUN chmod a+rwx /etc/bash.bashrc

#==
# Workspace
#==
RUN mkdir -p /ros_ws/src \
 && chmod -R ugo+rwx /ros_ws

#==
# Pre-build catkin workspace (also for CI)
#==
COPY catkin_workspace.vcs /holistic_fusion_prebuilt/holistic_fusion/catkin_workspace.vcs
COPY . /holistic_fusion_prebuilt/holistic_fusion/src/holistic_fusion
RUN mkdir -p /holistic_fusion_prebuilt/holistic_fusion/src \
 && cd /holistic_fusion_prebuilt/holistic_fusion/src \
 && vcs import . < /holistic_fusion_prebuilt/holistic_fusion/catkin_workspace.vcs \
 && cd /holistic_fusion_prebuilt/holistic_fusion \
 && /bin/bash -c "source /opt/ros/noetic/setup.bash \
    && catkin init \
    && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && catkin build graph_msf_ros_examples" \
 && chmod -R ugo+rwx /holistic_fusion_prebuilt

# ----------------------------------------------------------------------------

#==
# Execution
#==

COPY docker/ros1_noetic/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
#ENTRYPOINT ["/entrypoint.sh"]
CMD []

# EOF