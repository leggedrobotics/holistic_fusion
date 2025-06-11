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
# Log into the container as the host user
#==
# set home for host user
export HOME=/home/$HOST_USERNAME
export USER=$HOST_USERNAME
cd $HOME

# Enable sudo access without password
echo "root ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
echo "$HOST_USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Proceed as host user with superuser permissions
sudo -E -u $HOST_USERNAME bash --rcfile /etc/bash.bashrc

# EOF