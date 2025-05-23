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

LATEST_CONTAINER_NAME=$(docker ps --latest --format "{{.Names}}")
echo "Name of latest docker container is $LATEST_CONTAINER_NAME. Attaching to it..."
docker exec -it $LATEST_CONTAINER_NAME /entrypoint.sh