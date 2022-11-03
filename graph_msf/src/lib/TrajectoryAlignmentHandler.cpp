/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// C++
#include <iostream>
// Package
#include "graph_msf/trajectory_alignment/TrajectoryAlignmentHandler.h"

namespace graph_msf {

// Public -------------------------------------------------------------------
TrajectoryAlignmentHandler::TrajectoryAlignmentHandler() {
  std::cout << YELLOW_START << "TrajectoryAlignmentHandler" << GREEN_START << " Created Trajectory Alignment Handler instance." << COLOR_END
            << std::endl;
}

void TrajectoryAlignmentHandler::initHandler() {
  std::cout << YELLOW_START << "TrajectoryAlignmentHandler" << GREEN_START << " Initializing the handler." << COLOR_END << std::endl;
}

void TrajectoryAlignmentHandler::addLidarPose(Eigen::Vector3d position, double time) {
  trajectoryAlignment_.addLidarPose(position, time);
}

void TrajectoryAlignmentHandler::addGnssPose(Eigen::Vector3d position, double time) {
  trajectoryAlignment_.addGnssPose(position, time);
}

void TrajectoryAlignmentHandler::setGnssRate(const int& gnssRate) {
  trajectoryAlignment_.setGnssRate(gnssRate);
}

void TrajectoryAlignmentHandler::setLidarRate(const int& lidarRate) {
  trajectoryAlignment_.setLidarRate(lidarRate);
}

void TrajectoryAlignmentHandler::setMinDistanceHeadingInit(const double& minDistanceHeadingInit) {
  trajectoryAlignment_.setMinDistanceHeadingInit(minDistanceHeadingInit);
}

void TrajectoryAlignmentHandler::setNoMovementDistance(const double& noMovementDistance) {
  trajectoryAlignment_.setNoMovementDistance(noMovementDistance);
}

void TrajectoryAlignmentHandler::setNoMovementTime(const double& noMovementTime) {
  trajectoryAlignment_.setNoMovementTime(noMovementTime);
}

bool TrajectoryAlignmentHandler::alignTrajectories(double& yaw) {
  return trajectoryAlignment_.alignTrajectories(yaw);
}

}  // namespace graph_msf
