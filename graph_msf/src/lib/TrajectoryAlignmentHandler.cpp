/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
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

std::vector<std::pair<double, Eigen::Vector3d>> TrajectoryAlignmentHandler::getSe3Trajectory() {
  return trajectoryAlignment_.getSe3Trajectory();
}

std::vector<std::pair<double, Eigen::Vector3d>> TrajectoryAlignmentHandler::getR3Trajectory() {
  return trajectoryAlignment_.getR3Trajectory();
}

void TrajectoryAlignmentHandler::addSe3Position(Eigen::Vector3d position, double time) {
  trajectoryAlignment_.addSe3Position(position, time);
}

void TrajectoryAlignmentHandler::addR3Position(Eigen::Vector3d position, double time) {
  trajectoryAlignment_.addR3Position(position, time);
}

void TrajectoryAlignmentHandler::setR3Rate(const double& r3Rate) {
  trajectoryAlignment_.setR3Rate(r3Rate);
}

void TrajectoryAlignmentHandler::setSe3Rate(const double& se3Rate) {
  trajectoryAlignment_.setSe3Rate(se3Rate);
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

bool TrajectoryAlignmentHandler::alignTrajectories(double& yaw, Eigen::Isometry3d& se3) {
  bool success = trajectoryAlignment_.alignTrajectories(yaw, se3);
  return success;
}

}  // namespace graph_msf
