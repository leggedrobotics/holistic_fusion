/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef HOLISTIC_FUSION_TrajectoryAlignmentHandler_H
#define HOLISTIC_FUSION_TrajectoryAlignmentHandler_H

// C++
#include <Eigen/Eigen>
// Package
#include "holistic_fusion/geometry/Trajectory.h"
#include "holistic_fusion/trajectory_alignment/TrajectoryAlignment.h"

// Defined macros
#define GREEN_START "\033[92m"
#define YELLOW_START "\033[33m"
#define COLOR_END "\033[0m"

namespace holistic_fusion {

class TrajectoryAlignmentHandler {
 public:
  TrajectoryAlignmentHandler();

  // Methods
  void initHandler();
  void addSe3Position(Eigen::Vector3d position, double time);
  void addR3Position(Eigen::Vector3d position, double time);
  bool alignTrajectories(double& yaw, Eigen::Isometry3d& se3);

  // Setters
  void setR3Rate(const double& r3Rate);
  void setSe3Rate(const double& se3Rate);
  void setMinDistanceHeadingInit(const double& minDistanceHeadingInit);
  void setNoMovementDistance(const double& noMovementDistance);
  void setNoMovementTime(const double& noMovementTime);

  // Getters
  std::vector<std::pair<double, Eigen::Vector3d>> getSe3Trajectory();
  std::vector<std::pair<double, Eigen::Vector3d>> getR3Trajectory();

 private:
  // Member variables
  TrajectoryAlignment trajectoryAlignment_;
};

}  // namespace holistic_fusion

#endif  // HOLISTIC_FUSION_TrajectoryAlignmentHandler_H
