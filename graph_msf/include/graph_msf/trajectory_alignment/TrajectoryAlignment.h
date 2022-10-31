/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_TrajectoryAlignment_H
#define GRAPH_MSF_TrajectoryAlignment_H

// C++
#include <Eigen/Eigen>
// Package
#include "graph_msf/geometry/Trajectory.h"

// Defined macros
#define GREEN_START "\033[92m"
#define YELLOW_START "\033[33m"
#define COLOR_END "\033[0m"

namespace graph_msf {

class TrajectoryAlignment {
 public:
  TrajectoryAlignment();

  // Methods
  void addLidarPose(Eigen::Vector3d position, double time);
  void addGnssPose(Eigen::Vector3d position, double time);
  bool alignTrajectories(double& yaw);

  // Setters
  void setMinDistanceHeadingInit(const double minDistanceHeadingInit) { minDistanceHeadingInit_ = minDistanceHeadingInit; }

 private:
  // Member methods
  bool associateTrajectories(Trajectory& trajectoryA, Trajectory& trajectoryB, Trajectory& newTrajectoryA, Trajectory& newTrajectoryB);
  bool trajectoryAlignment(Trajectory& trajectoryA, Trajectory& trajectoryB, Eigen::Matrix4d& transform);

  // Member variables
  Trajectory gnssTrajectory_;
  Trajectory lidarTrajectory_;

  // Reference Parameters
  double minDistanceHeadingInit_;
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_TrajectoryAlignment_H
