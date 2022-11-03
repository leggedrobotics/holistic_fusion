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
#include <memory>
// Package
#include "graph_msf/config/TrajectoryAlignmentConfig.h"
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
  void setGnssRate(const int gnssRate) { gnssRate_ = gnssRate; }
  void setLidarRate(const int lidarRate) { lidarRate_ = lidarRate; }
  void setMinDistanceHeadingInit(const double minDistanceHeadingInit) { minDistanceHeadingInit_ = minDistanceHeadingInit; }
  void setNoMovementDistance(const double noMovementDistance) { noMovementDistance_ = noMovementDistance; }
  void setNoMovementTime(const double noMovementTime) { noMovementTime_ = noMovementTime; }

 private:
  // Member methods
  bool associateTrajectories(Trajectory& trajectoryA, Trajectory& trajectoryB, Trajectory& newTrajectoryA, Trajectory& newTrajectoryB);
  bool trajectoryAlignment(Trajectory& trajectoryA, Trajectory& trajectoryB, Eigen::Matrix4d& transform);

  // Member variables
  Trajectory gnssTrajectory_;
  Trajectory lidarTrajectory_;

  // Reference Parameters
  int gnssRate_;
  int lidarRate_;
  double minDistanceHeadingInit_;
  double noMovementDistance_;
  double noMovementTime_;
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_TrajectoryAlignment_H
