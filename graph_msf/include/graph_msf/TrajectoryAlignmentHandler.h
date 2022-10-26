/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_TrajectoryAlignmentHandler_H
#define GRAPH_MSF_TrajectoryAlignmentHandler_H

// C++
#include <Eigen/Eigen>
// Package
#include "graph_msf/geometry/Trajectory.h"
// Workspace
#include "graph_msf/gnss/Gnss.h"

// Defined macros
#define GREEN_START "\033[92m"
#define YELLOW_START "\033[33m"
#define COLOR_END "\033[0m"

namespace graph_msf {

class TrajectoryAlignmentHandler {
 public:
  TrajectoryAlignmentHandler();

  // Methods
  void initHandler();
  void addLidarPose(Eigen::Vector3d position, double time);
  void addGnssPose(Eigen::Vector3d position, double time);
  bool associateTrajectories(Trajectory& lidarTrajectory, Trajectory& gnssTrajectory, Trajectory& newLidarTrajectory,
                             Trajectory& newGnssTrajectory);
  bool trajectoryAlignment(Trajectory& newLidarTrajectory, Trajectory& newGnssTrajectory, Eigen::Matrix4d& transform);
  bool initializeYaw(double& yaw);
  bool rotationMatrixToEulerAngles(Eigen::Matrix3d& rotation, Eigen::Vector3d& eulerAngles);

  // Flags

  // Setters

  // Getters.
  double getInitYaw();

 private:
  // Member methods

  // Member variables
  Trajectory gnssTrajectory_;
  Trajectory lidarTrajectory_;

  // Reference Parameters
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_TrajectoryAlignmentHandler_H
