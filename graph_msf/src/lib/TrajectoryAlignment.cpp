/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// C++
#include <iostream>
// Package
#include "graph_msf/trajectory_alignment/TrajectoryAlignment.h"

namespace graph_msf {

// Public -------------------------------------------------------------------
TrajectoryAlignment::TrajectoryAlignment() {
  std::cout << YELLOW_START << "TrajectoryAlignment" << GREEN_START << " Created Trajectory Alignment instance." << COLOR_END << std::endl;
}

void TrajectoryAlignment::addLidarPose(Eigen::Vector3d position, double time) {
  lidarTrajectory_.addPose(position, time);
}

void TrajectoryAlignment::addGnssPose(Eigen::Vector3d position, double time) {
  gnssTrajectory_.addPose(position, time);
}

bool TrajectoryAlignment::associateTrajectories(Trajectory& trajectoryA, Trajectory& trajectoryB, Trajectory& newTrajectoryA,
                                                Trajectory& newTrajectoryB) {
  // matching trajectories with their timestamps.
  bool swapped = false;
  if (trajectoryA.poses().size() < trajectoryB.poses().size()) {
    std::swap(trajectoryA, trajectoryB);
    swapped = true;
  }

  int indexB = 0;
  int matchingIndexA = -1;
  int lastMatchingIndexA = 0;
  bool matched = false;
  for (const auto& poseB : trajectoryB.poses()) {  // shorter trajectory.
    int indexA = 0;
    double diff = std::numeric_limits<double>::max();
    for (auto it = trajectoryA.poses().begin() + lastMatchingIndexA; it != trajectoryA.poses().end(); ++it) {
      if ((abs(poseB.time() - it->time())) < diff && abs((poseB.time() - it->time())) < 0.1) {
        diff = poseB.time() - it->time();
        matchingIndexA = indexA;
        lastMatchingIndexA = matchingIndexA;
        matched = true;
      }
      ++indexA;
    }

    if (matched) {
      newTrajectoryA.addPose(trajectoryA.poses().at(matchingIndexA));
      newTrajectoryB.addPose(trajectoryB.poses().at(indexB));
      matched = false;
    }
    ++indexB;
  }

  if (swapped) {
    std::swap(newTrajectoryA, newTrajectoryB);
  }

  return true;
}

bool TrajectoryAlignment::trajectoryAlignment(Trajectory& trajectoryA, Trajectory& trajectoryB, Eigen::Matrix4d& transform) {
  // fill matrices to use Eigen Umeyama function.
  const int numberOfMeasurements = trajectoryA.poses().size();
  if (numberOfMeasurements < 2) return false;

  Eigen::MatrixXd posesA;
  Eigen::MatrixXd posesB;
  posesA.resize(3, numberOfMeasurements);
  posesB.resize(3, numberOfMeasurements);

  for (unsigned i = 0; i < numberOfMeasurements; ++i) {
    posesA.col(i) = trajectoryA.poses().at(i).position() - trajectoryA.poses().at(0).position();
    posesB.col(i) = trajectoryB.poses().at(i).position() - trajectoryB.poses().at(0).position();
  }

  // Umeyama Alignment.
  transform = umeyama(posesB, posesA, false);
  std::cout << YELLOW_START << "Trajectory Alignment" << GREEN_START << " Umeyama transform: " << std::endl
            << COLOR_END << transform << std::endl;

  return true;
}

bool TrajectoryAlignment::alignTrajectories(double& yaw) {
  if (lidarTrajectory_.distance() < minDistanceHeadingInit_) return false;
  if (gnssTrajectory_.distance() < minDistanceHeadingInit_) return false;

  // aligin trajectories
  Trajectory newLidarTrajectory;
  Trajectory newGnssTrajectory;
  if (!associateTrajectories(lidarTrajectory_, gnssTrajectory_, newLidarTrajectory, newGnssTrajectory)) {
    std::cout << "TrajectoryAlignment::initializeYaw associateTrajectories failed." << std::endl;
    return false;
  }

  Eigen::Matrix4d transform;
  if (!trajectoryAlignment(newLidarTrajectory, newGnssTrajectory, transform)) {
    std::cout << "TrajectoryAlignment::initializeYaw trajectoryAlignment failed." << std::endl;
    return false;
  }

  Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);
  Eigen::Vector3d eulerAngles = rotation.eulerAngles(2, 1, 0).reverse();
  yaw = eulerAngles(2);
  std::cout << YELLOW_START << "Trajectory Alignment" << GREEN_START << " Initial Roll/Pitch/Yaw(deg):" << COLOR_END
            << eulerAngles.transpose() * 180 / M_PI << std::endl;

  return true;
}

}  // namespace graph_msf
