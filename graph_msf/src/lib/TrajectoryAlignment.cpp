/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// C++
#include <boost/optional.hpp>
#include <iostream>

// Package
#include "graph_msf/trajectory_alignment/TrajectoryAlignment.h"

namespace graph_msf {

// Public -------------------------------------------------------------------
TrajectoryAlignment::TrajectoryAlignment() {
  std::cout << YELLOW_START << "TrajectoryAlignment" << GREEN_START << " Created Trajectory Alignment instance." << COLOR_END << std::endl;
}

std::vector<std::pair<double, Eigen::Vector3d>> TrajectoryAlignment::getSe3Trajectory() {
  std::vector<std::pair<double, Eigen::Vector3d>> trajectory;
  std::pair<double, Eigen::Vector3d> onePose;
  for (auto pose : se3Trajectory_.poses()) {
    onePose.first = pose.time();
    onePose.second = pose.position();
    trajectory.push_back(onePose);
  }
  return trajectory;
}

std::vector<std::pair<double, Eigen::Vector3d>> TrajectoryAlignment::getR3Trajectory() {
  std::vector<std::pair<double, Eigen::Vector3d>> trajectory;
  std::pair<double, Eigen::Vector3d> onePose;
  for (auto pose : r3Trajectory_.poses()) {
    onePose.first = pose.time();
    onePose.second = pose.position();
    trajectory.push_back(onePose);
  }
  return trajectory;
}

void TrajectoryAlignment::addSe3Position(Eigen::Vector3d position, double time) {
  const std::lock_guard<std::mutex> alignmentLock(alignmentMutex);
  se3Trajectory_.addPose(position, time);
}

void TrajectoryAlignment::addR3Position(Eigen::Vector3d position, double time) {
  const std::lock_guard<std::mutex> alignmentLock(alignmentMutex);
  r3Trajectory_.addPose(position, time);
}

bool TrajectoryAlignment::associateTrajectories(Trajectory& trajectoryA, Trajectory& trajectoryB, Trajectory& newTrajectoryA,
                                                Trajectory& newTrajectoryB) {
  // matching trajectories with their timestamps
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
    for (auto it = trajectoryA.poses().begin(); it != trajectoryA.poses().end(); ++it) {
      if ((abs(poseB.time() - it->time())) < diff && abs((poseB.time() - it->time())) < 0.1) {
        diff = abs(poseB.time() - it->time());
        matchingIndexA = indexA;
        // lastMatchingIndexA = matchingIndexA;
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

bool TrajectoryAlignment::trajectoryAlignment(Trajectory& trajectoryA, Trajectory& trajectoryB, Eigen::Isometry3d& returnTransform) {
  // fill matrices to use Eigen Umeyama function
  const int numberOfMeasurements = trajectoryA.poses().size();
  if (numberOfMeasurements < 2) return false;

  Eigen::MatrixXd posesA;
  Eigen::MatrixXd posesB;
  posesA.resize(3, numberOfMeasurements);
  posesB.resize(3, numberOfMeasurements);

  for (unsigned i = 0; i < numberOfMeasurements; ++i) {
    posesA.col(i) = trajectoryA.poses().at(i).position();
    posesB.col(i) = trajectoryB.poses().at(i).position();
  }

  // Umeyama Alignment
  Eigen::Matrix4d transform = umeyama(posesB, posesA, false);
  returnTransform = Eigen::Isometry3d(transform);
  std::cout << YELLOW_START << "Trajectory Alignment" << GREEN_START << " Umeyama transform: " << std::endl
            << COLOR_END << returnTransform.matrix() << std::endl;

  return true;
}

bool TrajectoryAlignment::alignTrajectories(double& yaw, Eigen::Isometry3d& returnTransform) {
  // Containers for running data processing
  Trajectory copySe3Trajectory;
  Trajectory copyR3Trajectory;
  {
    // Mutex for alignment Flag
    const std::lock_guard<std::mutex> alignmentLock(alignmentMutex);
    // Copy Trajectories
    copySe3Trajectory = se3Trajectory_;
    copyR3Trajectory = r3Trajectory_;
  }

  // Check whether we are standing still
  if (copySe3Trajectory.isStanding(se3Rate_, noMovementTime_, noMovementDistance_) ||
      copyR3Trajectory.isStanding(r3Rate_, noMovementTime_, noMovementDistance_) || firstAlignmentTryFlag_) {
    // Mutex for alignment Flag
    const std::lock_guard<std::mutex> alignmentLock(alignmentMutex);
    se3Trajectory_.poses().clear();
    r3Trajectory_.poses().clear();
    if (r3Trajectory_.poses().size() % 10 == 0) {
      std::cout << YELLOW_START << "Trajectory Alignment" << GREEN_START << " No movement detected. Clearing trajectories." << COLOR_END
                << std::endl;
    }
    if (firstAlignmentTryFlag_) {
      std::cout << YELLOW_START << "Trajectory Alignment" << GREEN_START << " First Alignment Try." << COLOR_END << std::endl;
      firstAlignmentTryFlag_ = false;
    }
    return false;
  }

  // Status
  if (copyR3Trajectory.poses().size() % 10 == 0) {
    std::cout << YELLOW_START << "Trajectory Alignment" << GREEN_START << " Current Distance of Se3 / R3 [m]: " << COLOR_END
              << copySe3Trajectory.distance() << " / " << copyR3Trajectory.distance() << " of required [m] " << minDistanceHeadingInit_
              << std::endl;
  }

  // Perform Checks
  if (copySe3Trajectory.distance() < minDistanceHeadingInit_) {
    return false;
  }
  if (copyR3Trajectory.distance() < minDistanceHeadingInit_) {
    return false;
  }

  // Align Trajectories
  Trajectory newSe3Trajectory;
  Trajectory newR3Trajectory;
  // Associate Trajectories
  associateTrajectories(copySe3Trajectory, copyR3Trajectory, newSe3Trajectory, newR3Trajectory);
  // Update Trajectories
  copySe3Trajectory = newSe3Trajectory;
  copyR3Trajectory = newR3Trajectory;
  Eigen::Isometry3d alignmentTransform;
  // Trajectory Alignment
  if (!trajectoryAlignment(newR3Trajectory, newSe3Trajectory, alignmentTransform)) {
    std::cout << "TrajectoryAlignment::initializeYaw trajectoryAlignment failed." << std::endl;
    return false;
  } else {
    std::cout << YELLOW_START << "Trajectory Alignment" << GREEN_START << " Trajectories Aligned." << COLOR_END << std::endl;
  }
  // Math --> Rotation to Yaw
  Eigen::Matrix3d rotation = alignmentTransform.rotation();
  double pitch = -asin(rotation(2, 0));
  double roll = atan2(rotation(2, 1), rotation(2, 2));
  yaw = atan2(rotation(1, 0) / cos(pitch), rotation(0, 0) / cos(pitch));
  std::cout << YELLOW_START << "Trajectory Alignment" << GREEN_START << " Initial Roll/Pitch/Yaw(deg):" << COLOR_END << roll * 180 / M_PI
            << "," << pitch * 180 / M_PI << "," << yaw * 180 / M_PI << std::endl;
  // Return
  returnTransform = alignmentTransform;
  return true;
}

}  // namespace graph_msf
