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

std::vector<std::pair<double, Eigen::Vector3d>> TrajectoryAlignment::getLidarTrajectory() {
  std::vector<std::pair<double, Eigen::Vector3d>> trajectory;
  std::pair<double, Eigen::Vector3d> onePose;
  for (auto pose : lidarTrajectory_.poses()) {
    onePose.first = pose.time();
    onePose.second = pose.position();
    trajectory.push_back(onePose);
  }
  return trajectory;
}

std::vector<std::pair<double, Eigen::Vector3d>> TrajectoryAlignment::getGnssTrajectory() {
  std::vector<std::pair<double, Eigen::Vector3d>> trajectory;
  std::pair<double, Eigen::Vector3d> onePose;
  for (auto pose : gnssTrajectory_.poses()) {
    onePose.first = pose.time();
    onePose.second = pose.position();
    trajectory.push_back(onePose);
  }
  return trajectory;
}

void TrajectoryAlignment::addLidarPose(Eigen::Vector3d position, double time) {
  const std::lock_guard<std::mutex> alingmentLock(alignmentMutex);
  lidarTrajectory_.addPose(position, time);
}

void TrajectoryAlignment::addGnssPose(Eigen::Vector3d position, double time) {
  const std::lock_guard<std::mutex> alingmentLock(alignmentMutex);
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

bool TrajectoryAlignment::trajectoryAlignment(Trajectory& trajectoryA, Trajectory& trajectoryB, Eigen::Matrix4d& transform) {
  // fill matrices to use Eigen Umeyama function.
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

  // Umeyama Alignment.
  transform = umeyama(posesB, posesA, false);
  std::cout << YELLOW_START << "Trajectory Alignment" << GREEN_START << " Umeyama transform: " << std::endl
            << COLOR_END << transform << std::endl;

  return true;
}

bool TrajectoryAlignment::alignTrajectories(double& yaw) {
  // Mutex for alignment Flag
  const std::lock_guard<std::mutex> alignmentLock(alignmentMutex);

  // We only check LIO since GNSS measurements might be jumpy, i.e. RTK Float.
  if (lidarTrajectory_.isStanding(10, 1, 0.50)) {
    lidarTrajectory_.poses().clear();
    gnssTrajectory_.poses().clear();
    return false;
  }

  // Status
  if (gnssTrajectory_.poses().size() % 10 == 0) {
    std::cout << YELLOW_START << "Trajectory Alignment" << GREEN_START << " Current Distance of LiDAR/GNSS [m]: " << COLOR_END
              << lidarTrajectory_.distance() << "/" << gnssTrajectory_.distance() << " of required [m] " << minDistanceHeadingInit_
              << std::endl;
  }

  // Perform Checks
  if (lidarTrajectory_.distance() < minDistanceHeadingInit_) {
    return false;
  }
  if (gnssTrajectory_.distance() < minDistanceHeadingInit_) {
    return false;
  }

  // Align Trajectories
  Trajectory newLidarTrajectory;
  Trajectory newGnssTrajectory;
  // Associate Trajectories
  associateTrajectories(lidarTrajectory_, gnssTrajectory_, newLidarTrajectory, newGnssTrajectory);
  // Update Trajectories
  lidarTrajectory_ = newLidarTrajectory;
  gnssTrajectory_ = newGnssTrajectory;
  Eigen::Matrix4d transform;
  // Trajectory Alignment
  if (!trajectoryAlignment(newGnssTrajectory, newLidarTrajectory, transform)) {
    std::cout << "TrajectoryAlignment::initializeYaw trajectoryAlignment failed." << std::endl;
    return false;
  } else {
    std::cout << YELLOW_START << "Trajectory Alignment" << GREEN_START << " Trajectories Aligned." << COLOR_END << std::endl;
  }
  // Math
  Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);
  double pitch = -asin(transform(2, 0));
  double roll = atan2(transform(2, 1), transform(2, 2));
  yaw = atan2(transform(1, 0) / cos(pitch), transform(0, 0) / cos(pitch));
  std::cout << YELLOW_START << "Trajectory Alignment" << GREEN_START << " Initial Roll/Pitch/Yaw(deg):" << COLOR_END << roll * 180 / M_PI
            << "," << pitch * 180 / M_PI << "," << yaw * 180 / M_PI << std::endl;
  // Return
  return true;
}

}  // namespace graph_msf
