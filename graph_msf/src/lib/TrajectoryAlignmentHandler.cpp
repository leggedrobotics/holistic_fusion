/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// C++
#include <iostream>
// Package
#include "graph_msf/TrajectoryAlignmentHandler.h"

namespace graph_msf {

// Public -------------------------------------------------------------------
TrajectoryAlignmentHandler::TrajectoryAlignmentHandler() {
  std::cout << YELLOW_START << "TrajectoryAlignmentHandler" << GREEN_START << " Created Gnss Handler instance." << COLOR_END << std::endl;
}

void TrajectoryAlignmentHandler::initHandler() {
  std::cout << YELLOW_START << "TrajectoryAlignmentHandler" << GREEN_START << " Initializing the handler." << COLOR_END << std::endl;
}

void TrajectoryAlignmentHandler::addLidarPose(Eigen::Vector3d position, double time) {
  lidarTrajectory_.addPose(position, time);
}

void TrajectoryAlignmentHandler::addGnssPose(Eigen::Vector3d position, double time) {
  gnssTrajectory_.addPose(position, time);
}

bool TrajectoryAlignmentHandler::associateTrajectories(Trajectory& lidarTrajectory, Trajectory& gnssTrajectory,
                                                       Trajectory& newLidarTrajectory, Trajectory& newGnssTrajectory) {
  Trajectory longerTrajectory;
  Trajectory shorterTrajectory;
  if (lidarTrajectory.poses().size() > gnssTrajectory.poses().size()) {
    longerTrajectory = lidarTrajectory;
    shorterTrajectory = gnssTrajectory;
  } else {
    longerTrajectory = gnssTrajectory;
    shorterTrajectory = lidarTrajectory;
  }

  std::vector<int> matchingIndexesShort;
  std::vector<int> matchingIndexesLong;
  int indexShort = 0;
  int matchingIndexLong = -1;
  int lastMatchingIndex = -1;
  for (auto poseShort : shorterTrajectory.poses()) {
    int indexLong = 0;
    double diff = std::numeric_limits<double>::max();
    for (auto poseLong : longerTrajectory.poses()) {
      if ((abs(poseShort.time() - poseLong.time())) < diff && abs((poseShort.time() - poseLong.time())) < 0.1 &&
          indexLong > lastMatchingIndex) {
        diff = poseShort.time() - poseLong.time();
        matchingIndexLong = indexLong;
        lastMatchingIndex = matchingIndexLong;
      }
      ++indexLong;
    }
    if (matchingIndexLong > -1) {
      matchingIndexesShort.push_back(indexShort);
      matchingIndexesLong.push_back(matchingIndexLong);
    }
    ++indexShort;
  }

  if (lidarTrajectory.poses().size() > gnssTrajectory.poses().size()) {
    for (auto index : matchingIndexesLong) {
      newLidarTrajectory.addPose(lidarTrajectory.poses().at(index));
    }
    for (auto index : matchingIndexesShort) {
      newGnssTrajectory.addPose(gnssTrajectory.poses().at(index));
    }
  } else {
    for (auto index : matchingIndexesShort) {
      newLidarTrajectory.addPose(lidarTrajectory.poses().at(index));
    }
    for (auto index : matchingIndexesLong) {
      newGnssTrajectory.addPose(gnssTrajectory.poses().at(index));
    }
  }

  return true;
}

bool TrajectoryAlignmentHandler::trajectoryAlignment(Trajectory& newLidarTrajectory, Trajectory& newGnssTrajectory,
                                                     Eigen::Matrix4d& transform) {
  // fill matrices.
  const int numberOfMeasurements = newLidarTrajectory.poses().size();
  std::cout << " number of Measurements " << numberOfMeasurements << " " << newGnssTrajectory.poses().size() << std::endl;
  if (numberOfMeasurements < 2) return false;

  Eigen::MatrixXd lidarPoses;
  Eigen::MatrixXd gnssPoses;
  lidarPoses.resize(3, numberOfMeasurements);
  gnssPoses.resize(3, numberOfMeasurements);

  for (unsigned i = 0; i < numberOfMeasurements; ++i) {
    lidarPoses.col(i) = newLidarTrajectory.poses().at(i).position();
    gnssPoses.col(i) = newGnssTrajectory.poses().at(i).position() - newGnssTrajectory.poses().at(0).position();
  }

  // Umeyama Alignment.
  std::cout << newGnssTrajectory.poses().at(0).position() << std::endl;
  std::cout << "umeyama " << std::endl;
  std::cout << lidarPoses << std::endl;
  std::cout << gnssPoses << std::endl;
  transform = umeyama(gnssPoses, lidarPoses, false);

  std::cout << transform << std::endl;

  return true;
}

bool TrajectoryAlignmentHandler::initializeYaw(double& yaw) {
  if (lidarTrajectory_.distance() < 0.2) return false;
  if (gnssTrajectory_.distance() < 0.2) return false;

  // aligin trajectories
  Trajectory newLidarTrajectory;
  Trajectory newGnssTrajectory;
  if (!associateTrajectories(lidarTrajectory_, gnssTrajectory_, newLidarTrajectory, newGnssTrajectory)) {
    std::cout << "TrajectoryAlignmentHandler::initializeYaw associateTrajectories failed." << std::endl;
    return false;
  }

  Eigen::Matrix4d transform;
  if (!trajectoryAlignment(newLidarTrajectory, newGnssTrajectory, transform)) {
    std::cout << "TrajectoryAlignmentHandler::initializeYaw trajectoryAlignment failed." << std::endl;
    return false;
  }

  Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);
  std::cout << rotation << std::endl;
  Eigen::Vector3d eulerAngles;
  rotationMatrixToEulerAngles(rotation, eulerAngles);
  std::cout << eulerAngles << std::endl;
  std::cout << " yaw " << eulerAngles(2) * 180 / M_PI << std::endl;
  yaw = eulerAngles(2);
  return true;
}

bool TrajectoryAlignmentHandler::rotationMatrixToEulerAngles(Eigen::Matrix3d& rotation, Eigen::Vector3d& eulerAngles) {
  double sy = sqrt(rotation.coeff(0, 0) * rotation.coeff(0, 0) + rotation.coeff(1, 0) * rotation.coeff(1, 0));
  bool singular = sy < 1e-6;
  double x, y, z;

  if (!singular) {
    x = atan2(rotation.coeff(2, 1), rotation.coeff(2, 2));
    y = atan2(-rotation.coeff(2, 0), sy);
    z = atan2(rotation.coeff(1, 0), rotation.coeff(0, 0));
  } else {
    x = atan2(-rotation.coeff(1, 2), rotation.coeff(1, 1));
    y = atan2(-rotation.coeff(2, 0), sy);
    z = 0;
  }
  eulerAngles = Eigen::Vector3d(x, y, z);
  std::cout << "roll pitch yaw " << rotation << std::endl;

  return true;
}

}  // namespace graph_msf
