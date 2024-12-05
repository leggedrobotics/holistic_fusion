/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// GTSAM
#include <gtsam/base/types.h>
#include <gtsam/geometry/Rot3.h>

// Workspace
#include "graph_msf/core/FileLogger.h"

// Implementation
#include "graph_msf/interface/eigen_wrapped_gtsam_utils.h"

namespace graph_msf {

// Transformation Functions ---------------------------------------------------
void inPlaceRemoveRollPitch(Eigen::Isometry3d& T) {
  // Extract the rotation matrix
  gtsam::Rot3 R(T.rotation());
  // Remove the roll and pitch
  R = gtsam::Rot3::Ypr(R.yaw(), 0.0, 0.0);
  // Set the rotation matrix
  T.linear() = R.matrix();
}

// Creation of Files -----------------------------------------------------------
void createPose3CsvFileStream(std::map<std::string, std::ofstream>& fileStreams, const std::string& savePath,
                              const std::string& transformIdentifier, const std::string& timeString, const bool saveCovarianceFlag) {
  // Create the file stream
  FileLogger::createPose3CsvFileStream(fileStreams, savePath, transformIdentifier, timeString, saveCovarianceFlag);
}

// Addition to Files -----------------------------------------------------------
void writePose3ToCsvFile(std::map<std::string, std::ofstream>& fileStreams, const Eigen::Isometry3d pose,
                         const std::string& transformIdentifier, const double timeStamp, const bool saveCovarianceFlag,
                         boost::optional<const Eigen::Matrix<double, 6, 6>&> optionalPoseCovarianceInWorldRos) {
  // Convert
  gtsam::Pose3 poseGtsam = gtsam::Pose3(pose.matrix());
  // Write to the file
  FileLogger::writePose3ToCsvFile(fileStreams, poseGtsam, transformIdentifier, timeStamp, saveCovarianceFlag, optionalPoseCovarianceInWorldRos);
}

}  // namespace graph_msf
