/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Eigen
#include <Eigen/Eigen>

#ifndef GMSF_EIGEN_WRAPPED_GTSAM_UTILS_H
#define GMSF_EIGEN_WRAPPED_GTSAM_UTILS_H

namespace graph_msf {

// Transformation Functions
void inPlaceRemoveRollPitch(Eigen::Isometry3d& T);

// Creation of Files
// Pose 3
void createPose3CsvFileStream(std::map<std::string, std::ofstream>& fileStreams, const std::string& savePath,
                              const std::string& transformIdentifier, const std::string& timeString, const bool saveCovarianceFlag);
// Latitude, Longitude, Altitude
void createLatLonAltCsvFileStream(std::map<std::string, std::ofstream>& fileStreams, const std::string& savePath,
                                  const std::string& transformIdentifier, const std::string& timeString);

// Addition to Files
// Pose 3
void writePose3ToCsvFile(std::map<std::string, std::ofstream>& fileStreams, const Eigen::Isometry3d pose,
                         const std::string& transformIdentifier, const double timeStamp, const bool saveCovarianceFlag,
                         boost::optional<const Eigen::Matrix<double, 6, 6>&> optionalPoseCovarianceInWorldRos = boost::none);
// Latitude, Longitude, Altitude
void writeLatLonAltToCsvFile(std::map<std::string, std::ofstream>& fileStreams, const Eigen::Vector3d& latLonAlt,
                             const std::string& transformIdentifier, const double timeStamp);

}  // namespace graph_msf
#endif  // GMSF_EIGEN_WRAPPED_GTSAM_UTILS_H
