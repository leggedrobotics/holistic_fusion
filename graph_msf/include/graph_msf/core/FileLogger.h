/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef FILE_LOGGER_H
#define FILE_LOGGER_H

// GTSAM Values
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>

// Workspace
#include "graph_msf/core/optimizer/OptimizerBase.h"

namespace graph_msf {

struct FileFormats {
  static constexpr const char* kTum = "tum";
  static constexpr const char* kG2o = "g2o";
  static constexpr const char* kCsv = "csv";
};

enum class FileFormatType { kCsv, kTum, kG2o };

class FileLogger {
 public:
  FileLogger() = default;
  virtual ~FileLogger() = default;

  // Create File Streams
  // 1. CSV
  // Pose3
  static void createPose3CsvFileStream(std::map<std::string, std::ofstream>& fileStreams, const std::string& savePath,
                                       const std::string& transformIdentifier, const std::string& timeString,
                                       const bool saveCovarianceFlag);
  // Latitude, Longitude, Altitude
  static void createLatLonAltCsvFileStream(std::map<std::string, std::ofstream>& fileStreams, const std::string& savePath,
                                           const std::string& transformIdentifier, const std::string& timeString);
  // Point3
  static void createPoint3CsvFileStream(std::map<std::string, std::ofstream>& fileStreams, const std::string& savePath,
                                        const std::string& transformIdentifier, const std::string& timeString);
  // Imu Bias
  static void createImuBiasCsvFileStream(std::map<std::string, std::ofstream>& fileStreams, const std::string& savePath,
                                         const std::string& stateCategoryIdentifier, const std::string& timeString);
  // 2. TUM
  // Pose3
  static void createPose3TumFileStream(std::map<std::string, std::ofstream>& fileStreams, const std::string& savePath,
                                       const std::string& transformIdentifier, const std::string& timeString);

  // Write to File
  // 1. CSV
  // Pose3
  static void writePose3ToCsvFile(std::map<std::string, std::ofstream>& fileStreams, const gtsam::Pose3& pose,
                                  const std::string& transformIdentifier, const double timeStamp, const bool saveCovarianceFlag,
                                  boost::optional<const Eigen::Matrix<double, 6, 6>&> optionalPoseCovarianceInWorldRos = boost::none);
  // Latitude, Longitude, Altitude
  static void writeLatLonAltToCsvFile(std::map<std::string, std::ofstream>& fileStreams, const gtsam::Point3& point,
                                      const std::string& transformIdentifier, const double timeStamp);
  // Point3
  static void writePoint3ToCsvFile(std::map<std::string, std::ofstream>& fileStreams, const gtsam::Point3& point,
                                   const std::string& transformIdentifier, const double timeStamp);
  // Imu Bias
  static void writeImuBiasToCsvFile(std::map<std::string, std::ofstream>& fileStreams, const gtsam::imuBias::ConstantBias& imuBias,
                                    const std::string& stateCategoryIdentifier, const double timeStamp);
  // 2. TUM
  // Pose3
  static void writePose3ToTumFile(std::map<std::string, std::ofstream>& fileStreams, const gtsam::Pose3& pose,
                                  const std::string& transformIdentifier, const double timeStamp);
};

}  // namespace graph_msf

#endif  // FILE_LOGGER_H
