/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "graph_msf/core/FileLogger.h"

// C++
#include <gtsam/inference/Symbol.h>

// Output
#include "graph_msf/interface/constants.h"
#define REGULAR_COUT std::cout << YELLOW_START << "GMSF-TransformExpressionKeys" << COLOR_END

namespace graph_msf {

// Create File Streams
// 1. CSV
// Pose3
void FileLogger::createPose3CsvFileStream(std::map<std::string, std::ofstream>& fileStreams, const std::string& savePath,
                                          const std::string& transformIdentifier, const std::string& timeString,
                                          const bool saveCovarianceFlag) {
  if (fileStreams.find(transformIdentifier) == fileStreams.end()) {
    // If not, create a new file stream for this category
    const std::string stateFileName = savePath + timeString + "/" + transformIdentifier + ".csv";
    REGULAR_COUT << GREEN_START << " Saving 6D pose states to CSV-file: " << COLOR_END << stateFileName << std::endl;

    // Open for writing and appending
    fileStreams[transformIdentifier].open(stateFileName, std::ofstream::out | std::ofstream::app);
    // Write header
    fileStreams[transformIdentifier] << "time, x, y, z, quat_x, quat_y, quat_z, quat_w, roll, pitch, yaw\n";

    if (saveCovarianceFlag) {
      const std::string covarianceFileName = savePath + timeString + "/" + transformIdentifier + "_covariance.csv";
      REGULAR_COUT << GREEN_START << " Saving covariances to CSV-file: " << COLOR_END << covarianceFileName << std::endl;
      // Open for writing and appending
      fileStreams[transformIdentifier + "_covariance"].open(covarianceFileName, std::ofstream::out | std::ofstream::app);
      // Write header
      fileStreams[transformIdentifier + "_covariance"] << "time, cov_t1_t1, cov_t1_t2, cov_t1_t3, cov_t1_r1, cov_t1_r2, cov_t1_r3, "
                                                          "cov_t2_t1, cov_t2_t2, cov_t2_t3, cov_t2_r1, cov_t2_r2, cov_t2_r3, "
                                                          "cov_t3_t1, cov_t3_t2, cov_t3_t3, cov_t3_r1, cov_t3_r2, cov_t3_r3, "
                                                          "cov_r1_t1, cov_r1_t2, cov_r1_t3, cov_r1_r1, cov_r1_r2, cov_r1_r3, "
                                                          "cov_r2_t1, cov_r2_t2, cov_r2_t3, cov_r2_r1, cov_r2_r2, cov_r2_r3, "
                                                          "cov_r3_t1, cov_r3_t2, cov_r3_t3, cov_r3_r1, cov_r3_r2, cov_r3_r3\n";
    }
  }
}

// Latitude, Longitude, Altitude
void FileLogger::createLatLonAltCsvFileStream(std::map<std::string, std::ofstream>& fileStreams, const std::string& savePath,
                                              const std::string& transformIdentifier, const std::string& timeString) {
  if (fileStreams.find(transformIdentifier) == fileStreams.end()) {
    // If not, create a new file stream for this category
    std::string fileName = savePath + timeString + "/" + transformIdentifier + ".csv";
    REGULAR_COUT << GREEN_START << " Saving lat-lon-alt states to CSV-file: " << COLOR_END << fileName << std::endl;
    // Open for writing and appending
    fileStreams[transformIdentifier].open(fileName, std::ofstream::out | std::ofstream::app);
    // Write header
    fileStreams[transformIdentifier] << "time, latitude, longitude, altitude\n";
  }
}

// Point3
void FileLogger::createPoint3CsvFileStream(std::map<std::string, std::ofstream>& fileStreams, const std::string& savePath,
                                           const std::string& transformIdentifier, const std::string& timeString) {
  if (fileStreams.find(transformIdentifier) == fileStreams.end()) {
    // If not, create a new file stream for this category
    std::string fileName = savePath + timeString + "/" + transformIdentifier + ".csv";
    REGULAR_COUT << GREEN_START << " Saving 3D states to CSV-file: " << COLOR_END << fileName << std::endl;
    // Open for writing and appending
    fileStreams[transformIdentifier].open(fileName, std::ofstream::out | std::ofstream::app);
    // Write header
    fileStreams[transformIdentifier] << "time, x, y, z\n";
  }
}

// Imu Bias
void FileLogger::createImuBiasCsvFileStream(std::map<std::string, std::ofstream>& fileStreams, const std::string& savePath,
                                            const std::string& stateCategoryIdentifier, const std::string& timeString) {
  if (fileStreams.find(stateCategoryIdentifier) == fileStreams.end()) {
    // If not, create a new file stream for this category
    std::string fileName = savePath + timeString + "/" + stateCategoryIdentifier + ".csv";
    REGULAR_COUT << GREEN_START << " Saving IMU bias states to CSV-file: " << COLOR_END << fileName << std::endl;
    // Open for writing and appending
    fileStreams[stateCategoryIdentifier].open(fileName, std::ofstream::out | std::ofstream::app);
    // Write header
    fileStreams[stateCategoryIdentifier] << "time, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z\n";
  }
}

// Double
void FileLogger::createDoubleCsvFileStream(std::map<std::string, std::ofstream>& fileStreams, const std::string& savePath,
                                           const std::string& transformIdentifier, const std::string& timeString) {
  if (fileStreams.find(transformIdentifier) == fileStreams.end()) {
    // If not, create a new file stream for this category
    std::string fileName = savePath + timeString + "/" + transformIdentifier + ".csv";
    REGULAR_COUT << GREEN_START << " Saving double values to CSV-file: " << COLOR_END << fileName << std::endl;
    // Open for writing and appending
    fileStreams[transformIdentifier].open(fileName, std::ofstream::out | std::ofstream::app);
    // Write header
    fileStreams[transformIdentifier] << "time, value\n";
  }
}

// 2. TUM
// Pose3
void FileLogger::createPose3TumFileStream(std::map<std::string, std::ofstream>& fileStreams, const std::string& savePath,
                                          const std::string& transformIdentifier, const std::string& timeString) {
  if (fileStreams.find(transformIdentifier) == fileStreams.end()) {
    // If not, create a new file stream for this category
    std::string fileName = savePath + timeString + "/" + transformIdentifier + ".tum";
    REGULAR_COUT << GREEN_START << " Saving 6D pose states to TUM-file: " << COLOR_END << fileName << std::endl;
    // Open for writing and appending
    fileStreams[transformIdentifier].open(fileName, std::ofstream::out | std::ofstream::app);
    // Precision
    fileStreams[transformIdentifier].precision(std::numeric_limits<double>::max_digits10);
    // Write header
    fileStreams[transformIdentifier] << "# timestamp tx ty tz qx qy qz qw\n";
  }
}

// Write to File
// 1. CSV
// Pose3
void FileLogger::writePose3ToCsvFile(std::map<std::string, std::ofstream>& fileStreams, const gtsam::Pose3& pose,
                                     const std::string& transformIdentifier, const double timeStamp, const bool saveCovarianceFlag,
                                     boost::optional<const Eigen::Matrix<double, 6, 6>&> optionalPoseCovarianceInWorldRos) {
  fileStreams[transformIdentifier] << std::setprecision(14) << timeStamp << ", " << pose.x() << ", " << pose.y() << ", " << pose.z() << ", "
                                   << pose.rotation().toQuaternion().x() << ", " << pose.rotation().toQuaternion().y() << ", "
                                   << pose.rotation().toQuaternion().z() << ", " << pose.rotation().toQuaternion().w() << ", "
                                   << pose.rotation().roll() << ", " << pose.rotation().pitch() << ", " << pose.rotation().yaw() << "\n";
  // Write the covariance to the appropriate file
  if (saveCovarianceFlag) {
    if (!optionalPoseCovarianceInWorldRos) {
      throw std::runtime_error("FileLogger: Pose covariance not provided for writing to file.");
    } else {
      const auto& poseCovarianceInWorldRos = optionalPoseCovarianceInWorldRos.value();
      fileStreams[transformIdentifier + "_covariance"]
          << std::setprecision(14) << timeStamp << ", " << poseCovarianceInWorldRos(0, 0) << ", " << poseCovarianceInWorldRos(0, 1) << ", "
          << poseCovarianceInWorldRos(0, 2) << ", " << poseCovarianceInWorldRos(0, 3) << ", " << poseCovarianceInWorldRos(0, 4) << ", "
          << poseCovarianceInWorldRos(0, 5) << ", " << poseCovarianceInWorldRos(1, 0) << ", " << poseCovarianceInWorldRos(1, 1) << ", "
          << poseCovarianceInWorldRos(1, 2) << ", " << poseCovarianceInWorldRos(1, 3) << ", " << poseCovarianceInWorldRos(1, 4) << ", "
          << poseCovarianceInWorldRos(1, 5) << ", " << poseCovarianceInWorldRos(2, 0) << ", " << poseCovarianceInWorldRos(2, 1) << ", "
          << poseCovarianceInWorldRos(2, 2) << ", " << poseCovarianceInWorldRos(2, 3) << ", " << poseCovarianceInWorldRos(2, 4) << ", "
          << poseCovarianceInWorldRos(2, 5) << ", " << poseCovarianceInWorldRos(3, 0) << ", " << poseCovarianceInWorldRos(3, 1) << ", "
          << poseCovarianceInWorldRos(3, 2) << ", " << poseCovarianceInWorldRos(3, 3) << ", " << poseCovarianceInWorldRos(3, 4) << ", "
          << poseCovarianceInWorldRos(3, 5) << ", " << poseCovarianceInWorldRos(4, 0) << ", " << poseCovarianceInWorldRos(4, 1) << ", "
          << poseCovarianceInWorldRos(4, 2) << ", " << poseCovarianceInWorldRos(4, 3) << ", " << poseCovarianceInWorldRos(4, 4) << ", "
          << poseCovarianceInWorldRos(4, 5) << ", " << poseCovarianceInWorldRos(5, 0) << ", " << poseCovarianceInWorldRos(5, 1) << ", "
          << poseCovarianceInWorldRos(5, 2) << ", " << poseCovarianceInWorldRos(5, 3) << ", " << poseCovarianceInWorldRos(5, 4) << ", "
          << poseCovarianceInWorldRos(5, 5) << "\n";
    }
  }
}

// Latitude, Longitude, Altitude
void FileLogger::writeLatLonAltToCsvFile(std::map<std::string, std::ofstream>& fileStreams, const gtsam::Point3& point,
                                         const std::string& transformIdentifier, const double timeStamp) {
  fileStreams[transformIdentifier] << std::setprecision(14) << timeStamp << ", " << point.x() << ", " << point.y() << ", " << point.z()
                                   << "\n";
}

// Point3
void FileLogger::writePoint3ToCsvFile(std::map<std::string, std::ofstream>& fileStreams, const gtsam::Point3& point,
                                      const std::string& transformIdentifier, const double timeStamp) {
  fileStreams[transformIdentifier] << std::setprecision(14) << timeStamp << ", " << point.x() << ", " << point.y() << ", " << point.z()
                                   << "\n";
}

// Imu Bias
void FileLogger::writeImuBiasToCsvFile(std::map<std::string, std::ofstream>& fileStreams, const gtsam::imuBias::ConstantBias& imuBias,
                                       const std::string& stateCategoryIdentifier, const double timeStamp) {
  fileStreams[stateCategoryIdentifier] << std::setprecision(14) << timeStamp << ", " << imuBias.accelerometer().x() << ", "
                                       << imuBias.accelerometer().y() << ", " << imuBias.accelerometer().z() << ", "
                                       << imuBias.gyroscope().x() << ", " << imuBias.gyroscope().y() << ", " << imuBias.gyroscope().z()
                                       << "\n";
}

// Double
void FileLogger::writeDoubleToCsvFile(std::map<std::string, std::ofstream>& fileStreams, const double value,
                                      const std::string& transformIdentifier, const double timeStamp) {
  fileStreams[transformIdentifier] << std::setprecision(14) << timeStamp << ", " << value << "\n";
}

// 2. TUM
// Pose3
void FileLogger::writePose3ToTumFile(std::map<std::string, std::ofstream>& fileStreams, const gtsam::Pose3& pose,
                                     const std::string& transformIdentifier, const double timeStamp) {
  fileStreams[transformIdentifier] << std::setprecision(14) << timeStamp << " " << pose.x() << " " << pose.y() << " " << pose.z() << " "
                                   << pose.rotation().toQuaternion().x() << " " << pose.rotation().toQuaternion().y() << " "
                                   << pose.rotation().toQuaternion().z() << " " << pose.rotation().toQuaternion().w() << "\n";
}

}  // namespace graph_msf