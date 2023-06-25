/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */
#define GREEN_START "\033[92m"
#define YELLOW_START "\033[33m"
#define RED_START "\033[31m"
#define COLOR_END "\033[0m"

#ifndef IMU_MANAGER_HPP_
#define IMU_MANAGER_HPP_

// C++
#include <map>
#include <mutex>

// Eigen
#include <Eigen/Dense>

// GTSAM
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>

// Workspace
#include "graph_msf/core/Datatypes.hpp"
#include "graph_msf/imu/ImuSignalLowPassFilter.hpp"

namespace graph_msf {

// Datatypes
// Map from time to 6D IMU measurements
typedef std::map<double, graph_msf::ImuMeasurement, std::less<double>,
                 Eigen::aligned_allocator<std::pair<const double, graph_msf::ImuMeasurement>>>
    TimeToImuMap;
// Map from time to gtsam key
typedef std::map<double, gtsam::Key, std::less<double>, Eigen::aligned_allocator<std::pair<const double, gtsam::Vector6>>> TimeToKeyMap;

// Actual Class
class ImuBuffer {
 public:
  // Constructor
  ImuBuffer(const bool useImuSignalLowPassFilter, const double cutoffFrequencyHz = 60.0, const double samplingTime = 0.01)
      : useImuSignalLowPassFilter_(useImuSignalLowPassFilter) {
    // Reset IMU Buffer
    timeToImuBuffer_.clear();
    // If low pass filter is used, initialize it
    if (useImuSignalLowPassFilter_) {
      imuSignalLowPassFilterPtr_ = std::make_unique<ImuSignalLowPassFilter>(cutoffFrequencyHz, samplingTime);
    }
  }

  // Destructor
  ~ImuBuffer() = default;

  // Setters
  inline void setImuRate(double d) { imuRate_ = d; }
  inline void setImuBufferLength(int i) { imuBufferLength_ = i; }
  inline void setVerboseLevel(int i) { verboseLevel_ = i; }

  // Add to buffers
  Eigen::Matrix<double, 6, 1> addToImuBuffer(double ts, const Eigen::Vector3d& linearAcc, const Eigen::Vector3d& angularVel);
  void addToKeyBuffer(double ts, gtsam::Key key);

  // Getters
  inline double getImuRate() const { return imuRate_; }
  inline double getLatestTimestampInBuffer() const { return tLatestInBuffer_; }
  void getLastTwoMeasurements(TimeToImuMap& imuMap);
  bool getClosestKeyAndTimestamp(double& tInGraph, gtsam::Key& key, const std::string& callingName, const double maxSearchDeviation,
                                 const double tLidar);

  // Public member functions
  /// Determine initial IMU pose w.r.t to gravity vector pointing up
  bool estimateAttitudeFromImu(gtsam::Rot3& initAttitude, double& gravityMagnitude, Eigen::Vector3d& gyrBias);

  // Integrate NavState from Timestamp
  bool getIMUBufferIteratorsInInterval(const double& tsStart, const double& tsEnd, TimeToImuMap::iterator& startIterator,
                                       TimeToImuMap::iterator& endIterator);
  gtsam::NavState integrateNavStateFromTimestamp(const double& tsStart, const double& tsEnd, const gtsam::NavState& stateStart,
                                                 const gtsam::imuBias::ConstantBias& imuBias, const Eigen::Vector3d& W_gravityVector);

 private:
  // Member variables
  TimeToImuMap timeToImuBuffer_;  // IMU buffer
  TimeToKeyMap timeToKeyBuffer_;
  double imuRate_ = 100;  // Rate of IMU input (Hz) - Used to calculate minimum measurements needed to calculate gravity and init attitude
  int imuBufferLength_ = -1;
  double imuPoseInitWaitSecs_ = 1.0;  // Multiplied with _imuRate
  int verboseLevel_ = 0;
  double tLatestInBuffer_ = 0.0;
  std::mutex writeInBufferMutex_;
  bool useImuSignalLowPassFilter_;

  // Low pass filter
  std::unique_ptr<ImuSignalLowPassFilter> imuSignalLowPassFilterPtr_;
};

}  // namespace graph_msf

#endif  // IMU_MANAGER_HPP_
