/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "graph_msf/imu/ImuBuffer.hpp"

// CPP
#include <iomanip>

// Workspace
#include "graph_msf/core/Datatypes.hpp"
#include "graph_msf/interface/Terminal.h"

namespace graph_msf {

// Constructor ---------------------------------------------------
// Constructor
ImuBuffer::ImuBuffer(const std::shared_ptr<GraphConfig> graphConfigPtr) {
  // Reset IMU Buffer
  timeToImuBuffer_.clear();
  // Parameters
  useImuSignalLowPassFilter_ = graphConfigPtr->useImuSignalLowPassFilter_;
  imuRate_ = graphConfigPtr->imuRate_;
  imuBufferLength_ = graphConfigPtr->imuBufferLength_;
  verboseLevel_ = graphConfigPtr->verboseLevel_;
  // If low pass filter is used, initialize it
  if (useImuSignalLowPassFilter_) {
    imuSignalLowPassFilterPtr_ =
        std::make_unique<ImuSignalLowPassFilter>(graphConfigPtr->imuLowPassFilterCutoffFreqHz_, 1.0 / graphConfigPtr->imuRate_);
  }
}

// Public --------------------------------------------------------
// Returns actually added IMU measurements
Eigen::Matrix<double, 6, 1> ImuBuffer::addToImuBuffer(double ts, const Eigen::Vector3d& linearAcc, const Eigen::Vector3d& angularVel) {
  // Check that imuBufferLength was set
  if (imuBufferLength_ < 0) {
    throw std::runtime_error("GMsfImuBuffer: imuBufferLength has to be set by the user.");
  }

  // Copy of IMU measurements
  Eigen::Matrix<double, 6, 1> filteredImuMeas;

  // Potentially low pass filter IMU measurements
  if (useImuSignalLowPassFilter_) {
    filteredImuMeas = imuSignalLowPassFilterPtr_->filter(linearAcc, angularVel);
  } else {
    filteredImuMeas << linearAcc, angularVel;
  }

  // Convert to gtsam type
  graph_msf::ImuMeasurement imuMeas;
  imuMeas.timestamp = ts;
  imuMeas.acceleration = filteredImuMeas.head<3>();
  imuMeas.angularVelocity = filteredImuMeas.tail<3>();

  // Add to buffer
  {
    // Writing to IMU buffer --> acquire mutex
    const std::lock_guard<std::mutex> writeInBufferLock(writeInBufferMutex_);
    timeToImuBuffer_[ts] = imuMeas;
    // Update latest timestamp in buffer
    if (ts > tLatestInBuffer_) {
      tLatestInBuffer_ = ts;
    }
  }

  // If IMU buffer is too large, remove first element
  if (timeToImuBuffer_.size() > imuBufferLength_) {
    timeToImuBuffer_.erase(timeToImuBuffer_.begin());
  }

  if (timeToImuBuffer_.size() > imuBufferLength_) {
    std::ostringstream errorStream;
    errorStream << YELLOW_START << "GMsf-ImuBuffer" << COLOR_END << " IMU Buffer has grown too large. It contains "
                << timeToImuBuffer_.size() << " measurements instead of " << imuBufferLength_ << ".";
    throw std::runtime_error(errorStream.str());
  }

  return filteredImuMeas;
}

double ImuBuffer::getLatestTimestampInBuffer() {
  // Reading from IMU buffer --> acquire mutex
  const std::lock_guard<std::mutex> writeInBufferLock(writeInBufferMutex_);
  return tLatestInBuffer_;
}

void ImuBuffer::getLastTwoMeasurements(TimeToImuMap& imuMap) {
  TimeToImuMap::iterator endItr = --(timeToImuBuffer_.end());
  TimeToImuMap::iterator previousItr = --(--(timeToImuBuffer_.end()));

  // Write into IMU Map
  imuMap[previousItr->first] = previousItr->second;
  imuMap[endItr->first] = endItr->second;
}

bool ImuBuffer::estimateAttitudeFromImu(gtsam::Rot3& initAttitude, double& gravityMagnitude, Eigen::Vector3d& gyrBias) {
  // Make sure that imuBuffer is long enough
  if (imuBufferLength_ < (imuRate_ * imuPoseInitWaitSecs_)) {
    throw std::runtime_error(
        "ImuBufferLength is not large enough for "
        "initialization. Must be at least 1 second.");
  }

  // Get timestamp of first message for lookup
  if (timeToImuBuffer_.size() < (imuRate_ * imuPoseInitWaitSecs_)) {
    return false;
  } else {
    // Accumulate Acceleration part of IMU Messages
    Eigen::Vector3d initAccMean(0.0, 0.0, 0.0), initGyrMean(0.0, 0.0, 0.0);
    for (auto& itr : timeToImuBuffer_) {
      initAccMean += itr.second.acceleration;
      initGyrMean += itr.second.angularVelocity;
    }

    // Average IMU measurements and set assumed gravity direction
    initAccMean /= timeToImuBuffer_.size();
    gravityMagnitude = initAccMean.norm();
    Eigen::Vector3d gUnitVecInWorld = Eigen::Vector3d(0.0, 0.0, 1.0);  // ROS convention

    // Normalize gravity vectors to remove the affect of gravity magnitude from
    // place-to-place
    initAccMean.normalize();
    initAttitude = gtsam::Rot3(Eigen::Quaterniond().setFromTwoVectors(initAccMean, gUnitVecInWorld));

    // Gyro
    initGyrMean /= timeToImuBuffer_.size();
    gyrBias = initGyrMean;

    // Calculate robot initial orientation using gravity vector.
    std::cout << YELLOW_START << "GMsf-ImuBuffer" << COLOR_END << " Gravity Magnitude: " << gravityMagnitude << std::endl;
    std::cout << YELLOW_START << "GMsf-ImuBuffer" << COLOR_END << " Mean IMU Acceleration Vector(x,y,z): " << initAccMean.transpose()
              << " - Gravity Unit Vector(x,y,z): " << gUnitVecInWorld.transpose() << std::endl;
    std::cout << YELLOW_START << "GMsf-ImuBuffer" << GREEN_START << " RPY (deg): " << initAttitude.rpy().transpose() * (180.0 / M_PI)
              << COLOR_END << std::endl;
    std::cout << YELLOW_START << "GMsf-ImuBuffer" << COLOR_END << "  Gyro bias(x,y,z): " << initGyrMean.transpose() << std::endl;
  }
  return true;
}

bool ImuBuffer::getIMUBufferIteratorsInInterval(const double& tsStart, const double& tsEnd, TimeToImuMap::iterator& startIterator,
                                                TimeToImuMap::iterator& endIterator) {
  // Check if timestamps are in correct order
  if (tsStart >= tsEnd) {
    std::cerr << YELLOW_START << "GMsf-ImuBuffer" << RED_START << " IMU Lookup Timestamps are not correct ts_start(" << std::fixed
              << tsStart << ") >= ts_end(" << tsEnd << ")\n";
    return false;
  }

  // Get Iterator Belonging to ts_start
  startIterator = timeToImuBuffer_.lower_bound(tsStart);
  // Get Iterator Belonging to ts_end
  endIterator = timeToImuBuffer_.lower_bound(tsEnd);

  // Check if it is first value in the buffer which means there is no value
  // before to interpolate with
  if (startIterator == timeToImuBuffer_.begin()) {
    std::cerr << YELLOW_START << "GMsf-ImuBuffer" << RED_START
              << " Lookup requires first message of IMU buffer, cannot "
                 "Interpolate back, "
                 "Lookup Start/End: "
              << std::fixed << tsStart << "/" << tsEnd << ", Buffer Start/End: " << timeToImuBuffer_.begin()->first << "/"
              << timeToImuBuffer_.rbegin()->first << std::endl;
    return false;
  }

  // Check if lookup start time is ahead of buffer start time
  if (startIterator == timeToImuBuffer_.end()) {
    std::cerr << YELLOW_START << "GMsf-ImuBuffer" << RED_START
              << " IMU Lookup start time ahead latest IMU message in the "
                 "buffer, lookup: "
              << tsStart << ", latest IMU: " << timeToImuBuffer_.rbegin()->first << std::endl;
    return false;
  }

  // Check if last value is valid
  if (endIterator == timeToImuBuffer_.end()) {
    std::cerr << YELLOW_START << "GMsf-ImuBuffer" << RED_START << " Lookup is past IMU buffer, with lookup Start/End: " << std::fixed
              << tsStart << "/" << tsEnd << " and latest IMU: " << timeToImuBuffer_.rbegin()->first << std::endl;
    --endIterator;
  }

  // Check if two IMU messages are different
  if (startIterator == endIterator) {
    std::cerr << YELLOW_START << "GMsf-ImuBuffer" << RED_START
              << " Not Enough IMU values between timestamps , with Start/End: " << std::fixed << tsStart << "/" << tsEnd
              << ", with diff: " << tsEnd - tsStart << std::endl;
    return false;
  }

  // If everything is good
  return true;
}

gtsam::NavState ImuBuffer::integrateNavStateFromTimestamp(const double& tsStart, const double& tsEnd, const gtsam::NavState& stateStart,
                                                          const gtsam::imuBias::ConstantBias& imuBias,
                                                          const Eigen::Vector3d& W_gravityVector) {
  // Get iterators
  TimeToImuMap::iterator startIterator, nextToStartIterator, endIterator;
  // Get IMU iterators in interval
  if (!getIMUBufferIteratorsInInterval(tsStart, tsEnd, startIterator, endIterator)) {
    throw std::runtime_error("Could not get IMU iterators in interval");
  }
  nextToStartIterator = startIterator;
  ++nextToStartIterator;

  // Propagated state
  gtsam::NavState propagatedState = stateStart;

  // For Loop for IMU integration
  Eigen::Vector3d i_measAcceleration, i_measAngularVelocity;
  for (; startIterator != endIterator; ++startIterator, ++nextToStartIterator) {
    // Get IMU measurement
    i_measAcceleration = imuBias.correctAccelerometer(nextToStartIterator->second.acceleration);
    i_measAngularVelocity = imuBias.correctGyroscope(nextToStartIterator->second.angularVelocity);

    // Calculate dt
    double dt = nextToStartIterator->first - startIterator->first;

    // Update propagated state
    Eigen::Vector3d i_gravityVector = propagatedState.R().transpose() * W_gravityVector;
    propagatedState =
        propagatedState.update(i_gravityVector + i_measAcceleration, i_measAngularVelocity, dt, boost::none, boost::none, boost::none);
  }
  return propagatedState;
}
}  // namespace graph_msf
