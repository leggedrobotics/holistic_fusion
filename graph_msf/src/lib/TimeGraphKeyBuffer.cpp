/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Workspace
#include "graph_msf/core/TimeGraphKeyBuffer.h"
#include "graph_msf/interface/Terminal.h"

namespace graph_msf {

void TimeGraphKeyBuffer::addToKeyBuffer(const double ts, const gtsam::Key& key) {
  if (verboseLevel_ >= 5) {
    std::cout << YELLOW_START << "GMsf-Imu-Buffer" << COLOR_END << " Adding key " << key << " to timeToKeyBuffer for time " << ts
              << std::endl;
  }

  // Mutex block
  {
    // Writing to IMU buffer --> acquire mutex
    const std::lock_guard<std::mutex> writeInBufferLock(writeInBufferMutex_);
    timeToKeyBuffer_[ts] = key;
    if (ts > tLatestInBuffer_) {
      tLatestInBuffer_ = ts;
    }
  }

  // If Key buffer is too large, remove first element
  if (timeToKeyBuffer_.size() > imuBufferLength_) {
    timeToKeyBuffer_.erase(timeToKeyBuffer_.begin());
  }

  if (timeToKeyBuffer_.size() > imuBufferLength_) {
    std::ostringstream errorStream;
    errorStream << YELLOW_START << "GMsf-ImuBuffer" << COLOR_END << " Key Buffer has grown too large. It contains "
                << timeToKeyBuffer_.size() << " measurements instead of " << imuBufferLength_ << ".";
    throw std::runtime_error(errorStream.str());
  }
}

bool TimeGraphKeyBuffer::getClosestKeyAndTimestamp(double& tInGraph, gtsam::Key& key, const std::string& callingName,
                                                   const double maxSearchDeviation, const double tK) {
  std::_Rb_tree_iterator<std::pair<const double, gtsam::Key>> upperIterator;
  {
    // Read from IMU buffer --> acquire mutex
    const std::lock_guard<std::mutex> writeInBufferLock(writeInBufferMutex_);
    upperIterator = timeToKeyBuffer_.upper_bound(tK);
  }

  auto lowerIterator = upperIterator;
  --lowerIterator;

  // Keep key which is closer to tLidar
  tInGraph = std::abs(tK - lowerIterator->first) < std::abs(upperIterator->first - tK) ? lowerIterator->first : upperIterator->first;
  key = std::abs(tK - lowerIterator->first) < std::abs(upperIterator->first - tK) ? lowerIterator->second : upperIterator->second;
  double timeDeviation = tInGraph - tK;

  if (verboseLevel_ >= 2) {
    std::cout << YELLOW_START << "GMsf-ImuBuffer" << COLOR_END << " " << callingName << std::setprecision(14)
              << " searched time step: " << tK << std::endl;
    std::cout << YELLOW_START << "GMsf-ImuBuffer" << COLOR_END << " " << callingName << std::setprecision(14)
              << " found time step: " << tInGraph << " at key " << key << std::endl;
    std::cout << YELLOW_START << "GMsf-ImuBuffer" << COLOR_END << " Time Deviation (t_graph-t_request): " << 1000 * timeDeviation << " ms"
              << std::endl;
    std::cout << YELLOW_START << "GMsf-ImuBuffer" << COLOR_END << " Latest IMU timestamp: " << tLatestInBuffer_
              << ", hence absolut delay of measurement is " << 1000 * (tLatestInBuffer_ - tK) << "ms." << std::endl;
  }

  // Check for error and warn user
  if (std::abs(timeDeviation) > maxSearchDeviation) {
    std::cerr << YELLOW_START << "GMsf-ImuBuffer " << RED_START << callingName << " Time deviation at key " << key << " is "
              << 1000 * timeDeviation << " ms, being larger than admissible deviation of " << 1000 * maxSearchDeviation << " ms"
              << COLOR_END << std::endl;
    return false;
  }

  return true;
}

}  // namespace graph_msf
