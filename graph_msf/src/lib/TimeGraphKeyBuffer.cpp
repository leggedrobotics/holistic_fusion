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

void TimeGraphKeyBuffer::addToBuffer(const double ts, const gtsam::Key& key) {
  if (verboseLevel_ >= 5) {
    std::cout << YELLOW_START << "GMsf-TimeKeyBuffer" << COLOR_END << " Adding key " << key << " to timeToKeyBuffer for time " << ts
              << std::endl;
  }

  // Mutex block
  {
    // Writing to IMU buffer --> acquire mutex
    const std::lock_guard<std::mutex> writeInBufferLock(writeInBufferMutex_);
    timeToKeyBuffer_[ts] = key;
    keyToTimeBuffer_[key] = ts;
    if (ts > tLatestInBuffer_) {
      tLatestInBuffer_ = ts;
    }
  }

  // If Key buffer is too large, remove first element
  if (timeToKeyBuffer_.size() > bufferLength_) {
    timeToKeyBuffer_.erase(timeToKeyBuffer_.begin());
    keyToTimeBuffer_.erase(keyToTimeBuffer_.begin());
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

  // Empty buffer
  if (timeToKeyBuffer_.empty()) {
    std::cerr << YELLOW_START << "GMsf-TimeKeyBuffer " << RED_START << "called from " << callingName << ": Buffer is empty!" << COLOR_END
              << std::endl;
    return false;
  }

  auto lowerIterator = upperIterator;
  --lowerIterator;

  // Keep key which is closer to tLidar
  tInGraph = std::abs(tK - lowerIterator->first) < std::abs(upperIterator->first - tK) ? lowerIterator->first : upperIterator->first;
  key = std::abs(tK - lowerIterator->first) < std::abs(upperIterator->first - tK) ? lowerIterator->second : upperIterator->second;
  double timeDeviation = tInGraph - tK;

  if (verboseLevel_ >= 2) {
    std::cout << YELLOW_START << "GMsf-TimeKeyBuffer" << COLOR_END << " " << callingName << std::setprecision(14)
              << " searched time step: " << tK << std::endl;
    std::cout << YELLOW_START << "GMsf-TimeKeyBuffer" << COLOR_END << " " << callingName << std::setprecision(14)
              << " found time step: " << tInGraph << " at key " << key << std::endl;
    std::cout << YELLOW_START << "GMsf-TimeKeyBuffer" << COLOR_END << " Time Deviation (t_graph-t_request): " << 1000 * timeDeviation
              << " ms" << std::endl;
    std::cout << YELLOW_START << "GMsf-TimeKeyBuffer" << COLOR_END << " Latest IMU timestamp: " << tLatestInBuffer_
              << ", hence absolut delay of measurement is " << 1000 * (tLatestInBuffer_ - tK) << "ms." << std::endl;
  }

  // Check for error and warn user
  if (std::abs(timeDeviation) > maxSearchDeviation) {
    return false;
  }

  return true;
}

}  // namespace graph_msf
