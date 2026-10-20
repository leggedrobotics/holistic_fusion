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
    std::cout << YELLOW_START << "GMsf-TimeKeyBuffer" << COLOR_END << " Adding key " << key << " to timeToKeyBuffer for time "
              << std::setprecision(14) << ts << std::endl;
  }

  // Keep insertion and pruning in one critical section so lookup iterators
  // cannot race an erase.
  const std::lock_guard<std::mutex> writeInBufferLock(writeInBufferMutex_);
  timeToKeyBuffer_[ts] = key;
  keyToTimeBuffer_[key] = ts;
  if (ts > tLatestInBuffer_) {
    tLatestInBuffer_ = ts;
  }

  if (timeToKeyBuffer_.size() > static_cast<size_t>(bufferLength_)) {
    const auto oldestIterator = timeToKeyBuffer_.begin();
    keyToTimeBuffer_.erase(oldestIterator->second);
    timeToKeyBuffer_.erase(oldestIterator);
  }
}

bool TimeGraphKeyBuffer::getClosestKeyAndTimestamp(double& tInGraph, gtsam::Key& key, const std::string& callingName,
                                                   const double maxSearchDeviation, const double tK) {
  double latestTimestampInBuffer = 0.0;
  {
    // Hold the mutex until the selected timestamp and key have been copied.
    const std::lock_guard<std::mutex> writeInBufferLock(writeInBufferMutex_);
    if (timeToKeyBuffer_.empty()) {
      std::cerr << YELLOW_START << "GMsf-TimeKeyBuffer " << RED_START << "called from " << callingName << ": Buffer is empty!"
                << COLOR_END << std::endl;
      return false;
    }

    const auto upperIterator = timeToKeyBuffer_.upper_bound(tK);
    auto closestIterator = timeToKeyBuffer_.begin();
    if (upperIterator == timeToKeyBuffer_.begin()) {
      // Query is before the oldest key.
      closestIterator = upperIterator;
    } else if (upperIterator == timeToKeyBuffer_.end()) {
      // Query is at or after the newest key.
      closestIterator = timeToKeyBuffer_.end();
      --closestIterator;
    } else {
      auto lowerIterator = upperIterator;
      --lowerIterator;
      // Preserve the previous tie-break: choose the newer key.
      closestIterator = std::abs(tK - lowerIterator->first) < std::abs(upperIterator->first - tK) ? lowerIterator : upperIterator;
    }

    tInGraph = closestIterator->first;
    key = closestIterator->second;
    latestTimestampInBuffer = tLatestInBuffer_;
  }

  const double timeDeviation = tInGraph - tK;

  if (verboseLevel_ >= 2) {
    std::cout << YELLOW_START << "GMsf-TimeKeyBuffer" << COLOR_END << " " << callingName << std::setprecision(14)
              << " searched time step: " << tK << std::endl;
    std::cout << YELLOW_START << "GMsf-TimeKeyBuffer" << COLOR_END << " " << callingName << std::setprecision(14)
              << " found time step: " << tInGraph << " at key " << key << std::endl;
    std::cout << YELLOW_START << "GMsf-TimeKeyBuffer" << COLOR_END << " Time Deviation (t_graph-t_request): " << 1000 * timeDeviation
              << " ms" << std::endl;
    std::cout << YELLOW_START << "GMsf-TimeKeyBuffer" << COLOR_END << " Latest IMU timestamp: " << latestTimestampInBuffer
              << ", hence absolut delay of measurement is " << 1000 * (latestTimestampInBuffer - tK) << "ms." << std::endl;
  }

  // Check for error and warn user
  if (std::abs(timeDeviation) > maxSearchDeviation) {
    return false;
  }

  return true;
}

}  // namespace graph_msf
