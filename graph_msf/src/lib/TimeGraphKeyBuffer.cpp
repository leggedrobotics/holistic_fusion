/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Workspace
#include "graph_msf/core/TimeGraphKeyBuffer.h"
#include "graph_msf/interface/Terminal.h"

// C++
#include <iterator>

namespace graph_msf {

void TimeGraphKeyBuffer::addToBuffer(const double ts, const gtsam::Key& key) {
  if (verboseLevel_ >= 5) {
    std::cout << YELLOW_START << "GMsf-TimeKeyBuffer" << COLOR_END << " Adding key " << key << " to timeToKeyBuffer for time "
              << std::setprecision(14) << ts << std::endl;
  }

  // Keep insertion and pruning in one critical section so lookup iterators
  // cannot race an erase.
  const std::lock_guard<std::mutex> writeInBufferLock(writeInBufferMutex_);

  // Keep the forward and reverse maps one-to-one when a timestamp or key is replaced.
  if (const auto existingKey = timeToKeyBuffer_.find(ts); existingKey != timeToKeyBuffer_.end()) {
    keyToTimeBuffer_.erase(existingKey->second);
  }
  if (const auto existingTime = keyToTimeBuffer_.find(key); existingTime != keyToTimeBuffer_.end()) {
    timeToKeyBuffer_.erase(existingTime->second);
  }

  timeToKeyBuffer_[ts] = key;
  keyToTimeBuffer_[key] = ts;

  while (timeToKeyBuffer_.size() > static_cast<std::size_t>(bufferLength_)) {
    const auto oldest = timeToKeyBuffer_.begin();
    keyToTimeBuffer_.erase(oldest->second);
    timeToKeyBuffer_.erase(oldest);
  }

  tLatestInBuffer_ = timeToKeyBuffer_.rbegin()->first;
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
    TimeToKeyMap::const_iterator closestIterator;
    if (upperIterator == timeToKeyBuffer_.begin()) {
      // Query is before the oldest key.
      closestIterator = upperIterator;
    } else if (upperIterator == timeToKeyBuffer_.end()) {
      // Query is at or after the newest key.
      closestIterator = std::prev(timeToKeyBuffer_.end());
    } else {
      const auto lowerIterator = std::prev(upperIterator);
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

double TimeGraphKeyBuffer::getLatestTimestampInBuffer() const {
  const std::lock_guard<std::mutex> writeInBufferLock(writeInBufferMutex_);
  return tLatestInBuffer_;
}

TimeToKeyMap TimeGraphKeyBuffer::getTimeToKeyBuffer() const {
  const std::lock_guard<std::mutex> writeInBufferLock(writeInBufferMutex_);
  return timeToKeyBuffer_;
}

KeyToTimeMap TimeGraphKeyBuffer::getKeyToTimeBuffer() const {
  const std::lock_guard<std::mutex> writeInBufferLock(writeInBufferMutex_);
  return keyToTimeBuffer_;
}

}  // namespace graph_msf
