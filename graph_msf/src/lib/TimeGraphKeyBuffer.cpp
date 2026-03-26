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

  // Writing to IMU buffer --> acquire mutex
  const std::lock_guard<std::mutex> writeInBufferLock(writeInBufferMutex_);
  timeToKeyBuffer_[ts] = key;
  keyToTimeBuffer_[key] = ts;
  if (ts > tLatestInBuffer_) {
    tLatestInBuffer_ = ts;
  }

  // If Key buffer is too large, remove first element and the matching reverse entry.
  while (timeToKeyBuffer_.size() > bufferLength_) {
    const auto oldestIt = timeToKeyBuffer_.begin();
    const gtsam::Key oldestKey = oldestIt->second;
    timeToKeyBuffer_.erase(oldestIt);
    keyToTimeBuffer_.erase(oldestKey);
  }
}

bool TimeGraphKeyBuffer::getClosestKeyAndTimestamp(double& tInGraph, gtsam::Key& key, const std::string& callingName,
                                                   const double maxSearchDeviation, const double tK) {
  // Read from IMU buffer --> acquire mutex
  const std::lock_guard<std::mutex> writeInBufferLock(writeInBufferMutex_);

  // Empty buffer
  if (timeToKeyBuffer_.empty()) {
    std::cerr << YELLOW_START << "GMsf-TimeKeyBuffer " << RED_START << "called from " << callingName << ": Buffer is empty!" << COLOR_END
              << std::endl;
    return false;
  }

  const auto firstIt = timeToKeyBuffer_.begin();
  const auto lastIt = std::prev(timeToKeyBuffer_.end());
  const auto upperIterator = timeToKeyBuffer_.lower_bound(tK);
  TimeToKeyMap::const_iterator chosenIterator;

  // lower_bound keeps boundary cases valid without dereferencing end().
  if (upperIterator == timeToKeyBuffer_.begin()) {
    chosenIterator = upperIterator;
    if (verboseLevel_ >= 3 && tK < chosenIterator->first) {
      std::cout << YELLOW_START << "GMsf-TimeKeyBuffer" << COLOR_END << " " << callingName << " requested timestamp "
                << std::setprecision(14) << tK << " before oldest buffered timestamp " << chosenIterator->first << "." << std::endl;
    }
  } else if (upperIterator == timeToKeyBuffer_.end()) {
    chosenIterator = lastIt;
    if (verboseLevel_ >= 3 && tK > chosenIterator->first) {
      std::cout << YELLOW_START << "GMsf-TimeKeyBuffer" << COLOR_END << " " << callingName << " requested timestamp "
                << std::setprecision(14) << tK << " after newest buffered timestamp " << chosenIterator->first << "." << std::endl;
    }
  } else {
    const auto lowerIterator = std::prev(upperIterator);
    chosenIterator =
        (std::abs(tK - lowerIterator->first) <= std::abs(upperIterator->first - tK)) ? lowerIterator : upperIterator;
  }

  // Keep key which is closer to tK.
  tInGraph = chosenIterator->first;
  key = chosenIterator->second;
  double timeDeviation = tInGraph - tK;

  if (verboseLevel_ >= 2) {
    std::cout << YELLOW_START << "GMsf-TimeKeyBuffer" << COLOR_END << " " << callingName << std::setprecision(14)
              << " searched time step: " << tK << std::endl;
    std::cout << YELLOW_START << "GMsf-TimeKeyBuffer" << COLOR_END << " " << callingName << std::setprecision(14)
              << " found time step: " << tInGraph << " at key " << key << std::endl;
    std::cout << YELLOW_START << "GMsf-TimeKeyBuffer" << COLOR_END << " Time Deviation (t_graph-t_request): " << 1000 * timeDeviation
              << " ms" << std::endl;
    std::cout << YELLOW_START << "GMsf-TimeKeyBuffer" << COLOR_END << " Oldest/Newest buffered timestamps: " << firstIt->first << " / "
              << lastIt->first << std::endl;
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
