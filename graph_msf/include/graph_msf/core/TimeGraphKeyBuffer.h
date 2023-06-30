/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef TIMEGRAPHKEYBUFFER_H
#define TIMEGRAPHKEYBUFFER_H

// C++
#include <map>
#include <mutex>

// GTSAM
#include <gtsam/base/Vector.h>

namespace graph_msf {

typedef std::map<double, gtsam::Key, std::less<double>, Eigen::aligned_allocator<std::pair<const double, gtsam::Vector6>>> TimeToKeyMap;
typedef std::map<gtsam::Key, double, std::less<gtsam::Key>> KeyToTimeMap;

class TimeGraphKeyBuffer {
 public:
  // Constructor
  TimeGraphKeyBuffer() = default;

  // Destructor
  ~TimeGraphKeyBuffer() = default;

  // Getters
  bool getClosestKeyAndTimestamp(double& tInGraph, gtsam::Key& key, const std::string& callingName, const double maxSearchDeviation,
                                 const double tK);
  double getLatestTimestampInBuffer() const { return tLatestInBuffer_; }
  const TimeToKeyMap& getTimeToKeyBuffer() { return timeToKeyBuffer_; }
  const KeyToTimeMap& getKeyToTimeBuffer() { return keyToTimeBuffer_; }

  // Setters
  inline void setVerboseLevel(const int verboseLevel) { verboseLevel_ = verboseLevel; }
  inline void setImuBufferLength(const int imuBufferLength) { imuBufferLength_ = imuBufferLength; }

  // Add to buffers
  void addToKeyBuffer(const double ts, const gtsam::Key& key);

 private:
  // Key buffer
  TimeToKeyMap timeToKeyBuffer_;
  KeyToTimeMap keyToTimeBuffer_;
  // Mutex
  std::mutex writeInBufferMutex_;
  // Verbose level
  int verboseLevel_ = 0;
  int imuBufferLength_ = -1;
  double tLatestInBuffer_ = 0.0;
};

}  // namespace graph_msf

#endif  // TIMEGRAPHKEYBUFFER_H
