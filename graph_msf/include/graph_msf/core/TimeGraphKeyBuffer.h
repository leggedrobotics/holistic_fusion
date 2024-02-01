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

typedef std::map<double, gtsam::Key> TimeToKeyMap;
typedef std::map<gtsam::Key, double> KeyToTimeMap;

class TimeGraphKeyBuffer {
 public:
  // Constructor
  TimeGraphKeyBuffer(const int bufferLength, const int verboseLevel) : bufferLength_(bufferLength), verboseLevel_(verboseLevel){};

  // Destructor
  ~TimeGraphKeyBuffer() = default;

  // Getters
  bool getClosestKeyAndTimestamp(double& tInGraph, gtsam::Key& key, const std::string& callingName, const double maxSearchDeviation,
                                 const double tK);
  double getLatestTimestampInBuffer() const { return tLatestInBuffer_; }
  const TimeToKeyMap& getTimeToKeyBuffer() { return timeToKeyBuffer_; }
  const KeyToTimeMap& getKeyToTimeBuffer() { return keyToTimeBuffer_; }

  // Add to buffers
  void addToBuffer(const double ts, const gtsam::Key& key);

 private:
  // Key buffer
  TimeToKeyMap timeToKeyBuffer_;
  KeyToTimeMap keyToTimeBuffer_;
  // Mutex
  std::mutex writeInBufferMutex_;
  // Verbose level
  int verboseLevel_ = 0;
  int bufferLength_ = -1;
  double tLatestInBuffer_ = 0.0;
};

}  // namespace graph_msf

#endif  // TIMEGRAPHKEYBUFFER_H
