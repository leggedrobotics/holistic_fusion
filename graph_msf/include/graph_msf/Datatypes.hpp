/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_DATATYPES_HPP
#define GRAPH_MSF_DATATYPES_HPP

#include <gtsam/navigation/NavState.h>

namespace graph_msf {

// Map from time to 6D IMU measurements
typedef std::map<double, gtsam::Vector6, std::less<double>, Eigen::aligned_allocator<std::pair<const double, gtsam::Vector6>>> TimeToImuMap;

// Map from time to gtsam key
typedef std::map<double, gtsam::Key, std::less<double>, Eigen::aligned_allocator<std::pair<const double, gtsam::Vector6>>> TimeToKeyMap;

struct NavStateWithCovariance {
  // Initialization
  NavStateWithCovariance(const gtsam::NavState& navState, const gtsam::Matrix66& poseCovariance, const gtsam::Matrix33& velocityCovariance)
      : navState(navState), poseCovariance(poseCovariance), velocityCovariance(velocityCovariance) {}

  // Public Members
  gtsam::NavState navState;
  gtsam::Matrix66 poseCovariance;
  gtsam::Matrix33 velocityCovariance;
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_DATATYPES_HPP
