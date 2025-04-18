/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_DATATYPES_HPP
#define GRAPH_MSF_DATATYPES_HPP

#include <Eigen/Dense>

namespace graph_msf {

struct ImuMeasurement {
  double timestamp;
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_DATATYPES_HPP
