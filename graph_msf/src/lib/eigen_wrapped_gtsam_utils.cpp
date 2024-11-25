/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// GTSAM
#include <gtsam/base/types.h>
#include <gtsam/geometry/Rot3.h>

// Implementation
#include "graph_msf/interface/eigen_wrapped_gtsam_utils.h"

namespace graph_msf {

void inPlaceRemoveRollPitch(Eigen::Isometry3d &T) {
  // Extract the rotation matrix
  gtsam::Rot3 R(T.rotation());
  // Remove the roll and pitch
  R = gtsam::Rot3::Ypr(R.yaw(), 0.0, 0.0);
  // Set the rotation matrix
  T.linear() = R.matrix();
}

} // namespace graph_msf
