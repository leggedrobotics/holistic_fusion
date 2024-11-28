/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Eigen
#include <Eigen/Eigen>

#ifndef GMSF_EIGEN_WRAPPED_GTSAM_UTILS_H
#define GMSF_EIGEN_WRAPPED_GTSAM_UTILS_H

namespace graph_msf {

// Transformation Functions
void inPlaceRemoveRollPitch(Eigen::Isometry3d& T);

}  // namespace graph_msf
#endif  // GMSF_EIGEN_WRAPPED_GTSAM_UTILS_H
