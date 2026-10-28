/*
Copyright 2026 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef HOLISTIC_FUSION_BATCH_OPTIMIZATION_RESULT_H
#define HOLISTIC_FUSION_BATCH_OPTIMIZATION_RESULT_H

// C++
#include <optional>
#include <vector>

// Eigen
#include <Eigen/Eigen>

namespace holistic_fusion {

// Batch-optimized navigation state of the IMU frame I at one graph state.
struct BatchOptimizedNavState {
  double timeK;
  Eigen::Isometry3d T_W_Ik;
  Eigen::Vector3d W_v_W_I;
  Eigen::Vector3d accelerometerBias;
  Eigen::Vector3d gyroscopeBias;
  // Marginal covariance of T_W_Ik in the world frame, ROS ordering (x, y, z, roll, pitch, yaw).
  // Only set when the covariance was requested.
  std::optional<Eigen::Matrix<double, 6, 6>> poseCovarianceInWorld;
};

struct BatchOptimizationResult {
  // Sorted by ascending timeK.
  std::vector<BatchOptimizedNavState> navStates;
};

}  // namespace holistic_fusion

#endif  // HOLISTIC_FUSION_BATCH_OPTIMIZATION_RESULT_H
