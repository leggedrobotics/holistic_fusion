/*
Copyright 2022 by Julian Nubert, Timo Schoenegg, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_HEADING_FACTOR_H
#define GRAPH_MSF_HEADING_FACTOR_H

// C++
#include <cmath>

// GTSAM
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>

namespace graph_msf {

/**
 * Factor to estimate rotation given gnss robot heading
 */
class YawFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3> {
 public:
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3>;
  // Bring the convenience overloads (without Jacobian, with Jacobian by reference) into scope
  using Base::evaluateError;

  /**
   * Constructor of factor that constrains the yaw angle of a Pose3 variable
   * @param j key of the unknown pose in the factor graph
   * @param yaw measured yaw angle [rad]
   * @param model of the additive Gaussian noise that is assumed
   */
  YawFactor(gtsam::Key j, double yaw, const gtsam::SharedNoiseModel& model) : Base(model, j), yaw_(yaw) {}

  // Destructor
  ~YawFactor() override = default;

  /**
   * Evaluate error function
   * @brief vector of errors
   */
  gtsam::Vector evaluateError(const gtsam::Pose3& robotPose, gtsam::OptionalMatrixType H_Ptr = OptionalNone) const override {
    // If close to singularity, do not add measurement
    if (std::abs(robotPose.rotation().pitch()) >= M_PI / 2.0 - 0.1 || std::abs(robotPose.rotation().roll()) >= M_PI / 2.0 - 0.1) {
      if (H_Ptr) {
        (*H_Ptr) = gtsam::Matrix::Zero(1, 6);
      }
      return gtsam::Vector1::Zero();
    }

    // Measurement function (with Jacobian w.r.t. the rotation only if requested)
    gtsam::Matrix13 H_rot;
    const double estimatedYaw = H_Ptr ? robotPose.rotation().yaw(H_rot) : robotPose.rotation().yaw();

    // Calculate error
    double yawError = estimatedYaw - yaw_;

    // Smaller half circle
    while (yawError < -M_PI) yawError += 2 * M_PI;
    while (yawError > M_PI) yawError -= 2 * M_PI;

    // Jacobian: Pose3 tangent space is [rotation (3), translation (3)] --> [rad] [m]
    if (H_Ptr) {
      (*H_Ptr) = gtsam::Matrix::Zero(1, 6);
      H_Ptr->block<1, 3>(0, 0) = H_rot;
    }

    return gtsam::Vector1(yawError);
  }

 private:
  double yaw_;  // yaw measurement
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_HEADING_FACTOR_H
