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
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>

namespace graph_msf {

/**
 * Factor constraining the world yaw of a measurement frame S that is rigidly attached to the IMU frame I.
 * The yaw is evaluated on S, so the constraint holds for any IMU mounting orientation.
 */
class YawFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3> {
 public:
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3>;
  // Bring the convenience overloads (without Jacobian, with Jacobian by reference) into scope
  using Base::evaluateError;

  /**
   * Constructor of factor that constrains the yaw angle of frame S, given the pose T_W_I of the IMU frame
   * @param j key of the unknown IMU pose T_W_I in the factor graph
   * @param yaw_W_S measured yaw angle of frame S in the world frame [rad]
   * @param model of the additive Gaussian noise that is assumed
   * @param R_I_S rotation of frame S expressed in the IMU frame, identity if S is the IMU frame
   */
  YawFactor(gtsam::Key j, double yaw_W_S, const gtsam::SharedNoiseModel& model, const gtsam::Rot3& R_I_S = gtsam::Rot3())
      : Base(model, j), yaw_W_S_(yaw_W_S), R_I_S_(R_I_S) {}

  // Destructor
  ~YawFactor() override = default;

  /**
   * Evaluate error function
   * @brief vector of errors
   */
  gtsam::Vector evaluateError(const gtsam::Pose3& T_W_I, gtsam::OptionalMatrixType H_Ptr = OptionalNone) const override {
    gtsam::Matrix33 H_compose;
    const gtsam::Rot3 R_W_S = H_Ptr ? T_W_I.rotation().compose(R_I_S_, H_compose) : T_W_I.rotation().compose(R_I_S_);

    // Yaw is singular only at gimbal lock of frame S, so do not add the measurement there
    if (std::abs(R_W_S.pitch()) >= M_PI / 2.0 - 0.1) {
      if (H_Ptr) {
        (*H_Ptr) = gtsam::Matrix::Zero(1, 6);
      }
      return gtsam::Vector1::Zero();
    }

    // Measurement function (with Jacobian w.r.t. the rotation of S only if requested)
    gtsam::Matrix13 H_yaw;
    const double estimatedYaw = H_Ptr ? R_W_S.yaw(H_yaw) : R_W_S.yaw();

    // Calculate error
    double yawError = estimatedYaw - yaw_W_S_;

    // Smaller half circle
    while (yawError < -M_PI) yawError += 2 * M_PI;
    while (yawError > M_PI) yawError -= 2 * M_PI;

    // Jacobian: Pose3 tangent space is [rotation (3), translation (3)] --> [rad] [m]
    if (H_Ptr) {
      (*H_Ptr) = gtsam::Matrix::Zero(1, 6);
      H_Ptr->block<1, 3>(0, 0) = H_yaw * H_compose;
    }

    return gtsam::Vector1(yawError);
  }

 private:
  double yaw_W_S_;     // yaw measurement of frame S
  gtsam::Rot3 R_I_S_;  // rotation of frame S in the IMU frame
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_HEADING_FACTOR_H
