/*
Copyright 2022 by Julian Nubert, Timo Schoenegg, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_ROLL_FACTOR_H
#define GRAPH_MSF_ROLL_FACTOR_H

// STD
#include <cmath>

// GTSAM
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace graph_msf {

/**
 * Factor to estimate rotation given robot roll
 */
class RollFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  using Base = gtsam::NoiseModelFactor1<gtsam::Pose3>;

  /**
   * Constructor of factor that estimates nav to body rotation bRn
   * @param j      key of the unknown rotation bRn in the factor graph
   * @param roll   measured roll reading [rad]
   * @param model  additive Gaussian noise model
   */
  RollFactor(gtsam::Key j, double roll, const gtsam::SharedNoiseModel& model)
      : Base(model, j), roll_(roll) {}

  ~RollFactor() override = default;

  /**
   * Evaluate error function
   * @brief vector of errors
   */
  gtsam::Vector evaluateError(const gtsam::Pose3& robotPose,
                              gtsam::Matrix* H = nullptr) const override {
    const auto& R = robotPose.rotation();

    // If close to singularity, do not add measurement
    if (std::abs(R.roll()) >= M_PI / 2.0 - 0.1) {
      if (H) {
        *H = gtsam::Matrix::Zero(1, 6);
      }
      return gtsam::Vector1::Zero();
    }

    double roll_pred;
    Eigen::Matrix<double, 1, 3> Hroll_rot;

    if (H) {
      gtsam::OptionalJacobian<1, 3> Hroll(Hroll_rot);
      roll_pred = R.roll(Hroll);
    } else {
      roll_pred = R.roll();
    }

    double rollError = roll_pred - roll_;

    // Wrap into (-pi, pi]
    while (rollError < -M_PI) rollError += 2.0 * M_PI;
    while (rollError >  M_PI) rollError -= 2.0 * M_PI;

    // Jacobian: [d roll / d rot(3), 0 0 0]
    if (H) {
      H->resize(1, 6);
      H->setZero();
      H->block<1, 3>(0, 0) = Hroll_rot;  // rotation part
      // translation part remains zero
    }

    return gtsam::Vector1(rollError);
  }

 private:
  double roll_;  // roll measurement [rad]
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_ROLL_FACTOR_H
