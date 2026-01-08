/*
Copyright 2022 by Julian Nubert, Timo Schoenegg, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_HEADING_FACTOR_H
#define GRAPH_MSF_HEADING_FACTOR_H

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
 * Factor to estimate rotation given GNSS robot heading.
 *
 * Unary factor on Pose3 that constrains yaw to a measured value.
 */
class YawFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  using Base = gtsam::NoiseModelFactor1<gtsam::Pose3>;

  /**
   * Constructor of factor that estimates nav-to-body rotation bRn
   * @param key     key of the unknown pose in the factor graph
   * @param yaw     measured yaw [rad] in the navigation frame
   * @param model   additive Gaussian noise model on yaw
   */
  YawFactor(gtsam::Key key, double yaw, const gtsam::SharedNoiseModel& model)
      : Base(model, key), yaw_(yaw) {}

  ~YawFactor() override = default;

  /**
   * Evaluate error function.
   *
   * @param robotPose  current pose estimate
   * @param H          optional 1x6 Jacobian wrt Pose3 (rot-then-trans order)
   * @return 1x1 vector containing yaw error
   */
  gtsam::Vector evaluateError(const gtsam::Pose3& robotPose,
                              gtsam::Matrix* H = nullptr) const override {
    const auto& R = robotPose.rotation();

    // If close to singularity, do not add measurement (zero error and Jacobian)
    if (std::abs(R.pitch()) >= M_PI / 2.0 - 0.1 ||
        std::abs(R.roll())  >= M_PI / 2.0 - 0.1) {
      if (H) {
        *H = gtsam::Matrix::Zero(1, 6);
      }
      return gtsam::Vector1::Zero();
    }

    double yaw_pred;
    Eigen::Matrix<double, 1, 3> Hyaw_rot;

    if (H) {
      // Jacobian of yaw w.r.t. rotation tangent (3-dof)
      gtsam::OptionalJacobian<1, 3> Hyaw(Hyaw_rot);
      yaw_pred = R.yaw(Hyaw);
    } else {
      yaw_pred = R.yaw();
    }

    double yawError = yaw_pred - yaw_;

    // Wrap into (-pi, pi]
    while (yawError < -M_PI) yawError += 2.0 * M_PI;
    while (yawError >  M_PI) yawError -= 2.0 * M_PI;

    // Fill full Pose3 Jacobian: [d yaw / d rot(3) , d yaw / d trans(3) = 0]
    if (H) {
      H->resize(1, 6);
      H->setZero();
      H->block<1, 3>(0, 0) = Hyaw_rot;  // rotation part
      // translation part already zero
    }

    return gtsam::Vector1(yawError);
  }

 private:
  double yaw_;  // yaw measurement [rad]
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_HEADING_FACTOR_H