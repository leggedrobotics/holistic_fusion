/*
Copyright 2022 by Julian Nubert, Timo Schoenegg, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_PITCH_FACTOR_H
#define GRAPH_MSF_PITCH_FACTOR_H

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
 * Factor to estimate rotation given robot pitch
 */
class PitchFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  using Base = gtsam::NoiseModelFactor1<gtsam::Pose3>;

  /**
   * Constructor of factor that estimates nav to body rotation bRn
   * @param j      key of the unknown rotation bRn in the factor graph
   * @param pitch  measured pitch reading [rad]
   * @param model  additive Gaussian noise model
   */
  PitchFactor(gtsam::Key j, double pitch, const gtsam::SharedNoiseModel& model)
      : Base(model, j), pitch_(pitch) {}

  ~PitchFactor() override = default;

  /**
   * Evaluate error function
   * @brief vector of errors
   */
  gtsam::Vector evaluateError(const gtsam::Pose3& robotPose,
                              gtsam::Matrix* H = nullptr) const override {
    const auto& R = robotPose.rotation();

    // If close to singularity, do not add measurement
    if (std::abs(R.pitch()) >= M_PI / 2.0 - 0.1) {
      if (H) {
        *H = gtsam::Matrix::Zero(1, 6);
      }
      return gtsam::Vector1::Zero();
    }

    double pitch_pred;
    Eigen::Matrix<double, 1, 3> Hpitch_rot;

    if (H) {
      gtsam::OptionalJacobian<1, 3> Hpitch(Hpitch_rot);
      pitch_pred = R.pitch(Hpitch);
    } else {
      pitch_pred = R.pitch();
    }

    double pitchError = pitch_pred - pitch_;

    // Wrap into (-pi, pi]
    while (pitchError < -M_PI) pitchError += 2.0 * M_PI;
    while (pitchError >  M_PI) pitchError -= 2.0 * M_PI;

    // Jacobian: [d pitch / d rot(3), 0 0 0]
    if (H) {
      H->resize(1, 6);
      H->setZero();
      H->block<1, 3>(0, 0) = Hpitch_rot;  // rotation part
      // translation part remains zero
    }

    return gtsam::Vector1(pitchError);
  }

 private:
  double pitch_;  // pitch measurement [rad]
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_PITCH_FACTOR_H
