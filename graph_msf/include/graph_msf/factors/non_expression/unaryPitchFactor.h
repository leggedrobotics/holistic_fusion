/*
Copyright 2022 by Julian Nubert, Timo Schoenegg, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_PITCH_FACTOR_H
#define GRAPH_MSF_PITCH_FACTOR_H

// CPP
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
  /**
   * Constructor of factor that estimates nav to body rotation bRn
   * @param key of the unknown rotation bRn in the factor graph
   * @param measured pitch reading
   * @param model of the additive Gaussian noise that is assumed
   */
  PitchFactor(gtsam::Key j, double pitch, const gtsam::SharedNoiseModel& model)
      : gtsam::NoiseModelFactor1<gtsam::Pose3>(model, j), pitch_(pitch) {}

  // Destructor
  virtual ~PitchFactor() {}

  /**
   * Evaluate error function
   * @brief vector of errors
   */
  gtsam::Vector evaluateError(const gtsam::Pose3& robotPose, gtsam::OptionalMatrixType H = OptionalNone) const override {
    // If close to singularity, do not add measurement
    if (std::abs(robotPose.rotation().pitch()) >= M_PI / 2.0 - 0.1) {
      if (H) {
        (*H) = gtsam::Matrix::Zero(1, 6);
      }
      return gtsam::Vector1::Zero();
    }

    gtsam::Matrix13 H_rot;
    const double pitch_meas = H ? robotPose.rotation().pitch(gtsam::OptionalJacobian<1, 3>(H_rot)) : robotPose.rotation().pitch();

    // calculate error
    double pitchError = pitch_meas - pitch_;

    // Smaller half circle
    while (pitchError < -M_PI) pitchError += 2 * M_PI;
    while (pitchError > M_PI) pitchError -= 2 * M_PI;

    // Jacobian
    if (H) {
      (*H) = gtsam::Matrix::Zero(1, 6);
      (*H).block<1, 3>(0, 0) = H_rot;
    }

    return gtsam::Vector1(pitchError);
  }

 private:
  double pitch_;  // pitch measurement
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_PITCH_FACTOR_H
