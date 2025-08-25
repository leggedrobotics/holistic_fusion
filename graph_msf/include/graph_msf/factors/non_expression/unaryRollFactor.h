/*
Copyright 2022 by Julian Nubert, Timo Schoenegg, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_ROLL_FACTOR_H
#define GRAPH_MSF_ROLL_FACTOR_H

// CPP
#include <boost/none.hpp>
#include <boost/shared_ptr.hpp>

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
  /**
   * Constructor of factor that estimates nav to body rotation bRn
   * @param key of the unknown rotation bRn in the factor graph
   * @param measured roll reading
   * @param model of the additive Gaussian noise that is assumed
   */
  RollFactor(gtsam::Key j, double roll, const gtsam::SharedNoiseModel& model) : gtsam::NoiseModelFactor1<gtsam::Pose3>(model, j), roll_(roll) {}

  // Destructor
  virtual ~RollFactor() {}

  /**
   * Evaluate error function
   * @brief vector of errors
   */
  gtsam::Vector evaluateError(const gtsam::Pose3& robotPose, boost::optional<gtsam::Matrix&> H_Ptr = boost::none) const {
    // calculate error
    double rollError = robotPose.rotation().roll(H_Ptr) - roll_;

    // Smaller half circle
    while (rollError < -M_PI) rollError += 2 * M_PI;
    while (rollError > M_PI) rollError -= 2 * M_PI;

    // Jacobian
    if (H_Ptr) {
      (*H_Ptr) = (gtsam::Matrix(1, 6) << *H_Ptr, 0.0, 0.0, 0.0).finished();  // [rad] [m]
    }

    return gtsam::Vector1(rollError);
  }

 private:
  double roll_;  // roll measurement
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_ROLL_FACTOR_H
