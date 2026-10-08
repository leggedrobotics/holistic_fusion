#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/expressionTesting.h>
#include <gtsam/slam/expressions.h>

#include "holistic_fusion/factors/gmsf_expression/GmsfUnaryExpressionAbsoluteHeading.h"

namespace {

void require(bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

using gtsam::symbol_shorthand::R;
using gtsam::symbol_shorthand::X;

// Rotation with the x-axis along z, where the Euler yaw is undefined
const gtsam::Rot3 kXAxisUp = gtsam::Rot3::Ry(-M_PI_2);

// heading(R_M_S * R_M_Smeas^-1) with R_M_S = R_W_M^-1 * R_W_I * R_I_S, as the heading expression composes it
gtsam::Expression<gtsam::Rot2> headingOfSensorInFixedFrame(const gtsam::Rot3& R_I_S, const gtsam::Rot3& R_M_Smeas) {
  const gtsam::Rot3_ R_W_I = gtsam::rotation(gtsam::Pose3_(X(0)));
  const gtsam::Rot3_ R_W_M = gtsam::rotation(gtsam::Pose3_(R(0)));
  const gtsam::Rot3_ R_M_S = holistic_fusion::inverseRot3(R_W_M) * R_W_I * gtsam::Rot3_(R_I_S);
  return gtsam::Expression<gtsam::Rot2>(&holistic_fusion::headingAsRot2, R_M_S * gtsam::Rot3_(R_M_Smeas.inverse()));
}

double heading(const gtsam::Rot3& R_M_S, const gtsam::Rot3& R_M_Smeas) {
  return holistic_fusion::headingAsRot2(R_M_S * R_M_Smeas.inverse(), {}).theta();
}

void headingIsYawDifferenceForEqualTilt() {
  const std::vector<gtsam::Rot3> tilts{gtsam::Rot3(), gtsam::Rot3::Ry(0.3) * gtsam::Rot3::Rx(-0.2), kXAxisUp, gtsam::Rot3::Rx(M_PI_2)};
  for (const gtsam::Rot3& tilt : tilts) {
    const double headingValue = heading(gtsam::Rot3::Rz(0.9) * tilt, gtsam::Rot3::Rz(0.2) * tilt);
    require(std::abs(headingValue - 0.7) < 1e-9, "Heading is not the yaw difference: " + std::to_string(headingValue));
  }
}

// At the measured orientation, rotating S about the horizontal axes of M must not change the heading
void tiltLeavesHeadingUnchangedToFirstOrder() {
  for (const gtsam::Rot3& R_M_S : {gtsam::Rot3::RzRyRx(0.4, -0.3, 0.2), gtsam::Rot3::Rz(1.1) * kXAxisUp}) {
    const gtsam::ExpressionFactor<gtsam::Rot2> factor(
        gtsam::noiseModel::Isotropic::Sigma(1, 1.0), gtsam::Rot2(),
        gtsam::Expression<gtsam::Rot2>(&holistic_fusion::headingAsRot2, gtsam::Rot3_(X(0)) * gtsam::Rot3_(R_M_S.inverse())));
    gtsam::Values values;
    values.insert(X(0), R_M_S);
    std::vector<gtsam::Matrix> H(1);
    factor.unwhitenedError(values, H);

    // A rotation about axis a of M is the right perturbation R_M_S^T * a of R_M_S
    const gtsam::Matrix13 H_aboutAxesOfM = H.front() * R_M_S.matrix().transpose();

    require(H_aboutAxesOfM.head<2>().norm() < 1e-9, "Tilt changes the heading to first order");
    require(std::abs(H_aboutAxesOfM(2) - 1.0) < 1e-9, "Rotation about z of M does not change the heading one to one");
  }
}

void headingIsEvaluatedInTheFixedFrame() {
  const gtsam::Rot3 R_I_S = gtsam::Rot3::Rx(M_PI);
  gtsam::Values values;
  values.insert(X(0), gtsam::Pose3(gtsam::Rot3::Rz(0.9) * kXAxisUp * R_I_S.inverse(), gtsam::Point3(1.0, 2.0, 3.0)));
  values.insert(R(0), gtsam::Pose3(gtsam::Rot3::Rz(0.3), gtsam::Point3(-1.0, 0.5, 0.0)));

  const double headingValue = headingOfSensorInFixedFrame(R_I_S, gtsam::Rot3::Rz(0.1) * kXAxisUp).value(values).theta();

  require(std::abs(headingValue - 0.5) < 1e-9, "Heading of S in M is wrong: " + std::to_string(headingValue));
}

void jacobiansMatchNumericalDerivatives() {
  const gtsam::Rot3 R_I_S = gtsam::Rot3::RzRyRx(2.9, -0.3, 0.4);
  const std::vector<gtsam::Rot3> sensorAttitudes{gtsam::Rot3::RzRyRx(0.2, 0.1, -2.5), gtsam::Rot3::Rz(0.7) * kXAxisUp};
  for (const gtsam::Rot3& R_W_S : sensorAttitudes) {
    gtsam::Values values;
    values.insert(X(0), gtsam::Pose3(R_W_S * R_I_S.inverse(), gtsam::Point3(0.5, -1.0, 0.2)));
    values.insert(R(0), gtsam::Pose3(gtsam::Rot3::RzRyRx(0.05, -0.02, 0.4), gtsam::Point3(0.1, 0.2, 0.3)));
    const gtsam::Rot3 R_M_Smeas = gtsam::Rot3::Rz(-1.2) * R_W_S * gtsam::Rot3::Rx(0.1);

    const gtsam::ExpressionFactor<gtsam::Rot2> factor(gtsam::noiseModel::Isotropic::Sigma(1, 0.1), gtsam::Rot2(),
                                                      headingOfSensorInFixedFrame(R_I_S, R_M_Smeas));

    require(gtsam::internal::testFactorJacobians("heading", factor, values, 1e-7, 1e-5), "Heading expression Jacobians are wrong");
  }
}

void errorWrapsAroundPi() {
  gtsam::Values values;
  values.insert(X(0), gtsam::Pose3(gtsam::Rot3::Rz(-M_PI + 0.01), gtsam::Point3()));
  values.insert(R(0), gtsam::Pose3());
  const gtsam::ExpressionFactor<gtsam::Rot2> factor(gtsam::noiseModel::Isotropic::Sigma(1, 1.0), gtsam::Rot2(),
                                                    headingOfSensorInFixedFrame(gtsam::Rot3(), gtsam::Rot3::Rz(M_PI - 0.01)));

  const double error = factor.unwhitenedError(values)(0);

  require(std::abs(std::abs(error) - 0.02) < 1e-9, "Heading error does not wrap around pi: " + std::to_string(error));
}

}  // namespace

int main() {
  try {
    headingIsYawDifferenceForEqualTilt();
    tiltLeavesHeadingUnchangedToFirstOrder();
    headingIsEvaluatedInTheFixedFrame();
    jacobiansMatchNumericalDerivatives();
    errorWrapsAroundPi();
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
