#include <algorithm>
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

// Heading of S, with R_W_S = R_W_I * R_I_S, relative to the measured R_M_Smeas brought to the world through R_W_M
gtsam::Expression<gtsam::Rot2> headingOfSensor(const gtsam::Rot3& R_I_S, const gtsam::Rot3& R_M_Smeas) {
  const gtsam::Rot3_ R_W_I = gtsam::rotation(gtsam::Pose3_(X(0)));
  const gtsam::Rot3_ R_W_M = gtsam::rotation(gtsam::Pose3_(R(0)));
  return holistic_fusion::headingInWorld(R_W_I * gtsam::Rot3_(R_I_S), R_W_M, R_M_Smeas);
}

double heading(const gtsam::Rot3& R_W_S, const gtsam::Rot3& R_W_Smeas) {
  return holistic_fusion::headingAsRot2(R_W_S * R_W_Smeas.inverse(), {}).theta();
}

void headingIsYawDifferenceForEqualTilt() {
  const std::vector<gtsam::Rot3> tilts{gtsam::Rot3(), gtsam::Rot3::Ry(0.3) * gtsam::Rot3::Rx(-0.2), kXAxisUp, gtsam::Rot3::Rx(M_PI_2)};
  for (const gtsam::Rot3& tilt : tilts) {
    const double headingValue = heading(gtsam::Rot3::Rz(0.9) * tilt, gtsam::Rot3::Rz(0.2) * tilt);
    require(std::abs(headingValue - 0.7) < 1e-9, "Heading is not the yaw difference: " + std::to_string(headingValue));
  }
}

void headingOfMeasurementIsTakenInTheWorld() {
  const gtsam::Rot3 R_I_S = gtsam::Rot3::Rx(M_PI);
  gtsam::Values values;
  values.insert(X(0), gtsam::Pose3(gtsam::Rot3::Rz(0.9) * kXAxisUp * R_I_S.inverse(), gtsam::Point3(1.0, 2.0, 3.0)));
  values.insert(R(0), gtsam::Pose3(gtsam::Rot3::Rz(0.3), gtsam::Point3(-1.0, 0.5, 0.0)));

  const double headingValue = headingOfSensor(R_I_S, gtsam::Rot3::Rz(0.1) * kXAxisUp).value(values).theta();

  require(std::abs(headingValue - 0.5) < 1e-9, "Heading of S in W is wrong: " + std::to_string(headingValue));
}

// With a tilted fixed frame M, the factor must pull only about the z-axis of the world, for the state and for the alignment R_W_M
void tiltedFixedFrameOnlyConstrainsRotationAboutWorldZ() {
  const gtsam::Rot3 R_W_M = gtsam::Rot3::RzRyRx(0.3, -0.25, 0.35);
  for (const gtsam::Rot3& R_W_S : {gtsam::Rot3::RzRyRx(0.4, -0.3, 0.2), gtsam::Rot3::Rz(1.1) * kXAxisUp}) {
    gtsam::Values values;
    values.insert(X(0), gtsam::Pose3(R_W_S, gtsam::Point3(0.5, -1.0, 0.2)));
    values.insert(R(0), gtsam::Pose3(R_W_M, gtsam::Point3(0.1, 0.2, 0.3)));
    const gtsam::ExpressionFactor<gtsam::Rot2> factor(gtsam::noiseModel::Isotropic::Sigma(1, 1.0), gtsam::Rot2(),
                                                      headingOfSensor(gtsam::Rot3(), R_W_M.inverse() * R_W_S));
    std::vector<gtsam::Matrix> H(2);
    factor.unwhitenedError(values, H);
    const gtsam::KeyVector& keys = factor.keys();
    const gtsam::Matrix& H_state = H[std::find(keys.begin(), keys.end(), X(0)) - keys.begin()];
    const gtsam::Matrix& H_alignment = H[std::find(keys.begin(), keys.end(), R(0)) - keys.begin()];

    // A rotation about axis a of W is the right perturbation R_W_X^T * a of R_W_X
    const gtsam::Matrix13 H_state_aboutAxesOfW = H_state.leftCols<3>() * R_W_S.matrix().transpose();
    const gtsam::Matrix13 H_alignment_aboutAxesOfW = H_alignment.leftCols<3>() * R_W_M.matrix().transpose();

    require(H_state_aboutAxesOfW.isApprox(gtsam::Matrix13(0.0, 0.0, 1.0), 1e-9), "State is not pulled only about z of the world");
    require(H_alignment_aboutAxesOfW.isApprox(gtsam::Matrix13(0.0, 0.0, -1.0), 1e-9), "Alignment is not pulled only about z of the world");
    require(H_state.rightCols<3>().isZero(1e-12) && H_alignment.rightCols<3>().isZero(1e-12), "Heading depends on a position");
  }
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
                                                      headingOfSensor(R_I_S, R_M_Smeas));

    require(gtsam::internal::testFactorJacobians("heading", factor, values, 1e-7, 1e-5), "Heading expression Jacobians are wrong");
  }
}

void errorWrapsAroundPi() {
  gtsam::Values values;
  values.insert(X(0), gtsam::Pose3(gtsam::Rot3::Rz(-M_PI + 0.01), gtsam::Point3()));
  values.insert(R(0), gtsam::Pose3());
  const gtsam::ExpressionFactor<gtsam::Rot2> factor(gtsam::noiseModel::Isotropic::Sigma(1, 1.0), gtsam::Rot2(),
                                                    headingOfSensor(gtsam::Rot3(), gtsam::Rot3::Rz(M_PI - 0.01)));

  const double error = factor.unwhitenedError(values)(0);

  require(std::abs(std::abs(error) - 0.02) < 1e-9, "Heading error does not wrap around pi: " + std::to_string(error));
}

}  // namespace

int main() {
  try {
    headingIsYawDifferenceForEqualTilt();
    headingOfMeasurementIsTakenInTheWorld();
    tiltedFixedFrameOnlyConstrainsRotationAboutWorldZ();
    jacobiansMatchNumericalDerivatives();
    errorWrapsAroundPi();
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
