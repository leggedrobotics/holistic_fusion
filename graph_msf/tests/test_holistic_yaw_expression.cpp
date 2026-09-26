#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/expressionTesting.h>
#include <gtsam/slam/expressions.h>

#include "graph_msf/factors/gmsf_expression/GmsfUnaryExpressionAbsoluteYaw.h"

namespace {

void require(bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

using gtsam::symbol_shorthand::R;
using gtsam::symbol_shorthand::X;

// yaw(R_M_S) with R_M_S = R_W_M^-1 * R_W_I * R_I_S, as the yaw expression composes it
gtsam::Expression<gtsam::Rot2> yawOfSensorInFixedFrame(const gtsam::Rot3& R_I_S) {
  const gtsam::Rot3_ R_W_I = gtsam::rotation(gtsam::Pose3_(X(0)));
  const gtsam::Rot3_ R_W_M = gtsam::rotation(gtsam::Pose3_(R(0)));
  const gtsam::Rot3_ R_M_S = graph_msf::inverseRot3(R_W_M) * R_W_I * gtsam::Rot3_(R_I_S);
  return gtsam::Expression<gtsam::Rot2>(&graph_msf::yawAsRot2, R_M_S);
}

void yawIsEvaluatedInTheFixedFrame() {
  const gtsam::Rot3 R_I_S = gtsam::Rot3::Rx(M_PI);
  gtsam::Values values;
  values.insert(X(0), gtsam::Pose3(gtsam::Rot3::Rz(0.9) * R_I_S.inverse(), gtsam::Point3(1.0, 2.0, 3.0)));
  values.insert(R(0), gtsam::Pose3(gtsam::Rot3::Rz(0.3), gtsam::Point3(-1.0, 0.5, 0.0)));

  const double yaw_M_S = yawOfSensorInFixedFrame(R_I_S).value(values).theta();

  require(std::abs(yaw_M_S - 0.6) < 1e-9, "Yaw of S in M is wrong: " + std::to_string(yaw_M_S));
}

void jacobiansMatchNumericalDerivatives() {
  const gtsam::Rot3 R_I_S = gtsam::Rot3::RzRyRx(2.9, -0.3, 0.4);
  gtsam::Values values;
  values.insert(X(0), gtsam::Pose3(gtsam::Rot3::RzRyRx(0.2, 0.1, -2.5), gtsam::Point3(0.5, -1.0, 0.2)));
  values.insert(R(0), gtsam::Pose3(gtsam::Rot3::RzRyRx(0.05, -0.02, 0.4), gtsam::Point3(0.1, 0.2, 0.3)));

  const gtsam::ExpressionFactor<gtsam::Rot2> factor(gtsam::noiseModel::Isotropic::Sigma(1, 0.1), gtsam::Rot2::fromAngle(-1.2),
                                                    yawOfSensorInFixedFrame(R_I_S));

  require(gtsam::internal::testFactorJacobians("holistic yaw", factor, values, 1e-7, 1e-5), "Yaw expression Jacobians are wrong");
}

void errorWrapsAroundPi() {
  gtsam::Values values;
  values.insert(X(0), gtsam::Pose3(gtsam::Rot3::Rz(-M_PI + 0.01), gtsam::Point3()));
  values.insert(R(0), gtsam::Pose3());
  const gtsam::ExpressionFactor<gtsam::Rot2> factor(gtsam::noiseModel::Isotropic::Sigma(1, 1.0), gtsam::Rot2::fromAngle(M_PI - 0.01),
                                                    yawOfSensorInFixedFrame(gtsam::Rot3()));

  const double error = factor.unwhitenedError(values)(0);

  require(std::abs(std::abs(error) - 0.02) < 1e-9, "Yaw error does not wrap around pi: " + std::to_string(error));
}

}  // namespace

int main() {
  try {
    yawIsEvaluatedInTheFixedFrame();
    jacobiansMatchNumericalDerivatives();
    errorWrapsAroundPi();
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
