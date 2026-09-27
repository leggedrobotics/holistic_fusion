#include <cmath>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>

#include "graph_msf/factors/non_expression/unaryYawFactor.h"

namespace {

void require(bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

const gtsam::SharedNoiseModel kNoise = gtsam::noiseModel::Isotropic::Sigma(1, 0.1);

void constrainsYawOfUpsideDownImu() {
  const gtsam::Rot3 R_I_S = gtsam::Rot3::Rx(M_PI);
  const gtsam::Pose3 T_W_I(gtsam::Rot3::Rz(0.7) * R_I_S.inverse(), gtsam::Point3(1.0, 2.0, 3.0));
  const graph_msf::YawFactor factor(0, 0.5, kNoise, R_I_S);

  gtsam::Matrix H;
  const gtsam::Vector error = factor.evaluateError(T_W_I, H);

  require(std::abs(error(0) - 0.2) < 1e-9, "Yaw error of the measurement frame is wrong: " + std::to_string(error(0)));
  require(H.norm() > 0.5, "Upside-down IMU has no yaw Jacobian");
}

void jacobianMatchesNumericalDerivative() {
  const gtsam::Rot3 R_I_S = gtsam::Rot3::RzRyRx(2.9, -0.3, 0.4);
  const gtsam::Pose3 T_W_I(gtsam::Rot3::RzRyRx(0.2, 0.1, -2.5), gtsam::Point3(0.5, -1.0, 0.2));
  const graph_msf::YawFactor factor(0, -1.2, kNoise, R_I_S);

  gtsam::Matrix H;
  factor.evaluateError(T_W_I, H);
  const std::function<gtsam::Vector1(const gtsam::Pose3&)> yawError = [&factor](const gtsam::Pose3& pose) {
    return gtsam::Vector1(factor.evaluateError(pose, nullptr));
  };
  const gtsam::Matrix H_numerical = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(yawError, T_W_I);

  require(H.isApprox(H_numerical, 1e-6), "Analytic yaw Jacobian differs from the numerical one");
}

void skipsGimbalLockOfMeasurementFrame() {
  const gtsam::Rot3 R_I_S = gtsam::Rot3::Rx(M_PI);
  const gtsam::Pose3 T_W_I(gtsam::Rot3::Ry(M_PI / 2.0) * R_I_S.inverse(), gtsam::Point3());
  const graph_msf::YawFactor factor(0, 0.3, kNoise, R_I_S);

  gtsam::Matrix H;
  const gtsam::Vector error = factor.evaluateError(T_W_I, H);

  require(error.isZero() && H.isZero(), "Yaw factor acts at gimbal lock of the measurement frame");
}

}  // namespace

int main() {
  try {
    constrainsYawOfUpsideDownImu();
    jacobianMatchesNumericalDerivative();
    skipsGimbalLockOfMeasurementFrame();
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
