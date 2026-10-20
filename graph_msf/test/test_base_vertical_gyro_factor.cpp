#include <graph_msf/factors/non_expression/BaseVerticalGyroFactor.h>

#include <gtest/gtest.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/PriorFactor.h>

#include <cmath>
#include <limits>

namespace {

gtsam::Key x(size_t i) { return gtsam::Symbol('x', i); }
gtsam::Key q(size_t i) { return gtsam::Symbol('q', i); }
const gtsam::Key kBias = gtsam::Symbol('l', 0);

gtsam::SharedNoiseModel scalarNoise(double sigma = 1e-3) {
  return gtsam::noiseModel::Isotropic::Sigma(1, sigma);
}

gtsam::SharedNoiseModel poseNoise(double sigma) {
  gtsam::Vector6 sigmas;
  sigmas << sigma, sigma, sigma, 0.01, 0.01, 0.01;
  return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
}

gtsam::Pose3 cabinPose(const gtsam::Rot3& base, double turn,
                       const gtsam::Rot3& R_IC = gtsam::Rot3()) {
  return gtsam::Pose3(base * gtsam::Rot3::Rz(turn) * R_IC.inverse(),
                      gtsam::Point3(1.2, -0.5, 0.3));
}

gtsam::Values solve(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& initial) {
  gtsam::LevenbergMarquardtParams params;
  params.setAbsoluteErrorTol(1e-12);
  params.setRelativeErrorTol(1e-12);
  params.setMaxIterations(100);
  return gtsam::LevenbergMarquardtOptimizer(graph, initial, params).optimize();
}

struct Integral {
  double vertical_angle = 0.0;
  gtsam::Vector3 A = gtsam::Vector3::Zero();
};

// Independent synthetic raw leg data. R_BL is a fixed mounting transform;
// angular rate is constant in BASE during this one integration interval.
// The factor receives only z and A, never the mounting or true BASE attitude.
Integral rawLegIntegral(const gtsam::Rot3& base_i, const gtsam::Vector3& omega_base,
                        const gtsam::Rot3& R_BL, const gtsam::Vector3& bias_sensor,
                        double dt) {
  Integral result;
  constexpr int substeps = 100;
  const gtsam::Vector3 measured_gyro = R_BL.unrotate(omega_base) + bias_sensor;
  for (int k = 0; k < substeps; ++k) {
    const double t = (k + 0.5) * dt / substeps;
    const auto base_t = base_i * gtsam::Rot3::Expmap(omega_base * t);
    const auto gravity_sensor = R_BL.unrotate(base_t.unrotate(gtsam::Vector3::UnitZ()));
    result.A += gravity_sensor * (dt / substeps);
    result.vertical_angle += gravity_sensor.dot(measured_gyro) * (dt / substeps);
  }
  return result;
}

TEST(BaseVerticalGyroFactor, AnalyticJacobiansRespectTiltExtrinsicAndSensorBias) {
  const auto R_IC = gtsam::Rot3::RzRyRx(0.12, -0.18, 0.4);
  const auto base_i = gtsam::Rot3::RzRyRx(0.25, -0.21, 3.139);
  const auto base_j = base_i * gtsam::Rot3::Expmap(gtsam::Vector3(0.003, -0.004, 0.009));
  const double qi = 3.13, qj = -3.12;
  const auto pi = cabinPose(base_i, qi, R_IC), pj = cabinPose(base_j, qj, R_IC);
  const gtsam::Vector3 A(0.004, -0.006, 0.018);
  const gtsam::Vector3 bias(0.01, -0.02, 0.008);
  const graph_msf::BaseVerticalGyroFactor factor(x(0), q(0), x(1), q(1), kBias, 0.007, A, R_IC, scalarNoise());
  gtsam::Matrix Hxi, Hqi, Hxj, Hqj, Hb;
  factor.evaluateError(pi, qi, pj, qj, bias, Hxi, Hqi, Hxj, Hqj, Hb);
  const auto fxi = [&](const gtsam::Pose3& p) { return factor.evaluateError(p, qi, pj, qj, bias); };
  const auto fqi = [&](double a) { return factor.evaluateError(pi, a, pj, qj, bias); };
  const auto fxj = [&](const gtsam::Pose3& p) { return factor.evaluateError(pi, qi, p, qj, bias); };
  const auto fqj = [&](double a) { return factor.evaluateError(pi, qi, pj, a, bias); };
  const auto fb = [&](const gtsam::Vector3& b) { return factor.evaluateError(pi, qi, pj, qj, b); };
  EXPECT_LT((Hxi - gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(fxi, pi)).norm(), 1e-8);
  EXPECT_LT((Hqi - gtsam::numericalDerivative11<gtsam::Vector, double>(fqi, qi)).norm(), 1e-8);
  EXPECT_LT((Hxj - gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(fxj, pj)).norm(), 1e-8);
  EXPECT_LT((Hqj - gtsam::numericalDerivative11<gtsam::Vector, double>(fqj, qj)).norm(), 1e-8);
  EXPECT_LT((Hb - gtsam::numericalDerivative11<gtsam::Vector, gtsam::Vector3>(fb, bias)).norm(), 1e-8);
  EXPECT_EQ(Hxi.rightCols(3).norm(), 0.0);
  EXPECT_EQ(Hxj.rightCols(3).norm(), 0.0);
  gtsam::Values values;
  values.insert(x(0), pi); values.insert(q(0), qi);
  values.insert(x(1), pj); values.insert(q(1), qj); values.insert(kBias, bias);
  EXPECT_NEAR(factor.clone()->error(values), factor.error(values), 1e-15);
}

TEST(BaseVerticalGyroFactor, RawSensorProjectionHandlesChangingGravityAndAllBiasAxes) {
  const auto R_IC = gtsam::Rot3::RzRyRx(-0.1, 0.2, -0.3);
  const auto R_BL = gtsam::Rot3::RzRyRx(0.5, -0.7, 1.2);
  const auto base_i = gtsam::Rot3::RzRyRx(0.22, -0.31, 1.1);
  const gtsam::Vector3 omega(0.25, -0.18, 0.4), bias(0.02, -0.015, 0.01);
  constexpr double dt = 0.02;
  const auto base_j = base_i * gtsam::Rot3::Expmap(omega * dt);
  const auto integral = rawLegIntegral(base_i, omega, R_BL, bias, dt);
  const graph_msf::BaseVerticalGyroFactor factor(
      x(0), q(0), x(1), q(1), kBias, integral.vertical_angle, integral.A, R_IC, scalarNoise());
  EXPECT_NEAR(factor.evaluateError(cabinPose(base_i, 0.4, R_IC), 0.4,
                                  cabinPose(base_j, 0.41, R_IC), 0.41, bias)(0), 0.0, 1e-12);
  EXPECT_GT((integral.A - dt * R_BL.unrotate(base_i.unrotate(gtsam::Vector3::UnitZ()))).norm(), 1e-5);
  EXPECT_NEAR(factor.evaluateError(cabinPose(base_i, 0.4, R_IC), 0.4,
                                  cabinPose(base_j, 0.41, R_IC), 0.41, gtsam::Vector3::Zero())(0),
              -integral.A.dot(bias), 1e-12);
  const double euler_heading_increment = base_j.yaw() - base_i.yaw();
  const double vertical_increment = gtsam::Rot3::Logmap(base_j * base_i.inverse()).z();
  EXPECT_GT(std::abs(euler_heading_increment - vertical_increment), 1e-4);
}

TEST(BaseVerticalGyroFactor, DrivingAndSimultaneousSlewRecoverWithoutParkedHeading) {
  // Exact synthetic cabin observations are each used once. This checks the
  // graph model, not real leg calibration, gates, transport or absolute zero.
  const auto R_IC = gtsam::Rot3::RzRyRx(0.1, -0.13, 0.35);
  const auto R_BL = gtsam::Rot3::RzRyRx(-0.4, 0.5, 0.9);
  const gtsam::Vector3 bias(0.001, -0.002, 0.003);
  constexpr double dt = 0.02;
  for (bool simultaneous_slew : {false, true}) {
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(
        kBias, bias, gtsam::noiseModel::Isotropic::Sigma(3, 0.003));
    initial.insert(kBias, bias);
    auto base = gtsam::Rot3::RzRyRx(0.2, -0.15, 3.13);
    for (size_t i = 0; i < 40; ++i) {
      const double turn = simultaneous_slew ? 0.4 * std::sin(0.1 * i) : 0.0;
      if (i > 0) {
        const gtsam::Vector3 rate(0.12 * std::cos(0.1 * i), -0.08 * std::sin(0.15 * i), 0.2);
        const auto integral = rawLegIntegral(base, rate, R_BL, bias, dt);
        graph.emplace_shared<graph_msf::BaseVerticalGyroFactor>(
            x(i - 1), q(i - 1), x(i), q(i), kBias,
            integral.vertical_angle, integral.A, R_IC, scalarNoise(1e-4));
        base = base * gtsam::Rot3::Expmap(rate * dt);
      } else {
        graph.emplace_shared<gtsam::PriorFactor<double>>(q(i), turn, scalarNoise(0.035));
      }
      const auto pose = cabinPose(base, turn, R_IC);
      graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(x(i), pose, poseNoise(0.001));
      initial.insert(x(i), pose);
      initial.insert(q(i), turn + 0.03);
    }
    const auto result = solve(graph, initial);
    EXPECT_LT(graph.error(result), 1e-9);
    for (size_t i = 0; i < 40; ++i) {
      const double turn = simultaneous_slew ? 0.4 * std::sin(0.1 * i) : 0.0;
      EXPECT_NEAR(result.at<double>(q(i)), turn, 1e-7);
    }
  }
}

TEST(BaseVerticalGyroFactor, SharedUnknownSensorBiasProducesQuadraticTurnVariance) {
  constexpr double dt = 0.02, sigma_bias = 0.006, sigma_rate = 0.0002;
  constexpr double sigma_pose = 0.0001, sigma_q0 = 0.01, true_bias_z = 0.006;
  const auto R_BL = gtsam::Rot3::RzRyRx(0.3, -0.4, 0.7);
  const gtsam::Vector3 n_sensor = R_BL.unrotate(gtsam::Vector3::UnitZ());
  // The bias uncertainty is fully three-dimensional in sensor coordinates.
  // A tilted mounting projects all three components into this level yaw test.
  for (size_t count : {size_t{11}, size_t{101}}) {
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(
        kBias, gtsam::Vector3::Zero(), gtsam::noiseModel::Isotropic::Sigma(3, sigma_bias));
    initial.insert(kBias, gtsam::Vector3::Zero().eval());
    for (size_t i = 0; i < count; ++i) {
      const double t = i * dt, turn = 0.4 + 0.01 * t;
      const auto pose = cabinPose(gtsam::Rot3::Rz(0.3 + 0.04 * t), turn);
      graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(x(i), pose, poseNoise(sigma_pose));
      if (i == 0) {
        graph.emplace_shared<gtsam::PriorFactor<double>>(q(i), turn, scalarNoise(sigma_q0));
      } else {
        graph.emplace_shared<graph_msf::BaseVerticalGyroFactor>(
            x(i - 1), q(i - 1), x(i), q(i), kBias, (0.04 + true_bias_z) * dt,
            n_sensor * dt, gtsam::Rot3(), scalarNoise(sigma_rate * std::sqrt(dt)));
      }
      initial.insert(x(i), pose);
      initial.insert(q(i), turn - true_bias_z * t);
    }
    const auto result = solve(graph, initial);
    const double elapsed = (count - 1) * dt;
    const double expected = sigma_q0 * sigma_q0 + 2 * sigma_pose * sigma_pose +
                            elapsed * sigma_rate * sigma_rate +
                            elapsed * elapsed * sigma_bias * sigma_bias;
    const gtsam::Marginals marginals(graph, result);
    EXPECT_LT(graph.error(result), 1e-10);
    EXPECT_NEAR(result.at<double>(q(count - 1)), 0.4 + (0.01 - true_bias_z) * elapsed, 1e-9);
    EXPECT_NEAR(marginals.marginalCovariance(q(count - 1))(0, 0), expected, expected * 1e-5);
    EXPECT_LT(result.at<gtsam::Vector3>(kBias).norm(), 1e-10);
    const auto covariance = marginals.marginalCovariance(kBias);
    EXPECT_NEAR(n_sensor.dot(covariance * n_sensor), sigma_bias * sigma_bias, 1e-10);
  }
}

TEST(BaseVerticalGyroFactor, ChangingAxesExposeTheDocumentedDiscretizationError) {
  // Two successive world-horizontal rotations have zero integrated vertical
  // rate but a nonzero endpoint log-z. Do not mistake this model error for
  // independent sensor precision, or bridge longer intervals with this factor.
  double previous_error = 0.0;
  for (double dt : {0.02, 0.04}) {
    const auto base_i = gtsam::Rot3::RzRyRx(0.2, -0.3, 0.4);
    const auto base_j = gtsam::Rot3::Ry(0.5 * dt) * gtsam::Rot3::Rx(0.5 * dt) * base_i;
    const graph_msf::BaseVerticalGyroFactor factor(
        x(0), q(0), x(1), q(1), kBias, 0.0, gtsam::Vector3(0, 0, dt),
        gtsam::Rot3(), scalarNoise());
    const double error = std::abs(factor.evaluateError(
        cabinPose(base_i, 0.0), 0.0, cabinPose(base_j, 0.0), 0.0, gtsam::Vector3::Zero())(0));
    EXPECT_NEAR(error, 0.125 * dt * dt, 1e-8);
    if (previous_error != 0.0) EXPECT_NEAR(error / previous_error, 4.0, 1e-3);
    previous_error = error;
  }
}

TEST(BaseVerticalGyroFactor, RejectsNonfiniteMeasurementsAndWrongNoiseDimension) {
  const auto make = [](double z, const gtsam::Vector3& A, const gtsam::SharedNoiseModel& model) {
    return graph_msf::BaseVerticalGyroFactor(x(0), q(0), x(1), q(1), kBias, z, A, gtsam::Rot3(), model);
  };
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(make(nan, gtsam::Vector3(0, 0, 0.02), scalarNoise()), std::invalid_argument);
  EXPECT_THROW(make(0, gtsam::Vector3(nan, 0, 0.02), scalarNoise()), std::invalid_argument);
  EXPECT_THROW(make(0, gtsam::Vector3(0, 0, 0.02),
                    gtsam::noiseModel::Isotropic::Sigma(3, 0.01)), std::invalid_argument);
}

}  // namespace
