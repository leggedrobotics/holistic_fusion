#include <gtest/gtest.h>

#include <graph_msf/factors/non_expression/ArticulatedTurnFactors.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>

#include <functional>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;

const gtsam::Key kPoseI = gtsam::Symbol('x', 0);
const gtsam::Key kPoseJ = gtsam::Symbol('x', 1);
const gtsam::Key kTurnI = gtsam::Symbol('q', 0);
const gtsam::Key kTurnJ = gtsam::Symbol('q', 1);
const gtsam::Key kHeading = gtsam::Symbol('h', 0);

gtsam::SharedNoiseModel noise(size_t dimension) {
  return gtsam::noiseModel::Isotropic::Sigma(dimension, 0.02);
}

gtsam::Pose3 cabinImuPose(const gtsam::Rot3& base_rotation, double q,
                         const gtsam::Rot3& R_IC) {
  return gtsam::Pose3(base_rotation * gtsam::Rot3::Rz(q) * R_IC.inverse(),
                      gtsam::Point3(1.2, -3.4, 0.7));
}

// Independent central differences in GTSAM's manifold coordinates. Production
// factors do not use these functions or numerical derivatives.
gtsam::Matrix finitePoseJacobian(
    const std::function<gtsam::Vector(const gtsam::Pose3&)>& residual,
    const gtsam::Pose3& pose) {
  constexpr double step = 1e-6;
  gtsam::Matrix jacobian(residual(pose).size(), 6);
  for (int column = 0; column < 6; ++column) {
    gtsam::Vector6 delta = gtsam::Vector6::Zero();
    delta(column) = step;
    jacobian.col(column) =
        (residual(pose.retract(delta)) - residual(pose.retract(-delta))) / (2.0 * step);
  }
  return jacobian;
}

gtsam::Matrix finiteScalarJacobian(
    const std::function<gtsam::Vector(double)>& residual, double value) {
  constexpr double step = 1e-6;
  return (residual(value + step) - residual(value - step)) / (2.0 * step);
}

void expectMatrixNear(const gtsam::Matrix& actual, const gtsam::Matrix& expected,
                       double tolerance = 2e-7) {
  ASSERT_EQ(actual.rows(), expected.rows());
  ASSERT_EQ(actual.cols(), expected.cols());
  EXPECT_LT((actual - expected).cwiseAbs().maxCoeff(), tolerance)
      << "actual:\n" << actual << "\nexpected:\n" << expected;
}

TEST(ArticulatedTurnFactors, FullRotationRecoversTiltedBaseWithExtrinsic) {
  const auto R_IC = gtsam::Rot3::RzRyRx(0.12, -0.21, 0.37);
  const auto base = gtsam::Rot3::RzRyRx(10.0 * kDeg, -13.0 * kDeg, 174.0 * kDeg);
  for (const double turn_deg : {-179.9, -130.0, -90.0, -45.0, 0.0, 45.0, 90.0, 130.0, 179.9}) {
    const double q = turn_deg * kDeg;
    const auto pose = cabinImuPose(base, q, R_IC);
    EXPECT_LT(gtsam::Rot3::Logmap(
                  base.between(graph_msf::articulatedBaseRotation(pose, q, R_IC))).norm(),
              1e-12);
  }
}

TEST(ArticulatedTurnFactors, ParkedHeadingAllowsLeanAndBothSlewDirections) {
  const auto R_IC = gtsam::Rot3::RzRyRx(-0.08, 0.04, 0.31);
  const graph_msf::ParkedBaseHeadingFactor factor(kPoseI, kTurnI, kHeading, R_IC, noise(1));
  const double heading = -178.0 * kDeg;
  for (const double roll : {-15.0 * kDeg, 0.0, 12.0 * kDeg}) {
    for (const double pitch : {-10.0 * kDeg, 0.0, 17.0 * kDeg}) {
      const auto base = gtsam::Rot3::RzRyRx(roll, pitch, heading);
      for (const double q : {-130.0 * kDeg, -45.0 * kDeg, 0.0, 90.0 * kDeg, 130.0 * kDeg}) {
        EXPECT_NEAR(factor.evaluateError(cabinImuPose(base, q, R_IC), q, heading)(0), 0.0, 1e-12);
      }
    }
  }
}

TEST(ArticulatedTurnFactors, ParkedHeadingWrapsWithoutEulerSubtraction) {
  const auto R_IC = gtsam::Rot3::Identity();
  const auto base = gtsam::Rot3::RzRyRx(10.0 * kDeg, 10.0 * kDeg, 179.9 * kDeg);
  const double q = 90.0 * kDeg;
  const auto pose = cabinImuPose(base, q, R_IC);
  const graph_msf::ParkedBaseHeadingFactor factor(kPoseI, kTurnI, kHeading, R_IC, noise(1));
  EXPECT_NEAR(factor.evaluateError(pose, q, -180.1 * kDeg)(0), 0.0, 1e-12);
  EXPECT_NEAR(factor.evaluateError(pose, q + 2.0 * kPi, -179.9 * kDeg)(0), -0.2 * kDeg, 1e-12);
  const double naive_error = graph_msf::articulated_turn_detail::wrapAngle(
      pose.rotation().yaw() - q - base.yaw());
  EXPECT_GT(std::abs(naive_error), 1.0 * kDeg);
}

TEST(ArticulatedTurnFactors, ParkedHeadingAnalyticJacobiansAtTiltAndYawWrap) {
  const auto R_IC = gtsam::Rot3::RzRyRx(0.11, -0.16, 0.29);
  const double q = -175.0 * kDeg;
  const double heading = -179.8 * kDeg;
  const auto base = gtsam::Rot3::RzRyRx(0.21, -0.18, 179.9 * kDeg);
  const auto pose = cabinImuPose(base, q, R_IC);
  const graph_msf::ParkedBaseHeadingFactor factor(kPoseI, kTurnI, kHeading, R_IC, noise(1));
  gtsam::Matrix H_pose, H_turn, H_heading;
  factor.evaluateError(pose, q, heading, H_pose, H_turn, H_heading);
  expectMatrixNear(H_pose, finitePoseJacobian(
      [&](const auto& p) { return factor.evaluateError(p, q, heading); }, pose));
  expectMatrixNear(H_turn, finiteScalarJacobian(
      [&](double turn) { return factor.evaluateError(pose, turn, heading); }, q));
  expectMatrixNear(H_heading, finiteScalarJacobian(
      [&](double h) { return factor.evaluateError(pose, q, h); }, heading));
  EXPECT_EQ(H_pose.rightCols(3).norm(), 0.0);
}

TEST(ArticulatedTurnFactors, VerticalBaseForwardAxisHasNoHeading) {
  const auto R_IC = gtsam::Rot3::Identity();
  const auto base = gtsam::Rot3::Ry(kPi / 2.0);
  const graph_msf::ParkedBaseHeadingFactor factor(kPoseI, kTurnI, kHeading, R_IC, noise(1));
  EXPECT_THROW(factor.evaluateError(cabinImuPose(base, 0.7, R_IC), 0.7, 0.0), std::domain_error);
}

TEST(ArticulatedTurnFactors, GravityIsCoupledToTurnAndInvariantToWorldYaw) {
  const auto R_IC = gtsam::Rot3::RzRyRx(0.08, 0.12, -0.19);
  const auto base = gtsam::Rot3::RzRyRx(0.18, -0.24, 1.4);
  const gtsam::Unit3 measured(base.unrotate(gtsam::Vector3::UnitZ()));
  const graph_msf::BaseGravityFactor factor(kPoseI, kTurnI, measured, R_IC, noise(2));
  const double q = -1.9;
  const auto pose = cabinImuPose(base, q, R_IC);
  EXPECT_LT(factor.evaluateError(pose, q).norm(), 1e-12);
  EXPECT_GT(factor.evaluateError(pose, q + 0.1).norm(), 0.01);
  const auto rotated = gtsam::Pose3(gtsam::Rot3::Rz(0.7) * pose.rotation(), pose.translation());
  EXPECT_LT(factor.evaluateError(rotated, q).norm(), 1e-12);
}

TEST(ArticulatedTurnFactors, GravityAnalyticJacobiansAtZeroAndNonzeroResidual) {
  const auto R_IC = gtsam::Rot3::RzRyRx(0.12, -0.06, 0.28);
  const auto base = gtsam::Rot3::RzRyRx(-0.17, 0.22, 2.3);
  const double q = 1.8;
  const auto pose = cabinImuPose(base, q, R_IC);
  for (const double offset : {0.0, 0.04, -0.4}) {
    const auto measurement_rotation = base * gtsam::Rot3::Rx(offset);
    const gtsam::Unit3 measured(measurement_rotation.unrotate(gtsam::Vector3::UnitZ()));
    const graph_msf::BaseGravityFactor factor(kPoseI, kTurnI, measured, R_IC, noise(2));
    gtsam::Matrix H_pose, H_turn;
    const auto residual = factor.evaluateError(pose, q, H_pose, H_turn);
    expectMatrixNear(H_pose, finitePoseJacobian(
        [&](const auto& p) { return factor.evaluateError(p, q); }, pose));
    expectMatrixNear(H_turn, finiteScalarJacobian(
        [&](double turn) { return factor.evaluateError(pose, turn); }, q));
    const gtsam::Unit3 predicted(base.unrotate(gtsam::Vector3::UnitZ()));
    EXPECT_LT((residual - measured.localCoordinates(predicted)).norm(), 1e-12);
    EXPECT_EQ(H_pose.rightCols(3).norm(), 0.0);
  }
}

TEST(ArticulatedTurnFactors, GravityDoesNotReportZeroForAntipodalDirection) {
  const auto R_IC = gtsam::Rot3::Identity();
  const graph_msf::BaseGravityFactor factor(
      kPoseI, kTurnI, gtsam::Unit3(0.0, 0.0, -1.0), R_IC, noise(2));
  EXPECT_THROW(factor.evaluateError(gtsam::Pose3(), 0.0), std::domain_error);
  const auto near_antipodal = cabinImuPose(gtsam::Rot3::Rx(0.01), 0.0, R_IC);
  EXPECT_NEAR(factor.evaluateError(near_antipodal, 0.0).norm(), kPi - 0.01, 1e-12);
}

TEST(ArticulatedTurnFactors, BaseBetweenAcceptsSlewWithoutBaseRotation) {
  const auto R_IC = gtsam::Rot3::RzRyRx(0.09, -0.11, 0.26);
  const auto base = gtsam::Rot3::RzRyRx(0.14, 0.19, -2.2);
  const double q_i = -130.0 * kDeg;
  const double q_j = 130.0 * kDeg;
  const graph_msf::BaseRotationBetweenFactor factor(
      kPoseI, kTurnI, kPoseJ, kTurnJ, gtsam::Rot3::Identity(), R_IC, noise(3));
  EXPECT_LT(factor.evaluateError(cabinImuPose(base, q_i, R_IC), q_i,
                                 cabinImuPose(base, q_j, R_IC), q_j).norm(), 1e-12);
}

TEST(ArticulatedTurnFactors, BaseBetweenAllowsBaseMotionWithFixedTurn) {
  const auto R_IC = gtsam::Rot3::RzRyRx(-0.07, 0.13, 0.37);
  const auto base_i = gtsam::Rot3::RzRyRx(0.1, -0.12, 2.8);
  const auto base_j = gtsam::Rot3::RzRyRx(0.15, -0.08, -2.9);
  const double q = 1.3;
  const graph_msf::BaseRotationBetweenFactor factor(
      kPoseI, kTurnI, kPoseJ, kTurnJ, base_i.between(base_j), R_IC, noise(3));
  const auto pose_i = cabinImuPose(base_i, q, R_IC);
  const auto pose_j = cabinImuPose(base_j, q, R_IC);
  EXPECT_LT(factor.evaluateError(pose_i, q, pose_j, q).norm(), 1e-12);
  EXPECT_NEAR(factor.evaluateError(pose_i, q, pose_j, q + 0.04).norm(), 0.04, 1e-12);
}

TEST(ArticulatedTurnFactors, BaseBetweenAnalyticJacobiansAtNonzeroResidual) {
  const auto R_IC = gtsam::Rot3::RzRyRx(0.13, -0.16, 0.29);
  const auto base_i = gtsam::Rot3::RzRyRx(0.19, -0.25, 3.1);
  const auto base_j = gtsam::Rot3::RzRyRx(0.24, -0.21, -3.0);
  const double q_i = 2.2;
  const double q_j = -2.1;
  const auto pose_i = cabinImuPose(base_i, q_i, R_IC);
  const auto pose_j = cabinImuPose(base_j, q_j, R_IC);
  const auto measurement = base_i.between(base_j) * gtsam::Rot3::RzRyRx(0.03, -0.02, 0.04);
  const graph_msf::BaseRotationBetweenFactor factor(
      kPoseI, kTurnI, kPoseJ, kTurnJ, measurement, R_IC, noise(3));
  gtsam::Matrix H_pose_i, H_turn_i, H_pose_j, H_turn_j;
  factor.evaluateError(pose_i, q_i, pose_j, q_j, H_pose_i, H_turn_i, H_pose_j, H_turn_j);
  expectMatrixNear(H_pose_i, finitePoseJacobian(
      [&](const auto& p) { return factor.evaluateError(p, q_i, pose_j, q_j); }, pose_i));
  expectMatrixNear(H_turn_i, finiteScalarJacobian(
      [&](double q) { return factor.evaluateError(pose_i, q, pose_j, q_j); }, q_i));
  expectMatrixNear(H_pose_j, finitePoseJacobian(
      [&](const auto& p) { return factor.evaluateError(pose_i, q_i, p, q_j); }, pose_j));
  expectMatrixNear(H_turn_j, finiteScalarJacobian(
      [&](double q) { return factor.evaluateError(pose_i, q_i, pose_j, q); }, q_j));
}

TEST(ArticulatedTurnFactors, NativeFactorsCloneAndEvaluateThroughValues) {
  const auto R_IC = gtsam::Rot3::Identity();
  const auto base = gtsam::Rot3::RzRyRx(0.12, 0.17, 1.2);
  const double q_i = 0.7;
  const double q_j = -1.1;
  gtsam::Values values;
  values.insert(kPoseI, cabinImuPose(base, q_i, R_IC));
  values.insert(kPoseJ, cabinImuPose(base, q_j, R_IC));
  values.insert(kTurnI, q_i);
  values.insert(kTurnJ, q_j);
  values.insert(kHeading, base.yaw());
  const graph_msf::ParkedBaseHeadingFactor heading(kPoseI, kTurnI, kHeading, R_IC, noise(1));
  const graph_msf::BaseGravityFactor gravity(
      kPoseI, kTurnI, gtsam::Unit3(base.unrotate(gtsam::Vector3::UnitZ())), R_IC, noise(2));
  const graph_msf::BaseRotationBetweenFactor between(
      kPoseI, kTurnI, kPoseJ, kTurnJ, gtsam::Rot3::Identity(), R_IC, noise(3));
  for (const gtsam::NonlinearFactor* factor :
       std::vector<const gtsam::NonlinearFactor*>{&heading, &gravity, &between}) {
    const auto clone = factor->clone();
    EXPECT_NE(clone.get(), factor);
    EXPECT_EQ(clone->keys(), factor->keys());
    EXPECT_NEAR(clone->error(values), factor->error(values), 1e-12);
    EXPECT_NEAR(clone->error(values), 0.0, 1e-20);
  }
}

TEST(ArticulatedTurnFactors, BaseAhrsAnalyticJacobiansIncludeSharedGyroBias) {
  const auto R_IC = gtsam::Rot3::RzRyRx(0.13, -0.16, 0.29);
  const auto base_i = gtsam::Rot3::RzRyRx(0.19, -0.25, 3.1);
  const gtsam::Vector3 nominal_bias(0.01, -0.015, 0.02);
  const gtsam::Vector3 evaluated_bias(0.012, -0.016, 0.018);
  const auto params = std::make_shared<gtsam::PreintegratedRotationParams>();
  params->gyroscopeCovariance = 1e-6 * gtsam::Matrix33::Identity();
  gtsam::PreintegratedAhrsMeasurements measurements(params, nominal_bias);
  for (int i = 0; i < 40; ++i) {
    measurements.integrateMeasurement(gtsam::Vector3(0.12 + 0.001 * i, -0.09, 0.45), 0.0025);
  }
  const auto base_j = measurements.predict(base_i, evaluated_bias) *
                      gtsam::Rot3::RzRyRx(0.03, -0.02, 0.04);
  const double q_i = 2.2;
  const double q_j = -2.1;
  const auto pose_i = cabinImuPose(base_i, q_i, R_IC);
  const auto pose_j = cabinImuPose(base_j, q_j, R_IC);
  const auto bias_key = gtsam::Symbol('d', 0);
  const graph_msf::BaseAhrsBetweenFactor factor(
      kPoseI, kTurnI, kPoseJ, kTurnJ, bias_key, measurements, R_IC);
  gtsam::Matrix H_pose_i, H_turn_i, H_pose_j, H_turn_j, H_bias;
  factor.evaluateError(pose_i, q_i, pose_j, q_j, evaluated_bias,
                        H_pose_i, H_turn_i, H_pose_j, H_turn_j, H_bias);
  expectMatrixNear(H_pose_i, finitePoseJacobian(
      [&](const auto& p) { return factor.evaluateError(p, q_i, pose_j, q_j, evaluated_bias); }, pose_i));
  expectMatrixNear(H_turn_i, finiteScalarJacobian(
      [&](double q) { return factor.evaluateError(pose_i, q, pose_j, q_j, evaluated_bias); }, q_i));
  expectMatrixNear(H_pose_j, finitePoseJacobian(
      [&](const auto& p) { return factor.evaluateError(pose_i, q_i, p, q_j, evaluated_bias); }, pose_j));
  expectMatrixNear(H_turn_j, finiteScalarJacobian(
      [&](double q) { return factor.evaluateError(pose_i, q_i, pose_j, q, evaluated_bias); }, q_j));
  gtsam::Matrix33 finite_bias;
  for (int column = 0; column < 3; ++column) {
    gtsam::Vector3 step = gtsam::Vector3::Zero();
    step(column) = 1e-6;
    finite_bias.col(column) =
        (factor.evaluateError(pose_i, q_i, pose_j, q_j, evaluated_bias + step) -
         factor.evaluateError(pose_i, q_i, pose_j, q_j, evaluated_bias - step)) / 2e-6;
  }
  expectMatrixNear(H_bias, finite_bias);
  gtsam::Values values;
  values.insert(kPoseI, pose_i);
  values.insert(kTurnI, q_i);
  values.insert(kPoseJ, pose_j);
  values.insert(kTurnJ, q_j);
  values.insert(bias_key, evaluated_bias);
  EXPECT_NEAR(factor.clone()->error(values), factor.error(values), 1e-12);
}

}  // namespace
