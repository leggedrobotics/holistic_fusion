#include <cmath>
#include <functional>
#include <vector>

#include <Eigen/LU>
#include <gtest/gtest.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#if __has_include(<gtsam/nonlinear/IncrementalFixedLagSmoother.h>)
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#else
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#endif
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>

#include "graph_msf/factors/non_expression/ArticulatedTurnFactors.h"

namespace graph_msf {
namespace {

constexpr double kDeg = M_PI / 180.0;
const gtsam::Key kHeading = gtsam::Symbol('h', 0);

gtsam::Key poseKey(size_t i) { return gtsam::Symbol('x', i); }
gtsam::Key turnKey(size_t i) { return gtsam::Symbol('q', i); }

gtsam::SharedNoiseModel scalarNoise(double sigma) {
  return gtsam::noiseModel::Isotropic::Sigma(1, sigma);
}

gtsam::SharedNoiseModel poseNoise(double rotationSigma) {
  gtsam::Vector6 sigmas;
  sigmas << rotationSigma, rotationSigma, rotationSigma, 0.01, 0.01, 0.01;
  return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
}

gtsam::Pose3 cabinImuPose(const gtsam::Rot3& base, double turn,
                         const gtsam::Rot3& cabinToImu = gtsam::Rot3()) {
  return gtsam::Pose3(base * gtsam::Rot3::Rz(turn) * cabinToImu.inverse(),
                      gtsam::Point3(1.0, 2.0, 0.5));
}

gtsam::Values optimize(const gtsam::NonlinearFactorGraph& graph,
                       const gtsam::Values& initial) {
  gtsam::LevenbergMarquardtParams params;
  params.setMaxIterations(100);
  params.setAbsoluteErrorTol(1e-12);
  params.setRelativeErrorTol(1e-12);
  return gtsam::LevenbergMarquardtOptimizer(graph, initial, params).optimize();
}

struct StationGraph {
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial;
  std::vector<double> turns;
};

// Each synthetic absolute cabin attitude is used exactly once. In particular,
// there is no additional q observation derived from these same cabin poses.
// The sole initial q prior represents an independently established reference.
StationGraph makeLevelStation(size_t count, double poseSigma, double initialTurnSigma,
                              double parkedSigma, bool addInitialReference = true) {
  StationGraph station;
  constexpr double baseHeading = 0.4;
  station.initial.insert(kHeading, baseHeading + 0.03);
  for (size_t i = 0; i < count; ++i) {
    const double q = 1.2 * std::sin(0.4 * static_cast<double>(i));
    const auto pose = cabinImuPose(gtsam::Rot3::Rz(baseHeading), q);
    station.turns.push_back(q);
    station.graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(poseKey(i), pose, poseNoise(poseSigma));
    station.graph.emplace_shared<ParkedBaseHeadingFactor>(poseKey(i), turnKey(i), kHeading,
                                                         gtsam::Rot3(), scalarNoise(parkedSigma));
    station.initial.insert(poseKey(i), pose);
    station.initial.insert(turnKey(i), q - 0.03);
  }
  if (addInitialReference) {
    station.graph.emplace_shared<gtsam::PriorFactor<double>>(turnKey(0), station.turns.front(),
                                                           scalarNoise(initialTurnSigma));
  }
  return station;
}

TEST(ArticulatedTurnGraph, TiltedSwingsAndChangingLeanRecoverJointWithNonidentityImuExtrinsic) {
  const gtsam::Rot3 R_IC = gtsam::Rot3::RzRyRx(0.11, -0.07, 0.6);
  constexpr double heading = 1.1;
  const std::vector<double> turns{0, 45 * kDeg, 90 * kDeg, -90 * kDeg, 130 * kDeg, -130 * kDeg, 0};
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial;
  initial.insert(kHeading, heading + 0.04);
  graph.emplace_shared<gtsam::PriorFactor<double>>(turnKey(0), 0.0, scalarNoise(0.2 * kDeg));
  for (size_t i = 0; i < turns.size(); ++i) {
    const auto base = gtsam::Rot3::RzRyRx((10.0 + std::sin(i)) * kDeg,
                                         (-12.0 + 2.0 * std::cos(i)) * kDeg, heading);
    const auto measured = cabinImuPose(base, turns[i], R_IC);
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(poseKey(i), measured, poseNoise(0.1 * kDeg));
    graph.emplace_shared<ParkedBaseHeadingFactor>(poseKey(i), turnKey(i), kHeading, R_IC,
                                                scalarNoise(0.01 * kDeg));
    gtsam::Vector6 perturbation = gtsam::Vector6::Zero();
    perturbation.head<3>() << 0.01, -0.015, 0.02;
    initial.insert(poseKey(i), measured.retract(perturbation));
    initial.insert(turnKey(i), turns[i] + 0.05);
  }
  const auto result = optimize(graph, initial);
  EXPECT_LT(graph.error(result), 1e-9);
  EXPECT_NEAR(result.at<double>(kHeading), heading, 1e-8);
  for (size_t i = 0; i < turns.size(); ++i) {
    EXPECT_NEAR(result.at<double>(turnKey(i)), turns[i], 1e-8);
  }
}

TEST(ArticulatedTurnGraph, RepeatedCabinObservationsDoNotAverageAwayCommonInitialTurnUncertainty) {
  const double sigmaPose = 0.1 * kDeg;
  const double sigmaReference = 2.0 * kDeg;
  const double sigmaParked = 0.01 * kDeg;
  std::vector<double> headingVariances;
  for (size_t count : {size_t{2}, size_t{100}}) {
    const auto station = makeLevelStation(count, sigmaPose, sigmaReference, sigmaParked);
    const auto result = optimize(station.graph, station.initial);
    const gtsam::Marginals marginals(station.graph, result);
    const double expectedHeadingVariance =
        sigmaReference * sigmaReference + sigmaPose * sigmaPose + sigmaParked * sigmaParked;
    const double varHeading = marginals.marginalCovariance(kHeading)(0, 0);
    const double varTurn = marginals.marginalCovariance(turnKey(count - 1))(0, 0);
    headingVariances.push_back(varHeading);
    EXPECT_NEAR(varHeading, expectedHeadingVariance, expectedHeadingVariance * 1e-5);
    EXPECT_GE(varTurn, sigmaReference * sigmaReference);
    EXPECT_NEAR(varTurn, expectedHeadingVariance + sigmaPose * sigmaPose + sigmaParked * sigmaParked,
                expectedHeadingVariance * 1e-5);
    // A free q at this epoch cannot improve the original cabin measurement.
    // Feeding a cabin-derived pseudo-q back as independent information would.
    EXPECT_NEAR(marginals.marginalCovariance(poseKey(count - 1))(2, 2), sigmaPose * sigmaPose,
                sigmaPose * sigmaPose * 1e-5);
  }
  EXPECT_NEAR(headingVariances[0], headingVariances[1], headingVariances[0] * 1e-5);
}

TEST(ArticulatedTurnGraph, BaseHeadingCovarianceRequiresPoseTurnCrossCovariance) {
  const double sigmaPose = 0.05;
  const double sigmaReference = 0.01;
  const double sigmaParked = 0.0001;
  const auto station = makeLevelStation(2, sigmaPose, sigmaReference, sigmaParked);
  const auto result = optimize(station.graph, station.initial);
  const gtsam::Marginals marginals(station.graph, result);
  const auto joint = marginals.jointMarginalCovariance({poseKey(1), turnKey(1)});
  const auto X = result.at<gtsam::Pose3>(poseKey(1));
  const double q = result.at<double>(turnKey(1));
  const std::function<gtsam::Vector1(const gtsam::Pose3&, const double&)> baseHeading =
      [](const gtsam::Pose3& pose, const double& turn) {
        const gtsam::Matrix3 R = pose.rotation().matrix() * gtsam::Rot3::Rz(-turn).matrix();
        return gtsam::Vector1(std::atan2(R(1, 0), R(0, 0)));
      };
  const gtsam::Matrix Hx = gtsam::numericalDerivative21<gtsam::Vector1, gtsam::Pose3, double>(baseHeading, X, q);
  const gtsam::Matrix Hq = gtsam::numericalDerivative22<gtsam::Vector1, gtsam::Pose3, double>(baseHeading, X, q);
  const double independentVariance =
      (Hx * joint.at(poseKey(1), poseKey(1)) * Hx.transpose() +
       Hq * joint.at(turnKey(1), turnKey(1)) * Hq.transpose())(0, 0);
  const double crossTerm =
      (Hx * joint.at(poseKey(1), turnKey(1)) * Hq.transpose() +
       Hq * joint.at(turnKey(1), poseKey(1)) * Hx.transpose())(0, 0);
  const double fullVariance = independentVariance + crossTerm;
  const double expected = sigmaReference * sigmaReference + sigmaPose * sigmaPose +
                          2 * sigmaParked * sigmaParked;
  EXPECT_NEAR(fullVariance, expected, expected * 1e-5);
  EXPECT_NEAR(crossTerm, -2 * sigmaPose * sigmaPose, 1e-8);
  EXPECT_GT(independentVariance, 2.5 * fullVariance);
  EXPECT_GT(joint.at(poseKey(1), turnKey(1)).norm(), 1e-3);
}

TEST(ArticulatedTurnGraph, LevelGroundWithoutInitialReferenceHasAnUnobservableCommonAngleGauge) {
  auto station = makeLevelStation(4, 0.01, 0.1, 0.001, false);
  // Evaluate at an exact feasible point, then shift q and H together without
  // changing a single physical cabin observation.
  station.initial.update(kHeading, 0.4);
  for (size_t i = 0; i < station.turns.size(); ++i) {
    station.initial.update(turnKey(i), station.turns[i]);
  }
  auto shifted = station.initial;
  shifted.update(kHeading, 0.4 - 0.25);
  for (size_t i = 0; i < station.turns.size(); ++i) {
    shifted.update(turnKey(i), station.turns[i] + 0.25);
  }
  EXPECT_LT(station.graph.error(station.initial), 1e-15);
  EXPECT_LT(station.graph.error(shifted), 1e-15);
  const auto jacobian = station.graph.linearize(station.initial)->jacobian().first;
  Eigen::FullPivLU<gtsam::Matrix> decomposition(jacobian);
  decomposition.setThreshold(1e-9);
  EXPECT_EQ(decomposition.rank(), jacobian.cols() - 1);
}

TEST(ArticulatedTurnGraph, IndependentlyMeasuredBaseGravityMakesJointObservableOnASlope) {
  // This gravity measurement is independently calibrated. A leg vector first
  // anchored using the graph's own q/X is not an independent measurement and
  // would require its calibration state/correlation in the graph.
  const gtsam::Rot3 R_IC = gtsam::Rot3::RzRyRx(0.12, -0.03, 0.4);
  const auto base = gtsam::Rot3::RzRyRx(10 * kDeg, -8 * kDeg, 0.7);
  const double trueTurn = 0.9;
  const auto pose = cabinImuPose(base, trueTurn, R_IC);
  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(poseKey(0), pose, poseNoise(0.05 * kDeg));
  graph.emplace_shared<BaseGravityFactor>(poseKey(0), turnKey(0),
                                         gtsam::Unit3(base.unrotate(gtsam::Vector3::UnitZ())),
                                         R_IC, gtsam::noiseModel::Isotropic::Sigma(2, 0.05 * kDeg));
  graph.emplace_shared<ParkedBaseHeadingFactor>(poseKey(0), turnKey(0), kHeading, R_IC,
                                               scalarNoise(0.01 * kDeg));
  gtsam::Values initial;
  initial.insert(poseKey(0), pose);
  initial.insert(turnKey(0), trueTurn + 0.15);
  initial.insert(kHeading, 0.7 - 0.15);
  const auto result = optimize(graph, initial);
  EXPECT_NEAR(result.at<double>(turnKey(0)), trueTurn, 1e-8);
  EXPECT_NEAR(result.at<double>(kHeading), 0.7, 1e-8);
  const gtsam::Marginals marginals(graph, result);
  const double turnSigma = std::sqrt(marginals.marginalCovariance(turnKey(0))(0, 0));
  EXPECT_GT(turnSigma, 0.0);
  EXPECT_LT(turnSigma, 1 * kDeg);
}

TEST(ArticulatedTurnGraph, MeasuredBaseMotionDoesNotBecomeFalseJointRotation) {
  const gtsam::Rot3 R_IC = gtsam::Rot3::RzRyRx(-0.1, 0.08, 0.5);
  const auto base0 = gtsam::Rot3::RzRyRx(8 * kDeg, -6 * kDeg, 0.2);
  const auto base1 = gtsam::Rot3::RzRyRx(11 * kDeg, -4 * kDeg, 0.8);
  constexpr double trueTurn = 0.65;
  const auto pose0 = cabinImuPose(base0, trueTurn, R_IC);
  const auto pose1 = cabinImuPose(base1, trueTurn, R_IC);
  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(poseKey(0), pose0, poseNoise(0.001));
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(poseKey(1), pose1, poseNoise(0.001));
  graph.emplace_shared<gtsam::PriorFactor<double>>(turnKey(0), trueTurn, scalarNoise(0.01));
  graph.emplace_shared<BaseRotationBetweenFactor>(
      poseKey(0), turnKey(0), poseKey(1), turnKey(1), base0.between(base1), R_IC,
      gtsam::noiseModel::Isotropic::Sigma(3, 0.001));
  gtsam::Values initial;
  initial.insert(poseKey(0), pose0);
  initial.insert(poseKey(1), pose1);
  initial.insert(turnKey(0), trueTurn);
  initial.insert(turnKey(1), trueTurn + 0.6);
  const auto result = optimize(graph, initial);
  EXPECT_LT(graph.error(result), 1e-9);
  EXPECT_NEAR(result.at<double>(turnKey(1)), trueTurn, 1e-8);
  EXPECT_NEAR(result.at<double>(turnKey(1)) - result.at<double>(turnKey(0)), 0.0, 1e-8);
}

TEST(ArticulatedTurnGraph, RefreshingStationTimestampRetainsReferenceUncertaintyAfterFixedLagMarginalization) {
  gtsam::ISAM2Params params;
  params.relinearizeSkip = 1;
  gtsam::IncrementalFixedLagSmoother smoother(2.0, params);
  constexpr double trueHeading = 0.4;
  const double sigmaReference = 2 * kDeg;
  const double sigmaPose = 0.1 * kDeg;
  const double sigmaParked = 0.01 * kDeg;
  for (size_t i = 0; i < 12; ++i) {
    const double t = static_cast<double>(i);
    const double trueTurn = 0.8 * std::sin(0.3 * t);
    const auto pose = cabinImuPose(gtsam::Rot3::Rz(trueHeading), trueTurn);
    gtsam::NonlinearFactorGraph factors;
    factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(poseKey(i), pose, poseNoise(sigmaPose));
    factors.emplace_shared<ParkedBaseHeadingFactor>(poseKey(i), turnKey(i), kHeading, gtsam::Rot3(),
                                                   scalarNoise(sigmaParked));
    gtsam::Values values;
    values.insert(poseKey(i), pose);
    values.insert(turnKey(i), trueTurn);
    if (i == 0) {
      values.insert(kHeading, trueHeading);
      factors.emplace_shared<gtsam::PriorFactor<double>>(turnKey(0), trueTurn, scalarNoise(sigmaReference));
    }
    // Refresh an existing H key's timestamp; do not insert another H value or
    // add another prior. The q0 uncertainty survives in the marginal density.
    gtsam::FixedLagSmoother::KeyTimestampMap timestamps{
        {poseKey(i), t}, {turnKey(i), t}, {kHeading, t}};
    ASSERT_NO_THROW(smoother.update(factors, values, timestamps));
    const auto result = smoother.calculateEstimate();
    ASSERT_TRUE(result.exists(kHeading));
    EXPECT_NEAR(result.at<double>(turnKey(i)), trueTurn, 1e-8);
    EXPECT_NEAR(result.at<double>(kHeading), trueHeading, 1e-8);
    EXPECT_GE(smoother.marginalCovariance(kHeading)(0, 0), sigmaReference * sigmaReference);
    if (i >= 4) {
      EXPECT_FALSE(result.exists(poseKey(0)));
      EXPECT_FALSE(result.exists(turnKey(0)));
    }
  }
  const double expectedHeadingVariance =
      sigmaReference * sigmaReference + sigmaPose * sigmaPose + sigmaParked * sigmaParked;
  EXPECT_NEAR(smoother.marginalCovariance(kHeading)(0, 0), expectedHeadingVariance,
              expectedHeadingVariance * 1e-4);
}

TEST(ArticulatedTurnGraph, DrivingAndSimultaneousSlewRecoverTiltedTrajectoryWithoutParkedConstraint) {
  // Independent, calibrated BASE-frame increments are synthetic observations
  // in this test. No parked constraint or absolute BASE heading is supplied.
  const auto R_IC = gtsam::Rot3::RzRyRx(-0.12, 0.07, 0.35);
  const size_t count = 25;
  for (const bool slew_during_driving : {false, true}) {
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    std::vector<gtsam::Rot3> bases;
    std::vector<double> turns;
    for (size_t i = 0; i < count; ++i) {
      const double phase = static_cast<double>(i);
      const auto base = gtsam::Rot3::RzRyRx(
          (8.0 + 3.0 * std::sin(0.3 * phase)) * kDeg,
          (-7.0 + 2.0 * std::cos(0.2 * phase)) * kDeg,
          170.0 * kDeg + 0.045 * phase);
      const double q = slew_during_driving ? 0.65 + 1.4 * std::sin(0.25 * phase) : 0.65;
      bases.push_back(base);
      turns.push_back(q);
      const auto pose = cabinImuPose(base, q, R_IC);
      graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
          poseKey(i), pose, poseNoise(0.05 * kDeg));
      if (i == 0) {
        graph.emplace_shared<gtsam::PriorFactor<double>>(turnKey(i), q, scalarNoise(2.0 * kDeg));
      } else {
        graph.emplace_shared<BaseRotationBetweenFactor>(
            poseKey(i - 1), turnKey(i - 1), poseKey(i), turnKey(i),
            bases[i - 1].between(base), R_IC,
            gtsam::noiseModel::Isotropic::Sigma(3, 0.02 * kDeg));
      }
      gtsam::Vector6 perturbation = gtsam::Vector6::Zero();
      perturbation.head<3>() << 0.01, -0.015, 0.02;
      initial.insert(poseKey(i), pose.retract(perturbation));
      initial.insert(turnKey(i), q + 0.08);
    }
    const auto result = optimize(graph, initial);
    EXPECT_LT(graph.error(result), 1e-8);
    for (size_t i = 0; i < count; ++i) {
      EXPECT_NEAR(result.at<double>(turnKey(i)), turns[i], 1e-7);
      const auto recovered_base = articulatedBaseRotation(
          result.at<gtsam::Pose3>(poseKey(i)), result.at<double>(turnKey(i)), R_IC);
      EXPECT_LT(gtsam::Rot3::Logmap(bases[i].between(recovered_base)).norm(), 1e-7);
    }
  }
}

TEST(ArticulatedTurnGraph, UnobservedDrivingGyroBiasRetainsQuadraticJointUncertainty) {
  // On level ground, absolute cabin attitude plus an independent yaw-rate
  // sensor cannot distinguish BASE gyro bias from slow joint motion. The
  // deliberately biased measurement must produce honest drift/uncertainty,
  // rather than an invented calibration or a white-noise-only confidence.
  const auto bias_key = gtsam::Symbol('d', 0);
  constexpr double dt = 0.5;
  constexpr double true_base_rate = 0.04;
  constexpr double true_turn_rate = 0.01;
  constexpr double true_bias_z = 0.006;
  constexpr double sigma_bias = 0.006;
  constexpr double sigma_gyro = 0.0002;
  constexpr double sigma_pose = 0.0001;
  constexpr double sigma_reference = 0.01;
  const auto params = std::make_shared<gtsam::PreintegratedRotationParams>();
  params->gyroscopeCovariance = sigma_gyro * sigma_gyro * gtsam::Matrix33::Identity();
  for (size_t count : {size_t{3}, size_t{21}, size_t{41}}) {
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(
        bias_key, gtsam::Vector3::Zero(), gtsam::noiseModel::Isotropic::Sigma(3, sigma_bias));
    initial.insert(bias_key, gtsam::Vector3::Zero().eval());
    for (size_t i = 0; i < count; ++i) {
      const double t = i * dt;
      const double true_q = 0.65 + true_turn_rate * t;
      const auto pose = cabinImuPose(gtsam::Rot3::Rz(0.2 + true_base_rate * t), true_q);
      graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(poseKey(i), pose, poseNoise(sigma_pose));
      if (i == 0) {
        graph.emplace_shared<gtsam::PriorFactor<double>>(turnKey(i), true_q, scalarNoise(sigma_reference));
      } else {
        gtsam::PreintegratedAhrsMeasurements pim(params, gtsam::Vector3::Zero());
        pim.integrateMeasurement(gtsam::Vector3(0.0, 0.0, true_base_rate + true_bias_z), dt);
        graph.emplace_shared<BaseAhrsBetweenFactor>(
            poseKey(i - 1), turnKey(i - 1), poseKey(i), turnKey(i), bias_key, pim, gtsam::Rot3());
      }
      initial.insert(poseKey(i), pose);
      initial.insert(turnKey(i), true_q - true_bias_z * t);
    }
    const auto result = optimize(graph, initial);
    EXPECT_LT(graph.error(result), 1e-9);
    const gtsam::Marginals marginals(graph, result);
    const double elapsed = (count - 1) * dt;
    const double true_q_final = 0.65 + true_turn_rate * elapsed;
    const double expected_variance = sigma_reference * sigma_reference +
                                     2.0 * sigma_pose * sigma_pose +
                                     elapsed * sigma_gyro * sigma_gyro +
                                     elapsed * elapsed * sigma_bias * sigma_bias;
    EXPECT_NEAR(result.at<double>(turnKey(count - 1)),
                true_q_final - true_bias_z * elapsed, 1e-8);
    EXPECT_NEAR(result.at<gtsam::Vector3>(bias_key).z(), 0.0, 1e-10);
    EXPECT_NEAR(marginals.marginalCovariance(bias_key)(2, 2),
                sigma_bias * sigma_bias, sigma_bias * sigma_bias * 1e-5);
    EXPECT_NEAR(marginals.marginalCovariance(turnKey(count - 1))(0, 0),
                expected_variance, expected_variance * 1e-5);
    const double incorrectly_independent_bias_variance =
        sigma_reference * sigma_reference + 2.0 * sigma_pose * sigma_pose +
        elapsed * sigma_gyro * sigma_gyro + elapsed * dt * sigma_bias * sigma_bias;
    if (count >= 21) {
      EXPECT_GT(marginals.marginalCovariance(turnKey(count - 1))(0, 0),
                10.0 * incorrectly_independent_bias_variance);
    }
  }
}

TEST(ArticulatedTurnGraph, FixedLagParkDriveParkCarriesOneJointReferenceAndBiasStateContinuously) {
  gtsam::ISAM2Params isam_params;
  isam_params.relinearizeSkip = 1;
  isam_params.factorization = gtsam::ISAM2Params::QR;
  gtsam::IncrementalFixedLagSmoother smoother(1.0, isam_params);
  const auto bias_key = gtsam::Symbol('d', 0);
  const gtsam::Key second_heading_key = gtsam::Symbol('h', 1);
  const auto R_IC = gtsam::Rot3::RzRyRx(0.08, -0.06, 0.31);
  const gtsam::Vector3 true_bias(0.0, 0.0, 0.003);
  constexpr double dt = 0.2;
  const double sigma_reference = 2.0 * kDeg;
  const double sigma_pose = 0.05 * kDeg;
  const double sigma_parked = 0.01 * kDeg;
  const auto params = std::make_shared<gtsam::PreintegratedRotationParams>();
  params->gyroscopeCovariance = 4e-6 * gtsam::Matrix33::Identity();
  double previous_heading = 0.4;
  double variance_before_drive = 0.0;
  double variance_end_drive = 0.0;
  double previous_q_estimate = 0.0;
  double previous_true_q = 0.0;
  for (size_t i = 0; i < 60; ++i) {
    const double t = i * dt;
    const bool first_station = i < 12;
    const bool second_station = i >= 35;
    const double heading = i < 12 ? 0.4 : (i < 35 ? 0.4 + 0.04 * (i - 11) : 1.32);
    const double q = 0.65 + 1.2 * std::sin(0.12 * i);
    const auto pose = cabinImuPose(gtsam::Rot3::Rz(heading), q, R_IC);
    gtsam::NonlinearFactorGraph factors;
    gtsam::Values values;
    gtsam::FixedLagSmoother::KeyTimestampMap timestamps{
        {poseKey(i), t}, {turnKey(i), t}, {bias_key, t}};
    factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(poseKey(i), pose, poseNoise(sigma_pose));
    values.insert(poseKey(i), pose);
    values.insert(turnKey(i), q);
    if (i == 0) {
      // Exactly one reference prior over the complete parked/driving sequence.
      factors.emplace_shared<gtsam::PriorFactor<double>>(turnKey(i), q, scalarNoise(sigma_reference));
      factors.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(
          bias_key, true_bias, gtsam::noiseModel::Isotropic::Sigma(3, 0.006));
      values.insert(bias_key, true_bias);
    } else {
      gtsam::PreintegratedAhrsMeasurements pim(params, true_bias);
      const gtsam::Vector3 true_rate(0.0, 0.0, (heading - previous_heading) / dt);
      pim.integrateMeasurement(true_rate + true_bias, dt);
      factors.emplace_shared<BaseAhrsBetweenFactor>(
          poseKey(i - 1), turnKey(i - 1), poseKey(i), turnKey(i), bias_key, pim, R_IC);
    }
    if (first_station || second_station) {
      const auto station_key = first_station ? kHeading : second_heading_key;
      if (i == 0 || i == 35) {
        // A numerical seed only: no replacement q prior and no heading prior.
        values.insert(station_key, heading);
      }
      timestamps[station_key] = t;
      factors.emplace_shared<ParkedBaseHeadingFactor>(
          poseKey(i), turnKey(i), station_key, R_IC, scalarNoise(sigma_parked));
    }
    ASSERT_NO_THROW(smoother.update(factors, values, timestamps));
    const auto estimate = smoother.calculateEstimate();
    const double q_estimate = estimate.at<double>(turnKey(i));
    const double q_variance = smoother.marginalCovariance(turnKey(i))(0, 0);
    EXPECT_NEAR(q_estimate, q, 1e-8);
    EXPECT_GE(q_variance, sigma_reference * sigma_reference * (1.0 - 1e-5));
    EXPECT_TRUE(estimate.exists(bias_key));
    if (i > 0) {
      EXPECT_NEAR(q_estimate - previous_q_estimate, q - previous_true_q, 1e-8);
    }
    if (i >= 18) {
      EXPECT_FALSE(estimate.exists(kHeading));
      EXPECT_FALSE(estimate.exists(turnKey(0)));
    }
    if (i == 11) variance_before_drive = q_variance;
    if (i == 34) variance_end_drive = q_variance;
    if (i == 35) {
      // Creating the next station must not reset accumulated uncertainty.
      EXPECT_GE(q_variance, variance_before_drive);
      EXPECT_GT(q_variance, 0.99 * variance_end_drive);
      EXPECT_TRUE(estimate.exists(second_heading_key));
    }
    previous_heading = heading;
    previous_q_estimate = q_estimate;
    previous_true_q = q;
  }
  EXPECT_GT(variance_end_drive, variance_before_drive);
  const auto final = smoother.calculateEstimate();
  EXPECT_NEAR(final.at<double>(second_heading_key), 1.32, 1e-8);
  EXPECT_NEAR(final.at<gtsam::Vector3>(bias_key).z(), true_bias.z(), 1e-8);
}

}  // namespace
}  // namespace graph_msf
