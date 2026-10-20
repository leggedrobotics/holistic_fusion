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

}  // namespace
}  // namespace graph_msf
