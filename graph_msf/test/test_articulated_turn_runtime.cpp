#include <graph_msf/core/ArticulatedTurnGraph.h>
#include <graph_msf/core/optimizer/OptimizerIsam2FixedLag.hpp>
#include <graph_msf/interface/constants.h>
#include <gtest/gtest.h>
#include <gtsam/nonlinear/Marginals.h>

namespace {
using namespace graph_msf;
using gtsam::Symbol;
ArticulatedTurnSample sample(double t, bool parked, double base_rate = 0.0) {
  ArticulatedTurnSample result;
  result.stamp_sec = t;
  result.parked = parked;
  result.initial_turn_reference_rad = 0.3;
  for (uint32_t id = 0; id < 2; ++id) {
    ArticulatedTurnLegSample leg;
    leg.sensor_id = id;
    leg.stamp_sec = t;
    leg.vertical_rate_rad_s = base_rate;
    leg.gravity_sensor = gtsam::Vector3::UnitZ();
    leg.sigma_vertical_rate_rad_s = 0.001;
    result.legs.push_back(leg);
  }
  return result;
}
struct RuntimeFixture {
  ArticulatedTurnConfig config;
  std::shared_ptr<GraphConfig> graph_config = std::make_shared<GraphConfig>();
  std::unique_ptr<ArticulatedTurnGraph> runtime;
  std::unique_ptr<OptimizerIsam2FixedLag> optimizer;
  RuntimeFixture() {
    config.enable = true;
    runtime = std::make_unique<ArticulatedTurnGraph>(config);
    graph_config->realTimeSmootherLag_ = 0.3;
    graph_config->relinearizeSkip_ = 1;
    graph_config->rotationReLinTh_ = 1e-7;
    graph_config->gyroBiasReLinTh_ = 1e-7;
    optimizer = std::make_unique<OptimizerIsam2FixedLag>(graph_config);
    optimizer->setRejectRecovery(true);
  }
  gtsam::NonlinearFactorGraph step(uint64_t i, double t, double cabin_yaw,
                                   const ArticulatedTurnSample &input) {
    EXPECT_TRUE(runtime->addSample(input));
    const gtsam::Pose3 pose(gtsam::Rot3::Yaw(cabin_yaw),
                            gtsam::Point3(0, 0, 0));
    gtsam::NonlinearFactorGraph factors;
    gtsam::Values values;
    values.insert(Symbol('x', i), pose);
    std::map<gtsam::Key, double> times{{Symbol('x', i), t}};
    // Synthetic independent pose observations exercise the runtime lifecycle,
    // not physical accuracy; real production X uses CombinedImuFactor/GNSS.
    factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
        Symbol('x', i), pose, gtsam::noiseModel::Isotropic::Sigma(6, 0.001));
    runtime->append(i, t, pose, factors, values, times);
    EXPECT_TRUE(optimizer->update(factors, values, times));
    runtime->optimized(*optimizer, i);
    EXPECT_FALSE(runtime->state().fault_latched) << runtime->state().reason;
    return factors;
  }
};

TEST(ArticulatedTurnRuntime, FixedLagDrivingParkingAndJointCovariance) {
  RuntimeFixture f;
  for (uint64_t i = 0; i <= 120; ++i) {
    const double t = 0.02 * i;
    const bool parked = i < 21 || i >= 100;
    const double base_yaw =
        i < 21 ? 0.0 : (i >= 100 ? 0.158 : (i - 20) * 0.002);
    const double q = 0.3 + 0.1 * std::sin(2 * t);
    f.step(i, t, base_yaw + q, sample(t, parked, parked ? 0.0 : 0.1));
    if (i > 1 && i != 21 && i != 100) {
      const auto state = f.runtime->state();
      EXPECT_TRUE(state.valid) << state.reason;
      EXPECT_NEAR(state.position_rad, q, 0.003);
      EXPECT_GE(state.joint_covariance(6, 6),
                0.99 * std::pow(f.config.initial_turn_sigma_rad, 2));
      EXPECT_EQ(state.leg_bias_covariance_sensor_rad2_s2.size(), 2u);
    }
  }
  const auto state = f.runtime->state();
  EXPECT_TRUE(state.parked);
  const auto actual = f.optimizer->calculateJointCovarianceAtKeys(
      Symbol('x', 120), Symbol('q', 120));
  EXPECT_LT((state.joint_covariance - actual).norm(), 1e-10);
  EXPECT_GT(std::abs(actual(2, 6)),
            1e-8); // shared cabin/turn uncertainty preserved
}

TEST(ArticulatedTurnRuntime, ParkedConstraintReleasedAtExactEndpoint) {
  RuntimeFixture f;
  f.step(0, 0, 0.3, sample(0, true));
  const auto factors = f.step(1, 0.02, 0.3, sample(0.02, false));
  for (const auto &factor : factors)
    EXPECT_EQ(dynamic_cast<const ParkedBaseHeadingFactor *>(factor.get()),
              nullptr);
  EXPECT_FALSE(f.runtime->state().parked);
}

TEST(ArticulatedTurnRuntime, MissingLegsIncreaseUncertaintyAndDoNotReanchor) {
  RuntimeFixture f;
  f.step(0, 0, 0.3, sample(0, true));
  f.step(1, 0.02, 0.3, sample(0.02, true));
  const auto before = f.runtime->state();
  auto missing = sample(0.04, false);
  missing.legs.clear();
  f.step(2, 0.04, 0.3, missing);
  missing.stamp_sec = 0.06;
  f.step(3, 0.06, 0.3, missing);
  EXPECT_FALSE(f.runtime->state().valid);
  EXPECT_GT(f.runtime->state().joint_covariance(6, 6),
            before.joint_covariance(6, 6));
  auto recovered = sample(0.08, true);
  recovered.initial_turn_reference_rad = 2.0;
  f.step(4, 0.08, 0.3, recovered);
  recovered = sample(0.10, true);
  recovered.initial_turn_reference_rad = 2.0;
  f.step(5, 0.10, 0.3, recovered);
  EXPECT_NEAR(f.runtime->state().position_rad, 0.3, 1e-8);
}

TEST(ArticulatedTurnRuntime, FreshHeldSourceRemainsValidUntilExpiryAndFaultLatches) {
  RuntimeFixture f;
  f.step(0, 0, 0.3, sample(0, true));
  f.step(1, 0.02, 0.3, sample(0.02, true));
  auto held = sample(0.04, true);
  for (auto &leg : held.legs)
    leg.stamp_sec = 0.02;
  f.step(2, 0.04, 0.3, held);
  held.stamp_sec = 0.06;
  f.step(3, 0.06, 0.3, held);
  EXPECT_TRUE(f.runtime->state().valid);
  held.stamp_sec = 0.08;
  held.legs.clear(); // the same source is now older than the freshness gate
  f.step(4, 0.08, 0.3, held);
  EXPECT_FALSE(f.runtime->state().valid);
  f.runtime->fault("injected optimizer rejection");
  EXPECT_FALSE(f.runtime->addSample(sample(0.08, true)));
  EXPECT_TRUE(f.runtime->state().fault_latched);
  EXPECT_FALSE(f.runtime->state().valid);
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  std::map<gtsam::Key, double> times;
  f.runtime->append(5, 0.10, gtsam::Pose3(), graph, values, times);
  EXPECT_TRUE(graph.empty());
  EXPECT_TRUE(values.empty());
}

TEST(ArticulatedTurnRuntime, HeldNoiseBoundDominatesCorrelatedUnequalIntervals) {
  const gtsam::Vector3 weights(.002,.015,.023);
  const double maximum_age=.05;
  const double sigma=.01;
  const gtsam::Matrix33 correlated=sigma*sigma*weights*weights.transpose();
  const gtsam::Matrix33 conservative=(sigma*sigma*maximum_age*weights).asDiagonal();
  EXPECT_GE(Eigen::SelfAdjointEigenSolver<gtsam::Matrix33>(conservative-correlated).eigenvalues().minCoeff(),-1e-15);
}

TEST(ArticulatedTurnRuntime, StaleInputsAndClockRewindFailClosed) {
  RuntimeFixture f;
  f.step(0, 0, 0.3, sample(0, true));
  f.step(1, 0.02, 0.3, sample(0.02, true));
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  const gtsam::Pose3 pose(gtsam::Rot3::Yaw(0.3), gtsam::Point3::Zero());
  values.insert(Symbol('x', 2), pose);
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      Symbol('x', 2), pose, gtsam::noiseModel::Isotropic::Sigma(6, .001));
  std::map<gtsam::Key, double> times{{Symbol('x', 2), 0.2}};
  f.runtime->append(2, 0.2, pose, graph, values, times);
  ASSERT_TRUE(f.optimizer->update(graph, values, times));
  f.runtime->optimized(*f.optimizer, 2);
  EXPECT_FALSE(f.runtime->state().valid);
  EXPECT_FALSE(f.runtime->state().parked);
  EXPECT_FALSE(f.runtime->addSample(sample(.01, true)));
  graph.resize(0);
  values.clear();
  times.clear();
  f.runtime->append(3, 0.1, pose, graph, values, times);
  EXPECT_TRUE(f.runtime->state().fault_latched);
  EXPECT_TRUE(graph.empty());
  EXPECT_TRUE(values.empty());
}

TEST(ArticulatedTurnRuntime,
     FutureRawSourceRejectedAndLateEvidenceCannotRewritePosterior) {
  RuntimeFixture f;
  f.step(0, 0, 0.3, sample(0, true));
  f.step(1, 0.02, 0.3, sample(0.02, true));
  const auto before = f.runtime->state();
  auto future = sample(.03, true);
  future.legs.front().stamp_sec = .04;
  EXPECT_FALSE(f.runtime->addSample(future));
  auto delayed = sample(.015, false);
  EXPECT_TRUE(f.runtime->addSample(delayed));
  EXPECT_DOUBLE_EQ(f.runtime->state().stamp_sec, before.stamp_sec);
  EXPECT_DOUBLE_EQ(f.runtime->state().position_rad, before.position_rad);
  EXPECT_EQ(f.runtime->state().parked, before.parked);
}

TEST(ArticulatedTurnRuntime,
     StrictOptimizerDoesNotSilentlyFilterInvalidFactor) {
  RuntimeFixture f;
  f.step(0, 0, 0.3, sample(0, true));
  gtsam::NonlinearFactorGraph bad;
  bad.emplace_shared<gtsam::BetweenFactor<double>>(
      Symbol('q', 0), Symbol('q', 999), 0.0,
      gtsam::noiseModel::Isotropic::Sigma(1, 1));
  EXPECT_FALSE(f.optimizer->update(bad, gtsam::Values(), {}));
  EXPECT_NEAR(f.optimizer->calculateEstimatedScalar(Symbol('q', 0)), 0.3,
              1e-10);
}
} // namespace
