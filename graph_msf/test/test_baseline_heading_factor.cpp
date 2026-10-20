#include <gtest/gtest.h>
#include <graph_msf/factors/non_expression/BaselineHeadingFactor.h>
#include <gtsam/inference/Symbol.h>

namespace {
graph_msf::BaselineHeadingFactor factor(double heading, const gtsam::Vector3& baseline) {
  return {gtsam::Symbol('x', 0), {heading, baseline},
          gtsam::noiseModel::Isotropic::Sigma(1, 0.01)};
}

TEST(BaselineHeadingFactor, MatchesTiltedBaselineAndManifoldJacobian) {
  const gtsam::Vector3 baseline(0.04, 1.87, -0.12);
  const gtsam::Pose3 pose(gtsam::Rot3::RzRyRx(0.2, -0.3, 2.9), {1.0, 2.0, 3.0});
  const auto direction = pose.rotation().rotate(baseline);
  const auto f = factor(std::atan2(direction.y(), direction.x()), baseline);
  gtsam::Matrix H;
  EXPECT_NEAR(f.evaluateError(pose, H)(0), 0.0, 1e-12);
  gtsam::Matrix numerical(1, 6);
  for (int column = 0; column < 6; ++column) {
    gtsam::Vector6 step = gtsam::Vector6::Zero();
    step(column) = 1e-6;
    numerical.col(column) =
        (f.evaluateError(pose.retract(step)) - f.evaluateError(pose.retract(-step))) / 2e-6;
  }
  EXPECT_LT((H - numerical).norm(), 1e-8);
}

TEST(BaselineHeadingFactor, WrapsAndRejectsUndefinedAzimuth) {
  const double pi = std::acos(-1.0);
  EXPECT_NEAR(factor(-pi + 0.01, gtsam::Vector3::UnitX())
                  .evaluateError(gtsam::Pose3(gtsam::Rot3::Rz(pi - 0.01), {}))(0),
              -0.02, 1e-12);
  EXPECT_THROW(factor(0.0, gtsam::Vector3::Zero()), std::invalid_argument);
  EXPECT_THROW(factor(0.0, gtsam::Vector3::UnitZ()).evaluateError(gtsam::Pose3()),
               std::domain_error);
}
}  // namespace
