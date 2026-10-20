#include <functional>
#include <limits>

#include <gtest/gtest.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>

#include "graph_msf/interface/ArticulatedBaseState.h"

namespace graph_msf {
namespace {

TEST(ArticulatedBaseState, MatchesFullPoseCompositionAndNumericalJacobian) {
  const gtsam::Pose3 imu(gtsam::Rot3::RzRyRx(.2, -.3, .8),
                        gtsam::Point3(1, 2, 3));
  const gtsam::Pose3 extrinsic(gtsam::Rot3::RzRyRx(-.1, .15, -.4),
                              gtsam::Point3(.5, -.3, .7));
  const gtsam::Point3 lever(.2, .4, -.8);
  constexpr double q = 1.3;
  const auto covariance = Eigen::Matrix<double, 7, 7>::Identity().eval();
  const auto result = deriveArticulatedBaseState(imu, q, extrinsic, lever, covariance);
  const std::function<gtsam::Pose3(const gtsam::Pose3&, const double&)> fn =
      [&](const gtsam::Pose3& x, const double& angle) {
        return x.compose(extrinsic).compose(
            gtsam::Pose3(gtsam::Rot3::Yaw(-angle), lever));
      };
  EXPECT_TRUE(result.poseWorldBase.equals(fn(imu, q), 1e-12));
  const auto dx = gtsam::numericalDerivative21<gtsam::Pose3, gtsam::Pose3, double>(fn, imu, q);
  const auto dq = gtsam::numericalDerivative22<gtsam::Pose3, gtsam::Pose3, double>(fn, imu, q);
  EXPECT_TRUE(result.jointToBaseJacobian.leftCols<6>().isApprox(dx, 1e-8));
  EXPECT_TRUE(result.jointToBaseJacobian.rightCols<1>().isApprox(dq, 1e-8));
}

TEST(ArticulatedBaseState, SharedCabinYawErrorCancelsThroughCrossCovariance) {
  Eigen::Matrix<double, 7, 7> covariance =
      Eigen::Matrix<double, 7, 7>::Identity() * 1e-5;
  // q = cabin_yaw - uncertain_base_heading. The shared cabin error must cancel
  // from the derived BASE yaw, leaving the independent BASE heading component.
  covariance(2, 2) = .04;
  covariance(6, 6) = .05;
  covariance(2, 6) = covariance(6, 2) = .04;
  const auto state = deriveArticulatedBaseState(
      gtsam::Pose3(), 0, gtsam::Pose3(), gtsam::Point3::Zero(), covariance);
  EXPECT_NEAR(state.covarianceWorldRos(5, 5), .01, 1e-12);
  covariance(2, 6) = covariance(6, 2) = 0;
  const auto independent = deriveArticulatedBaseState(
      gtsam::Pose3(), 0, gtsam::Pose3(), gtsam::Point3::Zero(), covariance);
  EXPECT_NEAR(independent.covarianceWorldRos(5, 5), .09, 1e-12);
}

TEST(ArticulatedBaseState, RejectsNonfiniteAndAsymmetricCovariance) {
  Eigen::Matrix<double, 7, 7> covariance =
      Eigen::Matrix<double, 7, 7>::Identity();
  covariance(0, 1) = .1;
  EXPECT_THROW(deriveArticulatedBaseState(
      gtsam::Pose3(), 0, gtsam::Pose3(), gtsam::Point3::Zero(), covariance),
      std::invalid_argument);
  covariance.setIdentity();
  covariance(0, 0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(deriveArticulatedBaseState(
      gtsam::Pose3(), 0, gtsam::Pose3(), gtsam::Point3::Zero(), covariance),
      std::invalid_argument);
}

}  // namespace
}  // namespace graph_msf
