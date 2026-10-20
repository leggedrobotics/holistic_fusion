#include <cmath>

#include <gtest/gtest.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/AttitudeFactor.h>
#include <gtsam/nonlinear/Values.h>

namespace graph_msf {
namespace {

TEST(Pose3AttitudeFactor, ConstrainsTiltButNotWorldYawOrTranslation) {
  const gtsam::Key key = gtsam::Symbol('x', 0);
  const gtsam::Rot3 R_W_I = gtsam::Rot3::RzRyRx(0.21, -0.13, 0.72);
  const gtsam::Unit3 up_W(gtsam::Vector3::UnitZ());
  const gtsam::Unit3 up_I(R_W_I.unrotate(gtsam::Vector3::UnitZ()));
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(2, 0.01);
  const gtsam::Pose3AttitudeFactor factor(key, up_W, noise, up_I);

  gtsam::Matrix jacobian;
  const gtsam::Vector matchingError = factor.evaluateError(
      gtsam::Pose3(R_W_I, gtsam::Point3(4.0, -2.0, 1.0)), jacobian);
  EXPECT_NEAR(matchingError.norm(), 0.0, 1e-12);
  ASSERT_EQ(jacobian.rows(), 2);
  ASSERT_EQ(jacobian.cols(), 6);
  EXPECT_NEAR(jacobian.rightCols<3>().norm(), 0.0, 1e-12);

  const gtsam::Rot3 yawed = gtsam::Rot3::Yaw(-1.17) * R_W_I;
  const gtsam::Vector yawedError =
      factor.evaluateError(gtsam::Pose3(yawed, gtsam::Point3(-8.0, 7.0, 3.0)));
  EXPECT_NEAR(yawedError.norm(), 0.0, 1e-12);

  const gtsam::Rot3 wrongTilt = gtsam::Rot3::Roll(0.12) * R_W_I;
  EXPECT_GT(factor.evaluateError(gtsam::Pose3(wrongTilt, gtsam::Point3::Zero()))
                .norm(),
            0.05);
}

} // namespace
} // namespace graph_msf
