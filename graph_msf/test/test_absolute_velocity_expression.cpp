#include <functional>
#include <memory>
#include <set>
#include <vector>

#include <gtest/gtest.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/Values.h>

#include "graph_msf/core/DynamicDictionaryContainer.h"
#include "graph_msf/factors/gmsf_expression/GmsfUnaryExpressionAbsoluteVelocity3.h"

namespace graph_msf {
namespace {

std::shared_ptr<UnaryMeasurementXDAbsolute<Eigen::Vector3d, 3>> makeMeasurement(
    const std::string& fixedFrame = "map", const std::string& worldFrame = "map",
    const Eigen::Vector3d& velocity = Eigen::Vector3d::Zero(), const Eigen::Vector3d& noise = Eigen::Vector3d::Constant(0.05)) {
  const Eigen::Matrix<double, 6, 1> alignmentNoise = Eigen::Matrix<double, 6, 1>::Ones();
  return std::make_shared<UnaryMeasurementXDAbsolute<Eigen::Vector3d, 3>>(
      "GnssVelocity", 10, "gnss", "gnss_corrected", RobustNorm::Huber(1.345), 10.0, 10.0, velocity, noise, fixedFrame, worldFrame,
      boost::optional<Eigen::Matrix<double, 6, 1>>(alignmentNoise));
}

struct SplitExpressionEvaluation {
  gtsam::Vector3 value;
  std::set<gtsam::Key> horizontalKeys;
  std::set<gtsam::Key> verticalKeys;
  std::vector<gtsam::Matrix> horizontalJacobians;
  std::vector<gtsam::Matrix> verticalJacobians;
};

SplitExpressionEvaluation evaluateExpressions(const Eigen::Isometry3d& T_I_G, const Eigen::Vector3d& rawOmega,
                                              const gtsam::Pose3& T_W_I, const gtsam::Vector3& velocity_W_I,
                                              const gtsam::imuBias::ConstantBias& bias) {
  constexpr gtsam::Key k = 7;
  const auto measurement = makeMeasurement();
  GmsfUnaryExpressionAbsoluteVelocityXY horizontalExpression(measurement, "imu", T_I_G, rawOmega);
  GmsfUnaryExpressionAbsoluteVelocityZ verticalExpression(measurement, "imu", T_I_G, rawOmega);
  DynamicDictionaryContainer dynamicDictionary;
  const gtsam::NavState propagatedState(T_W_I, velocity_W_I);
  const auto horizontalGtsamExpression =
      horizontalExpression.createAndReturnExpression(k, dynamicDictionary, propagatedState, true, false, false);
  const auto verticalGtsamExpression =
      verticalExpression.createAndReturnExpression(k, dynamicDictionary, propagatedState, true, false, false);

  gtsam::Values values;
  values.insert(gtsam::symbol_shorthand::X(k), T_W_I);
  values.insert(gtsam::symbol_shorthand::V(k), velocity_W_I);
  values.insert(gtsam::symbol_shorthand::B(k), bias);

  SplitExpressionEvaluation result;
  result.horizontalKeys = horizontalGtsamExpression.keys();
  result.verticalKeys = verticalGtsamExpression.keys();
  result.horizontalJacobians.resize(result.horizontalKeys.size());
  result.verticalJacobians.resize(result.verticalKeys.size());
  const gtsam::Vector2 horizontalValue = horizontalGtsamExpression.value(values, &result.horizontalJacobians);
  const gtsam::Vector1 verticalValue = verticalGtsamExpression.value(values, &result.verticalJacobians);
  result.value.head<2>() = horizontalValue;
  result.value.z() = verticalValue.x();
  return result;
}

TEST(AbsoluteVelocityExpression, SplitComponentsPreserveLeverArmOrientationAndGyroBiasModel) {
  Eigen::Isometry3d T_I_G = Eigen::Isometry3d::Identity();
  T_I_G.translation() = Eigen::Vector3d(2.0, 0.0, 0.0);
  const Eigen::Vector3d rawOmega(0.0, 0.0, 0.3);
  const Eigen::Vector3d gyroBias(0.0, 0.0, 0.1);
  const gtsam::Rot3 R_W_I = gtsam::Rot3::RzRyRx(0.0, 0.0, M_PI_2);
  const gtsam::Vector3 velocity_W_I(1.0, -0.5, 0.2);

  const SplitExpressionEvaluation result =
      evaluateExpressions(T_I_G, rawOmega, gtsam::Pose3(R_W_I, gtsam::Point3::Zero()), velocity_W_I,
                          gtsam::imuBias::ConstantBias(Eigen::Vector3d::Zero(), gyroBias));

  const Eigen::Vector3d expected = velocity_W_I + R_W_I.matrix() * ((rawOmega - gyroBias).cross(T_I_G.translation()));
  EXPECT_TRUE(result.value.isApprox(expected, 1e-12));
  const std::set<gtsam::Key> expectedKeys{gtsam::symbol_shorthand::X(7), gtsam::symbol_shorthand::V(7),
                                         gtsam::symbol_shorthand::B(7)};
  EXPECT_EQ(result.horizontalKeys, expectedKeys);
  EXPECT_EQ(result.verticalKeys, expectedKeys);
}

TEST(AbsoluteVelocityExpression, GyroBiasEqualToRawRateRemovesLeverVelocity) {
  Eigen::Isometry3d T_I_G = Eigen::Isometry3d::Identity();
  T_I_G.translation() = Eigen::Vector3d(1.0, -2.0, 0.5);
  const Eigen::Vector3d rawOmega(0.03, -0.02, 0.2);
  const gtsam::Vector3 velocity_W_I(-0.4, 0.8, 0.1);

  const SplitExpressionEvaluation result =
      evaluateExpressions(T_I_G, rawOmega, gtsam::Pose3::Identity(), velocity_W_I,
                          gtsam::imuBias::ConstantBias(Eigen::Vector3d::Zero(), rawOmega));
  EXPECT_TRUE(result.value.isApprox(velocity_W_I, 1e-12));
}

TEST(AbsoluteVelocityExpression, ProjectsMeasurementAndNoiseByComponent) {
  const Eigen::Vector3d velocity(1.2, -0.7, 0.4);
  const Eigen::Vector3d noise(0.03, 0.05, 0.11);
  const auto measurement = makeMeasurement("map", "map", velocity, noise);
  GmsfUnaryExpressionAbsoluteVelocityXY horizontalExpression(measurement, "imu", Eigen::Isometry3d::Identity(),
                                                             Eigen::Vector3d::Zero());
  GmsfUnaryExpressionAbsoluteVelocityZ verticalExpression(measurement, "imu", Eigen::Isometry3d::Identity(),
                                                           Eigen::Vector3d::Zero());

  EXPECT_TRUE(horizontalExpression.getGtsamMeasurementValue().isApprox(velocity.head<2>()));
  EXPECT_TRUE(verticalExpression.getGtsamMeasurementValue().isApprox(velocity.tail<1>()));
  EXPECT_TRUE(horizontalExpression.getNoiseDensity().isApprox(noise.head<2>()));
  EXPECT_TRUE(verticalExpression.getNoiseDensity().isApprox(noise.tail<1>()));
}

TEST(AbsoluteVelocityExpression, SplitProjectionPreservesThreeDimensionalJacobians) {
  Eigen::Isometry3d T_I_G = Eigen::Isometry3d::Identity();
  T_I_G.translation() = Eigen::Vector3d(1.3, -0.8, 0.6);
  const Eigen::Vector3d rawOmega(0.11, -0.07, 0.23);
  const gtsam::Pose3 T_W_I(gtsam::Rot3::RzRyRx(0.2, -0.1, 0.4), gtsam::Point3(2.0, -1.0, 0.3));
  const gtsam::Vector3 velocity_W_I(0.8, -0.2, 0.15);
  const gtsam::imuBias::ConstantBias bias(Eigen::Vector3d(0.01, -0.02, 0.03), Eigen::Vector3d(0.02, -0.01, 0.04));
  const SplitExpressionEvaluation result = evaluateExpressions(T_I_G, rawOmega, T_W_I, velocity_W_I, bias);

  const std::function<gtsam::Vector3(const gtsam::Pose3&, const gtsam::Vector3&, const gtsam::imuBias::ConstantBias&)> prediction =
      [&T_I_G, &rawOmega](const gtsam::Pose3& pose, const gtsam::Vector3& velocity,
                          const gtsam::imuBias::ConstantBias& imuBias) {
        return velocity + pose.rotation().rotate(imuBias.correctGyroscope(rawOmega).cross(T_I_G.translation()));
      };
  const gtsam::Matrix H_pose = gtsam::numericalDerivative31<gtsam::Vector3, gtsam::Pose3, gtsam::Vector3,
                                                            gtsam::imuBias::ConstantBias>(prediction, T_W_I, velocity_W_I, bias);
  const gtsam::Matrix H_velocity = gtsam::numericalDerivative32<gtsam::Vector3, gtsam::Pose3, gtsam::Vector3,
                                                                gtsam::imuBias::ConstantBias>(prediction, T_W_I, velocity_W_I, bias);
  const gtsam::Matrix H_bias = gtsam::numericalDerivative33<gtsam::Vector3, gtsam::Pose3, gtsam::Vector3,
                                                            gtsam::imuBias::ConstantBias>(prediction, T_W_I, velocity_W_I, bias);

  ASSERT_EQ(result.horizontalJacobians.size(), result.verticalJacobians.size());
  ASSERT_EQ(result.horizontalJacobians.size(), result.horizontalKeys.size());
  size_t jacobianIndex = 0;
  for (const gtsam::Key key : result.horizontalKeys) {
    const auto& horizontalJacobian = result.horizontalJacobians.at(jacobianIndex);
    const auto& verticalJacobian = result.verticalJacobians.at(jacobianIndex);
    ASSERT_EQ(horizontalJacobian.cols(), verticalJacobian.cols());
    gtsam::Matrix splitJacobian(3, horizontalJacobian.cols());
    splitJacobian.topRows<2>() = horizontalJacobian;
    splitJacobian.bottomRows<1>() = verticalJacobian;

    const gtsam::Matrix* expected = nullptr;
    if (key == gtsam::symbol_shorthand::X(7)) {
      expected = &H_pose;
    } else if (key == gtsam::symbol_shorthand::V(7)) {
      expected = &H_velocity;
    } else if (key == gtsam::symbol_shorthand::B(7)) {
      expected = &H_bias;
    }
    ASSERT_NE(expected, nullptr);
    EXPECT_TRUE(splitJacobian.isApprox(*expected, 1e-7));
    ++jacobianIndex;
  }
}

TEST(AbsoluteVelocityExpression, BuildsIndependentHorizontalAndVerticalFactorResiduals) {
  constexpr gtsam::Key k = 7;
  const auto measurement =
      makeMeasurement("map", "map", Eigen::Vector3d(0.1, -0.2, 4.0), Eigen::Vector3d(0.05, 0.05, 0.2));
  GmsfUnaryExpressionAbsoluteVelocityXY horizontalExpression(measurement, "imu", Eigen::Isometry3d::Identity(),
                                                             Eigen::Vector3d::Zero());
  GmsfUnaryExpressionAbsoluteVelocityZ verticalExpression(measurement, "imu", Eigen::Isometry3d::Identity(),
                                                           Eigen::Vector3d::Zero());
  DynamicDictionaryContainer dynamicDictionary;
  const gtsam::NavState propagatedState(gtsam::Pose3::Identity(), gtsam::Vector3::Zero());
  const auto horizontalGtsamExpression =
      horizontalExpression.createAndReturnExpression(k, dynamicDictionary, propagatedState, true, false, false);
  const auto verticalGtsamExpression =
      verticalExpression.createAndReturnExpression(k, dynamicDictionary, propagatedState, true, false, false);

  const auto horizontalNoise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(1.345),
      gtsam::noiseModel::Diagonal::Sigmas(horizontalExpression.getNoiseDensity()));
  const auto verticalNoise = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.345),
                                                               gtsam::noiseModel::Diagonal::Sigmas(verticalExpression.getNoiseDensity()));
  const gtsam::ExpressionFactor<gtsam::Vector2> horizontalFactor(
      horizontalNoise, horizontalExpression.getGtsamMeasurementValue(), horizontalGtsamExpression);
  const gtsam::ExpressionFactor<gtsam::Vector1> verticalFactor(verticalNoise, verticalExpression.getGtsamMeasurementValue(),
                                                               verticalGtsamExpression);

  gtsam::Values values;
  values.insert(gtsam::symbol_shorthand::X(k), gtsam::Pose3::Identity());
  const gtsam::Vector3 zeroVelocity = gtsam::Vector3::Zero();
  values.insert(gtsam::symbol_shorthand::V(k), zeroVelocity);
  values.insert(gtsam::symbol_shorthand::B(k), gtsam::imuBias::ConstantBias());
  EXPECT_EQ(horizontalFactor.unwhitenedError(values).size(), 2);
  EXPECT_EQ(verticalFactor.unwhitenedError(values).size(), 1);
}

TEST(AbsoluteVelocityExpression, RejectsNonWorldFixedFrame) {
  EXPECT_THROW(GmsfUnaryExpressionAbsoluteVelocityXY(makeMeasurement("enu", "map"), "imu", Eigen::Isometry3d::Identity(),
                                                      Eigen::Vector3d::Zero()),
               std::invalid_argument);
  EXPECT_THROW(GmsfUnaryExpressionAbsoluteVelocityZ(makeMeasurement("enu", "map"), "imu", Eigen::Isometry3d::Identity(),
                                                     Eigen::Vector3d::Zero()),
               std::invalid_argument);
}

}  // namespace
}  // namespace graph_msf
