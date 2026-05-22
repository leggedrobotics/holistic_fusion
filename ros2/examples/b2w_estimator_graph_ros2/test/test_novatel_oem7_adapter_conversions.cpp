#include <cmath>

#include <gtest/gtest.h>

#include "b2w_estimator_graph_ros2/NovatelOem7AdapterConversions.h"

namespace {

constexpr double kTolerance = 1e-6;
constexpr double kPi = 3.14159265358979323846;
constexpr const char* kSensorFrame = "cpt7_imu";

novatel_oem7_msgs::msg::HEADING2 makeValidHeading2() {
  novatel_oem7_msgs::msg::HEADING2 msg;
  msg.header.stamp.sec = 123;
  msg.header.stamp.nanosec = 456;
  msg.sol_status.status = novatel_oem7_msgs::msg::SolutionStatus::SOL_COMPUTED;
  msg.pos_type.type = novatel_oem7_msgs::msg::PositionOrVelocityType::NARROW_INT;
  msg.heading = 0.0F;
  msg.heading_stdev = 2.0F;
  return msg;
}

}  // namespace

TEST(NovatelOem7AdapterConversions, Heading2ValidityRequiresComputedFiniteHeading) {
  auto heading2 = makeValidHeading2();
  EXPECT_TRUE(b2w_se::novatel_oem7_adapter::hasValidHeading2(heading2));

  heading2.sol_status.status = novatel_oem7_msgs::msg::SolutionStatus::INSUFFICIENT_OBS;
  EXPECT_FALSE(b2w_se::novatel_oem7_adapter::hasValidHeading2(heading2));

  heading2 = makeValidHeading2();
  heading2.pos_type.type = novatel_oem7_msgs::msg::PositionOrVelocityType::NONE;
  EXPECT_FALSE(b2w_se::novatel_oem7_adapter::hasValidHeading2(heading2));

  heading2 = makeValidHeading2();
  heading2.heading_stdev = -1.0F;
  EXPECT_FALSE(b2w_se::novatel_oem7_adapter::hasValidHeading2(heading2));
}

TEST(NovatelOem7AdapterConversions, HeadingDegreesConvertToEstimatorYawRadians) {
  EXPECT_NEAR(b2w_se::novatel_oem7_adapter::headingDegToYawRad(0.0, 0.0), 0.5 * kPi, kTolerance);
  EXPECT_NEAR(b2w_se::novatel_oem7_adapter::headingDegToYawRad(90.0, 0.0), 0.0, kTolerance);
  EXPECT_NEAR(b2w_se::novatel_oem7_adapter::headingDegToYawRad(180.0, 0.0), -0.5 * kPi, kTolerance);
  EXPECT_NEAR(b2w_se::novatel_oem7_adapter::headingDegToYawRad(270.0, 0.0), kPi, kTolerance);
}

TEST(NovatelOem7AdapterConversions, HeadingYawOffsetIsAppliedBeforeNormalization) {
  EXPECT_NEAR(b2w_se::novatel_oem7_adapter::headingDegToYawRad(90.0, 15.0), 15.0 * kPi / 180.0, kTolerance);
  EXPECT_NEAR(b2w_se::novatel_oem7_adapter::headingDegToYawRad(0.0, 180.0), -0.5 * kPi, kTolerance);
}

TEST(NovatelOem7AdapterConversions, Heading2ToInitialYawCarriesHeaderYawAndCovariance) {
  const auto heading2 = makeValidHeading2();
  geometry_msgs::msg::PoseWithCovarianceStamped initialYaw;

  b2w_se::novatel_oem7_adapter::fillInitialYawFromHeading2(heading2, 0.0, kSensorFrame, initialYaw);

  EXPECT_EQ(initialYaw.header.stamp.sec, heading2.header.stamp.sec);
  EXPECT_EQ(initialYaw.header.stamp.nanosec, heading2.header.stamp.nanosec);
  EXPECT_EQ(initialYaw.header.frame_id, kSensorFrame);
  EXPECT_DOUBLE_EQ(initialYaw.pose.pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(initialYaw.pose.pose.position.y, 0.0);
  EXPECT_DOUBLE_EQ(initialYaw.pose.pose.position.z, 0.0);
  EXPECT_NEAR(initialYaw.pose.pose.orientation.z, std::sin(0.25 * kPi), kTolerance);
  EXPECT_NEAR(initialYaw.pose.pose.orientation.w, std::cos(0.25 * kPi), kTolerance);
  EXPECT_NEAR(initialYaw.pose.covariance[35], std::pow(2.0 * kPi / 180.0, 2.0), kTolerance);
}
