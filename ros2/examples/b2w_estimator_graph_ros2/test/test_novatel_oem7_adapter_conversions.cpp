#include <cmath>

#include <gtest/gtest.h>

#include "b2w_estimator_graph_ros2/NovatelOem7AdapterConversions.h"

namespace {

constexpr double kTolerance = 1e-6;
constexpr double kPi = 3.14159265358979323846;

novatel_oem7_msgs::msg::BESTPOS makeValidBestpos() {
  novatel_oem7_msgs::msg::BESTPOS msg;
  msg.sol_status.status = novatel_oem7_msgs::msg::SolutionStatus::SOL_COMPUTED;
  msg.pos_type.type = novatel_oem7_msgs::msg::PositionOrVelocityType::NARROW_INT;
  msg.lat = 47.1;
  msg.lon = 8.2;
  msg.hgt = 500.0;
  msg.undulation = 42.0;
  msg.lat_stdev = 0.3F;
  msg.lon_stdev = 0.4F;
  msg.hgt_stdev = 0.5F;
  return msg;
}

novatel_oem7_msgs::msg::HEADING2 makeValidHeading2() {
  novatel_oem7_msgs::msg::HEADING2 msg;
  msg.sol_status.status = novatel_oem7_msgs::msg::SolutionStatus::SOL_COMPUTED;
  msg.pos_type.type = novatel_oem7_msgs::msg::PositionOrVelocityType::NARROW_INT;
  msg.heading = 0.0F;
  msg.heading_stdev = 2.0F;
  return msg;
}

}  // namespace

TEST(NovatelOem7AdapterConversions, BestposToNavSatFixUsesLlaAndEllipsoidAltitude) {
  const auto bestpos = makeValidBestpos();
  sensor_msgs::msg::NavSatFix navSatFix;

  b2w_se::novatel_oem7_adapter::fillNavSatFixFromBestpos(bestpos, true, "gnss", navSatFix);

  EXPECT_EQ(navSatFix.header.frame_id, "gnss");
  EXPECT_DOUBLE_EQ(navSatFix.latitude, bestpos.lat);
  EXPECT_DOUBLE_EQ(navSatFix.longitude, bestpos.lon);
  EXPECT_DOUBLE_EQ(navSatFix.altitude, bestpos.hgt + bestpos.undulation);
}

TEST(NovatelOem7AdapterConversions, BestposToNavSatFixMapsCovarianceAsEnu) {
  const auto bestpos = makeValidBestpos();
  sensor_msgs::msg::NavSatFix navSatFix;

  b2w_se::novatel_oem7_adapter::fillNavSatFixFromBestpos(bestpos, true, "gnss", navSatFix);

  EXPECT_NEAR(navSatFix.position_covariance[0], 0.16, kTolerance);
  EXPECT_NEAR(navSatFix.position_covariance[4], 0.09, kTolerance);
  EXPECT_NEAR(navSatFix.position_covariance[8], 0.25, kTolerance);
  EXPECT_EQ(navSatFix.position_covariance_type, sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN);
}

TEST(NovatelOem7AdapterConversions, InvalidBestposIsRejected) {
  auto bestpos = makeValidBestpos();
  EXPECT_TRUE(b2w_se::novatel_oem7_adapter::hasValidBestpos(bestpos));

  bestpos.sol_status.status = novatel_oem7_msgs::msg::SolutionStatus::INSUFFICIENT_OBS;
  EXPECT_FALSE(b2w_se::novatel_oem7_adapter::hasValidBestpos(bestpos));
}

TEST(NovatelOem7AdapterConversions, HeadingDegreesConvertToEstimatorYawRadians) {
  EXPECT_NEAR(b2w_se::novatel_oem7_adapter::headingDegToYawRad(0.0, 0.0), 0.5 * kPi, kTolerance);
  EXPECT_NEAR(b2w_se::novatel_oem7_adapter::headingDegToYawRad(90.0, 0.0), 0.0, kTolerance);
  EXPECT_NEAR(b2w_se::novatel_oem7_adapter::headingDegToYawRad(180.0, 0.0), -0.5 * kPi, kTolerance);
}

TEST(NovatelOem7AdapterConversions, Heading2ToInitialYawCarriesYawCovariance) {
  const auto heading2 = makeValidHeading2();
  geometry_msgs::msg::PoseWithCovarianceStamped initialYaw;

  b2w_se::novatel_oem7_adapter::fillInitialYawFromHeading2(heading2, 0.0, "gnss", initialYaw);

  EXPECT_EQ(initialYaw.header.frame_id, "gnss");
  EXPECT_NEAR(initialYaw.pose.pose.orientation.z, std::sin(0.25 * kPi), kTolerance);
  EXPECT_NEAR(initialYaw.pose.pose.orientation.w, std::cos(0.25 * kPi), kTolerance);
  EXPECT_NEAR(initialYaw.pose.covariance[35], std::pow(2.0 * kPi / 180.0, 2.0), kTolerance);
}
