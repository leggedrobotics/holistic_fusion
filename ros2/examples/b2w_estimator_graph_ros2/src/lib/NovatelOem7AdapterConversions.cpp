/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#include "b2w_estimator_graph_ros2/NovatelOem7AdapterConversions.h"

#include <algorithm>
#include <cmath>

namespace b2w_se::novatel_oem7_adapter {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

bool isFiniteNonNegative(const double value) {
  return std::isfinite(value) && value >= 0.0;
}

}  // namespace

bool hasValidSolution(const std::uint32_t solutionStatus, const std::uint32_t positionType) {
  using SolutionStatus = novatel_oem7_msgs::msg::SolutionStatus;
  using PositionType = novatel_oem7_msgs::msg::PositionOrVelocityType;
  return solutionStatus == SolutionStatus::SOL_COMPUTED && positionType != PositionType::NONE;
}

bool hasValidHeading2(const novatel_oem7_msgs::msg::HEADING2& msg) {
  return hasValidSolution(msg.sol_status.status, msg.pos_type.type) &&
         std::isfinite(static_cast<double>(msg.heading)) &&
         isFiniteNonNegative(static_cast<double>(msg.heading_stdev));
}

double normalizeAngle(double angleRad) {
  while (angleRad <= -kPi) {
    angleRad += 2.0 * kPi;
  }
  while (angleRad > kPi) {
    angleRad -= 2.0 * kPi;
  }
  return angleRad;
}

double headingDegToYawRad(const double headingDeg, const double yawOffsetDeg) {
  return normalizeAngle((90.0 - headingDeg + yawOffsetDeg) * kDegToRad);
}

double headingStdDevDegToYawVariance(const double headingStdDevDeg) {
  const double yawStdDevRad = headingStdDevDeg * kDegToRad;
  return yawStdDevRad * yawStdDevRad;
}

void fillInitialYawFromHeading2(const novatel_oem7_msgs::msg::HEADING2& heading2, const double yawOffsetDeg,
                                const std::string& frameId,
                                geometry_msgs::msg::PoseWithCovarianceStamped& initialYaw) {
  const double yaw = headingDegToYawRad(static_cast<double>(heading2.heading), yawOffsetDeg);
  const double halfYaw = 0.5 * yaw;

  initialYaw.header = heading2.header;
  initialYaw.header.frame_id = frameId;
  initialYaw.pose.pose.position.x = 0.0;
  initialYaw.pose.pose.position.y = 0.0;
  initialYaw.pose.pose.position.z = 0.0;
  initialYaw.pose.pose.orientation.x = 0.0;
  initialYaw.pose.pose.orientation.y = 0.0;
  initialYaw.pose.pose.orientation.z = std::sin(halfYaw);
  initialYaw.pose.pose.orientation.w = std::cos(halfYaw);

  std::fill(initialYaw.pose.covariance.begin(), initialYaw.pose.covariance.end(), 0.0);
  initialYaw.pose.covariance[35] = headingStdDevDegToYawVariance(static_cast<double>(heading2.heading_stdev));
}

}  // namespace b2w_se::novatel_oem7_adapter
