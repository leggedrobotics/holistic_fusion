/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#pragma once

#include <cstdint>
#include <string>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <novatel_oem7_msgs/msg/bestpos.hpp>
#include <novatel_oem7_msgs/msg/heading2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace b2w_se::novatel_oem7_adapter {

bool hasValidSolution(std::uint32_t solutionStatus, std::uint32_t positionType);

bool hasValidBestpos(const novatel_oem7_msgs::msg::BESTPOS& msg);

bool hasValidHeading2(const novatel_oem7_msgs::msg::HEADING2& msg);

int navSatStatusForPositionType(std::uint32_t positionType);

double altitudeFromBestpos(const novatel_oem7_msgs::msg::BESTPOS& msg, bool useEllipsoidAltitude);

double normalizeAngle(double angleRad);

double headingDegToYawRad(double headingDeg, double yawOffsetDeg);

double headingStdDevDegToYawVariance(double headingStdDevDeg);

void fillNavSatFixFromBestpos(const novatel_oem7_msgs::msg::BESTPOS& bestpos, bool useEllipsoidAltitude,
                              const std::string& frameId, sensor_msgs::msg::NavSatFix& navSatFix);

void fillInitialYawFromHeading2(const novatel_oem7_msgs::msg::HEADING2& heading2, double yawOffsetDeg,
                                const std::string& frameId,
                                geometry_msgs::msg::PoseWithCovarianceStamped& initialYaw);

}  // namespace b2w_se::novatel_oem7_adapter
