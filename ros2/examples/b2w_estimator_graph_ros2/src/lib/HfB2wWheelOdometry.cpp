/*
Copyright 2026 by Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#include "b2w_estimator_graph_ros2/HfB2wWheelOdometry.h"

// std
#include <algorithm>
#include <cmath>
#include <utility>

// Eigen
#include <Eigen/Geometry>

namespace b2w_se {

HfB2wWheelOdometry::HfB2wWheelOdometry(HfB2wDescription description, HfB2wWheelOdometryParams params)
    : description_(std::move(description)), params_(std::move(params)) {
  description_.abductionAxis.normalize();
  description_.pitchAxis.normalize();
}

bool HfB2wWheelOdometry::setJointNames(const std::vector<std::string>& names) {
  jointMapValid_ = false;
  for (std::size_t l = 0; l < description_.legs.size(); ++l) {
    for (std::size_t j = 0; j < 4; ++j) {
      const auto it = std::find(names.begin(), names.end(), description_.legs[l].jointNames[j]);
      if (it == names.end()) {
        return false;
      }
      jointIndex_[l][j] = static_cast<int>(std::distance(names.begin(), it));
    }
  }
  jointMapValid_ = true;
  return true;
}

Eigen::Vector3d HfB2wWheelOdometry::wheelBodyVelocity(const HfB2wDescription& description, const HfB2wDescription::Leg& leg,
                                                      const std::array<double, 4>& q, const std::array<double, 4>& dq,
                                                      const Eigen::Vector3d& B_w_W_B) {
  const Eigen::Vector3d& ax = description.abductionAxis;
  const Eigen::Vector3d& ap = description.pitchAxis;
  // Forward kinematics of the wheel centre (all quantities in base axes)
  const Eigen::Matrix3d R1 = Eigen::AngleAxisd(q[0], ax).toRotationMatrix();
  const Eigen::Matrix3d R12 = R1 * Eigen::AngleAxisd(q[1], ap).toRotationMatrix();
  const Eigen::Matrix3d R123 = R12 * Eigen::AngleAxisd(q[2], ap).toRotationMatrix();
  const Eigen::Vector3d p1 = leg.origins[0];
  const Eigen::Vector3d p2 = p1 + R1 * leg.origins[1];
  const Eigen::Vector3d p3 = p2 + R12 * leg.origins[2];
  const Eigen::Vector3d pc = p3 + R123 * leg.origins[3];
  // Joint axes in base axes
  const Eigen::Vector3d a1 = ax, a2 = R1 * ap, a3 = R12 * ap, aw = R123 * ap;
  // Wheel-centre velocity relative to the base from the leg joints
  const Eigen::Vector3d Jqd = a1.cross(pc - p1) * dq[0] + a2.cross(pc - p2) * dq[1] + a3.cross(pc - p3) * dq[2];
  // Absolute angular velocity of the wheel
  const Eigen::Vector3d omegaWheel = B_w_W_B + a1 * dq[0] + a2 * dq[1] + a3 * dq[2] + aw * dq[3];
  // Contact point (pc - r n) at rest: v_B + w_B x pc + Jqd + omegaWheel x (-r n) = 0
  const Eigen::Vector3d n = Eigen::Vector3d::UnitZ();
  return description.wheelRadius * omegaWheel.cross(n) - B_w_W_B.cross(pc) - Jqd;
}

HfB2wWheelOdometryResult HfB2wWheelOdometry::estimate(const std::vector<double>& q, const std::vector<double>& dq,
                                                      const Eigen::Vector3d& B_w_W_B, const Eigen::Vector3d* B_v_W_B_state,
                                                      const double timeK) {
  using Status = HfB2wWheelOdometryResult::Status;
  HfB2wWheelOdometryResult result;
  if (!jointMapValid_) {
    result.status = Status::MissingJoints;
    return result;
  }

  // Per-wheel estimates of the base velocity, wheel and leg rates
  std::array<Eigen::Vector3d, 4> v;
  double maxWheelRate = 0.0, maxLegRate = 0.0;
  for (std::size_t l = 0; l < 4; ++l) {
    std::array<double, 4> ql{}, dql{};
    for (std::size_t j = 0; j < 4; ++j) {
      const int i = jointIndex_[l][j];
      if (i >= static_cast<int>(q.size()) || i >= static_cast<int>(dq.size()) || !std::isfinite(q[i]) || !std::isfinite(dq[i])) {
        result.status = Status::MissingJoints;
        return result;
      }
      ql[j] = q[i];
      dql[j] = dq[i];
    }
    maxLegRate = std::max({maxLegRate, std::abs(dql[0]), std::abs(dql[1]), std::abs(dql[2])});
    maxWheelRate = std::max(maxWheelRate, std::abs(dql[3]));
    v[l] = wheelBodyVelocity(description_, description_.legs[l], ql, dql, B_w_W_B);
  }

  // Standstill with hysteresis: enter after standstillMinDuration of continuous stillness, leave on the first moving sample.
  // Until then (and when the current estimate still moves) the regular rolling measurement below applies; it reads ~0 at rest.
  const bool stillNow = maxWheelRate < params_.standstillWheelRate && maxLegRate < params_.standstillLegRate &&
                        B_w_W_B.norm() < params_.standstillAngularRate;
  if (!stillNow || stillSinceTime_ > timeK) {
    stillSinceTime_ = stillNow ? timeK : -1.0;
  } else if (stillSinceTime_ < 0.0) {
    stillSinceTime_ = timeK;
  }
  if (stillNow && timeK - stillSinceTime_ >= params_.standstillMinDuration &&
      (B_v_W_B_state == nullptr || B_v_W_B_state->norm() < params_.standstillMaxStateSpeed)) {
    result.standstill = true;
    result.consistentWheels = 4;
    result.B_v_W_B.setZero();
    result.sigma.setConstant(params_.standstillSigma);
    return result;
  }

  // Skid-steer turning: the wheels slide sideways, the rolling model does not hold
  if (std::abs(B_w_W_B.z()) > params_.maxYawRate) {
    result.status = Status::Turning;
    return result;
  }

  // Slip / lifted wheel: vote against the per-axis median of the four estimates
  Eigen::Vector3d median;
  for (int d = 0; d < 3; ++d) {
    std::array<double, 4> c{v[0](d), v[1](d), v[2](d), v[3](d)};
    std::sort(c.begin(), c.end());
    median(d) = 0.5 * (c[1] + c[2]);
  }
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  int consistent = 0;
  for (const auto& vl : v) {
    if ((vl - median).head<2>().norm() < params_.wheelConsistencyGate) {
      sum += vl;
      ++consistent;
    }
  }
  result.consistentWheels = consistent;
  if (consistent < params_.minConsistentWheels) {
    result.status = Status::TooFewConsistentWheels;
    return result;
  }
  result.B_v_W_B = sum / consistent;

  // All-wheel slip: compare with the current velocity estimate
  if (B_v_W_B_state != nullptr && (result.B_v_W_B - *B_v_W_B_state).norm() > params_.maxDeviationFromState) {
    result.status = Status::DeviatesFromState;
    return result;
  }

  // Noise: base + terrain-dependent scale + leg motion
  const double speed = result.B_v_W_B.norm();
  for (int d = 0; d < 3; ++d) {
    result.sigma(d) = std::sqrt(params_.sigma(d) * params_.sigma(d) + std::pow(params_.sigmaPerSpeed * speed, 2) +
                                std::pow(params_.sigmaPerLegRate * maxLegRate, 2));
  }
  return result;
}

}  // namespace b2w_se
