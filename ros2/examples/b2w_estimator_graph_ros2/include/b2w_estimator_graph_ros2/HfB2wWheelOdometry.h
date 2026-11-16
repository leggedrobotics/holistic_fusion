/*
Copyright 2026 by Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#pragma once

// std
#include <array>
#include <string>
#include <vector>

// Eigen
#include <Eigen/Core>

namespace b2w_se {

// hf_b2w_description: the minimal kinematic description of the B2W legs that the wheel odometry needs, owned by Holistic Fusion.
// Deliberately independent of every URDF / robot_description / description package of the rest of the stack (nothing is
// published, nothing is looked up). Values: joint origins and axes of the B2W URDF, wheel radius from the measured distance.
struct HfB2wDescription {
  struct Leg {
    std::string name;                        // e.g. "LF"
    std::array<std::string, 4> jointNames;   // HAA, HFE, KFE, WHEEL names as published on /joint_states
    std::array<Eigen::Vector3d, 4> origins;  // HAA in base, HFE in hip, KFE in thigh, WHEEL in calf [m]
  };
  std::array<Leg, 4> legs;
  Eigen::Vector3d abductionAxis{1.0, 0.0, 0.0};  // HAA axis (joint frame = parent frame, no rotation in the origins)
  Eigen::Vector3d pitchAxis{0.0, 1.0, 0.0};      // HFE, KFE and wheel axis
  double wheelRadius = 0.1125;                   // [m]
};

struct HfB2wWheelOdometryParams {
  // Slip / lifted wheel: a wheel whose body-velocity estimate deviates more than this from the 4-wheel median is ignored
  double wheelConsistencyGate = 0.25;  // [m/s]
  int minConsistentWheels = 3;
  // Skid-steer turning (wheels slide sideways): no measurement above this yaw rate
  double maxYawRate = 0.7;  // [rad/s]
  // All-wheel slip (e.g. spinning on mud/ice): reject if the estimate deviates this much from the current velocity estimate
  double maxDeviationFromState = 0.6;  // [m/s]
  // Noise (standard deviation at the joint-state rate) of the base velocity in base axes x, y, z
  Eigen::Vector3d sigma{0.3, 0.4, 0.4};                // [m/s]
  double sigmaPerSpeed = 0.05;                         // [-] wheel scale changes with terrain, added in quadrature * |v|
  double sigmaPerLegRate = 0.1;                        // [m/s per rad/s] added in quadrature * max leg joint rate
  // Standstill (zero-velocity update) with hysteresis: all wheel, leg and body rates below the thresholds continuously for
  // standstillMinDuration (entry), left immediately on any motion (exit). Without the dwell, ~17 % of single-sample "still"
  // detections on the Katzensee drive happened while moving > 0.1 m/s (deceleration tails, momentary zero wheel rates).
  double standstillWheelRate = 0.1;      // [rad/s]
  double standstillLegRate = 0.05;       // [rad/s]
  double standstillAngularRate = 0.05;   // [rad/s]
  double standstillMinDuration = 1.0;    // [s]
  double standstillMaxStateSpeed = 0.2;  // [m/s] no zero-velocity update while the current estimate moves (sliding, locked wheels)
  double standstillSigma = 0.05;         // [m/s]
};

struct HfB2wWheelOdometryResult {
  enum class Status { Ok, MissingJoints, TooFewConsistentWheels, Turning, DeviatesFromState };
  Status status = Status::Ok;
  Eigen::Vector3d B_v_W_B = Eigen::Vector3d::Zero();  // velocity of the base origin w.r.t. the world, in base axes [m/s]
  Eigen::Vector3d sigma = Eigen::Vector3d::Zero();    // [m/s]
  int consistentWheels = 0;
  bool standstill = false;
};

class HfB2wWheelOdometry {
 public:
  HfB2wWheelOdometry(HfB2wDescription description, HfB2wWheelOdometryParams params);

  // Resolves the joint names of a /joint_states message once; returns false if any described joint is missing.
  bool setJointNames(const std::vector<std::string>& names);
  bool hasJointMap() const { return jointMapValid_; }

  // q, dq: joint positions / velocities in /joint_states order. B_w_W_B: body angular velocity in base axes (bias corrected).
  // B_v_W_B_state: current velocity estimate of the base origin in base axes (nullptr: no all-wheel slip check).
  // timeK: measurement time [s], for the standstill hysteresis (estimate() is stateful only through it).
  HfB2wWheelOdometryResult estimate(const std::vector<double>& q, const std::vector<double>& dq, const Eigen::Vector3d& B_w_W_B,
                                    const Eigen::Vector3d* B_v_W_B_state, double timeK);

  // Rolling contact without slip on a ground plane with normal = base z, for one leg:
  //   v_B = r (Omega_w x n) - w_B x p_c - sum_k a_k x (p_c - p_k) qdot_k,   Omega_w = w_B + sum_k a_k qdot_k + a_wheel qdot_wheel
  static Eigen::Vector3d wheelBodyVelocity(const HfB2wDescription& description, const HfB2wDescription::Leg& leg,
                                           const std::array<double, 4>& q, const std::array<double, 4>& dq,
                                           const Eigen::Vector3d& B_w_W_B);

  const HfB2wDescription& description() const { return description_; }
  const HfB2wWheelOdometryParams& params() const { return params_; }

 private:
  HfB2wDescription description_;
  HfB2wWheelOdometryParams params_;
  std::array<std::array<int, 4>, 4> jointIndex_{};  // [leg][HAA, HFE, KFE, WHEEL] -> index in /joint_states
  bool jointMapValid_ = false;
  double stillSinceTime_ = -1.0;  // < 0: not still
};

}  // namespace b2w_se
