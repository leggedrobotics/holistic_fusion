#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtsam/geometry/Pose3.h>

#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace graph_msf {

// ROS epoch timestamps stored as double have sub-microsecond roundoff.
// This is comparison tolerance, not an extension of the physical age limit.
inline constexpr double kArticulatedTimestampToleranceSec = 1e-6;

struct ArticulatedTurnConfig {
  bool enable{false};
  Eigen::Matrix3d R_imu_cabin{Eigen::Matrix3d::Identity()};
  double initial_turn_sigma_rad{0.03490658503988659};
  double leg_bias_prior_sigma_rad_s{0.01};
  double leg_bias_random_walk_rad_s_sqrt_s{0.00002};
  double parked_heading_sigma_rad{0.0008726646259971648};
  double minimum_interval_sigma_rad{0.0001};
  double maximum_sample_age_s{0.05};
  double maximum_interval_s{0.1};
  double maximum_output_age_s{0.25};
  double maximum_turn_sigma_rad{0.08726646259971647};
  double missing_rate_sigma_rad_s{1.0};
  double maximum_base_angular_rate_rad_s{1.0};
  double prediction_cabin_rate_sigma_rad_s{0.02};
  double prediction_translation_rate_sigma_m_s{0.1};
  int minimum_leg_count{2};
  size_t maximum_queued_samples{20000};
};

struct ArticulatedTurnLegSample {
  uint32_t sensor_id{0};
  double stamp_sec{std::numeric_limits<double>::quiet_NaN()};
  // RAW independent leg gyro projected on gravity; never cabin-minus-leg.
  double vertical_rate_rad_s{std::numeric_limits<double>::quiet_NaN()};
  // World up expressed in this raw gyro's sensor frame; yaw calibration is
  // unnecessary. Root adapter must reject moving/non-rigid legs.
  Eigen::Vector3d gravity_sensor{Eigen::Vector3d::Zero()};
  double sigma_vertical_rate_rad_s{0.01};
};

struct ArticulatedTurnSample {
  double stamp_sec{std::numeric_limits<double>::quiet_NaN()};
  // A one-time uncertain reference. Subsequent values are ignored after the
  // first q state is created; no station transition resets joint zero.
  double initial_turn_reference_rad{std::numeric_limits<double>::quiet_NaN()};
  // Must mean positively qualified stationary BASE, including drive/steer/
  // leg evidence. False also represents unknown or stale parking evidence.
  bool parked{false};
  std::vector<ArticulatedTurnLegSample> legs;
};

struct ArticulatedTurnState {
  bool enabled{false};
  bool initialized{false};
  bool valid{false};
  bool fault_latched{false};
  bool parked{false};
  std::string reason{"disabled"};
  uint64_t graph_key{0};
  double stamp_sec{std::numeric_limits<double>::quiet_NaN()};
  double position_rad{std::numeric_limits<double>::quiet_NaN()};
  gtsam::Pose3 pose_world_imu;
  // Explicit [Pose3 local rotation(3), local translation(3), q(1)] order.
  Eigen::Matrix<double, 7, 7> joint_covariance{
      Eigen::Matrix<double, 7, 7>::Constant(
          std::numeric_limits<double>::quiet_NaN())};
  std::map<uint32_t, Eigen::Vector3d> leg_bias_sensor_rad_s;
  std::map<uint32_t, Eigen::Matrix3d> leg_bias_covariance_sensor_rad2_s2;
  int fused_leg_count{0};
};

} // namespace graph_msf
