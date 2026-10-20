#pragma once

#include <graph_msf/factors/non_expression/ArticulatedTurnFactors.h>

#include <cmath>
#include <stdexcept>

namespace graph_msf {

// Independent leg-gyro likelihood for a SHORT graph interval [i,j].
//
// Let n_L(t) be world-up expressed in a rigid leg sensor's coordinates,
// z = integral n_L(t)' * measured_omega_L(t) dt, and
// A = integral n_L(t) dt. The sensor-frame gyro bias b_L remains a shared
// Vector3 graph variable; the factor must not independently absorb its
// uncertainty into every interval's scalar noise.
//
// The endpoint prediction is e_z' Log(R_WBj * R_WBi^-1). It equals the integral
// of world-vertical angular velocity for a constant rotation axis. For varying
// axes it omits noncommutativity terms: in general an O(angle^2) discrepancy,
// not an exact Euler-heading increment. Callers must bound interval length,
// angular motion and gaps, and include discretization/projection uncertainty
// in the scalar noise. This approximation must not bridge long missing spans.
//
// n_L and omega_L must share the SAME sensor frame and clock. Validate gravity
// quality, acceleration, rigid-leg kinematics and sensor freshness upstream.
// A vendor gravity estimate is only a projection direction with uncertainty;
// it is not an independent full BASE attitude. Do not feed a cabin-minus-leg
// rate here: the cabin IMU already enters the cabin pose factors.
//
// R_IC maps CABIN coordinates into IMU coordinates. The implied BASE rotations
// come from the graph's cabin poses and joint angles, without an encoder-derived
// frame transform or an independently added cabin-pose posterior measurement.
class BaseVerticalGyroFactor
    : public gtsam::NoiseModelFactor5<gtsam::Pose3, double, gtsam::Pose3, double,
                                     gtsam::Vector3> {
 public:
  using Base = gtsam::NoiseModelFactor5<gtsam::Pose3, double, gtsam::Pose3, double,
                                       gtsam::Vector3>;
  using Base::evaluateError;

  BaseVerticalGyroFactor(
      gtsam::Key pose_i_key, gtsam::Key turn_i_key,
      gtsam::Key pose_j_key, gtsam::Key turn_j_key, gtsam::Key leg_bias_key,
      double measured_vertical_angle_rad,
      const gtsam::Vector3& integrated_gravity_sensor_seconds,
      const gtsam::Rot3& R_IC, const gtsam::SharedNoiseModel& model)
      : Base(model, pose_i_key, turn_i_key, pose_j_key, turn_j_key, leg_bias_key),
        measured_vertical_angle_rad_(measured_vertical_angle_rad),
        integrated_gravity_sensor_seconds_(integrated_gravity_sensor_seconds),
        R_IC_(R_IC) {
    if (!std::isfinite(measured_vertical_angle_rad_) ||
        !integrated_gravity_sensor_seconds_.allFinite() ||
        !R_IC_.matrix().allFinite()) {
      throw std::invalid_argument("BaseVerticalGyroFactor requires finite measurements and extrinsic");
    }
    if (!model || model->dim() != 1) {
      throw std::invalid_argument("BaseVerticalGyroFactor requires a scalar noise model");
    }
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(new BaseVerticalGyroFactor(*this));
  }

  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose_i, const double& turn_i,
      const gtsam::Pose3& pose_j, const double& turn_j,
      const gtsam::Vector3& leg_bias,
      gtsam::OptionalMatrixType H_pose_i = OptionalNone,
      gtsam::OptionalMatrixType H_turn_i = OptionalNone,
      gtsam::OptionalMatrixType H_pose_j = OptionalNone,
      gtsam::OptionalMatrixType H_turn_j = OptionalNone,
      gtsam::OptionalMatrixType H_bias = OptionalNone) const override {
    gtsam::Matrix36 H_base_pose_i, H_base_pose_j;
    gtsam::Matrix31 H_base_turn_i, H_base_turn_j;
    const gtsam::Rot3 base_i = articulatedBaseRotation(
        pose_i, turn_i, R_IC_, H_base_pose_i, H_base_turn_i);
    const gtsam::Rot3 base_j = articulatedBaseRotation(
        pose_j, turn_j, R_IC_, H_base_pose_j, H_base_turn_j);
    const gtsam::Rot3 delta_world = base_j * base_i.inverse();
    gtsam::Matrix33 H_log;
    const gtsam::Vector3 angle_world = gtsam::Rot3::Logmap(delta_world, H_log);
    // Right-local perturbations of R_i and R_j induce respectively
    // -R_i*d_i and +R_i*d_j in delta_world's right-local tangent.
    const gtsam::Matrix13 H_vertical_base_j = H_log.row(2) * base_i.matrix();
    if (H_pose_i) *H_pose_i = -H_vertical_base_j * H_base_pose_i;
    if (H_turn_i) *H_turn_i = -H_vertical_base_j * H_base_turn_i;
    if (H_pose_j) *H_pose_j = H_vertical_base_j * H_base_pose_j;
    if (H_turn_j) *H_turn_j = H_vertical_base_j * H_base_turn_j;
    if (H_bias) *H_bias = integrated_gravity_sensor_seconds_.transpose();
    return gtsam::Vector1(
        angle_world.z() + integrated_gravity_sensor_seconds_.dot(leg_bias) -
        measured_vertical_angle_rad_);
  }

 private:
  double measured_vertical_angle_rad_;
  gtsam::Vector3 integrated_gravity_sensor_seconds_;
  gtsam::Rot3 R_IC_;
};

}  // namespace graph_msf
