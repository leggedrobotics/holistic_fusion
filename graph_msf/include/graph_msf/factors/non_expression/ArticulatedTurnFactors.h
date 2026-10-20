#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <cmath>
#include <stdexcept>

namespace graph_msf {

// X is the cabin IMU pose in world. R_IC maps CABIN coordinates into IMU
// coordinates. The CABIN-to-BASE transform is Rz(-q); the slew axis is CABIN Z.
// Pose3's local coordinates put its three rotational components first.
inline gtsam::Rot3 articulatedBaseRotation(const gtsam::Pose3& X, double q,
                                          const gtsam::Rot3& R_IC,
                                          gtsam::OptionalJacobian<3, 6> H_X = {},
                                          gtsam::OptionalJacobian<3, 1> H_q = {}) {
  const gtsam::Rot3 R_IB = R_IC * gtsam::Rot3::Rz(-q);
  if (H_X) {
    H_X->setZero();
    H_X->template leftCols<3>() = R_IB.matrix().transpose();
  }
  if (H_q) {
    *H_q = -gtsam::Vector3::UnitZ();
  }
  return X.rotation() * R_IB;
}

namespace articulated_turn_detail {

inline double wrapAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

// Spherical logarithm in the measured Unit3 tangent basis, with derivative
// with respect to the predicted unit vector. Unlike a projected chord residual,
// this has no spurious zero at the antipodal direction. Its unavoidable
// antipodal singularity is rejected explicitly.
inline gtsam::Vector2 gravityLogResidual(const gtsam::Unit3& measured,
                                         const gtsam::Vector3& predicted,
                                         gtsam::OptionalJacobian<2, 3> H = {}) {
  const gtsam::Matrix23 basis_transpose = measured.basis().transpose();
  const gtsam::Vector3 reference = measured.unitVector();
  const gtsam::Vector2 tangent = basis_transpose * predicted;
  const double sine = tangent.norm();
  const double cosine = reference.dot(predicted);
  if (sine < 1e-10) {
    if (cosine <= 0.0) {
      throw std::domain_error("Articulated turn gravity directions are antipodal");
    }
    // atan(sine/cosine)/sine has the limit 1/cosine. The omitted terms
    // are O(sine^2), below floating-point precision at this threshold.
    if (H) {
      *H = basis_transpose / cosine -
           tangent * reference.transpose() / (cosine * cosine);
    }
    return tangent / cosine;
  }
  const double angle = std::atan2(sine, cosine);
  const double scale = angle / sine;
  if (H) {
    const double squared_norm = sine * sine + cosine * cosine;
    *H = scale * basis_transpose +
         ((cosine / squared_norm - scale) / (sine * sine)) *
             tangent * (tangent.transpose() * basis_transpose) -
         tangent * reference.transpose() / squared_norm;
  }
  return scale * tangent;
}

}  // namespace articulated_turn_detail

// A parked constraint relates cabin attitude and the turn state to ONE shared
// uncertain station heading. This factor supplies no independent heading
// measurement or repeated heading prior. It permits BASE roll/pitch changes.
class ParkedBaseHeadingFactor
    : public gtsam::NoiseModelFactor3<gtsam::Pose3, double, double> {
 public:
  using Base = gtsam::NoiseModelFactor3<gtsam::Pose3, double, double>;
  using Base::evaluateError;

  ParkedBaseHeadingFactor(gtsam::Key pose_key, gtsam::Key turn_key,
                          gtsam::Key heading_key, const gtsam::Rot3& R_IC,
                          const gtsam::SharedNoiseModel& model)
      : Base(model, pose_key, turn_key, heading_key), R_IC_(R_IC) {}

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(new ParkedBaseHeadingFactor(*this));
  }

  gtsam::Vector evaluateError(const gtsam::Pose3& pose, const double& turn,
                              const double& heading,
                              gtsam::OptionalMatrixType H_pose = OptionalNone,
                              gtsam::OptionalMatrixType H_turn = OptionalNone,
                              gtsam::OptionalMatrixType H_heading = OptionalNone) const override {
    gtsam::Matrix36 H_rotation_pose;
    gtsam::Matrix31 H_rotation_turn;
    const gtsam::Rot3 base_rotation =
        articulatedBaseRotation(pose, turn, R_IC_, H_rotation_pose, H_rotation_turn);
    if (base_rotation.matrix().col(0).head<2>().squaredNorm() < 1e-12) {
      throw std::domain_error("Articulated BASE heading is undefined for a vertical forward axis");
    }
    gtsam::Matrix13 H_heading_rotation;
    const double predicted_heading = base_rotation.yaw(H_heading_rotation);
    if (H_pose) {
      *H_pose = H_heading_rotation * H_rotation_pose;
    }
    if (H_turn) {
      *H_turn = H_heading_rotation * H_rotation_turn;
    }
    if (H_heading) {
      *H_heading = -gtsam::Matrix11::Identity();
    }
    // As with every wrapped heading residual, the derivative is undefined at
    // a residual of exactly +/-pi. Normal initialization keeps us away from it.
    return gtsam::Vector1(
        articulated_turn_detail::wrapAngle(predicted_heading - heading));
  }

 private:
  gtsam::Rot3 R_IC_;
};

// BASE gravity is a measurement expressed in BASE, never pre-rotated using a
// separately filtered encoder. Both the cabin pose and q remain graph variables.
class BaseGravityFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, double> {
 public:
  using Base = gtsam::NoiseModelFactor2<gtsam::Pose3, double>;
  using Base::evaluateError;

  BaseGravityFactor(gtsam::Key pose_key, gtsam::Key turn_key,
                     const gtsam::Unit3& measured_up_base, const gtsam::Rot3& R_IC,
                     const gtsam::SharedNoiseModel& model)
      : Base(model, pose_key, turn_key), measured_up_base_(measured_up_base), R_IC_(R_IC) {}

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(new BaseGravityFactor(*this));
  }

  gtsam::Vector evaluateError(const gtsam::Pose3& pose, const double& turn,
                              gtsam::OptionalMatrixType H_pose = OptionalNone,
                              gtsam::OptionalMatrixType H_turn = OptionalNone) const override {
    gtsam::Matrix36 H_rotation_pose;
    gtsam::Matrix31 H_rotation_turn;
    const gtsam::Rot3 base_rotation =
        articulatedBaseRotation(pose, turn, R_IC_, H_rotation_pose, H_rotation_turn);
    gtsam::Matrix33 H_up_rotation;
    const gtsam::Vector3 predicted_up =
        base_rotation.unrotate(gtsam::Vector3::UnitZ(), H_up_rotation);
    gtsam::Matrix23 H_residual_up;
    const gtsam::Vector2 residual = articulated_turn_detail::gravityLogResidual(
        measured_up_base_, predicted_up, H_residual_up);
    if (H_pose) {
      *H_pose = H_residual_up * H_up_rotation * H_rotation_pose;
    }
    if (H_turn) {
      *H_turn = H_residual_up * H_up_rotation * H_rotation_turn;
    }
    return residual;
  }

 private:
  gtsam::Unit3 measured_up_base_;
  gtsam::Rot3 R_IC_;
};

// The measured delta rotates BASE(j) coordinates into BASE(i) coordinates.
// It must come from independent BASE angular sensing, not a differential
// cabin-gyro measurement: the cabin IMU already enters CombinedImuFactor.
// This factor assumes a bias-corrected increment. Its noise model describes
// integration noise; a persistent shared gyro bias requires a bias state or
// correlated measurement model, not repeated independent noise inflation.
class BaseRotationBetweenFactor
    : public gtsam::NoiseModelFactor4<gtsam::Pose3, double, gtsam::Pose3, double> {
 public:
  using Base = gtsam::NoiseModelFactor4<gtsam::Pose3, double, gtsam::Pose3, double>;
  using Base::evaluateError;

  BaseRotationBetweenFactor(gtsam::Key pose_i_key, gtsam::Key turn_i_key,
                             gtsam::Key pose_j_key, gtsam::Key turn_j_key,
                             const gtsam::Rot3& measured_delta_base,
                             const gtsam::Rot3& R_IC,
                             const gtsam::SharedNoiseModel& model)
      : Base(model, pose_i_key, turn_i_key, pose_j_key, turn_j_key),
        measured_delta_base_(measured_delta_base),
        R_IC_(R_IC) {}

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(new BaseRotationBetweenFactor(*this));
  }

  gtsam::Vector evaluateError(const gtsam::Pose3& pose_i, const double& turn_i,
                              const gtsam::Pose3& pose_j, const double& turn_j,
                              gtsam::OptionalMatrixType H_pose_i = OptionalNone,
                              gtsam::OptionalMatrixType H_turn_i = OptionalNone,
                              gtsam::OptionalMatrixType H_pose_j = OptionalNone,
                              gtsam::OptionalMatrixType H_turn_j = OptionalNone) const override {
    gtsam::Matrix36 H_rotation_pose_i, H_rotation_pose_j;
    gtsam::Matrix31 H_rotation_turn_i, H_rotation_turn_j;
    const gtsam::Rot3 base_i = articulatedBaseRotation(
        pose_i, turn_i, R_IC_, H_rotation_pose_i, H_rotation_turn_i);
    const gtsam::Rot3 base_j = articulatedBaseRotation(
        pose_j, turn_j, R_IC_, H_rotation_pose_j, H_rotation_turn_j);
    gtsam::Matrix33 H_delta_i, H_delta_j;
    const gtsam::Rot3 delta = base_i.between(base_j, H_delta_i, H_delta_j);
    // Left multiplication by a constant preserves the right-local tangent.
    const gtsam::Rot3 residual_rotation = measured_delta_base_.inverse() * delta;
    gtsam::Matrix33 H_log;
    const gtsam::Vector3 residual = gtsam::Rot3::Logmap(residual_rotation, H_log);
    if (H_pose_i) {
      *H_pose_i = H_log * H_delta_i * H_rotation_pose_i;
    }
    if (H_turn_i) {
      *H_turn_i = H_log * H_delta_i * H_rotation_turn_i;
    }
    if (H_pose_j) {
      *H_pose_j = H_log * H_delta_j * H_rotation_pose_j;
    }
    if (H_turn_j) {
      *H_turn_j = H_log * H_delta_j * H_rotation_turn_j;
    }
    return residual;
  }

 private:
  gtsam::Rot3 measured_delta_base_;
  gtsam::Rot3 R_IC_;
};

}  // namespace graph_msf
