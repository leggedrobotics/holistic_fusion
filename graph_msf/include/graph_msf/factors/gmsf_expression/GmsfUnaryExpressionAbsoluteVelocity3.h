/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GMSF_UNARY_EXPRESSION_ABSOLUTE_VELOCITY3_H
#define GMSF_UNARY_EXPRESSION_ABSOLUTE_VELOCITY3_H

// C++
#include <stdexcept>

// GTSAM
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/slam/expressions.h>

// Workspace
#include "graph_msf/factors/gmsf_expression/GmsfUnaryExpression.h"
#include "graph_msf/measurements/UnaryMeasurementXDAbsolute.h"

namespace graph_msf {

namespace detail {

struct AbsoluteVelocityXYProjection {
  using Output = gtsam::Vector2;

  static Output project(const gtsam::Point3& velocity, gtsam::OptionalJacobian<2, 3> jacobian = {}) {
    if (jacobian) {
      *jacobian << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;
    }
    return velocity.head<2>();
  }

  static gtsam::Vector projectNoise(const Eigen::Vector3d& noise) { return noise.head<2>(); }

  static gtsam::Pose3 toPose3(const Output& velocity) {
    return gtsam::Pose3(gtsam::Rot3::Identity(), gtsam::Point3(velocity.x(), velocity.y(), 0.0));
  }

  static Output fromPose3(const gtsam::Pose3& pose) { return pose.translation().head<2>(); }
};

struct AbsoluteVelocityZProjection {
  using Output = gtsam::Vector1;

  static Output project(const gtsam::Point3& velocity, gtsam::OptionalJacobian<1, 3> jacobian = {}) {
    if (jacobian) {
      *jacobian << 0.0, 0.0, 1.0;
    }
    return Output(velocity.z());
  }

  static gtsam::Vector projectNoise(const Eigen::Vector3d& noise) { return noise.tail<1>(); }

  static gtsam::Pose3 toPose3(const Output& velocity) {
    return gtsam::Pose3(gtsam::Rot3::Identity(), gtsam::Point3(0.0, 0.0, velocity.x()));
  }

  static Output fromPose3(const gtsam::Pose3& pose) { return Output(pose.z()); }
};

}  // namespace detail

/**
 * Absolute velocity of a rigidly mounted sensor, expressed in the graph world frame.
 *
 * The prediction is
 *
 *   W_v_W_S = W_v_W_I + R_W_I * ((I_omega_W_I_raw - I_b_gyro) x I_r_I_S).
 *
 * A velocity measurement alone cannot initialize the SE(3) alignment between an arbitrary fixed frame and the graph world. This
 * expression therefore deliberately supports world-frame measurements only.
 *
 * The component projection is applied only after the complete three-dimensional rigid-body velocity has been built. This lets horizontal
 * and vertical factors use independent robust weights without changing the lever-arm or gyro-bias model.
 */
template <class Projection>
class GmsfUnaryExpressionAbsoluteVelocityComponent final
    : public GmsfUnaryExpression<typename Projection::Output, UnaryExpressionType::Absolute, 'd'> {
  using Output = typename Projection::Output;
  using Base = GmsfUnaryExpression<Output, UnaryExpressionType::Absolute, 'd'>;

 public:
  GmsfUnaryExpressionAbsoluteVelocityComponent(
      const std::shared_ptr<UnaryMeasurementXDAbsolute<Eigen::Vector3d, 3>>& velocityUnaryMeasurementPtr,
      const std::string& imuFrameName, const Eigen::Isometry3d& T_I_sensorFrame, const Eigen::Vector3d& rawAngularVelocity)
      : Base(velocityUnaryMeasurementPtr, imuFrameName, T_I_sensorFrame),
        velocityUnaryMeasurementPtr_(velocityUnaryMeasurementPtr),
        rawAngularVelocity_(rawAngularVelocity),
        exp_W_v_W_sensorFrame_(gtsam::Point3::Identity()),
        exp_R_W_I_(gtsam::Rot3::Identity()),
        exp_correctedAngularVelocity_I_(gtsam::Point3::Identity()) {
    if (velocityUnaryMeasurementPtr_->fixedFrameName() != velocityUnaryMeasurementPtr_->worldFrameName()) {
      throw std::invalid_argument("Absolute velocity expressions support measurements in the graph world frame only.");
    }
    if (!rawAngularVelocity_.allFinite()) {
      throw std::invalid_argument("Absolute velocity expression received a non-finite angular velocity.");
    }
  }

  ~GmsfUnaryExpressionAbsoluteVelocityComponent() = default;

  [[nodiscard]] const gtsam::Vector getNoiseDensity() const override {
    return Projection::projectNoise(velocityUnaryMeasurementPtr_->unaryMeasurementNoiseDensity());
  }

  [[nodiscard]] const Output getGtsamMeasurementValue() const override {
    return Projection::project(velocityUnaryMeasurementPtr_->unaryMeasurement());
  }

 protected:
  void generateImuStateInWorldFrameAtKey(const gtsam::Key& closestGeneralKey) final {
    exp_W_v_W_sensorFrame_ = gtsam::Expression<gtsam::Vector3>(gtsam::symbol_shorthand::V(closestGeneralKey));
    exp_R_W_I_ = gtsam::rotation(gtsam::Expression<gtsam::Pose3>(gtsam::symbol_shorthand::X(closestGeneralKey)));

    const gtsam::Expression<gtsam::imuBias::ConstantBias> exp_imuBias(gtsam::symbol_shorthand::B(closestGeneralKey));
    const gtsam::Point3_ exp_rawAngularVelocity(rawAngularVelocity_);
    exp_correctedAngularVelocity_I_ =
        gtsam::Point3_(exp_imuBias, &gtsam::imuBias::ConstantBias::correctGyroscope, exp_rawAngularVelocity);
  }

  void transformImuStateToSensorFrameState() final {
    const gtsam::Point3_ exp_I_r_I_sensorFrame(this->T_I_sensorFrameInit_.translation());
    exp_W_v_W_sensorFrame_ =
        exp_W_v_W_sensorFrame_ +
        gtsam::rotate(exp_R_W_I_, gtsam::cross(exp_correctedAngularVelocity_I_, exp_I_r_I_sensorFrame));
  }

  void transformImuStateFromWorldToReferenceFrame(DynamicDictionaryContainer&, const gtsam::NavState&, const bool) final {
    // Constructor validation guarantees this is already a graph-world-frame measurement.
  }

  void transformLandmarkInWorldToImuFrame(DynamicDictionaryContainer&, const gtsam::NavState&) final {
    throw std::logic_error("Absolute velocity is not a landmark measurement.");
  }

  void applyExtrinsicCalibrationCorrection(const gtsam::Expression<Output>&) final {
    throw std::logic_error("Absolute velocity extrinsic calibration is not supported.");
  }

  gtsam::Pose3 convertToPose3(const Output& measurement) final { return Projection::toPose3(measurement); }

  Output convertFromPose3(const gtsam::Pose3& pose) final { return Projection::fromPose3(pose); }

  [[nodiscard]] const gtsam::Expression<Output> getGtsamExpression() const override {
    return gtsam::Expression<Output>(&Projection::project, exp_W_v_W_sensorFrame_);
  }

 private:
  std::shared_ptr<UnaryMeasurementXDAbsolute<Eigen::Vector3d, 3>> velocityUnaryMeasurementPtr_;
  Eigen::Vector3d rawAngularVelocity_;
  gtsam::Point3_ exp_W_v_W_sensorFrame_;
  gtsam::Rot3_ exp_R_W_I_;
  gtsam::Point3_ exp_correctedAngularVelocity_I_;
};

using GmsfUnaryExpressionAbsoluteVelocityXY =
    GmsfUnaryExpressionAbsoluteVelocityComponent<detail::AbsoluteVelocityXYProjection>;
using GmsfUnaryExpressionAbsoluteVelocityZ =
    GmsfUnaryExpressionAbsoluteVelocityComponent<detail::AbsoluteVelocityZProjection>;

}  // namespace graph_msf

#endif  // GMSF_UNARY_EXPRESSION_ABSOLUTE_VELOCITY3_H
