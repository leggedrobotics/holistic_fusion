/*
Copyright 2026 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GMSF_UNARY_EXPRESSION_ABSOLUTE_HEADING_H
#define GMSF_UNARY_EXPRESSION_ABSOLUTE_HEADING_H

// C++
#include <stdexcept>

// GTSAM
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/expressions.h>

// Workspace
#include "holistic_fusion/factors/gmsf_expression/GmsfUnaryExpressionAbsolut.h"
#include "holistic_fusion/measurements/UnaryMeasurementXDAbsolute.h"

namespace holistic_fusion {

// Rotation angle of R_W_Wmeas about the z-axis of W, as a planar rotation, so that the factor error wraps around at +-pi.
// It reads the rotation vector, so it is defined for every attitude.
inline gtsam::Rot2 headingAsRot2(const gtsam::Rot3& R_W_Wmeas, gtsam::OptionalJacobian<1, 3> H) {
  gtsam::Matrix3 H_log;
  const gtsam::Vector3 rotationVector = gtsam::Rot3::Logmap(R_W_Wmeas, H ? &H_log : nullptr);
  if (H) {
    *H = H_log.row(2);
  }
  return gtsam::Rot2::fromAngle(rotationVector.z());
}

// Heading of the estimated orientation R_W_S relative to the measured orientation R_M_Smeas, about the z-axis of the world W.
// The measurement is brought to W through R_W_M, so a tilted fixed frame M does not tilt the constrained axis.
inline gtsam::Expression<gtsam::Rot2> headingInWorld(const gtsam::Rot3_& exp_R_W_S, const gtsam::Rot3_& exp_R_W_M,
                                                     const gtsam::Rot3& R_M_Smeas) {
  return gtsam::Expression<gtsam::Rot2>(&headingAsRot2, exp_R_W_S * gtsam::Rot3_(R_M_Smeas.inverse()) * inverseRot3(exp_R_W_M));
}

/**
 * Expression that constrains the heading of a sensor frame S: the rotation about the z-axis of the world between the estimated and the
 * measured orientation of S. Rotations about the horizontal axes of the world leave it unchanged to first order, so it does not compete
 * with gravity over roll and pitch. For equal roll and pitch, it equals the Euler yaw difference.
 * If the fixed frame M of the measurement is not the world frame and fixed frames are optimized, the measurement goes through the
 * alignment R_W_M. A heading measurement has no position, so it reuses the current alignment keyframe of M and never creates one.
 */
class GmsfUnaryExpressionAbsoluteHeading final : public GmsfUnaryExpressionAbsolut<gtsam::Rot2, 'c'> {
 public:
  // Constructor
  GmsfUnaryExpressionAbsoluteHeading(const std::shared_ptr<UnaryMeasurementXDAbsolute<Eigen::Matrix3d, 1>>& headingUnaryMeasurementPtr,
                                     const std::string& imuFrameName, const Eigen::Isometry3d& T_I_sensorFrame,
                                     const double createReferenceAlignmentKeyframeEveryNSeconds)
      : GmsfUnaryExpressionAbsolut(headingUnaryMeasurementPtr, imuFrameName, T_I_sensorFrame,
                                   createReferenceAlignmentKeyframeEveryNSeconds),
        headingUnaryMeasurementPtr_(headingUnaryMeasurementPtr),
        exp_R_W_S_(gtsam::Rot3::Identity()),
        exp_R_W_fixedFrame_(gtsam::Rot3::Identity()) {}

  // Destructor
  ~GmsfUnaryExpressionAbsoluteHeading() = default;

  // Noise as GTSAM Datatype
  [[nodiscard]] const gtsam::Vector getNoiseDensity() const override { return headingUnaryMeasurementPtr_->unaryMeasurementNoiseDensity(); }

  // The expression is the heading offset to the measured orientation, which the factor drives to zero
  [[nodiscard]] const gtsam::Rot2 getGtsamMeasurementValue() const override { return gtsam::Rot2::Identity(); }

 protected:
  // i) Generate Expression for Basic IMU State in World Frame at Key -------------------------------------
  void generateImuStateInWorldFrameAtKey(const gtsam::Key& closestGeneralKey) final {
    exp_R_W_S_ = gtsam::rotation(gtsam::Expression<gtsam::Pose3>(gtsam::symbol_shorthand::X(closestGeneralKey)));  // R_W_I at this point
  }

  // ii) Holistically Optimize over Fixed Frames -----------------------------------------------------------
  bool measuresPosition() const final { return false; }

  // A heading measurement never creates a keyframe, so the graph never uses this guess
  gtsam::Pose3 computeT_W_fixedFrame_initial(const gtsam::NavState& /*W_currentPropagatedState*/) final { return gtsam::Pose3::Identity(); }

  const Eigen::Vector3d getMeasurementPosition() final { return Eigen::Vector3d::Zero(); }

  void setMeasurementPosition(const Eigen::Vector3d& /*position*/) final {
    throw std::logic_error("GmsfUnaryExpressionAbsoluteHeading: a heading measurement has no position.");
  }

  // The state stays in the world, as the heading is taken about the z-axis of the world
  void transformStateToReferenceFrameMeasurement(const gtsam::Pose3_& exp_T_W_fixedFrame) override {
    exp_R_W_fixedFrame_ = gtsam::rotation(exp_T_W_fixedFrame);
  }

  // iii) Transform Measurement to Core Imu Frame -----------------------------------------------------------
  void transformImuStateToSensorFrameState() final { exp_R_W_S_ = exp_R_W_S_ * gtsam::Rot3_(gtsam::Rot3(T_I_sensorFrameInit_.rotation())); }

  // iv) Extrinsic Calibration ---------------------------------------------------------------------
  void transformSensorFrameStateToSensorFrameCorrectedState(DynamicDictionaryContainer& /*gtsamDynamicExpressionKeys*/) final {
    throw std::logic_error("GmsfUnaryExpressionAbsoluteHeading: extrinsic calibration is not supported for heading measurements.");
  }

  void applyExtrinsicCalibrationCorrection(const gtsam::Expression<gtsam::Rot2>& /*exp_C_sensorFrame_sensorFrameCorrected*/) final {
    throw std::logic_error("GmsfUnaryExpressionAbsoluteHeading: extrinsic calibration is not supported for heading measurements.");
  }

  // Only extrinsic calibration converts, which heading measurements do not support
  gtsam::Pose3 convertToPose3(const gtsam::Rot2& /*heading*/) final {
    throw std::logic_error("GmsfUnaryExpressionAbsoluteHeading: extrinsic calibration is not supported for heading measurements.");
  }

  gtsam::Rot2 convertFromPose3(const gtsam::Pose3& /*pose*/) final {
    throw std::logic_error("GmsfUnaryExpressionAbsoluteHeading: extrinsic calibration is not supported for heading measurements.");
  }

  // Return Expression
  [[nodiscard]] const gtsam::Expression<gtsam::Rot2> getGtsamExpression() const override {
    return headingInWorld(exp_R_W_S_, exp_R_W_fixedFrame_, gtsam::Rot3(headingUnaryMeasurementPtr_->unaryMeasurement()));
  }

 private:
  // Full Measurement Type
  std::shared_ptr<UnaryMeasurementXDAbsolute<Eigen::Matrix3d, 1>> headingUnaryMeasurementPtr_;

  // Expressions
  gtsam::Expression<gtsam::Rot3> exp_R_W_S_;
  gtsam::Expression<gtsam::Rot3> exp_R_W_fixedFrame_;
};

}  // namespace holistic_fusion

#endif  // GMSF_UNARY_EXPRESSION_ABSOLUTE_HEADING_H
