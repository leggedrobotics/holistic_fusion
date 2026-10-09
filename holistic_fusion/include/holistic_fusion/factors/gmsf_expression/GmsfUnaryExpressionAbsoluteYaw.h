/*
Copyright 2026 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GMSF_UNARY_EXPRESSION_ABSOLUTE_YAW_H
#define GMSF_UNARY_EXPRESSION_ABSOLUTE_YAW_H

// C++
#include <stdexcept>

// GTSAM
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/expressions.h>

// Workspace
#include "holistic_fusion/factors/gmsf_expression/GmsfUnaryExpressionAbsolut.h"
#include "holistic_fusion/measurements/UnaryMeasurementXDAbsolute.h"

namespace holistic_fusion {

// Yaw of a rotation as a planar rotation, so that the factor error wraps around at +-pi.
inline gtsam::Rot2 yawAsRot2(const gtsam::Rot3& R, gtsam::OptionalJacobian<1, 3> H) {
  return gtsam::Rot2::fromAngle(R.yaw(H));
}

// Yaw of S in the world, relative to the yaw of the fixed frame M in the world. It equals the yaw of S in M if M is level.
// For a tilted M, it stays about the z-axis of the world, so it does not pull on the roll and pitch of the state.
inline gtsam::Expression<gtsam::Rot2> yawRelativeToFixedFrameInWorld(const gtsam::Rot3_& exp_R_W_S, const gtsam::Rot3_& exp_R_W_M) {
  const gtsam::Expression<gtsam::Rot2> exp_yaw_W_S(&yawAsRot2, exp_R_W_S);
  const gtsam::Expression<gtsam::Rot2> exp_yaw_W_M(&yawAsRot2, exp_R_W_M);
  // yaw_M_S = yaw_W_M^-1 * yaw_W_S
  const gtsam::Expression<gtsam::Rot2> exp_yaw_M_S = gtsam::between(exp_yaw_W_M, exp_yaw_W_S);
  return exp_yaw_M_S;
}

/**
 * Expression that constrains the yaw of a sensor frame S in a fixed frame M.
 * If M is not the world frame and fixed frames are optimized, the yaw is taken in the world, relative to the yaw of the alignment T_W_M.
 * The tilt of M is ignored, so the measurement assumes a level M.
 * The yaw is evaluated on S, so the constraint holds for any IMU mounting orientation.
 * A yaw measurement has no position, so it reuses the current alignment keyframe of M and never creates one.
 */
class GmsfUnaryExpressionAbsoluteYaw final : public GmsfUnaryExpressionAbsolut<gtsam::Rot2, 'c'> {
 public:
  // Constructor
  GmsfUnaryExpressionAbsoluteYaw(const std::shared_ptr<UnaryMeasurementXDAbsolute<double, 1>>& yawUnaryMeasurementPtr,
                                 const std::string& imuFrameName, const Eigen::Isometry3d& T_I_sensorFrame,
                                 const double createReferenceAlignmentKeyframeEveryNSeconds)
      : GmsfUnaryExpressionAbsolut(yawUnaryMeasurementPtr, imuFrameName, T_I_sensorFrame, createReferenceAlignmentKeyframeEveryNSeconds),
        yawUnaryMeasurementPtr_(yawUnaryMeasurementPtr),
        exp_R_W_S_(gtsam::Rot3::Identity()),
        exp_R_W_fixedFrame_(gtsam::Rot3::Identity()) {}

  // Destructor
  ~GmsfUnaryExpressionAbsoluteYaw() = default;

  // Noise as GTSAM Datatype
  [[nodiscard]] const gtsam::Vector getNoiseDensity() const override { return yawUnaryMeasurementPtr_->unaryMeasurementNoiseDensity(); }

  // Return Measurement as GTSAM Datatype
  [[nodiscard]] const gtsam::Rot2 getGtsamMeasurementValue() const override {
    return gtsam::Rot2::fromAngle(yawUnaryMeasurementPtr_->unaryMeasurement());
  }

 protected:
  // i) Generate Expression for Basic IMU State in World Frame at Key -------------------------------------
  void generateImuStateInWorldFrameAtKey(const gtsam::Key& closestGeneralKey) final {
    exp_R_W_S_ = gtsam::rotation(gtsam::Expression<gtsam::Pose3>(gtsam::symbol_shorthand::X(closestGeneralKey)));  // R_W_I at this point
  }

  // ii) Holistically Optimize over Fixed Frames -----------------------------------------------------------
  bool measuresPosition() const final { return false; }

  // A yaw measurement never creates a keyframe, so the graph never uses this guess
  gtsam::Pose3 computeT_W_fixedFrame_initial(const gtsam::NavState& /*W_currentPropagatedState*/) final { return gtsam::Pose3::Identity(); }

  const Eigen::Vector3d getMeasurementPosition() final { return Eigen::Vector3d::Zero(); }

  void setMeasurementPosition(const Eigen::Vector3d& /*position*/) final {
    throw std::logic_error("GmsfUnaryExpressionAbsoluteYaw: a yaw measurement has no position.");
  }

  // The state stays in the world, as the yaw is taken about the z-axis of the world
  void transformStateToReferenceFrameMeasurement(const gtsam::Pose3_& exp_T_W_fixedFrame) override {
    exp_R_W_fixedFrame_ = gtsam::rotation(exp_T_W_fixedFrame);
  }

  // iii) Transform Measurement to Core Imu Frame -----------------------------------------------------------
  void transformImuStateToSensorFrameState() final { exp_R_W_S_ = exp_R_W_S_ * gtsam::Rot3_(gtsam::Rot3(T_I_sensorFrameInit_.rotation())); }

  // iv) Extrinsic Calibration ---------------------------------------------------------------------
  void transformSensorFrameStateToSensorFrameCorrectedState(DynamicDictionaryContainer& /*gtsamDynamicExpressionKeys*/) final {
    throw std::logic_error("GmsfUnaryExpressionAbsoluteYaw: extrinsic calibration is not supported for yaw measurements.");
  }

  void applyExtrinsicCalibrationCorrection(const gtsam::Expression<gtsam::Rot2>& /*exp_C_sensorFrame_sensorFrameCorrected*/) final {
    throw std::logic_error("GmsfUnaryExpressionAbsoluteYaw: extrinsic calibration is not supported for yaw measurements.");
  }

  gtsam::Pose3 convertToPose3(const gtsam::Rot2& measurement) final {
    return gtsam::Pose3(gtsam::Rot3::Yaw(measurement.theta()), gtsam::Point3::Zero());
  }

  gtsam::Rot2 convertFromPose3(const gtsam::Pose3& pose) final { return gtsam::Rot2::fromAngle(pose.rotation().yaw()); }

  // Return Expression
  [[nodiscard]] const gtsam::Expression<gtsam::Rot2> getGtsamExpression() const override {
    return yawRelativeToFixedFrameInWorld(exp_R_W_S_, exp_R_W_fixedFrame_);
  }

 private:
  // Full Measurement Type
  std::shared_ptr<UnaryMeasurementXDAbsolute<double, 1>> yawUnaryMeasurementPtr_;

  // Expressions
  gtsam::Expression<gtsam::Rot3> exp_R_W_S_;
  gtsam::Expression<gtsam::Rot3> exp_R_W_fixedFrame_;
};

}  // namespace holistic_fusion

#endif  // GMSF_UNARY_EXPRESSION_ABSOLUTE_YAW_H
