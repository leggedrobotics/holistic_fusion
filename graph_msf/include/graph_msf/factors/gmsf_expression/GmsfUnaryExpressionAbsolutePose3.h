/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GMSF_UNARY_EXPRESSION_POSE3_H
#define GMSF_UNARY_EXPRESSION_POSE3_H

// GTSAM
#include <gtsam/base/types.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/expressions.h>

// Workspace
#include "graph_msf/factors/gmsf_expression/GmsfUnaryExpressionAbsolut.h"
#include "graph_msf/measurements/UnaryMeasurementXDAbsolute.h"

namespace graph_msf {

using GMSF_MEASUREMENT_CLASS = UnaryMeasurementXDAbsolute<Eigen::Isometry3d, 6>;

class GmsfUnaryExpressionAbsolutePose3 final : public GmsfUnaryExpressionAbsolut<gtsam::Pose3, 'c'> {
 public:
  // Constructor
  GmsfUnaryExpressionAbsolutePose3(const std::shared_ptr<GMSF_MEASUREMENT_CLASS> poseUnaryMeasurementPtr, const std::string& imuFrameName,
                                   const Eigen::Isometry3d& T_I_sensorFrame, const double createReferenceAlignmentKeyframeEveryNSeconds)
      : GmsfUnaryExpressionAbsolut(poseUnaryMeasurementPtr, imuFrameName, T_I_sensorFrame, createReferenceAlignmentKeyframeEveryNSeconds),
        gmsfPoseUnaryMeasurementPtr_(poseUnaryMeasurementPtr),
        exp_T_fixedFrame_sensorFrame_(gtsam::Pose3::Identity())  // Placeholder --> will be modified later
  {}

  // Destructor
  ~GmsfUnaryExpressionAbsolutePose3() = default;

  // Return Measurement as GTSAM Datatype
  [[nodiscard]] const gtsam::Pose3 getGtsamMeasurementValue() const final {
    return gtsam::Pose3(gmsfPoseUnaryMeasurementPtr_->unaryMeasurement().matrix());
  }

  // Noise as GTSAM Datatype
  [[nodiscard]] const gtsam::Vector getNoiseDensity() const final { return gmsfPoseUnaryMeasurementPtr_->unaryMeasurementNoiseDensity(); }

 protected:
  // i) Generate Expression for Basic IMU State in World Frame at Key -------------------------------------
  void generateImuStateInWorldFrameAtKey(const gtsam::Key& closestGeneralKey) final {
    exp_T_fixedFrame_sensorFrame_ = gtsam::Expression<gtsam::Pose3>(gtsam::symbol_shorthand::X(closestGeneralKey));  // T_W_I at this point
  }

  // Interface with three cases (non-exclusive):
  // ii) holistically optimize over fixed frames --> All methods needed for this -------------------------------------
  gtsam::Pose3 computeT_fixedFrame_W_initial(const gtsam::NavState& W_currentPropagatedState) final {
    const Eigen::Matrix4d& T_W_I_est = W_currentPropagatedState.pose().matrix();
    const Eigen::Isometry3d& T_fixedFrame_sensorFrame_meas = gmsfPoseUnaryMeasurementPtr_->unaryMeasurement();
    return gtsam::Pose3(T_fixedFrame_sensorFrame_meas * T_I_sensorFrameInit_.inverse() * T_W_I_est.inverse());
  }

  const Eigen::Vector3d getMeasurementPosition() final {
    // Get Measurement
    return gmsfPoseUnaryMeasurementPtr_->unaryMeasurement().translation();
  }

  void setMeasurementPosition(const Eigen::Vector3d& position) final {
    // Set Measurement
    gmsfPoseUnaryMeasurementPtr_->unaryMeasurement().translation() = position;
  }

  void transformStateToReferenceFrameMeasurement(const gtsam::Pose3_& exp_T_fixedFrame_W) override {
    // Transform state to fixed frame
    exp_T_fixedFrame_sensorFrame_ = exp_T_fixedFrame_W * exp_T_fixedFrame_sensorFrame_;  // T_fixedFrame_imu at this point
  }

  // iii) transform estimate to sensor frame --------------------------------------------------------------
  void transformImuStateToSensorFrameState() final {
    exp_T_fixedFrame_sensorFrame_ = composeRigidTransformations(
        exp_T_fixedFrame_sensorFrame_, gtsam::Pose3(T_I_sensorFrameInit_.matrix()));  // T_fixedFrame_sensorFrameInit
  }

  // iv) extrinsic calibration correction --------------------------------------------------------------
  void applyExtrinsicCalibrationCorrection(const gtsam::Pose3_& expt_T_sensorFrame_sensorFrameCorrect) final {
    // Apply calibration correction
    exp_T_fixedFrame_sensorFrame_ =
        exp_T_fixedFrame_sensorFrame_ * expt_T_sensorFrame_sensorFrameCorrect;  // T_fixedFrame_sensorFrameCorrected
  }

  gtsam::Pose3 convertToPose3(const gtsam::Pose3& measurement) final { return measurement; }

  gtsam::Pose3 convertFromPose3(const gtsam::Pose3& measurement) final { return measurement; }

  // Return Expression
  [[nodiscard]] const gtsam::Expression<gtsam::Pose3> getGtsamExpression() const final { return exp_T_fixedFrame_sensorFrame_; }

 private:
  // Members
  std::shared_ptr<GMSF_MEASUREMENT_CLASS> gmsfPoseUnaryMeasurementPtr_;

  // Expression
  gtsam::Expression<gtsam::Pose3> exp_T_fixedFrame_sensorFrame_;
};
}  // namespace graph_msf

#endif  // GMSF_UNARY_EXPRESSION_POSE3_H