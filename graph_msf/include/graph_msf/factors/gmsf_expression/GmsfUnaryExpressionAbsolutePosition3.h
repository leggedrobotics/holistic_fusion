/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GMSF_UNARY_EXPRESSION_POSITION3_H
#define GMSF_UNARY_EXPRESSION_POSITION3_H

// GTSAM
#include <gtsam/base/types.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/expressions.h>

// Workspace
#include "graph_msf/factors/gmsf_expression/GmsfUnaryExpressionAbsolut.h"
#include "graph_msf/measurements/UnaryMeasurementXDAbsolute.h"

namespace graph_msf {

class GmsfUnaryExpressionAbsolutePosition3 final : public GmsfUnaryExpressionAbsolut<gtsam::Point3, 'd'> {
 public:
  // Constructor
  GmsfUnaryExpressionAbsolutePosition3(const std::shared_ptr<UnaryMeasurementXDAbsolute<Eigen::Vector3d, 3>>& positionUnaryMeasurementPtr,
                                       const std::string& worldFrameName, const std::string& imuFrameName,
                                       const Eigen::Isometry3d& T_I_sensorFrame)
      : GmsfUnaryExpressionAbsolut(positionUnaryMeasurementPtr, worldFrameName, imuFrameName, T_I_sensorFrame),
        positionUnaryMeasurementPtr_(positionUnaryMeasurementPtr),
        exp_fixedFrame_t_fixedFrame_sensorFrame_(gtsam::Point3::Identity()),
        exp_R_fixedFrame_I_(gtsam::Rot3::Identity()) {}

  // Destructor
  ~GmsfUnaryExpressionAbsolutePosition3() = default;

  // Noise as GTSAM Datatype
  [[nodiscard]] const gtsam::Vector getNoiseDensity() const override {
    return positionUnaryMeasurementPtr_->unaryMeasurementNoiseDensity();
  }

  // Return Measurement as GTSAM Datatype
  [[nodiscard]] const gtsam::Point3 getGtsamMeasurementValue() const override {
    return gtsam::Point3(positionUnaryMeasurementPtr_->unaryMeasurement().matrix());
  }

 protected:
  // i) Generate Expression for Basic IMU State in World Frame at Key -------------------------------------
  void generateImuStateInWorldFrameAtKey(const gtsam::Key& closestGeneralKey) final {
    // Translation (core part)
    exp_fixedFrame_t_fixedFrame_sensorFrame_ =
        gtsam::translation(gtsam::Expression<gtsam::Pose3>(gtsam::symbol_shorthand::X(closestGeneralKey)));  // W_t_W_I at this point

    // Rotation (needed for the transformation to the sensor frame)
    exp_R_fixedFrame_I_ =
        gtsam::rotation(gtsam::Expression<gtsam::Pose3>(gtsam::symbol_shorthand::X(closestGeneralKey)));  // R_W_I at this point
  }

  // Interface with three cases (non-exclusive):
  // ii) Holistically Optimize over Fixed Frames -----------------------------------------------------------
  gtsam::Pose3 computeT_fixedFrame_W_initial(const gtsam::NavState& W_currentPropagatedState) final {
    gtsam::Pose3 T_fixedFrame_sensorFrame_meas_noOrientation =
        gtsam::Pose3(gtsam::Rot3::Identity(), positionUnaryMeasurementPtr_->unaryMeasurement());
    gtsam::Pose3 T_W_sensorFrame_est = W_currentPropagatedState.pose() * gtsam::Pose3(T_I_sensorFrameInit_.matrix());
    return T_fixedFrame_sensorFrame_meas_noOrientation * T_W_sensorFrame_est.inverse();
  }

  const Eigen::Vector3d getMeasurementPosition() final {
    // Get Measurement
    return positionUnaryMeasurementPtr_->unaryMeasurement();
  }

  void setMeasurementPosition(const Eigen::Vector3d& position) final {
    // Set Measurement
    positionUnaryMeasurementPtr_->unaryMeasurement() = position;
  }

  void transformStateToReferenceFrameMeasurement(const gtsam::Pose3_& exp_T_fixedFrame_W) override {
    // Transform state to fixed frame
    exp_fixedFrame_t_fixedFrame_sensorFrame_ =
        gtsam::transformFrom(exp_T_fixedFrame_W, exp_fixedFrame_t_fixedFrame_sensorFrame_);  // T_fixedFrame_I at this point

    // Transform rotation from world to fixed frame
    exp_R_fixedFrame_I_ = gtsam::rotation(exp_T_fixedFrame_W) * exp_R_fixedFrame_I_;  // R_fixedFrame_I at this point
  }

  // iii) Transform Measurement to Core Imu Frame -----------------------------------------------------------
  void transformImuStateToSensorFrameState() final {
    // Get relative translation
    Eigen::Vector3d I_t_I_sensorFrame = T_I_sensorFrameInit_.translation();

    // Rotation of IMU frame
    exp_fixedFrame_t_fixedFrame_sensorFrame_ =
        exp_fixedFrame_t_fixedFrame_sensorFrame_ +
        gtsam::rotate(exp_R_fixedFrame_I_, I_t_I_sensorFrame);  // fixedFrame_t_fixedFrame_sensorFrameInit
  }

  // iv) Extrinsic Calibration ---------------------------------------------------------------------
  void applyExtrinsicCalibrationCorrection(const gtsam::Point3_& exp_sensorFrame_t_sensorFrame_correctSensorFrame) final {
    // Apply Correction
    gtsam::Rot3_ exp_R_fixedFrame_sensorFrame = exp_R_fixedFrame_I_ * gtsam::Rot3_(gtsam::Rot3(T_I_sensorFrameInit_.rotation()));
    exp_fixedFrame_t_fixedFrame_sensorFrame_ =
        exp_fixedFrame_t_fixedFrame_sensorFrame_ +
        gtsam::rotate(exp_R_fixedFrame_sensorFrame,
                      exp_sensorFrame_t_sensorFrame_correctSensorFrame);  // fixedFrame_t_fixedFrame_sensorFrameCorrected
  }

  gtsam::Pose3 convertToPose3(const gtsam::Point3& measurement) final { return gtsam::Pose3(gtsam::Rot3::Identity(), measurement); }

  // Return Expression
  [[nodiscard]] const gtsam::Expression<gtsam::Point3> getGtsamExpression() const override {
    return exp_fixedFrame_t_fixedFrame_sensorFrame_;
  }

 private:
  // Full Measurement Type
  std::shared_ptr<UnaryMeasurementXDAbsolute<Eigen::Vector3d, 3>> positionUnaryMeasurementPtr_;

  // Expression
  gtsam::Expression<gtsam::Point3> exp_fixedFrame_t_fixedFrame_sensorFrame_;  // Translation
  gtsam::Expression<gtsam::Rot3> exp_R_fixedFrame_I_;                         // Rotation
};
}  // namespace graph_msf

#endif  // GMSF_UNARY_EXPRESSION_POSITION3_H