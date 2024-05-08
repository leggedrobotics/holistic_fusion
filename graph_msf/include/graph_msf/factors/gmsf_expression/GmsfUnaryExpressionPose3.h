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
#include "graph_msf/factors/gmsf_expression/GmsfUnaryExpression.h"
#include "graph_msf/measurements/UnaryMeasurementXD.h"

namespace graph_msf {

class GmsfUnaryExpressionPose3 final : public GmsfUnaryExpression<gtsam::Pose3> {
 public:
  // Constructor
  GmsfUnaryExpressionPose3(const std::shared_ptr<UnaryMeasurementXD<Eigen::Isometry3d, 6>>& poseUnaryMeasurementPtr,
                           const std::string& worldFrameName, const Eigen::Isometry3d& T_I_sensorFrame)
      : GmsfUnaryExpression(poseUnaryMeasurementPtr, worldFrameName, T_I_sensorFrame),
        poseUnaryMeasurementPtr_(poseUnaryMeasurementPtr),
        exp_T_fixedFrame_sensorFrame_(gtsam::Pose3::Identity()) {}

  // Destructor
  ~GmsfUnaryExpressionPose3() override = default;
  // i) Generate Expression for Basic IMU State in World Frame at Key
  void generateExpressionForBasicImuStateInWorldFrameAtKey(const gtsam::Key& closestGeneralKey) override {
    exp_T_fixedFrame_sensorFrame_ = gtsam::Expression<gtsam::Pose3>(gtsam::symbol_shorthand::X(closestGeneralKey));
  }

  // Interface with three cases (non-exclusive):
  // ii) holistically optimize over fixed frames
  void transformStateFromWorldToFixedFrame(TransformsExpressionKeys& transformsExpressionKeys,
                                           const gtsam::NavState& W_currentPropagatedState) override {
    // Get Measurement & Estimate aliases
    const auto& T_fixedFrame_sensorFrame_meas = poseUnaryMeasurementPtr_->unaryMeasurement();
    const auto& T_W_I_est = W_currentPropagatedState.pose().matrix();

    // Search for the new graph key of T_fixedFrame_W
    bool newGraphKeyAdded = false;
    gtsam::Key newGraphKey = transformsExpressionKeys.getTransformationExpression<gtsam::symbol_shorthand::T>(
        newGraphKeyAdded, poseUnaryMeasurementPtr_->fixedFrameName(), worldFrameName_, poseUnaryMeasurementPtr_->timeK());

    // Transform state to fixed frame
    exp_T_fixedFrame_sensorFrame_ = gtsam::Pose3_(newGraphKey) * exp_T_fixedFrame_sensorFrame_;  // T_fixedFrame_imu at this point
    if (newGraphKeyAdded) {
      // Compute Initial guess
      gtsam::Pose3 T_fixedFrame_W_initial(T_fixedFrame_sensorFrame_meas * T_I_sensorFrame_.inverse() * T_W_I_est.inverse());
      std::cout << "GmsfUnaryExpressionPose3: Initial Guess for T_" << poseUnaryMeasurementPtr_->fixedFrameName()
                << "_W, RPY (deg): " << T_fixedFrame_W_initial.rotation().rpy().transpose() * (180.0 / M_PI)
                << ", t (x, y, z): " << T_fixedFrame_W_initial.translation().transpose() << std::endl;
      // Insert Values
      newStateValues_.insert(newGraphKey, T_fixedFrame_W_initial);
      // Prior maybe not needed, but for safety (to keep well conditioned)
      newPriorPoseFactors_.emplace_back(newGraphKey, T_fixedFrame_W_initial,
                                        gtsam::noiseModel::Diagonal::Sigmas(1.0 * gtsam::Vector::Ones(6)));
    }
  }

  // iii) transform measurement to core imu frame
  void transformStateToSensorFrame() override {
    exp_T_fixedFrame_sensorFrame_ =
        composeRigidTransformations(exp_T_fixedFrame_sensorFrame_, gtsam::Pose3(T_I_sensorFrame_.matrix()));  // T_fixedFrame_sensorFrame
  }

  // iv) extrinsic calibration
  void addExtrinsicCalibrationCorrection(TransformsExpressionKeys& transformsExpressionKeys) override {
    // Initial Guess
    const gtsam::Pose3& T_sensorFrame_sensorFrameCorrected_initial = gtsam::Pose3::Identity();  // alias for readability

    // Search for the new graph key of T_sensorFrame_sensorFrameCorrected
    bool newGraphKeyAdded = false;
    gtsam::Key newGraphKey = transformsExpressionKeys.getTransformationExpression<gtsam::symbol_shorthand::T>(
        newGraphKeyAdded, poseUnaryMeasurementPtr_->sensorFrameName(), poseUnaryMeasurementPtr_->sensorFrameName() + "_corr",
        poseUnaryMeasurementPtr_->timeK());

    // Apply calibration correction
    exp_T_fixedFrame_sensorFrame_ = exp_T_fixedFrame_sensorFrame_ * gtsam::Pose3_(newGraphKey);  // T_fixedFrame_sensorFrameCorrected
    if (newGraphKeyAdded) {
      // Insert Values
      newStateValues_.insert(newGraphKey, T_sensorFrame_sensorFrameCorrected_initial);
      // Insert Prior (maybe not needed, but for safety (to keep well conditioned))
      //      newPriorFactors_.emplace_back(newGraphKey, T_sensorFrame_sensorFrameCorrected_initial,
      //                                    gtsam::noiseModel::Diagonal::Sigmas(1.0e-03 * gtsam::Vector::Ones(6)));
    }
    // std::cout << "GmsfUnaryExpressionPose3: Extrinsic Calibration Correction added." << std::endl;
  }

  // Noise as GTSAM Datatype
  [[nodiscard]] const gtsam::Vector getNoiseDensity() const override { return poseUnaryMeasurementPtr_->unaryMeasurementNoiseDensity(); }

  // Return Measurement as GTSAM Datatype
  [[nodiscard]] const gtsam::Pose3 getMeasurement() const override {
    return gtsam::Pose3(poseUnaryMeasurementPtr_->unaryMeasurement().matrix());
  }

  // Return Expression
  [[nodiscard]] const gtsam::Expression<gtsam::Pose3> getExpression() const override { return exp_T_fixedFrame_sensorFrame_; }

 protected:
  // Full Measurement Type
  std::shared_ptr<UnaryMeasurementXD<Eigen::Isometry3d, 6>> poseUnaryMeasurementPtr_;

  // Expression
  gtsam::Expression<gtsam::Pose3> exp_T_fixedFrame_sensorFrame_;
};
}  // namespace graph_msf

#endif  // GMSF_UNARY_EXPRESSION_POSE3_H