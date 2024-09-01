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

class GmsfUnaryExpressionAbsolutePosition3 final : public GmsfUnaryExpressionAbsolut<gtsam::Point3> {
 public:
  // Constructor
  GmsfUnaryExpressionAbsolutePosition3(const std::shared_ptr<UnaryMeasurementXDAbsolute<Eigen::Vector3d, 3>>& positionUnaryMeasurementPtr,
                                       const std::string& worldFrameName, const Eigen::Isometry3d& T_I_sensorFrame)
      : GmsfUnaryExpressionAbsolut(positionUnaryMeasurementPtr, worldFrameName, T_I_sensorFrame),
        positionUnaryMeasurementPtr_(positionUnaryMeasurementPtr),
        exp_fixedFrame_t_fixedFrame_sensorFrame_(gtsam::Point3::Identity()),
        exp_R_fixedFrame_I_(gtsam::Rot3::Identity()) {}

  // Destructor
  ~GmsfUnaryExpressionAbsolutePosition3() override = default;

  // i) Generate Expression for Basic IMU State in World Frame at Key
  void generateExpressionForBasicImuStateInWorldFrameAtKey(const gtsam::Key& closestGeneralKey) override {
    // Translation (core part)
    exp_fixedFrame_t_fixedFrame_sensorFrame_ =
        gtsam::translation(gtsam::Expression<gtsam::Pose3>(gtsam::symbol_shorthand::X(closestGeneralKey)));  // W_t_W_I at this point

    // Rotation (needed for the transformation to the sensor frame)
    exp_R_fixedFrame_I_ =
        gtsam::rotation(gtsam::Expression<gtsam::Pose3>(gtsam::symbol_shorthand::X(closestGeneralKey)));  // R_W_I at this point
  }

  // Interface with three cases (non-exclusive):
  // ii) Holistically Optimize over Fixed Frames
  inline gtsam::Pose3 computeT_fixedFrame_W_initial(const gtsam::NavState& W_currentPropagatedState) {
    gtsam::Pose3 T_fixedFrame_sensorFrame_meas_noOrientation =
        gtsam::Pose3(gtsam::Rot3::Identity(), positionUnaryMeasurementPtr_->unaryMeasurement());
    gtsam::Pose3 T_W_sensorFrame_est = W_currentPropagatedState.pose() * gtsam::Pose3(T_I_sensorFrameInit_.matrix());
    return T_fixedFrame_sensorFrame_meas_noOrientation * T_W_sensorFrame_est.inverse();
  }

  void transformStateFromWorldToFixedFrame(TransformsExpressionKeys& transformsExpressionKeys,
                                           const gtsam::NavState& W_currentPropagatedState,
                                           const bool centerMeasurementsAtRobotPositionBeforeAlignment) override {
    // Compute the initial guess for T_fixedFrame_W --> for orientation not possible for single measurement
    gtsam::Pose3 T_fixedFrame_W_initial = computeT_fixedFrame_W_initial(W_currentPropagatedState);

    // If it should be centered --> create keyframe for measurement
    Eigen::Vector3d measurementOriginPosition = Eigen::Vector3d::Zero();
    if (centerMeasurementsAtRobotPositionBeforeAlignment) {
      measurementOriginPosition = positionUnaryMeasurementPtr_->unaryMeasurement();
    }

    // Search for the new graph key of T_fixedFrame_W
    bool newGraphKeyAddedFlag = false;
    VariableType variableType = VariableType::RefFrame(measurementOriginPosition);
    FactorGraphStateKey newGraphKey = transformsExpressionKeys.getTransformationKey<'r'>(
        newGraphKeyAddedFlag, positionUnaryMeasurementPtr_->fixedFrameName(), worldFrameName_, positionUnaryMeasurementPtr_->timeK(),
        T_fixedFrame_W_initial, variableType);

    // Shift the measurement to the robot position and recompute initial guess if we create keyframes
    if (centerMeasurementsAtRobotPositionBeforeAlignment) {
      // Shift the measurement to the robot position
      positionUnaryMeasurementPtr_->unaryMeasurement() =
          positionUnaryMeasurementPtr_->unaryMeasurement() - newGraphKey.getMeasurementKeyframePosition();
      // Recompute initial guess
      T_fixedFrame_W_initial = computeT_fixedFrame_W_initial(W_currentPropagatedState);
      // Update the initial guess
      transformsExpressionKeys.lv_T_frame1_frame2(positionUnaryMeasurementPtr_->fixedFrameName(), worldFrameName_)
          .setApproximateTransformationBeforeOptimization(T_fixedFrame_W_initial);
    }

    // Define expression for T_fixedFrame_W
    gtsam::Pose3_ exp_T_fixedFrame_W(newGraphKey.key());  // T_fixedFrame_W

    // Transform state to fixed frame
    exp_fixedFrame_t_fixedFrame_sensorFrame_ =
        gtsam::transformFrom(exp_T_fixedFrame_W, exp_fixedFrame_t_fixedFrame_sensorFrame_);  // T_fixedFrame_I at this point

    // Transform rotation from world to fixed frame
    exp_R_fixedFrame_I_ = gtsam::rotation(exp_T_fixedFrame_W) * exp_R_fixedFrame_I_;  // R_fixedFrame_I at this point

    // Initial values (and priors)
    if (newGraphKeyAddedFlag) {
      REGULAR_COUT << " Initial Guess for T_" << positionUnaryMeasurementPtr_->fixedFrameName()
                   << "_W, RPY (deg): " << T_fixedFrame_W_initial.rotation().rpy().transpose() * (180.0 / M_PI)
                   << ", t (x, y, z): " << T_fixedFrame_W_initial.translation().transpose() << std::endl;
      // Insert Values
      newStateValues_.insert(newGraphKey.key(), T_fixedFrame_W_initial);
      // Insert Prior, might not be necessary (if long enough horizon)
      newPriorPoseFactors_.emplace_back(newGraphKey.key(), T_fixedFrame_W_initial,
                                        gtsam::noiseModel::Diagonal::Sigmas(positionUnaryMeasurementPtr_->initialSe3AlignmentNoise()));
    }
  }

  // iii) Transform Measurement to Core Imu Frame
  void transformStateToSensorFrame() override {
    // Get relative translation
    Eigen::Vector3d I_t_I_sensorFrame = T_I_sensorFrameInit_.translation();

    // Rotation of IMU frame
    exp_fixedFrame_t_fixedFrame_sensorFrame_ =
        exp_fixedFrame_t_fixedFrame_sensorFrame_ +
        gtsam::rotate(exp_R_fixedFrame_I_, I_t_I_sensorFrame);  // fixedFrame_t_fixedFrame_sensorFrameInit
  }

  // iv) Extrinsic Calibration
  void addExtrinsicCalibrationCorrection(TransformsExpressionKeys& transformsExpressionKeys) override {
    // Initial Guess
    gtsam::Point3 sensorFrame_t_sensorFrame_correctSensorFrame_initial = gtsam::Point3::Zero();

    // Get delta displacement from sensorFrame to correctSensorFrame
    bool newGraphKeyAddedFlag = false;
    VariableType variableType = VariableType::Global();
    FactorGraphStateKey newGraphKey = transformsExpressionKeys.getTransformationKey<'d'>(
        newGraphKeyAddedFlag, positionUnaryMeasurementPtr_->sensorFrameName(), positionUnaryMeasurementPtr_->sensorFrameCorrectedName(),
        positionUnaryMeasurementPtr_->timeK(), gtsam::Pose3(gtsam::Rot3::Identity(), sensorFrame_t_sensorFrame_correctSensorFrame_initial),
        variableType);

    // Define expression for sensorFrame_t_sensorFrame_correctSensorFrame
    gtsam::Point3_ exp_sensorFrame_t_sensorFrame_correctSensorFrame = gtsam::Point3_(newGraphKey.key());

    // Apply Correction
    gtsam::Rot3_ exp_R_fixedFrame_sensorFrame = exp_R_fixedFrame_I_ * gtsam::Rot3_(gtsam::Rot3(T_I_sensorFrameInit_.rotation()));
    exp_fixedFrame_t_fixedFrame_sensorFrame_ =
        exp_fixedFrame_t_fixedFrame_sensorFrame_ +
        gtsam::rotate(exp_R_fixedFrame_sensorFrame,
                      exp_sensorFrame_t_sensorFrame_correctSensorFrame);  // fixedFrame_t_fixedFrame_sensorFrameCorrected

    // Initial Values
    if (newGraphKeyAddedFlag) {
      newStateValues_.insert(newGraphKey.key(), sensorFrame_t_sensorFrame_correctSensorFrame_initial);
    }
  }

  // Accessors
  [[nodiscard]] const auto& getUnaryMeasurementPtr() const { return positionUnaryMeasurementPtr_; }

  // Noise as GTSAM Datatype
  [[nodiscard]] const gtsam::Vector getNoiseDensity() const override {
    return positionUnaryMeasurementPtr_->unaryMeasurementNoiseDensity();
  }

  // Return Measurement as GTSAM Datatype
  [[nodiscard]] const gtsam::Point3 getMeasurement() const override {
    return gtsam::Point3(positionUnaryMeasurementPtr_->unaryMeasurement().matrix());
  }

  // Return Expression
  [[nodiscard]] const gtsam::Expression<gtsam::Point3> getExpression() const override { return exp_fixedFrame_t_fixedFrame_sensorFrame_; }

 private:
  // Full Measurement Type
  std::shared_ptr<UnaryMeasurementXDAbsolute<Eigen::Vector3d, 3>> positionUnaryMeasurementPtr_;

  // Expression
  gtsam::Expression<gtsam::Point3> exp_fixedFrame_t_fixedFrame_sensorFrame_;  // Translation
  gtsam::Expression<gtsam::Rot3> exp_R_fixedFrame_I_;                         // Rotation
};
}  // namespace graph_msf

#endif  // GMSF_UNARY_EXPRESSION_POSITION3_H