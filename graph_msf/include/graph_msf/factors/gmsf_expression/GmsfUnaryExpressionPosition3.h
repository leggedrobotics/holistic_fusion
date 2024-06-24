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
#include "graph_msf/factors/gmsf_expression/GmsfUnaryExpression.h"
#include "graph_msf/measurements/UnaryMeasurementXD.h"

namespace graph_msf {

class GmsfUnaryExpressionPosition3 final : public GmsfUnaryExpression<gtsam::Point3> {
 public:
  // Constructor
  GmsfUnaryExpressionPosition3(const std::shared_ptr<UnaryMeasurementXD<Eigen::Vector3d, 3>>& poseUnaryMeasurementPtr,
                               const std::string& worldFrameName, const Eigen::Isometry3d& T_I_sensorFrame)
      : GmsfUnaryExpression(poseUnaryMeasurementPtr, worldFrameName, T_I_sensorFrame),
        positionUnaryMeasurementPtr_(poseUnaryMeasurementPtr),
        exp_fixedFrame_t_fixedFrame_sensorFrame_(gtsam::Point3::Identity()),
        exp_R_fixedFrame_I_(gtsam::Rot3::Identity()) {}

  // Destructor
  ~GmsfUnaryExpressionPosition3() override = default;
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
  void transformStateFromWorldToFixedFrame(TransformsExpressionKeys& transformsExpressionKeys,
                                           const gtsam::NavState& W_currentPropagatedState,
                                           const bool centerMeasurementsAtRobotPositionBeforeAlignment) override {
    // Compute the initial guess for T_fixedFrame_W
    // TODO: Add good initial guess (containing position)
    //    Eigen::Vector3d fixedFrame_t_fixedFrame_sensorFrame_meas = positionUnaryMeasurementPtr_->unaryMeasurement();
    //    Eigen::Vector3d W_t_W_I_est = W_currentPropagatedState.pose().translation();
    //    Eigen::Vector3d fixedFrame_t_fixedFrame_W_initial = fixedFrame_t_fixedFrame_sensorFrame_meas + W_t_W_I_est;

    // Search for the new graph key of T_fixedFrame_W
    bool newGraphKeyAdded = false;
    Eigen::Vector3d _;  // Placeholder
    gtsam::Pose3 T_fixedFrame_W_initial(gtsam::Pose3::Identity());
    gtsam::Key newGraphKey = transformsExpressionKeys.getTransformationExpression<gtsam::symbol_shorthand::T>(
        newGraphKeyAdded, _, positionUnaryMeasurementPtr_->fixedFrameName(), worldFrameName_, positionUnaryMeasurementPtr_->timeK(),
        T_fixedFrame_W_initial, false);

    // Define expression for T_fixedFrame_W
    gtsam::Pose3_ exp_T_fixedFrame_W(newGraphKey);  // T_fixedFrame_W

    // Transform state to fixed frame
    exp_fixedFrame_t_fixedFrame_sensorFrame_ =
        gtsam::transformFrom(exp_T_fixedFrame_W, exp_fixedFrame_t_fixedFrame_sensorFrame_);  // T_fixedFrame_I at this point

    // Transform rotation from world to fixed frame
    exp_R_fixedFrame_I_ = gtsam::rotation(exp_T_fixedFrame_W) * exp_R_fixedFrame_I_;  // R_fixedFrame_I at this point

    if (newGraphKeyAdded) {
      std::cout << "GmsfUnaryExpressionPose3: Initial Guess for T_" << positionUnaryMeasurementPtr_->fixedFrameName()
                << "_W, RPY (deg): " << T_fixedFrame_W_initial.rotation().rpy().transpose() * (180.0 / M_PI)
                << ", t (x, y, z): " << T_fixedFrame_W_initial.translation().transpose() << std::endl;
      // Insert Values
      newStateValues_.insert(newGraphKey, T_fixedFrame_W_initial);
      // Insert Prior
      newPriorPoseFactors_.emplace_back(newGraphKey, T_fixedFrame_W_initial,
                                        gtsam::noiseModel::Diagonal::Sigmas(baseUnaryMeasurementPtr_->initialSe3AlignmentNoise()));
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
    // Terminal output for now
    REGULAR_COUT << "GmsfUnaryExpressionPosition3: Running Extrinsic Calibration Correction." << std::endl;

    // Get delta transformation from sensorFrame to correctSensorFrame
    bool newGraphKeyAddedFlag = false;
    gtsam::Point3 sensorFrame_t_sensorFrame_correctSensorFrame_initial = gtsam::Point3::Zero();
    gtsam::Key newGraphKey = transformsExpressionKeys.getTransformationExpression<gtsam::symbol_shorthand::D>(
        newGraphKeyAddedFlag, positionUnaryMeasurementPtr_->sensorFrameName(), positionUnaryMeasurementPtr_->sensorFrameCorrectedName(),
        positionUnaryMeasurementPtr_->timeK(), gtsam::Pose3(gtsam::Rot3::Identity(), sensorFrame_t_sensorFrame_correctSensorFrame_initial));
    gtsam::Point3_ exp_sensorFrame_t_sensorFrame_correctSensorFrame = gtsam::Point3_(newGraphKey);

    // Apply Correction
    gtsam::Rot3_ exp_R_fixedFrame_sensorFrame = exp_R_fixedFrame_I_ * gtsam::Rot3_(gtsam::Rot3(T_I_sensorFrameInit_.rotation()));
    exp_fixedFrame_t_fixedFrame_sensorFrame_ =
        exp_fixedFrame_t_fixedFrame_sensorFrame_ +
        gtsam::rotate(exp_R_fixedFrame_sensorFrame,
                      exp_sensorFrame_t_sensorFrame_correctSensorFrame);  // fixedFrame_t_fixedFrame_sensorFrameCorrected

    // Initial Values
    if (newGraphKeyAddedFlag) {
      newStateValues_.insert(newGraphKey, sensorFrame_t_sensorFrame_correctSensorFrame_initial);
    }
  }

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
  std::shared_ptr<UnaryMeasurementXD<Eigen::Vector3d, 3>> positionUnaryMeasurementPtr_;

  // Expression
  gtsam::Expression<gtsam::Point3> exp_fixedFrame_t_fixedFrame_sensorFrame_;  // Translation
  gtsam::Expression<gtsam::Rot3> exp_R_fixedFrame_I_;                         // Rotation
};
}  // namespace graph_msf

#endif  // GMSF_UNARY_EXPRESSION_POSITION3_H