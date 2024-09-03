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

class GmsfUnaryExpressionAbsolutePose3 final : public GmsfUnaryExpressionAbsolut<gtsam::Pose3> {
 public:
  // Constructor
  GmsfUnaryExpressionAbsolutePose3(const std::shared_ptr<GMSF_MEASUREMENT_CLASS> poseUnaryMeasurementPtr, const std::string& worldFrameName,
                                   const std::string& imuFrameName, const Eigen::Isometry3d& T_I_sensorFrame)
      : GmsfUnaryExpressionAbsolut(poseUnaryMeasurementPtr, worldFrameName, imuFrameName, T_I_sensorFrame),
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
  // i) Generate Expression for Basic IMU State in World Frame at Key
  void generateExpressionForBasicImuStateInWorldFrameAtKey(const gtsam::Key& closestGeneralKey) final {
    exp_T_fixedFrame_sensorFrame_ = gtsam::Expression<gtsam::Pose3>(gtsam::symbol_shorthand::X(closestGeneralKey));
  }

  // Interface with three cases (non-exclusive):
  // ii) holistically optimize over fixed frames
  inline gtsam::Pose3 computeT_fixedFrame_W_initial(const Eigen::Matrix4d& T_W_I_est) {
    const Eigen::Isometry3d& T_fixedFrame_sensorFrame_meas = gmsfPoseUnaryMeasurementPtr_->unaryMeasurement();
    return gtsam::Pose3(T_fixedFrame_sensorFrame_meas * T_I_sensorFrameInit_.inverse() * T_W_I_est.inverse());
  }

  void transformStateFromWorldToFixedFrame(TransformsExpressionKeys<gtsam::Pose3>& transformsExpressionKeys,
                                           const gtsam::NavState& W_currentPropagatedState,
                                           const bool centerMeasurementsAtRobotPositionBeforeAlignment) override {
    if (gmsfPoseUnaryMeasurementPtr_->fixedFrameName() == worldFrameName_) {
        // Do nothing as this is a world frame measurement
        return;
    }

    // Get Measurement & Estimate Aliases;
    const Eigen::Matrix4d& T_W_I_est = W_currentPropagatedState.pose().matrix();
    Eigen::Isometry3d& T_fixedFrame_sensorFrame_meas = gmsfPoseUnaryMeasurementPtr_->unaryMeasurement();

    // Initial Guess
    gtsam::Pose3 T_fixedFrame_W_initial = computeT_fixedFrame_W_initial(T_W_I_est);

    // If it should be centered --> create keyframe for measurement
    Eigen::Vector3d measurementOriginPosition = Eigen::Vector3d::Zero();
    if (centerMeasurementsAtRobotPositionBeforeAlignment) {
      measurementOriginPosition = T_fixedFrame_sensorFrame_meas.translation();
    }

    // Search for the new graph key of T_fixedFrame_W
    bool newGraphKeyAddedFlag = false;
    VariableType variableType = VariableType::RefFrame(measurementOriginPosition);
    FactorGraphStateKey newGraphKey = transformsExpressionKeys.getTransformationKey<'r'>(
        newGraphKeyAddedFlag, gmsfPoseUnaryMeasurementPtr_->fixedFrameName(), worldFrameName_, gmsfPoseUnaryMeasurementPtr_->timeK(),
        T_fixedFrame_W_initial, variableType);

    // Shift the measurement to the robot position and recompute initial guess if we create keyframes
    if (centerMeasurementsAtRobotPositionBeforeAlignment) {
      // Shift the measurement to the robot position
      T_fixedFrame_sensorFrame_meas.translation() =
          T_fixedFrame_sensorFrame_meas.translation() - newGraphKey.getMeasurementKeyframePosition();
      // Recompute initial guess
      T_fixedFrame_W_initial = computeT_fixedFrame_W_initial(T_W_I_est);
      // Update the initial guess
      transformsExpressionKeys.lv_T_frame1_frame2(gmsfPoseUnaryMeasurementPtr_->fixedFrameName(), worldFrameName_)
          .setApproximateTransformationBeforeOptimization(T_fixedFrame_W_initial);
    }

    // Transform state to fixed frame
    // Corresponding expression
    exp_T_fixedFrame_sensorFrame_ = gtsam::Pose3_(newGraphKey.key()) * exp_T_fixedFrame_sensorFrame_;  // T_fixedFrame_imu at this point
    if (newGraphKeyAddedFlag) {
      // Compute Initial guess
      std::cout << "GmsfUnaryExpressionPose3: Initial Guess for T_" << gmsfPoseUnaryMeasurementPtr_->fixedFrameName()
                << "_W, RPY (deg): " << T_fixedFrame_W_initial.rotation().rpy().transpose() * (180.0 / M_PI)
                << ", t (x, y, z): " << T_fixedFrame_W_initial.translation().transpose() << std::endl;
      // Insert Values
      newStateValues_.insert(newGraphKey.key(), T_fixedFrame_W_initial);
      // Prior maybe not needed, but for safety (to keep well conditioned)
      newPriorPoseFactors_.emplace_back(newGraphKey.key(), T_fixedFrame_W_initial,
                                        gtsam::noiseModel::Diagonal::Sigmas(gmsfPoseUnaryMeasurementPtr_->initialSe3AlignmentNoise()));
    }
  }

  // iii) transform estimate to sensor frame
  void transformStateToSensorFrame() override {
    exp_T_fixedFrame_sensorFrame_ = composeRigidTransformations(
        exp_T_fixedFrame_sensorFrame_, gtsam::Pose3(T_I_sensorFrameInit_.matrix()));  // T_fixedFrame_sensorFrameInit
  }

  // iv) extrinsic calibration
  void addExtrinsicCalibrationCorrection(TransformsExpressionKeys<gtsam::Pose3>& transformsExpressionKeys) override {
    // Initial Guess
    const gtsam::Pose3& T_sensorFrame_sensorFrameCorrected_initial = gtsam::Pose3::Identity();  // alias for readability

    // Search for the new graph key of T_sensorFrame_sensorFrameCorrected
    bool newGraphKeyAdded = false;
    VariableType variableType = VariableType::Global();
    FactorGraphStateKey newGraphKey = transformsExpressionKeys.getTransformationKey<'c'>(
        newGraphKeyAdded, gmsfPoseUnaryMeasurementPtr_->sensorFrameName(), gmsfPoseUnaryMeasurementPtr_->sensorFrameCorrectedName(),
        gmsfPoseUnaryMeasurementPtr_->timeK(), T_sensorFrame_sensorFrameCorrected_initial, variableType);

    // Apply calibration correction
    exp_T_fixedFrame_sensorFrame_ = exp_T_fixedFrame_sensorFrame_ * gtsam::Pose3_(newGraphKey.key());  // T_fixedFrame_sensorFrameCorrected
    if (newGraphKeyAdded) {
      // Insert Values
      newStateValues_.insert(newGraphKey.key(), T_sensorFrame_sensorFrameCorrected_initial);
      // Insert Prior (maybe not needed, but for safety (to keep well conditioned))
      //      newPriorFactors_.emplace_back(newGraphKey, T_sensorFrame_sensorFrameCorrected_initial,
      //                                    gtsam::noiseModel::Diagonal::Sigmas(1.0e-03 * gtsam::Vector::Ones(6)));
    }
  }

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