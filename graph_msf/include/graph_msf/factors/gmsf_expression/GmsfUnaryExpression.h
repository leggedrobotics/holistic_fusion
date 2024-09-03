/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GMSF_UNARY_EXPRESSION_H
#define GMSF_UNARY_EXPRESSION_H

// GTSAM
#include <gtsam/base/types.h>

// Workspace
#include "graph_msf/core/TransformsExpressionKeys.h"
#include "graph_msf/measurements/UnaryMeasurement.h"

namespace graph_msf {

enum class UnaryExpressionType { Absolute, Relative, Landmark };

// Function for composing rigid body transformations with GTSAM expression factors
inline gtsam::Pose3_ composeRigidTransformations(const gtsam::Pose3_& T_frame1_frame2, const gtsam::Pose3_& T_frame2_frame3) {
  return gtsam::Pose3_(T_frame1_frame2, &gtsam::Pose3::transformPoseFrom, T_frame2_frame3);
}

// Function for Transposing of a Rot3
inline gtsam::Rot3_ inverseRot3(const gtsam::Rot3_& R) {
  return gtsam::Rot3_(&gtsam::LieGroup<gtsam::Rot3, 3>::inverse, R);
}

/**
 * UnaryExpression is a base class for unary expressions.
 * Unary expressions are used to represent unary factors in the factor graph.
 * It optionally supports the HolisticGraph paradigm to align different measurements
 * It optionally supports extrinsic calibration of the sensor
 **/

template <class GTSAM_MEASUREMENT_TYPE, UnaryExpressionType TYPE, char CALIBRATION_CHAR>
class GmsfUnaryExpression {
 public:
  // Type of the Template
  using template_type = GTSAM_MEASUREMENT_TYPE;

  // Constructor
  GmsfUnaryExpression(const std::shared_ptr<UnaryMeasurement>& gmsfBaseUnaryMeasurementPtr, const std::string& worldFrameName,
                      const std::string& imuFrameName, const Eigen::Isometry3d& T_I_sensorFrame)
      : gmsfBaseUnaryMeasurementPtr_(gmsfBaseUnaryMeasurementPtr),
        worldFrameName_(worldFrameName),
        imuFrame_(imuFrameName),
        T_I_sensorFrameInit_(T_I_sensorFrame) {}

  // Destructor
  ~GmsfUnaryExpression() = default;

  // Main method for creating the expression
  gtsam::Expression<GTSAM_MEASUREMENT_TYPE> createAndReturnExpression(const gtsam::Key& closestGeneralKey,
                                                                      TransformsExpressionKeys<gtsam::Pose3>& gtsamTransformsExpressionKeys,
                                                                      const gtsam::NavState& W_imuPropagatedState,
                                                                      const bool optimizeReferenceFramePosesWrtWorldFlag,
                                                                      const bool centerReferenceFramesAtRobotPositionBeforeAlignmentFlag,
                                                                      const bool optimizeExtrinsicSensorToSensorCorrectedOffsetFlag) {
    // Measurement Pointer
    const auto& gmsfUnaryMeasurement = *getGmsfBaseUnaryMeasurementPtr();

    // A. Generate Expression for Basic IMU State in World Frame at Key --------------------------------
    generateImuStateInWorldFrameAtKey(closestGeneralKey);

    // B.A. Holistic Fusion: Optimize over fixed frame poses --------------------------------------------
    if constexpr (TYPE == UnaryExpressionType::Absolute) {
      if (optimizeReferenceFramePosesWrtWorldFlag) {
        transformImuStateFromWorldToReferenceFrame(gtsamTransformsExpressionKeys, W_imuPropagatedState,
                                                centerReferenceFramesAtRobotPositionBeforeAlignmentFlag);
      }
    }

    // B.B. Holistic Fusion: Create Landmark State in Dynamic Memory -------------------------------------
    if constexpr (TYPE == UnaryExpressionType::Landmark) {
      transformLandmarkInWorldToImuFrame(gtsamTransformsExpressionKeys, W_imuPropagatedState);
    }

    // C. Transform State to Sensor Frame -----------------------------------------------------
    if (gmsfUnaryMeasurement.sensorFrameName() != imuFrame_) {
      transformImuStateToSensorFrameState();
    }

    // D. Extrinsic Calibration: Add correction to sensor pose -----------------------------------------
    if (optimizeExtrinsicSensorToSensorCorrectedOffsetFlag) {
      transformSensorFrameStateToSensorFrameCorrectedState(gtsamTransformsExpressionKeys);
    }

    // Return
    return getGtsamExpression();
  }

  // Noise as GTSAM Datatype
  virtual const gtsam::Vector getNoiseDensity() const = 0;

  // Return Measurement as GTSAM Datatype
  virtual const GTSAM_MEASUREMENT_TYPE getGtsamMeasurementValue() const = 0;

  virtual const std::shared_ptr<UnaryMeasurement>& getGmsfBaseUnaryMeasurementPtr() { return gmsfBaseUnaryMeasurementPtr_; }

  // Time
  [[nodiscard]] double getTimestamp() const { return gmsfBaseUnaryMeasurementPtr_->timeK(); }

  // New Values
  [[nodiscard]] const gtsam::Values& getNewStateValues() const { return newStateValues_; }

  // New Prior Factors
  const std::vector<gtsam::PriorFactor<gtsam::Pose3>>& getNewPriorPoseFactors() const { return newPriorPoseFactors_; }

  const Eigen::Isometry3d& getT_I_sensorFrameInit() const { return T_I_sensorFrameInit_; }

 protected:
  // Methods ---------------------------------------------------------------------
  // Four cases (non-exclusive, but has to be correct order!):
  // i) Generate Expression for Basic IMU State in World Frame at Key
  virtual void generateImuStateInWorldFrameAtKey(const gtsam::Key& closestGeneralKey) = 0;

  // ii).A holistically optimize over fixed frames
  virtual void transformImuStateFromWorldToReferenceFrame(TransformsExpressionKeys<gtsam::Pose3>& transformsExpressionKeys,
                                                       const gtsam::NavState& W_currentPropagatedState,
                                                       const bool centerMeasurementsAtRobotPositionBeforeAlignment) = 0;

  // ii).B adding landmark state in dynamic memory
  virtual void transformLandmarkInWorldToImuFrame(TransformsExpressionKeys<gtsam::Pose3>& transformsExpressionKeys,
                                                          const gtsam::NavState& W_currentPropagatedState) = 0;

  // iii) transform measurement to core imu frame
  virtual void transformImuStateToSensorFrameState() = 0;

  // iv) extrinsic calibration
  virtual void transformSensorFrameStateToSensorFrameCorrectedState(TransformsExpressionKeys<gtsam::Pose3>& transformsExpressionKeys) {
    // Initial Guess
    GTSAM_MEASUREMENT_TYPE initialGuess = GTSAM_MEASUREMENT_TYPE::Identity();

    // Search for new graph key
    bool newGraphKeyAddedFlag = false;
    VariableType variableType = VariableType::Global();
    FactorGraphStateKey newGraphKey = transformsExpressionKeys.getTransformationKey<CALIBRATION_CHAR>(
        newGraphKeyAddedFlag, gmsfBaseUnaryMeasurementPtr_->sensorFrameName(), gmsfBaseUnaryMeasurementPtr_->sensorFrameCorrectedName(),
        gmsfBaseUnaryMeasurementPtr_->timeK(), convertToPose3(initialGuess), variableType);

    // Define expression
    gtsam::Expression<GTSAM_MEASUREMENT_TYPE> exp_C_sensorFrame_sensorFrameCorrected(newGraphKey.key());

    // Apply calibration correction
    applyExtrinsicCalibrationCorrection(exp_C_sensorFrame_sensorFrameCorrected);

    // Initial Values
    if (newGraphKeyAddedFlag) {
      newStateValues_.insert(newGraphKey.key(), initialGuess);
    }
  }

  virtual void applyExtrinsicCalibrationCorrection(
      const gtsam::Expression<GTSAM_MEASUREMENT_TYPE>& exp_C_sensorFrame_sensorFrameCorrected) = 0;

  virtual gtsam::Pose3 convertToPose3(const GTSAM_MEASUREMENT_TYPE& measurement) = 0;

  // Return Expression
  virtual const gtsam::Expression<GTSAM_MEASUREMENT_TYPE> getGtsamExpression() const = 0;

  // Members ---------------------------------------------------------------------
  // Main Measurement Pointer
  // Full Measurement Type
  const std::shared_ptr<UnaryMeasurement> gmsfBaseUnaryMeasurementPtr_;

  // Frame Name References
  const std::string worldFrameName_;
  const std::string imuFrame_;

  // IMU to Sensor Frame
  Eigen::Isometry3d T_I_sensorFrameInit_;

  // Containers
  gtsam::Values newStateValues_;
  std::vector<gtsam::PriorFactor<gtsam::Pose3>> newPriorPoseFactors_;
};

}  // namespace graph_msf

#endif  // GMSF_UNARY_EXPRESSION_H
