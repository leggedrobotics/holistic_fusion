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
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

// Workspace
#include "graph_msf/core/DynamicDictionaryContainer.h"
#include "graph_msf/measurements/UnaryMeasurement.h"

namespace graph_msf {

enum class UnaryExpressionType { Absolute, Local, Landmark };

// Function for composing rigid body transformations with GTSAM expression factors
inline gtsam::Pose3_ composeRigidTransformations(const gtsam::Pose3_& T_frame1_frame2, const gtsam::Pose3_& T_frame2_frame3) {
  return gtsam::Pose3_(T_frame1_frame2, &gtsam::Pose3::transformPoseFrom, T_frame2_frame3);
}

// Function for Transposing of a Rot3
inline gtsam::Rot3_ inverseRot3(const gtsam::Rot3_& R) {
  return gtsam::Rot3_(&gtsam::LieGroup<gtsam::Rot3, 3>::inverse, R);
}

/**
 * @class GmsfUnaryExpression is a base class for unary expressions.
 * Unary expressions are used to represent unary factors in the factor graph.
 * It optionally supports the HolisticGraph paradigm to align different measurements.
 * It optionally supports extrinsic calibration of the sensor.
 *
 * @tparam GTSAM_MEASUREMENT_TYPE Type of the measurement (e.g. gtsam::Pose3).
 * @tparam TYPE Type of the unary expression (e.g. UnaryExpressionType::Absolute).
 * @tparam CALIBRATION_CHAR Character representing the calibration type (e.g. 'c').
 */
template <class GTSAM_MEASUREMENT_TYPE, UnaryExpressionType TYPE, char CALIBRATION_CHAR>
class GmsfUnaryExpression {
 public:
  // Type of the Template
  using template_type = GTSAM_MEASUREMENT_TYPE;

  // Constructor
  GmsfUnaryExpression(const std::shared_ptr<UnaryMeasurement>& gmsfBaseUnaryMeasurementPtr, const std::string& imuFrameName,
                      const Eigen::Isometry3d& T_I_sensorFrame)
      : gmsfBaseUnaryMeasurementPtr_(gmsfBaseUnaryMeasurementPtr), imuFrame_(imuFrameName), T_I_sensorFrameInit_(T_I_sensorFrame) {}

  // Destructor
  ~GmsfUnaryExpression() = default;

  // Main method for creating the expression
  gtsam::Expression<GTSAM_MEASUREMENT_TYPE> createAndReturnExpression(const gtsam::Key& closestGeneralKey,
                                                                      DynamicDictionaryContainer& gtsamDynamicExpressionKeys,
                                                                      const gtsam::NavState& W_imuPropagatedState,
                                                                      const bool optimizeReferenceFramePosesWrtWorldFlag,
                                                                      const bool centerMeasurementsAtKeyframePositionBeforeAlignmentFlag,
                                                                      const bool optimizeExtrinsicSensorToSensorCorrectedOffsetFlag) {
    // Measurement Pointer
    const auto& gmsfUnaryMeasurement = *getGmsfBaseUnaryMeasurementPtr();

    // A. Generate Expression for Basic IMU State in World Frame at Key --------------------------------
    generateImuStateInWorldFrameAtKey(closestGeneralKey);

    // B.A. Holistic Fusion: Optimize over reference frame poses --------------------------------------------
    if constexpr (TYPE == UnaryExpressionType::Absolute) {
      if (optimizeReferenceFramePosesWrtWorldFlag) {
        transformImuStateFromWorldToReferenceFrame(gtsamDynamicExpressionKeys, W_imuPropagatedState,
                                                   centerMeasurementsAtKeyframePositionBeforeAlignmentFlag);
      }
    }

    // B.B. Holistic Fusion: Create Landmark State in Dynamic Memory -------------------------------------
    if constexpr (TYPE == UnaryExpressionType::Landmark) {
      transformLandmarkInWorldToImuFrame(gtsamDynamicExpressionKeys, W_imuPropagatedState);
    }

    // C. Transform State to Sensor Frame -----------------------------------------------------
    if (gmsfUnaryMeasurement.sensorFrameName() != imuFrame_) {
      transformImuStateToSensorFrameState();
    }

    // D. Extrinsic Calibration: Add correction to sensor pose -----------------------------------------
    if (optimizeExtrinsicSensorToSensorCorrectedOffsetFlag) {
      transformSensorFrameStateToSensorFrameCorrectedState(gtsamDynamicExpressionKeys);
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
  [[nodiscard]] const gtsam::Values& getNewOnlineGraphStateValues() const { return newOnlineStateValues_; }

  [[nodiscard]] const gtsam::Values& getNewOfflineGraphStateValues() const { return newOfflineStateValues_; }

  // New Prior Factors
  const std::vector<gtsam::PriorFactor<gtsam::Pose3>>& getNewOnlinePosePriorFactors() const { return newOnlinePosePriorFactors_; }

  const std::vector<gtsam::PriorFactor<GTSAM_MEASUREMENT_TYPE>>& getNewOnlineDynamicPriorFactors() const {
    return newOnlineDynamicPriorFactors_;
  }

  // New Between Factors
  const std::vector<gtsam::BetweenFactor<gtsam::Pose3>>& getNewOnlineAndOfflinePoseBetweenFactors() const {
    return newOnlineAndOfflinePoseBetweenFactors_;
  }

  // Get Imu to sensor frame calibration
  [[nodiscard]] const Eigen::Isometry3d& getT_I_sensorFrameInit() const { return T_I_sensorFrameInit_; }

 protected:
  // Methods ---------------------------------------------------------------------
  // Four cases (non-exclusive, but has to be correct order!):
  // i) Generate Expression for Basic IMU State in World Frame at Key
  virtual void generateImuStateInWorldFrameAtKey(const gtsam::Key& closestGeneralKey) = 0;

  // ii).A Holistically optimize over fixed frames
  virtual void transformImuStateFromWorldToReferenceFrame(DynamicDictionaryContainer& gtsamDynamicExpressionKeys,
                                                          const gtsam::NavState& W_currentPropagatedState,
                                                          const bool centerMeasurementsAtKeyframePositionBeforeAlignmentFlag) = 0;

  // ii).B Adding landmark state in dynamic memory
  virtual void transformLandmarkInWorldToImuFrame(DynamicDictionaryContainer& gtsamDynamicExpressionKeys,
                                                  const gtsam::NavState& W_currentPropagatedState) = 0;

  // iii) Transform state to sensor frame
  virtual void transformImuStateToSensorFrameState() = 0;

  // iv) Extrinsic calibration
  virtual void transformSensorFrameStateToSensorFrameCorrectedState(DynamicDictionaryContainer& gtsamDynamicExpressionKeys) {
    // Mutex because we are changing the dynamically allocated graphKeys
    std::lock_guard<std::mutex> modifyGraphKeysLock(gtsamDynamicExpressionKeys.get<gtsam::Pose3>().mutex());

    // Initial Guess
    GTSAM_MEASUREMENT_TYPE initialGuess = GTSAM_MEASUREMENT_TYPE::Identity();

    // Search for new graph key
    bool newGraphKeyAddedFlag = false;
    DynamicVariableType variableType = DynamicVariableType::Global(gmsfBaseUnaryMeasurementPtr_->timeK());
    DynamicFactorGraphStateKey graphKey = gtsamDynamicExpressionKeys.get<gtsam::Pose3>().getTransformationKey<CALIBRATION_CHAR>(
        newGraphKeyAddedFlag, gmsfBaseUnaryMeasurementPtr_->sensorFrameName(), gmsfBaseUnaryMeasurementPtr_->sensorFrameCorrectedName(),
        gmsfBaseUnaryMeasurementPtr_->timeK(), convertToPose3(initialGuess), variableType);

    // Define expression
    gtsam::Expression<GTSAM_MEASUREMENT_TYPE> exp_C_sensorFrame_sensorFrameCorrected(graphKey.key());

    // Apply calibration correction
    this->applyExtrinsicCalibrationCorrection(exp_C_sensorFrame_sensorFrameCorrected);  // Virtual method to be implemented

    // If key was newly added, just add a value to the new state values to online and offline graph (as completely new)
    if (newGraphKeyAddedFlag) {
      // Add value to new state values of online and offline graph
      newOnlineStateValues_.insert(graphKey.key(), initialGuess);
      newOfflineStateValues_.insert(graphKey.key(), initialGuess);
      // Add prior factor to online graph --> make sure it is not fully unconstrained
      //      newOnlineDynamicPriorFactors_.emplace_back(
      //          graphKey.key(), initialGuess,
      //          gtsam::noiseModel::Gaussian::Covariance(gtsam::Matrix::Identity(GTSAM_MEASUREMENT_TYPE::dimension,
      //          GTSAM_MEASUREMENT_TYPE::dimension)));
      // New key has to be active
      assert(graphKey.isVariableActive());
    }
    // If the key is inactive(), we have to activate it again and add a prior with the previous belief ONLY to the online graph
    // TODO: Finish up
    else if (!graphKey.isVariableActive() && graphKey.getNumberStepsOptimized() > 0) {
      assert(graphKey.getNumberStepsOptimized() > 0);
      REGULAR_COUT << " GmsfUnaryExpression: Transformation between " << gmsfBaseUnaryMeasurementPtr_->sensorFrameName() << " and "
                   << gmsfBaseUnaryMeasurementPtr_->sensorFrameCorrectedName()
                   << " was inactive. Activating it again and adding a prior to the online graph." << std::endl;
      // Add prior factor
      gtsam::Pose3 priorBelief = graphKey.getTransformationAfterOptimization();  // TODO: Does not have to be 6D
      gtsam::Matrix66 priorCovariance = graphKey.getCovarianceAfterOptimization();
      // Convert back to GTSAM_MEASUREMENT_TYPE
      GTSAM_MEASUREMENT_TYPE priorBeliefConverted = convertFromPose3(priorBelief);
      gtsam::Matrix priorCovarianceConverted = priorCovariance.bottomRightCorner<6, 6>();  // TODO: Not correct
      // Add prior factor to online graph
      newOnlineDynamicPriorFactors_.emplace_back(graphKey.key(), priorBeliefConverted,
                                                 gtsam::noiseModel::Gaussian::Covariance(priorCovarianceConverted));
      // Add value to new state values of online graph
      newOnlineStateValues_.insert(graphKey.key(), priorBeliefConverted);
      // Activate key and set number of steps optimized to 0
      graphKey.activateVariable();
      graphKey.resetNumberStepsOptimized();
    }
    // If it was never optimized, we can also remove it as we will not be able to reactivate it
    else if (!graphKey.isVariableActive() && graphKey.getNumberStepsOptimized() == 0) {
      REGULAR_COUT << " GmsfUnaryExpression: Transformation between " << gmsfBaseUnaryMeasurementPtr_->sensorFrameName() << " and "
                   << gmsfBaseUnaryMeasurementPtr_->sensorFrameCorrectedName()
                   << " is deactivated but was never optimized. Hence, Removing it from the graph." << std::endl;
      // Remove key
      DynamicFactorGraphStateKey<gtsam::Pose3> keyToRemoveOrDeactivate;
      std::ignore = gtsamDynamicExpressionKeys.get<gtsam::Pose3>().removeTransform(gmsfBaseUnaryMeasurementPtr_->sensorFrameName(),
                                                                                   gmsfBaseUnaryMeasurementPtr_->sensorFrameCorrectedName(),
                                                                                   keyToRemoveOrDeactivate);
    }
  }

  virtual void applyExtrinsicCalibrationCorrection(
      const gtsam::Expression<GTSAM_MEASUREMENT_TYPE>& exp_C_sensorFrame_sensorFrameCorrected) = 0;

  // TODO: Make clean, GTSAM_MEASUREMENT_TYPE is not always the type of our dynamic variables

  virtual gtsam::Pose3 convertToPose3(const GTSAM_MEASUREMENT_TYPE& measurement) = 0;

  virtual GTSAM_MEASUREMENT_TYPE convertFromPose3(const gtsam::Pose3& pose) = 0;

  // Return Expression
  virtual const gtsam::Expression<GTSAM_MEASUREMENT_TYPE> getGtsamExpression() const = 0;

  // Members ---------------------------------------------------------------------
  // Main Measurement Pointer
  // Full Measurement Type
  const std::shared_ptr<UnaryMeasurement> gmsfBaseUnaryMeasurementPtr_;

  // Frame Name References
  const std::string imuFrame_;

  // IMU to Sensor Frame
  Eigen::Isometry3d T_I_sensorFrameInit_;

  // Containers
  // New Values
  gtsam::Values newOnlineStateValues_;
  gtsam::Values newOfflineStateValues_;
  // New Prior Factors
  std::vector<gtsam::PriorFactor<gtsam::Pose3>> newOnlinePosePriorFactors_;
  // TODO: Make this more general
  std::vector<gtsam::PriorFactor<GTSAM_MEASUREMENT_TYPE>> newOnlineDynamicPriorFactors_;
  // New Between Factors
  std::vector<gtsam::BetweenFactor<gtsam::Pose3>> newOnlineAndOfflinePoseBetweenFactors_;
};

}  // namespace graph_msf

#endif  // GMSF_UNARY_EXPRESSION_H
