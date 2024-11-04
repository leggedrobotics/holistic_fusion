/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GMSF_UNARY_EXPRESSION_ABSOLUT_H
#define GMSF_UNARY_EXPRESSION_ABSOLUT_H

// GTSAM
#include <gtsam/base/types.h>
#include <gtsam/navigation/NavState.h>

// Workspace
#include "graph_msf/factors/gmsf_expression/GmsfUnaryExpression.h"
#include "graph_msf/measurements/UnaryMeasurementAbsolute.h"

namespace graph_msf {

template <class GTSAM_MEASUREMENT_TYPE, char CALIBRATION_CHAR>
class GmsfUnaryExpressionAbsolut : public GmsfUnaryExpression<GTSAM_MEASUREMENT_TYPE, UnaryExpressionType::Absolute, CALIBRATION_CHAR> {
 public:
  // Constructor
  GmsfUnaryExpressionAbsolut(const std::shared_ptr<UnaryMeasurementAbsolute>& gmsfUnaryAbsoluteMeasurementPtr,
                             const std::string& imuFrameName, const Eigen::Isometry3d& T_I_sensorFrame,
                             const double createReferenceAlignmentKeyframeEveryNSeconds)
      : GmsfUnaryExpression<GTSAM_MEASUREMENT_TYPE, UnaryExpressionType::Absolute, CALIBRATION_CHAR>(gmsfUnaryAbsoluteMeasurementPtr,
                                                                                                     imuFrameName, T_I_sensorFrame),
        gmsfUnaryAbsoluteMeasurementPtr_(gmsfUnaryAbsoluteMeasurementPtr),
        createReferenceAlignmentKeyframeEveryNSeconds_(createReferenceAlignmentKeyframeEveryNSeconds) {}

  // Destructor
  ~GmsfUnaryExpressionAbsolut() = default;

 protected:
  // Virtual Methods --------------------------------------------------------------
  // ii).A Holistically Optimize over Fixed Frames
  void transformImuStateFromWorldToReferenceFrame(DynamicTransformDictionary<gtsam::Pose3>& transformsExpressionKeys,
                                                  const gtsam::NavState& W_currentPropagatedState,
                                                  const bool centerMeasurementsAtRobotPositionBeforeAlignment) final {
    if (gmsfUnaryAbsoluteMeasurementPtr_->fixedFrameName() == gmsfUnaryAbsoluteMeasurementPtr_->worldFrameName()) {
      // Do nothing as this is a world frame measurement
      return;
    }

    // Mutex because we are changing the dynamically allocated graphKeys
    std::lock_guard<std::mutex> modifyGraphKeysLock(transformsExpressionKeys.mutex());

    // Run through steps needed for absolute measurements
    // If it should be centered --> create keyframe for measurement
    Eigen::Vector3d measurementOriginPosition = Eigen::Vector3d::Zero();
    if (centerMeasurementsAtRobotPositionBeforeAlignment) {
      measurementOriginPosition = getMeasurementPosition();
    }

    // Initial Guess
    gtsam::Pose3 T_fixedFrame_W_initial = computeT_fixedFrame_W_initial(W_currentPropagatedState);

    // Search for the new graph key of T_fixedFrame_W
    bool newGraphKeyAddedFlag = false;
    const DynamicVariableType dynamicVariableType =
        DynamicVariableType::RefFrame(measurementOriginPosition, gmsfUnaryAbsoluteMeasurementPtr_->timeK());
    DynamicFactorGraphStateKey graphKey = transformsExpressionKeys.getTransformationKey<'r'>(
        newGraphKeyAddedFlag, gmsfUnaryAbsoluteMeasurementPtr_->fixedFrameName(), gmsfUnaryAbsoluteMeasurementPtr_->worldFrameName(),
        gmsfUnaryAbsoluteMeasurementPtr_->timeK(), T_fixedFrame_W_initial, dynamicVariableType);

    const double keyframeAge = graphKey.computeVariableAge(gmsfUnaryAbsoluteMeasurementPtr_->timeK());
    //    std::cout << "Keyframe age: " << keyframeAge << std::endl;

    // If deactivated or too old, we should remove the old keyframe and create a new one (with or without random walk)
    // In any case, we have to add relation between old and new keyframe
    // Offline graph: we can always add the delta as all states are part of the optimization (random walk or deterministic keyframe
    // displacement).
    // Online graph: we can only add the delta if the old keyframe is still active (random walk or deterministic keyframe displacement. If
    // it is not active, add prior from old keyframe to new keyframe (including the displacement).
    bool introducedNewKeyframeDisplacement = false;
    // Old GTSAM key --> needed for adding constraints between previous and new keyframe
    const gtsam::Key oldGtsamKey = graphKey.key();
    const Eigen::Vector3d oldKeyframePosition = graphKey.getReferenceFrameKeyframePosition();
    bool oldVariableWasActive = graphKey.isVariableActive();
    if (!graphKey.isVariableActive() || keyframeAge > createReferenceAlignmentKeyframeEveryNSeconds_) {
      // Remove the old keyframe
      transformsExpressionKeys.removeTransform(gmsfUnaryAbsoluteMeasurementPtr_->fixedFrameName(),
                                               gmsfUnaryAbsoluteMeasurementPtr_->worldFrameName(), graphKey);
      // Create a new keyframe
      graphKey = transformsExpressionKeys.getTransformationKey<'r'>(
          newGraphKeyAddedFlag, gmsfUnaryAbsoluteMeasurementPtr_->fixedFrameName(), gmsfUnaryAbsoluteMeasurementPtr_->worldFrameName(),
          gmsfUnaryAbsoluteMeasurementPtr_->timeK(), T_fixedFrame_W_initial, dynamicVariableType);
      // Assert that new keyframe was added and that it is active
      assert(newGraphKeyAddedFlag && graphKey.isVariableActive());
      // Set flag that we introduced a new keyframe
      introducedNewKeyframeDisplacement = true;
    }

    // Shift the measurement to the robot position and recompute initial guess if we create keyframes
    // Has to be done here, as we did not know the keyframe position before
    if (centerMeasurementsAtRobotPositionBeforeAlignment) {
      // Shift the measurement to the robot position
      setMeasurementPosition(getMeasurementPosition() - graphKey.getReferenceFrameKeyframePosition());
      // Recompute initial guess
      T_fixedFrame_W_initial = computeT_fixedFrame_W_initial(W_currentPropagatedState);
      // Update the initial guess
      transformsExpressionKeys
          .lv_T_frame1_frame2(gmsfUnaryAbsoluteMeasurementPtr_->fixedFrameName(), gmsfUnaryAbsoluteMeasurementPtr_->worldFrameName())
          .setApproximateTransformationBeforeOptimization(T_fixedFrame_W_initial);
    }

    // Define expression for T_fixedFrame_W
    gtsam::Pose3_ exp_T_fixedFrame_W(graphKey.key());  // T_fixedFrame_W

    // Call main child function
    transformStateToReferenceFrameMeasurement(exp_T_fixedFrame_W);

    // Initial values (and priors)
    if (newGraphKeyAddedFlag) {
      REGULAR_COUT << " Initial Guess for T_" << gmsfUnaryAbsoluteMeasurementPtr_->fixedFrameName()
                   << "_W, RPY (deg): " << T_fixedFrame_W_initial.rotation().rpy().transpose() * (180.0 / M_PI)
                   << ", t (x, y, z): " << T_fixedFrame_W_initial.translation().transpose() << std::endl;
      // Insert Values
      this->newOnlineStateValues_.insert(graphKey.key(), T_fixedFrame_W_initial);
      this->newOfflineStateValues_.insert(graphKey.key(), T_fixedFrame_W_initial);

      // Case 1: Entirely new keyframe has been added
      if (!introducedNewKeyframeDisplacement) {
        // Insert Prior ONLY for online graph (offline is observable regardless)
        this->newOnlinePosePriorFactors_.emplace_back(
            graphKey.key(), T_fixedFrame_W_initial,
            gtsam::noiseModel::Diagonal::Sigmas(gmsfUnaryAbsoluteMeasurementPtr_->initialSe3AlignmentNoise()));
      }
      // Case 2: New keyframe has been added, but one existed before already --> add relative constraint from old to new keyframe
      else {
        // Add prior from old keyframe to new keyframe
        REGULAR_COUT << GREEN_START << "Adding relative constraint from key " << gtsam::Symbol(oldGtsamKey) << " to key "
                     << gtsam::Symbol(graphKey.key()) << COLOR_END << std::endl;
      }
    }
  }

  // ii).B Adding Landmark State in Dynamic Memory
  void transformLandmarkInWorldToImuFrame(DynamicTransformDictionary<gtsam::Pose3>& transformsExpressionKeys,
                                          const gtsam::NavState& W_currentPropagatedState) final {
    // Raise logic error, as it is not a landmark measurement
    throw std::logic_error("GmsfUnaryExpressionAbsolut: convertRobotAndLandmarkStatesToMeasurement() is not implemented.");
  }

  // Sub Functions that have to be implemented in derived classes ----------------
  virtual gtsam::Pose3 computeT_fixedFrame_W_initial(const gtsam::NavState& W_currentPropagatedState) = 0;

  virtual const Eigen::Vector3d getMeasurementPosition() = 0;

  virtual void setMeasurementPosition(const Eigen::Vector3d& position) = 0;

  virtual void transformStateToReferenceFrameMeasurement(const gtsam::Pose3_& exp_T_fixedFrame_W) = 0;

 private:
  // Pointer to the GMSF Unary Absolute Measurement
  std::shared_ptr<UnaryMeasurementAbsolute> gmsfUnaryAbsoluteMeasurementPtr_;
  // Create Reference Alignment Keyframe Every N Seconds
  const double createReferenceAlignmentKeyframeEveryNSeconds_;
};

}  // namespace graph_msf

#endif  // GMSF_UNARY_EXPRESSION_ABSOLUT_H
