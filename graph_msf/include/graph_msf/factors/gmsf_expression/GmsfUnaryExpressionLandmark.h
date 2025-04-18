/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GMSF_UNARY_EXPRESSION_LANDMARK_H
#define GMSF_UNARY_EXPRESSION_LANDMARK_H

// GTSAM
#include <gtsam/base/types.h>

// Workspace
#include "graph_msf/factors/gmsf_expression/GmsfUnaryExpression.h"
#include "graph_msf/measurements/UnaryMeasurementLandmark.h"

namespace graph_msf {

template <class GTSAM_MEASUREMENT_TYPE, char CALIBRATION_CHAR>
class GmsfUnaryExpressionLandmark : public GmsfUnaryExpression<GTSAM_MEASUREMENT_TYPE, UnaryExpressionType::Landmark, CALIBRATION_CHAR> {
 public:
  // Constructor
  GmsfUnaryExpressionLandmark(const std::shared_ptr<UnaryMeasurementLandmark>& gmsfUnaryLandmarkMeasurementPtr,
                              const std::string& imuFrameName, const Eigen::Isometry3d& T_I_sensorFrame, const long landmarkCreationCounter)
      : GmsfUnaryExpression<GTSAM_MEASUREMENT_TYPE, UnaryExpressionType::Landmark, CALIBRATION_CHAR>(gmsfUnaryLandmarkMeasurementPtr,
                                                                                                     imuFrameName, T_I_sensorFrame),
        landmarkName_(this->gmsfBaseUnaryMeasurementPtr_->measurementName()),
        landmarkCreationCounter_(landmarkCreationCounter),
        exp_T_W_I_(gtsam::Pose3::Identity()),
        exp_sensorFrame_t_sensorFrame_landmark_(gtsam::Point3::Identity()),
        gmsfUnaryLandmarkMeasurementPtr_(gmsfUnaryLandmarkMeasurementPtr) {}

  // Destructor
  ~GmsfUnaryExpressionLandmark() = default;

 protected:
  // Virtual Methods
  // i) Generate Expression for Basic IMU State in World Frame at Key
  void generateImuStateInWorldFrameAtKey(const gtsam::Key& closestGeneralKey) final {
    // Get robot state
    exp_T_W_I_ = gtsam::Expression<gtsam::Pose3>(gtsam::symbol_shorthand::X(closestGeneralKey));  // T_W_I
  }

  // ii.A) Holistically Optimize over Fixed Frames
  void transformImuStateFromWorldToReferenceFrame(DynamicDictionaryContainer& gtsamDynamicExpressionKeys,
                                                  const gtsam::NavState& W_currentPropagatedState,
                                                  const bool centerMeasurementsAtKeyframePositionBeforeAlignmentFlag) final {
    // Do nothing as this is a landmark measurement
    throw std::logic_error("GmsfUnaryExpressionLandmark: transformStateFromWorldToFixedFrame not implemented for Landmark Measurements.");
  }

  // ii.B) Adding Landmark State in Dynamic Memory
  void transformLandmarkInWorldToImuFrame(DynamicDictionaryContainer& gtsamDynamicExpressionKeys,
                                          const gtsam::NavState& W_currentPropagatedState) final {
    // Mutex because we are changing the dynamically allocated graphKeys
    std::lock_guard<std::mutex> modifyGraphKeysLock(gtsamDynamicExpressionKeys.get<gtsam::Pose3>().mutex());

    // Get initial guess (computed geometrically)
    gtsam::Point3 W_t_W_L_initial = computeW_t_W_L_initial(W_currentPropagatedState);

    // Landmark names
    const std::string newLandmarkName = landmarkName_ + "_" + std::to_string(landmarkCreationCounter_);
    const std::string previousLandmarkName = landmarkName_ + "_" + std::to_string(landmarkCreationCounter_ - 1);

    // Create new graph key for landmark dynamically
    bool newGraphKeyAddedFlag = false;
    DynamicVariableType variableType = DynamicVariableType::Landmark(this->gmsfBaseUnaryMeasurementPtr_->timeK());
    const DynamicFactorGraphStateKey newGraphKey = gtsamDynamicExpressionKeys.get<gtsam::Pose3>().getTransformationKey<'l'>(
        newGraphKeyAddedFlag, gmsfUnaryLandmarkMeasurementPtr_->worldFrameName(), newLandmarkName,
        this->gmsfBaseUnaryMeasurementPtr_->timeK(), gtsam::Pose3(gtsam::Rot3::Identity(), W_t_W_L_initial), variableType);
    // Make sure that the variable at the key is active (landmarks always have to be active or removed)
    assert(newGraphKey.isVariableActive());

    // Remove previous landmark with same landmark name;
    if (newGraphKeyAddedFlag) {
      std::ignore = gtsamDynamicExpressionKeys.get<gtsam::Pose3>().removeOrDeactivateTransform(
          gmsfUnaryLandmarkMeasurementPtr_->worldFrameName(), previousLandmarkName);
    }

    // Create expression for landmark
    const gtsam::Point3_ exp_W_t_W_L = gtsam::Point3_(newGraphKey.key());

    // Convert to Imu frame
    exp_sensorFrame_t_sensorFrame_landmark_ = gtsam::transformTo(exp_T_W_I_, exp_W_t_W_L);  // I_t_I_L at this point

    // Add values, if a new state was created
    if (newGraphKeyAddedFlag) {
      // Add initial guess
      this->newOnlineStateValues_.insert(newGraphKey.key(), W_t_W_L_initial);
      this->newOfflineStateValues_.insert(newGraphKey.key(), W_t_W_L_initial);
    }
  }

  // Sub Functions that have to be implemented in derived classes --------------------------------
  virtual gtsam::Point3 computeW_t_W_L_initial(const gtsam::NavState& W_currentPropagatedState) = 0;

  // Variables ----------------------------------------------------------------------------------
  // Landmark Identifier
  const std::string& landmarkName_;
  const long landmarkCreationCounter_;

  // Expression
  gtsam::Pose3_ exp_T_W_I_;                                // Robot Pose
  gtsam::Point3_ exp_sensorFrame_t_sensorFrame_landmark_;  // Measurement --> this is what h(x) has to be

 private:
  // Pointer to the GMSF Unary Landmark Measurement
  std::shared_ptr<UnaryMeasurementLandmark> gmsfUnaryLandmarkMeasurementPtr_;
};

}  // namespace graph_msf

#endif  // GMSF_UNARY_EXPRESSION_LANDMARK_H
