/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GMSF_BINARY_EXPRESSION_POSE3_H
#define GMSF_BINARY_EXPRESSION_POSE3_H

// C++
#include <stdexcept>

// GTSAM
#include <gtsam/base/types.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/expressions.h>

// Workspace
#include "graph_msf/factors/gmsf_expression/GmsfUnaryExpression.h"
#include "graph_msf/factors/gmsf_expression/Pose3CalibrationState.h"
#include "graph_msf/measurements/BinaryMeasurementXD.h"

namespace graph_msf {

using GMSF_BINARY_MEASUREMENT_CLASS = BinaryMeasurementXD<Eigen::Isometry3d, 6>;

class GmsfBinaryExpressionPose3 final {
 public:
  using template_type = gtsam::Pose3;

  GmsfBinaryExpressionPose3(const std::shared_ptr<GMSF_BINARY_MEASUREMENT_CLASS> poseBinaryMeasurementPtr, const std::string& imuFrameName,
                            const Eigen::Isometry3d& T_I_sensorFrame)
      : gmsfBaseBinaryMeasurementPtr_(poseBinaryMeasurementPtr),
        gmsfPoseBinaryMeasurementPtr_(poseBinaryMeasurementPtr),
        imuFrame_(imuFrameName),
        T_I_sensorFrameInit_(T_I_sensorFrame),
        gtsamMeasurementValue_(gtsam::Pose3::Identity()),
        exp_T_W_sensorFrameKm1_(gtsam::Pose3::Identity()),
        exp_T_W_sensorFrameK_(gtsam::Pose3::Identity()),
        exp_T_sensorFrameKm1_sensorFrameK_(gtsam::Pose3::Identity()),
        exp_factorOutput_(gtsam::Pose3::Identity()) {}

  ~GmsfBinaryExpressionPose3() = default;

  [[nodiscard]] const gtsam::Vector getNoiseDensity() const { return gmsfPoseBinaryMeasurementPtr_->measurementNoiseDensity(); }

  [[nodiscard]] const gtsam::Pose3 getGtsamMeasurementValue() const { return gtsamMeasurementValue_; }

  [[nodiscard]] gtsam::Pose3 getScaledGtsamMeasurementValue(const double matchedKeyTimeDistance) const {
    const double measurementDt = gmsfPoseBinaryMeasurementPtr_->timeK() - gmsfPoseBinaryMeasurementPtr_->timeKm1();
    if (measurementDt <= 1e-9) {
      throw std::logic_error("GmsfBinaryExpressionPose3: binary measurement must have strictly positive dt.");
    }
    if (matchedKeyTimeDistance < 0.0) {
      throw std::logic_error("GmsfBinaryExpressionPose3: matched key timestamp distance must be non-negative.");
    }

    const gtsam::Pose3 rawMeasurement(gmsfPoseBinaryMeasurementPtr_->deltaMeasurement().matrix());
    if (matchedKeyTimeDistance <= 1e-12) {
      return gtsam::Pose3::Identity();
    }

    const double scale = matchedKeyTimeDistance / measurementDt;
    return gtsam::Pose3::Expmap(scale * gtsam::Pose3::Logmap(rawMeasurement));
  }

  [[nodiscard]] double getTimestamp() const { return gmsfPoseBinaryMeasurementPtr_->timeK(); }

  [[nodiscard]] const std::shared_ptr<BinaryMeasurement>& getGmsfBaseBinaryMeasurementPtr() const {
    return gmsfBaseBinaryMeasurementPtr_;
  }

  [[nodiscard]] const gtsam::Values& getNewOnlineGraphStateValues() const { return newOnlineStateValues_; }
  [[nodiscard]] const gtsam::Values& getNewOfflineGraphStateValues() const { return newOfflineStateValues_; }

  const std::vector<gtsam::PriorFactor<gtsam::Pose3>>& getNewOnlineDynamicPriorFactors() const {
    return newOnlineDynamicPriorFactors_;
  }

  // Semantics:
  // - The binary measurement is always the raw relative motion of the physical sensor frame.
  // - When online extrinsic calibration is enabled, C corrects the IMU->sensor extrinsic on the state side, exactly like the unary
  //   pose path. The predicted relative sensor motion therefore changes because the endpoint sensor poses change, but the raw
  //   between measurement itself must stay untouched.
  // - Temporal resampling for timestamp-mismatched graph keys is handled separately when constructing the factor measurement value.
  // - Conjugating the measurement by C^-1 * Z * C would apply the same correction twice.
  gtsam::Expression<gtsam::Pose3> createAndReturnExpression(const gtsam::Key& closestGeneralKeyKm1, const gtsam::Key& closestGeneralKeyK,
                                                            DynamicDictionaryContainer& gtsamDynamicExpressionKeys,
                                                            const Eigen::Matrix<double, 6, 1>& initialCalibrationPriorSigmas,
                                                            const bool optimizeExtrinsicSensorToSensorCorrectedOffsetFlag,
                                                            const bool useExtrinsicCalibrationResidualFlag) {
    exp_T_W_sensorFrameKm1_ = gtsam::Expression<gtsam::Pose3>(gtsam::symbol_shorthand::X(closestGeneralKeyKm1));
    exp_T_W_sensorFrameK_ = gtsam::Expression<gtsam::Pose3>(gtsam::symbol_shorthand::X(closestGeneralKeyK));

    if (gmsfBaseBinaryMeasurementPtr_->sensorFrameName() != imuFrame_) {
      transformImuStateToSensorFrameState();
    }

    if (optimizeExtrinsicSensorToSensorCorrectedOffsetFlag && useExtrinsicCalibrationResidualFlag) {
      const auto calibrationState = ensurePose3ExtrinsicCalibrationState<'c'>(
          gtsamDynamicExpressionKeys, gmsfBaseBinaryMeasurementPtr_->sensorFrameName(),
          gmsfBaseBinaryMeasurementPtr_->sensorFrameCorrectedName(), gmsfBaseBinaryMeasurementPtr_->timeK(),
          initialCalibrationPriorSigmas);
      applyExtrinsicCalibrationCorrection(calibrationState.expression);
      mergePose3CalibrationStateResult(calibrationState, newOnlineStateValues_, newOfflineStateValues_,
                                       newOnlineDynamicPriorFactors_);
    }

    exp_T_sensorFrameKm1_sensorFrameK_ = transformPoseTo(exp_T_W_sensorFrameKm1_, exp_T_W_sensorFrameK_);
    exp_factorOutput_ = exp_T_sensorFrameKm1_sensorFrameK_;
    gtsamMeasurementValue_ = gtsam::Pose3(gmsfPoseBinaryMeasurementPtr_->deltaMeasurement().matrix());
    return exp_factorOutput_;
  }

 private:
  void transformImuStateToSensorFrameState() {
    const gtsam::Pose3 T_I_sensorFrame(T_I_sensorFrameInit_.matrix());
    exp_T_W_sensorFrameKm1_ = composeRigidTransformations(exp_T_W_sensorFrameKm1_, T_I_sensorFrame);
    exp_T_W_sensorFrameK_ = composeRigidTransformations(exp_T_W_sensorFrameK_, T_I_sensorFrame);
  }

  void applyExtrinsicCalibrationCorrection(const gtsam::Pose3_& exp_T_sensorFrame_sensorFrameCorrected) {
    exp_T_W_sensorFrameKm1_ = composeRigidTransformations(exp_T_W_sensorFrameKm1_, exp_T_sensorFrame_sensorFrameCorrected);
    exp_T_W_sensorFrameK_ = composeRigidTransformations(exp_T_W_sensorFrameK_, exp_T_sensorFrame_sensorFrameCorrected);
  }

  std::shared_ptr<BinaryMeasurement> gmsfBaseBinaryMeasurementPtr_;
  std::shared_ptr<GMSF_BINARY_MEASUREMENT_CLASS> gmsfPoseBinaryMeasurementPtr_;

  std::string imuFrame_;
  Eigen::Isometry3d T_I_sensorFrameInit_;

  gtsam::Values newOnlineStateValues_;
  gtsam::Values newOfflineStateValues_;
  std::vector<gtsam::PriorFactor<gtsam::Pose3>> newOnlineDynamicPriorFactors_;
  gtsam::Pose3 gtsamMeasurementValue_;

  gtsam::Expression<gtsam::Pose3> exp_T_W_sensorFrameKm1_;
  gtsam::Expression<gtsam::Pose3> exp_T_W_sensorFrameK_;
  gtsam::Expression<gtsam::Pose3> exp_T_sensorFrameKm1_sensorFrameK_;
  gtsam::Expression<gtsam::Pose3> exp_factorOutput_;
};

}  // namespace graph_msf

#endif  // GMSF_BINARY_EXPRESSION_POSE3_H
