/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GMSF_POINT3_CALIBRATION_STATE_H
#define GMSF_POINT3_CALIBRATION_STATE_H

// C++
#include <cassert>
#include <mutex>
#include <stdexcept>
#include <vector>

// GTSAM
#include <gtsam/base/types.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/PriorFactor.h>

// Workspace
#include "graph_msf/core/DynamicDictionaryContainer.h"

namespace graph_msf {

struct Point3CalibrationStateResult {
  gtsam::Expression<gtsam::Point3> expression{gtsam::Point3::Identity()};
  gtsam::Values newOnlineStateValues;
  gtsam::Values newOfflineStateValues;
  std::vector<gtsam::PriorFactor<gtsam::Point3>> newOnlineDynamicPriorFactors;
};

inline void mergePoint3CalibrationStateResult(const Point3CalibrationStateResult& result, gtsam::Values& newOnlineStateValues,
                                              gtsam::Values& newOfflineStateValues,
                                              std::vector<gtsam::PriorFactor<gtsam::Point3>>& newOnlineDynamicPriorFactors) {
  if (!result.newOnlineStateValues.empty()) {
    newOnlineStateValues.insert(result.newOnlineStateValues);
  }
  if (!result.newOfflineStateValues.empty()) {
    newOfflineStateValues.insert(result.newOfflineStateValues);
  }
  newOnlineDynamicPriorFactors.insert(newOnlineDynamicPriorFactors.end(), result.newOnlineDynamicPriorFactors.begin(),
                                      result.newOnlineDynamicPriorFactors.end());
}

template <char CALIBRATION_CHAR>
Point3CalibrationStateResult ensurePoint3ExtrinsicCalibrationState(DynamicDictionaryContainer& gtsamDynamicExpressionKeys,
                                                                   const std::string& sensorFrameName,
                                                                   const std::string& sensorFrameCorrectedName, const double timeK,
                                                                   const Eigen::Matrix<double, 6, 1>& initialCalibrationPriorSigmas) {
  Point3CalibrationStateResult result;

  // Translation-only calibration states are still tracked in the Pose3 dictionary because the rest of the dynamic-transform
  // lifecycle, fixed-lag reactivation, covariance export, and TF publishing all iterate that shared bookkeeping structure.
  std::lock_guard<std::mutex> modifyGraphKeysLock(gtsamDynamicExpressionKeys.get<gtsam::Pose3>().mutex());

  const gtsam::Point3 initialGuess = gtsam::Point3::Identity();
  const gtsam::Pose3 initialGuessAsPose3(gtsam::Rot3::Identity(), initialGuess);
  bool newGraphKeyAddedFlag = false;
  const DynamicVariableType variableType = DynamicVariableType::Global(timeK);

  auto graphKey = gtsamDynamicExpressionKeys.get<gtsam::Pose3>().getTransformationKey<CALIBRATION_CHAR>(
      newGraphKeyAddedFlag, sensorFrameName, sensorFrameCorrectedName, timeK, initialGuessAsPose3, variableType);

  if (!newGraphKeyAddedFlag && !graphKey.isVariableActive() && graphKey.getNumberStepsOptimized() == 0) {
    DynamicFactorGraphStateKey<gtsam::Pose3> removedKey;
    const bool removedKeyFlag =
        gtsamDynamicExpressionKeys.get<gtsam::Pose3>().removeTransform(sensorFrameName, sensorFrameCorrectedName, removedKey);
    if (!removedKeyFlag) {
      throw std::logic_error("Failed to remove inactive, unoptimized translation-only calibration state before recreating it.");
    }

    graphKey = gtsamDynamicExpressionKeys.get<gtsam::Pose3>().getTransformationKey<CALIBRATION_CHAR>(
        newGraphKeyAddedFlag, sensorFrameName, sensorFrameCorrectedName, timeK, initialGuessAsPose3, variableType);
    if (!newGraphKeyAddedFlag) {
      throw std::logic_error(
          "Expected a new translation-only calibration state to be created after removing an unoptimized inactive state.");
    }
  }

  auto& storedGraphKey =
      gtsamDynamicExpressionKeys.get<gtsam::Pose3>().lv_T_frame1_frame2(sensorFrameName, sensorFrameCorrectedName);

  result.expression = gtsam::Expression<gtsam::Point3>(storedGraphKey.key());

  if (newGraphKeyAddedFlag) {
    result.newOnlineStateValues.insert(storedGraphKey.key(), initialGuess);
    result.newOfflineStateValues.insert(storedGraphKey.key(), initialGuess);

    const Eigen::Matrix<double, 3, 1> priorSigmas =
        initialCalibrationPriorSigmas.tail<3>().cwiseMax(Eigen::Matrix<double, 3, 1>::Constant(1e-9));
    result.newOnlineDynamicPriorFactors.emplace_back(storedGraphKey.key(), initialGuess,
                                                     gtsam::noiseModel::Diagonal::Sigmas(priorSigmas));
    assert(storedGraphKey.isVariableActive());
    return result;
  }

  if (!storedGraphKey.isVariableActive() && storedGraphKey.getNumberStepsOptimized() > 0) {
    const gtsam::Point3 priorBelief = storedGraphKey.getTransformationAfterOptimization().translation();
    gtsam::Matrix33 priorCovariance = storedGraphKey.getCovarianceAfterOptimization().block<3, 3>(3, 3);

    if (priorCovariance.diagonal().cwiseAbs().maxCoeff() <= 1e-12) {
      const Eigen::Matrix<double, 3, 1> priorSigmas =
          initialCalibrationPriorSigmas.tail<3>().cwiseMax(Eigen::Matrix<double, 3, 1>::Constant(1e-9));
      result.newOnlineDynamicPriorFactors.emplace_back(storedGraphKey.key(), priorBelief,
                                                       gtsam::noiseModel::Diagonal::Sigmas(priorSigmas));
    } else {
      priorCovariance.diagonal() = priorCovariance.diagonal().array().max(1e-12).matrix();
      result.newOnlineDynamicPriorFactors.emplace_back(storedGraphKey.key(), priorBelief,
                                                       gtsam::noiseModel::Gaussian::Covariance(priorCovariance));
    }

    result.newOnlineStateValues.insert(storedGraphKey.key(), priorBelief);
    storedGraphKey.activateVariable();
    storedGraphKey.resetNumberStepsOptimized();
  }

  return result;
}

}  // namespace graph_msf

#endif  // GMSF_POINT3_CALIBRATION_STATE_H
