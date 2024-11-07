/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GMSF_UNARY_EXPRESSION_LOCAL_H
#define GMSF_UNARY_EXPRESSION_LOCAL_H

// GTSAM
#include <gtsam/base/types.h>

// Workspace
#include "graph_msf/factors/gmsf_expression/GmsfUnaryExpression.h"
#include "graph_msf/measurements/UnaryMeasurement.h"

namespace graph_msf {

template <class GTSAM_MEASUREMENT_TYPE, char CALIBRATION_CHAR>
class GmsfUnaryExpressionLocal : public GmsfUnaryExpression<GTSAM_MEASUREMENT_TYPE, UnaryExpressionType::Local, CALIBRATION_CHAR> {
 public:
  // Constructor
  GmsfUnaryExpressionLocal(const std::shared_ptr<UnaryMeasurement>& baseUnaryAbsoluteMeasurementPtr,
                           const std::string& imuFrameName, const Eigen::Isometry3d& T_I_sensorFrame)
      : GmsfUnaryExpression<GTSAM_MEASUREMENT_TYPE, UnaryExpressionType::Local, CALIBRATION_CHAR>(
            baseUnaryAbsoluteMeasurementPtr, imuFrameName, T_I_sensorFrame) {}

  // Destructor
  ~GmsfUnaryExpressionLocal() = default;

 protected:
  // Virtual Methods
  // ii.A) Holistically Optimize over Fixed Frames
  void transformImuStateFromWorldToReferenceFrame(DynamicDictionaryContainer& gtsamDynamicExpressionKeys,
                                                  const gtsam::NavState& W_currentPropagatedState,
                                                  const bool centerMeasurementsAtRobotPositionBeforeAlignment) final {
    // Do nothing as this velocity measurement is purely local
    throw std::logic_error("GmsfUnaryExpressionLocal: transformStateFromWorldToFixedFrame not implemented, as it is a local measurement.");
  }

  // ii.B) Adding Landmark State in Dynamic Memory
  void transformLandmarkInWorldToImuFrame(DynamicDictionaryContainer& gtsamDynamicExpressionKeys,
                                          const gtsam::NavState& W_currentPropagatedState) final {
    // Raise runtime error, as it is not a landmark measurement
    throw std::logic_error(
        "GmsfUnaryExpressionLocal: convertRobotAndLandmarkStatesToMeasurement not implemented, as it is not a landmark "
        "measurement.");
  }
};

}  // namespace graph_msf

#endif  // GMSF_UNARY_EXPRESSION_LOCAL_H
