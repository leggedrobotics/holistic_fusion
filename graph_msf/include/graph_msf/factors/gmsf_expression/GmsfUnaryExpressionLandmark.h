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
#include "graph_msf/measurements/UnaryMeasurement.h"

namespace graph_msf {

template <class GTSAM_MEASUREMENT_TYPE>
class GmsfUnaryExpressionLandmark : public GmsfUnaryExpression<GTSAM_MEASUREMENT_TYPE> {
  using Base = GmsfUnaryExpression<GTSAM_MEASUREMENT_TYPE>;

 public:
  // Constructor
  GmsfUnaryExpressionLandmark(const std::shared_ptr<UnaryMeasurement>& baseUnaryAbsoluteMeasurementPtr, const std::string& worldFrameName,
                              const Eigen::Isometry3d& T_I_sensorFrame)
      : GmsfUnaryExpression<GTSAM_MEASUREMENT_TYPE>(baseUnaryAbsoluteMeasurementPtr, worldFrameName, T_I_sensorFrame),
        baseUnaryAbsoluteMeasurementPtr_(baseUnaryAbsoluteMeasurementPtr) {}

  // Destructor
  virtual ~GmsfUnaryExpressionLandmark() = default;

  // Virtual Methods
  // ii.A) Holistically Optimize over Fixed Frames
  void transformStateFromWorldToFixedFrame(TransformsExpressionKeys& transformsExpressionKeys,
                                           const gtsam::NavState& W_currentPropagatedState,
                                           const bool centerMeasurementsAtRobotPositionBeforeAlignment) override final {
    // Do nothing as this velocity measurement is expressed in the sensor frame
    std::runtime_error(
        "GmsfUnaryExpressionLandmark: transformStateFromWorldToFixedFrame not implemented, as it is a landmark measurement.");
  }

  // ii.B) Adding Landmark State in Dynamic Memory
  void convertRobotAndLandmarkStatesToMeasurement(TransformsExpressionKeys& transformsExpressionKeys) override = 0;

  // Accessors
  [[nodiscard]] const auto& getBaseUnaryAbsoluteMeasurementPtr() const { return baseUnaryAbsoluteMeasurementPtr_; }

 protected:
  // Main Measurement Pointer
  const std::shared_ptr<UnaryMeasurement> baseUnaryAbsoluteMeasurementPtr_;
};

}  // namespace graph_msf

#endif  // GMSF_UNARY_EXPRESSION_LANDMARK_H
