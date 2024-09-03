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

template <class GTSAM_MEASUREMENT_TYPE>
class GmsfUnaryExpressionAbsolut : public GmsfUnaryExpression<GTSAM_MEASUREMENT_TYPE, UnaryExpressionType::Absolute> {

 public:
  // Constructor
  GmsfUnaryExpressionAbsolut(const std::shared_ptr<UnaryMeasurementAbsolute>& gmsfUnaryAbsoluteMeasurementPtr,
                             const std::string& worldFrameName, const std::string& imuFrameName, const Eigen::Isometry3d& T_I_sensorFrame)
      : GmsfUnaryExpression<GTSAM_MEASUREMENT_TYPE, UnaryExpressionType::Absolute>(gmsfUnaryAbsoluteMeasurementPtr, worldFrameName, imuFrameName, T_I_sensorFrame) {}

  // Destructor
  ~GmsfUnaryExpressionAbsolut() = default;

 protected:
  // Virtual Methods
  // ii).A Holistically Optimize over Fixed Frames
  void transformStateFromWorldToFixedFrame(TransformsExpressionKeys<gtsam::Pose3>& transformsExpressionKeys,
                                           const gtsam::NavState& W_currentPropagatedState,
                                           const bool centerMeasurementsAtRobotPositionBeforeAlignment) override = 0;

  // ii).B Adding Landmark State in Dynamic Memory
  void convertRobotAndLandmarkStatesToMeasurement(TransformsExpressionKeys<gtsam::Pose3>& transformsExpressionKeys,
                                                  const gtsam::NavState& W_currentPropagatedState) final {
    // Raise logic error, as it is not a landmark measurement
    throw std::logic_error("GmsfUnaryExpressionAbsolut: convertRobotAndLandmarkStatesToMeasurement() is not implemented.");
  }
};

}  // namespace graph_msf

#endif  // GMSF_UNARY_EXPRESSION_ABSOLUT_H
