/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GMSFUNARYEXPRESSION_H
#define GMSFUNARYEXPRESSION_H

// GTSAM
#include <gtsam/base/types.h>

// Workspace
#include "graph_msf/core/TransformsExpressionKeys.h"
#include "graph_msf/measurements/UnaryMeasurement.h"

namespace graph_msf {

/**
 * UnaryExpression is a base class for unary expressions.
 * Unary expressions are used to represent unary factors in the factor graph.
 * It optionally supports the HolisticGraph paradigm to align different measurements
 * It optionally supports extrinsic calibration of the sensor
 **/

class GmsfUnaryExpression {
 public:
  // Constructor
  GmsfUnaryExpression(const std::shared_ptr<UnaryMeasurement>& unaryMeasurementPtr, const gtsam::Key& closestGeneralKey)
      : unaryMeasurementPtr_(unaryMeasurementPtr), closestGeneralKey_(closestGeneralKey) {}

  // Destructor
  virtual ~GmsfUnaryExpression() = default;

  // Interface with three cases (non-exclustive):
  // i) holistically optimize over fixed frames
  virtual void transformStateFromWorldToFixedFrame(TransformsExpressionKeys& transformsExpressionKeys) = 0;

  // ii) extrinsic calibration
  virtual void addExtrinsicCalibrationCorrection(TransformsExpressionKeys& transformsExpressionKeys) = 0;

  // iii) transform measurement to core imu frame
  virtual void transformMeasurementToCoreImuFrame(TransformsExpressionKeys& transformsExpressionKeys) = 0;

  // Accessors
  const std::shared_ptr<UnaryMeasurement>& getUnaryMeasurementPtr() const { return unaryMeasurementPtr_; }
  const gtsam::Key& getClosestGeneralKey() const { return closestGeneralKey_; }

 protected:
  // Main Measurement Pointer
  const std::shared_ptr<UnaryMeasurement> unaryMeasurementPtr_;

  // Closest Key of main state (only dependant on timestamp) --> has to be mapped to correct state type key
  const gtsam::Key closestGeneralKey_;
};

}  // namespace graph_msf

#endif  // GMSFUNARYEXPRESSION_H
