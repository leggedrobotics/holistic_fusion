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

// Function for composing rigid body transformations with GTSAM expression factors
inline gtsam::Pose3_ composeRigidTransformations(const gtsam::Pose3_& T_frame1_frame2, const gtsam::Pose3_& T_frame2_frame3) {
  return gtsam::Pose3_(T_frame1_frame2, &gtsam::Pose3::transformPoseFrom, T_frame2_frame3);
}

/**
 * UnaryExpression is a base class for unary expressions.
 * Unary expressions are used to represent unary factors in the factor graph.
 * It optionally supports the HolisticGraph paradigm to align different measurements
 * It optionally supports extrinsic calibration of the sensor
 **/

template <class GTSAM_MEASUREMENT_TYPE>
class GmsfUnaryExpression {
 public:
  // Constructor
  GmsfUnaryExpression(const std::shared_ptr<UnaryMeasurement>& baseUnaryMeasurementPtr, const std::string& worldFrameName,
                      const Eigen::Isometry3d& T_I_sensorFrame)
      : baseUnaryMeasurementPtr_(baseUnaryMeasurementPtr), worldFrameName_(worldFrameName), T_I_sensorFrameInit_(T_I_sensorFrame) {}

  // Destructor
  virtual ~GmsfUnaryExpression() = default;

  // Interface with four cases (non-exclustive, but has to be correct order!):
  // i) Generate Expression for Basic IMU State in World Frame at Key
  virtual void generateExpressionForBasicImuStateInWorldFrameAtKey(const gtsam::Key& closestGeneralKey) = 0;

  // ii) holistically optimize over fixed frames
  virtual void transformStateFromWorldToFixedFrame(TransformsExpressionKeys& transformsExpressionKeys,
                                                   const gtsam::NavState& W_currentPropagatedState,
                                                   const bool centerMeasurementsAtRobotPositionBeforeAlignment) = 0;

  // iii) transform measurement to core imu frame
  virtual void transformStateToSensorFrame() = 0;

  // iv) extrinsic calibration
  virtual void addExtrinsicCalibrationCorrection(TransformsExpressionKeys& transformsExpressionKeys) = 0;

  // Noise as GTSAM Datatype
  virtual const gtsam::Vector getNoiseDensity() const = 0;

  // Return Measurement as GTSAM Datatype
  virtual const GTSAM_MEASUREMENT_TYPE getMeasurement() const = 0;

  // Return Expression
  virtual const gtsam::Expression<GTSAM_MEASUREMENT_TYPE> getExpression() const = 0;

  // Time
  [[nodiscard]] double getTimestamp() const { return baseUnaryMeasurementPtr_->timeK(); }

  // New Values
  [[nodiscard]] const gtsam::Values& getNewStateValues() const { return newStateValues_; }

  // New Prior Factors
  const std::vector<gtsam::PriorFactor<gtsam::Pose3>>& getNewPriorPoseFactors() const { return newPriorPoseFactors_; }

  // Accessors
  const std::shared_ptr<UnaryMeasurement>& getUnaryMeasurementPtr() const { return baseUnaryMeasurementPtr_; }

  const Eigen::Isometry3d& getT_I_sensorFrameInit() const { return T_I_sensorFrameInit_; }

 protected:
  // Main Measurement Pointer
  const std::shared_ptr<UnaryMeasurement> baseUnaryMeasurementPtr_;

  // Frame Name References
  const std::string worldFrameName_;

  // IMU to Sensor Frame
  Eigen::Isometry3d T_I_sensorFrameInit_;

  // Containers
  gtsam::Values newStateValues_;
  std::vector<gtsam::PriorFactor<gtsam::Pose3>> newPriorPoseFactors_;
};

}  // namespace graph_msf

#endif  // GMSFUNARYEXPRESSION_H
