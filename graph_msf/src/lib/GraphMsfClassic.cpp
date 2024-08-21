/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "graph_msf/interface/GraphMsfClassic.h"

// Workspace
#include "graph_msf/core/GraphManager.hpp"

// Unary Factors
#include "graph_msf/factors/non_expression/unaryYawFactor.h"

namespace graph_msf {

// Constructor
GraphMsfClassic::GraphMsfClassic() {
  REGULAR_COUT << GREEN_START << " GraphMsfClassic-Constructor called." << COLOR_END << std::endl;
}

// Unary ----------------------------
void GraphMsfClassic::addUnaryYawAbsoluteMeasurement(const UnaryMeasurementXDAbsolute<double, 1>& yaw_W_frame) {
  // Only take actions if graph has been initialized
  if (!initedGraphFlag_) {
    return;
  }

  // Check for covariance violation
  bool covarianceViolatedFlag =
      isCovarianceViolated_<1>(yaw_W_frame.unaryMeasurementNoiseDensity(), yaw_W_frame.covarianceViolationThreshold());

  // Transform yaw to imu frame
  gtsam::Rot3 yawR_W_frame = gtsam::Rot3::Yaw(yaw_W_frame.unaryMeasurement());
  gtsam::Rot3 yawR_W_I =
      yawR_W_frame *
      gtsam::Rot3(staticTransformsPtr_->rv_T_frame1_frame2(yaw_W_frame.sensorFrameName(), staticTransformsPtr_->getImuFrame()).rotation());

  // Add factor
  if (!covarianceViolatedFlag) {
    graphMgrPtr_->addUnaryFactorInImuFrame<double, 1, YawFactor, gtsam::symbol_shorthand::X>(
        yawR_W_I.yaw(), yaw_W_frame.unaryMeasurementNoiseDensity(), yaw_W_frame.timeK());
    {
      // Mutex for optimizeGraph Flag
      const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
      optimizeGraphFlag_ = true;
    }
  }
}

// Binary ----------------------------
// Binary Measurements
void GraphMsfClassic::addBinaryPose3Measurement(const BinaryMeasurementXD<Eigen::Isometry3d, 6>& deltaMeasurement) {
  // Valid measurement received
  if (!validFirstMeasurementReceivedFlag_) {
    validFirstMeasurementReceivedFlag_ = true;
  }

  // Only take actions if graph has been initialized
  if (!initedGraphFlag_) {
    return;
  }

  Eigen::Isometry3d T_fkm1_fk = deltaMeasurement.deltaMeasurement();

  // Check frame of measuremnts
  if (deltaMeasurement.measurementName() != staticTransformsPtr_->getImuFrame()) {
    T_fkm1_fk = staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(), deltaMeasurement.sensorFrameName()) *
                T_fkm1_fk *
                staticTransformsPtr_->rv_T_frame1_frame2(deltaMeasurement.sensorFrameName(), staticTransformsPtr_->getImuFrame());
  }

  static_cast<void>(graphMgrPtr_->addPoseBetweenFactor(
      gtsam::Pose3(T_fkm1_fk.matrix()), deltaMeasurement.measurementNoiseDensity(), deltaMeasurement.timeKm1(), deltaMeasurement.timeK(),
      deltaMeasurement.measurementRate(), deltaMeasurement.robustNormEnum(), deltaMeasurement.robustNormConstant()));

  // Optimize
  {
    // Mutex for optimizeGraph Flag
    const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
    optimizeGraphFlag_ = true;
  }
}

}  // namespace graph_msf
