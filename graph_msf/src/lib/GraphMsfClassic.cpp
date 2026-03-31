/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// CPP
#include <stdexcept>

// Implementation
#include "graph_msf/interface/GraphMsfClassic.h"

// Workspace
#include "graph_msf/core/GraphManager.h"

// Unary Factors
#include "graph_msf/factors/non_expression/unaryRollFactor.h"
#include "graph_msf/factors/non_expression/unaryPitchFactor.h"
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
  } else {
    std::cout << YELLOW_START << "GMsf-Classic" << RED_START
              << " Covariance violation detected in yaw unary measurement at time " << yaw_W_frame.timeK()
              << ". Measurement not added to graph." << COLOR_END << std::endl;
  }
}

void GraphMsfClassic::addUnaryRollAbsoluteMeasurement(const UnaryMeasurementXDAbsolute<double, 1>& roll_W_frame) {
  // Only take actions if graph has been initialized
  if (!initedGraphFlag_) {
    return;
  }

  // Check for covariance violation
  bool covarianceViolatedFlag =
      isCovarianceViolated_<1>(roll_W_frame.unaryMeasurementNoiseDensity(), roll_W_frame.covarianceViolationThreshold());

  // Transform roll to imu frame
  gtsam::Rot3 rollR_W_frame = gtsam::Rot3::Roll(roll_W_frame.unaryMeasurement());
  gtsam::Rot3 rollR_W_I =
      rollR_W_frame *
      gtsam::Rot3(staticTransformsPtr_->rv_T_frame1_frame2(roll_W_frame.sensorFrameName(), staticTransformsPtr_->getImuFrame()).rotation());

  // Add factor
  if (!covarianceViolatedFlag) {
    graphMgrPtr_->addUnaryFactorInImuFrame<double, 1, RollFactor, gtsam::symbol_shorthand::X>(
        rollR_W_I.roll(), roll_W_frame.unaryMeasurementNoiseDensity(), roll_W_frame.timeK());
    {
      // Mutex for optimizeGraph Flag
      const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
      optimizeGraphFlag_ = true;
    }
  } else {
    std::cout << YELLOW_START << "GMsf-Classic" << RED_START
              << " Covariance violation detected in roll unary measurement at time " << roll_W_frame.timeK()
              << ". Measurement not added to graph." << COLOR_END << std::endl;
  }
}

void GraphMsfClassic::addUnaryPitchAbsoluteMeasurement(const UnaryMeasurementXDAbsolute<double, 1>& pitch_W_frame) {
  // Only take actions if graph has been initialized
  if (!initedGraphFlag_) {
    return;
  }

  // Check for covariance violation
  bool covarianceViolatedFlag =
      isCovarianceViolated_<1>(pitch_W_frame.unaryMeasurementNoiseDensity(), pitch_W_frame.covarianceViolationThreshold());

  // Transform pitch to imu frame
  gtsam::Rot3 pitchR_W_frame = gtsam::Rot3::Pitch(pitch_W_frame.unaryMeasurement());
  gtsam::Rot3 pitchR_W_I =
      pitchR_W_frame *
      gtsam::Rot3(staticTransformsPtr_->rv_T_frame1_frame2(pitch_W_frame.sensorFrameName(), staticTransformsPtr_->getImuFrame()).rotation());

  // Add factor
  if (!covarianceViolatedFlag) {
    graphMgrPtr_->addUnaryFactorInImuFrame<double, 1, PitchFactor, gtsam::symbol_shorthand::X>(
        pitchR_W_I.pitch(), pitch_W_frame.unaryMeasurementNoiseDensity(), pitch_W_frame.timeK());
    {
      // Mutex for optimizeGraph Flag
      const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
      optimizeGraphFlag_ = true;
    }
  } else {
    std::cout << YELLOW_START << "GMsf-Classic" << RED_START
              << " Covariance violation detected in pitch unary measurement at time " << pitch_W_frame.timeK()
              << ". Measurement not added to graph." << COLOR_END << std::endl;
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
