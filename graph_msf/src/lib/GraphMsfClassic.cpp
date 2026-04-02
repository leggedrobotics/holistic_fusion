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
#include "graph_msf/factors/non_expression/unaryPitchFactor.h"
#include "graph_msf/factors/non_expression/unaryRollFactor.h"
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

  // Add factor
  if (!covarianceViolatedFlag) {
    const std::string measurementName = yaw_W_frame.measurementName();
    const double measurementTime = yaw_W_frame.timeK();
    runOrDeferUnaryMeasurement_(measurementName, measurementTime, [this, measurement = yaw_W_frame]() -> UnaryAddOutcome {
      gtsam::Rot3 yawR_W_frame = gtsam::Rot3::Yaw(measurement.unaryMeasurement());
      gtsam::Rot3 yawR_W_I =
          yawR_W_frame *
          gtsam::Rot3(
              staticTransformsPtr_->rv_T_frame1_frame2(measurement.sensorFrameName(), staticTransformsPtr_->getImuFrame()).rotation());

      return graphMgrPtr_->addUnaryFactorInImuFrame<double, 1, YawFactor, gtsam::symbol_shorthand::X>(
          yawR_W_I.yaw(), measurement.unaryMeasurementNoiseDensity(), measurement.timeK());
    });
  } else {
    std::cout << YELLOW_START << "GMsf-Classic" << RED_START << " Covariance violation detected in yaw unary measurement at time "
              << yaw_W_frame.timeK() << ". Measurement not added to graph." << COLOR_END << std::endl;
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

  // Add factor
  if (!covarianceViolatedFlag) {
    const std::string measurementName = roll_W_frame.measurementName();
    const double measurementTime = roll_W_frame.timeK();
    runOrDeferUnaryMeasurement_(measurementName, measurementTime, [this, measurement = roll_W_frame]() -> UnaryAddOutcome {
      gtsam::Rot3 rollR_W_frame = gtsam::Rot3::Roll(measurement.unaryMeasurement());
      gtsam::Rot3 rollR_W_I =
          rollR_W_frame *
          gtsam::Rot3(
              staticTransformsPtr_->rv_T_frame1_frame2(measurement.sensorFrameName(), staticTransformsPtr_->getImuFrame()).rotation());

      return graphMgrPtr_->addUnaryFactorInImuFrame<double, 1, RollFactor, gtsam::symbol_shorthand::X>(
          rollR_W_I.roll(), measurement.unaryMeasurementNoiseDensity(), measurement.timeK());
    });
  } else {
    std::cout << YELLOW_START << "GMsf-Classic" << RED_START << " Covariance violation detected in roll unary measurement at time "
              << roll_W_frame.timeK() << ". Measurement not added to graph." << COLOR_END << std::endl;
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

  // Add factor
  if (!covarianceViolatedFlag) {
    const std::string measurementName = pitch_W_frame.measurementName();
    const double measurementTime = pitch_W_frame.timeK();
    runOrDeferUnaryMeasurement_(measurementName, measurementTime, [this, measurement = pitch_W_frame]() -> UnaryAddOutcome {
      gtsam::Rot3 pitchR_W_frame = gtsam::Rot3::Pitch(measurement.unaryMeasurement());
      gtsam::Rot3 pitchR_W_I =
          pitchR_W_frame *
          gtsam::Rot3(
              staticTransformsPtr_->rv_T_frame1_frame2(measurement.sensorFrameName(), staticTransformsPtr_->getImuFrame()).rotation());

      return graphMgrPtr_->addUnaryFactorInImuFrame<double, 1, PitchFactor, gtsam::symbol_shorthand::X>(
          pitchR_W_I.pitch(), measurement.unaryMeasurementNoiseDensity(), measurement.timeK());
    });
  } else {
    std::cout << YELLOW_START << "GMsf-Classic" << RED_START << " Covariance violation detected in pitch unary measurement at time "
              << pitch_W_frame.timeK() << ". Measurement not added to graph." << COLOR_END << std::endl;
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

  // Transform non-IMU binary measurements into the IMU frame before adding the factor.
  if (deltaMeasurement.sensorFrameName() != staticTransformsPtr_->getImuFrame()) {
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
