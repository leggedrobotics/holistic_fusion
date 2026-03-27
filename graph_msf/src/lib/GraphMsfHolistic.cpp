/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "graph_msf/interface/GraphMsfHolistic.h"

// Workspace
#include "graph_msf/core/GraphManager.h"
#include "graph_msf/interface/constants.h"

// Unary Expression Factors
/// Absolute
#include "graph_msf/factors/gmsf_expression/GmsfUnaryExpressionAbsolutePose3.h"
#include "graph_msf/factors/gmsf_expression/GmsfUnaryExpressionAbsolutePosition3.h"
/// Local
#include "graph_msf/factors/gmsf_expression/GmsfUnaryExpressionLocalVelocity3.h"

// Landmark Expression Factors
#include "graph_msf/factors/gmsf_expression/GmsfUnaryExpressionLandmarkPosition3.h"

// Binary Expression Factors
// TODO: add binary factors

namespace graph_msf {

// Constructor
GraphMsfHolistic::GraphMsfHolistic() {
  REGULAR_COUT << GREEN_START << " GraphMsfHolistic-Constructor called." << COLOR_END << std::endl;
}

// Unary Measurements: In reference frame --> systematic drift ---------------------------------------------------------

// Pose3
void GraphMsfHolistic::addUnaryPose3AbsoluteMeasurement(const UnaryMeasurementXDAbsolute<Eigen::Isometry3d, 6>& R_T_R_S,
                                                        const bool addToOnlineSmootherFlag) {
  // Valid measurement received
  if (!validFirstMeasurementReceivedFlag_) {
    validFirstMeasurementReceivedFlag_ = true;
  }

  // Only take actions if graph has been initialized
  if (!initedGraphFlag_) {  // Graph not yet initialized
    return;
  } else {  // Graph initialized
    // Check for covariance violation
    bool covarianceViolatedFlag = isCovarianceViolated_<6>(R_T_R_S.unaryMeasurementNoiseDensity(), R_T_R_S.covarianceViolationThreshold());
    if (checkAndPrintCovarianceViolation_(R_T_R_S.measurementName(), covarianceViolatedFlag)) {
      return;
    }

    const std::string measurementName = R_T_R_S.measurementName();
    const double measurementTime = R_T_R_S.timeK();
    runOrDeferUnaryMeasurement_(
        measurementName, measurementTime,
        [this, measurement = R_T_R_S, addToOnlineSmootherFlag]() -> UnaryAddOutcome {
          auto gmsfUnaryExpressionPose3Ptr = std::make_shared<GmsfUnaryExpressionAbsolutePose3>(
              std::make_shared<UnaryMeasurementXDAbsolute<Eigen::Isometry3d, 6>>(measurement), staticTransformsPtr_->getImuFrame(),
              staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(), measurement.sensorFrameName()),
              graphConfigPtr_->createReferenceAlignmentKeyframeEveryNSeconds_);

          return graphMgrPtr_->addUnaryGmsfExpressionFactor<GmsfUnaryExpressionAbsolutePose3>(gmsfUnaryExpressionPose3Ptr,
                                                                                               addToOnlineSmootherFlag);
        });
  }
}

// Position3
void GraphMsfHolistic::addUnaryPosition3AbsoluteMeasurement(
    UnaryMeasurementXDAbsolute<Eigen::Vector3d, 3>& fixedFrame_t_fixedFrame_sensorFrame) {
  // Valid measurement received
  if (!validFirstMeasurementReceivedFlag_) {
    validFirstMeasurementReceivedFlag_ = true;
  }

  // Only take actions if graph has been initialized
  if (!initedGraphFlag_) {  // Case 1: Graph not yet initialized
    return;
  } else {  // Case 2: Graph Initialized
    // Check for covariance violation
    bool covarianceViolatedFlag = isCovarianceViolated_<3>(fixedFrame_t_fixedFrame_sensorFrame.unaryMeasurementNoiseDensity(),
                                                           fixedFrame_t_fixedFrame_sensorFrame.covarianceViolationThreshold());
    if (checkAndPrintCovarianceViolation_(fixedFrame_t_fixedFrame_sensorFrame.measurementName(), covarianceViolatedFlag)) {
      return;
    }

    const std::string measurementName = fixedFrame_t_fixedFrame_sensorFrame.measurementName();
    const double measurementTime = fixedFrame_t_fixedFrame_sensorFrame.timeK();
    runOrDeferUnaryMeasurement_(
        measurementName, measurementTime,
        [this, measurement = fixedFrame_t_fixedFrame_sensorFrame]() -> UnaryAddOutcome {
          auto gmsfUnaryExpressionPosition3Ptr = std::make_shared<GmsfUnaryExpressionAbsolutePosition3>(
              std::make_shared<UnaryMeasurementXDAbsolute<Eigen::Vector3d, 3>>(measurement), staticTransformsPtr_->getImuFrame(),
              staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(), measurement.sensorFrameName()),
              graphConfigPtr_->createReferenceAlignmentKeyframeEveryNSeconds_);

          return graphMgrPtr_->addUnaryGmsfExpressionFactor<GmsfUnaryExpressionAbsolutePosition3>(
              gmsfUnaryExpressionPosition3Ptr);
        });
  }
}

// Velocity3 in Fixed Frame
void GraphMsfHolistic::addUnaryVelocity3AbsoluteMeasurement(UnaryMeasurementXDAbsolute<Eigen::Vector3d, 3>& F_v_F_S) {
  throw std::runtime_error("Velocity measurements in fixed frame are not yet supported.");
}

// Local Measurements: Fully Local ---------------------------------------------------------
// Velocity3 in Body Frame
void GraphMsfHolistic::addUnaryVelocity3LocalMeasurement(UnaryMeasurementXD<Eigen::Vector3d, 3>& S_v_F_S) {
  // Valid measurement received
  if (!validFirstMeasurementReceivedFlag_) {
    validFirstMeasurementReceivedFlag_ = true;
  }

  // Only take actions if graph has been initialized
  if (!initedGraphFlag_) {  // Case 1: Graph not yet initialized
    return;
  } else {  // Case 2: Graph Initialized
    // Check for covariance violation
    bool covarianceViolatedFlag = isCovarianceViolated_<3>(S_v_F_S.unaryMeasurementNoiseDensity(), S_v_F_S.covarianceViolationThreshold());
    if (checkAndPrintCovarianceViolation_(S_v_F_S.measurementName(), covarianceViolatedFlag)) {
      return;
    }

    const std::string measurementName = S_v_F_S.measurementName();
    const double measurementTime = S_v_F_S.timeK();
    runOrDeferUnaryMeasurement_(
        measurementName, measurementTime,
        [this, measurement = S_v_F_S]() -> UnaryAddOutcome {
          auto gmsfUnaryExpressionVelocity3SensorFramePtr = std::make_shared<GmsfUnaryExpressionLocalVelocity3>(
              std::make_shared<UnaryMeasurementXD<Eigen::Vector3d, 3>>(measurement), staticTransformsPtr_->getImuFrame(),
              staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(), measurement.sensorFrameName()),
              coreImuBufferPtr_);

          return graphMgrPtr_->addUnaryGmsfExpressionFactor<GmsfUnaryExpressionLocalVelocity3>(
              gmsfUnaryExpressionVelocity3SensorFramePtr);
        });
  }
}

// Landmark Measurements: No systematic drift ------------------------------------------------------
// Position3
void GraphMsfHolistic::addUnaryPosition3LandmarkMeasurement(UnaryMeasurementXDLandmark<Eigen::Vector3d, 3>& S_t_S_L,
                                                            const int landmarkCreationCounter) {
  // Valid measurement received
  if (!validFirstMeasurementReceivedFlag_) {
    validFirstMeasurementReceivedFlag_ = true;
  }

  // Only take actions if graph has been initialized
  if (!initedGraphFlag_) {  // Case 1: Graph not yet initialized
    return;
  } else {  // Case 2: Graph Initialized
    // Check for covariance violation
    bool covarianceViolatedFlag = isCovarianceViolated_<3>(S_t_S_L.unaryMeasurementNoiseDensity(), S_t_S_L.covarianceViolationThreshold());
    if (checkAndPrintCovarianceViolation_(S_t_S_L.measurementName(), covarianceViolatedFlag)) {
      return;
    }

    // TODO: Change this to more explicit handling of counter
    // S_t_S_L.setMeasurementName(S_t_S_L.measurementName() + "_" + std::to_string(landmarkCreationCounter));

    const std::string measurementName = S_t_S_L.measurementName();
    const double measurementTime = S_t_S_L.timeK();
    runOrDeferUnaryMeasurement_(
        measurementName, measurementTime,
        [this, measurement = S_t_S_L, landmarkCreationCounter]() -> UnaryAddOutcome {
          auto gmsfUnaryExpressionPosition3LandmarkPtr = std::make_shared<GmsfUnaryExpressionLandmarkPosition3>(
              std::make_shared<UnaryMeasurementXDLandmark<Eigen::Vector3d, 3>>(measurement), staticTransformsPtr_->getImuFrame(),
              staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(), measurement.sensorFrameName()),
              landmarkCreationCounter);

          return graphMgrPtr_->addUnaryGmsfExpressionFactor<GmsfUnaryExpressionLandmarkPosition3>(
              gmsfUnaryExpressionPosition3LandmarkPtr);
        });
  }
}

// Bearing3
void GraphMsfHolistic::addUnaryBearing3LandmarkMeasurement(UnaryMeasurementXDLandmark<Eigen::Vector3d, 3>& S_bearing_S_L) {
  throw std::runtime_error("Landmark measurements are not yet supported for the holistic MSF.");
}

// Binary Measurements: Purely relative --------------------------------------------------------

}  // namespace graph_msf
