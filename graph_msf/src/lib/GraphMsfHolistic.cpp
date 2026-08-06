/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// C++
#include <algorithm>

// Implementation
#include "graph_msf/interface/GraphMsfHolistic.h"

// Workspace
#include "graph_msf/core/GraphManager.h"
#include "graph_msf/interface/constants.h"

// Unary Expression Factors
/// Absolute
#include "graph_msf/factors/gmsf_expression/GmsfUnaryExpressionAbsolutePose3.h"
#include "graph_msf/factors/gmsf_expression/GmsfUnaryExpressionAbsolutePosition3.h"
#include "graph_msf/factors/gmsf_expression/GmsfUnaryExpressionAbsoluteVelocity3.h"
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

    // Create GMSF expression
    auto gmsfUnaryExpressionPose3Ptr = std::make_shared<GmsfUnaryExpressionAbsolutePose3>(
        std::make_shared<UnaryMeasurementXDAbsolute<Eigen::Isometry3d, 6>>(R_T_R_S), staticTransformsPtr_->getImuFrame(),
        staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(), R_T_R_S.sensorFrameName()),
        graphConfigPtr_->createReferenceAlignmentKeyframeEveryNSeconds_);

    // Add factor to graph
    graphMgrPtr_->addUnaryGmsfExpressionFactor<GmsfUnaryExpressionAbsolutePose3>(gmsfUnaryExpressionPose3Ptr, addToOnlineSmootherFlag);

    // Optimize ---------------------------------------------------------------
    {
      // Mutex for optimizeGraph Flag
      const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
      optimizeGraphFlag_ = true;
    }
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

    // Create GMSF expression
    auto gmsfUnaryExpressionPosition3Ptr = std::make_shared<GmsfUnaryExpressionAbsolutePosition3>(
        std::make_shared<UnaryMeasurementXDAbsolute<Eigen::Vector3d, 3>>(fixedFrame_t_fixedFrame_sensorFrame),
        staticTransformsPtr_->getImuFrame(),
        staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(),
                                                 fixedFrame_t_fixedFrame_sensorFrame.sensorFrameName()),
        graphConfigPtr_->createReferenceAlignmentKeyframeEveryNSeconds_);

    // Add factor to graph
    graphMgrPtr_->addUnaryGmsfExpressionFactor<GmsfUnaryExpressionAbsolutePosition3>(gmsfUnaryExpressionPosition3Ptr);

    // Optimize ---------------------------------------------------------------
    {
      // Mutex for optimizeGraph Flag
      const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
      optimizeGraphFlag_ = true;
    }
  }
}

// Velocity3 in Fixed Frame
void GraphMsfHolistic::addUnaryVelocity3AbsoluteMeasurement(UnaryMeasurementXDAbsolute<Eigen::Vector3d, 3>& F_v_F_S) {
  if (!validFirstMeasurementReceivedFlag_) {
    validFirstMeasurementReceivedFlag_ = true;
  }

  if (!initedGraphFlag_) {
    return;
  }

  const bool covarianceViolatedFlag =
      isCovarianceViolated_<3>(F_v_F_S.unaryMeasurementNoiseDensity(), F_v_F_S.covarianceViolationThreshold());
  if (checkAndPrintCovarianceViolation_(F_v_F_S.measurementName(), covarianceViolatedFlag)) {
    return;
  }

  double imuTimestamp = 0.0;
  ImuMeasurement imuMeasurement;
  const double maxImuSearchDeviation = std::max(graphConfigPtr_->maxSearchDeviation_, 1.5 / graphConfigPtr_->imuRate_);
  if (!coreImuBufferPtr_->getClosestImuMeasurement(imuTimestamp, imuMeasurement, maxImuSearchDeviation, F_v_F_S.timeK())) {
    REGULAR_COUT << RED_START << " No IMU measurement close enough to absolute velocity '" << F_v_F_S.measurementName() << "' at "
                 << std::setprecision(14) << F_v_F_S.timeK() << "; not adding the factor." << COLOR_END << std::endl;
    return;
  }

  const auto measurementPtr = std::make_shared<UnaryMeasurementXDAbsolute<Eigen::Vector3d, 3>>(F_v_F_S);
  const Eigen::Isometry3d T_I_sensorFrame =
      staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(), F_v_F_S.sensorFrameName());

  auto horizontalExpressionPtr = std::make_shared<GmsfUnaryExpressionAbsoluteVelocityXY>(
      measurementPtr, staticTransformsPtr_->getImuFrame(), T_I_sensorFrame, imuMeasurement.angularVelocity);
  auto verticalExpressionPtr = std::make_shared<GmsfUnaryExpressionAbsoluteVelocityZ>(
      measurementPtr, staticTransformsPtr_->getImuFrame(), T_I_sensorFrame, imuMeasurement.angularVelocity);
  const bool factorsAdded = graphMgrPtr_->addUnaryGmsfExpressionFactorPair(horizontalExpressionPtr, verticalExpressionPtr);
  if (!factorsAdded) {
    return;
  }

  {
    const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
    optimizeGraphFlag_ = true;
  }
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

    // Create GMSF expression
    auto gmsfUnaryExpressionVelocity3SensorFramePtr = std::make_shared<GmsfUnaryExpressionLocalVelocity3>(
        std::make_shared<UnaryMeasurementXD<Eigen::Vector3d, 3>>(S_v_F_S), staticTransformsPtr_->getImuFrame(),
        staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(), S_v_F_S.sensorFrameName()), coreImuBufferPtr_);

    // Add factor to graph
    graphMgrPtr_->addUnaryGmsfExpressionFactor<GmsfUnaryExpressionLocalVelocity3>(gmsfUnaryExpressionVelocity3SensorFramePtr);

    // Optimize ---------------------------------------------------------------
    {
      // Mutex for optimizeGraph Flag
      const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
      optimizeGraphFlag_ = true;
    }
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

    // Create GMSF expression
    auto gmsfUnaryExpressionPosition3LandmarkPtr = std::make_shared<GmsfUnaryExpressionLandmarkPosition3>(
        std::make_shared<UnaryMeasurementXDLandmark<Eigen::Vector3d, 3>>(S_t_S_L), staticTransformsPtr_->getImuFrame(),
        staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(), S_t_S_L.sensorFrameName()), landmarkCreationCounter);

    // Add factor to graph
    graphMgrPtr_->addUnaryGmsfExpressionFactor<GmsfUnaryExpressionLandmarkPosition3>(gmsfUnaryExpressionPosition3LandmarkPtr);

    // Optimize ---------------------------------------------------------------
    {
      // Mutex for optimizeGraph Flag
      const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
      optimizeGraphFlag_ = true;
    }
  }
}

// Bearing3
void GraphMsfHolistic::addUnaryBearing3LandmarkMeasurement(UnaryMeasurementXDLandmark<Eigen::Vector3d, 3>& S_bearing_S_L) {
  throw std::runtime_error("Landmark measurements are not yet supported for the holistic MSF.");
}

// Binary Measurements: Purely relative --------------------------------------------------------

}  // namespace graph_msf
