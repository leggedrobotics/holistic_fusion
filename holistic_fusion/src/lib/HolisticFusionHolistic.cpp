/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "holistic_fusion/interface/HolisticFusionHolistic.h"

// Workspace
#include "holistic_fusion/core/GraphManager.h"
#include "holistic_fusion/interface/constants.h"

// Unary Expression Factors
/// Absolute
#include "holistic_fusion/factors/gmsf_expression/GmsfUnaryExpressionAbsoluteHeading.h"
#include "holistic_fusion/factors/gmsf_expression/GmsfUnaryExpressionAbsolutePose3.h"
#include "holistic_fusion/factors/gmsf_expression/GmsfUnaryExpressionAbsolutePosition3.h"
#include "holistic_fusion/factors/gmsf_expression/GmsfUnaryExpressionAbsoluteYaw.h"
/// Local
#include "holistic_fusion/factors/gmsf_expression/GmsfUnaryExpressionLocalVelocity3.h"

// Landmark Expression Factors
#include "holistic_fusion/factors/gmsf_expression/GmsfUnaryExpressionLandmarkPosition3.h"

// Binary Expression Factors
// TODO: add binary factors

namespace holistic_fusion {

// Constructor
HolisticFusionHolistic::HolisticFusionHolistic() {
  REGULAR_COUT << GREEN_START << " HolisticFusionHolistic-Constructor called." << COLOR_END << std::endl;
}

// The alignment keyframe of a fixed frame is only created by a measurement with a position
bool HolisticFusionHolistic::hasAlignmentKeyframeForMeasurementWithoutPosition_(const UnaryMeasurementAbsolute& measurement) const {
  const std::string& worldFrame = measurement.worldFrameName();
  const std::string& fixedFrame = measurement.fixedFrameName();
  if (graphConfigPtr_->optimizeReferenceFramePosesWrtWorldFlag_ && fixedFrame != worldFrame &&
      !graphMgrPtr_->hasReferenceFrameKeyframe(worldFrame, fixedFrame)) {
    REGULAR_COUT << YELLOW_START << " Skipping measurement " << measurement.measurementName() << ": frame " << fixedFrame
                 << " has no alignment keyframe yet. A position or pose measurement of this frame creates it." << COLOR_END << std::endl;
    return false;
  }
  return true;
}

// Unary Measurements: In reference frame --> systematic drift ---------------------------------------------------------

// Pose3
void HolisticFusionHolistic::addUnaryPose3AbsoluteMeasurement(const UnaryMeasurementXDAbsolute<Eigen::Isometry3d, 6>& R_T_R_S,
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
    graphMgrPtr_->addUnaryHolisticFactor<GmsfUnaryExpressionAbsolutePose3>(gmsfUnaryExpressionPose3Ptr, addToOnlineSmootherFlag);

    // Optimize ---------------------------------------------------------------
    {
      // Mutex for optimizeGraph Flag
      const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
      optimizeGraphFlag_ = true;
    }
  }
}

// Position3
void HolisticFusionHolistic::addUnaryPosition3AbsoluteMeasurement(
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
    graphMgrPtr_->addUnaryHolisticFactor<GmsfUnaryExpressionAbsolutePosition3>(gmsfUnaryExpressionPosition3Ptr);

    // Optimize ---------------------------------------------------------------
    {
      // Mutex for optimizeGraph Flag
      const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
      optimizeGraphFlag_ = true;
    }
  }
}

void HolisticFusionHolistic::addUnaryYawAbsoluteMeasurement(const UnaryMeasurementXDAbsolute<double, 1>& fixedFrame_yaw_fixedFrame_sensorFrame) {
  // Valid measurement received
  if (!validFirstMeasurementReceivedFlag_) {
    validFirstMeasurementReceivedFlag_ = true;
  }

  // Only take actions if graph has been initialized
  if (!initedGraphFlag_) {
    return;
  }

  // Check for covariance violation
  bool covarianceViolatedFlag = isCovarianceViolated_<1>(fixedFrame_yaw_fixedFrame_sensorFrame.unaryMeasurementNoiseDensity(),
                                                         fixedFrame_yaw_fixedFrame_sensorFrame.covarianceViolationThreshold());
  if (checkAndPrintCovarianceViolation_(fixedFrame_yaw_fixedFrame_sensorFrame.measurementName(), covarianceViolatedFlag)) {
    return;
  }

  if (!hasAlignmentKeyframeForMeasurementWithoutPosition_(fixedFrame_yaw_fixedFrame_sensorFrame)) {
    return;
  }

  // Create GMSF expression
  auto gmsfUnaryExpressionYawPtr = std::make_shared<GmsfUnaryExpressionAbsoluteYaw>(
      std::make_shared<UnaryMeasurementXDAbsolute<double, 1>>(fixedFrame_yaw_fixedFrame_sensorFrame), staticTransformsPtr_->getImuFrame(),
      staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(),
                                               fixedFrame_yaw_fixedFrame_sensorFrame.sensorFrameName()),
      graphConfigPtr_->createReferenceAlignmentKeyframeEveryNSeconds_);

  // Add factor to graph
  graphMgrPtr_->addUnaryHolisticFactor<GmsfUnaryExpressionAbsoluteYaw>(gmsfUnaryExpressionYawPtr);

  // Optimize ---------------------------------------------------------------
  {
    // Mutex for optimizeGraph Flag
    const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
    optimizeGraphFlag_ = true;
  }
}

void HolisticFusionHolistic::addUnaryHeadingAbsoluteMeasurement(
    const UnaryMeasurementXDAbsolute<Eigen::Matrix3d, 1>& R_M_S) {
  // Valid measurement received
  if (!validFirstMeasurementReceivedFlag_) {
    validFirstMeasurementReceivedFlag_ = true;
  }

  // Only take actions if graph has been initialized
  if (!initedGraphFlag_) {
    return;
  }

  // Check for covariance violation
  bool covarianceViolatedFlag = isCovarianceViolated_<1>(R_M_S.unaryMeasurementNoiseDensity(),
                                                         R_M_S.covarianceViolationThreshold());
  if (checkAndPrintCovarianceViolation_(R_M_S.measurementName(), covarianceViolatedFlag)) {
    return;
  }

  if (!hasAlignmentKeyframeForMeasurementWithoutPosition_(R_M_S)) {
    return;
  }

  // Create GMSF expression
  auto gmsfUnaryExpressionHeadingPtr = std::make_shared<GmsfUnaryExpressionAbsoluteHeading>(
      std::make_shared<UnaryMeasurementXDAbsolute<Eigen::Matrix3d, 1>>(R_M_S),
      staticTransformsPtr_->getImuFrame(),
      staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(), R_M_S.sensorFrameName()),
      graphConfigPtr_->createReferenceAlignmentKeyframeEveryNSeconds_);

  // Add factor to graph
  graphMgrPtr_->addUnaryHolisticFactor<GmsfUnaryExpressionAbsoluteHeading>(gmsfUnaryExpressionHeadingPtr);

  // Optimize ---------------------------------------------------------------
  {
    // Mutex for optimizeGraph Flag
    const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
    optimizeGraphFlag_ = true;
  }
}

// Velocity3 in Fixed Frame
void HolisticFusionHolistic::addUnaryVelocity3AbsoluteMeasurement(UnaryMeasurementXDAbsolute<Eigen::Vector3d, 3>& F_v_F_S) {
  throw std::runtime_error("Velocity measurements in fixed frame are not yet supported.");
}

// Local Measurements: Fully Local ---------------------------------------------------------
// Velocity3 in Body Frame
void HolisticFusionHolistic::addUnaryVelocity3LocalMeasurement(UnaryMeasurementXD<Eigen::Vector3d, 3>& S_v_F_S) {
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
    graphMgrPtr_->addUnaryHolisticFactor<GmsfUnaryExpressionLocalVelocity3>(gmsfUnaryExpressionVelocity3SensorFramePtr);

    // Optimize ---------------------------------------------------------------
    {
      // Mutex for optimizeGraph Flag
      const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
      optimizeGraphFlag_ = true;
    }
  }
}

// Velocity3 in Body Frame, sensor frame not rigidly attached to the IMU
void HolisticFusionHolistic::addUnaryVelocity3LocalMovingFrameMeasurement(UnaryMeasurementXD<Eigen::Vector3d, 3>& S_v_F_S,
                                                                    const Eigen::Isometry3d& T_I_sensorFrame,
                                                                    const Eigen::Vector3d& I_w_W_I) {
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

    // Create GMSF expression, with the caller-supplied extrinsics and angular velocity
    auto gmsfUnaryExpressionVelocity3SensorFramePtr = std::make_shared<GmsfUnaryExpressionLocalVelocity3>(
        std::make_shared<UnaryMeasurementXD<Eigen::Vector3d, 3>>(S_v_F_S), staticTransformsPtr_->getImuFrame(), T_I_sensorFrame, I_w_W_I);

    // Add factor to graph
    graphMgrPtr_->addUnaryHolisticFactor<GmsfUnaryExpressionLocalVelocity3>(gmsfUnaryExpressionVelocity3SensorFramePtr);

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
void HolisticFusionHolistic::addUnaryPosition3LandmarkMeasurement(UnaryMeasurementXDLandmark<Eigen::Vector3d, 3>& S_t_S_L,
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
    graphMgrPtr_->addUnaryHolisticFactor<GmsfUnaryExpressionLandmarkPosition3>(gmsfUnaryExpressionPosition3LandmarkPtr);

    // Optimize ---------------------------------------------------------------
    {
      // Mutex for optimizeGraph Flag
      const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
      optimizeGraphFlag_ = true;
    }
  }
}

// Bearing3
void HolisticFusionHolistic::addUnaryBearing3LandmarkMeasurement(UnaryMeasurementXDLandmark<Eigen::Vector3d, 3>& S_bearing_S_L) {
  throw std::runtime_error("Landmark measurements are not yet supported for the holistic MSF.");
}

// Binary Measurements: Purely relative --------------------------------------------------------

}  // namespace holistic_fusion
