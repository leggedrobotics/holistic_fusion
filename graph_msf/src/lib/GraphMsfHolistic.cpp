/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "graph_msf/interface/GraphMsfHolistic.h"

// Workspace
#include "graph_msf/core/GraphManager.hpp"
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
void GraphMsfHolistic::addUnaryPose3AbsoluteMeasurement(const UnaryMeasurementXDAbsolute<Eigen::Isometry3d, 6>& T_fixedFrame_sensorFrame) {
  // Valid measurement received
  if (!validFirstMeasurementReceivedFlag_) {
    validFirstMeasurementReceivedFlag_ = true;
  }

  // Only take actions if graph has been initialized
  if (!initedGraphFlag_) {  // Graph not yet initialized
    return;
  } else {  // Graph initialized
    // Check for covariance violation
    bool covarianceViolatedFlag = isCovarianceViolated_<6>(T_fixedFrame_sensorFrame.unaryMeasurementNoiseDensity(),
                                                           T_fixedFrame_sensorFrame.covarianceViolationThreshold());
    if (covarianceViolatedFlag) {
      REGULAR_COUT << RED_START << " Pose covariance violated. Not adding factor." << COLOR_END << std::endl;
      return;
    }

    // Create GMSF expression
    auto gmsfUnaryExpressionPose3Ptr = std::make_shared<GmsfUnaryExpressionAbsolutePose3>(
        std::make_shared<UnaryMeasurementXDAbsolute<Eigen::Isometry3d, 6>>(T_fixedFrame_sensorFrame), staticTransformsPtr_->getWorldFrame(),
        staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(), T_fixedFrame_sensorFrame.sensorFrameName()));

    // Add factor to graph
    graphMgrPtr_->addUnaryGmsfExpressionFactor<GmsfUnaryExpressionAbsolutePose3>(gmsfUnaryExpressionPose3Ptr);

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
    if (covarianceViolatedFlag) {
      REGULAR_COUT << RED_START << " Position covariance violated. Not adding factor." << COLOR_END << std::endl;
      return;
    }

    // Create GMSF expression
    auto gmsfUnaryExpressionPosition3Ptr = std::make_shared<GmsfUnaryExpressionAbsolutePosition3>(
        std::make_shared<UnaryMeasurementXDAbsolute<Eigen::Vector3d, 3>>(fixedFrame_t_fixedFrame_sensorFrame),
        staticTransformsPtr_->getWorldFrame(),
        staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(),
                                                 fixedFrame_t_fixedFrame_sensorFrame.sensorFrameName()));

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
  throw std::runtime_error("Velocity measurements in fixed frame are not yet supported.");
}

// Roll
void GraphMsfHolistic::addUnaryRollAbsoluteMeasurement(const UnaryMeasurementXDAbsolute<double, 1>& roll_W_frame) {
  throw std::runtime_error("Roll measurements are not yet supported for the holistic MSF.");
}

// Pitch
void GraphMsfHolistic::addUnaryPitchAbsoluteMeasurement(const UnaryMeasurementXDAbsolute<double, 1>& pitch_W_frame) {
  throw std::runtime_error("Pitch measurements are not yet supported for the holistic MSF.");
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
    if (covarianceViolatedFlag) {
      REGULAR_COUT << RED_START << " Velocity covariance violated. Not adding factor." << COLOR_END << std::endl;
      return;
    }

    // Create GMSF expression
    auto gmsfUnaryExpressionVelocity3SensorFramePtr = std::make_shared<GmsfUnaryExpressionLocalVelocity3>(
        std::make_shared<UnaryMeasurementXD<Eigen::Vector3d, 3>>(S_v_F_S), staticTransformsPtr_->getWorldFrame(),
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
void GraphMsfHolistic::addUnaryPosition3LandmarkMeasurement(UnaryMeasurementXD<Eigen::Vector3d, 3>& S_t_S_L,
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
    if (covarianceViolatedFlag) {
      REGULAR_COUT << RED_START << " Position covariance violated. Not adding factor." << COLOR_END << std::endl;
      return;
    }

    // TODO: Change this to more explicit handling of counter
    S_t_S_L.setMeasurementName(S_t_S_L.measurementName() + "_" + std::to_string(landmarkCreationCounter));

    // Create GMSF expression
    auto gmsfUnaryExpressionPosition3LandmarkPtr = std::make_shared<GmsfUnaryExpressionLandmarkPosition3>(
        std::make_shared<UnaryMeasurementXD<Eigen::Vector3d, 3>>(S_t_S_L), staticTransformsPtr_->getWorldFrame(),
        staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(), S_t_S_L.sensorFrameName()));

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
void GraphMsfHolistic::addUnaryBearing3LandmarkMeasurement(UnaryMeasurementXD<Eigen::Vector3d, 3>& S_bearing_S_L) {
  throw std::runtime_error("Landmark measurements are not yet supported for the holistic MSF.");
}

// Binary Measurements: Purely relative --------------------------------------------------------

}  // namespace graph_msf
