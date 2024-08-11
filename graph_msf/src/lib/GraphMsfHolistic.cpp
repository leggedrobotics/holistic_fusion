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
#include "graph_msf/factors/gmsf_expression/GmsfUnaryExpressionPose3.h"
#include "graph_msf/factors/gmsf_expression/GmsfUnaryExpressionPosition3.h"
#include "graph_msf/factors/gmsf_expression/GmsfUnaryExpressionVelocity3SensorFrame.h"

// Binary Expression Factors
// TODO: add binary factors

namespace graph_msf {

// Constructor
GraphMsfHolistic::GraphMsfHolistic() {
  REGULAR_COUT << GREEN_START << " GraphMsfHolistic-Constructor called." << COLOR_END << std::endl;
}

// Unary Measurements ---------------------------------------------------------

// Pose3
void GraphMsfHolistic::addUnaryPose3Measurement(const UnaryMeasurementXD<Eigen::Isometry3d, 6>& T_fixedFrame_sensorFrame) {
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
    auto gmsfUnaryExpressionPose3Ptr = std::make_shared<GmsfUnaryExpressionPose3>(
        std::make_shared<UnaryMeasurementXD<Eigen::Isometry3d, 6>>(T_fixedFrame_sensorFrame), staticTransformsPtr_->getWorldFrame(),
        staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(), T_fixedFrame_sensorFrame.sensorFrameName()));

    // Add factor to graph
    graphMgrPtr_->addUnaryGmsfExpressionFactor<gtsam::Pose3>(gmsfUnaryExpressionPose3Ptr);

    // Optimize ---------------------------------------------------------------
    {
      // Mutex for optimizeGraph Flag
      const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
      optimizeGraphFlag_ = true;
    }
  }
}

// Position3
void GraphMsfHolistic::addUnaryPosition3Measurement(UnaryMeasurementXD<Eigen::Vector3d, 3>& fixedFrame_t_fixedFrame_sensorFrame) {
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
    auto gmsfUnaryExpressionPosition3Ptr = std::make_shared<GmsfUnaryExpressionPosition3>(
        std::make_shared<UnaryMeasurementXD<Eigen::Vector3d, 3>>(fixedFrame_t_fixedFrame_sensorFrame),
        staticTransformsPtr_->getWorldFrame(),
        staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(),
                                                 fixedFrame_t_fixedFrame_sensorFrame.sensorFrameName()));

    // Add factor to graph
    graphMgrPtr_->addUnaryGmsfExpressionFactor<gtsam::Vector3>(gmsfUnaryExpressionPosition3Ptr);

    // Optimize ---------------------------------------------------------------
    {
      // Mutex for optimizeGraph Flag
      const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
      optimizeGraphFlag_ = true;
    }
  }
}

// Velocity3 in Fixed Frame
void GraphMsfHolistic::addUnaryVelocity3FixedFrameMeasurement(UnaryMeasurementXD<Eigen::Vector3d, 3>& F_v_F_S) {
  throw std::runtime_error("Velocity measurements in fixed frame are not yet supported.");
}

// Velocity3 in Body Frame
void GraphMsfHolistic::addUnaryVelocity3SensorFrameMeasurement(UnaryMeasurementXD<Eigen::Vector3d, 3>& S_v_F_S) {
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
    auto gmsfUnaryExpressionVelocity3SensorFramePtr = std::make_shared<GmsfUnaryExpressionVelocity3SensorFrame>(
        std::make_shared<UnaryMeasurementXD<Eigen::Vector3d, 3>>(S_v_F_S), staticTransformsPtr_->getWorldFrame(),
        staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(), S_v_F_S.sensorFrameName()), coreImuBufferPtr_);

    // Add factor to graph
    graphMgrPtr_->addUnaryGmsfExpressionFactor<gtsam::Vector3>(gmsfUnaryExpressionVelocity3SensorFramePtr);

    // Optimize ---------------------------------------------------------------
    {
      // Mutex for optimizeGraph Flag
      const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
      optimizeGraphFlag_ = true;
    }
  }
}

// Roll
void GraphMsfHolistic::addUnaryRollMeasurement(const UnaryMeasurementXD<double, 1>& roll_W_frame) {
  throw std::runtime_error("Roll measurements are not yet supported for the holistic MSF.");
}

// Pitch
void GraphMsfHolistic::addUnaryPitchMeasurement(const UnaryMeasurementXD<double, 1>& pitch_W_frame) {
  throw std::runtime_error("Pitch measurements are not yet supported for the holistic MSF.");
}

// Binary Measurements --------------------------------------------------------

}  // namespace graph_msf
