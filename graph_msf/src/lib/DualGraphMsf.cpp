/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#include "graph_msf/interface/DualGraphMsf.h"
#include "graph_msf/core/GraphManager.hpp"

namespace graph_msf {

// Public -----------------------------------------------------------
/// Constructor -----------
DualGraphMsf::DualGraphMsf() : GraphMsf() {
  std::cout << YELLOW_START << "DualGMsf" << GREEN_START << " Instance created." << COLOR_END
            << " Waiting for setup() with graphConfiguration and staticTransforms." << std::endl;
}

std::shared_ptr<SafeNavState> GraphMsf::addDualOdometryMeasurementAndReturnNavState(const UnaryMeasurement6D& odometryKm1,
                                                                                    const UnaryMeasurement6D& odometryK,
                                                                                    const Eigen::Matrix<double, 6, 1>& poseBetweenNoise) {
  // Measurement
  const Eigen::Isometry3d T_M_Lj = odometryK.measurementPose();

  // Check whether World->Map is already set
  if (!validFirstMeasurementReceivedFlag_) {
    Eigen::Isometry3d T_M_Ij =
        Eigen::Isometry3d(T_M_Lj) * staticTransformsPtr_->rv_T_frame1_frame2(odometryK.frameName(), staticTransformsPtr_->getImuFrame());
    preIntegratedNavStatePtr_->updatePoseInMap(T_M_Ij);
    // Received
    validFirstMeasurementReceivedFlag_ = true;
  }

  // Only take actions if graph has been initialized
  if (!initedGraphFlag_) {
    return nullptr;
  }

  /// Delta Factor ---------------------------------------------------------------
  const Eigen::Isometry3d T_Lkm1_Lk(odometryKm1.measurementPose().inverse() * odometryK.measurementPose());
  const Eigen::Isometry3d T_Ikm1_Ik =
      staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(), odometryKm1.frameName()) * T_Lkm1_Lk *
      staticTransformsPtr_->rv_T_frame1_frame2(odometryK.frameName(), staticTransformsPtr_->getImuFrame());
  const gtsam::Key keyAtMeasurementK =
      graphMgrPtr_->addPoseBetweenFactorToGlobalGraph(odometryKm1.timeK(), odometryK.timeK(), odometryK.measurementRate(), poseBetweenNoise,
                                                      gtsam::Pose3(T_Ikm1_Ik.matrix()), odometryK.measurementName());

  // Trigger Optimization
  {
    // Mutex for optimizeGraph Flag
    const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
    optimizeGraphFlag_ = true;
  }

  // Create Pseudo Unary Factor ---------------------------------------------------

  if (graphMgrPtr_->globalGraphActiveFlag()) {  // Case 1: Global graph --> Compute World to Map frame    // Lidar State
                                                // Calculate imu state of LiDAR timestamp
    bool computeSuccessfulFlag = false;
    Eigen::Isometry3d T_W_Ij_Graph(graphMgrPtr_->calculateActiveStateAtKey(computeSuccessfulFlag, keyAtMeasurementK).pose().matrix());
    if (computeSuccessfulFlag) {
      Eigen::Isometry3d T_M_Ij =
          Eigen::Isometry3d(T_M_Lj) * staticTransformsPtr_->rv_T_frame1_frame2(odometryK.frameName(), staticTransformsPtr_->getImuFrame());
      preIntegratedNavStatePtr_->updateWorldToMap(T_W_Ij_Graph * T_M_Ij.inverse());
    }
  } else if (graphMgrPtr_->fallbackGraphActiveFlag()) {  // Case 1: Fallback graph --> Add pseudo unary factor to fallback graph
    /// Pseudo Unary Factor
    gtsam::Pose3 pseudo_T_W_Ik(
        preIntegratedNavStatePtr_->getT_W_M() * T_M_Lj *
        staticTransformsPtr_->rv_T_frame1_frame2(odometryK.frameName(), staticTransformsPtr_->getImuFrame()).matrix());
    graphMgrPtr_->addPoseUnaryFactorToFallbackGraph(odometryK.timeK(), odometryK.measurementRate(), odometryK.measurementNoise(),
                                                    pseudo_T_W_Ik);
  }

  // Trigger Optimization
  {
    // Mutex for optimizeGraph Flag
    const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
    optimizeGraphFlag_ = true;
  }

  return std::make_shared<SafeNavState>(*preIntegratedNavStatePtr_);
}

// Counter is only counted if graph switching attemptGraphSwitching is off
void GraphMsf::addDualGnssPositionMeasurement(const UnaryMeasurement3D& W_t_W_frame, const Eigen::Vector3d& W_t_W_frame_km1,
                                              const Eigen::Vector3d& gnssCovarianceXYZ, const bool attemptGraphSwitching,
                                              const bool addedYawBefore) {
  // Check covariance
  bool gnssCovarianceViolatedFlagThisTimestep = isGnssCovarianceViolated_(gnssCovarianceXYZ);
  // Handle GNSS Covariance Violation
  if (gnssCovarianceViolatedFlagThisTimestep) {  // If Violated
    // Printout
    if (!gnssCovarianceViolatedFlag_) {
      std::cout << YELLOW_START << "GMsf" << RED_START << " Gnss measurments now ABSENT due to too big covariance." << COLOR_END
                << std::endl;
    }
    // Set Flag
    gnssCovarianceViolatedFlag_ = true;
    gnssNotJumpingCounter_ = 0;
  } else {  // If not violated
    // Valid measurement received
    if (!validFirstMeasurementReceivedFlag_) {
      validFirstMeasurementReceivedFlag_ = true;
    }
    // Printout
    if (gnssCovarianceViolatedFlag_) {
      std::cout << YELLOW_START << "GMsf" << GREEN_START << " Gnss returned. Low covariance. Now Waiting for GNSS not jumping." << COLOR_END
                << std::endl;
    }
    // Set Flag
    gnssCovarianceViolatedFlag_ = false;
  }

  // Only take actions if graph has been initialized
  if (!initedGraphFlag_) {
    return;
  }

  // Gnss jumping?
  if (!gnssCovarianceViolatedFlag_ &&
      (W_t_W_frame_km1 - W_t_W_frame.measurementVector()).norm() < graphConfigPtr_->poseMotionOutlierThresold) {
    ++gnssNotJumpingCounter_;
    if (gnssNotJumpingCounter_ == REQUIRED_GNSS_NUM_NOT_JUMPED) {
      std::cout << YELLOW_START << "GMsf" << GREEN_START << " Gnss was not jumping recently. Jumping counter valid again." << COLOR_END
                << std::endl;
    }
  } else if ((W_t_W_frame_km1 - W_t_W_frame.measurementVector()).norm() >= graphConfigPtr_->poseMotionOutlierThresold) {
    if (gnssNotJumpingCounter_ >= REQUIRED_GNSS_NUM_NOT_JUMPED) {
      std::cout << YELLOW_START << "GMsf" << RED_START << " Gnss was jumping more than the allowed distance of  "
                << graphConfigPtr_->poseMotionOutlierThresold << "m.  Reset outlier counter." << COLOR_END << std::endl;
    }
    gnssNotJumpingCounter_ = 0;
  }

  // Modifying graph
  if (!gnssCovarianceViolatedFlag_) {  // Case 1: Gnss is good --> Write to graph and perform logic
    gtsam::Rot3 R_W_I_meas(preIntegratedNavStatePtr_->getT_W_Ik().rotation());
    Eigen::Vector3d W_T_W_I = W_t_W_frame.measurementVector();
    // Check whether yaw has been added before --> use this then for position computation
    if (addedYawBefore) {
      R_W_I_meas = gtsam::Rot3::Ypr(lastGnssYaw_W_I_, R_W_I_meas.pitch(), R_W_I_meas.roll());
      W_T_W_I = W_t_W_Frame1_to_W_t_W_Frame2_(W_T_W_I, W_t_W_frame.frameName(), staticTransformsPtr_->getImuFrame(), R_W_I_meas.matrix());
    }

    // Check whether GNSS has just returned, if attempting graph switching then do it
    // if (gnssNotJumpingCounter_ == REQUIRED_GNSS_NUM_NOT_JUMPED && attemptGraphSwitching) {
    graphMgrPtr_->activateGlobalGraph(W_T_W_I, R_W_I_meas, W_t_W_frame.timeK());
    // }
    // If GNSS is not jumping already for a while, then add position measurement
    if ((gnssNotJumpingCounter_ >= REQUIRED_GNSS_NUM_NOT_JUMPED)) {
      addGnssPositionMeasurement(W_t_W_frame);
    }
  } else if (graphConfigPtr_->usingFallbackGraphFlag) {  // Case 2: Gnss is bad --> Do not write to graph, switch to fallback graph
    // Case: Gnss is bad --> Do not write to graph, set flags for odometry unary factor to true
    graphMgrPtr_->activateFallbackGraph();
  }
}

}  // namespace graph_msf
