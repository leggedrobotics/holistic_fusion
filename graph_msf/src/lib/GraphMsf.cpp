/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#include "graph_msf/frontend/GraphMsf.h"
#include "graph_msf/core/GraphManager.hpp"

namespace graph_msf {

// Public -----------------------------------------------------------
/// Constructor -----------
GraphMsf::GraphMsf() {
  std::cout << YELLOW_START << "GMsf" << GREEN_START << " Instance created." << COLOR_END
            << " Waiting for setup() with graphConfiguration and staticTransforms." << std::endl;
}

/// Setup ------------
bool GraphMsf::setup() {
  std::cout << YELLOW_START << "GMsf" << GREEN_START << " Setting up." << COLOR_END << std::endl;

  // Graph Config
  if (graphConfigPtr_ == nullptr || staticTransformsPtr_ == nullptr) {
    std::cout << YELLOW_START << "GMsf" << RED_START << " GraphConfig or StaticTransforms not set. Finishing" << COLOR_END << std::endl;
    std::runtime_error("GraphConfig or StaticTransforms not set. Finishing");
    return false;
  }

  graphMgrPtr_ = std::make_shared<GraphManager>(graphConfigPtr_);

  // Configs
  graphMgrPtr_->getIsamParamsReference().findUnusedFactorSlots = graphConfigPtr_->findUnusedFactorSlotsFlag;
  graphMgrPtr_->getIsamParamsReference().enableDetailedResults = graphConfigPtr_->enableDetailedResultsFlag;
  graphMgrPtr_->getIsamParamsReference().relinearizeSkip = graphConfigPtr_->relinearizeSkip;
  graphMgrPtr_->getIsamParamsReference().enableRelinearization = graphConfigPtr_->enableRelinearizationFlag;
  graphMgrPtr_->getIsamParamsReference().evaluateNonlinearError = graphConfigPtr_->evaluateNonlinearErrorFlag;
  graphMgrPtr_->getIsamParamsReference().cacheLinearizedFactors = graphConfigPtr_->cacheLinearizedFactorsFlag;
  graphMgrPtr_->getIsamParamsReference().enablePartialRelinearizationCheck = graphConfigPtr_->enablePartialRelinearizationCheckFlag;

  /// Initialize helper threads
  optimizeGraphThread_ = std::thread(&GraphMsf::optimizeGraph_, this);
  std::cout << YELLOW_START << "GMsf" << COLOR_END << " Initialized thread for optimizing the graph in parallel." << std::endl;

  std::cout << YELLOW_START << "GMsf" << GREEN_START << " Set up successfully." << COLOR_END << std::endl;
  return true;
}

bool GraphMsf::areYawAndPositionInited() {
  return foundInitialYawAndPositionFlag_;
}

bool GraphMsf::areRollAndPitchInited() {
  return alignedImuFlag_;
}

void GraphMsf::activateFallbackGraph() {
  if (graphConfigPtr_->usingFallbackGraphFlag) {
    graphMgrPtr_->activateFallbackGraph();
  } else {
    std::cout << YELLOW_START << "GMsf" << RED_START << " Not activating fallback graph, disabled in config." << COLOR_END << std::endl;
  }
}

bool GraphMsf::initYawAndPosition(const double yaw_W_frame1, const std::string& frame1, const Eigen::Vector3d& W_t_W_frame2,
                                  const std::string& frame2) {
  // Locking
  const std::lock_guard<std::mutex> initYawAndPositionLock(initYawAndPositionMutex_);

  // Different Modes
  if (!alignedImuFlag_) {  // Case 1: IMU not yet aligned --> wait for IMU callback to align roll and pitch of IMU

    std::cout << YELLOW_START << "GMsf" << RED_START << " Tried to set initial yaw, but initial attitude is not yet set." << COLOR_END
              << std::endl;
    return false;

  } else if (!areYawAndPositionInited()) {  // Case 2: Imu is aligned, but roll and pitch not yet --> do it
                                            // Transform yaw to imu frame
    gtsam::Rot3 yawR_W_frame1 = gtsam::Rot3::Yaw(yaw_W_frame1);
    std::cout << YELLOW_START << "GMsf" << GREEN_START << " Setting yaw in " << frame1 << " frame." << COLOR_END << std::endl;
    double yaw_W_I0_ =
        (yawR_W_frame1 *
         gtsam::Pose3(staticTransformsPtr_->rv_T_frame1_frame2(frame1, staticTransformsPtr_->getImuFrame()).matrix()).rotation())
            .yaw();
    // Set Yaw
    preIntegratedNavStatePtr_->updateYawInWorld(yaw_W_I0_, graphConfigPtr_->reLocalizeWorldToMapAtStartFlag);

    // Transform position to imu frame
    Eigen::Matrix3d R_W_I0 = preIntegratedNavStatePtr_->getT_W_Ik().rotation().matrix();
    Eigen::Vector3d W_t_W_I0 = W_t_W_Frame1_to_W_t_W_Frame2_(W_t_W_frame2, frame2, staticTransformsPtr_->getImuFrame(), R_W_I0);
    // Set Position
    preIntegratedNavStatePtr_->updatePositionInWorld(W_t_W_I0, graphConfigPtr_->reLocalizeWorldToMapAtStartFlag);

    // Wrap Up
    foundInitialYawAndPositionFlag_ = true;
    std::cout << YELLOW_START << "GMsf" << GREEN_START << " Initial global yaw of from world frame to imu frame has been set to (deg) "
              << 180.0 * yaw_W_I0_ / M_PI << "." << COLOR_END << std::endl;
    return true;

  } else {  // Case 3: Initial yaw and position already set --> do nothing
    std::cout << YELLOW_START << "GMsf" << RED_START << " Tried to set initial yaw, but it has been set before." << COLOR_END << std::endl;
    return false;
  }
}

bool GraphMsf::initYawAndPosition(const Eigen::Isometry3d& T_O_frame, const std::string& frameName) {
  gtsam::Pose3 T_O_frame_gtsam(T_O_frame.matrix());
  return initYawAndPosition(T_O_frame_gtsam.rotation().yaw(), frameName, T_O_frame.translation(), frameName);
}

// Private ---------------------------------------------------------------
/// Callbacks -----------------------
bool GraphMsf::addImuMeasurementAndGetState(
    const Eigen::Vector3d& linearAcc, const Eigen::Vector3d& angularVel, const double imuTimeK,
    std::shared_ptr<SafeNavState>& returnPreIntegratedNavStatePtr,
    std::shared_ptr<SafeNavStateWithCovarianceAndBias>& returnOptimizedStateWithCovarianceAndBiasPtr,
    Eigen::Matrix<double, 6, 1>& returnAddedImuMeasurements) {
  // Setup -------------------------
  // Increase counter
  ++imuCallbackCounter_;
  // First Iteration
  if (preIntegratedNavStatePtr_ == nullptr) {
    preIntegratedNavStatePtr_ = std::make_shared<SafeNavState>();
    preIntegratedNavStatePtr_->updateLatestMeasurementTimestamp(imuTimeK);
  }
  // Filter out imu messages with same time stamp
  if (std::abs(imuTimeK - preIntegratedNavStatePtr_->getTimeK()) < 1e-8 && imuCallbackCounter_ > 1) {
    std::cout << YELLOW_START << " GMsf" << RED_START << " Imu time " << std::setprecision(14) << imuTimeK << " was repeated." << COLOR_END
              << std::endl;
    return false;
  }
  // Add measurement to buffer
  returnAddedImuMeasurements = graphMgrPtr_->addToIMUBuffer(imuTimeK, linearAcc, angularVel);

  // Locking
  const std::lock_guard<std::mutex> initYawAndPositionLock(initYawAndPositionMutex_);

  // State Machine in form of if-else statements -----------------
  if (!alignedImuFlag_) {  // Case 1: IMU not aligned
    // Try to align
    double imuAttitudeRoll, imuAttitudePitch = 0.0;
    if (!alignImu_(imuAttitudeRoll, imuAttitudePitch)) {  // Case 1.1: IMU alignment failed --> try again next time
      // Print only once per second
      if (imuCallbackCounter_ % int(graphConfigPtr_->imuRate) == 0) {
        std::cout << YELLOW_START << "GMsf" << COLOR_END << " NOT ENOUGH IMU MESSAGES TO INITIALIZE POSE. WAITING FOR MORE..." << std::endl;
      }
      return false;
    } else {  // Case 1.2: IMU alignment succeeded --> continue next call iteration
      Eigen::Matrix3d R_W_I0_attitude = gtsam::Rot3::Ypr(0.0, imuAttitudePitch, imuAttitudeRoll).matrix();
      gtsam::Rot3 R_W_Init0_attitude(
          R_W_I0_attitude *
          staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(), staticTransformsPtr_->getInitializationFrame())
              .rotation()
              .matrix());
      // Set yaw of base frame to zero
      R_W_Init0_attitude = gtsam::Rot3::Ypr(0.0, R_W_Init0_attitude.pitch(), R_W_Init0_attitude.roll());
      R_W_I0_attitude =
          R_W_Init0_attitude.matrix() *
          staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getInitializationFrame(), staticTransformsPtr_->getImuFrame())
              .rotation()
              .matrix();
      Eigen::Isometry3d T_O_Ik_attitude = Eigen::Isometry3d::Identity();
      T_O_Ik_attitude.matrix().block<3, 3>(0, 0) = R_W_I0_attitude;
      T_O_Ik_attitude.matrix().block<3, 1>(0, 3) =
          Eigen::Vector3d(0, 0, 0) -
          R_W_I0_attitude *
              staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(), staticTransformsPtr_->getInitializationFrame())
                  .translation();
      Eigen::Vector3d zeroPVeloctiy = Eigen::Vector3d(0, 0, 0);
      preIntegratedNavStatePtr_ = std::make_shared<SafeNavState>(T_O_Ik_attitude, zeroPVeloctiy, zeroPVeloctiy, imuTimeK);
      alignedImuFlag_ = true;
      return false;
    }
  } else if (!areYawAndPositionInited()) {  // Case 2: IMU aligned, but yaw and position not initialized, waiting for external
    // initialization, meanwhile publishing initial roll and pitch
    // Printing every second
    if (imuCallbackCounter_ % int(graphConfigPtr_->imuRate) == 0) {
      std::cout << YELLOW_START << "GMsf" << COLOR_END << " IMU callback waiting for initialization of global yaw and initial position."
                << std::endl;
    }
    // Publish state with correct roll and pitch, nothing has changed compared to Case 1.2
    preIntegratedNavStatePtr_->updateLatestMeasurementTimestamp(imuTimeK);
    returnPreIntegratedNavStatePtr = std::make_shared<SafeNavState>(*preIntegratedNavStatePtr_);
    return true;
  } else if (!validFirstMeasurementReceivedFlag_) {  // Case 3: No valid measurement received yet, e.g. because GNSS Covariance is too high
    std::cout << YELLOW_START << "GMsf" << RED_START << " ...waiting for first valid measurement before initializing graph." << COLOR_END
              << std::endl;
    preIntegratedNavStatePtr_->updateLatestMeasurementTimestamp(imuTimeK);
    returnPreIntegratedNavStatePtr = std::make_shared<SafeNavState>(*preIntegratedNavStatePtr_);
    return true;
  } else if (!initedGraphFlag_) {  // Case 4: IMU aligned, yaw and position initialized, valid measurement received, but graph not yet
                                   // initialized
    preIntegratedNavStatePtr_->updateLatestMeasurementTimestamp(imuTimeK);
    initGraph_(imuTimeK);
    std::cout << YELLOW_START << "GMsf" << GREEN_START << " ...graph is initialized." << COLOR_END << std::endl;
    returnPreIntegratedNavStatePtr = std::make_shared<SafeNavState>(*preIntegratedNavStatePtr_);
    if (optimizedNavStateWithCovariancePtr_ != nullptr) {
      returnOptimizedStateWithCovarianceAndBiasPtr =
          std::make_shared<SafeNavStateWithCovarianceAndBias>(*optimizedNavStateWithCovariancePtr_);
    }
    return true;
  } else if (!normalOperationFlag_) {  // Case 5: IMU aligned, yaw and position initialized, graph initialized --> normal operation, meaning
                                       // predicting the next state
                                       // via integration
    normalOperationFlag_ = true;
  }

  // Normal operation ------------------------------------------------------------
  bool relocalizeWorldToMapFlag = false;
  // Add IMU factor and get propagated state
  gtsam::NavState T_W_Ik_nav = graphMgrPtr_->addImuFactorAndGetState(imuTimeK, linearAcc, angularVel, relocalizeWorldToMapFlag);

  // Assign poses and velocities ---------------------------------------------------
  preIntegratedNavStatePtr_->updateInWorld(Eigen::Isometry3d(T_W_Ik_nav.pose().matrix()), T_W_Ik_nav.bodyVelocity(),
                                           graphMgrPtr_->getOptimizedImuBias().correctGyroscope(angularVel), imuTimeK,
                                           relocalizeWorldToMapFlag);

  // Return Corresponding State ----------------------------------------------------------------
  returnPreIntegratedNavStatePtr = std::make_shared<SafeNavState>(*preIntegratedNavStatePtr_);
  if (optimizedNavStateWithCovariancePtr_ != nullptr) {
    returnOptimizedStateWithCovarianceAndBiasPtr =
        std::make_shared<SafeNavStateWithCovarianceAndBias>(*optimizedNavStateWithCovariancePtr_);
  }

  return true;
}

void GraphMsf::addOdometryMeasurement(const BinaryMeasurement6D& deltaMeasurement) {
  // Valid measurement received
  if (!validFirstMeasurementReceivedFlag_) {
    validFirstMeasurementReceivedFlag_ = true;
  }

  // Only take actions if graph has been initialized
  if (!initedGraphFlag_) {
    return;
  }

  Eigen::Isometry3d T_fkm1_fk = deltaMeasurement.T_delta();

  // Check frame of measuremnts
  if (deltaMeasurement.measurementName() != staticTransformsPtr_->getImuFrame()) {
    T_fkm1_fk = staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(), deltaMeasurement.frameName()) * T_fkm1_fk *
                staticTransformsPtr_->rv_T_frame1_frame2(deltaMeasurement.frameName(), staticTransformsPtr_->getImuFrame());
  }

  const gtsam::Key keyAtMeasurementK = graphMgrPtr_->addPoseBetweenFactorToGlobalGraph(
      deltaMeasurement.timeKm1(), deltaMeasurement.timeK(), deltaMeasurement.measurementRate(), deltaMeasurement.measurementNoise(),
      gtsam::Pose3(T_fkm1_fk.matrix()), deltaMeasurement.measurementName());

  // Optimize
  {
    // Mutex for optimizeGraph Flag
    const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
    optimizeGraphFlag_ = true;
  }
}

void GraphMsf::addUnaryPoseMeasurement(const UnaryMeasurement6D& unary) {
  // Valid measurement received
  if (!validFirstMeasurementReceivedFlag_) {
    validFirstMeasurementReceivedFlag_ = true;
  }

  // Only take actions if graph has been initialized
  if (!initedGraphFlag_) {
    return;
  }

  gtsam::Pose3 T_W_frame(unary.measurementPose().matrix());
  gtsam::Pose3 T_W_I =
      T_W_frame * gtsam::Pose3(staticTransformsPtr_->rv_T_frame1_frame2(unary.frameName(), staticTransformsPtr_->getImuFrame()).matrix());

  if (initedGraphFlag_) {
    graphMgrPtr_->addPoseUnaryFactorToGlobalGraph(unary.timeK(), unary.measurementRate(), unary.measurementNoise(), T_W_I);
    graphMgrPtr_->addPoseUnaryFactorToFallbackGraph(unary.timeK(), unary.measurementRate(), unary.measurementNoise(), T_W_I);

    // Optimize
    {
      // Mutex for optimizeGraph Flag
      const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
      optimizeGraphFlag_ = true;
    }
  }
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

bool GraphMsf::isGnssCovarianceViolated_(const Eigen::Vector3d& gnssCovarianceXYZ) {
  return gnssCovarianceXYZ(0) > GNSS_COVARIANCE_VIOLATION_THRESHOLD || gnssCovarianceXYZ(1) > GNSS_COVARIANCE_VIOLATION_THRESHOLD ||
         gnssCovarianceXYZ(2) > GNSS_COVARIANCE_VIOLATION_THRESHOLD;
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

void GraphMsf::addGnssPositionMeasurement(const UnaryMeasurement3D& W_t_W_frame) {
  // Valid measurement received
  if (!validFirstMeasurementReceivedFlag_) {
    validFirstMeasurementReceivedFlag_ = true;
  }

  // Only take actions if graph has been initialized
  if (!initedGraphFlag_) {
    return;
  }

  // Check whether IMU has been added already
  if (graphMgrPtr_->getPropagatedStateKey() == 0) {
    return;
  }

  // Add unary factor
  gtsam::Point3 W_t_W_I =
      W_t_W_Frame1_to_W_t_W_Frame2_(W_t_W_frame.measurementVector(), W_t_W_frame.frameName(), staticTransformsPtr_->getImuFrame(),
                                    preIntegratedNavStatePtr_->getT_W_Ik().rotation());

  graphMgrPtr_->addGnssPositionUnaryFactor(W_t_W_frame.timeK(), W_t_W_frame.measurementRate(), W_t_W_frame.measurementNoise(), W_t_W_I);
  {
    // Mutex for optimizeGraph Flag
    const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
    optimizeGraphFlag_ = true;
  }
}

void GraphMsf::addGnssHeadingMeasurement(const UnaryMeasurement1D& yaw_W_frame) {
  // Only take actions if graph has been initialized
  if (!initedGraphFlag_) {
    return;
  }

  // Transform yaw to imu frame
  gtsam::Rot3 yawR_W_frame = gtsam::Rot3::Yaw(yaw_W_frame.measurementValue());
  gtsam::Rot3 yawR_W_I =
      yawR_W_frame *
      gtsam::Rot3(staticTransformsPtr_->rv_T_frame1_frame2(yaw_W_frame.frameName(), staticTransformsPtr_->getImuFrame()).rotation());

  if (!gnssCovarianceViolatedFlag_ && (gnssNotJumpingCounter_ >= REQUIRED_GNSS_NUM_NOT_JUMPED)) {
    graphMgrPtr_->addGnssHeadingUnaryFactor(yaw_W_frame.timeK(), yaw_W_frame.measurementRate(), yaw_W_frame.measurementNoise(),
                                            yawR_W_I.yaw());
    {
      // Mutex for optimizeGraph Flag
      const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
      optimizeGraphFlag_ = true;
    }
  }

  // Set yaw for potential resetting
  lastGnssYaw_W_I_ = yawR_W_I.yaw();
}

/// Worker Functions -----------------------
bool GraphMsf::alignImu_(double& imuAttitudeRoll, double& imuAttitudePitch) {
  gtsam::Rot3 R_W_I_rollPitch;
  static int alignImuCounter__ = -1;
  ++alignImuCounter__;
  double estimatedGravityMagnitude;
  if (graphMgrPtr_->estimateAttitudeFromImu(R_W_I_rollPitch, estimatedGravityMagnitude, graphMgrPtr_->getInitGyrBiasReference())) {
    imuAttitudeRoll = R_W_I_rollPitch.roll();
    imuAttitudePitch = R_W_I_rollPitch.pitch();
    if (graphConfigPtr_->estimateGravityFromImuFlag) {
      graphConfigPtr_->gravityMagnitude = estimatedGravityMagnitude;
      std::cout << YELLOW_START << "GMsf" << COLOR_END
                << " Attitude of IMU is initialized. Determined Gravity Magnitude: " << estimatedGravityMagnitude << std::endl;
    } else {
      std::cout << YELLOW_START << "GMsf" << COLOR_END << " Estimated gravity magnitude from IMU is: " << estimatedGravityMagnitude
                << std::endl;
      std::cout << YELLOW_START << "GMsf" << COLOR_END
                << " This gravity is not used, because estimateGravityFromImu is set to false. Gravity set to "
                << graphConfigPtr_->gravityMagnitude << "." << std::endl;
      gtsam::Vector3 gravityVector = gtsam::Vector3(0, 0, graphConfigPtr_->gravityMagnitude);
      gtsam::Vector3 estimatedGravityVector = gtsam::Vector3(0, 0, estimatedGravityMagnitude);
      gtsam::Vector3 gravityVectorError = estimatedGravityVector - gravityVector;
      gtsam::Vector3 gravityVectorErrorInImuFrame = R_W_I_rollPitch.inverse().rotate(gravityVectorError);
      graphMgrPtr_->getInitAccBiasReference() = gravityVectorErrorInImuFrame;
      std::cout << YELLOW_START << "GMsf" << COLOR_END << " Gravity error in IMU frame is: " << gravityVectorErrorInImuFrame.transpose()
                << std::endl;
    }
    return true;
  } else {
    return false;
  }
}

// Graph initialization for roll & pitch from starting attitude, assume zero yaw
void GraphMsf::initGraph_(const double timeStamp_k) {
  // Calculate initial attitude;
  gtsam::Pose3 T_W_I0 = gtsam::Pose3(preIntegratedNavStatePtr_->getT_W_Ik().matrix());
  // Print
  std::cout << YELLOW_START << "GMsf" << GREEN_START
            << " Total initial IMU attitude is Yaw/Pitch/Roll(deg): " << T_W_I0.rotation().ypr().transpose() * (180.0 / M_PI) << COLOR_END
            << std::endl;

  // Gravity
  graphMgrPtr_->initImuIntegrators(graphConfigPtr_->gravityMagnitude);
  /// Initialize graph node
  graphMgrPtr_->initPoseVelocityBiasGraph(timeStamp_k, T_W_I0);

  // Read initial pose from graph
  T_W_I0 = graphMgrPtr_->getOptimizedGraphState().navState().pose();
  std::cout << YELLOW_START << "GMsf " << GREEN_START
            << " INITIAL POSE after first optimization, x,y,z (m): " << T_W_I0.translation().transpose()
            << ", RPY (deg): " << T_W_I0.rotation().rpy().transpose() * (180.0 / M_PI) << COLOR_END << std::endl;
  std::cout << YELLOW_START << "GMsf" << COLOR_END << " Factor graph key of very first node: " << graphMgrPtr_->getPropagatedStateKey()
            << std::endl;

  // Set flag
  initedGraphFlag_ = true;
}

void GraphMsf::optimizeGraph_() {
  // While loop
  std::cout << YELLOW_START << "GMsf" << COLOR_END << " Thread for updating graph is ready." << std::endl;
  double lastOptimizedTime = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
  ;
  double currentTime = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
  ;
  while (true) {
    bool optimizeGraphFlag = false;
    // Mutex for optimizeGraph Flag
    {
      // Lock
      const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
      currentTime = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
      if (optimizeGraphFlag_ && (currentTime - lastOptimizedTime > 1.0 / graphConfigPtr_->maxOptimizationFrequency)) {
        optimizeGraphFlag = optimizeGraphFlag_;
        optimizeGraphFlag_ = false;
        lastOptimizedTime = currentTime;
      }
    }

    if (optimizeGraphFlag) {
      // Get result
      double currentTime;

      SafeNavStateWithCovarianceAndBias optimizedStateWithCovarianceAndBias = graphMgrPtr_->updateActiveGraphAndGetState(currentTime);
      optimizedNavStateWithCovariancePtr_ = std::make_shared<SafeNavStateWithCovarianceAndBias>(optimizedStateWithCovarianceAndBias);

    }  // else just sleep for a short amount of time before polling again
    else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

Eigen::Vector3d GraphMsf::W_t_W_Frame1_to_W_t_W_Frame2_(const Eigen::Vector3d& W_t_W_frame1, const std::string& frame1,
                                                        const std::string& frame2, const Eigen::Matrix3d& R_W_frame2) {
  // Static transforms
  const Eigen::Isometry3d& T_frame2_frame1 = staticTransformsPtr_->rv_T_frame1_frame2(frame2, frame1);
  const Eigen::Vector3d& frame1_t_frame1_frame2 = staticTransformsPtr_->rv_T_frame1_frame2(frame1, frame2).translation();

  /// Global rotation
  Eigen::Matrix3d R_W_frame1 = R_W_frame2 * T_frame2_frame1.rotation();

  /// Translation in global frame
  Eigen::Vector3d W_t_frame1_frame2 = R_W_frame1 * frame1_t_frame1_frame2;

  /// Shift observed Gnss position to IMU frame (instead of Gnss antenna)
  return W_t_W_frame1 + W_t_frame1_frame2;
}

void GraphMsf::pretendFirstMeasurementReceived() {
  validFirstMeasurementReceivedFlag_ = true;
}

}  // namespace graph_msf
