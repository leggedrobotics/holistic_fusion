/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#include "graph_msf/core/GraphMsf.h"
#include "graph_msf/optimization/GraphManager.hpp"

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
  graphMgrPtr_->getIsamParamsReference().findUnusedFactorSlots = graphConfigPtr_->findUnusedFactorSlots;
  graphMgrPtr_->getIsamParamsReference().enableDetailedResults = graphConfigPtr_->enableDetailedResults;
  graphMgrPtr_->getIsamParamsReference().relinearizeSkip = graphConfigPtr_->relinearizeSkip;
  graphMgrPtr_->getIsamParamsReference().enableRelinearization = graphConfigPtr_->enableRelinearization;
  graphMgrPtr_->getIsamParamsReference().evaluateNonlinearError = graphConfigPtr_->evaluateNonlinearError;
  graphMgrPtr_->getIsamParamsReference().cacheLinearizedFactors = graphConfigPtr_->cacheLinearizedFactors;
  graphMgrPtr_->getIsamParamsReference().enablePartialRelinearizationCheck = graphConfigPtr_->enablePartialRelinearizationCheck;

  /// Initialize helper threads
  optimizeGraphThread_ = std::thread(&GraphMsf::optimizeGraph_, this);
  std::cout << YELLOW_START << "GMsf" << COLOR_END << " Initialized thread for optimizing the graph in parallel." << std::endl;

  std::cout << YELLOW_START << "GMsf" << GREEN_START << " Set up successfully." << COLOR_END << std::endl;
  return true;
}

bool GraphMsf::areYawAndPositionInited() {
  return foundInitialYawAndPositionFlag_;
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
  if (not alignedImuFlag_) {  // Case 1: IMU not yet aligned --> wait for IMU callback to align roll and pitch of IMU

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
    preIntegratedNavStatePtr_->updateGlobalYaw(yaw_W_I0_, graphConfigPtr_->reLocalizeWorldToMapAtStart);

    // Transform position to imu frame
    Eigen::Matrix3d R_W_I0 = preIntegratedNavStatePtr_->getT_W_Ik().rotation().matrix();
    Eigen::Vector3d W_t_W_I0 = W_t_W_Frame1_to_W_t_W_Frame2_(W_t_W_frame2, frame2, staticTransformsPtr_->getImuFrame(), R_W_I0);
    // Set Position
    preIntegratedNavStatePtr_->updateGlobalPosition(W_t_W_I0, graphConfigPtr_->reLocalizeWorldToMapAtStart);

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

bool GraphMsf::initYawAndPosition(const Eigen::Matrix4d& T_O_frame, const std::string& frameName) {
  gtsam::Pose3 T_O_frame_gtsam(T_O_frame);
  return initYawAndPosition(T_O_frame_gtsam.rotation().yaw(), frameName, T_O_frame.block<3, 1>(0, 3), frameName);
}

// Private ---------------------------------------------------------------
/// Callbacks -----------------------
bool GraphMsf::addImuMeasurement(const Eigen::Vector3d& linearAcc, const Eigen::Vector3d& angularVel, const double imuTimeK,
                                 std::shared_ptr<NavState>& returnPreIntegratedNavStatePtr) {
  // Setyp -------------------------
  // Increase counter
  ++imuCallbackCounter_;
  // First Iteration
  if (preIntegratedNavStatePtr_ == nullptr) {
    preIntegratedNavStatePtr_ = std::make_shared<NavState>();
    preIntegratedNavStatePtr_->updateTime(imuTimeK);
  }
  // Filter out imu messages with same time stamp
  if (std::abs(imuTimeK - preIntegratedNavStatePtr_->getTimeK()) < 1e-8 && imuCallbackCounter_ > 1) {
    std::cout << YELLOW_START << " GMsf" << RED_START << " Imu time " << std::setprecision(14) << imuTimeK << " was repeated." << COLOR_END
              << std::endl;
    return false;
  }
  // Add measurement to buffer
  graphMgrPtr_->addToIMUBuffer(imuTimeK, linearAcc, angularVel);

  // State Machine in form of if-else statements -----------------
  if (!alignedImuFlag_) {  // Case 1: IMU not aligned
    // Try to align
    double imuAttitudeRoll, imuAttitudePitch;
    if (!alignImu_(imuAttitudeRoll, imuAttitudePitch)) {  // Case 1.1: IMU alignment failed --> try again next time
      // Print only once per second
      if (imuCallbackCounter_ % int(graphConfigPtr_->imuRate) == 0) {
        std::cout << YELLOW_START << "GMsf" << COLOR_END << " NOT ENOUGH IMU MESSAGES TO INITIALIZE POSE. WAITING FOR MORE..." << std::endl;
      }
      return false;
    } else {  // Case 1.2: IMU alignment succeeded --> continue next call iteration
      Eigen::Matrix3d R_W_I0_attitude = gtsam::Rot3::Ypr(0.0, imuAttitudePitch, imuAttitudeRoll).matrix();
      Eigen::Isometry3d T_O_Ik_attitude = Eigen::Isometry3d::Identity();
      T_O_Ik_attitude.matrix().block<3, 3>(0, 0) = R_W_I0_attitude;
      preIntegratedNavStatePtr_ = std::make_shared<NavState>(Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity(), T_O_Ik_attitude,
                                                             Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0), imuTimeK);
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
    preIntegratedNavStatePtr_->updateTime(imuTimeK);
    std::cout << "time: " << preIntegratedNavStatePtr_->getTimeK() << std::endl;
    returnPreIntegratedNavStatePtr = std::make_shared<NavState>(*preIntegratedNavStatePtr_);
    return true;
  } else if (!initedGraphFlag_) {  // Case 3: IMU aligned, yaw and position initialized, but graph not yet initialized
    preIntegratedNavStatePtr_->updateTime(imuTimeK);
    initGraph_(imuTimeK);
    std::cout << YELLOW_START << "GMsf" << GREEN_START << " ...graph is initialized." << COLOR_END << std::endl;
    returnPreIntegratedNavStatePtr = std::make_shared<NavState>(*preIntegratedNavStatePtr_);
    return true;
  } else if (!normalOperationFlag_) {  // Case 4: IMU aligned, yaw and position initialized, graph initialized --> normal operation, meaning
                                       // predicting the next state
                                       // via integration
    normalOperationFlag_ = true;
  }

  // Normal operation ------------------------------------------------------------
  bool relocalizeWorldToMapFlag = false;
  // Add IMU factor and get propagated state
  gtsam::NavState T_W_Ik_nav = graphMgrPtr_->addImuFactorAndGetState(imuTimeK, linearAcc, angularVel, relocalizeWorldToMapFlag);

  // Assign poses and velocities ---------------------------------------------------
  preIntegratedNavStatePtr_->update(Eigen::Isometry3d(T_W_Ik_nav.pose().matrix()), T_W_Ik_nav.bodyVelocity(),
                                    graphMgrPtr_->getOptimizedImuBias().correctGyroscope(angularVel), imuTimeK, relocalizeWorldToMapFlag);

  // Return Corresponding State ----------------------------------------------------------------
  returnPreIntegratedNavStatePtr = std::make_shared<NavState>(*preIntegratedNavStatePtr_);

  return true;
}

void GraphMsf::addOdometryMeasurement(const BinaryMeasurement6D& delta) {}  // TODO

void GraphMsf::addUnaryPoseMeasurement(const UnaryMeasurement6D& unary) {
  // Only take actions if graph has been initialized
  if (!initedGraphFlag_) {
    return;
  }

  gtsam::Pose3 T_W_frame(unary.measurementPose());
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
  if (!receivedOdometryFlag_) {
    receivedOdometryFlag_ = true;
  }
}

void GraphMsf::addDualOdometryMeasurement(const UnaryMeasurement6D& odometryKm1, const UnaryMeasurement6D& odometryK,
                                          const Eigen::Matrix<double, 6, 1>& poseBetweenNoise) {
  // Only take actions if graph has been initialized
  if (!initedGraphFlag_) {
    return;
  }

  // Static variables
  static bool lidarUnaryFactorInitialized__ = false;
  static Eigen::Isometry3d T_Wl_Lj__;
  static gtsam::Pose3 T_W_Ij_Graph__;
  static gtsam::Key lastDeltaMeasurementKey__;

  // Create Pseudo Unary Factor
  if (graphMgrPtr_->globalGraphActiveFlag()) {
    /// Reset LiDAR Unary factor intiialization
    lidarUnaryFactorInitialized__ = false;
    // Write current odometry pose to latest delta pose
    T_Wl_Lj__ = odometryK.measurementPose();
  }  //
  else if (graphMgrPtr_->fallbackGraphActiveFlag()) {
    if (!lidarUnaryFactorInitialized__) {
      // Calculate state still from globalGraph
      T_W_Ij_Graph__ = graphMgrPtr_->calculateActiveStateAtKey(lastDeltaMeasurementKey__).pose();
      std::cout << YELLOW_START << "GMsf" << GREEN_START " Initialized LiDAR unary factors." << COLOR_END << std::endl;
      lidarUnaryFactorInitialized__ = true;
    }
    /// Delta pose
    gtsam::Pose3 T_Ij_Ik((staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(), odometryKm1.frameName()) *
                          T_Wl_Lj__.inverse() * odometryK.measurementPose() *
                          staticTransformsPtr_->rv_T_frame1_frame2(odometryK.frameName(), staticTransformsPtr_->getImuFrame()))
                             .matrix());
    gtsam::Pose3 pseudo_T_W_Ik = T_W_Ij_Graph__ * T_Ij_Ik;
    graphMgrPtr_->addPoseUnaryFactorToFallbackGraph(odometryK.timeK(), odometryK.measurementRate(), odometryK.measurementNoise(),
                                                    pseudo_T_W_Ik);
  }
  /// Delta pose
  Eigen::Isometry3d T_Lkm1_Lk(odometryKm1.measurementPose().inverse() * odometryK.measurementPose());
  Eigen::Isometry3d T_Ikm1_Ik = staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(), odometryKm1.frameName()) *
                                T_Lkm1_Lk *
                                staticTransformsPtr_->rv_T_frame1_frame2(odometryK.frameName(), staticTransformsPtr_->getImuFrame());
  lastDeltaMeasurementKey__ = graphMgrPtr_->addPoseBetweenFactorToGlobalGraph(
      odometryKm1.timeK(), odometryK.timeK(), odometryK.measurementRate(), poseBetweenNoise, gtsam::Pose3(T_Ikm1_Ik.matrix()));

  // Optimize
  {
    // Mutex for optimizeGraph Flag
    const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
    optimizeGraphFlag_ = true;
  }

  if (!receivedOdometryFlag_) {
    receivedOdometryFlag_ = true;
  }
}

// Counter is only counted if graph switching attemptGraphSwitching is off
void GraphMsf::addDualGnssPositionMeasurement(const UnaryMeasurement3D& W_t_W_frame, const Eigen::Vector3d& W_t_W_frame_km1,
                                              const Eigen::Vector3d& estCovarianceXYZ, const bool attemptGraphSwitching,
                                              const bool addedYawBefore) {
  // Only take actions if graph has been initialized
  if (!initedGraphFlag_) {
    return;
  }

  // Read covariance
  bool gnssCovarianceViolatedFlagThisTimestep = estCovarianceXYZ(0) > GNSS_COVARIANCE_VIOLATION_THRESHOLD ||
                                                estCovarianceXYZ(1) > GNSS_COVARIANCE_VIOLATION_THRESHOLD ||
                                                estCovarianceXYZ(2) > GNSS_COVARIANCE_VIOLATION_THRESHOLD;
  if (gnssCovarianceViolatedFlagThisTimestep && !gnssCovarianceViolatedFlag_) {
    std::cout << YELLOW_START << "GMsf" << RED_START << " Gnss measurments now ABSENT due to too big covariance." << COLOR_END << std::endl;
    gnssCovarianceViolatedFlag_ = true;
    gnssNotJumpingCounter_ = 0;
  } else if (!gnssCovarianceViolatedFlagThisTimestep && gnssCovarianceViolatedFlag_) {
    std::cout << YELLOW_START << "GMsf" << GREEN_START << " Gnss returned. Low covariance. Now Waiting for GNSS not jumping." << COLOR_END
              << std::endl;
    gnssCovarianceViolatedFlag_ = false;
    gnssNotJumpingCounter_ = 0;
  }

  // Gnss jumping?
  if (!gnssCovarianceViolatedFlag_ && (W_t_W_frame_km1 - W_t_W_frame.measurementVector()).norm() < graphConfigPtr_->gnssOutlierThresold &&
      !attemptGraphSwitching) {
    ++gnssNotJumpingCounter_;
    if (gnssNotJumpingCounter_ == REQUIRED_GNSS_NUM_NOT_JUMPED) {
      std::cout << YELLOW_START << "GMsf" << GREEN_START << " Gnss was not jumping recently. Jumping counter valid again." << COLOR_END
                << std::endl;
    }
  } else if ((W_t_W_frame_km1 - W_t_W_frame.measurementVector()).norm() >= graphConfigPtr_->gnssOutlierThresold) {
    if (gnssNotJumpingCounter_ >= REQUIRED_GNSS_NUM_NOT_JUMPED) {
      std::cout << YELLOW_START << "GMsf" << RED_START << " Gnss was jumping more than the allowed distance of  "
                << graphConfigPtr_->gnssOutlierThresold << "m.  Reset outlier counter." << COLOR_END << std::endl;
    }
    gnssNotJumpingCounter_ = 0;
  }

  // Case: Gnss is good --> Write to graph and perform logic
  if (!gnssCovarianceViolatedFlag_ && (gnssNotJumpingCounter_ == REQUIRED_GNSS_NUM_NOT_JUMPED)) {
    if (attemptGraphSwitching) {
      // Switch graph if desired
      gtsam::Rot3 R_W_I_meas(preIntegratedNavStatePtr_->getT_W_Ik().rotation());
      if (addedYawBefore) {
        R_W_I_meas = gtsam::Rot3::Ypr(lastGnssYaw_W_I, R_W_I_meas.pitch(), R_W_I_meas.roll());
      }
      // Switch graph
      graphMgrPtr_->activateGlobalGraph(W_t_W_Frame1_to_W_t_W_Frame2_(W_t_W_frame.measurementVector(), W_t_W_frame.frameName(),
                                                                      staticTransformsPtr_->getImuFrame(), R_W_I_meas.matrix()),
                                        R_W_I_meas, W_t_W_frame.timeK());
    }
  }
  if (!gnssCovarianceViolatedFlag_ && (gnssNotJumpingCounter_ >= REQUIRED_GNSS_NUM_NOT_JUMPED)) {
    // Position factor
    addGnssPositionMeasurement(W_t_W_frame);
  } else if (graphConfigPtr_->usingFallbackGraphFlag) {
    // Case: Gnss is bad --> Do not write to graph, set flags for odometry unary factor to true
    graphMgrPtr_->activateFallbackGraph();
  }
}

void GraphMsf::addGnssPositionMeasurement(const UnaryMeasurement3D& W_t_W_frame) {
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
  lastGnssYaw_W_I = yawR_W_I.yaw();
}

/// Worker Functions -----------------------
bool GraphMsf::alignImu_(double& imuAttitudeRoll, double& imuAttitudePitch) {
  gtsam::Rot3 imuAttitude;
  static int alignImuCounter__ = -1;
  ++alignImuCounter__;
  if (graphMgrPtr_->estimateAttitudeFromImu(graphConfigPtr_->imuGravityDirection, imuAttitude, gravityConstant_,
                                            graphMgrPtr_->getInitGyrBiasReference())) {
    imuAttitudeRoll = imuAttitude.roll();
    imuAttitudePitch = imuAttitude.pitch();
    std::cout << YELLOW_START << "GMsf" << COLOR_END
              << " Attitude of IMU is initialized. Determined Gravity Magnitude: " << gravityConstant_ << std::endl;
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
  graphMgrPtr_->initImuIntegrators(gravityConstant_, graphConfigPtr_->imuGravityDirection);
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
  while (true) {
    bool optimizeGraphFlag = false;
    // Mutex for optimizeGraph Flag
    {
      // Lock
      const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
      if (optimizeGraphFlag_) {
        optimizeGraphFlag = optimizeGraphFlag_;
        optimizeGraphFlag_ = false;
      }
    }

    if (optimizeGraphFlag) {
      // Get result
      double currentTime;

      gtsam::NavState optimizedState = graphMgrPtr_->updateActiveGraphAndGetState(currentTime);
      // TODO: Write optimizedState

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

}  // namespace graph_msf
