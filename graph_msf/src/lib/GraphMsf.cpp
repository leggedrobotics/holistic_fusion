/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// C++
#include <chrono>

// Implementation
#include "graph_msf/interface/GraphMsf.h"

// Workspace
#include "graph_msf/core/GraphManager.h"
#include "graph_msf/interface/constants.h"

namespace graph_msf {

// Public -----------------------------------------------------------
/// Constructor -----------
GraphMsf::GraphMsf() {
  REGULAR_COUT << GREEN_START << " GraphMsf-Constructor called." << COLOR_END << std::endl;
}

void GraphMsf::setup(const std::shared_ptr<GraphConfig> graphConfigPtr, const std::shared_ptr<StaticTransforms> staticTransformsPtr) {
  REGULAR_COUT << GREEN_START << " GraphMsf-Setup called." << COLOR_END << std::endl;

  // Check if setup has been called before
  if (graphConfigPtr == nullptr || staticTransformsPtr == nullptr) {
    throw std::runtime_error("setup() of inheriting classes did not  has not been called before.");
  } else {
    graphConfigPtr_ = graphConfigPtr;
    staticTransformsPtr_ = staticTransformsPtr;
  }

  // Imu Buffer
  // Initialize IMU buffer
  coreImuBufferPtr_ = std::make_shared<graph_msf::ImuBuffer>(graphConfigPtr_);

  // Graph Manager
  graphMgrPtr_ =
      std::make_shared<GraphManager>(graphConfigPtr_, staticTransformsPtr_->getImuFrame(), staticTransformsPtr_->getWorldFrame());

  /// Initialize helper threads
  optimizeGraphThread_ = std::thread(&GraphMsf::optimizeGraph_, this);
  REGULAR_COUT << " Initialized thread for optimizing the graph in parallel." << std::endl;
}

// Trigger functions -----------------------
bool GraphMsf::optimizeSlowBatchSmoother(int maxIterations, const std::string& savePath, const bool saveCovarianceFlag) {
  return graphMgrPtr_->optimizeSlowBatchSmoother(maxIterations, savePath, saveCovarianceFlag);
}

bool GraphMsf::logRealTimeStates(const std::string& savePath) {
  // String of time without line breaks: year_month_day_hour_min_sec
  std::ostringstream oss;
  std::time_t now_time_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::tm now_tm = *std::localtime(&now_time_t);
  oss << std::put_time(&now_tm, "%Y_%m_%d_%H_%M_%S");
  // Convert stream to string
  std::string timeString = oss.str();
  // Return
  return graphMgrPtr_->logRealTimeStates(savePath, timeString);
}

// Getter functions -----------------------
bool GraphMsf::areYawAndPositionInited() const {
  return foundInitialYawAndPositionFlag_;
}

bool GraphMsf::areRollAndPitchInited() const {
  return alignedImuFlag_;
}

bool GraphMsf::isGraphInited() const {
  return initedGraphFlag_;
}

// Initialization -----------------------
bool GraphMsf::initYawAndPositionInWorld(const double yaw_fixedFrame_frame1, const Eigen::Vector3d& fixedFrame_t_fixedFrame_frame2,
                                         const std::string& frame1, const std::string& frame2) {
  // Locking
  const std::lock_guard<std::mutex> initYawAndPositionLock(initYawAndPositionMutex_);

  // Different Modes
  if (!alignedImuFlag_) {  // Case 1: IMU not yet aligned --> wait for IMU callback to align roll and pitch of IMU

    REGULAR_COUT << RED_START << " Tried to set initial yaw, but initial attitude is not yet set." << COLOR_END << std::endl;
    return false;

  } else if (!areYawAndPositionInited()) {  // Case 2: Imu is aligned, but roll and pitch not yet --> do it
                                            // Transform yaw to imu frame
    REGULAR_COUT << " Pre-integrated state before init: " << preIntegratedNavStatePtr_->getT_O_Ik_gravityAligned().matrix() << std::endl;

    const gtsam::Rot3 yawR_W_frame1 = gtsam::Rot3::Yaw(yaw_fixedFrame_frame1);
    REGULAR_COUT << GREEN_START << " Setting yaw of " << frame1 << " frame in " << staticTransformsPtr_->getWorldFrame() << " frame."
                 << COLOR_END << std::endl;
    const double yaw_W_I0_ =
        (yawR_W_frame1 *
         gtsam::Pose3(staticTransformsPtr_->rv_T_frame1_frame2(frame1, staticTransformsPtr_->getImuFrame()).matrix()).rotation())
            .yaw();
    // Set Yaw
    preIntegratedNavStatePtr_->updateYawInWorld(yaw_W_I0_, graphConfigPtr_->odomNotJumpAtStartFlag_);

    // Transform position to imu frame
    Eigen::Matrix3d R_W_I0 = preIntegratedNavStatePtr_->getT_W_Ik().rotation().matrix();
    // TODO: fixedFrame not necessarily world
    Eigen::Vector3d W_t_W_I0 =
        W_t_W_Frame1_to_W_t_W_Frame2_(fixedFrame_t_fixedFrame_frame2, frame2, staticTransformsPtr_->getImuFrame(), R_W_I0);
    // Set Position
    preIntegratedNavStatePtr_->updatePositionInWorld(W_t_W_I0, graphConfigPtr_->odomNotJumpAtStartFlag_);

    REGULAR_COUT << " Preintegrated state after init: " << preIntegratedNavStatePtr_->getT_O_Ik_gravityAligned().matrix() << std::endl;

    // Wrap Up
    foundInitialYawAndPositionFlag_ = true;
    // World Frame
    REGULAR_COUT << " --------------------" << std::endl;
    REGULAR_COUT << GREEN_START << " Initial global yaw of from world frame to imu frame has been set to (deg) " << 180.0 * yaw_W_I0_ / M_PI
                 << "." << COLOR_END << std::endl;
    REGULAR_COUT << GREEN_START << " Initial global position of imu frame in world frame has been set to (m) " << W_t_W_I0.transpose()
                 << "." << COLOR_END << std::endl;
    // Odom Frame
    const double& yaw_O_I0 = gtsam::Rot3(preIntegratedNavStatePtr_->getT_O_Ik_gravityAligned().rotation().matrix()).yaw();  // alias
    REGULAR_COUT << " --------------------" << std::endl;
    REGULAR_COUT << GREEN_START << " Initial global yaw of from odom frame to imu frame has been set to (deg) " << 180.0 * yaw_O_I0 / M_PI
                 << "." << COLOR_END << std::endl;
    REGULAR_COUT << GREEN_START << " Initial global position of imu frame in odom frame has been set to (m) "
                 << preIntegratedNavStatePtr_->getT_O_Ik_gravityAligned().translation().transpose() << "." << COLOR_END << std::endl;
    // Return
    return true;
  } else {  // Case 3: Initial yaw and position already set --> do nothing
    REGULAR_COUT << RED_START << " Tried to set initial yaw, but it has been set before." << COLOR_END << std::endl;
    return false;
  }
}

bool GraphMsf::initYawAndPosition(const UnaryMeasurementXD<Eigen::Isometry3d, 6>& unary6DMeasurement) {
  gtsam::Pose3 T_fixedFrame_frame1(unary6DMeasurement.unaryMeasurement().matrix());
  return initYawAndPositionInWorld(T_fixedFrame_frame1.rotation().yaw(), T_fixedFrame_frame1.translation(),
                                   unary6DMeasurement.sensorFrameName(), unary6DMeasurement.sensorFrameName());
}

// Adders --------------------------------
/// Main: IMU -----------------------
bool GraphMsf::addCoreImuMeasurementAndGetState(
    const Eigen::Vector3d& linearAcc, const Eigen::Vector3d& angularVel, const double imuTimeK,
    std::shared_ptr<SafeIntegratedNavState>& returnPreIntegratedNavStatePtr,
    std::shared_ptr<SafeNavStateWithCovarianceAndBias>& returnOptimizedStateWithCovarianceAndBiasPtr,
    Eigen::Matrix<double, 6, 1>& returnAddedImuMeasurements) {
  // Setup -------------------------
  // Increase counter
  ++imuCallbackCounter_;

  // First Iteration
  if (preIntegratedNavStatePtr_ == nullptr) {
    preIntegratedNavStatePtr_ = std::make_shared<SafeIntegratedNavState>();
    preIntegratedNavStatePtr_->updateLatestMeasurementTimestamp(imuTimeK);
  }

  // Filter out imu messages with same time stamp
  if (std::abs(imuTimeK - preIntegratedNavStatePtr_->getTimeK()) < 1e-8 && imuCallbackCounter_ > 1) {
    REGULAR_COUT << RED_START << " Imu time " << std::setprecision(14) << imuTimeK << " was repeated." << COLOR_END << std::endl;
    return false;
  }

  // Add measurement to buffer
  returnAddedImuMeasurements = coreImuBufferPtr_->addToImuBuffer(imuTimeK, linearAcc, angularVel);

  // Locking
  const std::lock_guard<std::mutex> initYawAndPositionLock(initYawAndPositionMutex_);

  // State Machine in form of if-else statements -----------------
  if (!alignedImuFlag_) {  // Case 1: IMU not aligned
    // Try to align
    double imuAttitudeRoll, imuAttitudePitch = 0.0;
    if (!alignImu_(imuAttitudeRoll, imuAttitudePitch)) {  // Case 1.1: IMU alignment failed --> try again next time
      // Print only once per second
      if (imuCallbackCounter_ % int(graphConfigPtr_->imuRate_) == 0) {
        REGULAR_COUT << " NOT ENOUGH IMU MESSAGES TO INITIALIZE POSE. WAITING FOR MORE..." << std::endl;
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
      // Set yaw of base frame to zero
      R_W_Init0_attitude = gtsam::Rot3::Ypr(0.0, R_W_Init0_attitude.pitch(), R_W_Init0_attitude.roll());
      R_W_I0_attitude =
          R_W_Init0_attitude.matrix() *
          staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getInitializationFrame(), staticTransformsPtr_->getImuFrame())
              .rotation()
              .matrix();
      Eigen::Isometry3d T_O_Ik_attitude = Eigen::Isometry3d::Identity();
      T_O_Ik_attitude.matrix().block<3, 3>(0, 0) = R_W_I0_attitude;
      Eigen::Vector3d O_t_O_Ik =
          Eigen::Vector3d(0, 0, 0) -
          R_W_I0_attitude *
              staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(), staticTransformsPtr_->getInitializationFrame())
                  .translation();
      T_O_Ik_attitude.matrix().block<3, 1>(0, 3) = O_t_O_Ik;
      REGULAR_COUT << " Setting zero position of " << staticTransformsPtr_->getInitializationFrame()
                   << ", hence iniital position of IMU is: " << O_t_O_Ik.transpose() << std::endl;
      Eigen::Vector3d zeroPVeloctiy = Eigen::Vector3d(0, 0, 0);
      preIntegratedNavStatePtr_ = std::make_shared<SafeIntegratedNavState>(T_O_Ik_attitude, zeroPVeloctiy, zeroPVeloctiy, imuTimeK);
      REGULAR_COUT << GREEN_START << " IMU aligned. Initial pre-integrated state in odom frame: "
                   << preIntegratedNavStatePtr_->getT_O_Ik_gravityAligned().matrix() << COLOR_END << std::endl;
      alignedImuFlag_ = true;
      return false;
    }
  } else if (!areYawAndPositionInited()) {  // Case 2: IMU aligned, but yaw and position not initialized, waiting for external
    // initialization, meanwhile publishing initial roll and pitch
    // Printing every second
    if (imuCallbackCounter_ % int(graphConfigPtr_->imuRate_) == 0) {
      REGULAR_COUT << " IMU callback waiting for initialization of global yaw and initial position." << std::endl;
    }
    // Publish state with correct roll and pitch, nothing has changed compared to Case 1.2
    preIntegratedNavStatePtr_->updateLatestMeasurementTimestamp(imuTimeK);
    returnPreIntegratedNavStatePtr = std::make_shared<SafeIntegratedNavState>(*preIntegratedNavStatePtr_);
    return true;
  } else if (!validFirstMeasurementReceivedFlag_) {  // Case 3: No valid measurement received yet
    if (imuCallbackCounter_ % int(graphConfigPtr_->imuRate_) == 0) {
      REGULAR_COUT << RED_START << " IMU callback waiting for first valid measurement before initializing graph." << COLOR_END << std::endl;
    }
    preIntegratedNavStatePtr_->updateLatestMeasurementTimestamp(imuTimeK);
    returnPreIntegratedNavStatePtr = std::make_shared<SafeIntegratedNavState>(*preIntegratedNavStatePtr_);
    return true;
  } else if (!initedGraphFlag_) {  // Case 4: IMU aligned, yaw and position initialized, valid measurement received, but graph not yet
                                   // initialized
    preIntegratedNavStatePtr_->updateLatestMeasurementTimestamp(imuTimeK);
    initGraph_(imuTimeK);
    returnPreIntegratedNavStatePtr = std::make_shared<SafeIntegratedNavState>(*preIntegratedNavStatePtr_);
    REGULAR_COUT << GREEN_START << " ...graph is initialized." << COLOR_END << std::endl;
    return true;
  }

  // Case 5: Normal operation, meaning predicting the next state via integration -------------
  // Only create state every n-th measurements (or at first successful iteration)
  bool createNewStateFlag = imuCallbackCounter_ % graphConfigPtr_->createStateEveryNthImuMeasurement_ == 0 || !normalOperationFlag_;
  // Add IMU factor and return propagated & optimized state
  graphMgrPtr_->addImuFactorAndGetState(*preIntegratedNavStatePtr_, returnOptimizedStateWithCovarianceAndBiasPtr, coreImuBufferPtr_,
                                        imuTimeK, createNewStateFlag);
  returnPreIntegratedNavStatePtr = std::make_shared<SafeIntegratedNavState>(*preIntegratedNavStatePtr_);

  // Set to normal operation
  if (!normalOperationFlag_) {
    normalOperationFlag_ = true;
  }

  // Return
  return true;
}

// Ambiguous Measurements -----------------------
bool GraphMsf::addZeroMotionFactor(double timeKm1, double timeK, double noiseDensity) {
  static_cast<void>(graphMgrPtr_->addPoseBetweenFactor(gtsam::Pose3::Identity(), noiseDensity * Eigen::Matrix<double, 6, 1>::Ones(),
                                                       timeKm1, timeK, 10, RobustNormEnum::None, 0.0));
  graphMgrPtr_->addUnaryFactorInImuFrame<gtsam::Vector3, 3, gtsam::PriorFactor<gtsam::Vector3>, gtsam::symbol_shorthand::V>(
      gtsam::Vector3::Zero(), noiseDensity * Eigen::Matrix<double, 3, 1>::Ones(), timeK);

  return true;
}

bool GraphMsf::addZeroVelocityFactor(double timeK, double noiseDensity) {
  graphMgrPtr_->addUnaryFactorInImuFrame<gtsam::Vector3, 3, gtsam::PriorFactor<gtsam::Vector3>, gtsam::symbol_shorthand::V>(
      gtsam::Vector3::Zero(), noiseDensity * Eigen::Matrix<double, 3, 1>::Ones(), timeK);

  return true;
}

// Private ---------------------------------------------------------------

/// Worker Functions -----------------------
bool GraphMsf::alignImu_(double& imuAttitudeRoll, double& imuAttitudePitch) {
  gtsam::Rot3 R_W_I_rollPitch;
  static int alignImuCounter__ = -1;
  ++alignImuCounter__;
  double estimatedGravityMagnitude;
  if (coreImuBufferPtr_->estimateAttitudeFromImu(R_W_I_rollPitch, estimatedGravityMagnitude, graphMgrPtr_->getInitGyrBiasReference())) {
    imuAttitudeRoll = R_W_I_rollPitch.roll();
    imuAttitudePitch = R_W_I_rollPitch.pitch();
    if (graphConfigPtr_->estimateGravityFromImuFlag_) {
      graphConfigPtr_->gravityMagnitude_ = estimatedGravityMagnitude;
      REGULAR_COUT << " Attitude of IMU is initialized. Determined Gravity Magnitude: " << estimatedGravityMagnitude << std::endl;
    } else {
      REGULAR_COUT << " Estimated gravity magnitude from IMU is: " << estimatedGravityMagnitude << std::endl;
      REGULAR_COUT << " This gravity is not used, because estimateGravityFromImu is set to false. Gravity set to "
                   << graphConfigPtr_->gravityMagnitude_ << "." << std::endl;
      gtsam::Vector3 gravityVector = gtsam::Vector3(0, 0, graphConfigPtr_->gravityMagnitude_);
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
  const gtsam::Pose3& T_W_I0 = gtsam::Pose3(preIntegratedNavStatePtr_->getT_W_Ik().matrix());
  const gtsam::Pose3& T_O_I0 = gtsam::Pose3(preIntegratedNavStatePtr_->getT_O_Ik_gravityAligned().matrix());
  // Print
  REGULAR_COUT << GREEN_START << " Total initial IMU attitude is RPY (deg): " << T_W_I0.rotation().rpy().transpose() * (180.0 / M_PI)
               << COLOR_END << std::endl;

  // Gravity
  graphMgrPtr_->initImuIntegrators(graphConfigPtr_->gravityMagnitude_);
  /// Initialize graph node
  graphMgrPtr_->initPoseVelocityBiasGraph(timeStamp_k, T_W_I0, T_O_I0);

  // Read initial pose from graph for optimized pose
  gtsam::Pose3 T_W_I0_opt = graphMgrPtr_->getOptimizedGraphState().navState().pose();
  REGULAR_COUT << GREEN_START
               << " INITIAL POSE of IMU in world frame after first optimization, x,y,z (m): " << T_W_I0_opt.translation().transpose()
               << ", RPY (deg): " << T_W_I0_opt.rotation().rpy().transpose() * (180.0 / M_PI) << COLOR_END << std::endl;
  REGULAR_COUT << GREEN_START << " INITIAL position of IMU in odom frame after first optimization, x,y,z (m): "
               << preIntegratedNavStatePtr_->getT_O_Ik_gravityAligned().translation().transpose() << std::endl;
  REGULAR_COUT << " Factor graph key of very first node: " << graphMgrPtr_->getPropagatedStateKey() << std::endl;

  // Set flag
  initedGraphFlag_ = true;
}

void GraphMsf::optimizeGraph_() {
  // While loop
  REGULAR_COUT << " Thread for updating graph is ready." << std::endl;
  double lastOptimizedTime = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
  double currentTime = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
  bool optimizedAtLeastOnce = false;
  while (true) {
    bool optimizeGraphFlag = false;
    // Mutex for optimizeGraph Flag
    {
      // Lock
      const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
      currentTime = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
      // Optimize at most at the rate of maxOptimizationFrequency but at least every second
      if ((optimizeGraphFlag_ && ((currentTime - lastOptimizedTime) > (1.0 / graphConfigPtr_->maxOptimizationFrequency_))) ||
          ((currentTime - lastOptimizedTime) > (1.0 / graphConfigPtr_->minOptimizationFrequency_) && optimizedAtLeastOnce)) {
        optimizeGraphFlag = true;
        lastOptimizedTime = currentTime;
        optimizedAtLeastOnce = true;
        optimizeGraphFlag_ = false;
      }
    }

    // Optimize
    if (optimizeGraphFlag) {
      graphMgrPtr_->updateGraph();
    }  // else just sleep for a short amount of time before polling again
    else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

/// Utility Functions -----------------------
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
