/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// C++
#include <chrono>

// Workspace
#include "graph_msf/core/GraphManager.hpp"
#include "graph_msf/interface/GraphMsf.h"
#include "graph_msf/interface/constants.h"

// Unary Factors
#include "graph_msf/factors/gmsf_expression/GmsfUnaryExpressionPose3.h"
#include "graph_msf/factors/gmsf_expression/GmsfUnaryExpressionPosition3.h"
#include "graph_msf/factors/non_expression/unaryYawFactor.h"

// Binary Factors
// TODO: add binary factors

namespace graph_msf {

// Public -----------------------------------------------------------
/// Constructor -----------
GraphMsf::GraphMsf() {
  REGULAR_COUT << GREEN_START << " Instance created." << COLOR_END << " Waiting for setup() with graphConfiguration and staticTransforms."
               << std::endl;
}

/// Setup ------------
bool GraphMsf::setup() {
  REGULAR_COUT << GREEN_START << " Setting up." << COLOR_END << std::endl;

  // Graph Config
  if (graphConfigPtr_ == nullptr || staticTransformsPtr_ == nullptr) {
    REGULAR_COUT << RED_START << " GraphConfig or StaticTransforms not set. Finishing" << COLOR_END << std::endl;
    throw std::runtime_error("GraphConfig or StaticTransforms not set. Finishing");
    return false;
  }

  // Imu Buffer
  // Initialize IMU buffer
  imuBufferPtr_ = std::make_shared<graph_msf::ImuBuffer>(graphConfigPtr_);

  // Graph Manager
  graphMgrPtr_ =
      std::make_shared<GraphManager>(graphConfigPtr_, staticTransformsPtr_->getImuFrame(), staticTransformsPtr_->getWorldFrame());

  /// Initialize helper threads
  optimizeGraphThread_ = std::thread(&GraphMsf::optimizeGraph_, this);
  REGULAR_COUT << " Initialized thread for optimizing the graph in parallel." << std::endl;

  REGULAR_COUT << GREEN_START << " Set up successfully." << COLOR_END << std::endl;
  return true;
}

// Trigger functions -----------------------
bool GraphMsf::optimizeSlowBatchSmoother(int maxIterations, const std::string& savePath) {
  return graphMgrPtr_->optimizeSlowBatchSmoother(maxIterations, savePath);
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
bool GraphMsf::initYawAndPosition(const double yaw_fixedFrame_frame1, const Eigen::Vector3d& fixedFrame_t_fixedFrame_frame2,
                                  const std::string& fixedFrame, const std::string& frame1, const std::string& frame2) {
  // Locking
  const std::lock_guard<std::mutex> initYawAndPositionLock(initYawAndPositionMutex_);

  // Different Modes
  if (!alignedImuFlag_) {  // Case 1: IMU not yet aligned --> wait for IMU callback to align roll and pitch of IMU

    REGULAR_COUT << RED_START << " Tried to set initial yaw, but initial attitude is not yet set." << COLOR_END << std::endl;
    return false;

  } else if (!areYawAndPositionInited()) {  // Case 2: Imu is aligned, but roll and pitch not yet --> do it
                                            // Transform yaw to imu frame
    REGULAR_COUT << " Preintegrated state before init: " << preIntegratedNavStatePtr_->getT_O_Ik_gravityAligned().matrix() << std::endl;

    // TODO: here assume that world is fixed frame, which is not necessarily the case
    const gtsam::Rot3 yawR_W_frame1 = gtsam::Rot3::Yaw(yaw_fixedFrame_frame1);
    REGULAR_COUT << GREEN_START << " Setting yaw of " << frame1 << " frame in " << fixedFrame << " frame." << COLOR_END << std::endl;
    const double yaw_W_I0_ =
        (yawR_W_frame1 *
         gtsam::Pose3(staticTransformsPtr_->rv_T_frame1_frame2(frame1, staticTransformsPtr_->getImuFrame()).matrix()).rotation())
            .yaw();
    // Set Yaw
    preIntegratedNavStatePtr_->updateYawInWorld(yaw_W_I0_, graphConfigPtr_->odomNotJumpAtStart_);

    // Transform position to imu frame
    Eigen::Matrix3d R_W_I0 = preIntegratedNavStatePtr_->getT_W_Ik().rotation().matrix();
    // TODO: fixedFrame not necessarily world
    Eigen::Vector3d W_t_W_I0 =
        W_t_W_Frame1_to_W_t_W_Frame2_(fixedFrame_t_fixedFrame_frame2, frame2, staticTransformsPtr_->getImuFrame(), R_W_I0);
    // Set Position
    preIntegratedNavStatePtr_->updatePositionInWorld(W_t_W_I0, graphConfigPtr_->odomNotJumpAtStart_);

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
  return initYawAndPosition(T_fixedFrame_frame1.rotation().yaw(), T_fixedFrame_frame1.translation(), unary6DMeasurement.fixedFrameName(),
                            unary6DMeasurement.sensorFrameName(), unary6DMeasurement.sensorFrameName());
}

// Callbacks --------------------------------
/// Main: IMU -----------------------
bool GraphMsf::addImuMeasurementAndGetState(
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
  returnAddedImuMeasurements = imuBufferPtr_->addToImuBuffer(imuTimeK, linearAcc, angularVel);

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
  graphMgrPtr_->addImuFactorAndGetState(*preIntegratedNavStatePtr_, returnOptimizedStateWithCovarianceAndBiasPtr, imuBufferPtr_, imuTimeK,
                                        createNewStateFlag);
  returnPreIntegratedNavStatePtr = std::make_shared<SafeIntegratedNavState>(*preIntegratedNavStatePtr_);

  // Set to normal operation
  if (!normalOperationFlag_) {
    normalOperationFlag_ = true;
  }

  // Return
  return true;
}

// Pose3
void GraphMsf::addUnaryPose3Measurement(const UnaryMeasurementXD<Eigen::Isometry3d, 6>& T_fixedFrame_sensorFrame) {
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
    GmsfUnaryExpressionPose3 gmsfUnaryExpressionPose3(
        std::make_shared<UnaryMeasurementXD<Eigen::Isometry3d, 6>>(T_fixedFrame_sensorFrame), staticTransformsPtr_->getWorldFrame(),
        staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(), T_fixedFrame_sensorFrame.sensorFrameName()));

    // Add factor to graph
    graphMgrPtr_->addUnaryGmsfExpressionFactor<gtsam::Pose3>(std::make_shared<GmsfUnaryExpressionPose3>(gmsfUnaryExpressionPose3));

    // Optimize ---------------------------------------------------------------
    {
      // Mutex for optimizeGraph Flag
      const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
      optimizeGraphFlag_ = true;
    }
  }
}

/// Position3
void GraphMsf::addUnaryPosition3Measurement(UnaryMeasurementXD<Eigen::Vector3d, 3>& fixedFrame_t_fixedFrame_sensorFrame) {
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

    //    std::cout << "Adding unary position measurement in frame " << fixedFrame_t_fixedFrame_sensorFrame.sensorFrameName()
    //              << " with covariance " << fixedFrame_t_fixedFrame_sensorFrame.unaryMeasurementNoiseDensity() << std::endl;

    // Create GMSF expression
    GmsfUnaryExpressionPosition3 gmsfUnaryExpressionPosition3(
        std::make_shared<UnaryMeasurementXD<Eigen::Vector3d, 3>>(fixedFrame_t_fixedFrame_sensorFrame),
        staticTransformsPtr_->getWorldFrame(),
        staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(),
                                                 fixedFrame_t_fixedFrame_sensorFrame.sensorFrameName()));

    // Add factor to graph
    graphMgrPtr_->addUnaryGmsfExpressionFactor<gtsam::Vector3>(
        std::make_shared<GmsfUnaryExpressionPosition3>(gmsfUnaryExpressionPosition3));

    // Optimize ---------------------------------------------------------------
    {
      // Mutex for optimizeGraph Flag
      const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
      optimizeGraphFlag_ = true;
    }
  }
}

// Velocity3
void GraphMsf::addUnaryVelocity3Measurement(UnaryMeasurementXD<Eigen::Vector3d, 3>& F_v_F_S) {
  throw std::runtime_error("Velocity measurements are not yet supported.");
}

// Velocity3 in Body Frame
void addUnaryVelocity3BodyMeasurement(UnaryMeasurementXD<Eigen::Vector3d, 3>& S_v_F_S) {
  throw std::runtime_error("Velocity measurements are not yet supported.");
}

// Roll
bool GraphMsf::addUnaryRollMeasurement(const UnaryMeasurementXD<double, 1>& roll_W_frame) {
  throw std::runtime_error("Roll measurements are not yet supported.");
}

// Pitch
bool GraphMsf::addUnaryPitchMeasurement(const UnaryMeasurementXD<double, 1>& pitch_W_frame) {
  throw std::runtime_error("Pitch measurements are not yet supported.");
}

// Yaw
bool GraphMsf::addUnaryYawMeasurement(const UnaryMeasurementXD<double, 1>& yaw_W_frame) {
  // Only take actions if graph has been initialized
  if (!initedGraphFlag_) {
    return false;
  }

  // Check for covariance violation
  bool covarianceViolatedFlag =
      isCovarianceViolated_<1>(yaw_W_frame.unaryMeasurementNoiseDensity(), yaw_W_frame.covarianceViolationThreshold());

  // Transform yaw to imu frame
  gtsam::Rot3 yawR_W_frame = gtsam::Rot3::Yaw(yaw_W_frame.unaryMeasurement());
  gtsam::Rot3 yawR_W_I =
      yawR_W_frame *
      gtsam::Rot3(staticTransformsPtr_->rv_T_frame1_frame2(yaw_W_frame.sensorFrameName(), staticTransformsPtr_->getImuFrame()).rotation());

  // Add factor
  if (!covarianceViolatedFlag) {
    graphMgrPtr_->addUnaryFactorInImuFrame<double, 1, YawFactor, gtsam::symbol_shorthand::X>(
        yawR_W_I.yaw(), yaw_W_frame.unaryMeasurementNoiseDensity(), yaw_W_frame.timeK());
    {
      // Mutex for optimizeGraph Flag
      const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
      optimizeGraphFlag_ = true;
    }
    return true;
  } else {
    return false;
  }
}

// Binary Measurements
void GraphMsf::addBinaryPoseMeasurement(const BinaryMeasurementXD<Eigen::Isometry3d, 6>& deltaMeasurement) {
  // Valid measurement received
  if (!validFirstMeasurementReceivedFlag_) {
    validFirstMeasurementReceivedFlag_ = true;
  }

  // Only take actions if graph has been initialized
  if (!initedGraphFlag_) {
    return;
  }

  Eigen::Isometry3d T_fkm1_fk = deltaMeasurement.deltaMeasurement();

  // Check frame of measuremnts
  if (deltaMeasurement.measurementName() != staticTransformsPtr_->getImuFrame()) {
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
  if (imuBufferPtr_->estimateAttitudeFromImu(R_W_I_rollPitch, estimatedGravityMagnitude, graphMgrPtr_->getInitGyrBiasReference())) {
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

/// Convenience Functions -----------------------
template <int DIM>
bool GraphMsf::isCovarianceViolated_(const Eigen::Matrix<double, DIM, 1>& covariance, const double covarianceViolationThreshold) {
  if (covarianceViolationThreshold > 0.0) {
    for (int i = 0; i < DIM; i++) {
      if (covariance(i) > covarianceViolationThreshold) {
        return true;
      }
    }
  }
  return false;
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
