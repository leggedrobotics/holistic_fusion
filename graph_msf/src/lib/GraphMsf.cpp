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
    std::runtime_error("GraphConfig or StaticTransforms not set. Finishing");
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

bool GraphMsf::areYawAndPositionInited() {
  return foundInitialYawAndPositionFlag_;
}

bool GraphMsf::areRollAndPitchInited() {
  return alignedImuFlag_;
}

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
    // TODO: here assume that world is fixed frame, which is not necessarily the case
    gtsam::Rot3 yawR_W_frame1 = gtsam::Rot3::Yaw(yaw_fixedFrame_frame1);
    REGULAR_COUT << GREEN_START << " Setting yaw in " << frame1 << " frame." << COLOR_END << std::endl;
    double yaw_W_I0_ =
        (yawR_W_frame1 *
         gtsam::Pose3(staticTransformsPtr_->rv_T_frame1_frame2(frame1, staticTransformsPtr_->getImuFrame()).matrix()).rotation())
            .yaw();
    // Set Yaw
    preIntegratedNavStatePtr_->updateYawInWorld(yaw_W_I0_, graphConfigPtr_->reLocalizeWorldToMapAtStartFlag);

    // Transform position to imu frame
    Eigen::Matrix3d R_W_I0 = preIntegratedNavStatePtr_->getT_W_Ik().rotation().matrix();
    // TODO: fixedFrame not necessarily world
    Eigen::Vector3d W_t_W_I0 =
        W_t_W_Frame1_to_W_t_W_Frame2_(fixedFrame_t_fixedFrame_frame2, frame2, staticTransformsPtr_->getImuFrame(), R_W_I0);
    // Set Position
    preIntegratedNavStatePtr_->updatePositionInWorld(W_t_W_I0, graphConfigPtr_->reLocalizeWorldToMapAtStartFlag);

    // Wrap Up
    foundInitialYawAndPositionFlag_ = true;
    REGULAR_COUT << GREEN_START << " Initial global yaw of from world frame to imu frame has been set to (deg) " << 180.0 * yaw_W_I0_ / M_PI
                 << "." << COLOR_END << std::endl;
    REGULAR_COUT << GREEN_START << " Initial global position of imu frame in world frame has been set to (m) " << W_t_W_I0.transpose()
                 << "." << COLOR_END << std::endl;
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

// Private ---------------------------------------------------------------
/// Callbacks -----------------------
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
      if (imuCallbackCounter_ % int(graphConfigPtr_->imuRate) == 0) {
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
      T_O_Ik_attitude.matrix().block<3, 1>(0, 3) =
          Eigen::Vector3d(0, 0, 0) -
          R_W_I0_attitude *
              staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(), staticTransformsPtr_->getInitializationFrame())
                  .translation();
      Eigen::Vector3d zeroPVeloctiy = Eigen::Vector3d(0, 0, 0);
      preIntegratedNavStatePtr_ = std::make_shared<SafeIntegratedNavState>(T_O_Ik_attitude, zeroPVeloctiy, zeroPVeloctiy, imuTimeK);
      alignedImuFlag_ = true;
      return false;
    }
  } else if (!areYawAndPositionInited()) {  // Case 2: IMU aligned, but yaw and position not initialized, waiting for external
    // initialization, meanwhile publishing initial roll and pitch
    // Printing every second
    if (imuCallbackCounter_ % int(graphConfigPtr_->imuRate) == 0) {
      REGULAR_COUT << " IMU callback waiting for initialization of global yaw and initial position." << std::endl;
    }
    // Publish state with correct roll and pitch, nothing has changed compared to Case 1.2
    preIntegratedNavStatePtr_->updateLatestMeasurementTimestamp(imuTimeK);
    returnPreIntegratedNavStatePtr = std::make_shared<SafeIntegratedNavState>(*preIntegratedNavStatePtr_);
    return true;
  } else if (!validFirstMeasurementReceivedFlag_) {  // Case 3: No valid measurement received yet, e.g. because GNSS Covariance is too high
    REGULAR_COUT << RED_START << " ...waiting for first valid measurement before initializing graph." << COLOR_END << std::endl;
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
  } else if (!normalOperationFlag_) {  // Case 5: IMU aligned, yaw and position initialized, graph initialized --> normal operation, meaning
                                       // predicting the next state
                                       // via integration
    normalOperationFlag_ = true;
  }
  // Case 6: Normal operation, meaning predicting the next state via integration -------------
  // Add IMU factor and return propagated & optimized state
  graphMgrPtr_->addImuFactorAndGetState(*preIntegratedNavStatePtr_, returnOptimizedStateWithCovarianceAndBiasPtr, imuBufferPtr_, imuTimeK);
  returnPreIntegratedNavStatePtr = std::make_shared<SafeIntegratedNavState>(*preIntegratedNavStatePtr_);

  return true;
}

void GraphMsf::addOdometryMeasurement(const BinaryMeasurementXD<Eigen::Isometry3d, 6>& deltaMeasurement) {
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
    staticTransformsPtr_->rv_T_frame1_frame2(deltaMeasurement.sensorFrameName(), staticTransformsPtr_->getImuFrame());
  }

  const gtsam::Key keyAtMeasurementK =
      graphMgrPtr_->addPoseBetweenFactor(gtsam::Pose3(T_fkm1_fk.matrix()), deltaMeasurement.measurementNoiseDensity(),
                                         deltaMeasurement.timeKm1(), deltaMeasurement.timeK(), deltaMeasurement.measurementRate());

  // Optimize
  {
    // Mutex for optimizeGraph Flag
    const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
    optimizeGraphFlag_ = true;
  }
}

void GraphMsf::addUnaryPoseMeasurement(const UnaryMeasurementXD<Eigen::Isometry3d, 6>& unary6DMeasurement) {
  // Valid measurement received
  if (!validFirstMeasurementReceivedFlag_) {
    validFirstMeasurementReceivedFlag_ = true;
  }

  // Only take actions if graph has been initialized
  if (!initedGraphFlag_) {
    return;
  }

  if (initedGraphFlag_) {
    graphMgrPtr_->addPoseUnaryFactor(unary6DMeasurement, staticTransformsPtr_->rv_T_frame1_frame2(unary6DMeasurement.sensorFrameName(),
                                                                                                  staticTransformsPtr_->getImuFrame()));
  }
  // Optimize ---------------------------------------------------------------
  {
    // Mutex for optimizeGraph Flag
    const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
    optimizeGraphFlag_ = true;
  }
}

bool GraphMsf::addPositionMeasurement(UnaryMeasurementXD<Eigen::Vector3d, 3>& fixedFrame_t_fixedFrame_sensorFrame) {
  // Valid measurement received
  if (!validFirstMeasurementReceivedFlag_) {
    validFirstMeasurementReceivedFlag_ = true;
  }

  // Only take actions if graph has been initialized
  if (!initedGraphFlag_) {
    return false;
  }

  // Check whether IMU has been added already
  if (graphMgrPtr_->getPropagatedStateKey() == 0) {
    return false;
  }

  // Check for covariance violation
  bool covarianceViolatedFlag = isCovarianceViolated_<3>(fixedFrame_t_fixedFrame_sensorFrame.unaryMeasurementNoiseDensity(),
                                                         fixedFrame_t_fixedFrame_sensorFrame.covarianceViolationThreshold());

  // Add factor
  if (!covarianceViolatedFlag) {
    // Transform to IMU frame in case lever should not be considered
    if (!graphConfigPtr_->optimizeWithImuToSensorLeverArm) {
      if (fixedFrame_t_fixedFrame_sensorFrame.fixedFrameName() != staticTransformsPtr_->getWorldFrame()) {
        throw std::runtime_error("Optimization without considering lever is only supported for positions expressed in world frame.");
      }
      fixedFrame_t_fixedFrame_sensorFrame.lv_unaryMeasurement() = W_t_W_Frame1_to_W_t_W_Frame2_(
          fixedFrame_t_fixedFrame_sensorFrame.unaryMeasurement(), fixedFrame_t_fixedFrame_sensorFrame.sensorFrameName(),
          staticTransformsPtr_->getImuFrame(), preIntegratedNavStatePtr_->getT_W_Ik().rotation());
      fixedFrame_t_fixedFrame_sensorFrame.lv_sensorFrameName() = staticTransformsPtr_->getImuFrame();
      // Add factor
      graphMgrPtr_->addPositionUnaryFactor(fixedFrame_t_fixedFrame_sensorFrame);
    } else {  // Consider lever
      // Add factor
      graphMgrPtr_->addPositionUnaryFactor(
          fixedFrame_t_fixedFrame_sensorFrame,
          staticTransformsPtr_
              ->rv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(), fixedFrame_t_fixedFrame_sensorFrame.sensorFrameName())
              .translation());
    }
    // Mutex for optimizeGraph Flag
    const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
    optimizeGraphFlag_ = true;
    return true;
  } else {
    return false;
  }
}

bool GraphMsf::addHeadingMeasurement(const UnaryMeasurementXD<double, 1>& yaw_W_frame) {
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
    graphMgrPtr_->addHeadingUnaryFactor(yawR_W_I.yaw(), yaw_W_frame.unaryMeasurementNoiseDensity(), yaw_W_frame.timeK());
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

bool GraphMsf::addZeroMotionFactor(double maxTimestampDistance, double timeKm1, double timeK, const gtsam::Pose3 pose) {
  // Find corresponding keys in graph
  gtsam::Key closestKeyKm1, closestKeyK;

  graphMgrPtr_->addPoseBetweenFactor(gtsam::Pose3::Identity(), 1e-3 * Eigen::Matrix<double, 6, 1>::Ones(), timeKm1, timeK, 10);
  graphMgrPtr_->addVelocityUnaryFactor(gtsam::Vector3::Zero(), 1e-3 * Eigen::Matrix<double, 3, 1>::Ones(), timeKm1);

  return true;
}

/// Worker Functions -----------------------
bool GraphMsf::alignImu_(double& imuAttitudeRoll, double& imuAttitudePitch) {
  gtsam::Rot3 R_W_I_rollPitch;
  static int alignImuCounter__ = -1;
  ++alignImuCounter__;
  double estimatedGravityMagnitude;
  if (imuBufferPtr_->estimateAttitudeFromImu(R_W_I_rollPitch, estimatedGravityMagnitude, graphMgrPtr_->getInitGyrBiasReference())) {
    imuAttitudeRoll = R_W_I_rollPitch.roll();
    imuAttitudePitch = R_W_I_rollPitch.pitch();
    if (graphConfigPtr_->estimateGravityFromImuFlag) {
      graphConfigPtr_->gravityMagnitude = estimatedGravityMagnitude;
      REGULAR_COUT << " Attitude of IMU is initialized. Determined Gravity Magnitude: " << estimatedGravityMagnitude << std::endl;
    } else {
      REGULAR_COUT << " Estimated gravity magnitude from IMU is: " << estimatedGravityMagnitude << std::endl;
      REGULAR_COUT << " This gravity is not used, because estimateGravityFromImu is set to false. Gravity set to "
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
  REGULAR_COUT << GREEN_START
               << " Total initial IMU attitude is Yaw/Pitch/Roll(deg): " << T_W_I0.rotation().ypr().transpose() * (180.0 / M_PI)
               << COLOR_END << std::endl;

  // Gravity
  graphMgrPtr_->initImuIntegrators(graphConfigPtr_->gravityMagnitude);
  /// Initialize graph node
  graphMgrPtr_->initPoseVelocityBiasGraph(timeStamp_k, T_W_I0);

  // Read initial pose from graph
  T_W_I0 = graphMgrPtr_->getOptimizedGraphState().navState().pose();
  REGULAR_COUT << GREEN_START
               << " INITIAL POSE of IMU in world frame after first optimization, x,y,z (m): " << T_W_I0.translation().transpose()
               << ", RPY (deg): " << T_W_I0.rotation().rpy().transpose() * (180.0 / M_PI) << COLOR_END << std::endl;
  REGULAR_COUT << " Factor graph key of very first node: " << graphMgrPtr_->getPropagatedStateKey() << std::endl;

  // Set flag
  initedGraphFlag_ = true;
}

void GraphMsf::optimizeGraph_() {
  // While loop
  REGULAR_COUT << " Thread for updating graph is ready." << std::endl;
  double lastOptimizedTime = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
  double currentTime = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
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
      graphMgrPtr_->updateGraph();
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
