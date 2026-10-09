/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// C++
#include <chrono>
#include <cmath>
#include <stdexcept>

// Implementation
#include "holistic_fusion/interface/HolisticFusion.h"

// Workspace
#include "holistic_fusion/core/GraphManager.h"
#include "holistic_fusion/interface/constants.h"

namespace holistic_fusion {

// Public -----------------------------------------------------------
/// Constructor -----------
HolisticFusion::HolisticFusion() {
  REGULAR_COUT << GREEN_START << " HolisticFusion-Constructor called." << COLOR_END << std::endl;
}

/// Destructor -----------
HolisticFusion::~HolisticFusion() {
  // Properly stop the optimizeGraphThread_ before destruction to avoid std::terminate.
  stopOptimizeGraphThread_ = true;
  if (optimizeGraphThread_.joinable()) {
    optimizeGraphThread_.join();
  }
}

void HolisticFusion::setup(const std::shared_ptr<GraphConfig> graphConfigPtr, const std::shared_ptr<StaticTransforms> staticTransformsPtr) {
  REGULAR_COUT << GREEN_START << " HolisticFusion-Setup called." << COLOR_END << std::endl;

  // Check if setup has been called before
  if (graphConfigPtr == nullptr || staticTransformsPtr == nullptr) {
    throw std::runtime_error("setup() of inheriting classes did not  has not been called before.");
  } else {
    graphConfigPtr_ = graphConfigPtr;
    staticTransformsPtr_ = staticTransformsPtr;
  }

  if (!graphConfigPtr_->staticAtStartup_) {
    graphConfigPtr_->gyroBiasPrior_.setZero();
    throw std::logic_error("HolisticFusion: initialization_params.static_at_startup=false: in-motion initialization is not implemented.");
  }

  if (graphConfigPtr_->additionalOptimizationIterations_ < 0) {
    throw std::invalid_argument("HolisticFusion: additionalOptimizationIterations must be nonnegative.");
  }
  if (graphConfigPtr_->createStateEveryNthImuMeasurement_ < 1) {
    throw std::invalid_argument("HolisticFusion: createStateEveryNthImuMeasurement must be positive.");
  }

  // Imu Buffer
  // Initialize IMU buffer
  coreImuBufferPtr_ = std::make_shared<holistic_fusion::ImuBuffer>(graphConfigPtr_);

  // Graph Manager
  std::cout << "HolisticFusion: Creating GraphManager." << std::endl;
  graphMgrPtr_ =
      std::make_shared<GraphManager>(graphConfigPtr_, staticTransformsPtr_->getImuFrame(), staticTransformsPtr_->getWorldFrame());
  std::cout << "HolisticFusion: GraphManager created." << std::endl;

  /// Initialize helper threads
  optimizeGraphThread_ = std::thread(&HolisticFusion::optimizeGraph_, this);
  REGULAR_COUT << " Initialized thread for optimizing the graph in parallel." << std::endl;
}

// Trigger functions -----------------------
bool HolisticFusion::optimizeSlowBatchSmoother(int maxIterations, const std::string& savePath, const bool saveCovarianceFlag) {
  return graphMgrPtr_->optimizeSlowBatchSmoother(maxIterations, savePath, saveCovarianceFlag);
}

bool HolisticFusion::logRealTimeNavStates(const std::string& savePath) {
  // String of time without line breaks: year_month_day_hour_min_sec
  std::ostringstream oss;
  std::time_t now_time_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::tm now_tm = *std::localtime(&now_time_t);
  oss << std::put_time(&now_tm, "%Y_%m_%d_%H_%M_%S");
  // Convert stream to string
  std::string timeString = oss.str();
  // Return
  return graphMgrPtr_->logRealTimeNavStates(savePath, timeString);
}

// Getter functions -----------------------
bool HolisticFusion::areYawAndPositionInited() const {
  return foundInitialYawAndPositionFlag_;
}

bool HolisticFusion::areRollAndPitchInited() const {
  return alignedImuFlag_;
}

bool HolisticFusion::isGraphInited() const {
  return initedGraphFlag_;
}

// Initialization -----------------------
bool HolisticFusion::initHeadingAndPosition(const UnaryMeasurementXDAbsolute<Eigen::Isometry3d, 6>& T_M_S) {
  const std::lock_guard<std::mutex> initYawAndPositionLock(initYawAndPositionMutex_);
  if (!canInitYawAndPosition_()) {
    return false;
  }

  const gtsam::Pose3 T_W_Smeas = initialT_W_fixedFrame_(T_M_S) * gtsam::Pose3(T_M_S.unaryMeasurement().matrix());
  const gtsam::Rot3 R_W_Iest(preIntegratedNavStatePtr_->getT_W_Ik().rotation());
  const gtsam::Rot3 R_I_S(
      staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(), T_M_S.sensorFrameName()).rotation());

  // Removing the twist of the offset about the z-axis of the world leaves a rotation about a horizontal axis, i.e. zero heading error
  const gtsam::Quaternion q_W_Wmeas = (R_W_Iest * R_I_S * T_W_Smeas.rotation().inverse()).toQuaternion();
  const gtsam::Rot3 R_W_I = gtsam::Rot3::Rz(-2.0 * std::atan2(q_W_Wmeas.z(), q_W_Wmeas.w())) * R_W_Iest;

  setInitialOrientationAndPosition_(R_W_I, T_W_Smeas.translation(), T_M_S.sensorFrameName());
  return true;
}

bool HolisticFusion::initYawAndPosition(const UnaryMeasurementXDAbsolute<double, 1>& yaw_M_S1,
                                        const UnaryMeasurementXDAbsolute<Eigen::Vector3d, 3>& M_t_M_S2) {
  const std::lock_guard<std::mutex> initYawAndPositionLock(initYawAndPositionMutex_);
  if (!canInitYawAndPosition_()) {
    return false;
  }

  const double yaw_W_S1meas = yaw_M_S1.unaryMeasurement() + initialT_W_fixedFrame_(yaw_M_S1).rotation().yaw();
  const gtsam::Rot3 R_W_Iest(preIntegratedNavStatePtr_->getT_W_Ik().rotation());
  const gtsam::Rot3 R_I_S1(
      staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(), yaw_M_S1.sensorFrameName()).rotation());
  // A rotation about the z-axis of the world adds its angle to the Euler yaw
  const gtsam::Rot3 R_W_I = gtsam::Rot3::Rz(yaw_W_S1meas - (R_W_Iest * R_I_S1).yaw()) * R_W_Iest;

  const Eigen::Vector3d W_t_W_S2 = initialT_W_fixedFrame_(M_t_M_S2).transformFrom(gtsam::Point3(M_t_M_S2.unaryMeasurement()));
  setInitialOrientationAndPosition_(R_W_I, W_t_W_S2, M_t_M_S2.sensorFrameName());
  return true;
}

bool HolisticFusion::initHeadingAndPositionAtStart() {
  const std::lock_guard<std::mutex> initYawAndPositionLock(initYawAndPositionMutex_);
  if (!canInitYawAndPosition_()) {
    return false;
  }
  foundInitialYawAndPositionFlag_ = true;
  REGULAR_COUT << GREEN_START << " Initialized the world at the gravity-aligned start pose of "
               << staticTransformsPtr_->getInitializationFrame() << "." << COLOR_END << std::endl;
  return true;
}

bool HolisticFusion::canInitYawAndPosition_() const {
  if (!alignedImuFlag_) {
    REGULAR_COUT << RED_START << " Tried to set initial yaw, but initial attitude is not yet set." << COLOR_END << std::endl;
    return false;
  }
  if (areYawAndPositionInited()) {
    REGULAR_COUT << RED_START << " Tried to set initial yaw, but it has been set before." << COLOR_END << std::endl;
    return false;
  }
  return true;
}

gtsam::Pose3 HolisticFusion::initialT_W_fixedFrame_(const UnaryMeasurementAbsolute& measurement) {
  // Without optimized fixed frames, measurements in M are taken as measurements in the world
  if (measurement.fixedFrameName() == measurement.worldFrameName() || !graphConfigPtr_->optimizeReferenceFramePosesWrtWorldFlag_) {
    return gtsam::Pose3::Identity();
  }
  return graphMgrPtr_->getInitialWorldFrameToFixedFrameTransform(measurement.fixedFrameName());
}

void HolisticFusion::setInitialOrientationAndPosition_(const gtsam::Rot3& R_W_I, const Eigen::Vector3d& W_t_W_S,
                                                       const std::string& sensorFrame) {
  preIntegratedNavStatePtr_->updateOrientationInWorld(R_W_I.matrix(), graphConfigPtr_->odomNotJumpAtStartFlag_);
  const Eigen::Vector3d W_t_W_I = W_t_W_Frame1_to_W_t_W_Frame2_(W_t_W_S, sensorFrame, staticTransformsPtr_->getImuFrame(), R_W_I.matrix());
  preIntegratedNavStatePtr_->updatePositionInWorld(W_t_W_I, graphConfigPtr_->odomNotJumpAtStartFlag_);
  foundInitialYawAndPositionFlag_ = true;

  REGULAR_COUT << GREEN_START
               << " Initial pose of imu frame in world frame has been set to RPY (deg): " << R_W_I.rpy().transpose() * (180.0 / M_PI)
               << ", t (m): " << W_t_W_I.transpose() << "." << COLOR_END << std::endl;
}

bool HolisticFusion::initWorldFrameToFixedFrameTransform(const Eigen::Isometry3d& T_W_F, const std::string& fixedFrame) {
  // Set the initial transform
  std::cout << "Setting initial transform from world frame to " << fixedFrame << " frame: " << T_W_F.matrix() << std::endl;
  // Actually set the transform
  return graphMgrPtr_->setInitialWorldFrameToFixedFrameTransform(T_W_F, fixedFrame);
}

// Adders --------------------------------
/// Main: IMU -----------------------
bool HolisticFusion::addCoreImuMeasurementAndGetState(
    const Eigen::Vector3d& linearAcc, const Eigen::Vector3d& angularVel, double imuTimeK,
    std::shared_ptr<SafeIntegratedNavState>& returnPreIntegratedNavStatePtr,
    std::shared_ptr<SafeNavStateWithCovarianceAndBias>& returnOptimizedStateWithCovarianceAndBiasPtr,
    Eigen::Matrix<double, 6, 1>& returnAddedImuMeasurements) {
  // Setup -------------------------
  // Increase counter
  ++imuCallbackCounter_;

  // Adapt time stamp
  imuTimeK += graphConfigPtr_->imuTimeOffset_;

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

  // Potentially convert to m/s^2
  Eigen::Vector3d linearAccInMps2 = linearAcc;
  if (graphConfigPtr_->isImuAccInG_) {
    constexpr double gravityConst = 9.80665;
    linearAccInMps2 *= gravityConst;
  }

  // Check the norm and spit out warning in case the acceleration is either too small or too big
  const double accNorm = linearAccInMps2.norm();
  if (accNorm < 2) {
    REGULAR_COUT << RED_START << " IMU linear acceleration is too small: " << linearAccInMps2.transpose() << COLOR_END << std::endl;
    REGULAR_COUT << RED_START << " Check whether isImuAccInG_ is not mistakenly set to false." << COLOR_END << std::endl;
  } else if (accNorm > 90) {
    REGULAR_COUT << RED_START << " IMU linear acceleration is too big: " << linearAccInMps2.transpose() << COLOR_END << std::endl;
    REGULAR_COUT << RED_START << " Check whether isImuAccInG_ is not mistakenly set to true." << COLOR_END << std::endl;
  }

  // Clamp linear acceleration to 60 m/s^2 to avoid outliers
  // constexpr double maxAcc = 60.0;
  // if (accNorm > maxAcc) {
  //   // Iterate through each element and scale
  //   for (int i = 0; i < 3; ++i) {
  //     // If element is too big, clamp
  //     if (std::abs(linearAccInMps2(i)) > maxAcc) {
  //       linearAccInMps2(i) = (linearAccInMps2(i) > 0 ? maxAcc : -maxAcc);
  //     }
  //   }
  //   REGULAR_COUT << YELLOW_START << " Clamped IMU linear acceleration to: " << linearAccInMps2.transpose() << COLOR_END << std::endl;
  // }

  // Add measurement to buffer
  returnAddedImuMeasurements = coreImuBufferPtr_->addToImuBuffer(imuTimeK, linearAccInMps2, angularVel);

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
    initGraph_(imuTimeK, returnAddedImuMeasurements.tail<3>());
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
bool HolisticFusion::addZeroMotionFactor(double timeKm1, double timeK, double noiseDensity) {
  static_cast<void>(graphMgrPtr_->addPoseBetweenFactor(gtsam::Pose3::Identity(), noiseDensity * Eigen::Matrix<double, 6, 1>::Ones(),
                                                       timeKm1, timeK, 10, RobustNormEnum::None, 0.0));
  graphMgrPtr_->addUnaryClassicFactor<gtsam::Vector3, 3, gtsam::PriorFactor<gtsam::Vector3>, gtsam::symbol_shorthand::V>(
      gtsam::Vector3::Zero(), noiseDensity * Eigen::Matrix<double, 3, 1>::Ones(), timeK);

  return true;
}

bool HolisticFusion::addZeroVelocityFactor(double timeK, double noiseDensity) {
  graphMgrPtr_->addUnaryClassicFactor<gtsam::Vector3, 3, gtsam::PriorFactor<gtsam::Vector3>, gtsam::symbol_shorthand::V>(
      gtsam::Vector3::Zero(), noiseDensity * Eigen::Matrix<double, 3, 1>::Ones(), timeK);

  return true;
}

// Private ---------------------------------------------------------------

/// Worker Functions -----------------------
bool HolisticFusion::alignImu_(double& imuAttitudeRoll, double& imuAttitudePitch) {
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
void HolisticFusion::initGraph_(const double timeStamp_k, const Eigen::Vector3d& imuAngularVelocity) {
  // Calculate initial attitude;
  const gtsam::Pose3& T_W_I0 = gtsam::Pose3(preIntegratedNavStatePtr_->getT_W_Ik().matrix());
  const gtsam::Pose3& T_O_I0 = gtsam::Pose3(preIntegratedNavStatePtr_->getT_O_Ik_gravityAligned().matrix());
  // Print
  REGULAR_COUT << GREEN_START << " Total initial IMU attitude is RPY (deg): " << T_W_I0.rotation().rpy().transpose() * (180.0 / M_PI)
               << COLOR_END << std::endl;

  // Gravity
  graphConfigPtr_->W_gravityVector_ = Eigen::Vector3d(0.0, 0.0, -graphConfigPtr_->gravityMagnitude_);
  graphMgrPtr_->initImuIntegrators(graphConfigPtr_->gravityMagnitude_);
  /// Initialize graph node
  graphMgrPtr_->initPoseVelocityBiasGraph(timeStamp_k, T_W_I0, T_O_I0, imuAngularVelocity);

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

void HolisticFusion::optimizeGraph_() {
  // While loop
  REGULAR_COUT << " Thread for updating graph is ready." << std::endl;
  double lastOptimizedTime = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
  double currentTime = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
  bool optimizedAtLeastOnce = false;
  while (!stopOptimizeGraphThread_) {
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
Eigen::Vector3d HolisticFusion::W_t_W_Frame1_to_W_t_W_Frame2_(const Eigen::Vector3d& W_t_W_frame1, const std::string& frame1,
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

void HolisticFusion::pretendFirstMeasurementReceived() {
  validFirstMeasurementReceivedFlag_ = true;
}

// Check whether measurement violated covariance, if yes, add to set and print once, if not, remove from set and print that returned
bool HolisticFusion::checkAndPrintCovarianceViolation_(const std::string& measurementName, const bool violatedFlag) {
  if (violatedFlag) {
    if (measurementsWithViolatedCovariance_.find(measurementName) == measurementsWithViolatedCovariance_.end()) {
      measurementsWithViolatedCovariance_.insert(measurementName);
      REGULAR_COUT << RED_START << " " << measurementName << " covariance violated. Not adding factor until not violated anymore."
                   << COLOR_END << std::endl;
    }
    return true;
  } else {
    if (measurementsWithViolatedCovariance_.find(measurementName) != measurementsWithViolatedCovariance_.end()) {
      measurementsWithViolatedCovariance_.erase(measurementName);
      REGULAR_COUT << GREEN_START << " " << measurementName << " covariance not violated anymore." << COLOR_END << std::endl;
    }
    return false;
  }
}

}  // namespace holistic_fusion
