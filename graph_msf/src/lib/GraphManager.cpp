/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#define MIN_ITERATIONS_BEFORE_REMOVING_STATIC_TRANSFORM 200

// C++
#include <iomanip>
#include <string>
#include <utility>

// IO
#include <gtsam/slam/dataset.h>

// Factors
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>

// Workspace
#include "graph_msf/config/AdmissibleGtsamSymbols.h"
#include "graph_msf/core/GraphManager.hpp"
// ISAM2
#include "graph_msf/core/optimizer/OptimizerIsam2Batch.hpp"
#include "graph_msf/core/optimizer/OptimizerIsam2FixedLag.hpp"
// LM
#include "graph_msf/core/optimizer/OptimizerLMBatch.hpp"
#include "graph_msf/core/optimizer/OptimizerLMFixedLag.hpp"

#define REGULAR_COUT std::cout << YELLOW_START << "GMSF-GraphManager" << COLOR_END

namespace graph_msf {

// Public --------------------------------------------------------------------

GraphManager::GraphManager(std::shared_ptr<GraphConfig> graphConfigPtr, std::string imuFrame, std::string worldFrame)
    : graphConfigPtr_(std::move(graphConfigPtr)),
      imuFrame_(std::move(imuFrame)),
      worldFrame_(std::move(worldFrame)),
      resultFixedFrameTransformations_(Eigen::Isometry3d::Identity()),
      resultFixedFrameTransformationsCovariance_(Eigen::Matrix<double, 6, 6>::Zero()) {
  // Buffer
  rtFactorGraphBufferPtr_ = std::make_shared<gtsam::NonlinearFactorGraph>();
  batchFactorGraphBufferPtr_ = std::make_shared<gtsam::NonlinearFactorGraph>();
  rtGraphValuesBufferPtr_ = std::make_shared<gtsam::Values>();
  batchGraphValuesBufferPtr_ = std::make_shared<gtsam::Values>();
  rtGraphKeysTimestampsMapBufferPtr_ = std::make_shared<std::map<gtsam::Key, double>>();
  batchGraphKeysTimestampsMapBufferPtr_ = std::make_shared<std::map<gtsam::Key, double>>();

  // Keys
  timeToKeyBufferPtr_ = std::make_shared<TimeGraphKeyBuffer>(graphConfigPtr_->imuBufferLength_, graphConfigPtr_->verboseLevel_);

  // Optimizers
  // A. Real-time Optimizer
  if (graphConfigPtr_->realTimeSmootherUseIsamFlag_) {
    rtOptimizerPtr_ = std::make_shared<OptimizerIsam2FixedLag>(graphConfigPtr_);
  } else {
    rtOptimizerPtr_ = std::make_shared<OptimizerLMFixedLag>(graphConfigPtr_);
  }
  // B. Batch Optimizer
  if (graphConfigPtr_->useAdditionalSlowBatchSmoother_) {
    if (graphConfigPtr_->slowBatchSmootherUseIsamFlag_) {
      batchOptimizerPtr_ = std::make_shared<OptimizerIsam2Batch>(graphConfigPtr_);
    } else {
      batchOptimizerPtr_ = std::make_shared<OptimizerLMBatch>(graphConfigPtr_);
    }
  }
}

// Initialization Interface ---------------------------------------------------
bool GraphManager::initImuIntegrators(const double gravityValue) {
  // Gravity direction definition
  imuParamsPtr_ = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(gravityValue);  // ROS convention

  // Set noise and bias parameters
  /// Position
  imuParamsPtr_->setAccelerometerCovariance(gtsam::Matrix33::Identity(3, 3) * std::pow(graphConfigPtr_->accNoiseDensity_, 2));
  imuParamsPtr_->setIntegrationCovariance(gtsam::Matrix33::Identity(3, 3) *
                                          std::pow(graphConfigPtr_->integrationNoiseDensity_, 2));  // error committed in integrating
                                                                                                    // position from velocities
  imuParamsPtr_->setUse2ndOrderCoriolis(graphConfigPtr_->use2ndOrderCoriolisFlag_);
  /// Rotation
  imuParamsPtr_->setGyroscopeCovariance(gtsam::Matrix33::Identity(3, 3) * std::pow(graphConfigPtr_->gyroNoiseDensity_, 2));
  imuParamsPtr_->setOmegaCoriolis(gtsam::Vector3(0, 0, 1) * graphConfigPtr_->omegaCoriolis_);
  /// Bias
  imuParamsPtr_->setBiasAccCovariance(gtsam::Matrix33::Identity(3, 3) * std::pow(graphConfigPtr_->accBiasRandomWalkNoiseDensity_, 2));
  imuParamsPtr_->setBiasOmegaCovariance(gtsam::Matrix33::Identity(3, 3) * std::pow(graphConfigPtr_->gyroBiasRandomWalkNoiseDensity_, 2));
  imuParamsPtr_->setBiasAccOmegaInit(gtsam::Matrix66::Identity(6, 6) * std::pow(graphConfigPtr_->biasAccOmegaInit_, 2));

  // Use previously defined prior for gyro
  imuBiasPriorPtr_ = std::make_shared<gtsam::imuBias::ConstantBias>(graphConfigPtr_->accBiasPrior_, graphConfigPtr_->gyroBiasPrior_);

  // Init Pre-integrators
  imuStepPreintegratorPtr_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(imuParamsPtr_, *imuBiasPriorPtr_);
  imuBufferPreintegratorPtr_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(imuParamsPtr_, *imuBiasPriorPtr_);
  imuParamsPtr_->print("GraphMSF-IMU PreIntegration Parameters:");
  return true;
}

bool GraphManager::initPoseVelocityBiasGraph(const double timeStamp, const gtsam::Pose3& T_W_I0, const gtsam::Pose3& T_O_I0) {
  // Create Prior factor ----------------------------------------------------
  /// Prior factor noise
  auto priorPoseNoise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << graphConfigPtr_->initialOrientationNoiseDensity_, graphConfigPtr_->initialOrientationNoiseDensity_,
       graphConfigPtr_->initialOrientationNoiseDensity_, graphConfigPtr_->initialPositionNoiseDensity_,
       graphConfigPtr_->initialPositionNoiseDensity_, graphConfigPtr_->initialPositionNoiseDensity_)
          .finished());                                                                                             // rad,rad,rad,m, m, m
  auto priorVelocityNoise = gtsam::noiseModel::Isotropic::Sigma(3, graphConfigPtr_->initialVelocityNoiseDensity_);  // m/s
  auto priorBiasNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << graphConfigPtr_->initialAccBiasNoiseDensity_,  // m/s^2
                                                             graphConfigPtr_->initialAccBiasNoiseDensity_,                      // m/s^2
                                                             graphConfigPtr_->initialAccBiasNoiseDensity_,                      // m/s^2
                                                             graphConfigPtr_->initialGyroBiasNoiseDensity_,                     // rad/s
                                                             graphConfigPtr_->initialGyroBiasNoiseDensity_,                     // rad/s
                                                             graphConfigPtr_->initialGyroBiasNoiseDensity_)                     // rad/s
                                                                .finished());  // acc, acc, acc, gyro, gyro, gyro

  // Pre-allocate
  gtsam::NonlinearFactorGraph newRtGraphFactors, newBatchGraphFactors;
  gtsam::Values newRtGraphValues, newBatchGraphValues;
  std::shared_ptr<std::map<gtsam::Key, double>> priorKeyTimestampMapPtr = std::make_shared<std::map<gtsam::Key, double>>();

  // Looking up from IMU buffer --> acquire mutex (otherwise values for key might not be set)
  {
    const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);

    // Initial estimate
    gtsam::Values valuesEstimate;
    REGULAR_COUT << " Initial Pose of imu in world frame: " << T_W_I0 << std::endl;
    valuesEstimate.insert(gtsam::symbol_shorthand::X(propagatedStateKey_), T_W_I0);
    REGULAR_COUT << " Initial velocity assumed to be: " << gtsam::Vector3(0, 0, 0) << std::endl;
    valuesEstimate.insert(gtsam::symbol_shorthand::V(propagatedStateKey_), gtsam::Vector3(0, 0, 0));
    REGULAR_COUT << " Initial bias set to: " << *imuBiasPriorPtr_ << std::endl;
    valuesEstimate.insert(gtsam::symbol_shorthand::B(propagatedStateKey_), *imuBiasPriorPtr_);
    /// Timestamp mapping for incremental fixed lag smoother
    rtGraphValuesBufferPtr_->insert(valuesEstimate);
    batchGraphValuesBufferPtr_->insert(valuesEstimate);  // TODO: Check whether velocity can be removed from batch
    writeValueKeysToKeyTimeStampMap_(valuesEstimate, timeStamp, priorKeyTimestampMapPtr);

    // Initialize graph -------------------------------------------------
    rtFactorGraphBufferPtr_->resize(0);
    batchFactorGraphBufferPtr_->resize(0);
    rtFactorGraphBufferPtr_->emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
        gtsam::symbol_shorthand::X(propagatedStateKey_), T_W_I0,
        priorPoseNoise);  // POSE - PriorFactor format is (key,value,matrix) value
    batchFactorGraphBufferPtr_->emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
        gtsam::symbol_shorthand::X(propagatedStateKey_), T_W_I0,
        priorPoseNoise);  // POSE - PriorFactor format is (key,value,matrix) value
    // is same type as type of PriorFactor
    rtFactorGraphBufferPtr_->emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(gtsam::symbol_shorthand::V(propagatedStateKey_),
                                                                                gtsam::Vector3(0, 0, 0),
                                                                                priorVelocityNoise);  // VELOCITY
    batchFactorGraphBufferPtr_->emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(gtsam::symbol_shorthand::V(propagatedStateKey_),
                                                                                   gtsam::Vector3(0, 0, 0),
                                                                                   priorVelocityNoise);  // VELOCITY
    rtFactorGraphBufferPtr_->emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
        gtsam::symbol_shorthand::B(propagatedStateKey_), *imuBiasPriorPtr_,
        priorBiasNoise);  // BIAS
    batchFactorGraphBufferPtr_->emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
        gtsam::symbol_shorthand::B(propagatedStateKey_), *imuBiasPriorPtr_,
        priorBiasNoise);  // BIAS

    // Add to Time Key Buffer
    timeToKeyBufferPtr_->addToBuffer(timeStamp, propagatedStateKey_);

    // Copy over
    // Rt
    newRtGraphFactors = *rtFactorGraphBufferPtr_;
    newRtGraphValues = *rtGraphValuesBufferPtr_;
    rtFactorGraphBufferPtr_->resize(0);
    rtGraphValuesBufferPtr_->clear();
    // Batch
    newBatchGraphFactors = *batchFactorGraphBufferPtr_;
    newBatchGraphValues = *batchGraphValuesBufferPtr_;
    batchFactorGraphBufferPtr_->resize(0);
    batchGraphValuesBufferPtr_->clear();
  }

  /// Add prior factor to graph and optimize for the first time ----------------
  addFactorsToSmootherAndOptimize(newRtGraphFactors, newRtGraphValues, *priorKeyTimestampMapPtr, newBatchGraphFactors, newBatchGraphValues,
                                  *priorKeyTimestampMapPtr, graphConfigPtr_, 0);

  // Update Current State ---------------------------------------------------
  const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
  optimizedGraphState_.updateNavStateAndBias(propagatedStateKey_, timeStamp, gtsam::NavState(T_W_I0, gtsam::Vector3(0, 0, 0)),
                                             gtsam::Vector3(0, 0, 0), *imuBiasPriorPtr_);
  O_imuPropagatedState_ = gtsam::NavState(T_O_I0, gtsam::Vector3(0, 0, 0));
  W_imuPropagatedState_ = gtsam::NavState(T_W_I0, gtsam::Vector3(0, 0, 0));
  T_W_O_ = (T_W_I0.inverse() * T_O_I0).matrix();
  return true;
}

// IMU at the core --------------------------------------------------------------
void GraphManager::addImuFactorAndGetState(SafeIntegratedNavState& returnPreIntegratedNavState,
                                           std::shared_ptr<SafeNavStateWithCovarianceAndBias>& newOptimizedNavStatePtr,
                                           const std::shared_ptr<ImuBuffer>& imuBufferPtr, const double imuTimeK, bool createNewStateFlag) {
  // Looking up from IMU buffer --> acquire mutex (otherwise values for key might not be set)
  const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);

  // Part 1 (ALWAYS): Propagate state and imu measurement to pre-integrator -----------------------
  // 1.1 Get last two measurements from buffer to determine dt
  TimeToImuMap imuMeas;
  imuBufferPtr->getLastTwoMeasurements(imuMeas);
  propagatedStateTime_ = imuTimeK;
  currentAngularVelocity_ = imuMeas.rbegin()->second.angularVelocity;

  // 1.2 Update IMU Pre-integrator
  updateImuIntegrators_(imuMeas);

  // 1.3 Predict propagated state via forward integration
  if (graphConfigPtr_->usingBiasForPreIntegrationFlag_) {
    O_imuPropagatedState_ =
        imuBufferPtr->integrateNavStateFromTimestamp(imuMeas.begin()->first, imuMeas.rbegin()->first, O_imuPropagatedState_,
                                                     optimizedGraphState_.imuBias(), graphConfigPtr_->W_gravityVector_);
    W_imuPropagatedState_ =
        imuBufferPtr->integrateNavStateFromTimestamp(imuMeas.begin()->first, imuMeas.rbegin()->first, W_imuPropagatedState_,
                                                     optimizedGraphState_.imuBias(), graphConfigPtr_->W_gravityVector_);
  } else {
    O_imuPropagatedState_ =
        imuBufferPtr->integrateNavStateFromTimestamp(imuMeas.begin()->first, imuMeas.rbegin()->first, O_imuPropagatedState_,
                                                     gtsam::imuBias::ConstantBias(), graphConfigPtr_->W_gravityVector_);
    W_imuPropagatedState_ =
        imuBufferPtr->integrateNavStateFromTimestamp(imuMeas.begin()->first, imuMeas.rbegin()->first, W_imuPropagatedState_,
                                                     gtsam::imuBias::ConstantBias(), graphConfigPtr_->W_gravityVector_);
  }

  // 1.4 Fill in optimized state container
  if (optimizedGraphState_.isOptimized()) {
    newOptimizedNavStatePtr = std::make_shared<SafeNavStateWithCovarianceAndBias>(optimizedGraphState_);
  } else {
    newOptimizedNavStatePtr = nullptr;
  }

  // 1.5 Return pre-integrated state
  gtsam::NavState& T_O_Ik_nav = O_imuPropagatedState_;  // Alias
  // Assign poses and velocities
  returnPreIntegratedNavState.update(T_W_O_, Eigen::Isometry3d(T_O_Ik_nav.pose().matrix()), T_O_Ik_nav.bodyVelocity(),
                                     optimizedGraphState_.imuBias().correctGyroscope(imuMeas.rbegin()->second.angularVelocity), imuTimeK,
                                     false);

  // Check whether new state can be created (in case fixed lag smoother has a short window)
  if (createNewStateFlag && imuTimeK - lastOptimizedStateTime_ > graphConfigPtr_->realTimeSmootherLag_ - WORST_CASE_OPTIMIZATION_TIME &&
      lastOptimizedStateTime_ > 0.0) {
    createNewStateFlag = false;
    REGULAR_COUT << RED_START
                 << " The current measurement would fall outside of the real-time smoother lag, hence skipping the creation of a new"
                 << "IMU measurement. Not creating new state." << COLOR_END << std::endl;
  }

  // Part 2 (OPTIONAL): Create new state and add IMU factor to graph ------------------------------
  if (createNewStateFlag) {
    // Get new key
    const gtsam::Key oldKey = propagatedStateKey_;
    const gtsam::Key newKey = newPropagatedStateKey_();

    // Add to time key buffer
    timeToKeyBufferPtr_->addToBuffer(imuTimeK, newKey);

    // Add IMU Factor to graph
    gtsam::CombinedImuFactor imuFactor(gtsam::symbol_shorthand::X(oldKey), gtsam::symbol_shorthand::V(oldKey),
                                       gtsam::symbol_shorthand::X(newKey), gtsam::symbol_shorthand::V(newKey),
                                       gtsam::symbol_shorthand::B(oldKey), gtsam::symbol_shorthand::B(newKey), *imuStepPreintegratorPtr_);
    addFactorToRtAndBatchGraph_<const gtsam::CombinedImuFactor*>(&imuFactor, imuTimeK, "imu");

    // Add IMU values
    gtsam::Values valuesEstimate;
    valuesEstimate.insert(gtsam::symbol_shorthand::X(newKey), W_imuPropagatedState_.pose());
    valuesEstimate.insert(gtsam::symbol_shorthand::V(newKey), W_imuPropagatedState_.velocity());
    valuesEstimate.insert(gtsam::symbol_shorthand::B(newKey), optimizedGraphState_.imuBias());
    rtGraphValuesBufferPtr_->insert(valuesEstimate);
    if (graphConfigPtr_->useAdditionalSlowBatchSmoother_) {
      batchGraphValuesBufferPtr_->insert(valuesEstimate);
    }

    // Add timestamps
    writeValueKeysToKeyTimeStampMap_(valuesEstimate, imuTimeK, rtGraphKeysTimestampsMapBufferPtr_);
    if (graphConfigPtr_->useAdditionalSlowBatchSmoother_) {
      writeValueKeysToKeyTimeStampMap_(valuesEstimate, imuTimeK, batchGraphKeysTimestampsMapBufferPtr_);
    }

    // After adding this factor we can again empty the step integrator
    // Reset IMU Step Pre-integration
    if (graphConfigPtr_->usingBiasForPreIntegrationFlag_) {
      imuStepPreintegratorPtr_->resetIntegrationAndSetBias(optimizedGraphState_.imuBias());
    } else {
      imuStepPreintegratorPtr_->resetIntegrationAndSetBias(gtsam::imuBias::ConstantBias());
    }
  }  // End of create new state
}

// Unary factors ----------------------------------------------------------------
// Key Lookup
bool GraphManager::getUnaryFactorGeneralKey(gtsam::Key& returnedKey, double& returnedGraphTime, const UnaryMeasurement& unaryMeasurement) {
  // Find the closest key in existing graph
  // Case 1: Can't add immediately
  if (!timeToKeyBufferPtr_->getClosestKeyAndTimestamp(returnedGraphTime, returnedKey, unaryMeasurement.measurementName(),
                                                      graphConfigPtr_->maxSearchDeviation_, unaryMeasurement.timeK())) {
    // Measurement coming from the future
    if (propagatedStateTime_ - unaryMeasurement.timeK() < 0.0) {  // Factor is coming from the future, hence add it to the buffer
      // Not too far in the future --> add to buffer and add later
      if (unaryMeasurement.timeK() - propagatedStateTime_ < 4 * graphConfigPtr_->maxSearchDeviation_) {
        // TODO: Add to buffer and return --> still add it until we are there
      }
      // Too far in the future --> do not add it
      else {
        REGULAR_COUT << RED_START << " Factor coming from the future, AND time deviation of " << typeid(unaryMeasurement).name()
                     << " at key " << returnedKey << " is " << 1000 * std::abs(returnedGraphTime - unaryMeasurement.timeK())
                     << " ms, being larger than admissible deviation of " << 2000 * graphConfigPtr_->maxSearchDeviation_
                     << " ms. Not adding to graph." << COLOR_END << std::endl;
        return false;
      }
    }
    // Measurement coming from the past, but could not find suitable key
    else {  // Otherwise do not add it
      REGULAR_COUT << RED_START << " Time deviation of " << typeid(unaryMeasurement).name() << " at key " << returnedKey << " is "
                   << 1000 * std::abs(returnedGraphTime - unaryMeasurement.timeK()) << " ms, being larger than admissible deviation of "
                   << 1000 * graphConfigPtr_->maxSearchDeviation_ << " ms. Not adding to graph." << COLOR_END << std::endl;
      return false;
    }
  }
  // Case 2: Can add immediately
  else {
    return true;
  }
}

// Robust Aware Between factors ------------------------------------------------------------------------------------------------------
gtsam::Key GraphManager::addPoseBetweenFactor(const gtsam::Pose3& deltaPose, const Eigen::Matrix<double, 6, 1>& poseBetweenNoiseDensity,
                                              const double timeKm1, const double timeK, const double rate,
                                              const RobustNormEnum& robustNormEnum, const double robustNormConstant) {
  // Find corresponding keys in graph
  // Find corresponding keys in graph
  const double maxLidarTimestampDistance = (1.0 / rate) + (2.0 * graphConfigPtr_->maxSearchDeviation_);
  gtsam::Key closestKeyKm1, closestKeyK;
  double keyTimeStampDistance{0.0};

  if (!findGraphKeys_(closestKeyKm1, closestKeyK, keyTimeStampDistance, maxLidarTimestampDistance, timeKm1, timeK, "pose between")) {
    REGULAR_COUT << RED_START << " Current propagated key: " << propagatedStateKey_ << " , PoseBetween factor not added between keys "
                 << closestKeyKm1 << " and " << closestKeyK << COLOR_END << std::endl;
    return closestKeyK;
  }

  // Scale delta pose according to timeStampDistance
  const double scale = keyTimeStampDistance / (timeK - timeKm1);
  if (graphConfigPtr_->verboseLevel_ > 3) {
    REGULAR_COUT << " Scaling factor in tangent space for pose between delta pose: " << scale << std::endl;
  }
  gtsam::Pose3 scaledDeltaPose = gtsam::Pose3::Expmap(scale * gtsam::Pose3::Logmap(deltaPose));

  // Create noise model
  assert(poseBetweenNoiseDensity.size() == 6);
  auto noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(poseBetweenNoiseDensity)));  // rad,rad,rad,m,m,m
  boost::shared_ptr<gtsam::noiseModel::Robust> robustErrorFunction;
  // Pick Robust Error Function
  switch (robustNormEnum) {
    case RobustNormEnum::Huber:
      robustErrorFunction = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(robustNormConstant), noise);
      break;
    case RobustNormEnum::Cauchy:
      robustErrorFunction = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Cauchy::Create(robustNormConstant), noise);
      break;
    case RobustNormEnum::Tukey:
      robustErrorFunction = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Tukey::Create(robustNormConstant), noise);
      break;
  }

  // Create pose between factor and add it
  gtsam::BetweenFactor<gtsam::Pose3> poseBetweenFactor;
  if (robustNormEnum == RobustNormEnum::None) {
    poseBetweenFactor = gtsam::BetweenFactor<gtsam::Pose3>(gtsam::symbol_shorthand::X(closestKeyKm1),
                                                           gtsam::symbol_shorthand::X(closestKeyK), scaledDeltaPose, noise);
  } else {
    poseBetweenFactor = gtsam::BetweenFactor<gtsam::Pose3>(gtsam::symbol_shorthand::X(closestKeyKm1),
                                                           gtsam::symbol_shorthand::X(closestKeyK), scaledDeltaPose, robustErrorFunction);
  }

  // Write to graph
  addFactorSafelyToRtAndBatchGraph_<const gtsam::BetweenFactor<gtsam::Pose3>*>(&poseBetweenFactor, timeKm1);

  // Print summary
  if (graphConfigPtr_->verboseLevel_ > 3) {
    REGULAR_COUT << " Current propagated key: " << propagatedStateKey_ << ", " << YELLOW_START << " PoseBetween factor added between key "
                 << closestKeyKm1 << " and key " << closestKeyK << COLOR_END << std::endl;
  }

  return closestKeyK;
}

gtsam::NavState GraphManager::calculateNavStateAtKey(bool& computeSuccessfulFlag,
                                                     const std::shared_ptr<graph_msf::OptimizerBase> optimizerPtr, const gtsam::Key& key,
                                                     const char* callingFunctionName) {
  // Nav State (pose and velocity)
  gtsam::Pose3 resultPose;
  gtsam::Vector3 resultVelocity;
  try {
    resultPose = optimizerPtr->calculateEstimatedPose3(gtsam::symbol_shorthand::X(key));  // auto result = mainGraphPtr_->estimate();
    resultVelocity = optimizerPtr->calculateEstimatedVelocity3(gtsam::symbol_shorthand::V(key));
    computeSuccessfulFlag = true;
  } catch (const std::out_of_range& outOfRangeExeception) {
    REGULAR_COUT << "Out of Range exeception while optimizing graph: " << outOfRangeExeception.what() << '\n';
    REGULAR_COUT << RED_START
                 << " This happens if the measurement delay is larger than the graph-smootherLag, i.e. the optimized graph instances are "
                    "not connected. Increase the lag in this case."
                 << COLOR_END << std::endl;
    REGULAR_COUT << RED_START << " CalculateNavStateAtKey called by " << callingFunctionName << COLOR_END << std::endl;
    computeSuccessfulFlag = false;
  }
  return gtsam::NavState(resultPose, resultVelocity);
}

void GraphManager::updateGraph() {
  // At compile time get the symbols for the 6D and 3D states
  constexpr int numDynamic6DStates = countNDStates<6>();
  constexpr int numDynamic3DStates = countNDStates<3>();
  constexpr std::array<char, numDynamic6DStates> dim6StateSymbols = getSymbolArrayForNDStates<6, numDynamic6DStates>();
  constexpr std::array<char, numDynamic3DStates> dim3StateSymbols = getSymbolArrayForNDStates<3, numDynamic3DStates>();

  // Method variables
  gtsam::NonlinearFactorGraph newRtGraphFactors, newBatchGraphFactors;
  gtsam::Values newRtGraphValues, newBatchGraphValues;
  std::map<gtsam::Key, double> newRtGraphKeysTimestampsMap, newBatchGraphKeysTimestampsMap;
  gtsam::Key currentPropagatedKey;
  gtsam::Vector3 currentAngularVelocity;
  double currentPropagatedTime;

  // Mutex Block 1 -----------------
  {
    // Lock
    const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
    // Get current key and time
    currentPropagatedKey = propagatedStateKey_;
    currentPropagatedTime = propagatedStateTime_;
    currentAngularVelocity = currentAngularVelocity_;
    // Get copy of factors and values and empty buffers
    newRtGraphFactors = *rtFactorGraphBufferPtr_;
    newRtGraphValues = *rtGraphValuesBufferPtr_;
    newRtGraphKeysTimestampsMap = *rtGraphKeysTimestampsMapBufferPtr_;
    rtFactorGraphBufferPtr_->resize(0);
    rtGraphValuesBufferPtr_->clear();
    rtGraphKeysTimestampsMapBufferPtr_->clear();
    if (graphConfigPtr_->useAdditionalSlowBatchSmoother_) {
      newBatchGraphFactors = *batchFactorGraphBufferPtr_;
      newBatchGraphValues = *batchGraphValuesBufferPtr_;
      newBatchGraphKeysTimestampsMap = *batchGraphKeysTimestampsMapBufferPtr_;
      batchFactorGraphBufferPtr_->resize(0);
      batchGraphValuesBufferPtr_->clear();
      batchGraphKeysTimestampsMapBufferPtr_->clear();
    }

    // Empty Buffer Pre-integrator --> everything missed during the update will
    // be in here
    if (graphConfigPtr_->usingBiasForPreIntegrationFlag_) {
      imuBufferPreintegratorPtr_->resetIntegrationAndSetBias(optimizedGraphState_.imuBias());
    } else {
      imuBufferPreintegratorPtr_->resetIntegrationAndSetBias(gtsam::imuBias::ConstantBias());
    }
  }  // end of locking

  // if no factors are present, return
  if (newRtGraphFactors.size() == 0) {
    if (graphConfigPtr_->verboseLevel_ > 3) {
      REGULAR_COUT << " No factors present, not optimizing." << std::endl;
    }
    return;
  }

  // Graph Update (time consuming) -------------------
  bool successfulOptimizationFlag = addFactorsToSmootherAndOptimize(
      newRtGraphFactors, newRtGraphValues, newRtGraphKeysTimestampsMap, newBatchGraphFactors, newBatchGraphValues,
      newBatchGraphKeysTimestampsMap, graphConfigPtr_, graphConfigPtr_->additionalOptimizationIterations_);
  if (!successfulOptimizationFlag) {
    REGULAR_COUT << RED_START << " Graph optimization failed. " << COLOR_END << std::endl;
    return;
  }

  // Compute entire results ----------------------------------------------
  // A. NavState ------------------------------
  gtsam::NavState resultNavState = calculateNavStateAtKey(successfulOptimizationFlag, rtOptimizerPtr_, currentPropagatedKey, __func__);
  // B. Bias ------------------------------
  gtsam::imuBias::ConstantBias resultBias = rtOptimizerPtr_->calculateEstimatedBias(gtsam::symbol_shorthand::B(currentPropagatedKey));
  // C. Compute & Transform Covariances ------------------------------
  gtsam::Matrix66 resultPoseCovarianceBodyFrame =
      rtOptimizerPtr_->calculateMarginalCovarianceMatrix(gtsam::symbol_shorthand::X(currentPropagatedKey));
  gtsam::Matrix33 resultVelocityCovariance =
      rtOptimizerPtr_->calculateMarginalCovarianceMatrix(gtsam::symbol_shorthand::V(currentPropagatedKey));
  // Transform covariance from I_S_I in body frame, to W_S_W in world frame
  gtsam::Matrix66 adjointMatrix = resultNavState.pose().AdjointMap();
  gtsam::Matrix66 resultPoseCovarianceWorldFrame = adjointMatrix * resultPoseCovarianceBodyFrame * adjointMatrix.transpose();

  // D. FixedFrame Transformations ------------------------------
  if (graphConfigPtr_->optimizeReferenceFramePosesWrtWorld_) {
    // Mutex because we are changing the dynamically allocated graphKeys
    std::lock_guard<std::mutex> modifyGraphKeysLock(gtsamTransformsExpressionKeys_.mutex());

    // Iterate through all dynamically allocated variables (holistic, calibration, landmarks) --------------------------------
    for (auto& framePairKeyMapIterator : gtsamTransformsExpressionKeys_.getTransformsMap()) {
      // Get Variable Key
      const gtsam::Key& gtsamKey = framePairKeyMapIterator.second.key();  // alias

      // State Category Character
      const char stateCategory = gtsam::Symbol(gtsamKey).chr();

      // Try to compute results and uncertainties (if variable is still active)
      if (framePairKeyMapIterator.second.isVariableActive()) {
        // Case 1: All worked ------------------------------------------
        try {  // Obtain estimate and covariance from the extrinsic transformations
          gtsam::Pose3 T_frame1_frame2;
          gtsam::Matrix66 T_frame1_frame2_covariance = gtsam::Z_6x6;

          // 6D Transformations
          if (isCharInCharArray<numDynamic6DStates>(stateCategory, dim6StateSymbols)) {
            T_frame1_frame2 = rtOptimizerPtr_->calculateEstimatedPose3(gtsamKey);
            T_frame1_frame2_covariance = rtOptimizerPtr_->calculateMarginalCovarianceMatrix(gtsamKey);
            // Add current belief back to the map
            framePairKeyMapIterator.second.updateLatestEstimate(T_frame1_frame2, T_frame1_frame2_covariance);
          }
          // 3D Position3 Vectors
          else if (isCharInCharArray<numDynamic3DStates>(stateCategory, dim3StateSymbols)) {
            T_frame1_frame2 = gtsam::Pose3(gtsam::Rot3::Identity(), rtOptimizerPtr_->calculateEstimatedPoint3(gtsamKey));
            T_frame1_frame2_covariance.block<3, 3>(3, 3) = rtOptimizerPtr_->calculateMarginalCovarianceMatrix(gtsamKey);
            // Add current belief back to the map
            framePairKeyMapIterator.second.updateLatestEstimate(T_frame1_frame2, T_frame1_frame2_covariance);
          }
          // Only these two types are allowed for dynamic states for now
          else {
            throw std::runtime_error("Key is neither a pose nor a displacement key.");
          }

          // Write to Result Dictionaries
          Eigen::Matrix4d T_frame1_frame2_corrected_matrix = T_frame1_frame2.matrix();
          T_frame1_frame2_corrected_matrix.block<3, 1>(0, 3) += framePairKeyMapIterator.second.getReferenceFrameKeyframePosition();
          // T_frame1_frame2 = gtsam::Pose3(T_frame1_frame2_matrix);
          resultFixedFrameTransformations_.set_T_frame1_frame2(framePairKeyMapIterator.first.first, framePairKeyMapIterator.first.second,
                                                               Eigen::Isometry3d(T_frame1_frame2_corrected_matrix));
          resultFixedFrameTransformationsCovariance_.set_T_frame1_frame2(framePairKeyMapIterator.first.first,
                                                                         framePairKeyMapIterator.first.second, T_frame1_frame2_covariance);

          // Mark that this key has at least been optimized once
          if (framePairKeyMapIterator.second.getNumberStepsOptimized() == 0 && graphConfigPtr_->verboseLevel_ > 1) {
            REGULAR_COUT << GREEN_START << " Fixed-frame Transformation between " << framePairKeyMapIterator.first.first << " and "
                         << framePairKeyMapIterator.first.second << " optimized for the first time." << COLOR_END << std::endl;
            REGULAR_COUT << GREEN_START << " Result, RPY (deg): " << T_frame1_frame2.rotation().rpy().transpose() * (180.0 / M_PI)
                         << ", t (x, y, z): " << T_frame1_frame2.translation().transpose() << COLOR_END << std::endl;
          }
          // Increase Counter
          framePairKeyMapIterator.second.incrementNumberStepsOptimized();

          // Check health status of transformation --> Delete if diverged too much --> only necessary for global fixed frames
          if (framePairKeyMapIterator.first.second == worldFrame_ &&
              framePairKeyMapIterator.second.getNumberStepsOptimized() > MIN_ITERATIONS_BEFORE_REMOVING_STATIC_TRANSFORM) {
            const gtsam::Pose3& T_frame1_frame2_initial =
                framePairKeyMapIterator.second.getApproximateTransformationBeforeOptimization();  // alias
            const double errorTangentSpace = gtsam::Pose3::Logmap(T_frame1_frame2_initial.between(T_frame1_frame2)).norm();

            // Check error in tangent space
            if (errorTangentSpace > graphConfigPtr_->referenceFramePosesResetThreshold_) {
              REGULAR_COUT << RED_START << "Error in tangent space: " << errorTangentSpace << std::endl;
              REGULAR_COUT << YELLOW_START << "GMsf-GraphManager" << RED_START << " Fixed Frame Transformation between "
                           << framePairKeyMapIterator.first.first << " and " << framePairKeyMapIterator.first.second
                           << " diverged too much. Removing from optimization and adding again freshly at next possibility." << COLOR_END
                           << std::endl;
              // Remove state from state dictionary
              gtsamTransformsExpressionKeys_.removeOrDeactivateTransform(framePairKeyMapIterator.first.first,
                                                                         framePairKeyMapIterator.first.second);
              break;
            }
          }
        }  // end: try statement
        // Case 2: Result computation failed -----------------------------------------
        catch (const std::out_of_range& exception) {
          // Only remove/deactivate measurements that are still active and have been optimized before
          if (framePairKeyMapIterator.second.getNumberStepsOptimized() > 0) {
            REGULAR_COUT << RED_START << " OutOfRange-exception while querying the active transformation and/or covariance at key "
                         << gtsam::Symbol(gtsamKey) << ", for frame pair " << framePairKeyMapIterator.first.first << ","
                         << framePairKeyMapIterator.first.second << std::endl
                         << " This happens if the requested variable is outside of the smoother window (e.g. because corresponding "
                            "measurement stopped). Hence, we keep this estimate unchanged and remove/deactivate it from the state "
                            "dictionary."
                         << COLOR_END << std::endl;
            // Remove state from state dictionary
            gtsamTransformsExpressionKeys_.removeOrDeactivateTransform(framePairKeyMapIterator.first.first,
                                                                       framePairKeyMapIterator.first.second);
            return;
          }
          // If active but added newly but never optimized
          else {
            REGULAR_COUT
                << GREEN_START << " Tried to query the transformation and/or covariance for frame pair "
                << framePairKeyMapIterator.first.first << " to " << framePairKeyMapIterator.first.second
                << ", at key: " << gtsam::Symbol(gtsamKey)
                << ". Not yet available, as it was not yet optimized. Waiting for next optimization iteration until publishing it. "
                   "Current state key: "
                << currentPropagatedKey << COLOR_END << std::endl;
          }
        }  // catch statement
      }  // end: if active statement
    }  // for loop over all transforms
  }

  // Mutex block 2 ------------------
  {
    // Lock
    const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
    // Optimized Graph State Status
    optimizedGraphState_.setIsOptimized();
    // Update Optimized Graph State
    optimizedGraphState_.updateNavStateAndBias(currentPropagatedKey, currentPropagatedTime, resultNavState,
                                               resultBias.correctGyroscope(currentAngularVelocity), resultBias);
    optimizedGraphState_.updateFixedFrameTransforms(resultFixedFrameTransformations_);
    optimizedGraphState_.updateFixedFrameTransformsCovariance(resultFixedFrameTransformationsCovariance_);
    optimizedGraphState_.updateCovariances(resultPoseCovarianceWorldFrame, resultVelocityCovariance);
    // Predict from solution to obtain refined propagated state
    if (graphConfigPtr_->usingBiasForPreIntegrationFlag_) {
      W_imuPropagatedState_ = imuBufferPreintegratorPtr_->predict(resultNavState, optimizedGraphState_.imuBias());
    } else {
      W_imuPropagatedState_ = imuBufferPreintegratorPtr_->predict(resultNavState, gtsam::imuBias::ConstantBias());
    }

    // Correct rotation only for roll and pitch, keep integrated yaw
    gtsam::Rot3 R_O_I_rp_corrected =
        gtsam::Rot3::Ypr(O_imuPropagatedState_.pose().rotation().yaw(), W_imuPropagatedState_.pose().rotation().pitch(),
                         W_imuPropagatedState_.pose().rotation().roll());
    // Rotate corrected velocity to odom frame
    gtsam::Vector3 O_v_O_I = R_O_I_rp_corrected * W_imuPropagatedState_.bodyVelocity();
    // Update the NavState
    O_imuPropagatedState_ = gtsam::NavState(R_O_I_rp_corrected, O_imuPropagatedState_.pose().translation(), O_v_O_I);
    T_W_O_ = Eigen::Isometry3d((W_imuPropagatedState_.pose() * O_imuPropagatedState_.pose().inverse()).matrix());

    // Update the time of the last optimized state
    lastOptimizedStateTime_ = currentPropagatedTime;
  }  // end of locking
}

bool GraphManager::optimizeSlowBatchSmoother(int maxIterations, const std::string& savePath) {
  if (graphConfigPtr_->useAdditionalSlowBatchSmoother_) {
    // Time duration of optimization
    std::chrono::time_point<std::chrono::high_resolution_clock> startOptimizationTime = std::chrono::high_resolution_clock::now();
    // Optimization
    batchOptimizerPtr_->optimize(maxIterations);
    const gtsam::Values& isam2OptimizedStates = batchOptimizerPtr_->getAllOptimizedStates();
    // Key to timestamp map
    const std::map<gtsam::Key, double>& keyTimestampMap = batchOptimizerPtr_->getFullKeyTimestampMap();
    // Calculate Duration
    std::chrono::time_point<std::chrono::high_resolution_clock> endOptimizationTime = std::chrono::high_resolution_clock::now();
    double optimizationDuration =
        std::chrono::duration_cast<std::chrono::milliseconds>(endOptimizationTime - startOptimizationTime).count();
    std::cout << "Optimization took " << optimizationDuration << " ms." << std::endl;

    // Save Optimized Result
    saveOptimizedValuesToFile(isam2OptimizedStates, keyTimestampMap, savePath);

    // Return
    return true;
  } else {
    return false;
  }
}

// Save optimized values to file
void GraphManager::saveOptimizedValuesToFile(const gtsam::Values& optimizedValues, const std::map<gtsam::Key, double>& keyTimestampMap,
                                             const std::string& savePath) {
  // At compile time get the symbols for the 6D and 3D states
  constexpr int num6DStates = countNDStates<6>();
  constexpr int num3DStates = countNDStates<3>();
  constexpr std::array<char, num6DStates> dim6StateSymbols = getSymbolArrayForNDStates<6, num6DStates>();
  constexpr std::array<char, num3DStates> dim3StateSymbols = getSymbolArrayForNDStates<3, num3DStates>();

  // Map to hold file streams, keyed by category
  std::map<std::string, std::ofstream> fileStreams;

  // Get current time as string for file name
  std::time_t now_time_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::tm now_tm = *std::localtime(&now_time_t);
  // String of time without line breaks: year_month_day_hour_min_sec
  std::ostringstream oss;
  oss << std::put_time(&now_tm, "%Y_%m_%d_%H_%M_%S");
  // Convert stream to string
  std::string timeString = oss.str();
  // Remove any line breaks
  timeString.erase(std::remove(timeString.begin(), timeString.end(), '\n'), timeString.end());
  // Replace spaces with underscores
  std::replace(timeString.begin(), timeString.end(), ' ', '_');

  // Save optimized states
  // A. 6D SE(3) states -----------------------------------------------------------
  for (const auto& keyPosePair : optimizedValues.extract<gtsam::Pose3>()) {
    // Read out information
    const gtsam::Key& key = keyPosePair.first;
    const gtsam::Pose3& pose = keyPosePair.second;
    const gtsam::Symbol symbol(key);
    const char stateCategory = symbol.chr();
    const double timeStamp = keyTimestampMap.at(key);

    // Additional strings
    std::string stateCategoryString = "";
    std::string frameInformation = "";

    // State Category Capital
    const std::string stateCategoryCapital = std::to_string(std::toupper(stateCategory));

    // A.A Creation of the identifier for the file -----------------------------------
    // Case 1: Navigation State Pose
    if (stateCategory == 'x') {
      stateCategoryString = stateCategoryCapital + "_state_6D_pose";
    }
    // Case 2: Frame Transform
    else if (isCharInCharArray<num6DStates>(stateCategory, dim6StateSymbols)) {
      // Get Frame Pair
      std::pair<std::string, std::string> framePair;
      if (gtsamTransformsExpressionKeys_.getFramePairFromGtsamKey(framePair, key)) {
        frameInformation = framePair.first + "_to_" + framePair.second;
      } else {
        REGULAR_COUT << RED_START << " Could not find frame pair for key: " << symbol << COLOR_END << std::endl;
        throw std::runtime_error("Could not find frame pair for key.");
      }
      // State Category
      stateCategoryString = stateCategoryCapital + "_6D_transform_";
    }
    // Otherwise: Undefined --> throw error
    else {
      throw std::runtime_error(stateCategoryString + " is an unknown 6D state category. Check the admissible GTSAM symbols in the config.");
    }
    // Put together the identifier
    std::string transformIdentifier = stateCategoryString + frameInformation;

    // A.B Write to file -----------------------------------------------------------
    // Check if we already have a file stream for this category --> if not, create one
    if (fileStreams.find(transformIdentifier) == fileStreams.end()) {
      // If not, create a new file stream for this category
      const std::string fileName = savePath + timeString + "_" + transformIdentifier + ".csv";
      REGULAR_COUT << GREEN_START << " Saving optimized states to file: " << COLOR_END << fileName << std::endl;
      // Open for writing and appending
      fileStreams[transformIdentifier].open(fileName, std::ofstream::out | std::ofstream::app);
      // Write header
      fileStreams[transformIdentifier] << "time, x, y, z, quat_x, quat_y, quat_z, quat_w, roll, pitch, yaw\n";
    }

    // Write the values to the appropriate file
    fileStreams[transformIdentifier] << std::setprecision(14) << timeStamp << ", " << pose.x() << ", " << pose.y() << ", " << pose.z()
                                     << ", " << pose.rotation().toQuaternion().x() << ", " << pose.rotation().toQuaternion().y() << ", "
                                     << pose.rotation().toQuaternion().z() << ", " << pose.rotation().toQuaternion().w() << ", "
                                     << pose.rotation().roll() << ", " << pose.rotation().pitch() << ", " << pose.rotation().yaw() << "\n";
  }  // end of for loop over all pose states

  // B. 3D R(3) states (e.g. velocity, calibration displacement, landmarks) -----------------------------------------------------------
  for (const auto& keyVectorPair : optimizedValues.extract<gtsam::Point3>()) {
    // Read out information
    const gtsam::Key& key = keyVectorPair.first;
    const gtsam::Vector& vector = keyVectorPair.second;
    const gtsam::Symbol symbol(key);
    const char stateCategory = symbol.chr();
    const double timeStamp = keyTimestampMap.at(key);

    // Additional strings
    std::string stateCategoryString = "";
    std::string frameInformation = "";

    // State Category Capital
    const std::string stateCategoryCapital = std::to_string(std::toupper(stateCategory));

    // B.A Creation of the identifier for the file -----------------------------------
    // Case 1: Navigation State Velocity
    if (stateCategory == 'v') {
      stateCategoryString = stateCategoryCapital + "_state_3D_velocity_";
    }
    // Case 2: Point3 (e.g. calibration displacement, landmarks)
    else if (isCharInCharArray<num3DStates>(stateCategory, dim3StateSymbols)) {
      // Get Frame Pair
      std::pair<std::string, std::string> framePair;
      if (gtsamTransformsExpressionKeys_.getFramePairFromGtsamKey(framePair, key)) {
        frameInformation = framePair.first + "_to_" + framePair.second;
      } else {
        REGULAR_COUT << RED_START << " Could not find frame pair for key: " << symbol.chr() << COLOR_END << std::endl;
      }
      // State Category
      stateCategoryString = stateCategoryCapital + "_3D_vector_";
    }
    // Otherwise: Undefined --> throw error
    else {
      throw std::runtime_error(stateCategoryString + " is an unknown 3D state category. Check the admissible GTSAM symbols in the config.");
    }
    // Put together the identifier
    std::string transformIdentifier = stateCategoryString + frameInformation;

    // B.B Write to file -----------------------------------------------------------
    // Check if we already have a file stream for this category --> if not, create one
    if (fileStreams.find(transformIdentifier) == fileStreams.end()) {
      // If not, create a new file stream for this category
      std::string fileName = savePath + timeString + "_" + transformIdentifier + ".csv";
      REGULAR_COUT << GREEN_START << " Saving optimized states to file: " << COLOR_END << fileName << std::endl;
      // Open for writing and appending
      fileStreams[transformIdentifier].open(fileName, std::ofstream::out | std::ofstream::app);
      // Write header
      fileStreams[transformIdentifier] << "time, x, y, z\n";
    }

    // Write the values to the appropriate file
    fileStreams[transformIdentifier] << std::setprecision(14) << timeStamp << ", " << vector.x() << ", " << vector.y() << ", " << vector.z()
                                     << "\n";
  }  // end of for loop over all vector states

  // C. Bias states (accelerometer and gyroscope) -----------------------------------------------------------
  for (const auto& keyBiasPair : optimizedValues.extract<gtsam::imuBias::ConstantBias>()) {
    // Read out information
    const gtsam::Key& key = keyBiasPair.first;
    const gtsam::imuBias::ConstantBias& bias = keyBiasPair.second;
    const gtsam::Symbol symbol(key);
    const char stateCategory = symbol.chr();
    const double timeStamp = keyTimestampMap.at(key);

    // Assert that the symbol is a bias
    assert(stateCategory == 'b');

    // Capitalized state category
    const std::string stateCategoryCapital = std::to_string(std::toupper(stateCategory));

    // C.A Creation of the identifier for the file -----------------------------------
    std::string stateCategoryString = stateCategoryCapital + "_imu_bias_";

    // C.B Write to file -----------------------------------------------------------
    // Check if we already have a file stream for this category --> if not, create one
    if (fileStreams.find(stateCategoryString) == fileStreams.end()) {
      // If not, create a new file stream for this category
      std::string fileName = savePath + timeString + "_" + stateCategoryString + ".csv";
      REGULAR_COUT << GREEN_START << " Saving optimized states to file: " << COLOR_END << fileName << std::endl;
      // Open for writing and appending
      fileStreams[stateCategoryString].open(fileName, std::ofstream::out | std::ofstream::app);
      // Write header
      fileStreams[stateCategoryString] << "time, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z\n";
    }

    // Write the values to the appropriate file
    fileStreams[stateCategoryString] << std::setprecision(14) << timeStamp << ", " << bias.accelerometer().x() << ", "
                                     << bias.accelerometer().y() << ", " << bias.accelerometer().z() << ", " << bias.gyroscope().x() << ", "
                                     << bias.gyroscope().y() << ", " << bias.gyroscope().z() << "\n";
  }

  // Close all file streams
  for (auto& pair : fileStreams) {
    pair.second.close();
  }
}

// Save optimized Graph to Common Open source G2o format
void GraphManager::saveOptimizedGraphToG2o(const OptimizerBase& optimizedGraph, const gtsam::Values& optimizedValues,
                                           const std::string& saveFileName) {
  // Safe optimized states
  gtsam::writeG2o(optimizedGraph.getNonlinearFactorGraph(), optimizedValues, saveFileName);
}

// Calculate State at Key
gtsam::NavState GraphManager::calculateStateAtKey(bool& computeSuccessfulFlag, const gtsam::Key& key) {
  return calculateNavStateAtKey(computeSuccessfulFlag, rtOptimizerPtr_, key, __func__);
}

// Private --------------------------------------------------------------------

// Update of the two IMU pre-integrators
void GraphManager::updateImuIntegrators_(const TimeToImuMap& imuMeas) {
  if (imuMeas.size() < 2) {
    REGULAR_COUT << " Received less than 2 IMU messages --- No Preintegration done." << std::endl;
    return;
  } else if (imuMeas.size() > 2) {
    REGULAR_COUT << RED_START << "Currently only supporting two IMU messages for pre-integration." << COLOR_END << std::endl;
    throw std::runtime_error("Terminating.");
  }

  // Start integrating with imu_meas.begin()+1 meas to calculate dt,
  // imu_meas.begin() meas was integrated before
  auto currItr = imuMeas.begin();
  auto prevItr = currItr++;

  // Calculate dt and integrate IMU measurements for both preintegrators
  for (; currItr != imuMeas.end(); ++currItr, ++prevItr) {
    double dt = currItr->first - prevItr->first;
    imuStepPreintegratorPtr_->integrateMeasurement(currItr->second.acceleration,       // acc
                                                   currItr->second.angularVelocity,    // gyro
                                                   dt);                                // delta t
    imuBufferPreintegratorPtr_->integrateMeasurement(currItr->second.acceleration,     // acc
                                                     currItr->second.angularVelocity,  // gyro
                                                     dt);
  }
}

// Returns true if the factors/values were added without any problems
// Otherwise returns false (e.g. if exception occurs while adding
bool GraphManager::addFactorsToSmootherAndOptimize(const gtsam::NonlinearFactorGraph& newRtGraphFactors,
                                                   const gtsam::Values& newRtGraphValues,
                                                   const std::map<gtsam::Key, double>& newRtGraphKeysTimestampsMap,
                                                   const gtsam::NonlinearFactorGraph& newBatchGraphFactors,
                                                   const gtsam::Values& newBatchGraphValues,
                                                   const std::map<gtsam::Key, double>& newBatchGraphKeysTimestampsMap,
                                                   const std::shared_ptr<GraphConfig>& graphConfigPtr, const int additionalIterations) {
  // Timing
  std::chrono::time_point<std::chrono::high_resolution_clock> startLoopTime = std::chrono::high_resolution_clock::now();
  std::chrono::time_point<std::chrono::high_resolution_clock> endLoopTime;

  // Lock for optimization (as shared rtOptimizer is optimized)
  const std::lock_guard<std::mutex> optimization(optimizationRunningMutex_);

  // Perform update of the real-time smoother, including optimization
  bool successFlag = rtOptimizerPtr_->update(newRtGraphFactors, newRtGraphValues, newRtGraphKeysTimestampsMap);
  // Additional iterations
  for (size_t itr = 0; itr < additionalIterations; ++itr) {
    successFlag = successFlag && rtOptimizerPtr_->update();
  }

  // Add Factors and States to Batch Optimization (if desired) without running optimization
  if (graphConfigPtr->useAdditionalSlowBatchSmoother_) {
    batchOptimizerPtr_->update(newBatchGraphFactors, newBatchGraphValues, newBatchGraphKeysTimestampsMap);
  }

  // Logging
  if (graphConfigPtr->verboseLevel_ > 0) {
    endLoopTime = std::chrono::high_resolution_clock::now();
    REGULAR_COUT << GREEN_START << " Whole optimization loop took "
                 << std::chrono::duration_cast<std::chrono::milliseconds>(endLoopTime - startLoopTime).count() << " milliseconds."
                 << COLOR_END << std::endl;
  }

  return successFlag;
}

bool GraphManager::findGraphKeys_(gtsam::Key& closestKeyKm1, gtsam::Key& closestKeyK, double& keyTimeStampDistance,
                                  const double maxTimestampDistance, const double timeKm1, const double timeK, const std::string& name) {
  // Find closest lidar keys in existing graph
  double closestGraphTimeKm1, closestGraphTimeK;
  {
    // Looking up from IMU buffer --> acquire mutex (otherwise values for key
    // might not be set)
    const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
    bool success =
        timeToKeyBufferPtr_->getClosestKeyAndTimestamp(closestGraphTimeKm1, closestKeyKm1, name + " km1", maxTimestampDistance, timeKm1);
    success =
        success && timeToKeyBufferPtr_->getClosestKeyAndTimestamp(closestGraphTimeK, closestKeyK, name + " k", maxTimestampDistance, timeK);
    if (!success) {
      REGULAR_COUT << RED_START << " Could not find closest keys for " << name << COLOR_END << std::endl;
      return false;
    }
  }

  // Check
  if (closestGraphTimeKm1 > closestGraphTimeK) {
    REGULAR_COUT << RED_START << " Time at time step k-1 must be smaller than time at time step k." << COLOR_END << std::endl;
    return false;
  }

  keyTimeStampDistance = std::abs(closestGraphTimeK - closestGraphTimeKm1);
  if (keyTimeStampDistance > maxTimestampDistance) {
    REGULAR_COUT << " Distance of " << name << " timestamps is too big. Found timestamp difference is  "
                 << closestGraphTimeK - closestGraphTimeKm1 << " which is larger than the maximum admissible distance of "
                 << maxTimestampDistance << ". Still adding constraints to graph." << COLOR_END << std::endl;
  }
  return true;
}

void GraphManager::writeValueKeysToKeyTimeStampMap_(const gtsam::Values& values, const double measurementTime,
                                                    std::shared_ptr<std::map<gtsam::Key, double>> keyTimestampMapPtr) {
  for (const auto& value : values) {
    writeKeyToKeyTimeStampMap_(value.key, measurementTime, keyTimestampMapPtr);
  }
}

}  // namespace graph_msf