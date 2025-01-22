/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#define MIN_ITERATIONS_BEFORE_REMOVING_STATIC_TRANSFORM 200

// C++
#include <chrono>
#include <filesystem>
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
#include "graph_msf/core/GraphManager.h"
#include "graph_msf/geometry/conversions.h"
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
    : graphConfigPtr_(std::move(graphConfigPtr)), imuFrame_(std::move(imuFrame)), worldFrame_(std::move(worldFrame)) {
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
  if (graphConfigPtr_->useAdditionalSlowBatchSmootherFlag_) {
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
  // Rt Smoother
  addFactorsToRtSmootherAndOptimize(newRtGraphFactors, newRtGraphValues, *priorKeyTimestampMapPtr, graphConfigPtr_, 0);
  // Batch Smoother
  addFactorsToBatchSmootherAndOptimize(newBatchGraphFactors, newBatchGraphValues, *priorKeyTimestampMapPtr, graphConfigPtr_);

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
    if (graphConfigPtr_->useAdditionalSlowBatchSmootherFlag_) {
      batchGraphValuesBufferPtr_->insert(valuesEstimate);
    }

    // Add timestamps
    writeValueKeysToKeyTimeStampMap_(valuesEstimate, imuTimeK, rtGraphKeysTimestampsMapBufferPtr_);
    if (graphConfigPtr_->useAdditionalSlowBatchSmootherFlag_) {
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

  // Potentially log real-time state to container
  if (graphConfigPtr_->logRealTimeStateToMemoryFlag_) {
    realTimeWorldPoseContainer_[imuTimeK] = W_imuPropagatedState_.pose();
    realTimeOdomPoseContainer_[imuTimeK] = O_imuPropagatedState_.pose();
  }
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
        return true;
      }
      // Too far in the future --> do not add it
      else {
        std::cout << 1000 * (propagatedStateTime_ - unaryMeasurement.timeK()) << std::endl;
        std::cout << 1000 * (returnedGraphTime - unaryMeasurement.timeK()) << std::endl;
        REGULAR_COUT << RED_START << " Factor coming from the future, AND time deviation of " << typeid(unaryMeasurement).name()
                     << " at key " << returnedKey << " is " << 1000 * std::abs(returnedGraphTime - unaryMeasurement.timeK())
                     << " ms, being larger than admissible deviation of " << 4 * 1000 * graphConfigPtr_->maxSearchDeviation_
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

gtsam::NavState GraphManager::calculateNavStateAtGeneralKey(bool& computeSuccessfulFlag,
                                                            const std::shared_ptr<graph_msf::OptimizerBase> optimizerPtr,
                                                            const gtsam::Key& generalKey, const char* callingFunctionName) {
  // Nav State (pose and velocity)
  gtsam::Pose3 resultPose;
  gtsam::Vector3 resultVelocity;
  try {
    resultPose = optimizerPtr->calculateEstimatedPose3(gtsam::symbol_shorthand::X(generalKey));  // auto result = mainGraphPtr_->estimate();
    resultVelocity = optimizerPtr->calculateEstimatedVelocity3(gtsam::symbol_shorthand::V(generalKey));
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

Eigen::Matrix<double, 6, 6> GraphManager::calculatePoseCovarianceAtKeyInWorldFrame(std::shared_ptr<graph_msf::OptimizerBase> graphPtr,
                                                                                   const gtsam::Key& graphKey,
                                                                                   const char* callingFunctionName,
                                                                                   std::optional<const gtsam::NavState> optionalNavState) {
  // Pose Covariance in Tangent Space
  std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
  gtsam::Matrix66 resultPoseCovarianceBodyFrame = graphPtr->calculateMarginalCovarianceMatrixAtKey(graphKey);
  std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
  if (!optionalNavState.has_value() && duration > 400) {
    std::cout << RED_START << "Time to calculate marginal covariance: " << duration << " microseconds, which is too much." << std::endl;
  }
  // Transform covariance from I_S_I in body frame, to W_S_W in world frame
  // Get NavState
  gtsam::NavState navState;
  if (optionalNavState.has_value()) {
    navState = optionalNavState.value();
  } else {
    gtsam::Pose3 resultPose = graphPtr->calculateEstimatedPose3(graphKey);  // auto result = mainGraphPtr_->estimate();
    gtsam::Point3 resultVelocity = gtsam::Point3::Identity();  // Assume Zero Velocity, but adjoint below is still fine to map to world
    navState = gtsam::NavState(resultPose, resultVelocity);
  }
  // Compute adjoint matrix
  gtsam::Matrix66 adjointMatrix = navState.pose().AdjointMap();
  // Transform covariance
  gtsam::Matrix66 resultPoseCovarianceWorldFrame = adjointMatrix * resultPoseCovarianceBodyFrame * adjointMatrix.transpose();
  // Return
  return resultPoseCovarianceWorldFrame;
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
    if (graphConfigPtr_->useAdditionalSlowBatchSmootherFlag_) {
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

  // Graph Update (time consuming) -------------------------------------------
  bool successfulOptimizationFlag = true;
  // Rt Smoother
  if (newRtGraphFactors.size() > 0) {
    successfulOptimizationFlag = addFactorsToRtSmootherAndOptimize(newRtGraphFactors, newRtGraphValues, newRtGraphKeysTimestampsMap,
                                                                   graphConfigPtr_, graphConfigPtr_->additionalOptimizationIterations_);
  } else if (graphConfigPtr_->verboseLevel_ > 3) {
    REGULAR_COUT << " No factors present in rt smoother, not optimizing." << std::endl;
  }
  // Batch Smoother
  if (graphConfigPtr_->useAdditionalSlowBatchSmootherFlag_ && newBatchGraphFactors.size() > 0) {
    successfulOptimizationFlag =
        addFactorsToBatchSmootherAndOptimize(newBatchGraphFactors, newBatchGraphValues, newBatchGraphKeysTimestampsMap, graphConfigPtr_);
  } else if (graphConfigPtr_->verboseLevel_ > 3) {
    REGULAR_COUT << " No factors present in batch smoother, not optimizing." << std::endl;
  }

  // Return if optimization failed
  if (!successfulOptimizationFlag) {
    REGULAR_COUT << RED_START << " Graph optimization failed. " << COLOR_END << std::endl;
    return;
  }

  // Compute entire results ----------------------------------------------
  // A. NavState ------------------------------
  gtsam::NavState resultNavState =
      calculateNavStateAtGeneralKey(successfulOptimizationFlag, rtOptimizerPtr_, currentPropagatedKey, __func__);
  // B. Bias ------------------------------
  gtsam::imuBias::ConstantBias resultBias = rtOptimizerPtr_->calculateEstimatedBias(gtsam::symbol_shorthand::B(currentPropagatedKey));
  // C. Compute & Transform Covariances ------------------------------
  // Pose Covariance in World Frame
  gtsam::Matrix66 resultPoseCovarianceWorldFrame =
      calculatePoseCovarianceAtKeyInWorldFrame(rtOptimizerPtr_, gtsam::symbol_shorthand::X(currentPropagatedKey), __func__, resultNavState);
  // Velocity Covariance
  gtsam::Matrix33 resultVelocityCovariance =
      rtOptimizerPtr_->calculateMarginalCovarianceMatrixAtKey(gtsam::symbol_shorthand::V(currentPropagatedKey));

  // D. FixedFrame Transformations ------------------------------
  // Containers
  TransformsDictionary<Eigen::Isometry3d> resultReferenceFrameTransformations(Eigen::Isometry3d::Identity());
  TransformsDictionary<Eigen::Matrix<double, 6, 6>> resultReferenceFrameTransformationsCovariance(gtsam::Z_6x6);
  TransformsDictionary<Eigen::Isometry3d> resultLandmarkTransformations(Eigen::Isometry3d::Identity());
  TransformsDictionary<Eigen::Matrix<double, 6, 6>> resultLandmarkTransformationsCovariance(gtsam::Z_6x6);

  // Actual computation of fixed-frame transformations
  if (graphConfigPtr_->optimizeReferenceFramePosesWrtWorldFlag_) {
    // Mutex because we are changing the dynamically allocated graphKeys
    std::lock_guard<std::mutex> modifyGraphKeysLock(gtsamDynamicExpressionKeys_.get<gtsam::Pose3>().mutex());

    // Iterate through all dynamically allocated variables (holistic, calibration, landmarks) --------------------------------
    for (auto& framePairKeyMapIterator : gtsamDynamicExpressionKeys_.get<gtsam::Pose3>().getTransformsMap()) {
      // Get Variable Key
      const gtsam::Key& gtsamKey = framePairKeyMapIterator.second.key();  // alias

      // Check whether it is landmark state
      bool isLandmarkState = false;
      if (framePairKeyMapIterator.second.getVariableTypeEnum() == DynamicVariableTypeEnum::Landmark) {
        isLandmarkState = true;
      }

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
            T_frame1_frame2_covariance = rtOptimizerPtr_->calculateMarginalCovarianceMatrixAtKey(gtsamKey);
            // Add current belief back to the map
            framePairKeyMapIterator.second.updateLatestEstimate(T_frame1_frame2, T_frame1_frame2_covariance);
            // Update the initial guess for the offline graph
            if (graphConfigPtr_->useAdditionalSlowBatchSmootherFlag_) {
              gtsam::Values valuesEstimate;
              valuesEstimate.insert(gtsamKey, T_frame1_frame2);
              std::ignore = batchOptimizerPtr_->updateExistingValues(valuesEstimate);
            }
          }
          // 3D Position3 Vectors
          else if (isCharInCharArray<numDynamic3DStates>(stateCategory, dim3StateSymbols)) {
            gtsam::Point3 point3Estimate = rtOptimizerPtr_->calculateEstimatedPoint3(gtsamKey);
            T_frame1_frame2 = gtsam::Pose3(gtsam::Rot3::Identity(), point3Estimate);
            T_frame1_frame2_covariance.block<3, 3>(3, 3) = rtOptimizerPtr_->calculateMarginalCovarianceMatrixAtKey(gtsamKey);
            // Add current belief back to the map
            framePairKeyMapIterator.second.updateLatestEstimate(T_frame1_frame2, T_frame1_frame2_covariance);
            // Update the initial guess for the offline graph
            if (graphConfigPtr_->useAdditionalSlowBatchSmootherFlag_) {
              gtsam::Values valuesEstimate;
              valuesEstimate.insert(gtsamKey, point3Estimate);
              std::ignore = batchOptimizerPtr_->updateExistingValues(valuesEstimate);
            }
          }
          // Only these two types are allowed for dynamic states for now
          else {
            throw std::runtime_error("Key is neither a pose nor a displacement key.");
          }

          // Transform quantities to world frame (instead of keyframe)
          // For correction first have to transform T_W_F to T_F_W, apply correction, and then transform back to T_W_F
          gtsam::Pose3 T_frame2_frame1_corrected = T_frame1_frame2.inverse();
          T_frame2_frame1_corrected =
              gtsam::Pose3(T_frame2_frame1_corrected.rotation(),
                           T_frame2_frame1_corrected.translation() + framePairKeyMapIterator.second.getReferenceFrameKeyframePosition());
          gtsam::Pose3 T_frame1_frame2_corrected = T_frame2_frame1_corrected.inverse();
          // Write to container
          if (isLandmarkState) {
            resultLandmarkTransformations.set_T_frame1_frame2(framePairKeyMapIterator.first.first, framePairKeyMapIterator.first.second,
                                                              Eigen::Isometry3d(T_frame1_frame2_corrected.matrix()));
            resultLandmarkTransformationsCovariance.set_T_frame1_frame2(framePairKeyMapIterator.first.first,
                                                                        framePairKeyMapIterator.first.second, T_frame1_frame2_covariance);
          } else {
            resultReferenceFrameTransformations.set_T_frame1_frame2(framePairKeyMapIterator.first.first,
                                                                    framePairKeyMapIterator.first.second,
                                                                    Eigen::Isometry3d(T_frame1_frame2_corrected.matrix()));
            resultReferenceFrameTransformationsCovariance.set_T_frame1_frame2(
                framePairKeyMapIterator.first.first, framePairKeyMapIterator.first.second, T_frame1_frame2_covariance);
          }

          // Mark that this key has at least been optimized once
          if (framePairKeyMapIterator.second.getNumberStepsOptimized() == 0) {
            if (graphConfigPtr_->verboseLevel_ >= VerbosityLevels::kFunctioningAndStateMachine1) {
              REGULAR_COUT << " Fixed-frame Transformation between " << framePairKeyMapIterator.first.first << " and "
                           << framePairKeyMapIterator.first.second << " optimized for the first time." << COLOR_END << std::endl;
            }
            if (graphConfigPtr_->verboseLevel_ >= VerbosityLevels::kOperationInformation3) {
              REGULAR_COUT << GREEN_START << " Result, RPY (deg): " << T_frame1_frame2.rotation().rpy().transpose() * (180.0 / M_PI)
                           << ", t (x, y, z): " << T_frame1_frame2.translation().transpose() << COLOR_END << std::endl;
            }
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
              gtsamDynamicExpressionKeys_.get<gtsam::Pose3>().removeOrDeactivateTransform(framePairKeyMapIterator.first.first,
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
            gtsamDynamicExpressionKeys_.get<gtsam::Pose3>().removeOrDeactivateTransform(framePairKeyMapIterator.first.first,
                                                                                        framePairKeyMapIterator.first.second);
            return;
          }
          // If active but added newly but never optimized
          else {
            // If too old but was never optimized --> remove or deactivate
            double variableAge = framePairKeyMapIterator.second.computeVariableAge(currentPropagatedTime);
            if (variableAge > graphConfigPtr_->realTimeSmootherLag_) {
              REGULAR_COUT << YELLOW_START << "GMsf-GraphManager" << RED_START << " Fixed Frame Transformation between "
                           << framePairKeyMapIterator.first.first << " and " << framePairKeyMapIterator.first.second << " is too old ("
                           << variableAge
                           << ")s and was never optimized. Removing from optimization and adding again freshly at next "
                              "possibility."
                           << COLOR_END << std::endl;
              // Remove state from state dictionary
              gtsamDynamicExpressionKeys_.get<gtsam::Pose3>().removeOrDeactivateTransform(framePairKeyMapIterator.first.first,
                                                                                          framePairKeyMapIterator.first.second);
            }
            // Otherwise keep for now and potentially print out
            else if (graphConfigPtr_->verboseLevel_ >= 1) {
              REGULAR_COUT
                  << YELLOW_START << " Tried to query the transformation and/or covariance for frame pair "
                  << framePairKeyMapIterator.first.first << " to " << framePairKeyMapIterator.first.second
                  << ", at key: " << gtsam::Symbol(gtsamKey)
                  << ". Not yet available, as it was not yet optimized. Waiting for next optimization iteration until publishing it. "
                     "Current state key: "
                  << currentPropagatedKey << COLOR_END << std::endl;
            }
          }
        }  // catch statement
      }    // end: if active statement
    }      // for loop over all transforms
  }

  // Mutex block 2 ------------------
  {
    // Lock
    const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);

    // 1. Optimized Graph State Status
    optimizedGraphState_.setIsOptimized();
    // Update Optimized Graph State
    optimizedGraphState_.updateNavStateAndBias(currentPropagatedKey, currentPropagatedTime, resultNavState,
                                               resultBias.correctGyroscope(currentAngularVelocity), resultBias);
    optimizedGraphState_.updateReferenceFrameTransforms(resultReferenceFrameTransformations);
    optimizedGraphState_.updateReferenceFrameTransformsCovariance(resultReferenceFrameTransformationsCovariance);
    optimizedGraphState_.updateLandmarkTransforms(resultLandmarkTransformations);
    optimizedGraphState_.updateLandmarkTransformsCovariance(resultLandmarkTransformationsCovariance);
    optimizedGraphState_.updateCovariances(resultPoseCovarianceWorldFrame, resultVelocityCovariance);

    // 2. State in World: Predict from solution to obtain refined propagated state
    if (graphConfigPtr_->usingBiasForPreIntegrationFlag_) {
      W_imuPropagatedState_ = imuBufferPreintegratorPtr_->predict(resultNavState, optimizedGraphState_.imuBias());
    } else {
      W_imuPropagatedState_ = imuBufferPreintegratorPtr_->predict(resultNavState, gtsam::imuBias::ConstantBias());
    }

    // 3. State in Odom
    // Correct rotation only for roll and pitch, keep integrated yaw
    gtsam::Rot3 R_O_I_rp_corrected =
        gtsam::Rot3::Ypr(O_imuPropagatedState_.pose().rotation().yaw(), W_imuPropagatedState_.pose().rotation().pitch(),
                         W_imuPropagatedState_.pose().rotation().roll());
    // Rotate corrected velocity to odom frame
    gtsam::Vector3 O_v_O_I = R_O_I_rp_corrected * W_imuPropagatedState_.bodyVelocity();
    // Update the NavState
    O_imuPropagatedState_ = gtsam::NavState(R_O_I_rp_corrected, O_imuPropagatedState_.pose().translation(), O_v_O_I);

    // 4. Transformation between World and Odom
    T_W_O_ = Eigen::Isometry3d((W_imuPropagatedState_.pose() * O_imuPropagatedState_.pose().inverse()).matrix());

    // Update the time of the last optimized state
    lastOptimizedStateTime_ = currentPropagatedTime;
  }  // end of locking
}

bool GraphManager::optimizeSlowBatchSmoother(int maxIterations, const std::string& savePath, const bool saveCovarianceFlag) {
  if (graphConfigPtr_->useAdditionalSlowBatchSmootherFlag_) {
    // Time duration of optimization
    std::chrono::time_point<std::chrono::high_resolution_clock> startOptimizationTime = std::chrono::high_resolution_clock::now();
    // Optimization
    batchOptimizerPtr_->optimize(maxIterations);
    const gtsam::Values& optimizedStateValues = batchOptimizerPtr_->getAllOptimizedStates();
    // Key to timestamp map
    const std::map<gtsam::Key, double>& keyTimestampMap = batchOptimizerPtr_->getFullKeyTimestampMap();
    // Calculate Duration
    std::chrono::time_point<std::chrono::high_resolution_clock> endOptimizationTime = std::chrono::high_resolution_clock::now();
    double optimizationDuration =
        std::chrono::duration_cast<std::chrono::milliseconds>(endOptimizationTime - startOptimizationTime).count();
    std::cout << "Optimization took " << optimizationDuration / 1000 << " seconds." << std::endl;
    std::cout << "Number of optimization variables: " << optimizedStateValues.size() << std::endl;

    // Save Optimized Result
    saveOptimizedValuesToFile(optimizedStateValues, keyTimestampMap, savePath, saveCovarianceFlag);

    // Return
    return true;
  } else {
    return false;
  }
}

// Logging of the real-time states
bool GraphManager::logRealTimeStates(const std::string& savePath, const std::string& timeString) {
  // Note enabled
  if (!graphConfigPtr_->logRealTimeStateToMemoryFlag_) {
    REGULAR_COUT << RED_START << " Logging of real-time states is disabled. " << COLOR_END << std::endl;
    return false;
  }
  // Enabled
  else {
    // Main container: map, although only one element
    std::map<std::string, std::ofstream> fileStreams;

    // Create directory if it does not exist
    if (!std::filesystem::exists(savePath + timeString)) {
      std::filesystem::create_directories(savePath + timeString);
    }

    // Create File Streams
    // CSV
    FileLogger::createPose3CsvFileStream(fileStreams, savePath, "rt_world_x_state_6D_pose", timeString, false);
    FileLogger::createPose3CsvFileStream(fileStreams, savePath, "rt_odom_x_state_6D_pose", timeString, false);
    // TUM
    FileLogger::createPose3TumFileStream(fileStreams, savePath, "rt_world_x_state_6D_pose_tum", timeString);
    FileLogger::createPose3TumFileStream(fileStreams, savePath, "rt_odom_x_state_6D_pose_tum", timeString);

    // Iterate through all states and write them to file
    // A. World Pose
    for (const auto& statePair : realTimeWorldPoseContainer_) {
      // Write to files
      // CSV
      FileLogger::writePose3ToCsvFile(fileStreams, statePair.second, "rt_world_x_state_6D_pose", statePair.first, false);
      // TUM
      FileLogger::writePose3ToTumFile(fileStreams, statePair.second, "rt_world_x_state_6D_pose_tum", statePair.first);
    }
    // B. Odom Pose
    for (const auto& statePair : realTimeOdomPoseContainer_) {
      // Write to files
      // CSV
      FileLogger::writePose3ToCsvFile(fileStreams, statePair.second, "rt_odom_x_state_6D_pose", statePair.first, false);
      // TUM
      FileLogger::writePose3ToTumFile(fileStreams, statePair.second, "rt_odom_x_state_6D_pose_tum", statePair.first);
    }
    return true;
  }
}

// Save optimized values to file
void GraphManager::saveOptimizedValuesToFile(const gtsam::Values& optimizedValues, const std::map<gtsam::Key, double>& keyTimestampMap,
                                             const std::string& savePath, const bool saveCovarianceFlag) {
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

  // Create directory if it does not exist
  if (!std::filesystem::exists(savePath + timeString)) {
    std::filesystem::create_directories(savePath + timeString);
  }

  // Save real-time states
  if (graphConfigPtr_->logRealTimeStateToMemoryFlag_) {
    std::ignore = logRealTimeStates(savePath, timeString);
  }

  // Save optimized states
  // A. 6D SE(3) states -----------------------------------------------------------
  for (const auto& keyPosePair : optimizedValues.extract<gtsam::Pose3>()) {
    // Read out information
    const gtsam::Key& graphKey = keyPosePair.first;
    gtsam::Pose3 pose = keyPosePair.second;
    const gtsam::Symbol stateSymbol(graphKey);
    const char stateCategory = stateSymbol.chr();
    const double timeStamp = keyTimestampMap.at(graphKey);

    // Additional strings
    std::string stateCategoryString = "";
    std::string frameInformation = "";

    // State Category Capital
    const std::string stateCategoryCapital(1, static_cast<char>(std::toupper(stateCategory)));

    // A.A Creation of the identifier for the file -----------------------------------
    // Case 1: Navigation State Pose
    if (stateCategory == 'x') {
      stateCategoryString = stateCategoryCapital + "_state_6D_pose";
    }
    // Case 2: Frame Transform
    else if (isCharInCharArray<num6DStates>(stateCategory, dim6StateSymbols)) {
      // State Category
      stateCategoryString = stateCategoryCapital + "_6D_transform";
      // Get Frame Pair
      std::pair<std::string, std::string> framePair;
      if (gtsamDynamicExpressionKeys_.get<gtsam::Pose3>().getFramePairFromGtsamKey(framePair, graphKey)) {
        frameInformation = "_" + framePair.first + "_to_" + framePair.second;
      } else {
        REGULAR_COUT << RED_START << " Could not find frame pair for key: " << stateSymbol << COLOR_END << std::endl;
        throw std::runtime_error("Could not find frame pair for key.");
      }
    }
    // Otherwise: Undefined --> throw error
    else {
      throw std::logic_error(stateCategoryString + " is an unknown 6D state category. Check the admissible GTSAM symbols in the config.");
    }
    // Put together the identifier
    std::string transformIdentifier = stateCategoryString + frameInformation;
    // Replace / with underscore
    std::replace(transformIdentifier.begin(), transformIdentifier.end(), '/', '_');

    // Check for the keyframe position
    Eigen::Vector3d keyframePosition = Eigen::Vector3d::Zero();
    gtsamDynamicExpressionKeys_.get<gtsam::Pose3>().getKeyframePositionFromGtsamKey(keyframePosition, graphKey);

    // A.B Write to file -----------------------------------------------------------
    // Check if we already have a file stream for this category --> if not, create one
    // CSV file for Pose3
    fileLogger_.createPose3CsvFileStream(fileStreams, savePath, transformIdentifier, timeString, saveCovarianceFlag);
    // TUM file for Pose3
    fileLogger_.createPose3TumFileStream(fileStreams, savePath, transformIdentifier + "_tum", timeString);

    // Compute Covariance of Pose
    Eigen::Matrix<double, 6, 6> poseCovarianceInWorldRos;
    if (saveCovarianceFlag) {
      gtsam::Matrix66 poseCovarianceInWorldGtsam = calculatePoseCovarianceAtKeyInWorldFrame(batchOptimizerPtr_, graphKey, __func__);
      // Convert to ROS Format
      poseCovarianceInWorldRos = convertCovarianceGtsamConventionToRosConvention(poseCovarianceInWorldGtsam);
    }

    // If keyframe position is not zero, move the frame location to capture the random walk
    if (keyframePosition.norm() > 1e-6) {
      const gtsam::Pose3& T_W_M = pose;  // alias
      gtsam::Pose3 T_M_W_corrected = T_W_M.inverse();
      T_M_W_corrected = gtsam::Pose3(T_M_W_corrected.rotation(),
                                     T_M_W_corrected.translation() + keyframePosition);  // Remove the effect of the keyframe position
      gtsam::Pose3 T_W_M_corrected = T_M_W_corrected.inverse();
      pose = T_W_M_corrected;
      REGULAR_COUT << " Corrected frame position for " << transformIdentifier << " (" << gtsam::Symbol(graphKey) << ") by "
                   << keyframePosition.transpose() << std::endl;
    }

    // Write the values to the appropriate file
    // CSV file for Pose3
    fileLogger_.writePose3ToCsvFile(fileStreams, pose, transformIdentifier, timeStamp, saveCovarianceFlag, poseCovarianceInWorldRos);
    // TUM file for Pose3
    fileLogger_.writePose3ToTumFile(fileStreams, pose, transformIdentifier + "_tum", timeStamp);
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

    // State Category Non-Capital (as it is a vector instead of a matrix)
    const std::string stateCategoryNonCapital(1, stateCategory);

    // B.A Creation of the identifier for the file -----------------------------------
    // Case 1: Navigation State Velocity
    if (stateCategory == 'v') {
      stateCategoryString = stateCategoryNonCapital + "_state_3D_velocity";
    }
    // Case 2: Point3 (e.g. calibration displacement, landmarks), TODO: no landmarks are saved for now
    else if (isCharInCharArray<num3DStates>(stateCategory, dim3StateSymbols)) {
      // State Category
      stateCategoryString = stateCategoryNonCapital + "_3D_vector";
      // Skip landmarks for now
      if (stateCategory == 'l') {
        continue;
      }

      // Get Frame Pair
      std::pair<std::string, std::string> framePair;
      if (gtsamDynamicExpressionKeys_.get<gtsam::Pose3>().getFramePairFromGtsamKey(framePair, key)) {
        frameInformation = "_" + framePair.first + "_to_" + framePair.second;
      } else {
        REGULAR_COUT << RED_START << " Could not find frame pair for key: " << symbol.chr() << COLOR_END << std::endl;
      }
    }
    // Otherwise: Undefined --> throw error
    else {
      throw std::logic_error(stateCategoryString + " is an unknown 3D state category. Check the admissible GTSAM symbols in the config.");
    }
    // Put together the identifier
    std::string transformIdentifier = stateCategoryString + frameInformation;

    // B.B Write to file -----------------------------------------------------------
    // Check if we already have a file stream for this category --> if not, create one
    fileLogger_.createPoint3CsvFileStream(fileStreams, savePath, transformIdentifier, timeString);

    // Write the values to the appropriate file
    fileLogger_.writePoint3ToCsvFile(fileStreams, vector, transformIdentifier, timeStamp);
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
    const std::string stateCategoryCapital(1, static_cast<char>(std::toupper(stateCategory)));
    ;

    // C.A Creation of the identifier for the file -----------------------------------
    std::string stateCategoryIdentifier = stateCategoryCapital + "_imu_bias";

    // C.B Write to file -----------------------------------------------------------
    // Check if we already have a file stream for this category --> if not, create one
    fileLogger_.createImuBiasCsvFileStream(fileStreams, savePath, stateCategoryIdentifier, timeString);

    // Write the values to the appropriate file
    fileLogger_.writeImuBiasToCsvFile(fileStreams, bias, stateCategoryIdentifier, timeStamp);
  }

  // Close all file streams
  for (auto& pair : fileStreams) {
    pair.second.close();
  }

  // Print
  REGULAR_COUT << GREEN_START << " Saved all optimized states to files." << COLOR_END << std::endl;
}

// Save optimized Graph to Common Open source G2o format
void GraphManager::saveOptimizedGraphToG2o(const OptimizerBase& optimizedGraph, const gtsam::Values& optimizedValues,
                                           const std::string& saveFileName) {
  // Safe optimized states
  gtsam::writeG2o(optimizedGraph.getNonlinearFactorGraph(), optimizedValues, saveFileName);
}

// Calculate State at Key
gtsam::NavState GraphManager::calculateStateAtGeneralKey(bool& computeSuccessfulFlag, const gtsam::Key& generalKey) {
  return calculateNavStateAtGeneralKey(computeSuccessfulFlag, rtOptimizerPtr_, generalKey, __func__);
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

  // Calculate dt and integrate IMU measurements for both pre-integrators
  for (; currItr != imuMeas.end(); ++currItr, ++prevItr) {
    double dt = currItr->first - prevItr->first;
    imuStepPreintegratorPtr_->integrateMeasurement(currItr->second.acceleration,       // acc
                                                   currItr->second.angularVelocity,    // gyro
                                                   dt);                                // delta t
    imuBufferPreintegratorPtr_->integrateMeasurement(currItr->second.acceleration,     // acc
                                                     currItr->second.angularVelocity,  // gyro
                                                     dt);                              // delta t
  }
}

// Returns true if the factors/values were added without any problems
// Otherwise returns false (e.g. if exception occurs while adding
bool GraphManager::addFactorsToRtSmootherAndOptimize(const gtsam::NonlinearFactorGraph& newRtGraphFactors,
                                                     const gtsam::Values& newRtGraphValues,
                                                     const std::map<gtsam::Key, double>& newRtGraphKeysTimestampsMap,
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

  // Logging
  if (graphConfigPtr->verboseLevel_ >= VerbosityLevels::kOptimizationEffort2) {
    endLoopTime = std::chrono::high_resolution_clock::now();
    REGULAR_COUT << GREEN_START << " Whole optimization loop took "
                 << std::chrono::duration_cast<std::chrono::milliseconds>(endLoopTime - startLoopTime).count() << " milliseconds."
                 << COLOR_END << std::endl;
  }

  return successFlag;
}

// Batch Smoother
bool GraphManager::addFactorsToBatchSmootherAndOptimize(const gtsam::NonlinearFactorGraph& newBatchGraphFactors,
                                                        const gtsam::Values& newBatchGraphValues,
                                                        const std::map<gtsam::Key, double>& newBatchGraphKeysTimestampsMap,
                                                        const std::shared_ptr<GraphConfig>& graphConfigPtr) {
  // Lock for optimization (as shared rtOptimizer is optimized)
  const std::lock_guard<std::mutex> optimization(optimizationRunningMutex_);

  // Add Factors and States to Batch Optimization (if desired) without running optimization
  bool successFlag = false;
  if (graphConfigPtr->useAdditionalSlowBatchSmootherFlag_) {
    successFlag = batchOptimizerPtr_->update(newBatchGraphFactors, newBatchGraphValues, newBatchGraphKeysTimestampsMap);
  }

  // Return
  return successFlag;
}

// Find closest keys in graph
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