/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#define REGULAR_COUT std::cout << YELLOW_START << "GMSF-GraphManager" << COLOR_END
#define MIN_ITERATIONS_BEFORE_REMOVING_STATIC_TRANSFORM 200

// C++
#include <string>
#include <type_traits>
#include <utility>

// IO
#include <gtsam/slam/dataset.h>

// Factors
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/expressions.h>

// Workspace
#include "graph_msf/core/GraphManager.hpp"
// ISAM2
#include "graph_msf/core/optimizer/OptimizerIsam2Batch.hpp"
#include "graph_msf/core/optimizer/OptimizerIsam2FixedLag.hpp"
// LM
#include "graph_msf/core/optimizer/OptimizerLMBatch.hpp"
#include "graph_msf/core/optimizer/OptimizerLMFixedLag.hpp"

namespace graph_msf {

// Public --------------------------------------------------------------------

GraphManager::GraphManager(std::shared_ptr<GraphConfig> graphConfigPtr, std::string imuFrame, std::string worldFrame)
    : graphConfigPtr_(std::move(graphConfigPtr)),
      imuFrame_(std::move(imuFrame)),
      worldFrame_(std::move(worldFrame)),
      resultFixedFrameTransformations_(Eigen::Isometry3d::Identity()),
      resultFixedFrameTransformationsCovariance_(Eigen::Matrix<double, 6, 6>::Zero()) {
  // Buffer
  factorGraphBufferPtr_ = std::make_shared<gtsam::NonlinearFactorGraph>();
  graphValuesBufferPtr_ = std::make_shared<gtsam::Values>();
  graphKeysTimestampsMapBufferPtr_ = std::make_shared<std::map<gtsam::Key, double>>();

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
  gtsam::NonlinearFactorGraph newGraphFactors;
  gtsam::Values newGraphValues;
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
    graphValuesBufferPtr_->insert(valuesEstimate);
    writeValueKeysToKeyTimeStampMap_(valuesEstimate, timeStamp, priorKeyTimestampMapPtr);

    // Initialize graph -------------------------------------------------
    factorGraphBufferPtr_->resize(0);
    factorGraphBufferPtr_->emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
        gtsam::symbol_shorthand::X(propagatedStateKey_), T_W_I0,
        priorPoseNoise);  // POSE - PriorFactor format is (key,value,matrix) value
    // is same type as type of PriorFactor
    factorGraphBufferPtr_->emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(gtsam::symbol_shorthand::V(propagatedStateKey_),
                                                                              gtsam::Vector3(0, 0, 0),
                                                                              priorVelocityNoise);  // VELOCITY
    factorGraphBufferPtr_->emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(gtsam::symbol_shorthand::B(propagatedStateKey_),
                                                                                            *imuBiasPriorPtr_,
                                                                                            priorBiasNoise);  // BIAS

    // Copy over
    newGraphFactors = *factorGraphBufferPtr_;
    newGraphValues = *graphValuesBufferPtr_;
    factorGraphBufferPtr_->resize(0);
    graphValuesBufferPtr_->clear();
  }

  /// Add prior factor to graph and optimize for the first time ----------------
  addFactorsToSmootherAndOptimize(newGraphFactors, newGraphValues, *priorKeyTimestampMapPtr, graphConfigPtr_, 0);

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

    // Add to key buffer
    timeToKeyBufferPtr_->addToBuffer(imuTimeK, newKey);

    // Add IMU Factor to graph
    gtsam::CombinedImuFactor imuFactor(gtsam::symbol_shorthand::X(oldKey), gtsam::symbol_shorthand::V(oldKey),
                                       gtsam::symbol_shorthand::X(newKey), gtsam::symbol_shorthand::V(newKey),
                                       gtsam::symbol_shorthand::B(oldKey), gtsam::symbol_shorthand::B(newKey), *imuStepPreintegratorPtr_);
    addFactorToGraph_<const gtsam::CombinedImuFactor*>(&imuFactor, imuTimeK, "imu");

    // Add IMU values
    gtsam::Values valuesEstimate;
    valuesEstimate.insert(gtsam::symbol_shorthand::X(newKey), W_imuPropagatedState_.pose());
    valuesEstimate.insert(gtsam::symbol_shorthand::V(newKey), W_imuPropagatedState_.velocity());
    valuesEstimate.insert(gtsam::symbol_shorthand::B(newKey), optimizedGraphState_.imuBias());
    graphValuesBufferPtr_->insert(valuesEstimate);
    writeValueKeysToKeyTimeStampMap_(valuesEstimate, imuTimeK, graphKeysTimestampsMapBufferPtr_);

    // After adding this factor we can again empty the step integrator
    // Reset IMU Step Preintegration
    if (graphConfigPtr_->usingBiasForPreIntegrationFlag_) {
      imuStepPreintegratorPtr_->resetIntegrationAndSetBias(optimizedGraphState_.imuBias());
    } else {
      imuStepPreintegratorPtr_->resetIntegrationAndSetBias(gtsam::imuBias::ConstantBias());
    }
  }  // End of create new state
}

// Unary factors ----------------------------------------------------------------
// Key Lookup
bool GraphManager::getUnaryFactorGeneralKey(gtsam::Key& returnedKey, const UnaryMeasurement& unaryMeasurement) {
  // Find the closest key in existing graph
  double closestGraphTime;
  if (!timeToKeyBufferPtr_->getClosestKeyAndTimestamp(closestGraphTime, returnedKey, unaryMeasurement.measurementName(),
                                                      graphConfigPtr_->maxSearchDeviation_, unaryMeasurement.timeK())) {
    if (propagatedStateTime_ - unaryMeasurement.timeK() < 0.0) {  // Factor is coming from the future, hence add it to the buffer
      // TODO: Add to buffer and return --> still add it until we are there
    } else {  // Otherwise do not add it
      REGULAR_COUT << RED_START << " Time deviation of " << typeid(unaryMeasurement).name() << " at key " << returnedKey << " is "
                   << 1000 * std::abs(closestGraphTime - unaryMeasurement.timeK()) << " ms, being larger than admissible deviation of "
                   << 1000 * graphConfigPtr_->maxSearchDeviation_ << " ms. Not adding to graph." << COLOR_END << std::endl;
      return false;
    }
  }
  return true;
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
  addFactorSafelyToGraph_<const gtsam::BetweenFactor<gtsam::Pose3>*>(&poseBetweenFactor, timeKm1);

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
  gtsam::Pose3 resultPose;
  gtsam::Vector3 resultVelocity;
  try {
    resultPose = optimizerPtr->calculateEstimatedPose(gtsam::symbol_shorthand::X(key));  // auto result = mainGraphPtr_->estimate();
    resultVelocity = optimizerPtr->calculateEstimatedVelocity(gtsam::symbol_shorthand::V(key));
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
  // Method variables
  gtsam::NonlinearFactorGraph newGraphFactors;
  gtsam::Values newGraphValues;
  std::map<gtsam::Key, double> newGraphKeysTimestampsMap;
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
    // Get copy of factors and values
    newGraphFactors = *factorGraphBufferPtr_;
    newGraphValues = *graphValuesBufferPtr_;
    newGraphKeysTimestampsMap = *graphKeysTimestampsMapBufferPtr_;
    // Empty buffers
    factorGraphBufferPtr_->resize(0);
    graphValuesBufferPtr_->clear();
    graphKeysTimestampsMapBufferPtr_->clear();

    // Empty Buffer Pre-integrator --> everything missed during the update will
    // be in here
    if (graphConfigPtr_->usingBiasForPreIntegrationFlag_) {
      imuBufferPreintegratorPtr_->resetIntegrationAndSetBias(optimizedGraphState_.imuBias());
    } else {
      imuBufferPreintegratorPtr_->resetIntegrationAndSetBias(gtsam::imuBias::ConstantBias());
    }
  }  // end of locking

  // Graph Update (time consuming) -------------------
  bool successfulOptimizationFlag = addFactorsToSmootherAndOptimize(newGraphFactors, newGraphValues, newGraphKeysTimestampsMap,
                                                                    graphConfigPtr_, graphConfigPtr_->additionalOptimizationIterations_);
  if (!successfulOptimizationFlag) {
    REGULAR_COUT << RED_START << " Graph optimization failed. " << COLOR_END << std::endl;
    return;
  }

  // Compute entire result
  // NavState
  gtsam::NavState resultNavState = calculateNavStateAtKey(successfulOptimizationFlag, rtOptimizerPtr_, currentPropagatedKey, __func__);
  // Bias
  gtsam::imuBias::ConstantBias resultBias = rtOptimizerPtr_->calculateEstimatedBias(gtsam::symbol_shorthand::B(currentPropagatedKey));
  // Compute Covariance
  gtsam::Matrix66 resultPoseCovarianceBodyFrame = rtOptimizerPtr_->marginalCovariance(gtsam::symbol_shorthand::X(currentPropagatedKey));
  gtsam::Matrix33 resultVelocityCovariance = rtOptimizerPtr_->marginalCovariance(gtsam::symbol_shorthand::V(currentPropagatedKey));
  // Transform covariance from I_S_I in body frame, to W_S_W in world frame
  gtsam::Matrix66 adjointMatrix = resultNavState.pose().AdjointMap();
  gtsam::Matrix66 resultPoseCovarianceWorldFrame = adjointMatrix * resultPoseCovarianceBodyFrame * adjointMatrix.transpose();

  // FixedFrame Transformations
  if (graphConfigPtr_->optimizeFixedFramePosesWrtWorld_) {
    // Mutex because we are changing the dynamically allocated graphKeys
    std::lock_guard<std::mutex> modifyGraphKeysLock(gtsamExpressionTransformsKeys_.mutex());

    // Iterate through all dynamically allocated transforms (holistic and calibration) --------------------------------
    for (auto& framePairIterator : gtsamExpressionTransformsKeys_.getTransformsMap()) {
      // Get Transform
      const gtsam::Key& key = framePairIterator.second.key();  // alias

      // Case 1: Try to compute results and uncertainties ------------------------------------------
      try {  // Obtain estimate and covariance from the extrinsic transformations
        gtsam::Pose3 T_frame1_frame2;
        gtsam::Matrix66 T_frame1_frame2_covariance = gtsam::Z_6x6;

        // 6D Transformation
        if (gtsam::Symbol(key).string()[0] == 't') {
          T_frame1_frame2 = rtOptimizerPtr_->calculateEstimatedPose(key);
          T_frame1_frame2_covariance = rtOptimizerPtr_->marginalCovariance(key);
        }
        // 3D Displacement
        else if (gtsam::Symbol(key).string()[0] == 'd') {
          T_frame1_frame2 = gtsam::Pose3(gtsam::Rot3(), rtOptimizerPtr_->calculateEstimatedDisplacement(key));
          T_frame1_frame2_covariance.block<3, 3>(3, 3) = rtOptimizerPtr_->marginalCovariance(key);
        }
        // Only these two are supported for now
        else {
          throw std::runtime_error("Key is neither a pose nor a displacement key.");
        }

        // Write to Result Dictionaries
        Eigen::Matrix4d T_frame1_frame2_corrected_matrix = T_frame1_frame2.matrix();
        T_frame1_frame2_corrected_matrix.block<3, 1>(0, 3) += framePairIterator.second.getMeasurementOriginPosition();
        // T_frame1_frame2 = gtsam::Pose3(T_frame1_frame2_matrix);
        resultFixedFrameTransformations_.set_T_frame1_frame2(framePairIterator.first.first, framePairIterator.first.second,
                                                             Eigen::Isometry3d(T_frame1_frame2_corrected_matrix));
        resultFixedFrameTransformationsCovariance_.set_T_frame1_frame2(framePairIterator.first.first, framePairIterator.first.second,
                                                                       T_frame1_frame2_covariance);

        // Mark that this key has at least been optimized once
        if (framePairIterator.second.getNumberStepsOptimized() == 0) {
          REGULAR_COUT << GREEN_START << " Fixed-frame Transformation between " << framePairIterator.first.first << " and "
                       << framePairIterator.first.second << " optimized for the first time." << COLOR_END << std::endl;
          REGULAR_COUT << GREEN_START << " Result, RPY (deg): " << T_frame1_frame2.rotation().rpy().transpose() * (180.0 / M_PI)
                       << ", t (x, y, z): " << T_frame1_frame2.translation().transpose() << COLOR_END << std::endl;
        }
        // Increase Counter
        framePairIterator.second.incrementNumberStepsOptimized();

        // Check health status of transformation --> Delete if diverged too much --> only necessary for global fixed frames
        if (framePairIterator.first.second == worldFrame_ &&
            framePairIterator.second.getNumberStepsOptimized() > MIN_ITERATIONS_BEFORE_REMOVING_STATIC_TRANSFORM) {
          const gtsam::Pose3& T_frame1_frame2_initial = framePairIterator.second.getApproximateTransformationBeforeOptimization();  // alias
          const double errorTangentSpace = gtsam::Pose3::Logmap(T_frame1_frame2_initial.between(T_frame1_frame2)).norm();
          // Check error in tangent space
          // std::cout << "Error in tangent space: " << errorTangentSpace << std::endl;
          if (errorTangentSpace > graphConfigPtr_->fixedFramePosesResetThreshold_) {
            REGULAR_COUT << RED_START << "Error in tangent space: " << errorTangentSpace << std::endl;
            std::cout << YELLOW_START << "GMsf-GraphManager" << RED_START << " Fixed Frame Transformation between "
                      << framePairIterator.first.first << " and " << framePairIterator.first.second
                      << " diverged too much. Removing from optimization and adding again freshly at next possibility." << COLOR_END
                      << std::endl;
            // Remove state from state dictionary
            gtsamExpressionTransformsKeys_.removeTransform(framePairIterator.first.first, framePairIterator.first.second);
            break;
          }
        }
      }  // end: try statement
      // Case 2: Result computation failed -----------------------------------------
      catch (const std::out_of_range& exception) {
        if (framePairIterator.second.getNumberStepsOptimized() > 0) {  // Was optimized before, so should also be available now in the graph
                                                                       // --> as querying was unsuccessful, we remove it from the graph
          REGULAR_COUT
              << RED_START << " OutOfRange-exception while querying the transformation and/or covariance at key " << gtsam::Symbol(key)
              << ", for frame pair " << framePairIterator.first.first << "," << framePairIterator.first.second << std::endl
              << " This happens if the requested variable is outside of the smoother window. Hence, we keep this estimate unchanged "
                 "and remove it from the state dictionary. To fix this, increase the smoother window."
              << COLOR_END << std::endl;
          // Remove state from state dictionary
          gtsamExpressionTransformsKeys_.removeTransform(framePairIterator.first.first, framePairIterator.first.second);
          return;
        } else {
          REGULAR_COUT << GREEN_START << " Tried to query the transformation and/or covariance for frame pair "
                       << framePairIterator.first.first << " to " << framePairIterator.first.second << ", at key: " << gtsam::Symbol(key)
                       << ". Not yet available, as it was not yet optimized. Waiting for next optimization iteration until publishing it. "
                          "Current state key: "
                       << currentPropagatedKey << COLOR_END << std::endl;
        }
      }  // catch statement
    }    // for loop over all transforms
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
  // Map to hold file streams, keyed by category
  std::map<char, std::ofstream> fileStreams;

  // Get current time as string for file name
  std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  // String of time
  std::string timeString = std::ctime(&now);
  // Replace spaces with underscores
  std::replace(timeString.begin(), timeString.end(), ' ', '_');

  // Save optimized states
  // SE(3) states
  for (const auto& keyPosePair : optimizedValues.extract<gtsam::Pose3>()) {
    // Read out information
    const gtsam::Key& key = keyPosePair.first;
    const gtsam::Pose3& pose = keyPosePair.second;
    const gtsam::Symbol symbol(key);
    const char stateCategory = symbol.chr();
    const double timeStamp = keyTimestampMap.at(key);

    // Check if we already have a file stream for this category --> if not, create one
    if (fileStreams.find(stateCategory) == fileStreams.end()) {
      // If not, create a new file stream for this category
      std::string fileName = savePath + "optimized_state-" + std::string(1, symbol.chr()) + "_" + timeString + ".csv";
      REGULAR_COUT << GREEN_START << " Saving optimized states to file: " << COLOR_END << fileName << std::endl;
      // Open for writing and appending
      fileStreams[stateCategory].open(fileName, std::ofstream::out | std::ofstream::app);
      // Write header
      fileStreams[stateCategory] << "time, x, y, z, roll, pitch, yaw\n";
    }

    // Write the values to the appropriate file
    fileStreams[stateCategory] << std::setprecision(14) << timeStamp << ", " << pose.x() << ", " << pose.y() << ", " << pose.z() << ", "
                               << pose.rotation().roll() << ", " << pose.rotation().pitch() << ", " << pose.rotation().yaw() << "\n";
  }

  // Close all file streams
  for (auto& pair : fileStreams) {
    pair.second.close();
  }

  // R(3) states (e.g. Velocity)
  // TODO: later the remaining states
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
bool GraphManager::addFactorsToSmootherAndOptimize(const gtsam::NonlinearFactorGraph& newGraphFactors, const gtsam::Values& newGraphValues,
                                                   const std::map<gtsam::Key, double>& newGraphKeysTimestampsMap,
                                                   const std::shared_ptr<GraphConfig>& graphConfigPtr, const int additionalIterations) {
  // Timing
  std::chrono::time_point<std::chrono::high_resolution_clock> startLoopTime = std::chrono::high_resolution_clock::now();
  std::chrono::time_point<std::chrono::high_resolution_clock> endLoopTime;

  // Lock for optimization (as shared rtOptimizer is optimized)
  const std::lock_guard<std::mutex> optimization(optimizationRunningMutex_);

  // Perform update of the real-time smoother, including optimization
  bool successFlag = rtOptimizerPtr_->update(newGraphFactors, newGraphValues, newGraphKeysTimestampsMap);
  // Additional iterations
  for (size_t itr = 0; itr < additionalIterations; ++itr) {
    successFlag = successFlag && rtOptimizerPtr_->update();
  }

  // Add Factors and States to Batch Optimization (if desired) without running optimization
  if (graphConfigPtr->useAdditionalSlowBatchSmoother_) {
    batchOptimizerPtr_->update(newGraphFactors, newGraphValues, newGraphKeysTimestampsMap);
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