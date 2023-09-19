/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#define WORST_CASE_OPTIMIZATION_TIME 0.1  // in seconds
#define REGULAR_COUT std::cout << YELLOW_START << "GMSF-GraphManager" << COLOR_END
#define MIN_ITERATIONS_BEFORE_REMOVING_STATIC_TRANSFORM 100

// C++
#include <type_traits>

// Factors
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/expressions.h>

// Workspace
#include "graph_msf/core/GraphManager.hpp"
#include "graph_msf/core/OptimizerIsam2.hpp"

namespace graph_msf {

// Public --------------------------------------------------------------------

GraphManager::GraphManager(std::shared_ptr<GraphConfig> graphConfigPtr, const std::string& imuFrame, const std::string& worldFrame)
    : graphConfigPtr_(graphConfigPtr),
      imuFrame_(imuFrame),
      worldFrame_(worldFrame),
      gtsamExpressionTransformsKeys_(graphConfigPtr->smootherLag),
      resultFixedFrameTransformations_(Eigen::Isometry3d::Identity()),
      resultFixedFrameTransformationsCovariance_(Eigen::Matrix<double, 6, 6>::Zero()) {
  // Buffer
  factorGraphBufferPtr_ = std::make_shared<gtsam::NonlinearFactorGraph>();
  graphValuesBufferPtr_ = std::make_shared<gtsam::Values>();
  graphKeysTimestampsMapBufferPtr_ = std::make_shared<std::map<gtsam::Key, double>>();

  // Keys
  timeToKeyBufferPtr_ = std::make_shared<TimeGraphKeyBuffer>(graphConfigPtr_->imuBufferLength, graphConfigPtr_->verboseLevel);

  // Optimizer
  if (graphConfigPtr_->useIsamFlag) {
    optimizerPtr_ = std::make_shared<OptimizerIsam2>(graphConfigPtr_);
  } else {
    // optimizerPtr_ = std::make_shared<OptimizerLM>(graphConfigPtr_,
    // worldFrame_); Not implmented
    REGULAR_COUT << RED_START << " OptimizerLM is not implemented yet." << COLOR_END << std::endl;
    std::runtime_error("OptimizerLM is not implemented yet.");
  }
}

// Initialization Interface ---------------------------------------------------
bool GraphManager::initImuIntegrators(const double g) {
  // Gravity direction definition
  imuParamsPtr_ = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(g);  // ROS convention

  // Set noise and bias parameters
  /// Position
  imuParamsPtr_->setAccelerometerCovariance(gtsam::Matrix33::Identity(3, 3) * std::pow(graphConfigPtr_->accNoiseDensity, 2));
  imuParamsPtr_->setIntegrationCovariance(gtsam::Matrix33::Identity(3, 3) *
                                          std::pow(graphConfigPtr_->integrationNoiseDensity, 2));  // error committed in integrating
                                                                                                   // position from velocities
  imuParamsPtr_->setUse2ndOrderCoriolis(graphConfigPtr_->use2ndOrderCoriolisFlag);
  /// Rotation
  imuParamsPtr_->setGyroscopeCovariance(gtsam::Matrix33::Identity(3, 3) * std::pow(graphConfigPtr_->gyroNoiseDensity, 2));
  imuParamsPtr_->setOmegaCoriolis(gtsam::Vector3(0, 0, 1) * graphConfigPtr_->omegaCoriolis);
  /// Bias
  imuParamsPtr_->setBiasAccCovariance(gtsam::Matrix33::Identity(3, 3) * std::pow(graphConfigPtr_->accBiasRandomWalkNoiseDensity, 2));
  imuParamsPtr_->setBiasOmegaCovariance(gtsam::Matrix33::Identity(3, 3) * std::pow(graphConfigPtr_->gyroBiasRandomWalkNoiseDensity, 2));
  imuParamsPtr_->setBiasAccOmegaInit(gtsam::Matrix66::Identity(6, 6) * std::pow(graphConfigPtr_->biasAccOmegaInit, 2));

  // Use previously defined prior for gyro
  imuBiasPriorPtr_ = std::make_shared<gtsam::imuBias::ConstantBias>(graphConfigPtr_->accBiasPrior, graphConfigPtr_->gyroBiasPrior);

  // Init Pre-integrators
  imuStepPreintegratorPtr_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(imuParamsPtr_, *imuBiasPriorPtr_);
  imuBufferPreintegratorPtr_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(imuParamsPtr_, *imuBiasPriorPtr_);
  imuParamsPtr_->print("GraphMSF-IMU PreIntegration Parameters:");
  return true;
}

bool GraphManager::initPoseVelocityBiasGraph(const double timeStep, const gtsam::Pose3& initialPose) {
  // Create Prior factor ----------------------------------------------------
  /// Prior factor noise
  auto priorPoseNoise =
      gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-6, 1e-6, 1e-6).finished());  // rad,rad,rad,m, m, m
  auto priorVelocityNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e-3);                                        // m/s
  auto priorBiasNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3).finished());

  // Looking up from IMU buffer --> acquire mutex (otherwise values for key
  // might not be set)
  const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);

  // Initial estimate
  gtsam::Values valuesEstimate;
  REGULAR_COUT << " Initial Pose: " << initialPose << std::endl;
  valuesEstimate.insert(gtsam::symbol_shorthand::X(propagatedStateKey_), initialPose);
  valuesEstimate.insert(gtsam::symbol_shorthand::V(propagatedStateKey_), gtsam::Vector3(0, 0, 0));
  valuesEstimate.insert(gtsam::symbol_shorthand::B(propagatedStateKey_), *imuBiasPriorPtr_);
  /// Timestamp mapping for incremental fixed lag smoother
  std::shared_ptr<std::map<gtsam::Key, double>> priorKeyTimestampMapPtr = std::make_shared<std::map<gtsam::Key, double>>();
  writeValueKeysToKeyTimeStampMap_(valuesEstimate, timeStep, priorKeyTimestampMapPtr);

  // Initialize graph -------------------------------------------------
  factorGraphBufferPtr_->resize(0);
  factorGraphBufferPtr_->emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      gtsam::symbol_shorthand::X(propagatedStateKey_), initialPose,
      priorPoseNoise);  // POSE - PriorFactor format is (key,value,matrix) value
                        // is same type as type of PriorFactor
  factorGraphBufferPtr_->emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(gtsam::symbol_shorthand::V(propagatedStateKey_),
                                                                            gtsam::Vector3(0, 0, 0),
                                                                            priorVelocityNoise);  // VELOCITY
  factorGraphBufferPtr_->emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(gtsam::symbol_shorthand::B(propagatedStateKey_),
                                                                                          *imuBiasPriorPtr_,
                                                                                          priorBiasNoise);  // BIAS

  /// Add prior factor to graph and optimize for the first time ----------------
  if (graphConfigPtr_->useIsamFlag) {
    addFactorsToSmootherAndOptimize(optimizerPtr_, *factorGraphBufferPtr_, valuesEstimate, *priorKeyTimestampMapPtr, graphConfigPtr_, 0);
  } else {
    throw std::runtime_error("Optimizer has to be ISAM at this stage.");
  }

  factorGraphBufferPtr_->resize(0);

  // Update Current State ---------------------------------------------------
  optimizedGraphState_.updateNavStateAndBias(propagatedStateKey_, timeStep, gtsam::NavState(initialPose, gtsam::Vector3(0, 0, 0)),
                                             gtsam::Vector3(0, 0, 0), *imuBiasPriorPtr_);
  O_imuPropagatedState_ = gtsam::NavState(initialPose, gtsam::Vector3(0, 0, 0));
  W_imuPropagatedState_ = gtsam::NavState(initialPose, gtsam::Vector3(0, 0, 0));
  return true;
}

// IMU at the core --------------------------------------------------------------
void GraphManager::addImuFactorAndGetState(SafeIntegratedNavState& changedPreIntegratedNavState,
                                           std::shared_ptr<SafeNavStateWithCovarianceAndBias>& newOptimizedNavStatePtr,
                                           const std::shared_ptr<ImuBuffer> imuBufferPtr, const double imuTimeK) {
  // Looking up from IMU buffer --> acquire mutex (otherwise values for key might not be set)
  const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);

  // Get new key
  gtsam::Key oldKey = propagatedStateKey_;
  gtsam::Key newKey = newPropagatedStateKey_();

  // Add to key buffer
  timeToKeyBufferPtr_->addToBuffer(imuTimeK, newKey);

  // Get last two measurements from buffer to determine dt
  TimeToImuMap imuMeas;
  imuBufferPtr->getLastTwoMeasurements(imuMeas);
  propagatedStateTime_ = imuTimeK;
  currentAngularVelocity_ = imuMeas.rbegin()->second.angularVelocity;

  // Update IMU preintegrator
  updateImuIntegrators_(imuMeas);
  // Predict propagated state
  if (graphConfigPtr_->usingBiasForPreIntegrationFlag) {
    O_imuPropagatedState_ =
        imuBufferPtr->integrateNavStateFromTimestamp(imuMeas.begin()->first, imuMeas.rbegin()->first, O_imuPropagatedState_,
                                                     optimizedGraphState_.imuBias(), graphConfigPtr_->W_gravityVector);
  } else {
    O_imuPropagatedState_ =
        imuBufferPtr->integrateNavStateFromTimestamp(imuMeas.begin()->first, imuMeas.rbegin()->first, O_imuPropagatedState_,
                                                     gtsam::imuBias::ConstantBias(), graphConfigPtr_->W_gravityVector);
  }

  // Add IMU Factor to graph
  gtsam::CombinedImuFactor imuFactor(gtsam::symbol_shorthand::X(oldKey), gtsam::symbol_shorthand::V(oldKey),
                                     gtsam::symbol_shorthand::X(newKey), gtsam::symbol_shorthand::V(newKey),
                                     gtsam::symbol_shorthand::B(oldKey), gtsam::symbol_shorthand::B(newKey), *imuStepPreintegratorPtr_);
  addFactorToGraph_<const gtsam::CombinedImuFactor*>(&imuFactor, imuTimeK);

  // Add IMU values
  gtsam::Values valuesEstimate;
  valuesEstimate.insert(gtsam::symbol_shorthand::X(newKey), W_imuPropagatedState_.pose());
  valuesEstimate.insert(gtsam::symbol_shorthand::V(newKey), W_imuPropagatedState_.velocity());
  valuesEstimate.insert(gtsam::symbol_shorthand::B(newKey), optimizedGraphState_.imuBias());
  graphValuesBufferPtr_->insert(valuesEstimate);
  writeValueKeysToKeyTimeStampMap_(valuesEstimate, imuTimeK, graphKeysTimestampsMapBufferPtr_);

  // Generate return objects -------------------------------------------------
  gtsam::NavState& T_O_Ik_nav = O_imuPropagatedState_;  // Alias
  // Assign poses and velocities
  changedPreIntegratedNavState.update(T_W_O_, Eigen::Isometry3d(T_O_Ik_nav.pose().matrix()), T_O_Ik_nav.bodyVelocity(),
                                      optimizedGraphState_.imuBias().correctGyroscope(imuMeas.rbegin()->second.angularVelocity), imuTimeK,
                                      false);
  if (optimizedGraphState_.isOptimized()) {
    newOptimizedNavStatePtr = std::make_shared<SafeNavStateWithCovarianceAndBias>(optimizedGraphState_);
  } else {
    newOptimizedNavStatePtr = nullptr;
  }
}

// Unary factors ----------------------------------------------------------------
// Unary Commodity Functions
bool GraphManager::addUnaryFactorToReturnedKey(gtsam::Key& returnedKey, const graph_msf::UnaryMeasurement& unaryMeasurement) {
  // Find the closest key in existing graph
  double closestGraphTime;
  if (!timeToKeyBufferPtr_->getClosestKeyAndTimestamp(closestGraphTime, returnedKey, unaryMeasurement.measurementName(),
                                                      graphConfigPtr_->maxSearchDeviation, unaryMeasurement.timeK())) {
    if (propagatedStateTime_ - unaryMeasurement.timeK() <
        0.0) {  // Factor is coming from the future, hence add it to the buffer and adding it later
      // TODO: Add to buffer and return --> still add it until we are there
    } else {  // Otherwise do not add it
      REGULAR_COUT << RED_START << " Time deviation of " << typeid(unaryMeasurement).name() << " at key " << returnedKey << " is "
                   << 1000 * std::abs(closestGraphTime - unaryMeasurement.timeK()) << " ms, being larger than admissible deviation of "
                   << 1000 * graphConfigPtr_->maxSearchDeviation << " ms. Not adding to graph." << COLOR_END << std::endl;
      return false;
    }
  }
  return true;
}
// Unary meta method
typedef gtsam::Key (*F)(std::uint64_t);
template <class MEASUREMENT_TYPE, int NOISE_DIM, class FACTOR_TYPE, F SYMBOL_SHORTHAND>
void GraphManager::addUnaryFactorInImuFrame(const MEASUREMENT_TYPE& unaryMeasurement,
                                            const Eigen::Matrix<double, NOISE_DIM, 1>& unaryNoiseDensity, const double measurementTime) {
  // Find the closest key in existing graph
  double closestGraphTime;
  gtsam::Key closestKey;
  std::string callingName = "GnssPositionUnaryFactor";
  if (!timeToKeyBufferPtr_->getClosestKeyAndTimestamp(closestGraphTime, closestKey, callingName, graphConfigPtr_->maxSearchDeviation,
                                                      measurementTime)) {
    if (propagatedStateTime_ - measurementTime < 0.0) {  // Factor is coming from the future, hence add it to the buffer and adding it later
      // TODO: Add to buffer and return --> still add it until we are there
    } else {  // Otherwise do not add it
      REGULAR_COUT << RED_START << " Time deviation of " << typeid(FACTOR_TYPE).name() << " at key " << closestKey << " is "
                   << 1000 * std::abs(closestGraphTime - measurementTime) << " ms, being larger than admissible deviation of "
                   << 1000 * graphConfigPtr_->maxSearchDeviation << " ms. Not adding to graph." << COLOR_END << std::endl;
      return;
    }
  }

  // Create noise model
  auto noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(unaryNoiseDensity)));  // m,m,m
  auto robustErrorFunction = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.0), noise);

  // Create unary factor and ADD IT
  std::shared_ptr<FACTOR_TYPE> unaryFactorPtr;
  // Case 1: Expression factor --> must be handled differently
  if constexpr (std::is_same<gtsam::ExpressionFactor<MEASUREMENT_TYPE>, FACTOR_TYPE>::value) {
  } else {  // Case 2: No expression factor
    unaryFactorPtr = std::make_shared<FACTOR_TYPE>(SYMBOL_SHORTHAND(closestKey), unaryMeasurement, robustErrorFunction);
    // Write to graph
    addFactorSafelyToGraph_<const FACTOR_TYPE*>(unaryFactorPtr.get(), measurementTime);
  }

  // Print summary
  if (graphConfigPtr_->verboseLevel > 1) {
    REGULAR_COUT << " Current propagated key " << propagatedStateKey_ << GREEN_START << ", " << typeid(FACTOR_TYPE).name()
                 << " factor added to key " << closestKey << COLOR_END << std::endl;
  }
}

// Expression factors
template <class MEASUREMENT_TYPE, int NOISE_DIM, class EXPRESSION, class PRIOR_FACTOR_TYPE>
void GraphManager::addUnaryExpressionFactor(const MEASUREMENT_TYPE& unaryMeasurement,
                                            const Eigen::Matrix<double, NOISE_DIM, 1>& unaryNoiseDensity, const EXPRESSION& unaryExpression,
                                            const double measurementTime, const gtsam::Values& newStateValues,
                                            std::vector<gtsam::PriorFactor<PRIOR_FACTOR_TYPE>>& priorFactors,
                                            std::vector<gtsam::BetweenFactor<PRIOR_FACTOR_TYPE>>& randomWalkFactors,
                                            const bool useRobustNorm) {
  // Noise & Error Function
  auto noiseModel = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector(unaryNoiseDensity));  // rad,rad,rad,x,y,z
  boost::shared_ptr<gtsam::noiseModel::Base> errorFunction;
  if (useRobustNorm) {
    errorFunction = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.345), noiseModel);
  } else {
    errorFunction = noiseModel;
  }

  // Create Factor
  gtsam::ExpressionFactor<MEASUREMENT_TYPE> unaryExpressionFactor(errorFunction, unaryMeasurement, unaryExpression);

  // Operating on graph data
  const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
  addFactorToGraph_<const gtsam::ExpressionFactor<MEASUREMENT_TYPE>*>(&unaryExpressionFactor, measurementTime);
  for (const auto& key : unaryExpressionFactor.keys()) {
    writeKeyToKeyTimeStampMap_(key, measurementTime, graphKeysTimestampsMapBufferPtr_);
  }
  // If one of the states was newly created, then add it to the values
  if (newStateValues.size() > 0) {
    graphValuesBufferPtr_->insert(newStateValues);
  }
  // If new factors are there (due to newly generated factor in the very beginning or for regularization), add it to the graph
  if (priorFactors.size() > 0) {
    factorGraphBufferPtr_->add(priorFactors);
  }
  // Random walk factors to constrain evolution of transformations
  if (randomWalkFactors.size() > 0) {
    factorGraphBufferPtr_->add(randomWalkFactors);
  }

  // Print summary --------------------------------------
  if (graphConfigPtr_->verboseLevel > 0) {
    REGULAR_COUT << " Current propagated key " << propagatedStateKey_ << GREEN_START << ", expression factor of type "
                 << typeid(MEASUREMENT_TYPE).name() << " added to keys ";
    for (const auto& key : unaryExpressionFactor.keys()) {
      std::cout << gtsam::Symbol(key) << ", ";
    }
    std::cout << COLOR_END << std::endl;
  }
}

// Unary Specializations
void GraphManager::addPoseUnaryFactor(const UnaryMeasurementXD<Eigen::Isometry3d, 6>& unary6DMeasurement,
                                      const Eigen::Isometry3d& T_sensorFrame_imu) {
  // Measurement handling
  gtsam::Pose3 T_fixedFrame_I(unary6DMeasurement.unaryMeasurement().matrix() * T_sensorFrame_imu.matrix());

  // Also optimize over fixedFrame Transformation --------------------------------
  if (graphConfigPtr_->optimizeFixedFramePosesWrtWorld || graphConfigPtr_->optimizeExtrinsicSensorToSensorCorrectedOffset) {
    // Find the closest key in existing graph
    gtsam::Key closestKey;
    if (!addUnaryFactorToReturnedKey(closestKey, unary6DMeasurement)) {
      return;
    }

    // Main state expression
    gtsam::Pose3_ T_W_I_(gtsam::symbol_shorthand::X(closestKey));
    // Placeholder for fixed frame transformation
    gtsam::Pose3_ T_M_I_ = T_W_I_;

    // Dynamic keys for expression values
    gtsam::Values valuesEstimates;
    std::vector<gtsam::PriorFactor<gtsam::Pose3>> priorFactors;
    std::vector<gtsam::BetweenFactor<gtsam::Pose3>> randomWalkFactors;
    const std::string& fixedFrameName = unary6DMeasurement.fixedFrameName();  // Alias

    // Optimize over fixed frame poses --------------------------------------------
    if (graphConfigPtr_->optimizeFixedFramePosesWrtWorld) {
      // Random Walk
      Eigen::Matrix<double, 6, 1> randomWalkNoiseDensity = 0.0 * Eigen::Matrix<double, 6, 1>::Ones();  // All
      // Compute initial
      const gtsam::Pose3& T_W_I_est = W_imuPropagatedState_.pose();  // alias for readability
      gtsam::Pose3 T_W_M_initial;
      gtsam::Matrix66 initialNoiseCovariance;
      if (resultFixedFrameTransformations_.isFramePairInDictionary(worldFrame_, fixedFrameName) &&
          gtsamExpressionTransformsKeys_.isFramePairInDictionary(worldFrame_, fixedFrameName)) {
        T_W_M_initial = gtsam::Pose3(resultFixedFrameTransformations_.rv_T_frame1_frame2(worldFrame_, fixedFrameName).matrix());
        initialNoiseCovariance = resultFixedFrameTransformationsCovariance_.rv_T_frame1_frame2(worldFrame_, fixedFrameName);
      } else {
        T_W_M_initial = T_W_I_est * T_fixedFrame_I.inverse();
        initialNoiseCovariance = gtsam::noiseModel::Diagonal::Sigmas(1.0e-01 * gtsam::Vector::Ones(6))->covariance();
      }
      gtsam::Pose3_ T_W_M_ = gtsamExpressionTransformsKeys_.getTransformationExpression<6, gtsam::Pose3>(
          valuesEstimates, priorFactors, randomWalkFactors, T_W_M_initial, worldFrame_, unary6DMeasurement.fixedFrameName(),
          unary6DMeasurement.timeK(), initialNoiseCovariance, randomWalkNoiseDensity);
      T_M_I_ = gtsam::transformPoseTo(T_W_M_, T_W_I_);
    }
    // Optimize over extrinsics ---------------------------------------------------
    else if (graphConfigPtr_->optimizeExtrinsicSensorToSensorCorrectedOffset) {
      const gtsam::Pose3& T_I_Is_initial = gtsam::Pose3::Identity();  // alias for readability
      T_M_I_ = gtsam::Pose3_(T_M_I_, &gtsam::Pose3::transformPoseTo,
                             gtsamExpressionTransformsKeys_.getTransformationExpression<6, gtsam::Pose3>(
                                 valuesEstimates, priorFactors, randomWalkFactors, T_I_Is_initial, imuFrame_,
                                 imuFrame_ + "_" + unary6DMeasurement.sensorFrameName(), unary6DMeasurement.timeK(),
                                 gtsam::noiseModel::Diagonal::Sigmas(1.0e-01 * gtsam::Vector::Ones(6))->covariance(),
                                 1e-9 * Eigen::Matrix<double, 6, 1>::Ones()));
    }

    // Meta function
    addUnaryExpressionFactor<gtsam::Pose3, 6, gtsam::Pose3_, gtsam::Pose3>(
        T_fixedFrame_I, unary6DMeasurement.unaryMeasurementNoiseVariances(), T_M_I_, unary6DMeasurement.timeK(), valuesEstimates,
        priorFactors, randomWalkFactors, true);
  }
  // Simple unary factor --------------------------------------
  else {
    addUnaryFactorInImuFrame<gtsam::Pose3, 6, gtsam::PriorFactor<gtsam::Pose3>, gtsam::symbol_shorthand::X>(
        T_fixedFrame_I, unary6DMeasurement.unaryMeasurementNoiseDensity(), unary6DMeasurement.timeK());
  }
}

void GraphManager::addVelocityUnaryFactor(const gtsam::Vector3& velocity, const Eigen::Matrix<double, 3, 1>& velocityUnaryNoiseDensity,
                                          const double lidarTimeK) {
  addUnaryFactorInImuFrame<gtsam::Vector3, 3, gtsam::PriorFactor<gtsam::Vector3>, gtsam::symbol_shorthand::V>(
      velocity, velocityUnaryNoiseDensity, lidarTimeK);
}

void GraphManager::addPositionUnaryFactor(const UnaryMeasurementXD<Eigen::Vector3d, 3>& unaryPositionMeasurement,
                                          const std::optional<Eigen::Vector3d>& I_t_I_sensorFrame) {
  // Case 1: Need expression factor: i) optimize over fixed frame poses, ii) position not in IMU frame,  iii) optimize over extrinsics
  if (unaryPositionMeasurement.sensorFrameName() != imuFrame_ || graphConfigPtr_->optimizeFixedFramePosesWrtWorld ||
      graphConfigPtr_->optimizeExtrinsicSensorToSensorCorrectedOffset) {
    gtsam::Key closestKey;
    if (!this->addUnaryFactorToReturnedKey(closestKey, unaryPositionMeasurement)) {
      return;
    }
    // Dynamic keys for expression values
    gtsam::Values valuesEstimates;
    std::vector<gtsam::PriorFactor<gtsam::Pose3>> priorFactors;
    std::vector<gtsam::BetweenFactor<gtsam::Pose3>> randomWalkFactors;

    // Main state expression
    const gtsam::Point3_ W_t_W_I_ = gtsam::translation(gtsam::Pose3_(gtsam::symbol_shorthand::X(closestKey)));
    gtsam::Point3_ fixedFrame_t_fixedFrame_sensorFrame_ = W_t_W_I_;
    const gtsam::Rot3_ R_W_I_ = gtsam::rotation(gtsam::Pose3_(gtsam::symbol_shorthand::X(closestKey)));
    gtsam::Rot3_ R_fixedFrame_I_ = R_W_I_;
    // Aliases
    const std::string& sensorFrameName = unaryPositionMeasurement.sensorFrameName();                              // Alias
    const std::string& fixedFrameName = unaryPositionMeasurement.fixedFrameName();                                // Alias
    const gtsam::Point3& fixedFrame_t_fixedFrame_sensorFrame_meas = unaryPositionMeasurement.unaryMeasurement();  // Alias

    // i) Optimize over fixed frame poses --> add transformation to fixed frame
    if (fixedFrameName != worldFrame_ && graphConfigPtr_->optimizeFixedFramePosesWrtWorld) {
      Eigen::Matrix<double, 6, 1> randomWalkNoiseDensity = 0.0 * Eigen::Matrix<double, 6, 1>::Ones();  // rad/s, m/s
      gtsam::Pose3 T_fixedFrame_W_initial;
      gtsam::Matrix66 initialNoiseCovariance;
      // Lookup previous relative transformation to have good initial guess for orientation, otherwise use identity
      if (resultFixedFrameTransformations_.isFramePairInDictionary(fixedFrameName, worldFrame_)) {  // Optimized before
        T_fixedFrame_W_initial = gtsam::Pose3(resultFixedFrameTransformations_.rv_T_frame1_frame2(fixedFrameName, worldFrame_).matrix());
        initialNoiseCovariance = 10 * resultFixedFrameTransformationsCovariance_.rv_T_frame1_frame2(fixedFrameName, worldFrame_);
        // If instance is created newly --> correct for position
        if (!gtsamExpressionTransformsKeys_.isFramePairInDictionary(fixedFrameName, worldFrame_)) {
          const gtsam::Pose3& T_W_I_est = W_imuPropagatedState_.pose();  // alias for readability
          const gtsam::Point3 W_t_W_sensorFrame_est = T_W_I_est.translation() + T_W_I_est.rotation() * I_t_I_sensorFrame.value();
          const gtsam::Point3 fixedFrame_t_W_sensorFrame_est = T_fixedFrame_W_initial.rotation() * W_t_W_sensorFrame_est;
          const gtsam::Point3 fixedFrame_t_fixedFrame_W_est = fixedFrame_t_fixedFrame_sensorFrame_meas - fixedFrame_t_W_sensorFrame_est;
          // Correct
          T_fixedFrame_W_initial = gtsam::Pose3(T_fixedFrame_W_initial.rotation(), fixedFrame_t_fixedFrame_W_est);
        }
      } else {  // Never optimized before
        T_fixedFrame_W_initial = gtsam::Pose3::Identity();
        initialNoiseCovariance = gtsam::noiseModel::Diagonal::Sigmas(1.0e-02 * gtsam::Vector::Ones(6))->covariance().matrix();
        std::cout << initialNoiseCovariance << std::endl;
      }
      gtsam::Pose3_ T_fixedFrame_W_ = gtsamExpressionTransformsKeys_.getTransformationExpression<6, gtsam::Pose3>(
          valuesEstimates, priorFactors, randomWalkFactors, T_fixedFrame_W_initial, unaryPositionMeasurement.fixedFrameName(), worldFrame_,
          unaryPositionMeasurement.timeK(), initialNoiseCovariance, randomWalkNoiseDensity);
      // Transform position to fixed frame
      fixedFrame_t_fixedFrame_sensorFrame_ = gtsam::transformFrom(T_fixedFrame_W_, W_t_W_I_);  // TODO
      R_fixedFrame_I_ = gtsam::rotation(T_fixedFrame_W_) * R_W_I_;
    }

    // ii) Position not in IMU frame --> convert state to measurement sensor frame
    if (sensorFrameName != imuFrame_) {
      if (!I_t_I_sensorFrame) {
        REGULAR_COUT << RED_START << " Position measurement in " << sensorFrameName << " but no transformation to frame " << imuFrame_
                     << " provided. Not adding unary factor." << COLOR_END << std::endl;
        throw std::runtime_error("Position measurement in " + sensorFrameName + " but no transformation to frame " + imuFrame_ +
                                 " provided. Not adding unary factor.");
      }
      fixedFrame_t_fixedFrame_sensorFrame_ =
          fixedFrame_t_fixedFrame_sensorFrame_ + gtsam::rotate(R_fixedFrame_I_, I_t_I_sensorFrame.value());
    }
    // iii) optimize over extrinsics
    if (graphConfigPtr_->optimizeExtrinsicSensorToSensorCorrectedOffset && sensorFrameName != imuFrame_) {
      std::cout << "case 3" << std::endl;
      // Get delta transformation from sensorFrame to correctSensorFrame
      //      gtsam::Point3 I_t_I_sensorFrame_correctSensorFrame_initial = gtsam::Point3::Zero();
      //      gtsam::Point3_ I_t_sensorFrame_correctSensorFrame = gtsamExpressionTransformsKeys_.getTransformationExpression<gtsam::Point3,
      //      3>(
      //          valuesEstimates, priorFactors, I_t_I_sensorFrame_correctSensorFrame_initial, sensorFrameName, sensorFrameName +
      //          "_corrected", unaryPositionMeasurement.timeK());
      //      fixedFrame_t_fixedFrame_sensorFrame =
      //          fixedFrame_t_fixedFrame_sensorFrame + gtsam::rotate(R_fixedFrame_I, I_t_sensorFrame_correctSensorFrame);
    }
    // Meta function
    addUnaryExpressionFactor<gtsam::Point3, 3, gtsam::Point3_, gtsam::Pose3>(
        unaryPositionMeasurement.unaryMeasurement(), unaryPositionMeasurement.unaryMeasurementNoiseVariances(),
        fixedFrame_t_fixedFrame_sensorFrame_, unaryPositionMeasurement.timeK(), valuesEstimates, priorFactors, randomWalkFactors, false);
  } else {
    addUnaryFactorInImuFrame<gtsam::Vector3, 3, gtsam::GPSFactor, gtsam::symbol_shorthand::X>(
        unaryPositionMeasurement.unaryMeasurement(), unaryPositionMeasurement.unaryMeasurementNoiseDensity(),
        unaryPositionMeasurement.timeK());
  }
}

void GraphManager::addHeadingUnaryFactor(const double measuredYaw, const Eigen::Matrix<double, 1, 1>& gnssHeadingUnaryNoiseDensity,
                                         const double gnssTimeK) {
  addUnaryFactorInImuFrame<double, 1, HeadingFactor, gtsam::symbol_shorthand::X>(measuredYaw, gnssHeadingUnaryNoiseDensity, gnssTimeK);
}

// Between factors --------------------------------------------------------------------------------------------------------
gtsam::Key GraphManager::addPoseBetweenFactor(const gtsam::Pose3& deltaPose, const Eigen::Matrix<double, 6, 1>& poseBetweenNoiseDensity,
                                              const double lidarTimeKm1, const double lidarTimeK, const double rate) {
  // Find corresponding keys in graph
  // Find corresponding keys in graph
  double maxLidarTimestampDistance = 1.0 / rate + 2.0 * graphConfigPtr_->maxSearchDeviation;
  gtsam::Key closestKeyKm1, closestKeyK;
  double keyTimeStampDistance;

  if (!findGraphKeys_(closestKeyKm1, closestKeyK, keyTimeStampDistance, maxLidarTimestampDistance, lidarTimeKm1, lidarTimeK,
                      "pose between")) {
    REGULAR_COUT << RED_START << " Current propagated key: " << propagatedStateKey_ << " , PoseBetween factor not added between keys "
                 << closestKeyKm1 << " and " << closestKeyK << COLOR_END << std::endl;
    return closestKeyK;
  }

  // Scale delta pose according to timeStampDistance
  double scale = keyTimeStampDistance / (lidarTimeK - lidarTimeKm1);
  if (graphConfigPtr_->verboseLevel > 3) {
    REGULAR_COUT << " Scaling factor in tangent space for pose between delta pose: " << scale << std::endl;
  }
  gtsam::Pose3 scaledDeltaPose = gtsam::Pose3::Expmap(scale * gtsam::Pose3::Logmap(deltaPose));

  // Create noise model
  assert(poseBetweenNoiseDensity.size() == 6);
  auto noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(poseBetweenNoiseDensity)));  // rad,rad,rad,m,m,m
  auto errorFunction = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.345), noise);

  // Create pose between factor and add it
  gtsam::BetweenFactor<gtsam::Pose3> poseBetweenFactor(gtsam::symbol_shorthand::X(closestKeyKm1), gtsam::symbol_shorthand::X(closestKeyK),
                                                       scaledDeltaPose, errorFunction);

  // Write to graph
  addFactorSafelyToGraph_<const gtsam::BetweenFactor<gtsam::Pose3>*>(&poseBetweenFactor, lidarTimeKm1);

  // Print summary
  if (graphConfigPtr_->verboseLevel > 3) {
    REGULAR_COUT << " Current propagated key: " << propagatedStateKey_ << ", " << YELLOW_START << " PoseBetween factor added between key "
                 << closestKeyKm1 << " and key " << closestKeyK << COLOR_END << std::endl;
  }

  return closestKeyK;
}

gtsam::NavState GraphManager::calculateNavStateAtKey(bool& computeSuccessfulFlag, const std::shared_ptr<graph_msf::Optimizer> optimizerPtr,
                                                     const gtsam::Key& key, const char* callingFunctionName) {
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
    if (graphConfigPtr_->usingBiasForPreIntegrationFlag) {
      imuBufferPreintegratorPtr_->resetIntegrationAndSetBias(optimizedGraphState_.imuBias());
    } else {
      imuBufferPreintegratorPtr_->resetIntegrationAndSetBias(gtsam::imuBias::ConstantBias());
    }
  }  // end of locking

  // Graph Update (time consuming) -------------------
  bool successfulOptimizationFlag =
      addFactorsToSmootherAndOptimize(optimizerPtr_, newGraphFactors, newGraphValues, newGraphKeysTimestampsMap, graphConfigPtr_,
                                      graphConfigPtr_->additionalOptimizationIterations);
  if (!successfulOptimizationFlag) {
    REGULAR_COUT << RED_START << " Graph optimization failed. " << COLOR_END << std::endl;
    return;
  }

  // Compute entire result
  // NavState
  gtsam::NavState resultNavState = calculateNavStateAtKey(successfulOptimizationFlag, optimizerPtr_, currentPropagatedKey, __func__);
  // Bias
  gtsam::imuBias::ConstantBias resultBias = optimizerPtr_->calculateEstimatedBias(gtsam::symbol_shorthand::B(currentPropagatedKey));
  // Compute Covariance
  gtsam::Matrix66 resultPoseCovariance = optimizerPtr_->marginalCovariance(gtsam::symbol_shorthand::X(currentPropagatedKey));
  gtsam::Matrix33 resultVelocityCovariance = optimizerPtr_->marginalCovariance(gtsam::symbol_shorthand::V(currentPropagatedKey));
  // Oldest key not marginalized out
  gtsam::Key oldestKey = 0;
  gtsam::Pose3 poseAtOldestKey;
  if (currentPropagatedKey > (int(graphConfigPtr_->smootherLag * graphConfigPtr_->imuRate) + 1)) {
    double oldestGraphTime = currentPropagatedTime - graphConfigPtr_->smootherLag;
    double foundGraphTime;
    if (timeToKeyBufferPtr_->getClosestKeyAndTimestamp(foundGraphTime, oldestKey, "oldest graph state", graphConfigPtr_->maxSearchDeviation,
                                                       oldestGraphTime)) {
      oldestKey += 4;
      std::cout << "Current key: " << currentPropagatedKey << ", oldest key: " << oldestKey << std::endl;
      poseAtOldestKey = optimizerPtr_->calculateEstimatedPose(gtsam::symbol_shorthand::X(oldestKey));
      gtsam::Matrix66 poseAtOldestKeyCovariance = optimizerPtr_->marginalCovariance(gtsam::symbol_shorthand::X(oldestKey));
      std::cout << "Pose: " << poseAtOldestKey << std::endl;
      std::cout << "Covariance: " << poseAtOldestKeyCovariance << std::endl;
    }
  }

  // FixedFrame Transformations
  if (graphConfigPtr_->optimizeFixedFramePosesWrtWorld) {
    // Mutex because we are changing the dynamically allocated graphKeys
    std::lock_guard<std::mutex> modifyGraphKeysLock(gtsamExpressionTransformsKeys_.mutex());
    // Get map with all frame-pair to key correspondences
    // Iterate through all dynamically allocated transforms --------------------------------
    for (auto& framePairIterator : gtsamExpressionTransformsKeys_.getTransformsMap()) {
      // Get Transform
      const gtsam::Key& key = framePairIterator.second.key();  // alias
      // Try to optimize ---------------------------------------------------------------
      try {  // Obtain estimate and covariance from the extrinsic transformations
        gtsam::Pose3 T_frame1_frame2;
        gtsam::Matrix66 T_frame1_frame2_covariance;
        if (gtsam::Symbol(key).string()[0] == 't') {
          T_frame1_frame2 = optimizerPtr_->calculateEstimatedPose(key);
          T_frame1_frame2_covariance = optimizerPtr_->marginalCovariance(key);
        } else if (gtsam::Symbol(key).string()[0] == 'd') {
          T_frame1_frame2 = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(optimizerPtr_->calculateEstimatedDisplacement(key)));
          T_frame1_frame2_covariance.block<3, 3>(3, 3) = optimizerPtr_->marginalCovariance(key);
        } else {
          throw std::runtime_error("Key is neither a pose nor a displacement key.");
        }

        // Success --> Write to dictionary
        resultFixedFrameTransformations_.set_T_frame1_frame2(framePairIterator.first.first, framePairIterator.first.second,
                                                             Eigen::Isometry3d(T_frame1_frame2.matrix()));
        resultFixedFrameTransformationsCovariance_.set_T_frame1_frame2(framePairIterator.first.first, framePairIterator.first.second,
                                                                       T_frame1_frame2_covariance);

        // Notify that this key has now at least been optimized once
        if (framePairIterator.second.getNumberStepsOptimized() == 0) {
          REGULAR_COUT << GREEN_START << " Fixed Frame Transformation between " << framePairIterator.first.first << " and "
                       << framePairIterator.first.second << " optimized for the first time." << COLOR_END << std::endl;
        }
        // Increase Counter
        framePairIterator.second.incrementNumberStepsOptimized();

        // Check health status of transformation --> Might need to be deleted if diverged too much --> only necessary for fixed frames
        if (framePairIterator.first.second == worldFrame_ &&
            framePairIterator.second.getNumberStepsOptimized() > MIN_ITERATIONS_BEFORE_REMOVING_STATIC_TRANSFORM) {
          const gtsam::Pose3& T_frame1_frame2_initial = framePairIterator.second.getApproximateTransformationBeforeOptimization();  // alias
          double errorTangentSpace =
              gtsam::Pose3::Logmap((T_frame1_frame2_initial * resultNavState.pose()).between(T_frame1_frame2) * resultNavState.pose())
                  .norm();
          if (errorTangentSpace > graphConfigPtr_->fixedFramePosesResetThreshold) {
            REGULAR_COUT << "Error in tangent space: " << errorTangentSpace << std::endl;
            std::cout << YELLOW_START << "GMsf-GraphManager" << RED_START << " Fixed Frame Transformation between "
                      << framePairIterator.first.first << " and " << framePairIterator.first.second
                      << " diverged too much. Removing from optimization and adding again freshly at next possibility." << COLOR_END
                      << std::endl;
            // Remove state from state dictionary
            gtsamExpressionTransformsKeys_.removeTransform(framePairIterator.first.first, framePairIterator.first.second);
            break;
          }
        }
      }
      // Optimization failed -----------------------------------------
      catch (const std::out_of_range& exception) {
        if (framePairIterator.second.getNumberStepsOptimized() > 0) {  // Was optimized before, so should also be available now in the graph
                                                                       // --> as querying was unsuccessful, we remove it from the graph
          REGULAR_COUT
              << RED_START << " Out of Range exeception while querying the transformation and/or covariance at key " << gtsam::Symbol(key)
              << ", for frame pair " << framePairIterator.first.first << "," << framePairIterator.first.second << std::endl
              << " This happen if the requested variable is outside of the smoother window. Hence, we keep this estimate unchanged "
                 "and remove it from the state dictionary. To fix this, increase the smoother window."
              << COLOR_END << std::endl;
          // Remove state from state dictionary
          gtsamExpressionTransformsKeys_.removeTransform(framePairIterator.first.first, framePairIterator.first.second);
          return;
        } else {
          REGULAR_COUT << GREEN_START << " Tried to query the transformation and/or covariance for frame pair "
                       << framePairIterator.first.first << " to " << framePairIterator.first.second << ", at key: " << gtsam::Symbol(key)
                       << ". Not yet available, as it was not yet optimized. Waiting for next optimization iteration until publishing it."
                       << COLOR_END << std::endl;
        }
      }
    }
  }

  // Update block 2 ------------------
  // Lock
  const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
  // Optimized Graph State Status
  optimizedGraphState_.setIsOptimized();
  // Update Optimized Graph State
  optimizedGraphState_.updateNavStateAndBias(currentPropagatedKey, currentPropagatedTime, resultNavState,
                                             resultBias.correctGyroscope(currentAngularVelocity), resultBias);
  optimizedGraphState_.updateFixedFrameTransforms(resultFixedFrameTransformations_);
  optimizedGraphState_.updateFixedFrameTransformsCovariance(resultFixedFrameTransformationsCovariance_);
  optimizedGraphState_.updateCovariances(resultPoseCovariance, resultVelocityCovariance);
  // Predict from solution to obtain refined propagated state
  if (graphConfigPtr_->usingBiasForPreIntegrationFlag) {
    W_imuPropagatedState_ = imuBufferPreintegratorPtr_->predict(resultNavState, optimizedGraphState_.imuBias());
  } else {
    W_imuPropagatedState_ = imuBufferPreintegratorPtr_->predict(resultNavState, gtsam::imuBias::ConstantBias());
  }
  O_imuPropagatedState_ = gtsam::NavState(W_imuPropagatedState_.pose().rotation(), O_imuPropagatedState_.pose().translation(),
                                          W_imuPropagatedState_.velocity());
  T_W_O_ = Eigen::Isometry3d((W_imuPropagatedState_.pose() * O_imuPropagatedState_.pose().inverse()).matrix());

  // Increase confidence of pose at oldest key --> reduce jumping
  if (oldestKey > 0) {
    factorGraphBufferPtr_->add(gtsam::PriorFactor<gtsam::Pose3>(gtsam::symbol_shorthand::X(oldestKey), poseAtOldestKey,
                                                                gtsam::noiseModel::Diagonal::Sigmas(1.0e-02 * gtsam::Vector::Ones(6))));
  }
}

// Returns true if the factors/values were added without any problems
// Otherwise returns false (e.g. if exception occurs while adding
bool GraphManager::addFactorsToSmootherAndOptimize(std::shared_ptr<graph_msf::Optimizer> optimizerPtr,
                                                   const gtsam::NonlinearFactorGraph& newGraphFactors, const gtsam::Values& newGraphValues,
                                                   const std::map<gtsam::Key, double>& newGraphKeysTimestampsMap,
                                                   const std::shared_ptr<GraphConfig>& graphConfigPtr, const int additionalIterations) {
  // Timing
  std::chrono::time_point<std::chrono::high_resolution_clock> startLoopTime;
  std::chrono::time_point<std::chrono::high_resolution_clock> endLoopTime;
  startLoopTime = std::chrono::high_resolution_clock::now();

  // Perform update
  bool successFlag = optimizerPtr->update(newGraphFactors, newGraphValues, newGraphKeysTimestampsMap);
  // Additional iterations
  for (size_t itr = 0; itr < additionalIterations; ++itr) {
    successFlag = successFlag && optimizerPtr->update();
  }

  if (graphConfigPtr->verboseLevel > 0) {
    endLoopTime = std::chrono::high_resolution_clock::now();
    REGULAR_COUT << GREEN_START << " Whole optimization loop took "
                 << std::chrono::duration_cast<std::chrono::milliseconds>(endLoopTime - startLoopTime).count() << " milliseconds."
                 << COLOR_END << std::endl;
  }

  return successFlag;
}  // namespace graph_msf

gtsam::NavState GraphManager::calculateStateAtKey(bool& computeSuccessfulFlag, const gtsam::Key& key) {
  return calculateNavStateAtKey(computeSuccessfulFlag, optimizerPtr_, key, __func__);
}

// Private --------------------------------------------------------------------
template <class CHILDPTR>
void GraphManager::addFactorToGraph_(const gtsam::NoiseModelFactor* noiseModelFactorPtr) {
  factorGraphBufferPtr_->add(*dynamic_cast<CHILDPTR>(noiseModelFactorPtr));
}

template <class CHILDPTR>
void GraphManager::addFactorToGraph_(const gtsam::NoiseModelFactor* noiseModelFactorPtr, const double measurementTimestamp) {
  // Check Timestamp of Measurement on Delay
  if (timeToKeyBufferPtr_->getLatestTimestampInBuffer() - measurementTimestamp >
      graphConfigPtr_->smootherLag - WORST_CASE_OPTIMIZATION_TIME) {
    REGULAR_COUT << RED_START
                 << " Measurement Delay is larger than the smootherLag - WORST_CASE_OPTIMIZATION_TIME, hence skipping this measurement."
                 << COLOR_END << std::endl;
  }
  // Add measurements
  return addFactorToGraph_<CHILDPTR>(noiseModelFactorPtr);
}

template <class CHILDPTR>
void GraphManager::addFactorSafelyToGraph_(const gtsam::NoiseModelFactor* noiseModelFactorPtr, const double measurementTimestamp) {
  // Operating on graph data --> acquire mutex
  const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
  // Add measurements
  addFactorToGraph_<CHILDPTR>(noiseModelFactorPtr, measurementTimestamp);
}

bool GraphManager::findGraphKeys_(gtsam::Key& closestKeyKm1, gtsam::Key& closestKeyK, double& keyTimeStampDistance,
                                  const double maxTimestampDistance, const double timeKm1, const double timeK, const std::string& name) {
  // Find closest lidar keys in existing graph
  double closestGraphTimeKm1, closestGraphTimeK;
  {
    // Looking up from IMU buffer --> acquire mutex (otherwise values for key
    // might not be set)
    const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
    bool success = timeToKeyBufferPtr_->getClosestKeyAndTimestamp(closestGraphTimeKm1, closestKeyKm1, name + " km1",
                                                                  graphConfigPtr_->maxSearchDeviation, timeKm1);
    success = success && timeToKeyBufferPtr_->getClosestKeyAndTimestamp(closestGraphTimeK, closestKeyK, name + " k",
                                                                        graphConfigPtr_->maxSearchDeviation, timeK);
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

void GraphManager::writeKeyToKeyTimeStampMap_(const gtsam::Key& key, const double measurementTime,
                                              std::shared_ptr<std::map<gtsam::Key, double>> keyTimestampMapPtr) {
  (*keyTimestampMapPtr)[key] = measurementTime;
}

void GraphManager::writeValueKeysToKeyTimeStampMap_(const gtsam::Values& values, const double measurementTime,
                                                    std::shared_ptr<std::map<gtsam::Key, double>> keyTimestampMapPtr) {
  for (const auto& value : values) {
    writeKeyToKeyTimeStampMap_(value.key, measurementTime, keyTimestampMapPtr);
  }
}

void GraphManager::updateImuIntegrators_(const TimeToImuMap& imuMeas) {
  if (imuMeas.size() < 2) {
    REGULAR_COUT << " Received less than 2 IMU messages --- No Preintegration done." << std::endl;
    return;
  } else if (imuMeas.size() > 2) {
    REGULAR_COUT << RED_START << "Currently only supporting two IMU messages for pre-integration." << COLOR_END << std::endl;
    throw std::runtime_error("Terminating.");
  }

  // Reset IMU Step Preintegration
  if (graphConfigPtr_->usingBiasForPreIntegrationFlag) {
    imuStepPreintegratorPtr_->resetIntegrationAndSetBias(optimizedGraphState_.imuBias());
  } else {
    imuStepPreintegratorPtr_->resetIntegrationAndSetBias(gtsam::imuBias::ConstantBias());
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

}  // namespace graph_msf