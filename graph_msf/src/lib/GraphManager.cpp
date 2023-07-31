/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#define WORST_CASE_OPTIMIZATION_TIME 0.1  // in seconds

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

GraphManager::GraphManager(std::shared_ptr<GraphConfig> graphConfigPtr, const std::string& worldFrame)
    : graphConfigPtr_(graphConfigPtr),
      worldFrame_(worldFrame),
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
    std::cout << YELLOW_START << "GMsf-GraphManager" << RED_START << " OptimizerLM is not implemented yet." << COLOR_END << std::endl;
    std::runtime_error("OptimizerLM is not implemented yet.");
  }
}

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
  std::cout << YELLOW_START << "GMsf-GraphManager" << COLOR_END << " Initial Pose: " << initialPose << std::endl;
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

gtsam::Key GraphManager::addPoseBetweenFactor(const double lidarTimeKm1, const double lidarTimeK, const double rate,
                                              const Eigen::Matrix<double, 6, 1>& poseBetweenNoiseDensity, const gtsam::Pose3& deltaPose,
                                              const std::string& measurementType) {
  // Find corresponding keys in graph
  // Find corresponding keys in graph
  double maxLidarTimestampDistance = 1.0 / rate + 2.0 * graphConfigPtr_->maxSearchDeviation;
  gtsam::Key closestKeyKm1, closestKeyK;
  double keyTimeStampDistance;

  if (!findGraphKeys_(closestKeyKm1, closestKeyK, keyTimeStampDistance, maxLidarTimestampDistance, lidarTimeKm1, lidarTimeK,
                      measurementType)) {
    std::cerr << YELLOW_START << "GMsf-GraphManager" << RED_START << " Current propagated key: " << propagatedStateKey_
              << " , PoseBetween factor not added between keys " << closestKeyKm1 << " and " << closestKeyK << COLOR_END << std::endl;
    return closestKeyK;
  }

  // Scale delta pose according to timeStampDistance
  double scale = keyTimeStampDistance / (lidarTimeK - lidarTimeKm1);
  if (graphConfigPtr_->verboseLevel > 3) {
    std::cout << YELLOW_START << "GMsf-GraphManager" << COLOR_END << " Scale for " << measurementType << " delta pose: " << scale
              << std::endl;
  }
  gtsam::Pose3 scaledDeltaPose = gtsam::Pose3::Expmap(scale * gtsam::Pose3::Logmap(deltaPose));

  // Create noise model
  assert(poseBetweenNoiseDensity.size() == 6);
  auto noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(poseBetweenNoiseDensity)));  // rad,rad,rad,m,m,m
  // Regular error function with noise
  // auto errorFunction = noise;
  // Robust error function
  auto errorFunction = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.3), noise);

  // Create pose between factor and add it
  gtsam::BetweenFactor<gtsam::Pose3> poseBetweenFactor(gtsam::symbol_shorthand::X(closestKeyKm1), gtsam::symbol_shorthand::X(closestKeyK),
                                                       scaledDeltaPose, errorFunction);

  // Write to graph
  addFactorSafelyToGraph_<const gtsam::BetweenFactor<gtsam::Pose3>*>(&poseBetweenFactor, lidarTimeKm1);

  // Print summary
  if (graphConfigPtr_->verboseLevel > 3) {
    std::cout << YELLOW_START << "GMsf-GraphManager" << COLOR_END << " Current propagated key: " << propagatedStateKey_ << ", "
              << YELLOW_START << measurementType << " PoseBetween factor added between key " << closestKeyKm1 << " and key " << closestKeyK
              << COLOR_END << std::endl;
  }

  return closestKeyK;
}

void GraphManager::addPoseUnaryFactor(const UnaryMeasurementXD<Eigen::Isometry3d, 6>& unary6DMeasurement,
                                      const Eigen::Isometry3d& T_sensorFrame_imu) {
  // Find closest key in existing graph
  double closestGraphTime;
  gtsam::Key closestKey;
  if (!timeToKeyBufferPtr_->getClosestKeyAndTimestamp(closestGraphTime, closestKey, unary6DMeasurement.measurementName(),
                                                      graphConfigPtr_->maxSearchDeviation, unary6DMeasurement.timeK())) {
    std::cerr << YELLOW_START << "GMsf-GraphManager" << RED_START << " Not adding " << unary6DMeasurement.measurementName()
              << " constraint to graph." << COLOR_END << std::endl;
    return;
  }

  // Measurement handling
  gtsam::Pose3 T_fixedFrame_imu(unary6DMeasurement.unaryMeasurement().matrix() * T_sensorFrame_imu.matrix());

  // Noise Set up
  const gtsam::Key statePoseKey = gtsam::symbol_shorthand::X(closestKey);
  auto noiseModel =
      gtsam::noiseModel::Diagonal::Variances(gtsam::Vector(unary6DMeasurement.unaryMeasurementNoiseVariances()));  // rad,rad,rad,x,y,z
  auto errorFunction = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.5), noiseModel);

  // Also optimize over fixedFrame Transformation
  // --------------------------------
  if (graphConfigPtr_->optimizeFixedFramePosesWrtWorld) {
    if (gtsamExpressionTransformsKeys_.newFramePairSafelyAddedToDictionary(unary6DMeasurement.fixedFrameName(), worldFrame_,
                                                                           unary6DMeasurement.timeK())) {  // Newly added to dictionary
      gtsam::Key T_M_W_key = gtsamExpressionTransformsKeys_.rv_T_frame1_frame2(unary6DMeasurement.fixedFrameName(), worldFrame_);

      // Values for T_M_W
      gtsam::Values valuesEstimate;
      gtsam::Pose3 T_W_I = W_imuPropagatedState_.pose();
      gtsam::Pose3 T_M_W_approx = T_fixedFrame_imu * T_W_I.inverse();
      valuesEstimate.insert(T_M_W_key, T_M_W_approx);

      // Expression Factor (copy from below)
      gtsam::Pose3_ T_M_W_(T_M_W_key);
      //    gtsam::Pose3_ T_M_W(gtsam::symbol_shorthand::T(0));
      gtsam::Pose3_ X_(statePoseKey);
      gtsam::ExpressionFactor<gtsam::Pose3> poseUnaryExpressionFactor(errorFunction, T_fixedFrame_imu, gtsam::Pose3_(T_M_W_ * X_));

      // Operating on graph data
      const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
      graphValuesBufferPtr_->insert(valuesEstimate);
      addFactorToGraph_<const gtsam::ExpressionFactor<gtsam::Pose3>*>(&poseUnaryExpressionFactor, unary6DMeasurement.timeK());
      writeValueKeysToKeyTimeStampMap_(valuesEstimate, unary6DMeasurement.timeK(), graphKeysTimestampsMapBufferPtr_);
      gtsam::Symbol keySymbol = gtsam::Symbol(T_M_W_key);
      std::cout << YELLOW_START << "GMsf-GraphManager" << GREEN_START << " Initialized new " << unary6DMeasurement.fixedFrameName()
                << " to " << worldFrame_ << " variable: " << keySymbol.string() << COLOR_END << std::endl;
      // TODO: Cover case that this transformation has been added before --> use covariance from before as prior
      //      if (resultFixedFrameTransformationsCovariance_.isFramePairInDictionary(unary6DMeasurement.fixedFrameName(), worldFrame_)) {
      //        std::cout << YELLOW_START << "GMsf-GraphManager" << COLOR_END
      //                  << " Initialization of this newly added transformation with covariance (from before): "
      //                  << resultFixedFrameTransformationsCovariance_.rv_T_frame1_frame2(unary6DMeasurement.fixedFrameName(), worldFrame_)
      //                  << std::endl;
      //
      //      }
    } else {  // Already in the transformation dictionary
      // Expression key
      gtsam::Key T_M_W_key = gtsamExpressionTransformsKeys_.rv_T_frame1_frame2(unary6DMeasurement.fixedFrameName(), worldFrame_);
      // Expression Factor
      gtsam::Pose3_ T_M_W_(T_M_W_key);
      //    gtsam::Pose3_ T_M_W(gtsam::symbol_shorthand::T(0));
      gtsam::Pose3_ X_(statePoseKey);
      gtsam::ExpressionFactor<gtsam::Pose3> poseUnaryExpressionFactor(errorFunction, T_fixedFrame_imu, gtsam::Pose3_(T_M_W_ * X_));
      // Write to graph
      const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
      addFactorToGraph_<const gtsam::ExpressionFactor<gtsam::Pose3>*>(&poseUnaryExpressionFactor, unary6DMeasurement.timeK());
      writeKeyToKeyTimeStampMap_(T_M_W_key, unary6DMeasurement.timeK(), graphKeysTimestampsMapBufferPtr_);
    }
  } else {  // Simple unary factor --------------------------------------
    // Regular Unary factor
    gtsam::PriorFactor<gtsam::Pose3> poseUnaryFactor(statePoseKey, T_fixedFrame_imu, errorFunction);
    // Write to graph
    addFactorSafelyToGraph_<const gtsam::PriorFactor<gtsam::Pose3>*>(&poseUnaryFactor, unary6DMeasurement.timeK());
  }

  // Print summary --------------------------------------
  if (graphConfigPtr_->verboseLevel > 0) {
    std::cout << YELLOW_START << "GMsf-GraphManager" << COLOR_END << " Current propagated key " << propagatedStateKey_ << GREEN_START
              << unary6DMeasurement.measurementName() << " factor added to key " << closestKey << COLOR_END << std::endl;
  }
}

void GraphManager::addVelocityUnaryFactor(const double timeK, const double rate,
                                          const Eigen::Matrix<double, 3, 1>& velocityUnaryNoiseDensity, const gtsam::Vector3& velocity,
                                          const std::string& measurementType) {
  // Find closest key in existing graph
  double closestGraphTime;
  gtsam::Key closestKey;

  if (!timeToKeyBufferPtr_->getClosestKeyAndTimestamp(closestGraphTime, closestKey, measurementType, graphConfigPtr_->maxSearchDeviation,
                                                      timeK)) {
    std::cerr << YELLOW_START << "GMsf-GraphManager" << RED_START << " Not adding " << measurementType << " constraint to graph."
              << COLOR_END << std::endl;
    return;
  }

  assert(velocityUnaryNoiseDensity.size() == 3);
  auto noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector(velocityUnaryNoiseDensity));  // rad,rad,rad,x,y,z
  auto errorFunction = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.5), noise);

  gtsam::PriorFactor<gtsam::Vector3> velocityUnaryFactor(gtsam::symbol_shorthand::V(closestKey), gtsam::Vector3::Zero(), errorFunction);

  /// Add Zero Velocity Factor
  addFactorSafelyToGraph_<const gtsam::PriorFactor<gtsam::Vector3>*>(&velocityUnaryFactor, timeK);

  // Print summary
  if (graphConfigPtr_->verboseLevel > 0) {
    std::cout << YELLOW_START << "GMsf-GraphManager" << COLOR_END << " Current propagated key " << propagatedStateKey_ << GREEN_START
              << measurementType << " factor added to key " << closestKey << COLOR_END << std::endl;
  }
}

void GraphManager::addGnssPositionUnaryFactor(double gnssTimeK, const double rate, const Eigen::Vector3d& gnssPositionUnaryNoiseDensity,
                                              const gtsam::Vector3& position) {
  // Find closest key in existing graph
  double closestGraphTime;
  gtsam::Key closestKey;
  if (!timeToKeyBufferPtr_->getClosestKeyAndTimestamp(closestGraphTime, closestKey, "GnssUnary", graphConfigPtr_->maxSearchDeviation,
                                                      gnssTimeK)) {
    // TODO
  }

  // Create noise model
  auto noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(gnssPositionUnaryNoiseDensity)));  // m,m,m
  auto tukeyErrorFunction = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(0.7), noise);

  // Create unary factor and add it
  gtsam::GPSFactor gnssPositionUnaryFactor(gtsam::symbol_shorthand::X(closestKey), position, tukeyErrorFunction);

  // Write to graph
  addFactorSafelyToGraph_<const gtsam::GPSFactor*>(&gnssPositionUnaryFactor, gnssTimeK);

  // Print summary
  if (graphConfigPtr_->verboseLevel > 1) {
    std::cout << YELLOW_START << "GMsf-GraphManager" << COLOR_END << " Current propagated key " << propagatedStateKey_ << GREEN_START
              << ", Gnss position factor added to key " << closestKey << COLOR_END << std::endl;
  }
}

void GraphManager::addGnssHeadingUnaryFactor(double gnssTimeK, const double rate,
                                             const Eigen::Matrix<double, 1, 1>& gnssHeadingUnaryNoiseDensity, const double measuredYaw) {
  // Print information
  if (graphConfigPtr_->verboseLevel > 2) {
    std::cout << YELLOW_START << "GMsf-GraphManager" << COLOR_END << " Current propagated key " << propagatedStateKey_
              << std::setprecision(14) << ", Gnss yaw measurement at time stamp " << gnssTimeK << " is: " << measuredYaw << std::endl;
  }

  // Find closest key in existing graph
  double closestGraphTime;
  gtsam::Key closestKey;
  if (!timeToKeyBufferPtr_->getClosestKeyAndTimestamp(closestGraphTime, closestKey, "GnssHeading", graphConfigPtr_->maxSearchDeviation,
                                                      gnssTimeK)) {
    // TODO
  }

  auto noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector(gnssHeadingUnaryNoiseDensity));  // rad,rad,rad,m,m,m
  auto tukeyErrorFunction = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(0.5), noise);

  HeadingFactor gnssHeadingUnaryFactor(gtsam::symbol_shorthand::X(closestKey), measuredYaw, tukeyErrorFunction);

  // Write to graph
  addFactorSafelyToGraph_<const HeadingFactor*>(&gnssHeadingUnaryFactor, gnssTimeK);

  // Print summary
  if (graphConfigPtr_->verboseLevel > 0) {
    std::cout << YELLOW_START << "GMsf-GraphManager" << COLOR_END << " Current propagated key " << propagatedStateKey_ << GREEN_START
              << " Gnss heading factor is added to key " << closestKey << COLOR_END << std::endl;
  }
}

gtsam::NavState GraphManager::calculateNavStateAtKey(bool& computeSuccessfulFlag, const std::shared_ptr<graph_msf::Optimizer> optimizerPtr,
                                                     const std::shared_ptr<GraphConfig>& graphConfigPtr, const gtsam::Key& key,
                                                     const char* callingFunctionName) {
  gtsam::Pose3 resultPose;
  gtsam::Vector3 resultVelocity;
  if (true) {
    try {
      resultPose = optimizerPtr->calculateEstimatedPose(gtsam::symbol_shorthand::X(key));  // auto result = mainGraphPtr_->estimate();
      resultVelocity = optimizerPtr->calculateEstimatedVelocity(gtsam::symbol_shorthand::V(key));
      computeSuccessfulFlag = true;
    } catch (const std::out_of_range& outOfRangeExeception) {
      std::cerr << "Out of Range exeception while optimizing graph: " << outOfRangeExeception.what() << '\n';
      std::cout << YELLOW_START << "GMsf-GraphManager" << RED_START
                << " This happens if the measurement delay is larger than the graph-smootherLag, i.e. the optimized graph instances are "
                   "not connected. Increase the lag in this case."
                << COLOR_END << std::endl;
      std::cout << YELLOW_START << "GMsf-GraphManager" << RED_START << "CalculateNavStateAtKey called by " << callingFunctionName
                << COLOR_END << std::endl;
      computeSuccessfulFlag = false;
    }
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
  addFactorsToSmootherAndOptimize(optimizerPtr_, newGraphFactors, newGraphValues, newGraphKeysTimestampsMap, graphConfigPtr_,
                                  graphConfigPtr_->additionalOptimizationIterations);

  // Compute entire result
  // NavState
  bool computeSuccessfulFlag = true;
  gtsam::NavState resultNavState =
      calculateNavStateAtKey(computeSuccessfulFlag, optimizerPtr_, graphConfigPtr_, currentPropagatedKey, __func__);
  // Bias
  gtsam::imuBias::ConstantBias resultBias = optimizerPtr_->calculateEstimatedBias(gtsam::symbol_shorthand::B(currentPropagatedKey));
  // Compute Covariance
  gtsam::Matrix66 resultPoseCovariance = optimizerPtr_->marginalCovariance(gtsam::symbol_shorthand::X(currentPropagatedKey));
  gtsam::Matrix33 resultVelocityCovariance = optimizerPtr_->marginalCovariance(gtsam::symbol_shorthand::V(currentPropagatedKey));

  // FixedFrame Transformations
  if (graphConfigPtr_->optimizeFixedFramePosesWrtWorld) {
    // Get map with all frame-pair to key correspondences
    for (const auto& framePairIterator : gtsamExpressionTransformsKeys_.getTransformsMap()) {
      // Get Transform
      const gtsam::Key& key = framePairIterator.second;
      try {
        gtsam::Pose3 T_frame1_frame2 = optimizerPtr_->calculateEstimatedPose(key);
        gtsam::Matrix66 T_frame1_frame2_covariance = optimizerPtr_->marginalCovariance(key);
        // Write to dictionary
        resultFixedFrameTransformations_.set_T_frame1_frame2(framePairIterator.first.first, framePairIterator.first.second,
                                                             Eigen::Isometry3d(T_frame1_frame2.matrix()), currentPropagatedTime);
        resultFixedFrameTransformationsCovariance_.set_T_frame1_frame2(framePairIterator.first.first, framePairIterator.first.second,
                                                                       T_frame1_frame2_covariance, currentPropagatedTime);
      } catch (const std::out_of_range& exception) {
        if ((currentPropagatedTime - gtsamExpressionTransformsKeys_.getLatestTransformTimestamp(
                                         framePairIterator.first.first, framePairIterator.first.second)) > graphConfigPtr_->smootherLag) {
          std::cout << YELLOW_START << "GMsf-GraphManager" << RED_START
                    << " Out of Range exeception while querying the transformation and/or covariance for frame pair "
                    << framePairIterator.first.first << "," << framePairIterator.first.second << ", at key : " << gtsam::Symbol(key) << "."
                    << std::endl
                    << " This can happen if the smoother window is too small. Hence, we keep this estimate unchanged." << COLOR_END
                    << std::endl;
          // Remove state from state dictionary
          gtsamExpressionTransformsKeys_.removeTransform(framePairIterator.first.first, framePairIterator.first.second);
        }
      }
    }
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
  }  // end of locking
}

void GraphManager::addFactorsToSmootherAndOptimize(std::shared_ptr<graph_msf::Optimizer> optimizerPtr,
                                                   const gtsam::NonlinearFactorGraph& newGraphFactors, const gtsam::Values& newGraphValues,
                                                   const std::map<gtsam::Key, double>& newGraphKeysTimestampsMap,
                                                   const std::shared_ptr<GraphConfig>& graphConfigPtr, const int additionalIterations) {
  // Timing
  std::chrono::time_point<std::chrono::high_resolution_clock> startLoopTime;
  std::chrono::time_point<std::chrono::high_resolution_clock> endLoopTime;
  startLoopTime = std::chrono::high_resolution_clock::now();

  // Perform update
  optimizerPtr->update(newGraphFactors, newGraphValues, newGraphKeysTimestampsMap);
  // Additional iterations
  for (size_t itr = 0; itr < additionalIterations; ++itr) {
    optimizerPtr->update();
  }

  if (graphConfigPtr->verboseLevel > 0) {
    endLoopTime = std::chrono::high_resolution_clock::now();
    std::cout << YELLOW_START << "GMsf-GraphManager" << GREEN_START << " Whole optimization loop took "
              << std::chrono::duration_cast<std::chrono::milliseconds>(endLoopTime - startLoopTime).count() << " milliseconds." << COLOR_END
              << std::endl;
  }
}  // namespace graph_msf

gtsam::NavState GraphManager::calculateStateAtKey(bool& computeSuccessfulFlag, const gtsam::Key& key) {
  return calculateNavStateAtKey(computeSuccessfulFlag, optimizerPtr_, graphConfigPtr_, key, __func__);
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
    std::cout << YELLOW_START << "GMsf-GraphManager" << RED_START
              << " Measurement Delay is larger than the smootherLag - "
                 "WORST_CASE_OPTIMIZATION_TIME, hence skipping this measurement."
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
      std::cerr << YELLOW_START << "GMsf-GraphManager" << RED_START << " Could not find closest keys for " << name << COLOR_END
                << std::endl;
      return false;
    }
  }

  // Check
  if (closestGraphTimeKm1 > closestGraphTimeK) {
    std::cerr << YELLOW_START << "GMsf-GraphManager" << RED_START << " Time at time step k-1 must be smaller than time at time step k."
              << COLOR_END << std::endl;
    return false;
  }

  keyTimeStampDistance = std::abs(closestGraphTimeK - closestGraphTimeKm1);
  if (keyTimeStampDistance > maxTimestampDistance) {
    std::cerr << YELLOW_START << "GMsf-GraphManager"
              << " Distance of " << name << " timestamps is too big. Found timestamp difference is  "
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
    std::cerr << YELLOW_START << "GMsf-GraphManager" << COLOR_END << " Received less than 2 IMU messages --- No Preintegration done."
              << std::endl;
    return;
  } else if (imuMeas.size() > 2) {
    std::cerr << YELLOW_START << "GMsf-GraphManager" << RED_START << "Currently only supporting two IMU messages for pre-integration."
              << COLOR_END << std::endl;
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