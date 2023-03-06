/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#include "graph_msf/core/GraphManager.hpp"

#define WORST_CASE_OPTIMIZATION_TIME 0.1  // in seconds

namespace graph_msf {

// Public --------------------------------------------------------------------

GraphManager::GraphManager(std::shared_ptr<GraphConfig> graphConfigPtr) : graphConfigPtr_(graphConfigPtr) {
  imuBuffer_.setImuRate(graphConfigPtr_->imuRate);
  imuBuffer_.setImuBufferLength(graphConfigPtr_->imuBufferLength);
  imuBuffer_.setVerboseLevel(graphConfigPtr_->verboseLevel);

  // Buffer
  globalFactorsBufferPtr_ = std::make_shared<gtsam::NonlinearFactorGraph>();
  fallbackFactorsBufferPtr_ = std::make_shared<gtsam::NonlinearFactorGraph>();
  globalGraphValuesBufferPtr_ = std::make_shared<gtsam::Values>();
  fallbackGraphValuesBufferPtr_ = std::make_shared<gtsam::Values>();
  globalGraphKeysTimestampsMapBufferPtr_ = std::make_shared<std::map<gtsam::Key, double>>();
  fallbackGraphKeysTimestampsMapBufferPtr_ = std::make_shared<std::map<gtsam::Key, double>>();
}

bool GraphManager::initImuIntegrators(const double g, const std::string& imuGravityDirection) {
  // Gravity direction definition
  if (imuGravityDirection == "up") {
    imuParamsPtr_ = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(g);  // ROS convention
  } else if (imuGravityDirection == "down") {
    imuParamsPtr_ = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedD(g);
  } else {
    throw std::runtime_error("Gravity direction must be either 'up' or 'down'.");
  }
  // Set noise and bias parameters
  /// Position
  imuParamsPtr_->setAccelerometerCovariance(gtsam::Matrix33::Identity(3, 3) * graphConfigPtr_->accNoiseDensity);
  imuParamsPtr_->setIntegrationCovariance(
      gtsam::Matrix33::Identity(3, 3) *
      graphConfigPtr_->integrationNoiseDensity);  // error committed in integrating position from velocities
  imuParamsPtr_->setUse2ndOrderCoriolis(graphConfigPtr_->use2ndOrderCoriolisFlag);
  /// Rotation
  imuParamsPtr_->setGyroscopeCovariance(gtsam::Matrix33::Identity(3, 3) * graphConfigPtr_->gyroNoiseDensity);
  imuParamsPtr_->setOmegaCoriolis(gtsam::Vector3(1, 1, 1) * graphConfigPtr_->omegaCoriolis);
  /// Bias
  imuParamsPtr_->setBiasAccCovariance(gtsam::Matrix33::Identity(3, 3) * graphConfigPtr_->accBiasRandomWalk);
  imuParamsPtr_->setBiasOmegaCovariance(gtsam::Matrix33::Identity(3, 3) * graphConfigPtr_->gyroBiasRandomWalk);
  imuParamsPtr_->setBiasAccOmegaInit(gtsam::Matrix66::Identity(6, 6) *
                                     graphConfigPtr_->biasAccOmegaInit);  // covariance of bias used for preintegration

  // Use previously defined prior for gyro
  imuBiasPriorPtr_ = std::make_shared<gtsam::imuBias::ConstantBias>(graphConfigPtr_->accBiasPrior, graphConfigPtr_->gyroBiasPrior);

  // Init Pre-integrators
  globalImuBufferPreintegratorPtr_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(imuParamsPtr_, *imuBiasPriorPtr_);
  fallbackImuBufferPreintegratorPtr_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(imuParamsPtr_, *imuBiasPriorPtr_);
  imuStepPreintegratorPtr_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(imuParamsPtr_, *imuBiasPriorPtr_);
  imuParamsPtr_->print("IMU Preintegration Parameters:");
  return true;
}

bool GraphManager::initPoseVelocityBiasGraph(const double timeStep, const gtsam::Pose3& initialPose) {
  // Configs ------------------------------------------------------------------
  // Set graph relinearization thresholds - must be lower case letters, check:gtsam::symbol_shorthand
  gtsam::FastMap<char, gtsam::Vector> relinTh;
  relinTh['x'] = (gtsam::Vector(6) << graphConfigPtr_->rotationReLinTh, graphConfigPtr_->rotationReLinTh, graphConfigPtr_->rotationReLinTh,
                  graphConfigPtr_->positionReLinTh, graphConfigPtr_->positionReLinTh, graphConfigPtr_->positionReLinTh)
                     .finished();
  relinTh['v'] =
      (gtsam::Vector(3) << graphConfigPtr_->velocityReLinTh, graphConfigPtr_->velocityReLinTh, graphConfigPtr_->velocityReLinTh).finished();
  relinTh['b'] = (gtsam::Vector(6) << graphConfigPtr_->accBiasReLinTh, graphConfigPtr_->accBiasReLinTh, graphConfigPtr_->accBiasReLinTh,
                  graphConfigPtr_->gyroBiasReLinTh, graphConfigPtr_->gyroBiasReLinTh, graphConfigPtr_->gyroBiasReLinTh)
                     .finished();
  isamParams_.relinearizeThreshold = relinTh;
  // Factorization
  if (graphConfigPtr_->usingCholeskyFactorizationFlag) {
    isamParams_.factorization = gtsam::ISAM2Params::CHOLESKY;  // CHOLESKY:Fast but non-stable
  } else {
    isamParams_.factorization = gtsam::ISAM2Params::QR;  // QR:Slower but more stable im poorly conditioned problems
  }
  // Set graph relinearization skip
  isamParams_.relinearizeSkip = graphConfigPtr_->relinearizeSkip;
  // Set relinearization
  isamParams_.enableRelinearization = graphConfigPtr_->enableRelinearizationFlag;
  // Enable Nonlinear Error
  isamParams_.evaluateNonlinearError = graphConfigPtr_->evaluateNonlinearErrorFlag;
  // Cache linearized factors
  isamParams_.cacheLinearizedFactors = graphConfigPtr_->cacheLinearizedFactorsFlag;
  // Enable particular relinearization check
  isamParams_.enablePartialRelinearizationCheck = graphConfigPtr_->enablePartialRelinearizationCheckFlag;

  // Initialize Smoothers -----------------------------------------------
  // Global Graph
  globalSmootherPtr_ = std::make_shared<gtsam::IncrementalFixedLagSmoother>(graphConfigPtr_->smootherLag,
                                                                            isamParams_);  // std::make_shared<gtsam::ISAM2>(isamParams_);
  globalSmootherPtr_->params().print("Factor Graph Parameters of global graph.");
  // Fallback Graph
  fallbackSmootherPtr_ = std::make_shared<gtsam::IncrementalFixedLagSmoother>(graphConfigPtr_->smootherLag,
                                                                              isamParams_);  // std::make_shared<gtsam::ISAM2>(isamParams_);

  // Create Prior factor ----------------------------------------------------
  /// Prior factor noise
  auto priorPoseNoise =
      gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-6, 1e-6, 1e-6).finished());  // rad,rad,rad,m, m, m
  auto priorVelocityNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e-3);                                        // m/s
  auto priorBiasNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3).finished());

  // Looking up from IMU buffer --> acquire mutex (otherwise values for key might not be set)
  const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);

  // Initial estimate
  gtsam::Values valuesEstimate;
  std::cout << YELLOW_START << "GraphManager" << COLOR_END << " Initial Pose: " << initialPose << std::endl;
  valuesEstimate.insert(gtsam::symbol_shorthand::X(propagatedStateKey_), initialPose);
  valuesEstimate.insert(gtsam::symbol_shorthand::V(propagatedStateKey_), gtsam::Vector3(0, 0, 0));
  valuesEstimate.insert(gtsam::symbol_shorthand::B(propagatedStateKey_), *imuBiasPriorPtr_);
  /// Timestamp mapping for incremental fixed lag smoother
  std::shared_ptr<std::map<gtsam::Key, double>> priorKeyTimestampMapPtr = std::make_shared<std::map<gtsam::Key, double>>();
  writeValueKeysToKeyTimeStampMap_(valuesEstimate, timeStep, priorKeyTimestampMapPtr);

  // Initialize both graphs -------------------------------------------------
  /// Global Factors
  globalFactorsBufferPtr_->resize(0);
  globalFactorsBufferPtr_->emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      gtsam::symbol_shorthand::X(propagatedStateKey_), initialPose,
      priorPoseNoise);  // POSE - PriorFactor format is (key,value,matrix) value is same type as type of PriorFactor
  globalFactorsBufferPtr_->emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(gtsam::symbol_shorthand::V(propagatedStateKey_),
                                                                              gtsam::Vector3(0, 0, 0),
                                                                              priorVelocityNoise);  // VELOCITY
  globalFactorsBufferPtr_->emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(gtsam::symbol_shorthand::B(propagatedStateKey_),
                                                                                            *imuBiasPriorPtr_,
                                                                                            priorBiasNoise);  // BIAS

  // Fallback Factors
  *fallbackFactorsBufferPtr_ = *globalFactorsBufferPtr_;

  /// Add prior factor to graph and optimize for the first time ----------------
  if (graphConfigPtr_->useIsamFlag) {
    addFactorsToSmootherAndOptimize(globalSmootherPtr_, *globalFactorsBufferPtr_, valuesEstimate, *priorKeyTimestampMapPtr, graphConfigPtr_,
                                    0);
  } else {
    throw std::runtime_error("Optimizer has to be ISAM at this stage.");
  }

  /// Add prior factor to graph and update
  if (graphConfigPtr_->useIsamFlag) {
    addFactorsToSmootherAndOptimize(fallbackSmootherPtr_, *fallbackFactorsBufferPtr_, valuesEstimate, *priorKeyTimestampMapPtr,
                                    graphConfigPtr_, 0);
  }

  globalFactorsBufferPtr_->resize(0);
  fallbackFactorsBufferPtr_->resize(0);

  // Wrap up ----------------------------------------------------------------
  // Set active graph to global graph in the beginning
  activeSmootherPtr_ = globalSmootherPtr_;
  activeFactorsBufferPtr_ = globalFactorsBufferPtr_;
  activeImuBufferPreintegratorPtr_ = globalImuBufferPreintegratorPtr_;
  activeGraphValuesBufferPtr_ = globalGraphValuesBufferPtr_;
  activeGraphKeysTimestampsMapBufferPtr_ = globalGraphKeysTimestampsMapBufferPtr_;

  // Update Current State ---------------------------------------------------
  optimizedGraphState_.updateNavStateAndBias(propagatedStateKey_, timeStep, gtsam::NavState(initialPose, gtsam::Vector3(0, 0, 0)),
                                             *imuBiasPriorPtr_);
  imuPropagatedState_ = gtsam::NavState(initialPose, gtsam::Vector3(0, 0, 0));
  return true;
}

gtsam::NavState GraphManager::addImuFactorAndGetState(const double imuTimeK, const Eigen::Vector3d& linearAcc,
                                                      const Eigen::Vector3d& angularVel, bool& relocalizationFlag) {
  // Keys
  gtsam::Key oldKey;
  gtsam::Key newKey;

  // Looking up from IMU buffer --> acquire mutex (otherwise values for key might not be set)
  const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);

  // Write current time
  propagatedStateTime_ = imuTimeK;

  // Get new key
  oldKey = propagatedStateKey_;
  newKey = newPropagatedStateKey_();

  // Add to key buffer
  imuBuffer_.addToKeyBuffer(imuTimeK, newKey);

  // Get last two measurements from buffer to determine dt
  TimeToImuMap imuMeas;
  imuBuffer_.getLastTwoMeasurements(imuMeas);

  // Update IMU preintegrator
  updateImuIntegrators_(imuMeas);
  // Predict propagated state
  if (graphConfigPtr_->usingBiasForPreIntegrationFlag) {
    imuPropagatedState_ = imuStepPreintegratorPtr_->predict(imuPropagatedState_, optimizedGraphState_.imuBias());
  } else {
    imuPropagatedState_ = imuStepPreintegratorPtr_->predict(imuPropagatedState_, gtsam::imuBias::ConstantBias());
  }

  // Add IMU Factor to graph
  gtsam::CombinedImuFactor imuFactor(gtsam::symbol_shorthand::X(oldKey), gtsam::symbol_shorthand::V(oldKey),
                                     gtsam::symbol_shorthand::X(newKey), gtsam::symbol_shorthand::V(newKey),
                                     gtsam::symbol_shorthand::B(oldKey), gtsam::symbol_shorthand::B(newKey), *imuStepPreintegratorPtr_);
  bool success = addFactorToGraph_<const gtsam::CombinedImuFactor*>(globalFactorsBufferPtr_, &imuFactor, imuTimeK);
  success = success && addFactorToGraph_<const gtsam::CombinedImuFactor*>(fallbackFactorsBufferPtr_, &imuFactor, imuTimeK);

  // Add IMU values
  gtsam::Values valuesEstimate;
  valuesEstimate.insert(gtsam::symbol_shorthand::X(newKey), imuPropagatedState_.pose());
  valuesEstimate.insert(gtsam::symbol_shorthand::V(newKey), imuPropagatedState_.velocity());
  valuesEstimate.insert(gtsam::symbol_shorthand::B(newKey), optimizedGraphState_.imuBias());
  globalGraphValuesBufferPtr_->insert(valuesEstimate);
  fallbackGraphValuesBufferPtr_->insert(valuesEstimate);  // Is emptied at each optimization step

  // Add timestamp for fixed lag smoother
  writeValueKeysToKeyTimeStampMap_(valuesEstimate, imuTimeK, globalGraphKeysTimestampsMapBufferPtr_);
  writeValueKeysToKeyTimeStampMap_(valuesEstimate, imuTimeK, fallbackGraphKeysTimestampsMapBufferPtr_);

  // Avoid growing of buffers
  int numKeysinBuffer =
      int(0.9 * graphConfigPtr_->smootherLag * graphConfigPtr_->imuRate);  // 10% buffer to be sure to stay withing smoother lag
  // 3 because of values for pose, velocity and bias
  if (globalGraphValuesBufferPtr_->size() > (3 * numKeysinBuffer)) {
    // Delete factors from graph buffer
    long keyToDelete = newKey - numKeysinBuffer;
    gtsam::Symbol xKeyToFilter = gtsam::symbol_shorthand::X(keyToDelete);
    gtsam::Symbol vKeyToFilter = gtsam::symbol_shorthand::V(keyToDelete);
    gtsam::Symbol bKeyToFilter = gtsam::symbol_shorthand::B(keyToDelete);

    // Values
    globalGraphValuesBufferPtr_->erase(xKeyToFilter);
    globalGraphValuesBufferPtr_->erase(vKeyToFilter);
    globalGraphValuesBufferPtr_->erase(bKeyToFilter);
    // KeyTimeMaps
    globalGraphKeysTimestampsMapBufferPtr_->erase(xKeyToFilter);
    globalGraphKeysTimestampsMapBufferPtr_->erase(vKeyToFilter);
    globalGraphKeysTimestampsMapBufferPtr_->erase(bKeyToFilter);
    // Factors
    if (graphConfigPtr_->verboseLevel > 4) {
      std::cout << YELLOW_START << "GMsf-GraphManager" << COLOR_END << " All keys before and with human readable key "
                << xKeyToFilter.index() << " have to be removed." << std::endl;
      std::cout << YELLOW_START << "GMsf-GraphManager" << COLOR_END
                << " Number of Factors before erasing:  " << globalFactorsBufferPtr_->size() << std::endl;
    }
    // Iterate through factors
    gtsam::Symbol thisKey;  // pre-allocate for speed
    for (auto factorIterator = globalFactorsBufferPtr_->begin(); factorIterator != globalFactorsBufferPtr_->end(); ++factorIterator) {
      // Iterate through keys of factor, usually 1 or 2
      for (int i = 0; i < (*factorIterator)->keys().size(); ++i) {
        thisKey = (*factorIterator)->keys()[i];
        if (thisKey.index() <= keyToDelete) {
          globalFactorsBufferPtr_->erase(factorIterator);
          if (graphConfigPtr_->verboseLevel > 3) {
            std::cout << YELLOW_START << "GMsf-GraphManager" << COLOR_END << GREEN_START << " Erased factor for key number " << COLOR_END
                      << thisKey.index() << std::endl;
          }
        }
      }
    }
    if (graphConfigPtr_->verboseLevel > 4) {
      std::cout << YELLOW_START << "GMsf-GraphManager" << COLOR_END
                << " Number of Factors after erasing:  " << globalFactorsBufferPtr_->size() << std::endl;
    }
  }

  // Relocalization Command
  if (!sentRelocalizationCommandAlready_ && numOptimizationsSinceGraphSwitching_ >= 1) {
    relocalizationFlag = true;
    sentRelocalizationCommandAlready_ = true;
    std::cout << YELLOW_START << "GMsf-GraphManager" << GREEN_START << " Triggering re-localization." << COLOR_END << std::endl;
  } else {
    relocalizationFlag = false;
  }

  // Return copy of propagated state (for publishing)
  return imuPropagatedState_;
}

gtsam::Key GraphManager::addPoseBetweenFactorToGlobalGraph(const double lidarTimeKm1, const double lidarTimeK, const double rate,
                                                           const Eigen::Matrix<double, 6, 1>& poseBetweenNoise, const gtsam::Pose3& pose) {
  // Find corresponding keys in graph
  double maxLidarTimestampDistance = 1.0 / rate + 2.0 * graphConfigPtr_->maxSearchDeviation;
  gtsam::Key closestLidarKeyKm1, closestLidarKeyK;

  if (!findGraphKeys_(maxLidarTimestampDistance, lidarTimeKm1, lidarTimeK, closestLidarKeyKm1, closestLidarKeyK, "lidar delta")) {
    std::cerr << YELLOW_START << "GMsf-GraphManager" << RED_START << " Current propagated key: " << propagatedStateKey_
              << " , PoseBetween factor not added to graph at key " << closestLidarKeyK << COLOR_END << std::endl;
    return closestLidarKeyK;
  }

  // Create noise model
  assert(poseBetweenNoise.size() == 6);
  auto noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << poseBetweenNoise(0), poseBetweenNoise(1), poseBetweenNoise(2),
                                                    poseBetweenNoise(3), poseBetweenNoise(4), poseBetweenNoise(5))
                                                       .finished());  // rad,rad,rad,m,m,m
  auto errorFunction = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(0.5), noise);

  // Create pose between factor and add it
  gtsam::BetweenFactor<gtsam::Pose3> poseBetweenFactor(gtsam::symbol_shorthand::X(closestLidarKeyKm1),
                                                       gtsam::symbol_shorthand::X(closestLidarKeyK), pose, errorFunction);

  // Write to graph
  bool success =
      addFactorSafelyToGraph_<const gtsam::BetweenFactor<gtsam::Pose3>*>(globalFactorsBufferPtr_, &poseBetweenFactor, lidarTimeKm1);

  // Print summary
  if (!success) {
    std::cout << YELLOW_START << "GMsf-GraphManager" << COLOR_END << " Current propagated key: " << propagatedStateKey_ << ","
              << YELLOW_START << " LiDAR PoseBetween factor NOT added between key " << closestLidarKeyKm1 << " and key " << closestLidarKeyK
              << COLOR_END << std::endl;
  } else if (graphConfigPtr_->verboseLevel > 1) {
    std::cout << YELLOW_START << "GMsf-GraphManager" << COLOR_END << " Current propagated key: " << propagatedStateKey_ << ","
              << GREEN_START << " LiDAR PoseBetween factor added between key " << closestLidarKeyKm1 << " and key " << closestLidarKeyK
              << COLOR_END << std::endl;
  }

  return closestLidarKeyK;
}

void GraphManager::addPoseUnaryFactorToFallbackGraph(const double lidarTimeK, const double rate,
                                                     const Eigen::Matrix<double, 6, 1>& poseUnaryNoise, const gtsam::Pose3& T_W_I) {
  // Find closest key in existing graph
  double closestGraphTime;
  gtsam::Key closestKey;

  if (!imuBuffer_.getClosestKeyAndTimestamp(closestGraphTime, closestKey, "FallbackLidarUnary", graphConfigPtr_->maxSearchDeviation,
                                            lidarTimeK)) {
    std::cerr << YELLOW_START << "GMsf-GraphManager" << RED_START << " Not adding lidar unary constraint to graph." << COLOR_END
              << std::endl;
    return;
  }

  assert(poseUnaryNoise.size() == 6);
  auto noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << poseUnaryNoise(0), poseUnaryNoise(1), poseUnaryNoise(2), poseUnaryNoise(3), poseUnaryNoise(4), poseUnaryNoise(5))
          .finished());  // rad,rad,rad,x,y,z
  auto errorFunction = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.5), noise);

  // Unary factor
  gtsam::PriorFactor<gtsam::Pose3> poseUnaryFactor(gtsam::symbol_shorthand::X(closestKey), T_W_I, errorFunction);

  // Write to graph
  bool success = addFactorSafelyToGraph_<const gtsam::PriorFactor<gtsam::Pose3>*>(fallbackFactorsBufferPtr_, &poseUnaryFactor, lidarTimeK);

  // Print summary
  if (!success) {
    std::cout << YELLOW_START << "GMsf-GraphManager" << COLOR_END << " Current propagated key " << propagatedStateKey_ << YELLOW_START
              << " LiDAR unary factor NOT added to key " << closestKey << COLOR_END << std::endl;
  } else if (graphConfigPtr_->verboseLevel > 0) {
    std::cout << YELLOW_START << "GMsf-GraphManager" << COLOR_END << " Current propagated key " << propagatedStateKey_ << GREEN_START
              << " LiDAR unary factor added to key " << closestKey << COLOR_END << std::endl;
  }
}

void GraphManager::addPoseUnaryFactorToGlobalGraph(const double lidarTimeK, const double rate,
                                                   const Eigen::Matrix<double, 6, 1>& poseUnaryNoise, const gtsam::Pose3& unaryPose) {
  // Find closest key in existing graph
  double closestGraphTime;
  gtsam::Key closestKey;

  if (!imuBuffer_.getClosestKeyAndTimestamp(closestGraphTime, closestKey, "GlobalLidarUnary", graphConfigPtr_->maxSearchDeviation,
                                            lidarTimeK)) {
    std::cerr << YELLOW_START << "GMsf-GraphManager" << RED_START << " Not adding lidar unary constraint to graph." << COLOR_END
              << std::endl;
    return;
  }

  assert(poseUnaryNoise.size() == 6);
  auto noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << poseUnaryNoise(0), poseUnaryNoise(1), poseUnaryNoise(2), poseUnaryNoise(3), poseUnaryNoise(4), poseUnaryNoise(5))
          .finished());  // rad,rad,rad,x,y,z
  auto errorFunction = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.5), noise);

  // Unary factor
  gtsam::PriorFactor<gtsam::Pose3> poseUnaryFactor(gtsam::symbol_shorthand::X(closestKey), unaryPose, errorFunction);

  // Write to graph
  bool success = addFactorSafelyToGraph_<const gtsam::PriorFactor<gtsam::Pose3>*>(globalFactorsBufferPtr_, &poseUnaryFactor, lidarTimeK);

  // Print summary
  if (!success) {
    std::cout << YELLOW_START << "GMsf-GraphManager" << COLOR_END << " Current propagated key " << propagatedStateKey_ << YELLOW_START
              << " LiDAR unary factor NOT added to key " << closestKey << COLOR_END << std::endl;
  } else if (graphConfigPtr_->verboseLevel > 0) {
    std::cout << YELLOW_START << "GMsf-GraphManager" << COLOR_END << " Current propagated key " << propagatedStateKey_ << GREEN_START
              << " LiDAR unary factor added to key " << closestKey << COLOR_END << std::endl;
  }
}

void GraphManager::addGnssPositionUnaryFactor(double gnssTimeK, const double rate, const Eigen::Vector3d& gnssPositionUnaryNoise,
                                              const gtsam::Vector3& position) {
  // Find closest key in existing graph
  double closestGraphTime;
  gtsam::Key closestKey;
  if (!imuBuffer_.getClosestKeyAndTimestamp(closestGraphTime, closestKey, "GnssUnary", graphConfigPtr_->maxSearchDeviation, gnssTimeK)) {
    //    std::cerr << YELLOW_START << "GMsf-GraphManager" << RED_START << " Not adding gnss unary constraint to graph." << COLOR_END
    //              << std::endl;
    //    return;
  }

  // Create noise model
  auto noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(3) << gnssPositionUnaryNoise[0], gnssPositionUnaryNoise[1], gnssPositionUnaryNoise[2])
          .finished());  // rad,rad,rad,m,m,m
  auto tukeyErrorFunction = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(0.7), noise);

  // Create unary factor and add it
  gtsam::GPSFactor gnssPositionUnaryFactor(gtsam::symbol_shorthand::X(closestKey), position, tukeyErrorFunction);

  // Write to graph
  bool success = addFactorSafelyToGraph_<const gtsam::GPSFactor*>(globalFactorsBufferPtr_, &gnssPositionUnaryFactor, gnssTimeK);

  // Print summary
  if (!success) {
    std::cout << YELLOW_START << "GMsf-GraphManager" << COLOR_END << " Current propagated key " << propagatedStateKey_ << YELLOW_START
              << ", Gnss position factor NOT added to key " << closestKey << COLOR_END << std::endl;
  } else if (graphConfigPtr_->verboseLevel > 1) {
    std::cout << YELLOW_START << "GMsf-GraphManager" << COLOR_END << " Current propagated key " << propagatedStateKey_ << GREEN_START
              << ", Gnss position factor added to key " << closestKey << COLOR_END << std::endl;
  }
}

void GraphManager::addGnssHeadingUnaryFactor(double gnssTimeK, const double rate, const double gnssHeadingUnaryNoise,
                                             const double measuredYaw) {
  // Print information
  if (graphConfigPtr_->verboseLevel > 2) {
    std::cout << YELLOW_START << "GMsf-GraphManager" << COLOR_END << " Current propagated key " << propagatedStateKey_
              << std::setprecision(14) << ", Gnss yaw measurement at time stamp " << gnssTimeK << " is: " << measuredYaw << std::endl;
  }

  // Find closest key in existing graph
  double closestGraphTime;
  gtsam::Key closestKey;
  if (!imuBuffer_.getClosestKeyAndTimestamp(closestGraphTime, closestKey, "GnssHeading", graphConfigPtr_->maxSearchDeviation, gnssTimeK)) {
    //    std::cerr << YELLOW_START << "GMsf-GraphManager" << RED_START << " Not adding gnss heading constraint to graph." << COLOR_END
    //              << std::endl;
    //    return;
  }

  // Create noise model
  //  auto gnssHeadingUnaryNoise =
  //      gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(1) << gnssHeadingUnaryNoise_).finished());  // rad,rad,rad,m,m,m
  auto noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(1) << gnssHeadingUnaryNoise).finished());  // rad,rad,rad,m,m,m
  auto tukeyErrorFunction = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(0.5), noise);

  HeadingFactor gnssHeadingUnaryFactor(gtsam::symbol_shorthand::X(closestKey), measuredYaw, tukeyErrorFunction);

  // Write to graph
  bool success = addFactorSafelyToGraph_<const HeadingFactor*>(globalFactorsBufferPtr_, &gnssHeadingUnaryFactor, gnssTimeK);

  // Print summary
  if (!success) {
    std::cout << YELLOW_START << "GMsf-GraphManager" << COLOR_END << " Current propagated key " << propagatedStateKey_ << YELLOW_START
              << " Gnss heading factor is NOT added to key " << closestKey << COLOR_END << std::endl;
  } else if (graphConfigPtr_->verboseLevel > 0) {
    std::cout << YELLOW_START << "GMsf-GraphManager" << COLOR_END << " Current propagated key " << propagatedStateKey_ << GREEN_START
              << " Gnss heading factor is added to key " << closestKey << COLOR_END << std::endl;
  }
}

bool GraphManager::addZeroMotionFactor(double maxTimestampDistance, double timeKm1, double timeK, const gtsam::Pose3 pose) {
  // Find corresponding keys in graph
  gtsam::Key closestKeyKm1, closestKeyK;
  if (!findGraphKeys_(maxTimestampDistance, timeKm1, timeK, closestKeyKm1, closestKeyK, "zero motion factor")) {
    return false;
  }

  // Factors
  gtsam::BetweenFactor<gtsam::Pose3> poseBetweenFactor(gtsam::symbol_shorthand::X(closestKeyKm1), gtsam::symbol_shorthand::X(closestKeyK),
                                                       gtsam::Pose3::Identity(), gtsam::noiseModel::Isotropic::Sigma(6, 1e-3));
  gtsam::PriorFactor<gtsam::Vector3> velocityUnaryFactor(gtsam::symbol_shorthand::V(closestKeyKm1), gtsam::Vector3::Zero(),
                                                         gtsam::noiseModel::Isotropic::Sigma(3, 1e-3));

  // Global
  /// Add Zero Pose Factor
  bool success = addFactorSafelyToGraph_<const gtsam::BetweenFactor<gtsam::Pose3>*>(globalFactorsBufferPtr_, &poseBetweenFactor, timeKm1);
  /// Add Zero Velocity Factor
  success =
      success && addFactorSafelyToGraph_<const gtsam::PriorFactor<gtsam::Vector3>*>(globalFactorsBufferPtr_, &velocityUnaryFactor, timeKm1);
  // Fallback
  /// Add Zero Pose Factor
  success =
      success && addFactorSafelyToGraph_<const gtsam::BetweenFactor<gtsam::Pose3>*>(fallbackFactorsBufferPtr_, &poseBetweenFactor, timeKm1);
  /// Add Zero Velocity Factor
  success = success &&
            addFactorSafelyToGraph_<const gtsam::PriorFactor<gtsam::Vector3>*>(fallbackFactorsBufferPtr_, &velocityUnaryFactor, timeKm1);

  if (graphConfigPtr_->verboseLevel > 0) {
    std::cout << YELLOW_START << "GMsf-GraphManager" << COLOR_END << " Current propagated key " << propagatedStateKey_
              << GREEN_START " Zero Motion Factor added between keys " << closestKeyKm1 << " and " << closestKeyK << COLOR_END << std::endl;
  }
  return true;
}

void GraphManager::activateGlobalGraph(const gtsam::Vector3& imuPosition, const gtsam::Rot3& imuRotation, const double measurementTime) {
  // Mutex, such s.t. the used graph is consistent
  const std::lock_guard<std::mutex> swappingActiveGraphLock(swappingActiveGraphMutex_);
  if (activeSmootherPtr_ != globalSmootherPtr_) {
    // Activate global graph
    int smootherNumberStates = int(graphConfigPtr_->smootherLag * graphConfigPtr_->imuRate);
    gtsam::NonlinearFactorGraph globalFactorsBuffer;
    gtsam::Values globalGraphValues;
    std::map<gtsam::Key, double> globalGraphKeysTimestampsMap;
    bool optimizeGraphFlag = false;
    gtsam::Key currentPropagatedKey;
    double currentPropagatedTime;

    // Check graph data with lock: Mutex Block 1 ------------------
    {
      const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
      currentPropagatedKey = propagatedStateKey_;
      currentPropagatedTime = propagatedStateTime_;
      int lastKeyInSmoother = (--globalSmootherPtr_->timestamps().end())->first;

      std::cout << YELLOW_START << "GMsf-GraphManager" << GREEN_START << " Last Key in Smoother is: " << lastKeyInSmoother
                << ", current key is: " << currentPropagatedKey << COLOR_END << std::endl;
      if (lastKeyInSmoother < (int(currentPropagatedKey) - smootherNumberStates)) {
        optimizeGraphFlag = true;
        std::cout
            << YELLOW_START << "GMsf-GraphManager" << GREEN_START
            << " Hence, previous global graph is too old --> previously optimized graph can not be used --> shrinking and re-optimizing..."
            << COLOR_END << std::endl;

        // Copy
        globalFactorsBuffer = *globalFactorsBufferPtr_;
        globalGraphValues = *globalGraphValuesBufferPtr_;
        globalGraphKeysTimestampsMap = *globalGraphKeysTimestampsMapBufferPtr_;
        globalFactorsBufferPtr_->resize(0);
        globalGraphValuesBufferPtr_->clear();
        globalGraphKeysTimestampsMapBufferPtr_->clear();
        if (graphConfigPtr_->usingBiasForPreIntegrationFlag) {
          globalImuBufferPreintegratorPtr_->resetIntegrationAndSetBias(optimizedGraphState_.imuBias());
        } else {
          globalImuBufferPreintegratorPtr_->resetIntegrationAndSetBias(gtsam::imuBias::ConstantBias());
        }
      } else {
        std::cout << YELLOW_START << "GMsf-GraphManager" << GREEN_START << " Hence, previously optimized graph can be used." << COLOR_END
                  << std::endl;
      }
    }

    if (optimizeGraphFlag) {
      // Find closest key
      double closestGraphTimeToGnssMeasurement;
      gtsam::Key closestKeyToGnssMeasurement;
      imuBuffer_.getClosestKeyAndTimestamp(closestGraphTimeToGnssMeasurement, closestKeyToGnssMeasurement, "GnssUnary",
                                           graphConfigPtr_->maxSearchDeviation, measurementTime);

      // Reoptimization of global graph outside of lock ---------------
      // Add prior factor for observability
      std::cout << YELLOW_START << "GMsf-GraphManager" << GREEN_START << " Adding prior factor for GNSS key " << closestKeyToGnssMeasurement
                << COLOR_END << std::endl;
      // Putting high uncertainty for pose prior because GNSS factors are added that constrain x,y,z and yaw
      // Roll and pitch are not constrained, and correct also in fallback graph, hence lower uncertainty for these
      // Pose
      // gtsam::Pose3 providedPose(imuRotation, imuPosition);
      gtsam::Pose3 providedPose(globalGraphValues.at<gtsam::Pose3>(gtsam::symbol_shorthand::X(currentPropagatedKey)).rotation(),
                                imuPosition);
      // Set Translation To Imu Position
      gtsam::PriorFactor<gtsam::Pose3> posePrior(
          gtsam::symbol_shorthand::X(closestKeyToGnssMeasurement), providedPose,
          gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-05, 1e-05, 1e-02, 1e02, 1e02, 1e02).finished()));
      globalFactorsBuffer.add(posePrior);
      // Velocity
      gtsam::PriorFactor<gtsam::Vector3> velPrior(gtsam::symbol_shorthand::V(currentPropagatedKey),
                                                  globalGraphValues.at<gtsam::Vector3>(gtsam::symbol_shorthand::V(currentPropagatedKey)),
                                                  gtsam::noiseModel::Isotropic::Sigma(3, 1e-03));  // VELOCITY
      globalFactorsBuffer.add(velPrior);
      // Bias
      gtsam::PriorFactor<gtsam::imuBias::ConstantBias> biasPrior(
          gtsam::symbol_shorthand::B(currentPropagatedKey), optimizedGraphState_.imuBias(),
          gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-03, 1e-03, 1e-03, 1e-03, 1e-03, 1e-03).finished()));  // BIAS
      globalFactorsBuffer.add(biasPrior);

      // Optimize over it with good intitial guesses
      globalSmootherPtr_ = std::make_shared<gtsam::IncrementalFixedLagSmoother>(
          graphConfigPtr_->smootherLag, isamParams_);  // std::make_shared<gtsam::ISAM2>(isamParams_);
      addFactorsToSmootherAndOptimize(globalSmootherPtr_, globalFactorsBuffer, globalGraphValues, globalGraphKeysTimestampsMap,
                                      graphConfigPtr_, 5);
    }

    {
      const std::lock_guard<std::mutex> activelyUSingActiveGraphLock(activelyUsingActiveGraphMutex_);

      // Switching of active Graph ---------------
      activeSmootherPtr_ = globalSmootherPtr_;
      activeFactorsBufferPtr_ = globalFactorsBufferPtr_;
      activeImuBufferPreintegratorPtr_ = globalImuBufferPreintegratorPtr_;
      activeGraphValuesBufferPtr_ = globalGraphValuesBufferPtr_;
      activeGraphKeysTimestampsMapBufferPtr_ = globalGraphKeysTimestampsMapBufferPtr_;
      {
        const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
        sentRelocalizationCommandAlready_ = false;
        numOptimizationsSinceGraphSwitching_ = 0;
      }
    }

    std::cout << YELLOW_START << "GMsf-GraphManager" << GREEN_START << " Activated global graph pointer." << COLOR_END << std::endl;
  }
}

void GraphManager::activateFallbackGraph() {
  // Mutex, such s.t. the used graph is consistent
  const std::lock_guard<std::mutex> swappingActiveGraphLock(swappingActiveGraphMutex_);
  if (activeSmootherPtr_ != fallbackSmootherPtr_) {
    {
      const std::lock_guard<std::mutex> activelyUsingActiveGraphLock(activelyUsingActiveGraphMutex_);
      *fallbackSmootherPtr_ = *globalSmootherPtr_;
      std::cout << YELLOW_START << "GraphManager" << GREEN_START << " Reset fallback graph to global graph." << COLOR_END << std::endl;
      activeSmootherPtr_ = fallbackSmootherPtr_;
      activeFactorsBufferPtr_ = fallbackFactorsBufferPtr_;
      activeImuBufferPreintegratorPtr_ = fallbackImuBufferPreintegratorPtr_;
      activeGraphValuesBufferPtr_ = fallbackGraphValuesBufferPtr_;
      activeGraphKeysTimestampsMapBufferPtr_ = fallbackGraphKeysTimestampsMapBufferPtr_;
      // Reset counter
      numOptimizationsSinceGraphSwitching_ = 0;
    }
    std::cout << YELLOW_START << "GraphManager" << GREEN_START << " Activated fallback graph pointer." << COLOR_END << std::endl;
  }
}

gtsam::NavState GraphManager::calculateNavStateAtKey(std::shared_ptr<gtsam::IncrementalFixedLagSmoother> graphPtr,
                                                     const std::shared_ptr<GraphConfig>& graphConfigPtr, const gtsam::Key& key) {
  gtsam::Pose3 resultPose;
  gtsam::Vector3 resultVelocity;
  if (true) {
    try {
      resultPose = graphPtr->calculateEstimate<gtsam::Pose3>(gtsam::symbol_shorthand::X(key));  // auto result = mainGraphPtr_->estimate();
      resultVelocity = graphPtr->calculateEstimate<gtsam::Vector3>(gtsam::symbol_shorthand::V(key));
    } catch (const std::out_of_range& outOfRangeExeception) {
      std::cerr << "Out of Range exeception while optimizing graph: " << outOfRangeExeception.what() << '\n';
      std::cout << YELLOW_START << "GMsf-GraphManager" << RED_START
                << " This happens if the measurement delay is larger than the graph-smootherLag, i.e. the optimized graph instances are "
                   "not connected. Increase the lag in this case."
                << COLOR_END << std::endl;
      throw std::out_of_range("");
    }
  }
  return gtsam::NavState(resultPose, resultVelocity);
}

SafeNavStateWithCovarianceAndBias GraphManager::updateActiveGraphAndGetState(double& currentPropagatedTime) {
  // Mutex, such s.t. the used graph is consistent
  const std::lock_guard<std::mutex> activelyUsingActiveGraphLock(activelyUsingActiveGraphMutex_);

  // Method variables
  gtsam::NonlinearFactorGraph newActiveGraphFactors;
  gtsam::Values newActiveGraphValues;
  std::map<gtsam::Key, double> newActiveGraphKeysTimestampsMap;
  gtsam::Key currentPropagatedKey;

  // Mutex Block 1 -----------------
  {
    // Lock
    const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
    // Get copy of factors and values
    newActiveGraphFactors = *activeFactorsBufferPtr_;
    newActiveGraphValues = *activeGraphValuesBufferPtr_;
    newActiveGraphKeysTimestampsMap = *activeGraphKeysTimestampsMapBufferPtr_;
    // Empty factor buffers, have to always empty fallback graph, as graphs are synced
    fallbackFactorsBufferPtr_->resize(0);
    fallbackGraphValuesBufferPtr_->clear();
    fallbackGraphKeysTimestampsMapBufferPtr_->clear();
    if (activeFactorsBufferPtr_ == globalFactorsBufferPtr_) {
      globalFactorsBufferPtr_->resize(0);
      globalGraphValuesBufferPtr_->clear();
      globalGraphKeysTimestampsMapBufferPtr_->clear();
    }

    // Empty Buffer Preintegrator --> everything missed during the update will be in here
    if (graphConfigPtr_->usingBiasForPreIntegrationFlag) {
      activeImuBufferPreintegratorPtr_->resetIntegrationAndSetBias(optimizedGraphState_.imuBias());
    } else {
      activeImuBufferPreintegratorPtr_->resetIntegrationAndSetBias(gtsam::imuBias::ConstantBias());
    }
    // Get current key and time
    currentPropagatedKey = propagatedStateKey_;
    currentPropagatedTime = propagatedStateTime_;
  }  // end of locking

  // Graph Update (time consuming) -------------------
  addFactorsToSmootherAndOptimize(activeSmootherPtr_, newActiveGraphFactors, newActiveGraphValues, newActiveGraphKeysTimestampsMap,
                                  graphConfigPtr_, graphConfigPtr_->additionalOptimizationIterations);

  // Compute entire result
  // NavState
  gtsam::NavState resultNavState = calculateNavStateAtKey(activeSmootherPtr_, graphConfigPtr_, currentPropagatedKey);
  // Bias
  gtsam::imuBias::ConstantBias resultBias =
      activeSmootherPtr_->calculateEstimate<gtsam::imuBias::ConstantBias>(gtsam::symbol_shorthand::B(currentPropagatedKey));
  // Compute Covariance
  gtsam::Matrix66 poseCovariance = activeSmootherPtr_->marginalCovariance(gtsam::symbol_shorthand::X(currentPropagatedKey));
  gtsam::Matrix33 velocityCovariance = activeSmootherPtr_->marginalCovariance(gtsam::symbol_shorthand::V(currentPropagatedKey));

  // Mutex block 2 ------------------
  {
    // Lock
    const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
    // Update Graph State
    optimizedGraphState_.updateNavStateAndBias(currentPropagatedKey, currentPropagatedTime, resultNavState, resultBias);
    // Predict from solution to obtain refined propagated state
    // Resetting is done at beginning of next optimization
    if (graphConfigPtr_->usingBiasForPreIntegrationFlag) {
      imuPropagatedState_ = activeImuBufferPreintegratorPtr_->predict(resultNavState, optimizedGraphState_.imuBias());
    } else {
      imuPropagatedState_ = activeImuBufferPreintegratorPtr_->predict(resultNavState, gtsam::imuBias::ConstantBias());
    }

    // Increase counter
    ++numOptimizationsSinceGraphSwitching_;
  }  // end of locking

  // Return result
  SafeNavStateWithCovarianceAndBias resultNavStateWithCovarianceAndBias(
      Eigen::Isometry3d(resultNavState.pose().matrix()), resultNavState.v(), currentPropagatedTime, poseCovariance, velocityCovariance,
      resultBias.accelerometer(), resultBias.gyroscope());

  return resultNavStateWithCovarianceAndBias;
}

void GraphManager::addFactorsToSmootherAndOptimize(std::shared_ptr<gtsam::IncrementalFixedLagSmoother> smootherPtr,
                                                   const gtsam::NonlinearFactorGraph& newGraphFactors, const gtsam::Values& newGraphValues,
                                                   const std::map<gtsam::Key, double>& newGraphKeysTimestampsMap,
                                                   const std::shared_ptr<GraphConfig> graphConfigPtr, const int additionalIterations) {
  // Timing
  std::chrono::time_point<std::chrono::high_resolution_clock> startLoopTime;
  std::chrono::time_point<std::chrono::high_resolution_clock> endLoopTime;
  startLoopTime = std::chrono::high_resolution_clock::now();

  // Perform update
  try {
    smootherPtr->update(newGraphFactors, newGraphValues, newGraphKeysTimestampsMap);
    // Additional iterations
    for (size_t itr = 0; itr < additionalIterations; ++itr) {
      smootherPtr->update();
    }
  } catch (const std::out_of_range& outOfRangeExeception) {
    std::cerr << YELLOW_START << "GMsf-GraphManager" << RED_START
              << "Out of Range exeception while optimizing graph: " << outOfRangeExeception.what() << COLOR_END << std::endl;
    std::cout << YELLOW_START << "GMsf-GraphManager" << RED_START
              << " This happens if the measurement delay is larger than the graph-smootherLag, i.e. the optimized graph instances are not "
                 "connected. Increase the lag in this case."
              << COLOR_END << std::endl;
    throw std::out_of_range("");
  } catch (const gtsam::ValuesKeyDoesNotExist& valuesKeyDoesNotExistException) {
    std::cerr << YELLOW_START << "GMsf-GraphManager" << RED_START
              << "Values Key Does Not Exist exeception while optimizing graph: " << valuesKeyDoesNotExistException.what() << COLOR_END
              << std::endl;
    std::cout << YELLOW_START << "GMsf-GraphManager" << RED_START
              << " This happens if a value (i.e. initial guess) for a certain variable is missing." << COLOR_END << std::endl;
    throw std::out_of_range("");
  }

  if (graphConfigPtr->verboseLevel > 0) {
    endLoopTime = std::chrono::high_resolution_clock::now();
    std::cout << YELLOW_START << "GMsf-GraphManager" << GREEN_START << " Whole optimization loop took "
              << std::chrono::duration_cast<std::chrono::milliseconds>(endLoopTime - startLoopTime).count() << " milliseconds." << COLOR_END
              << std::endl;
  }
}  // namespace graph_msf

gtsam::NavState GraphManager::calculateActiveStateAtKey(const gtsam::Key& key) {
  const std::lock_guard<std::mutex> activelyUSingActiveGraphLock(activelyUsingActiveGraphMutex_);
  return calculateNavStateAtKey(activeSmootherPtr_, graphConfigPtr_, key);
}

// Private --------------------------------------------------------------------
template <class CHILDPTR>
bool GraphManager::addFactorToGraph_(std::shared_ptr<gtsam::NonlinearFactorGraph> modifiedGraphPtr,
                                     const gtsam::NoiseModelFactor* noiseModelFactorPtr) {
  modifiedGraphPtr->add(*dynamic_cast<CHILDPTR>(noiseModelFactorPtr));
  return true;
}

template <class CHILDPTR>
bool GraphManager::addFactorToGraph_(std::shared_ptr<gtsam::NonlinearFactorGraph> modifiedGraphPtr,
                                     const gtsam::NoiseModelFactor* noiseModelFactorPtr, const double measurementTimestamp) {
  // Check Timestamp of Measurement on Delay
  if (imuBuffer_.getLatestTimestampInBuffer() - measurementTimestamp > graphConfigPtr_->smootherLag - WORST_CASE_OPTIMIZATION_TIME) {
    std::cout << YELLOW_START << "GMsf-GraphManager" << RED_START
              << " Measurement Delay is larger than the smootherLag - WORST_CASE_OPTIMIZATION_TIME, hence skipping this measurement."
              << COLOR_END << std::endl;
    return false;
  }
  // Add measurements
  return addFactorToGraph_<CHILDPTR>(modifiedGraphPtr, noiseModelFactorPtr);
}

template <class CHILDPTR>
bool GraphManager::addFactorSafelyToGraph_(std::shared_ptr<gtsam::NonlinearFactorGraph> modifiedGraphPtr,
                                           const gtsam::NoiseModelFactor* noiseModelFactorPtr, const double measurementTimestamp) {
  // Operating on graph data --> acquire mutex
  const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
  // Add measurements
  return addFactorToGraph_<CHILDPTR>(modifiedGraphPtr, noiseModelFactorPtr, measurementTimestamp);
}

bool GraphManager::findGraphKeys_(double maxTimestampDistance, double timeKm1, double timeK, gtsam::Key& closestKeyKm1,
                                  gtsam::Key& closestKeyK, const std::string& name) {
  // Find closest lidar keys in existing graph
  double closestGraphTimeKm1, closestGraphTimeK;
  {
    // Looking up from IMU buffer --> acquire mutex (otherwise values for key might not be set)
    const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
    bool successKm1 = imuBuffer_.getClosestKeyAndTimestamp(closestGraphTimeKm1, closestKeyKm1, name + " km1",
                                                           graphConfigPtr_->maxSearchDeviation, timeKm1);
    bool successK =
        imuBuffer_.getClosestKeyAndTimestamp(closestGraphTimeK, closestKeyK, name + " k", graphConfigPtr_->maxSearchDeviation, timeK);
    if (!successKm1 || !successK) {
      return false;
    }
  }

  // Check
  if (closestGraphTimeKm1 > closestGraphTimeK) {
    std::cerr << YELLOW_START << "GMsf-GraphManager" << RED_START << " Time at time step k-1 must be smaller than time at time step k."
              << COLOR_END << std::endl;
    return false;
  }

  double keyTimestampDistance = std::abs(closestGraphTimeK - closestGraphTimeKm1);
  if (keyTimestampDistance > maxTimestampDistance) {
    std::cerr << YELLOW_START << "GMsf-GraphManager"
              << " Distance of " << name << " timestamps is too big. Found timestamp difference is  "
              << closestGraphTimeK - closestGraphTimeKm1 << " which is larger than the maximum admissible distance of "
              << maxTimestampDistance << ". Still adding constraints to graph." << COLOR_END << std::endl;
  }
  return true;
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

  // Start integrating with imu_meas.begin()+1 meas to calculate dt, imu_meas.begin() meas was integrated before
  auto currItr = imuMeas.begin();
  auto prevItr = currItr;

  // Calculate dt and integrate IMU measurements for both preintegrators
  for (++currItr; currItr != imuMeas.end(); ++currItr, ++prevItr) {
    double dt = currItr->first - prevItr->first;
    imuStepPreintegratorPtr_->integrateMeasurement(currItr->second.head<3>(),          // acc
                                                   currItr->second.tail<3>(),          // gyro
                                                   dt);                                // delta t
    globalImuBufferPreintegratorPtr_->integrateMeasurement(currItr->second.head<3>(),  // acc
                                                           currItr->second.tail<3>(),  // gyro
                                                           dt);
    fallbackImuBufferPreintegratorPtr_->integrateMeasurement(currItr->second.head<3>(),  // acc
                                                             currItr->second.tail<3>(),  // gyro
                                                             dt);
  }
}

}  // namespace graph_msf