/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#define WORST_CASE_OPTIMIZATION_TIME 0.1  // in seconds

// Factors
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

// Workspace
#include "graph_msf/core/GraphManager.hpp"

namespace graph_msf {

// Public --------------------------------------------------------------------

GraphManager::GraphManager(std::shared_ptr<GraphConfig> graphConfigPtr) : graphConfigPtr_(graphConfigPtr) {
  // Buffer
  factorGraphBufferPtr_ = std::make_shared<gtsam::NonlinearFactorGraph>();
  graphValuesBufferPtr_ = std::make_shared<gtsam::Values>();
  graphKeysTimestampsMapBufferPtr_ = std::make_shared<std::map<gtsam::Key, double>>();

  // Keys
  timeToKeyBufferPtr_ = std::make_shared<TimeGraphKeyBuffer>(graphConfigPtr_->imuBufferLength, graphConfigPtr_->verboseLevel);

  // ISAM
  isamParams_.findUnusedFactorSlots = graphConfigPtr_->findUnusedFactorSlotsFlag;
  isamParams_.enableDetailedResults = graphConfigPtr_->enableDetailedResultsFlag;
  isamParams_.relinearizeSkip = graphConfigPtr_->relinearizeSkip;
  isamParams_.enableRelinearization = graphConfigPtr_->enableRelinearizationFlag;
  isamParams_.evaluateNonlinearError = graphConfigPtr_->evaluateNonlinearErrorFlag;
  isamParams_.cacheLinearizedFactors = graphConfigPtr_->cacheLinearizedFactorsFlag;
  isamParams_.enablePartialRelinearizationCheck = graphConfigPtr_->enablePartialRelinearizationCheckFlag;
}

bool GraphManager::initImuIntegrators(const double g) {
  // Gravity direction definition
  imuParamsPtr_ = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(g);  // ROS convention

  // Set noise and bias parameters
  /// Position
  imuParamsPtr_->setAccelerometerCovariance(gtsam::Matrix33::Identity(3, 3) * graphConfigPtr_->accNoiseDensity);
  imuParamsPtr_->setIntegrationCovariance(
      gtsam::Matrix33::Identity(3, 3) *
      graphConfigPtr_->integrationNoiseDensity);  // error committed in integrating position from velocities
  imuParamsPtr_->setUse2ndOrderCoriolis(graphConfigPtr_->use2ndOrderCoriolisFlag);
  /// Rotation
  imuParamsPtr_->setGyroscopeCovariance(gtsam::Matrix33::Identity(3, 3) * graphConfigPtr_->gyroNoiseDensity);
  imuParamsPtr_->setOmegaCoriolis(gtsam::Vector3(0, 0, 1) * graphConfigPtr_->omegaCoriolis);
  /// Bias
  imuParamsPtr_->setBiasAccCovariance(gtsam::Matrix33::Identity(3, 3) * graphConfigPtr_->accBiasRandomWalk);
  imuParamsPtr_->setBiasOmegaCovariance(gtsam::Matrix33::Identity(3, 3) * graphConfigPtr_->gyroBiasRandomWalk);
  imuParamsPtr_->setBiasAccOmegaInit(gtsam::Matrix66::Identity(6, 6) *
                                     graphConfigPtr_->biasAccOmegaInit);  // covariance of bias used for preintegration

  // Use previously defined prior for gyro
  imuBiasPriorPtr_ = std::make_shared<gtsam::imuBias::ConstantBias>(graphConfigPtr_->accBiasPrior, graphConfigPtr_->gyroBiasPrior);

  // Init Pre-integrators
  imuStepPreintegratorPtr_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(imuParamsPtr_, *imuBiasPriorPtr_);
  imuBufferPreintegratorPtr_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(imuParamsPtr_, *imuBiasPriorPtr_);
  imuParamsPtr_->print("GraphMSF-IMU PreIntegration Parameters:");
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
  fixedLagSmootherPtr_ = std::make_shared<gtsam::IncrementalFixedLagSmoother>(graphConfigPtr_->smootherLag,
                                                                              isamParams_);  // std::make_shared<gtsam::ISAM2>(isamParams_);
  fixedLagSmootherPtr_->params().print("GraphMSF: Factor Graph Parameters of global graph.");

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
  std::cout << YELLOW_START << "GMsf-GraphManager" << COLOR_END << " Initial Pose: " << initialPose << std::endl;
  valuesEstimate.insert(gtsam::symbol_shorthand::X(propagatedStateKey_), initialPose);
  valuesEstimate.insert(gtsam::symbol_shorthand::V(propagatedStateKey_), gtsam::Vector3(0, 0, 0));
  valuesEstimate.insert(gtsam::symbol_shorthand::B(propagatedStateKey_), *imuBiasPriorPtr_);
  /// Timestamp mapping for incremental fixed lag smoother
  std::shared_ptr<std::map<gtsam::Key, double>> priorKeyTimestampMapPtr = std::make_shared<std::map<gtsam::Key, double>>();
  writeValueKeysToKeyTimeStampMap_(valuesEstimate, timeStep, priorKeyTimestampMapPtr);

  // Initialize both graphs -------------------------------------------------
  /// Global Factors
  factorGraphBufferPtr_->resize(0);
  factorGraphBufferPtr_->emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      gtsam::symbol_shorthand::X(propagatedStateKey_), initialPose,
      priorPoseNoise);  // POSE - PriorFactor format is (key,value,matrix) value is same type as type of PriorFactor
  factorGraphBufferPtr_->emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(gtsam::symbol_shorthand::V(propagatedStateKey_),
                                                                            gtsam::Vector3(0, 0, 0),
                                                                            priorVelocityNoise);  // VELOCITY
  factorGraphBufferPtr_->emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(gtsam::symbol_shorthand::B(propagatedStateKey_),
                                                                                          *imuBiasPriorPtr_,
                                                                                          priorBiasNoise);  // BIAS

  /// Add prior factor to graph and optimize for the first time ----------------
  if (graphConfigPtr_->useIsamFlag) {
    addFactorsToSmootherAndOptimize(fixedLagSmootherPtr_, *factorGraphBufferPtr_, valuesEstimate, *priorKeyTimestampMapPtr, graphConfigPtr_,
                                    0);
  } else {
    throw std::runtime_error("Optimizer has to be ISAM at this stage.");
  }

  factorGraphBufferPtr_->resize(0);

  // Update Current State ---------------------------------------------------
  optimizedGraphState_.updateNavStateAndBias(propagatedStateKey_, timeStep, gtsam::NavState(initialPose, gtsam::Vector3(0, 0, 0)),
                                             *imuBiasPriorPtr_);
  imuPropagatedState_ = gtsam::NavState(initialPose, gtsam::Vector3(0, 0, 0));
  return true;
}

gtsam::NavState GraphManager::addImuFactorAndGetState(const double imuTimeK, std::shared_ptr<ImuBuffer> imuBufferPtr) {
  // Looking up from IMU buffer --> acquire mutex (otherwise values for key might not be set)
  const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);

  // Write current time
  propagatedStateTime_ = imuTimeK;

  // Get new key
  gtsam::Key oldKey = propagatedStateKey_;
  gtsam::Key newKey = newPropagatedStateKey_();

  // Add to key buffer
  timeToKeyBufferPtr_->addToBuffer(imuTimeK, newKey);

  // Get last two measurements from buffer to determine dt
  TimeToImuMap imuMeas;
  imuBufferPtr->getLastTwoMeasurements(imuMeas);

  // Update IMU preintegrator
  updateImuIntegrators_(imuMeas);
  // Predict propagated state
  if (graphConfigPtr_->usingBiasForPreIntegrationFlag) {
    imuPropagatedState_ = imuBufferPtr->integrateNavStateFromTimestamp(imuMeas.begin()->first, imuMeas.rbegin()->first, imuPropagatedState_,
                                                                       optimizedGraphState_.imuBias(), graphConfigPtr_->W_gravityVector);
    // imuPropagatedState_ = imuStepPreintegratorPtr_->predict(imuPropagatedState_, optimizedGraphState_.imuBias());
  } else {
    imuPropagatedState_ = imuBufferPtr->integrateNavStateFromTimestamp(imuMeas.begin()->first, imuMeas.rbegin()->first, imuPropagatedState_,
                                                                       gtsam::imuBias::ConstantBias(), graphConfigPtr_->W_gravityVector);
    // imuPropagatedState_ = imuStepPreintegratorPtr_->predict(imuPropagatedState_, gtsam::imuBias::ConstantBias());
  }

  // Add IMU Factor to graph
  gtsam::CombinedImuFactor imuFactor(gtsam::symbol_shorthand::X(oldKey), gtsam::symbol_shorthand::V(oldKey),
                                     gtsam::symbol_shorthand::X(newKey), gtsam::symbol_shorthand::V(newKey),
                                     gtsam::symbol_shorthand::B(oldKey), gtsam::symbol_shorthand::B(newKey), *imuStepPreintegratorPtr_);
  bool success = addFactorToGraph_<const gtsam::CombinedImuFactor*>(factorGraphBufferPtr_, &imuFactor, imuTimeK);

  // Add IMU values
  gtsam::Values valuesEstimate;
  valuesEstimate.insert(gtsam::symbol_shorthand::X(newKey), imuPropagatedState_.pose());
  valuesEstimate.insert(gtsam::symbol_shorthand::V(newKey), imuPropagatedState_.velocity());
  valuesEstimate.insert(gtsam::symbol_shorthand::B(newKey), optimizedGraphState_.imuBias());
  graphValuesBufferPtr_->insert(valuesEstimate);
  writeValueKeysToKeyTimeStampMap_(valuesEstimate, imuTimeK, graphKeysTimestampsMapBufferPtr_);

  //  // Add timestamp for fixed lag smoother
  //  writeValueKeysToKeyTimeStampMap_(valuesEstimate, imuTimeK, globalGraphKeysTimestampsMapBufferPtr_);
  //  writeValueKeysToKeyTimeStampMap_(valuesEstimate, imuTimeK, fallbackGraphKeysTimestampsMapBufferPtr_);
  //
  //  // Avoid growing of buffers
  //  int numKeysinBuffer =
  //      int(0.9 * graphConfigPtr_->smootherLag * graphConfigPtr_->imuRate);  // 10% buffer to be sure to stay withing smoother lag
  //  // 3 because of values for pose, velocity and bias
  //  if (globalGraphValuesBufferPtr_->size() > (3 * numKeysinBuffer)) {
  //    // Delete factors from graph buffer
  //    long keyToDelete = newKey - numKeysinBuffer;
  //    gtsam::Symbol xKeyToFilter = gtsam::symbol_shorthand::X(keyToDelete);
  //    gtsam::Symbol vKeyToFilter = gtsam::symbol_shorthand::V(keyToDelete);
  //    gtsam::Symbol bKeyToFilter = gtsam::symbol_shorthand::B(keyToDelete);
  //
  //    // Values
  //    globalGraphValuesBufferPtr_->erase(xKeyToFilter);
  //    globalGraphValuesBufferPtr_->erase(vKeyToFilter);
  //    globalGraphValuesBufferPtr_->erase(bKeyToFilter);
  //    // KeyTimeMaps
  //    globalGraphKeysTimestampsMapBufferPtr_->erase(xKeyToFilter);
  //    globalGraphKeysTimestampsMapBufferPtr_->erase(vKeyToFilter);
  //    globalGraphKeysTimestampsMapBufferPtr_->erase(bKeyToFilter);
  //    // Factors
  //    if (graphConfigPtr_->verboseLevel > 4) {
  //      std::cout << YELLOW_START << "GMsf-GraphManager" << COLOR_END << " All keys before and with human readable key "
  //                << xKeyToFilter.index() << " have to be removed." << std::endl;
  //      std::cout << YELLOW_START << "GMsf-GraphManager" << COLOR_END
  //                << " Number of Factors before erasing:  " << globalFactorsBufferPtr_->size() << std::endl;
  //    }
  //    // Iterate through factors
  //    gtsam::Symbol thisKey;  // pre-allocate for speed
  //    for (auto factorIterator = globalFactorsBufferPtr_->begin(); factorIterator != globalFactorsBufferPtr_->end(); ++factorIterator) {
  //      // Iterate through keys of factor, usually 1 or 2
  //      for (int i = 0; i < (*factorIterator)->keys().size(); ++i) {
  //        thisKey = (*factorIterator)->keys()[i];
  //        if (thisKey.index() <= keyToDelete) {
  //          globalFactorsBufferPtr_->erase(factorIterator);
  //          if (graphConfigPtr_->verboseLevel > 3) {
  //            std::cout << YELLOW_START << "GMsf-GraphManager" << COLOR_END << GREEN_START << " Erased factor for key number " <<
  //            COLOR_END
  //                      << thisKey.index() << std::endl;
  //          }
  //        }
  //      }
  //    }
  //    if (graphConfigPtr_->verboseLevel > 4) {
  //      std::cout << YELLOW_START << "GMsf-GraphManager" << COLOR_END
  //                << " Number of Factors after erasing:  " << globalFactorsBufferPtr_->size() << std::endl;
  //    }
  //  }

  //  // Relocalization Command
  //  if (!sentRelocalizationCommandAlready_ && numOptimizationsSinceGraphSwitching_ >= 1) {
  //    relocalizationFlag = true;
  //    sentRelocalizationCommandAlready_ = true;
  //    std::cout << YELLOW_START << "GMsf-GraphManager" << GREEN_START << " Triggering re-localization." << COLOR_END << std::endl;
  //  } else {
  //    relocalizationFlag = false;
  //  }

  // Return copy of propagated state (for publishing)
  return imuPropagatedState_;
}

gtsam::Key GraphManager::addPoseBetweenFactor(const double lidarTimeKm1, const double lidarTimeK, const double rate,
                                              const Eigen::Matrix<double, 6, 1>& poseBetweenNoise, const gtsam::Pose3& deltaPose,
                                              const std::string& measurementType) {
  // Find corresponding keys in graph
  // Find corresponding keys in graph
  double maxLidarTimestampDistance = 1.0 / rate + 2.0 * graphConfigPtr_->maxSearchDeviation;
  gtsam::Key closestKeyKm1, closestKeyK;
  double keyTimeStampDistance;

  if (!findGraphKeys_(closestKeyKm1, closestKeyK, keyTimeStampDistance, maxLidarTimestampDistance, lidarTimeKm1, lidarTimeK,
                      "lidar delta")) {
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
  assert(poseBetweenNoise.size() == 6);
  auto noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << poseBetweenNoise(0), poseBetweenNoise(1), poseBetweenNoise(2),
                                                    poseBetweenNoise(3), poseBetweenNoise(4), poseBetweenNoise(5))
                                                       .finished());  // rad,rad,rad,m,m,m
  // Regular error function with noise
  // auto errorFunction = noise;
  // Robust error function
  auto errorFunction = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.3), noise);

  // Create pose between factor and add it
  gtsam::BetweenFactor<gtsam::Pose3> poseBetweenFactor(gtsam::symbol_shorthand::X(closestKeyKm1), gtsam::symbol_shorthand::X(closestKeyK),
                                                       scaledDeltaPose, errorFunction);

  // Write to graph
  bool success =
      addFactorSafelyToGraph_<const gtsam::BetweenFactor<gtsam::Pose3>*>(factorGraphBufferPtr_, &poseBetweenFactor, lidarTimeKm1);

  // Print summary
  if (!success) {
    std::cout << YELLOW_START << "GMsf-GraphManager" << COLOR_END << " Current propagated key: " << propagatedStateKey_ << ", "
              << YELLOW_START << measurementType << " PoseBetween factor NOT added between key " << closestKeyKm1 << " and key "
              << closestKeyK << COLOR_END << std::endl;
  } else if (graphConfigPtr_->verboseLevel > 1) {
    std::cout << YELLOW_START << "GMsf-GraphManager" << COLOR_END << " Current propagated key: " << propagatedStateKey_ << ", "
              << GREEN_START << measurementType << " PoseBetween factor added between key " << closestKeyKm1 << " and key " << closestKeyK
              << COLOR_END << std::endl;
  }

  return closestKeyK;
}

void GraphManager::addPoseUnaryFactor(const double timeK, const double rate, const Eigen::Matrix<double, 6, 1>& poseUnaryNoise,
                                      const gtsam::Pose3& T_W_I, const std::string& measurementType) {
  // Find closest key in existing graph
  double closestGraphTime;
  gtsam::Key closestKey;

  if (!timeToKeyBufferPtr_->getClosestKeyAndTimestamp(closestGraphTime, closestKey, measurementType, graphConfigPtr_->maxSearchDeviation,
                                                      timeK)) {
    std::cerr << YELLOW_START << "GMsf-GraphManager" << RED_START << " Not adding " << measurementType << " constraint to graph."
              << COLOR_END << std::endl;
    return;
  }

  assert(poseUnaryNoise.size() == 6);
  auto noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector(poseUnaryNoise));  // rad,rad,rad,x,y,z
  auto errorFunction = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.5), noise);

  // Unary factor
  gtsam::PriorFactor<gtsam::Pose3> poseUnaryFactor(gtsam::symbol_shorthand::X(closestKey), T_W_I, errorFunction);

  // Write to graph
  bool success = addFactorSafelyToGraph_<const gtsam::PriorFactor<gtsam::Pose3>*>(factorGraphBufferPtr_, &poseUnaryFactor, timeK);

  // Print summary
  if (!success) {
    std::cout << YELLOW_START << "GMsf-GraphManager" << COLOR_END << " Current propagated key " << propagatedStateKey_ << YELLOW_START
              << measurementType << " factor NOT added to key " << closestKey << COLOR_END << std::endl;
  } else if (graphConfigPtr_->verboseLevel > 0) {
    std::cout << YELLOW_START << "GMsf-GraphManager" << COLOR_END << " Current propagated key " << propagatedStateKey_ << GREEN_START
              << measurementType << " factor added to key " << closestKey << COLOR_END << std::endl;
  }
}

void GraphManager::addVelocityUnaryFactor(const double timeK, const double rate, const Eigen::Matrix<double, 3, 1>& velocityUnaryNoise,
                                          const gtsam::Vector3& velocity, const std::string& measurementType) {
  // Find closest key in existing graph
  double closestGraphTime;
  gtsam::Key closestKey;

  if (!timeToKeyBufferPtr_->getClosestKeyAndTimestamp(closestGraphTime, closestKey, measurementType, graphConfigPtr_->maxSearchDeviation,
                                                      timeK)) {
    std::cerr << YELLOW_START << "GMsf-GraphManager" << RED_START << " Not adding " << measurementType << " constraint to graph."
              << COLOR_END << std::endl;
    return;
  }

  assert(velocityUnaryNoise.size() == 3);
  auto noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector(velocityUnaryNoise));  // rad,rad,rad,x,y,z
  auto errorFunction = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.5), noise);

  gtsam::PriorFactor<gtsam::Vector3> velocityUnaryFactor(gtsam::symbol_shorthand::V(closestKey), gtsam::Vector3::Zero(), errorFunction);

  /// Add Zero Velocity Factor
  bool success = addFactorSafelyToGraph_<const gtsam::PriorFactor<gtsam::Vector3>*>(factorGraphBufferPtr_, &velocityUnaryFactor, timeK);

  // Print summary
  if (!success) {
    std::cout << YELLOW_START << "GMsf-GraphManager" << COLOR_END << " Current propagated key " << propagatedStateKey_ << YELLOW_START
              << measurementType << " factor NOT added to key " << closestKey << COLOR_END << std::endl;
  } else if (graphConfigPtr_->verboseLevel > 0) {
    std::cout << YELLOW_START << "GMsf-GraphManager" << COLOR_END << " Current propagated key " << propagatedStateKey_ << GREEN_START
              << measurementType << " factor added to key " << closestKey << COLOR_END << std::endl;
  }
}

void GraphManager::addGnssPositionUnaryFactor(double gnssTimeK, const double rate, const Eigen::Vector3d& gnssPositionUnaryNoise,
                                              const gtsam::Vector3& position) {
  // Find closest key in existing graph
  double closestGraphTime;
  gtsam::Key closestKey;
  if (!timeToKeyBufferPtr_->getClosestKeyAndTimestamp(closestGraphTime, closestKey, "GnssUnary", graphConfigPtr_->maxSearchDeviation,
                                                      gnssTimeK)) {
    // TODO
  }

  // Create noise model
  auto noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(3) << gnssPositionUnaryNoise[0], gnssPositionUnaryNoise[1], gnssPositionUnaryNoise[2])
          .finished());  // rad,rad,rad,m,m,m
  auto tukeyErrorFunction = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(0.7), noise);

  // Create unary factor and add it
  gtsam::GPSFactor gnssPositionUnaryFactor(gtsam::symbol_shorthand::X(closestKey), position, tukeyErrorFunction);

  // Write to graph
  bool success = addFactorSafelyToGraph_<const gtsam::GPSFactor*>(factorGraphBufferPtr_, &gnssPositionUnaryFactor, gnssTimeK);

  // Print summary
  if (!success) {
    std::cout << YELLOW_START << "GMsf-GraphManager" << COLOR_END << " Current propagated key " << propagatedStateKey_ << YELLOW_START
              << ", Gnss position factor NOT added to key " << closestKey << COLOR_END << std::endl;
  } else if (graphConfigPtr_->verboseLevel > 1) {
    std::cout << YELLOW_START << "GMsf-GraphManager" << COLOR_END << " Current propagated key " << propagatedStateKey_ << GREEN_START
              << ", Gnss position factor added to key " << closestKey << COLOR_END << std::endl;
  }
}

void GraphManager::addGnssHeadingUnaryFactor(double gnssTimeK, const double rate, const Eigen::Matrix<double, 1, 1>& gnssHeadingUnaryNoise,
                                             const double measuredYaw) {
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

  // Create noise model
  //  auto gnssHeadingUnaryNoise =
  //      gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(1) << gnssHeadingUnaryNoise_).finished());  // rad,rad,rad,m,m,m
  auto noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(1) << gnssHeadingUnaryNoise).finished());  // rad,rad,rad,m,m,m
  auto tukeyErrorFunction = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(0.5), noise);

  HeadingFactor gnssHeadingUnaryFactor(gtsam::symbol_shorthand::X(closestKey), measuredYaw, tukeyErrorFunction);

  // Write to graph
  bool success = addFactorSafelyToGraph_<const HeadingFactor*>(factorGraphBufferPtr_, &gnssHeadingUnaryFactor, gnssTimeK);

  // Print summary
  if (!success) {
    std::cout << YELLOW_START << "GMsf-GraphManager" << COLOR_END << " Current propagated key " << propagatedStateKey_ << YELLOW_START
              << " Gnss heading factor is NOT added to key " << closestKey << COLOR_END << std::endl;
  } else if (graphConfigPtr_->verboseLevel > 0) {
    std::cout << YELLOW_START << "GMsf-GraphManager" << COLOR_END << " Current propagated key " << propagatedStateKey_ << GREEN_START
              << " Gnss heading factor is added to key " << closestKey << COLOR_END << std::endl;
  }
}

gtsam::NavState GraphManager::calculateNavStateAtKey(bool& computeSuccessfulFlag,
                                                     const std::shared_ptr<gtsam::IncrementalFixedLagSmoother> graphPtr,
                                                     const std::shared_ptr<GraphConfig>& graphConfigPtr, const gtsam::Key& key,
                                                     const char* callingFunctionName) {
  gtsam::Pose3 resultPose;
  gtsam::Vector3 resultVelocity;
  if (true) {
    try {
      resultPose = graphPtr->calculateEstimate<gtsam::Pose3>(gtsam::symbol_shorthand::X(key));  // auto result = mainGraphPtr_->estimate();
      resultVelocity = graphPtr->calculateEstimate<gtsam::Vector3>(gtsam::symbol_shorthand::V(key));
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

SafeNavStateWithCovarianceAndBias GraphManager::updateGraphAndGetState(double& currentPropagatedTime) {
  // Method variables
  gtsam::NonlinearFactorGraph newGraphFactors;
  gtsam::Values newGraphValues;
  std::map<gtsam::Key, double> newGraphKeysTimestampsMap;
  gtsam::Key currentPropagatedKey;

  // Mutex Block 1 -----------------
  {
    // Lock
    const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
    // Get copy of factors and values
    newGraphFactors = *factorGraphBufferPtr_;
    newGraphValues = *graphValuesBufferPtr_;
    newGraphKeysTimestampsMap = *graphKeysTimestampsMapBufferPtr_;
    // Empty buffers
    factorGraphBufferPtr_->resize(0);
    graphValuesBufferPtr_->clear();
    graphKeysTimestampsMapBufferPtr_->clear();

    // Empty Buffer Pre-integrator --> everything missed during the update will be in here
    if (graphConfigPtr_->usingBiasForPreIntegrationFlag) {
      imuBufferPreintegratorPtr_->resetIntegrationAndSetBias(optimizedGraphState_.imuBias());
    } else {
      imuBufferPreintegratorPtr_->resetIntegrationAndSetBias(gtsam::imuBias::ConstantBias());
    }
    // Get current key and time
    currentPropagatedKey = propagatedStateKey_;
    currentPropagatedTime = propagatedStateTime_;
  }  // end of locking

  // Graph Update (time consuming) -------------------
  addFactorsToSmootherAndOptimize(fixedLagSmootherPtr_, newGraphFactors, newGraphValues, newGraphKeysTimestampsMap, graphConfigPtr_,
                                  graphConfigPtr_->additionalOptimizationIterations);

  // Compute entire result
  // NavState
  bool computeSuccessfulFlag = true;
  gtsam::NavState resultNavState =
      calculateNavStateAtKey(computeSuccessfulFlag, fixedLagSmootherPtr_, graphConfigPtr_, currentPropagatedKey, __func__);
  // Bias
  gtsam::imuBias::ConstantBias resultBias =
      fixedLagSmootherPtr_->calculateEstimate<gtsam::imuBias::ConstantBias>(gtsam::symbol_shorthand::B(currentPropagatedKey));
  // Compute Covariance
  gtsam::Matrix66 poseCovariance = fixedLagSmootherPtr_->marginalCovariance(gtsam::symbol_shorthand::X(currentPropagatedKey));
  gtsam::Matrix33 velocityCovariance = fixedLagSmootherPtr_->marginalCovariance(gtsam::symbol_shorthand::V(currentPropagatedKey));

  // Mutex block 2 ------------------
  {
    // Lock
    const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
    // Update Graph State
    optimizedGraphState_.updateNavStateAndBias(currentPropagatedKey, currentPropagatedTime, resultNavState, resultBias);
    // Predict from solution to obtain refined propagated state
    // Resetting is done at beginning of next optimization
    if (graphConfigPtr_->usingBiasForPreIntegrationFlag) {
      imuPropagatedState_ = imuBufferPreintegratorPtr_->predict(resultNavState, optimizedGraphState_.imuBias());
    } else {
      imuPropagatedState_ = imuBufferPreintegratorPtr_->predict(resultNavState, gtsam::imuBias::ConstantBias());
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
                                                   const std::shared_ptr<GraphConfig>& graphConfigPtr, const int additionalIterations) {
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

gtsam::NavState GraphManager::calculateStateAtKey(bool& computeSuccessfulFlag, const gtsam::Key& key) {
  return calculateNavStateAtKey(computeSuccessfulFlag, fixedLagSmootherPtr_, graphConfigPtr_, key, __func__);
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
  if (timeToKeyBufferPtr_->getLatestTimestampInBuffer() - measurementTimestamp >
      graphConfigPtr_->smootherLag - WORST_CASE_OPTIMIZATION_TIME) {
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

bool GraphManager::findGraphKeys_(gtsam::Key& closestKeyKm1, gtsam::Key& closestKeyK, double& keyTimeStampDistance,
                                  const double maxTimestampDistance, const double timeKm1, const double timeK, const std::string& name) {
  // Find closest lidar keys in existing graph
  double closestGraphTimeKm1, closestGraphTimeK;
  {
    // Looking up from IMU buffer --> acquire mutex (otherwise values for key might not be set)
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

void GraphManager::writeValueKeysToKeyTimeStampMap_(const gtsam::Values& values, const double measurementTime,
                                                    std::shared_ptr<std::map<gtsam::Key, double>> keyTimestampMapPtr) {
  for (const auto& value : values) {
    (*keyTimestampMapPtr)[value.key] = measurementTime;
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

  // Start integrating with imu_meas.begin()+1 meas to calculate dt, imu_meas.begin() meas was integrated before
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