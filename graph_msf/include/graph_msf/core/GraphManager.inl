/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#define WORST_CASE_OPTIMIZATION_TIME 0.1  // in seconds

namespace graph_msf {
// Template Implementations

// 1) Unary meta method --> classic GTSAM Factors ----------------------------------------
typedef gtsam::Key (*F)(std::uint64_t);
template <class MEASUREMENT_TYPE, int NOISE_DIM, class FACTOR_TYPE, F SYMBOL_SHORTHAND>
void GraphManager::addUnaryFactorInImuFrame(const MEASUREMENT_TYPE& unaryMeasurement,
                                            const Eigen::Matrix<double, NOISE_DIM, 1>& unaryNoiseDensity, const double measurementTime) {
  // Find the closest key in existing graph
  double closestGraphTime;
  gtsam::Key closestKey;
  std::string callingName = "GnssPositionUnaryFactor";
  if (!timeToKeyBufferPtr_->getClosestKeyAndTimestamp(closestGraphTime, closestKey, callingName, graphConfigPtr_->maxSearchDeviation_,
                                                      measurementTime)) {
    if (propagatedStateTime_ - measurementTime < 0.0) {  // Factor is coming from the future, hence add it to the buffer and adding it later
      // TODO: Add to buffer and return --> still add it until we are there
    } else {  // Otherwise do not add it
      REGULAR_COUT << RED_START << " Time deviation of " << typeid(FACTOR_TYPE).name() << " at key " << closestKey << " is "
                   << 1000 * std::abs(closestGraphTime - measurementTime) << " ms, being larger than admissible deviation of "
                   << 1000 * graphConfigPtr_->maxSearchDeviation_ << " ms. Not adding to graph." << COLOR_END << std::endl;
      return;
    }
  }

  // Create noise model
  auto noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(unaryNoiseDensity)));  // m,m,m
  auto robustErrorFunction = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.345), noise);

  // Create unary factor and ADD IT
  std::shared_ptr<FACTOR_TYPE> unaryFactorPtr;
  // Case 1: Expression factor --> must be handled differently
  if constexpr (std::is_same<gtsam::ExpressionFactor<MEASUREMENT_TYPE>, FACTOR_TYPE>::value) {
  } else {  // Case 2: No expression factor
    unaryFactorPtr = std::make_shared<FACTOR_TYPE>(SYMBOL_SHORTHAND(closestKey), unaryMeasurement, noise);
    // Write to graph
    addFactorSafelyToRtAndBatchGraph_<const FACTOR_TYPE*>(unaryFactorPtr.get(), measurementTime);
  }

  // Print summary
  if (graphConfigPtr_->verboseLevel_ > 1) {
    REGULAR_COUT << " Current propagated key " << propagatedStateKey_ << GREEN_START << ", " << typeid(FACTOR_TYPE).name()
                 << " factor added to key " << closestKey << COLOR_END << std::endl;
  }
}

// 2) GMSF Holistic Graph Factors with Extrinsic Calibration ------------------------
template <class GMSF_EXPRESSION_TYPE>  // e.g. GmsfUnaryExpressionAbsolutePose3
void GraphManager::addUnaryGmsfExpressionFactor(const std::shared_ptr<GMSF_EXPRESSION_TYPE> gmsfUnaryExpressionPtr) {
  // Measurement
  const auto& unaryMeasurement = *gmsfUnaryExpressionPtr->getGmsfBaseUnaryMeasurementPtr();

  // Get corresponding key of robot state in graph
  gtsam::Key closestGeneralKey;
  double closestGeneralKeyTime;
  if (!getUnaryFactorGeneralKey(closestGeneralKey, closestGeneralKeyTime, unaryMeasurement)) {
    return;
  }

  // Create Expression --> exact type of expression is determined by the template
  const auto gmsfGtsamExpression = gmsfUnaryExpressionPtr->createAndReturnExpression(
      closestGeneralKey, gtsamTransformsExpressionKeys_, W_imuPropagatedState_, graphConfigPtr_->optimizeReferenceFramePosesWrtWorld_,
      graphConfigPtr_->centerReferenceFramesAtRobotPositionBeforeAlignment_,
      graphConfigPtr_->optimizeExtrinsicSensorToSensorCorrectedOffset_);

  // Factor
  std::shared_ptr<gtsam::ExpressionFactor<typename GMSF_EXPRESSION_TYPE::template_type>> unaryExpressionFactorPtr;

  // Noise & Error Function
  auto noiseModel = gtsam::noiseModel::Diagonal::Sigmas(gmsfUnaryExpressionPtr->getNoiseDensity());  // rad,rad,rad,x,y,z
  // Robust Error Function?
  const RobustNormEnum robustNormEnum(gmsfUnaryExpressionPtr->getGmsfBaseUnaryMeasurementPtr()->robustNormEnum());
  const double robustNormConstant = gmsfUnaryExpressionPtr->getGmsfBaseUnaryMeasurementPtr()->robustNormConstant();
  boost::shared_ptr<gtsam::noiseModel::Robust> robustErrorFunction;
  // Pick Robust Error Function
  switch (robustNormEnum) {
    case RobustNormEnum::Huber:
      robustErrorFunction = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(robustNormConstant), noiseModel);
      break;
    case RobustNormEnum::Cauchy:
      robustErrorFunction =
          gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Cauchy::Create(robustNormConstant), noiseModel);
      break;
    case RobustNormEnum::Tukey:
      robustErrorFunction = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Tukey::Create(robustNormConstant), noiseModel);
      break;
    case RobustNormEnum::None:
      unaryExpressionFactorPtr = std::make_shared<gtsam::ExpressionFactor<typename GMSF_EXPRESSION_TYPE::template_type>>(
          noiseModel, gmsfUnaryExpressionPtr->getGtsamMeasurementValue(), gmsfGtsamExpression);
      break;
  }

  // Create Factor
  if (robustNormEnum != RobustNormEnum::None) {
    unaryExpressionFactorPtr = std::make_shared<gtsam::ExpressionFactor<typename GMSF_EXPRESSION_TYPE::template_type>>(
        robustErrorFunction, gmsfUnaryExpressionPtr->getGtsamMeasurementValue(), gmsfGtsamExpression);
  }

  // Operating on graph data
  {
    const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
    // Main expression factor: add to graph
    const bool success = addFactorToRtAndBatchGraph_<const gtsam::ExpressionFactor<typename GMSF_EXPRESSION_TYPE::template_type>*>(
        unaryExpressionFactorPtr.get(), gmsfUnaryExpressionPtr->getTimestamp(), "GMSF-Expression");

    // If successful
    if (success) {
      // A. Write to timestamp map for fixed lag smoother if newer than existing one for all keys that are in expression factor
      for (const gtsam::Key& key : unaryExpressionFactorPtr->keys()) {
        // a) Rt Graph
        // Find timestamp in existing buffer and update: if i) not existent or ii) newer than existing one -> write
        auto rtKeyTimestampMapIterator = rtGraphKeysTimestampsMapBufferPtr_->find(key);
        // i) Not existent
        if (rtKeyTimestampMapIterator == rtGraphKeysTimestampsMapBufferPtr_->end()) {
          // TODO: Check if it should not be propagatedStateTime_ instead of gmsfUnaryExpressionPtr->getTimestamp()
          writeKeyToKeyTimeStampMap_(key, gmsfUnaryExpressionPtr->getTimestamp(), rtGraphKeysTimestampsMapBufferPtr_);
        }
        // ii) If timestamp is newer than existing one, write it --> dangerous, as it can get states out of order
        else if (gmsfUnaryExpressionPtr->getTimestamp() > rtKeyTimestampMapIterator->second) {
          writeKeyToKeyTimeStampMap_(key, gmsfUnaryExpressionPtr->getTimestamp(), rtGraphKeysTimestampsMapBufferPtr_);
        }
        // b) Batch Graph
        if (graphConfigPtr_->useAdditionalSlowBatchSmoother_) {
          // Find timestamp in existing buffer and update: if i) not existent or ii) newer than existing one -> write
          auto batchKeyTimestampMapIterator = batchGraphKeysTimestampsMapBufferPtr_->find(key);
          // i) Not existent
          if (batchKeyTimestampMapIterator == batchGraphKeysTimestampsMapBufferPtr_->end()) {
            writeKeyToKeyTimeStampMap_(key, gmsfUnaryExpressionPtr->getTimestamp(), batchGraphKeysTimestampsMapBufferPtr_);
          }
          // ii) If timestamp is newer than existing one, write it --> dangerous, as it can get states out of order
          else if (gmsfUnaryExpressionPtr->getTimestamp() > batchKeyTimestampMapIterator->second) {
            writeKeyToKeyTimeStampMap_(key, gmsfUnaryExpressionPtr->getTimestamp(), batchGraphKeysTimestampsMapBufferPtr_);
          }
        }
      }
      // B. If one of the states was newly created, then add it to the values
      if (!gmsfUnaryExpressionPtr->getNewOnlineGraphStateValues().empty()) {
        rtGraphValuesBufferPtr_->insert(gmsfUnaryExpressionPtr->getNewOnlineGraphStateValues());
      }
      if (!gmsfUnaryExpressionPtr->getNewOfflineGraphStateValues().empty()) {
        batchGraphValuesBufferPtr_->insert(gmsfUnaryExpressionPtr->getNewOfflineGraphStateValues());
      }
      // C. If new factors are there (due to newly generated factor or for regularization), add it to the graph
      if (!gmsfUnaryExpressionPtr->getNewOnlinePosePriorFactors().empty()) {
        // Prior factors --> only needed for online graph, as observable for offline graph
        rtFactorGraphBufferPtr_->add(gmsfUnaryExpressionPtr->getNewOnlinePosePriorFactors());
      }
      if (!gmsfUnaryExpressionPtr->getNewOnlineDynamicPriorFactors().empty()) {
        // Dynamic prior factors --> only needed for online graph, as observable for offline graph
        rtFactorGraphBufferPtr_->add(gmsfUnaryExpressionPtr->getNewOnlineDynamicPriorFactors());
      }
      if (!gmsfUnaryExpressionPtr->getNewOnlineAndOfflinePoseBetweenFactors().empty()) {
        // Between factors --> needed for both online and offline graph to model random walk
        rtFactorGraphBufferPtr_->add(gmsfUnaryExpressionPtr->getNewOnlineAndOfflinePoseBetweenFactors());
        batchFactorGraphBufferPtr_->add(gmsfUnaryExpressionPtr->getNewOnlineAndOfflinePoseBetweenFactors());
      }
    }
  }

  // Print summary --------------------------------------
  if (graphConfigPtr_->verboseLevel_ > 0) {
    REGULAR_COUT << " Current propagated key " << propagatedStateKey_ << GREEN_START << ", expression factor of type "
                 << typeid(GMSF_EXPRESSION_TYPE).name() << " added to keys ";
    for (const auto& key : unaryExpressionFactorPtr->keys()) {
      std::cout << gtsam::Symbol(key) << ", ";
    }
    std::cout << COLOR_END << std::endl;
  }
}

// Private -----------------------------------------------------------------------------------------
template <class CHILDPTR>
bool GraphManager::addFactorToRtAndBatchGraph_(const gtsam::NoiseModelFactor* noiseModelFactorPtr) {
  rtFactorGraphBufferPtr_->add(*dynamic_cast<CHILDPTR>(noiseModelFactorPtr));
  if (graphConfigPtr_->useAdditionalSlowBatchSmoother_) {
    batchFactorGraphBufferPtr_->add(*dynamic_cast<CHILDPTR>(noiseModelFactorPtr));
  }
  return true;
}

template <class CHILDPTR>
bool GraphManager::addFactorToRtAndBatchGraph_(const gtsam::NoiseModelFactor* noiseModelFactorPtr, const double measurementTimestamp,
                                               const std::string& measurementName) {
  // Check Timestamp of Measurement on Delay
  if (timeToKeyBufferPtr_->getLatestTimestampInBuffer() - measurementTimestamp >
      (graphConfigPtr_->realTimeSmootherLag_ - WORST_CASE_OPTIMIZATION_TIME)) {
    REGULAR_COUT << RED_START << " " << measurementName
                 << "-measurement delay is larger than the smootherLag - WORST_CASE_OPTIMIZATION_TIME, hence skipping this measurement."
                 << COLOR_END << std::endl;
    REGULAR_COUT << " Current propagated key " << propagatedStateKey_ << ", measurement time " << measurementTimestamp
                 << ", latest time in buffer " << timeToKeyBufferPtr_->getLatestTimestampInBuffer()
                 << ", delay: " << timeToKeyBufferPtr_->getLatestTimestampInBuffer() - measurementTimestamp << "s." << std::endl;
    return false;
  }
  // Add measurements
  return addFactorToRtAndBatchGraph_<CHILDPTR>(noiseModelFactorPtr);
}

template <class CHILDPTR>
bool GraphManager::addFactorSafelyToRtAndBatchGraph_(const gtsam::NoiseModelFactor* noiseModelFactorPtr,
                                                     const double measurementTimestamp) {
  // Operating on graph data --> acquire mutex
  const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
  // Add measurements
  return addFactorToRtAndBatchGraph_<CHILDPTR>(noiseModelFactorPtr, measurementTimestamp, "safe");
}

void GraphManager::writeKeyToKeyTimeStampMap_(const gtsam::Key& key, const double measurementTime,
                                              std::shared_ptr<std::map<gtsam::Key, double>> keyTimestampMapPtr) {
  (*keyTimestampMapPtr)[key] = measurementTime;
}

}  // namespace graph_msf
