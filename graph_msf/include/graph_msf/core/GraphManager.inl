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
/**
 * @brief Add a unary GMSF expression factor to the graph.
 *
 * @param gmsfUnaryExpressionPtr Pointer to the unary GMSF expression (can be found in core/factors/gmsf_expression).
 *
 * @tparam GMSF_EXPRESSION_TYPE Type of the GMSF expression (e.g. GmsfUnaryExpressionAbsolutePose3).
 */
template <class GMSF_EXPRESSION_TYPE>  // e.g. GmsfUnaryExpressionAbsolutePose3
void GraphManager::addUnaryGmsfExpressionFactor(const std::shared_ptr<GMSF_EXPRESSION_TYPE> gmsfUnaryExpressionPtr,
                                                const bool addToOnlineSmootherFlag) {
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
      closestGeneralKey, gtsamDynamicExpressionKeys_, W_imuPropagatedState_, graphConfigPtr_->optimizeReferenceFramePosesWrtWorldFlag_,
      graphConfigPtr_->centerMeasurementsAtKeyframePositionBeforeAlignmentFlag_,
      graphConfigPtr_->optimizeExtrinsicSensorToSensorCorrectedOffsetFlag_);

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
    // A. Main expression factor: add to graph ---------------------------------------------------------------------------------------------
    const bool success = addFactorToRtAndBatchGraph_<const gtsam::ExpressionFactor<typename GMSF_EXPRESSION_TYPE::template_type>*>(
        unaryExpressionFactorPtr.get(), gmsfUnaryExpressionPtr->getTimestamp(), "GMSF-Expression", addToOnlineSmootherFlag);

    // If successful
    if (success) {
      // B. Write to timestamp map for fixed lag smoother if newer than existing ------------
      // B.a) Check keys in expression factor and write to timestamp map
      for (const gtsam::Key& key : unaryExpressionFactorPtr->keys()) {
        // 1) Rt Graph
        if (addToOnlineSmootherFlag) {
          // Find timestamp in existing buffer and update: if i) not existent or ii) newer than existing one -> write
          auto rtKeyTimestampMapIterator = rtGraphKeysTimestampsMapBufferPtr_->find(key);
          // i) Not existent or ii) If timestamp is newer than existing one
          if (rtKeyTimestampMapIterator == rtGraphKeysTimestampsMapBufferPtr_->end() ||
              gmsfUnaryExpressionPtr->getTimestamp() > rtKeyTimestampMapIterator->second) {
            writeKeyToKeyTimeStampMap_(key, gmsfUnaryExpressionPtr->getTimestamp(), rtGraphKeysTimestampsMapBufferPtr_);
          }
        }
        // 2) Batch Graph (TODO: Check if this is needed, as batch does not do marginalization)
        if (graphConfigPtr_->useAdditionalSlowBatchSmootherFlag_) {
          // Find timestamp in existing buffer and update: if i) not existent or ii) newer than existing one -> write
          auto batchKeyTimestampMapIterator = batchGraphKeysTimestampsMapBufferPtr_->find(key);
          // i) Not existent or ii) If timestamp is newer than existing one
          if (batchKeyTimestampMapIterator == batchGraphKeysTimestampsMapBufferPtr_->end() ||
              gmsfUnaryExpressionPtr->getTimestamp() > batchKeyTimestampMapIterator->second) {
            writeKeyToKeyTimeStampMap_(key, gmsfUnaryExpressionPtr->getTimestamp(), batchGraphKeysTimestampsMapBufferPtr_);
          }
        }
      }
      // B.b) Check keys in online graph values and write to timestamp map (can be that inactive keys have been activated, which are not in
      // expression factor)
      // 1) Rt Graph
      if (addToOnlineSmootherFlag) {
        for (const auto& key : gmsfUnaryExpressionPtr->getNewOnlineGraphStateValues().keys()) {
          // Find timestamp in existing buffer and update: if i) not existent or ii) newer than existing one -> write
          auto rtKeyTimestampMapIterator = rtGraphKeysTimestampsMapBufferPtr_->find(key);
          // i) Not existent or ii) If timestamp is newer than existing one
          if (rtKeyTimestampMapIterator == rtGraphKeysTimestampsMapBufferPtr_->end() ||
              gmsfUnaryExpressionPtr->getTimestamp() > rtKeyTimestampMapIterator->second) {
            writeKeyToKeyTimeStampMap_(key, gmsfUnaryExpressionPtr->getTimestamp(), rtGraphKeysTimestampsMapBufferPtr_);
          }
        }
      }
      // 2) Batch Graph --> not needed, as these states are only added to online graph
      // C. If one of the states was newly created, then add it to the values buffer -------------------------------------------------------
      // a) Rt Graph
      if (!gmsfUnaryExpressionPtr->getNewOnlineGraphStateValues().empty() && addToOnlineSmootherFlag) {
        rtGraphValuesBufferPtr_->insert(gmsfUnaryExpressionPtr->getNewOnlineGraphStateValues());
      }
      // b) Batch Graph
      if (!gmsfUnaryExpressionPtr->getNewOfflineGraphStateValues().empty()) {
        batchGraphValuesBufferPtr_->insert(gmsfUnaryExpressionPtr->getNewOfflineGraphStateValues());
      }
      // C. If new factors are there (due to newly generated factor or for regularization), add it to the graph ----------------------------
      // a) Rt Graph
      if (!gmsfUnaryExpressionPtr->getNewOnlinePosePriorFactors().empty() && addToOnlineSmootherFlag) {
        // Prior factors --> only needed for online graph, as observable for offline graph
        rtFactorGraphBufferPtr_->add(gmsfUnaryExpressionPtr->getNewOnlinePosePriorFactors());
      }
      if (!gmsfUnaryExpressionPtr->getNewOnlineDynamicPriorFactors().empty() && addToOnlineSmootherFlag) {
        // Dynamic prior factors --> only needed for online graph, as observable for offline graph
        rtFactorGraphBufferPtr_->add(gmsfUnaryExpressionPtr->getNewOnlineDynamicPriorFactors());
      }
      // Both
      if (!gmsfUnaryExpressionPtr->getNewOnlineAndOfflinePoseBetweenFactors().empty()) {
        // Between factors --> needed for both online and offline graph to model random walk
        if (addToOnlineSmootherFlag) {
          rtFactorGraphBufferPtr_->add(gmsfUnaryExpressionPtr->getNewOnlineAndOfflinePoseBetweenFactors());
        }
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
bool GraphManager::addFactorToRtAndBatchGraph_(const gtsam::NoiseModelFactor* noiseModelFactorPtr, const bool addToOnlineSmootherFlag) {
  // Add to real-time graph
  if (addToOnlineSmootherFlag) {
    rtFactorGraphBufferPtr_->add(*dynamic_cast<CHILDPTR>(noiseModelFactorPtr));
  }

  // Add to batch graph
  if (graphConfigPtr_->useAdditionalSlowBatchSmootherFlag_) {
    batchFactorGraphBufferPtr_->add(*dynamic_cast<CHILDPTR>(noiseModelFactorPtr));
  }
  return true;
}

template <class CHILDPTR>
bool GraphManager::addFactorToRtAndBatchGraph_(const gtsam::NoiseModelFactor* noiseModelFactorPtr, const double measurementTimestamp,
                                               const std::string& measurementName, const bool addToOnlineSmootherFlag) {
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
  return addFactorToRtAndBatchGraph_<CHILDPTR>(noiseModelFactorPtr, addToOnlineSmootherFlag);
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
