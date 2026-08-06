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
bool GraphManager::addUnaryFactorInImuFrame(const MEASUREMENT_TYPE& unaryMeasurement,
                                            const Eigen::Matrix<double, NOISE_DIM, 1>& unaryNoiseDensity,
                                            const double measurementTime) {
  // Find the closest key in existing graph
  double closestGraphTime;
  gtsam::Key closestKey;
  const std::string callingName = typeid(FACTOR_TYPE).name();
  const UnaryFactorKeyStatus keyStatus =
      getUnaryFactorGeneralKey(closestKey, closestGraphTime, callingName, measurementTime);
  if (keyStatus == UnaryFactorKeyStatus::Future) {
    deferUnaryFactor_(measurementTime, callingName, [this, unaryMeasurement, unaryNoiseDensity, measurementTime]() {
      return addUnaryFactorInImuFrame<MEASUREMENT_TYPE, NOISE_DIM, FACTOR_TYPE, SYMBOL_SHORTHAND>(
          unaryMeasurement, unaryNoiseDensity, measurementTime);
    });
    return false;
  }
  if (keyStatus == UnaryFactorKeyStatus::Rejected) {
    ++unaryFactorsRejected_;
    return false;
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
    const bool success = addFactorSafelyToRtAndBatchGraph_<const FACTOR_TYPE*>(unaryFactorPtr.get(), measurementTime);
    if (!success) {
      ++unaryFactorsRejected_;
      return false;
    }
  }

  ++unaryFactorsAdded_;

  // Print summary
  if (graphConfigPtr_->verboseLevel_ > 1) {
    REGULAR_COUT << " Current propagated key " << propagatedStateKey_ << GREEN_START << ", " << typeid(FACTOR_TYPE).name()
                 << " factor added to key " << closestKey << COLOR_END << std::endl;
  }
  return true;
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
bool GraphManager::addUnaryGmsfExpressionFactor(const std::shared_ptr<GMSF_EXPRESSION_TYPE> gmsfUnaryExpressionPtr,
                                                const bool addToOnlineSmootherFlag) {
  // Measurement
  const auto& unaryMeasurement = *gmsfUnaryExpressionPtr->getGmsfBaseUnaryMeasurementPtr();

  // Get corresponding key of robot state in graph
  gtsam::Key closestGeneralKey;
  double closestGeneralKeyTime;
  const UnaryFactorKeyStatus keyStatus = getUnaryFactorGeneralKey(closestGeneralKey, closestGeneralKeyTime, unaryMeasurement);
  if (keyStatus == UnaryFactorKeyStatus::Future) {
    deferUnaryFactor_(unaryMeasurement.timeK(), unaryMeasurement.measurementName(),
                      [this, gmsfUnaryExpressionPtr, addToOnlineSmootherFlag]() {
                        return addUnaryGmsfExpressionFactor<GMSF_EXPRESSION_TYPE>(gmsfUnaryExpressionPtr, addToOnlineSmootherFlag);
                      });
    return false;
  }
  if (keyStatus == UnaryFactorKeyStatus::Rejected) {
    ++unaryFactorsRejected_;
    return false;
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
  gtsam::noiseModel::Robust::shared_ptr robustErrorFunction;
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
    case RobustNormEnum::GemanMcClure:
      robustErrorFunction =
          gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::GemanMcClure::Create(robustNormConstant), noiseModel);
      break;
    case RobustNormEnum::DCS:
      // robustNormConstant is the DCS parameter c (a.k.a. k in GTSAM)
      robustErrorFunction = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::DCS::Create(robustNormConstant), noiseModel);
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
  bool success = false;
  {
    const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
    // A. Main expression factor: add to graph ---------------------------------------------------------------------------------------------
    success = addFactorToRtAndBatchGraph_<const gtsam::ExpressionFactor<typename GMSF_EXPRESSION_TYPE::template_type>*>(
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

  if (!success) {
    ++unaryFactorsRejected_;
    return false;
  }
  ++unaryFactorsAdded_;

  // Print summary --------------------------------------
  if (graphConfigPtr_->verboseLevel_ >= 2) {
    REGULAR_COUT << " Current propagated key " << propagatedStateKey_ << ": expression factor of type "
                 << typeid(GMSF_EXPRESSION_TYPE).name() << " added to keys ";
    for (const auto& key : unaryExpressionFactorPtr->keys()) {
      std::cout << gtsam::Symbol(key) << ", ";
    }
    std::cout << COLOR_END << std::endl;
  }
  return true;
}

template <class FIRST_GMSF_EXPRESSION_TYPE, class SECOND_GMSF_EXPRESSION_TYPE>
bool GraphManager::addUnaryGmsfExpressionFactorPair(
    const std::shared_ptr<FIRST_GMSF_EXPRESSION_TYPE> firstExpressionPtr,
    const std::shared_ptr<SECOND_GMSF_EXPRESSION_TYPE> secondExpressionPtr, const bool addToOnlineSmootherFlag) {
  if (firstExpressionPtr->getGmsfBaseUnaryMeasurementPtr() != secondExpressionPtr->getGmsfBaseUnaryMeasurementPtr()) {
    throw std::invalid_argument("Grouped GMSF expression factors must share one measurement.");
  }

  using FirstMeasurementType = typename FIRST_GMSF_EXPRESSION_TYPE::template_type;
  using SecondMeasurementType = typename SECOND_GMSF_EXPRESSION_TYPE::template_type;
  const auto& unaryMeasurement = *firstExpressionPtr->getGmsfBaseUnaryMeasurementPtr();

  // Extrinsic calibration can allocate dynamic graph state while an expression is being constructed. Reject that mode before either
  // expression is evaluated so the grouped path inserts both component factors or neither.
  if (graphConfigPtr_->optimizeExtrinsicSensorToSensorCorrectedOffsetFlag_) {
    REGULAR_COUT << YELLOW_START << " Grouped expression factors do not support online extrinsic calibration; not adding either component."
                 << COLOR_END << std::endl;
    ++unaryFactorsRejected_;
    return false;
  }

  // Keep key lookup, delay validation, expression construction, and both graph insertions atomic with respect to graph updates.
  std::unique_lock<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);

  gtsam::Key closestGeneralKey;
  double closestGeneralKeyTime;
  const UnaryFactorKeyStatus keyStatus = getUnaryFactorGeneralKey(closestGeneralKey, closestGeneralKeyTime, unaryMeasurement);
  if (keyStatus == UnaryFactorKeyStatus::Future) {
    operateOnGraphDataLock.unlock();
    deferUnaryFactor_(unaryMeasurement.timeK(), unaryMeasurement.measurementName(),
                      [this, firstExpressionPtr, secondExpressionPtr, addToOnlineSmootherFlag]() {
                        return addUnaryGmsfExpressionFactorPair<FIRST_GMSF_EXPRESSION_TYPE, SECOND_GMSF_EXPRESSION_TYPE>(
                            firstExpressionPtr, secondExpressionPtr, addToOnlineSmootherFlag);
                      });
    return false;
  }
  if (keyStatus == UnaryFactorKeyStatus::Rejected) {
    ++unaryFactorsRejected_;
    return false;
  }

  const double measurementTimestamp = unaryMeasurement.timeK();
  if (timeToKeyBufferPtr_->getLatestTimestampInBuffer() - measurementTimestamp >
      (graphConfigPtr_->realTimeSmootherLag_ - WORST_CASE_OPTIMIZATION_TIME)) {
    REGULAR_COUT << RED_START << " GMSF-Expression-measurement delay is larger than the smootherLag - WORST_CASE_OPTIMIZATION_TIME, hence "
                                   "skipping both grouped factors."
                 << COLOR_END << std::endl;
    ++unaryFactorsRejected_;
    return false;
  }

  const auto firstGtsamExpression = firstExpressionPtr->createAndReturnExpression(
      closestGeneralKey, gtsamDynamicExpressionKeys_, W_imuPropagatedState_, graphConfigPtr_->optimizeReferenceFramePosesWrtWorldFlag_,
      graphConfigPtr_->centerMeasurementsAtKeyframePositionBeforeAlignmentFlag_,
      graphConfigPtr_->optimizeExtrinsicSensorToSensorCorrectedOffsetFlag_);
  const auto secondGtsamExpression = secondExpressionPtr->createAndReturnExpression(
      closestGeneralKey, gtsamDynamicExpressionKeys_, W_imuPropagatedState_, graphConfigPtr_->optimizeReferenceFramePosesWrtWorldFlag_,
      graphConfigPtr_->centerMeasurementsAtKeyframePositionBeforeAlignmentFlag_,
      graphConfigPtr_->optimizeExtrinsicSensorToSensorCorrectedOffsetFlag_);

  const auto createNoiseModel = [](const auto& expressionPtr) {
    const auto diagonalNoise = gtsam::noiseModel::Diagonal::Sigmas(expressionPtr->getNoiseDensity());
    const auto& measurement = *expressionPtr->getGmsfBaseUnaryMeasurementPtr();
    const double robustNormConstant = measurement.robustNormConstant();
    switch (measurement.robustNormEnum()) {
      case RobustNormEnum::None:
        return gtsam::SharedNoiseModel(diagonalNoise);
      case RobustNormEnum::Huber:
        return gtsam::SharedNoiseModel(gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(robustNormConstant), diagonalNoise));
      case RobustNormEnum::Cauchy:
        return gtsam::SharedNoiseModel(gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Cauchy::Create(robustNormConstant), diagonalNoise));
      case RobustNormEnum::Tukey:
        return gtsam::SharedNoiseModel(gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Tukey::Create(robustNormConstant), diagonalNoise));
      case RobustNormEnum::GemanMcClure:
        return gtsam::SharedNoiseModel(gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::GemanMcClure::Create(robustNormConstant), diagonalNoise));
      case RobustNormEnum::DCS:
        return gtsam::SharedNoiseModel(
            gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::DCS::Create(robustNormConstant), diagonalNoise));
    }
    throw std::logic_error("Unknown robust norm for grouped GMSF factors.");
  };

  const gtsam::ExpressionFactor<FirstMeasurementType> firstFactor(
      createNoiseModel(firstExpressionPtr), firstExpressionPtr->getGtsamMeasurementValue(), firstGtsamExpression);
  const gtsam::ExpressionFactor<SecondMeasurementType> secondFactor(
      createNoiseModel(secondExpressionPtr), secondExpressionPtr->getGtsamMeasurementValue(), secondGtsamExpression);

  if (firstFactor.keys() != secondFactor.keys()) {
    throw std::logic_error("Grouped GMSF expression factors resolved different graph keys.");
  }
  const auto hasAuxiliaryGraphData = [](const auto& expressionPtr) {
    return !expressionPtr->getNewOnlineGraphStateValues().empty() || !expressionPtr->getNewOfflineGraphStateValues().empty() ||
           !expressionPtr->getNewOnlinePosePriorFactors().empty() || !expressionPtr->getNewOnlineDynamicPriorFactors().empty() ||
           !expressionPtr->getNewOnlineAndOfflinePoseBetweenFactors().empty();
  };
  if (hasAuxiliaryGraphData(firstExpressionPtr) || hasAuxiliaryGraphData(secondExpressionPtr)) {
    throw std::logic_error("Grouped GMSF expression factors cannot create auxiliary graph data.");
  }

  if (addToOnlineSmootherFlag) {
    rtFactorGraphBufferPtr_->add(firstFactor);
    rtFactorGraphBufferPtr_->add(secondFactor);
  }
  if (graphConfigPtr_->useAdditionalSlowBatchSmootherFlag_) {
    batchFactorGraphBufferPtr_->add(firstFactor);
    batchFactorGraphBufferPtr_->add(secondFactor);
  }

  for (const gtsam::Key key : firstFactor.keys()) {
    if (addToOnlineSmootherFlag) {
      const auto timestampIterator = rtGraphKeysTimestampsMapBufferPtr_->find(key);
      if (timestampIterator == rtGraphKeysTimestampsMapBufferPtr_->end() || measurementTimestamp > timestampIterator->second) {
        writeKeyToKeyTimeStampMap_(key, measurementTimestamp, rtGraphKeysTimestampsMapBufferPtr_);
      }
    }
    if (graphConfigPtr_->useAdditionalSlowBatchSmootherFlag_) {
      const auto timestampIterator = batchGraphKeysTimestampsMapBufferPtr_->find(key);
      if (timestampIterator == batchGraphKeysTimestampsMapBufferPtr_->end() || measurementTimestamp > timestampIterator->second) {
        writeKeyToKeyTimeStampMap_(key, measurementTimestamp, batchGraphKeysTimestampsMapBufferPtr_);
      }
    }
  }

  if (graphConfigPtr_->verboseLevel_ >= 2) {
    REGULAR_COUT << " Current propagated key " << propagatedStateKey_ << ": grouped expression factors of types "
                 << typeid(FIRST_GMSF_EXPRESSION_TYPE).name() << " and " << typeid(SECOND_GMSF_EXPRESSION_TYPE).name()
                 << " added atomically to keys ";
    for (const gtsam::Key key : firstFactor.keys()) {
      std::cout << gtsam::Symbol(key) << ", ";
    }
    std::cout << COLOR_END << std::endl;
  }
  ++unaryFactorsAdded_;
  return true;
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
