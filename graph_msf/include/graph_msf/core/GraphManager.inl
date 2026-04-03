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
UnaryAddOutcome GraphManager::addUnaryFactorInImuFrame(const MEASUREMENT_TYPE& unaryMeasurement,
                                                       const Eigen::Matrix<double, NOISE_DIM, 1>& unaryNoiseDensity,
                                                       const double measurementTime) {
  // Find the closest key in existing graph
  double closestGraphTime = 0.0;
  gtsam::Key closestKey;
  std::string callingName = "GnssPositionUnaryFactor";
  const UnaryAddOutcome outcome = classifyUnaryMeasurementTime_(closestKey, closestGraphTime, callingName, measurementTime);
  if (outcome.status != UnaryAddStatus::Added) {
    return outcome;
  }

  // Create noise model
  auto noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(unaryNoiseDensity)));  // m,m,m

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
                 << " factor added to key " << closestKey << " at graph time " << std::setprecision(14) << closestGraphTime
                 << COLOR_END << std::endl;
  }

  return {UnaryAddStatus::Added, 0.0};
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
UnaryAddOutcome GraphManager::addUnaryGmsfExpressionFactor(const std::shared_ptr<GMSF_EXPRESSION_TYPE> gmsfUnaryExpressionPtr,
                                                           const bool addToOnlineSmootherFlag) {
  // Measurement
  const auto& unaryMeasurement = *gmsfUnaryExpressionPtr->getGmsfBaseUnaryMeasurementPtr();

  // Get corresponding key of robot state in graph
  gtsam::Key closestGeneralKey;
  double closestGeneralKeyTime = 0.0;
  const UnaryAddOutcome outcome = getUnaryFactorGeneralKey(closestGeneralKey, closestGeneralKeyTime, unaryMeasurement);
  if (outcome.status != UnaryAddStatus::Added) {
    return outcome;
  }

  // Create Expression --> exact type of expression is determined by the template
  std::shared_ptr<gtsam::ExpressionFactor<typename GMSF_EXPRESSION_TYPE::template_type>> unaryExpressionFactorPtr;

  // Noise & Error Function
  auto noiseModel = gtsam::noiseModel::Diagonal::Sigmas(gmsfUnaryExpressionPtr->getNoiseDensity());  // rad,rad,rad,x,y,z
  // Robust Error Function?
  const RobustNormEnum robustNormEnum(gmsfUnaryExpressionPtr->getGmsfBaseUnaryMeasurementPtr()->robustNormEnum());
  const double robustNormConstant = gmsfUnaryExpressionPtr->getGmsfBaseUnaryMeasurementPtr()->robustNormConstant();
  std::shared_ptr<gtsam::noiseModel::Robust> robustErrorFunction;

  // Pick Robust Error Function
  switch (robustNormEnum) {
    case RobustNormEnum::Huber:
      robustErrorFunction =
          gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(robustNormConstant), noiseModel);
      break;
    case RobustNormEnum::Cauchy:
      robustErrorFunction =
          gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Cauchy::Create(robustNormConstant), noiseModel);
      break;
    case RobustNormEnum::Tukey:
      robustErrorFunction =
          gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Tukey::Create(robustNormConstant), noiseModel);
      break;
    case RobustNormEnum::GemanMcClure:
      robustErrorFunction =
          gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::GemanMcClure::Create(robustNormConstant), noiseModel);
      break;
    case RobustNormEnum::DCS:
      // robustNormConstant is the DCS parameter c (a.k.a. k in GTSAM)
      robustErrorFunction =
          gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::DCS::Create(robustNormConstant), noiseModel);
      break;
    case RobustNormEnum::None:
      break;
  }

  const double ts = gmsfUnaryExpressionPtr->getTimestamp();
  // The pose/position factor is still added at the sensor rate. This flag only decides whether the residual also touches the
  // shared extrinsic-calibration state for this particular factor instance.
  const bool useExtrinsicCalibrationResidual =
      graphConfigPtr_->optimizeExtrinsicSensorToSensorCorrectedOffsetFlag_ &&
      shouldUseExtrinsicCalibrationForMeasurement_(*gmsfUnaryExpressionPtr->getGmsfBaseUnaryMeasurementPtr());
  Eigen::Matrix<double, 6, 1> initialCalibrationPriorSigmas;
  initialCalibrationPriorSigmas << graphConfigPtr_->initialOrientationNoiseDensity_, graphConfigPtr_->initialOrientationNoiseDensity_,
      graphConfigPtr_->initialOrientationNoiseDensity_, graphConfigPtr_->initialPositionNoiseDensity_,
      graphConfigPtr_->initialPositionNoiseDensity_, graphConfigPtr_->initialPositionNoiseDensity_;

  // Operating on graph data
  {
    const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
    DynamicDictionaryContainer dynamicExpressionKeysSnapshot;
    bool factorCommitted = false;
    {
      std::scoped_lock snapshotLock(gtsamDynamicExpressionKeys_.get<gtsam::Pose3>().mutex(),
                                    gtsamDynamicExpressionKeys_.get<gtsam::Point3>().mutex());
      dynamicExpressionKeysSnapshot = DynamicDictionaryContainer(gtsamDynamicExpressionKeys_);
    }

    try {
      // Create Expression --> exact type of expression is determined by the template
      const auto gmsfGtsamExpression = gmsfUnaryExpressionPtr->createAndReturnExpression(
          closestGeneralKey, gtsamDynamicExpressionKeys_, W_imuPropagatedState_, graphConfigPtr_->optimizeReferenceFramePosesWrtWorldFlag_,
          graphConfigPtr_->centerMeasurementsAtKeyframePositionBeforeAlignmentFlag_,
          graphConfigPtr_->optimizeExtrinsicSensorToSensorCorrectedOffsetFlag_, useExtrinsicCalibrationResidual,
          initialCalibrationPriorSigmas);

      // Factor
      if (robustNormEnum == RobustNormEnum::None) {
        unaryExpressionFactorPtr = std::make_shared<gtsam::ExpressionFactor<typename GMSF_EXPRESSION_TYPE::template_type>>(
            noiseModel, gmsfUnaryExpressionPtr->getGtsamMeasurementValue(), gmsfGtsamExpression);
      } else {
        unaryExpressionFactorPtr = std::make_shared<gtsam::ExpressionFactor<typename GMSF_EXPRESSION_TYPE::template_type>>(
            robustErrorFunction, gmsfUnaryExpressionPtr->getGtsamMeasurementValue(), gmsfGtsamExpression);
      }

      // A. Main expression factor: add to graph -------------------------------------------------------------------------------------------
      const bool success =
          addFactorToRtAndBatchGraph_<const gtsam::ExpressionFactor<typename GMSF_EXPRESSION_TYPE::template_type>*>(
              unaryExpressionFactorPtr.get(), ts, "GMSF-Expression", addToOnlineSmootherFlag);

      if (!success) {
        if (useExtrinsicCalibrationResidual || unaryMeasurement.extrinsicCalibrationResidualStride() > 1) {
          rollbackExtrinsicCalibrationReservation_(unaryMeasurement);
        }
        std::scoped_lock restoreLock(gtsamDynamicExpressionKeys_.get<gtsam::Pose3>().mutex(),
                                     gtsamDynamicExpressionKeys_.get<gtsam::Point3>().mutex());
        gtsamDynamicExpressionKeys_ = dynamicExpressionKeysSnapshot;
        return {UnaryAddStatus::Dropped, 0.0};
      }
      factorCommitted = true;

      // B. Write to timestamp map for fixed lag smoother (writeKeyToKeyTimeStampMap_ handles "newer-than" internally)
      // B.a) Keys in expression factor
      for (const gtsam::Key& key : unaryExpressionFactorPtr->keys()) {
        if (addToOnlineSmootherFlag) {
          writeKeyToKeyTimeStampMap_(key, ts, rtGraphKeysTimestampsMapBufferPtr_);
        }
        if (graphConfigPtr_->useAdditionalSlowBatchSmootherFlag_) {
          writeKeyToKeyTimeStampMap_(key, ts, batchGraphKeysTimestampsMapBufferPtr_);
        }
      }

      // Unary factors are attached to a full navigation state at closestGeneralKey.
      // In the fixed-lag smoother, extending only x_k can let v_k / b_k fall out of the
      // window first, leaving the pose state underconstrained on delayed measurements.
      if (addToOnlineSmootherFlag) {
        writeKeyToKeyTimeStampMap_(gtsam::symbol_shorthand::X(closestGeneralKey), ts, rtGraphKeysTimestampsMapBufferPtr_);
        writeKeyToKeyTimeStampMap_(gtsam::symbol_shorthand::V(closestGeneralKey), ts, rtGraphKeysTimestampsMapBufferPtr_);
        writeKeyToKeyTimeStampMap_(gtsam::symbol_shorthand::B(closestGeneralKey), ts, rtGraphKeysTimestampsMapBufferPtr_);
      }
      if (graphConfigPtr_->useAdditionalSlowBatchSmootherFlag_) {
        writeKeyToKeyTimeStampMap_(gtsam::symbol_shorthand::X(closestGeneralKey), ts, batchGraphKeysTimestampsMapBufferPtr_);
        writeKeyToKeyTimeStampMap_(gtsam::symbol_shorthand::V(closestGeneralKey), ts, batchGraphKeysTimestampsMapBufferPtr_);
        writeKeyToKeyTimeStampMap_(gtsam::symbol_shorthand::B(closestGeneralKey), ts, batchGraphKeysTimestampsMapBufferPtr_);
      }

      // B.b) Keys in newly created online graph values (can activate previously inactive keys)
      if (addToOnlineSmootherFlag) {
        for (const auto& key : gmsfUnaryExpressionPtr->getNewOnlineGraphStateValues().keys()) {
          writeKeyToKeyTimeStampMap_(key, ts, rtGraphKeysTimestampsMapBufferPtr_);
        }
      }

      // C. If one of the states was newly created, then add it to the values buffer -------------------------------------------------------
      // a) Rt Graph
      if (!gmsfUnaryExpressionPtr->getNewOnlineGraphStateValues().empty() && addToOnlineSmootherFlag) {
        rtGraphValuesBufferPtr_->insert(gmsfUnaryExpressionPtr->getNewOnlineGraphStateValues());
      }
      // b) Batch Graph
      if (!gmsfUnaryExpressionPtr->getNewOfflineGraphStateValues().empty()) {
        batchGraphValuesBufferPtr_->insert(gmsfUnaryExpressionPtr->getNewOfflineGraphStateValues());
      }

      // D. If new factors are there (due to newly generated factor or for regularization), add them to the graph ---------------------------
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
    } catch (...) {
      if (!factorCommitted && (useExtrinsicCalibrationResidual || unaryMeasurement.extrinsicCalibrationResidualStride() > 1)) {
        rollbackExtrinsicCalibrationReservation_(unaryMeasurement);
      }
      std::scoped_lock restoreLock(gtsamDynamicExpressionKeys_.get<gtsam::Pose3>().mutex(),
                                   gtsamDynamicExpressionKeys_.get<gtsam::Point3>().mutex());
      gtsamDynamicExpressionKeys_ = dynamicExpressionKeysSnapshot;
      throw;
    }
  }

  // Print summary --------------------------------------
  if (graphConfigPtr_->verboseLevel_ >= 2) {
    REGULAR_COUT << " Current propagated key " << propagatedStateKey_ << ": expression factor of type "
                 << typeid(GMSF_EXPRESSION_TYPE).name() << " added at graph time " << std::setprecision(14)
                 << closestGeneralKeyTime << " to keys ";
    for (const auto& key : unaryExpressionFactorPtr->keys()) {
      std::cout << gtsam::Symbol(key) << ", ";
    }
    std::cout << COLOR_END << std::endl;
  }

  return {UnaryAddStatus::Added, 0.0};
}

template <class GMSF_EXPRESSION_TYPE>
bool GraphManager::addBinaryGmsfExpressionFactor(const std::shared_ptr<GMSF_EXPRESSION_TYPE> gmsfBinaryExpressionPtr) {
  const auto& binaryMeasurement = *gmsfBinaryExpressionPtr->getGmsfBaseBinaryMeasurementPtr();

  const double maxTimestampDistance = (1.0 / binaryMeasurement.measurementRate()) + (2.0 * graphConfigPtr_->maxSearchDeviation_);
  gtsam::Key closestKeyKm1 = 0;
  gtsam::Key closestKeyK = 0;
  double keyTimeStampDistance = 0.0;
  if (!findGraphKeys_(closestKeyKm1, closestKeyK, keyTimeStampDistance, maxTimestampDistance, binaryMeasurement.timeKm1(),
                      binaryMeasurement.timeK(), binaryMeasurement.measurementName())) {
    return false;
  }

  // Binary factors follow the same policy: keep the motion factor, decimate only the coupling to the calibration state.
  const bool useExtrinsicCalibrationResidual =
      graphConfigPtr_->optimizeExtrinsicSensorToSensorCorrectedOffsetFlag_ &&
      shouldUseExtrinsicCalibrationForMeasurement_(binaryMeasurement);

  std::shared_ptr<gtsam::ExpressionFactor<typename GMSF_EXPRESSION_TYPE::template_type>> binaryExpressionFactorPtr;

  auto noiseModel = gtsam::noiseModel::Diagonal::Sigmas(gmsfBinaryExpressionPtr->getNoiseDensity());  // rad,rad,rad,m,m,m
  const RobustNormEnum robustNormEnum(binaryMeasurement.robustNormEnum());
  const double robustNormConstant = binaryMeasurement.robustNormConstant();
  std::shared_ptr<gtsam::noiseModel::Robust> robustErrorFunction;

  switch (robustNormEnum) {
    case RobustNormEnum::Huber:
      robustErrorFunction =
          gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(robustNormConstant), noiseModel);
      break;
    case RobustNormEnum::Cauchy:
      robustErrorFunction =
          gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Cauchy::Create(robustNormConstant), noiseModel);
      break;
    case RobustNormEnum::Tukey:
      robustErrorFunction =
          gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Tukey::Create(robustNormConstant), noiseModel);
      break;
    case RobustNormEnum::GemanMcClure:
      robustErrorFunction =
          gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::GemanMcClure::Create(robustNormConstant), noiseModel);
      break;
    case RobustNormEnum::DCS:
      robustErrorFunction =
          gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::DCS::Create(robustNormConstant), noiseModel);
      break;
    case RobustNormEnum::None:
      break;
  }

  Eigen::Matrix<double, 6, 1> initialCalibrationPriorSigmas;
  initialCalibrationPriorSigmas << graphConfigPtr_->initialOrientationNoiseDensity_, graphConfigPtr_->initialOrientationNoiseDensity_,
      graphConfigPtr_->initialOrientationNoiseDensity_, graphConfigPtr_->initialPositionNoiseDensity_,
      graphConfigPtr_->initialPositionNoiseDensity_, graphConfigPtr_->initialPositionNoiseDensity_;

  {
    const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
    DynamicDictionaryContainer dynamicExpressionKeysSnapshot;
    bool factorCommitted = false;
    {
      std::scoped_lock snapshotLock(gtsamDynamicExpressionKeys_.get<gtsam::Pose3>().mutex(),
                                    gtsamDynamicExpressionKeys_.get<gtsam::Point3>().mutex());
      dynamicExpressionKeysSnapshot = DynamicDictionaryContainer(gtsamDynamicExpressionKeys_);
    }

    try {
      const auto gmsfGtsamExpression =
          gmsfBinaryExpressionPtr->createAndReturnExpression(closestKeyKm1, closestKeyK, gtsamDynamicExpressionKeys_,
                                                             initialCalibrationPriorSigmas,
                                                             graphConfigPtr_->optimizeExtrinsicSensorToSensorCorrectedOffsetFlag_,
                                                             useExtrinsicCalibrationResidual);
      const auto scaledMeasurementValue = gmsfBinaryExpressionPtr->getScaledGtsamMeasurementValue(keyTimeStampDistance);

      if (robustNormEnum == RobustNormEnum::None) {
        binaryExpressionFactorPtr = std::make_shared<gtsam::ExpressionFactor<typename GMSF_EXPRESSION_TYPE::template_type>>(
            noiseModel, scaledMeasurementValue, gmsfGtsamExpression);
      } else {
        binaryExpressionFactorPtr = std::make_shared<gtsam::ExpressionFactor<typename GMSF_EXPRESSION_TYPE::template_type>>(
            robustErrorFunction, scaledMeasurementValue, gmsfGtsamExpression);
      }

      const bool success =
          addFactorToRtAndBatchGraph_<const gtsam::ExpressionFactor<typename GMSF_EXPRESSION_TYPE::template_type>*>(
              binaryExpressionFactorPtr.get(), binaryMeasurement.timeKm1(), "GMSF-BinaryExpression");
      if (!success) {
        if (useExtrinsicCalibrationResidual || binaryMeasurement.extrinsicCalibrationResidualStride() > 1) {
          rollbackExtrinsicCalibrationReservation_(binaryMeasurement);
        }
        std::scoped_lock restoreLock(gtsamDynamicExpressionKeys_.get<gtsam::Pose3>().mutex(),
                                     gtsamDynamicExpressionKeys_.get<gtsam::Point3>().mutex());
        gtsamDynamicExpressionKeys_ = dynamicExpressionKeysSnapshot;
        return false;
      }
      factorCommitted = true;

      for (const gtsam::Key& key : binaryExpressionFactorPtr->keys()) {
        writeKeyToKeyTimeStampMap_(key, binaryMeasurement.timeK(), rtGraphKeysTimestampsMapBufferPtr_);
        if (graphConfigPtr_->useAdditionalSlowBatchSmootherFlag_) {
          writeKeyToKeyTimeStampMap_(key, binaryMeasurement.timeK(), batchGraphKeysTimestampsMapBufferPtr_);
        }
      }

      writeKeyToKeyTimeStampMap_(gtsam::symbol_shorthand::X(closestKeyKm1), binaryMeasurement.timeKm1(), rtGraphKeysTimestampsMapBufferPtr_);
      writeKeyToKeyTimeStampMap_(gtsam::symbol_shorthand::V(closestKeyKm1), binaryMeasurement.timeKm1(), rtGraphKeysTimestampsMapBufferPtr_);
      writeKeyToKeyTimeStampMap_(gtsam::symbol_shorthand::B(closestKeyKm1), binaryMeasurement.timeKm1(), rtGraphKeysTimestampsMapBufferPtr_);
      writeKeyToKeyTimeStampMap_(gtsam::symbol_shorthand::X(closestKeyK), binaryMeasurement.timeK(), rtGraphKeysTimestampsMapBufferPtr_);
      writeKeyToKeyTimeStampMap_(gtsam::symbol_shorthand::V(closestKeyK), binaryMeasurement.timeK(), rtGraphKeysTimestampsMapBufferPtr_);
      writeKeyToKeyTimeStampMap_(gtsam::symbol_shorthand::B(closestKeyK), binaryMeasurement.timeK(), rtGraphKeysTimestampsMapBufferPtr_);
      if (graphConfigPtr_->useAdditionalSlowBatchSmootherFlag_) {
        writeKeyToKeyTimeStampMap_(gtsam::symbol_shorthand::X(closestKeyKm1), binaryMeasurement.timeKm1(),
                                   batchGraphKeysTimestampsMapBufferPtr_);
        writeKeyToKeyTimeStampMap_(gtsam::symbol_shorthand::V(closestKeyKm1), binaryMeasurement.timeKm1(),
                                   batchGraphKeysTimestampsMapBufferPtr_);
        writeKeyToKeyTimeStampMap_(gtsam::symbol_shorthand::B(closestKeyKm1), binaryMeasurement.timeKm1(),
                                   batchGraphKeysTimestampsMapBufferPtr_);
        writeKeyToKeyTimeStampMap_(gtsam::symbol_shorthand::X(closestKeyK), binaryMeasurement.timeK(), batchGraphKeysTimestampsMapBufferPtr_);
        writeKeyToKeyTimeStampMap_(gtsam::symbol_shorthand::V(closestKeyK), binaryMeasurement.timeK(), batchGraphKeysTimestampsMapBufferPtr_);
        writeKeyToKeyTimeStampMap_(gtsam::symbol_shorthand::B(closestKeyK), binaryMeasurement.timeK(), batchGraphKeysTimestampsMapBufferPtr_);
      }

      if (!gmsfBinaryExpressionPtr->getNewOnlineGraphStateValues().empty()) {
        writeValueKeysToKeyTimeStampMap_(gmsfBinaryExpressionPtr->getNewOnlineGraphStateValues(), binaryMeasurement.timeK(),
                                         rtGraphKeysTimestampsMapBufferPtr_);
        rtGraphValuesBufferPtr_->insert(gmsfBinaryExpressionPtr->getNewOnlineGraphStateValues());
      }
      if (!gmsfBinaryExpressionPtr->getNewOfflineGraphStateValues().empty()) {
        batchGraphValuesBufferPtr_->insert(gmsfBinaryExpressionPtr->getNewOfflineGraphStateValues());
        if (graphConfigPtr_->useAdditionalSlowBatchSmootherFlag_) {
          writeValueKeysToKeyTimeStampMap_(gmsfBinaryExpressionPtr->getNewOfflineGraphStateValues(), binaryMeasurement.timeK(),
                                           batchGraphKeysTimestampsMapBufferPtr_);
        }
      }
      if (!gmsfBinaryExpressionPtr->getNewOnlineDynamicPriorFactors().empty()) {
        rtFactorGraphBufferPtr_->add(gmsfBinaryExpressionPtr->getNewOnlineDynamicPriorFactors());
      }
    } catch (...) {
      if (!factorCommitted && (useExtrinsicCalibrationResidual || binaryMeasurement.extrinsicCalibrationResidualStride() > 1)) {
        rollbackExtrinsicCalibrationReservation_(binaryMeasurement);
      }
      std::scoped_lock restoreLock(gtsamDynamicExpressionKeys_.get<gtsam::Pose3>().mutex(),
                                   gtsamDynamicExpressionKeys_.get<gtsam::Point3>().mutex());
      gtsamDynamicExpressionKeys_ = dynamicExpressionKeysSnapshot;
      throw;
    }
  }

  return true;
}

// Private -----------------------------------------------------------------------------------------
template <class CHILDPTR>
bool GraphManager::addFactorToRtAndBatchGraph_(const gtsam::NoiseModelFactor* noiseModelFactorPtr,
                                               const bool addToOnlineSmootherFlag) {
#ifndef NDEBUG
  // Debug-time type check only (avoid RTTI cost in release)
  assert(dynamic_cast<CHILDPTR>(noiseModelFactorPtr) != nullptr);
#endif

  // Add to real-time graph
  if (addToOnlineSmootherFlag) {
    rtFactorGraphBufferPtr_->add(*static_cast<CHILDPTR>(noiseModelFactorPtr));
  }

  // Add to batch graph
  if (graphConfigPtr_->useAdditionalSlowBatchSmootherFlag_) {
    batchFactorGraphBufferPtr_->add(*static_cast<CHILDPTR>(noiseModelFactorPtr));
  }
  return true;
}

template <class CHILDPTR>
bool GraphManager::addFactorToRtAndBatchGraph_(const gtsam::NoiseModelFactor* noiseModelFactorPtr,
                                               const double measurementTimestamp, const std::string& measurementName,
                                               const bool addToOnlineSmootherFlag) {
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
  // Insert if new; otherwise update only if newer (reduces redundant finds at call sites)
  auto [it, inserted] = keyTimestampMapPtr->emplace(key, measurementTime);
  if (!inserted && measurementTime > it->second) {
    it->second = measurementTime;
  }
}

}  // namespace graph_msf
