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
    addFactorSafelyToGraph_<const FACTOR_TYPE*>(unaryFactorPtr.get(), measurementTime);
  }

  // Print summary
  if (graphConfigPtr_->verboseLevel_ > 1) {
    REGULAR_COUT << " Current propagated key " << propagatedStateKey_ << GREEN_START << ", " << typeid(FACTOR_TYPE).name()
                 << " factor added to key " << closestKey << COLOR_END << std::endl;
  }
}

// 2) GMSF Holistic Graph Factors with Extrinsic Calibration ------------------------
template <class GMSF_EXPRESSION_TYPE>
void GraphManager::addUnaryGmsfExpressionFactor(const std::shared_ptr<GMSF_EXPRESSION_TYPE> gmsfUnaryExpressionPtr) {
  // Measurement
  const auto& unaryMeasurement = *gmsfUnaryExpressionPtr->getUnaryMeasurementPtr();

  // Check at compile time whether unary measurement is absolute
  constexpr bool isAbsoluteMeasurement =
      std::is_base_of<GmsfUnaryExpressionAbsolut<typename GMSF_EXPRESSION_TYPE::template_type>, GMSF_EXPRESSION_TYPE>::value;
  constexpr bool isLandmarkMeasurement =
      std::is_base_of<GmsfUnaryExpressionLandmark<typename GMSF_EXPRESSION_TYPE::template_type>, GMSF_EXPRESSION_TYPE>::value;

  // A. Generate Expression for Basic IMU State in World Frame at Key --------------------------------
  gtsam::Key closestGeneralKey;
  double closestGeneralKeyTime;
  if (!getUnaryFactorGeneralKey(closestGeneralKey, closestGeneralKeyTime, unaryMeasurement)) {
    return;
  }
  gmsfUnaryExpressionPtr->generateExpressionForBasicImuStateInWorldFrameAtKey(closestGeneralKey);

  // B.A. Holistic Fusion: Optimize over fixed frame poses --------------------------------------------
  if constexpr (isAbsoluteMeasurement) {
    if (graphConfigPtr_->optimizeReferenceFramePosesWrtWorld_ && unaryMeasurement.fixedFrameName() != worldFrame_) {
      gmsfUnaryExpressionPtr->transformStateFromWorldToFixedFrame(gtsamExpressionTransformsKeys_, W_imuPropagatedState_,
                                                                  graphConfigPtr_->centerReferenceFramesAtRobotPositionBeforeAlignment_);
    }
  }

  // B.B. Holistic Fusion: Create Landmark State in Dynamic Memory -------------------------------------
  if constexpr (isLandmarkMeasurement) {
    gmsfUnaryExpressionPtr->convertRobotAndLandmarkStatesToMeasurement(gtsamExpressionTransformsKeys_, W_imuPropagatedState_);
  }

  // C. Transform State to Sensor Frame -----------------------------------------------------
  if (gmsfUnaryExpressionPtr->getUnaryMeasurementPtr()->sensorFrameName() != imuFrame_) {
    gmsfUnaryExpressionPtr->transformStateToSensorFrame();
  }

  // D. Extrinsic Calibration: Add correction to sensor pose -----------------------------------------
  if (graphConfigPtr_->optimizeExtrinsicSensorToSensorCorrectedOffset_) {
    gmsfUnaryExpressionPtr->addExtrinsicCalibrationCorrection(gtsamExpressionTransformsKeys_);
  }

  // Factor
  std::shared_ptr<gtsam::ExpressionFactor<typename GMSF_EXPRESSION_TYPE::template_type>> unaryExpressionFactorPtr;

  // Noise & Error Function
  auto noiseModel = gtsam::noiseModel::Diagonal::Sigmas(gmsfUnaryExpressionPtr->getNoiseDensity());  // rad,rad,rad,x,y,z
  // Robust Error Function?
  const RobustNormEnum robustNormEnum(gmsfUnaryExpressionPtr->getUnaryMeasurementPtr()->robustNormEnum());
  const double robustNormConstant = gmsfUnaryExpressionPtr->getUnaryMeasurementPtr()->robustNormConstant();
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
          noiseModel, gmsfUnaryExpressionPtr->getMeasurement(), gmsfUnaryExpressionPtr->getExpression());
      break;
  }

  // Create Factor
  if (robustNormEnum != RobustNormEnum::None) {
    unaryExpressionFactorPtr = std::make_shared<gtsam::ExpressionFactor<typename GMSF_EXPRESSION_TYPE::template_type>>(
        robustErrorFunction, gmsfUnaryExpressionPtr->getMeasurement(), gmsfUnaryExpressionPtr->getExpression());
  }

  // Operating on graph data
  {
    const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
    // Add to graph
    const bool success = addFactorToGraph_<const gtsam::ExpressionFactor<typename GMSF_EXPRESSION_TYPE::template_type>*>(
        unaryExpressionFactorPtr.get(), gmsfUnaryExpressionPtr->getTimestamp(), "GMSF-Expression");

    // If successful
    if (success) {
      // A. Write to timestamp map for fixed lag smoother if newer than existing one
      for (const gtsam::Key& key : unaryExpressionFactorPtr->keys()) {
        // Find timestamp in existing buffer and update: if i) not existent or ii) newer than existing one -> write
        auto keyTimestampMapIterator = graphKeysTimestampsMapBufferPtr_->find(key);
        if (keyTimestampMapIterator == graphKeysTimestampsMapBufferPtr_->end()) {
          // TODO: Check if it should not be propagatedStateTime_ instead of gmsfUnaryExpressionPtr->getTimestamp()
          writeKeyToKeyTimeStampMap_(key, gmsfUnaryExpressionPtr->getTimestamp(), graphKeysTimestampsMapBufferPtr_);
        }
        // If timestamp is newer than existing one, write it --> dangerous, as it can get states out of order
        else if (gmsfUnaryExpressionPtr->getTimestamp() > keyTimestampMapIterator->second) {
          writeKeyToKeyTimeStampMap_(key, gmsfUnaryExpressionPtr->getTimestamp(), graphKeysTimestampsMapBufferPtr_);
        }
      }
      // B. If one of the states was newly created, then add it to the values
      if (!gmsfUnaryExpressionPtr->getNewStateValues().empty()) {
        graphValuesBufferPtr_->insert(gmsfUnaryExpressionPtr->getNewStateValues());
      }
      // C. If new factors are there (due to newly generated factor or for regularization), add it to the graph
      if (!gmsfUnaryExpressionPtr->getNewPriorPoseFactors().empty()) {
        factorGraphBufferPtr_->add(gmsfUnaryExpressionPtr->getNewPriorPoseFactors());
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
bool GraphManager::addFactorToGraph_(const gtsam::NoiseModelFactor* noiseModelFactorPtr) {
  factorGraphBufferPtr_->add(*dynamic_cast<CHILDPTR>(noiseModelFactorPtr));
  return true;
}

template <class CHILDPTR>
bool GraphManager::addFactorToGraph_(const gtsam::NoiseModelFactor* noiseModelFactorPtr, const double measurementTimestamp,
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
  return addFactorToGraph_<CHILDPTR>(noiseModelFactorPtr);
}

template <class CHILDPTR>
bool GraphManager::addFactorSafelyToGraph_(const gtsam::NoiseModelFactor* noiseModelFactorPtr, const double measurementTimestamp) {
  // Operating on graph data --> acquire mutex
  const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
  // Add measurements
  return addFactorToGraph_<CHILDPTR>(noiseModelFactorPtr, measurementTimestamp, "safe");
}

void GraphManager::writeKeyToKeyTimeStampMap_(const gtsam::Key& key, const double measurementTime,
                                              std::shared_ptr<std::map<gtsam::Key, double>> keyTimestampMapPtr) {
  (*keyTimestampMapPtr)[key] = measurementTime;
}

}  // namespace graph_msf
