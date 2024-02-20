/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#define WORST_CASE_OPTIMIZATION_TIME 0.1  // in seconds

namespace graph_msf {
// Template Implementations
// GMSF Holistic Graph Factors with Extrinsic Calibration ------------------------
template <class GTSAM_MEASUREMENT_TYPE>
void GraphManager::addUnaryGmsfExpressionFactor(
    const std::shared_ptr<GmsfUnaryExpression<GTSAM_MEASUREMENT_TYPE>>& gmsfUnaryExpressionPtr) {
  // A. Generate Expression for Basic IMU State in World Frame at Key --------------------------------
  gtsam::Key closestGeneralKey;
  if (!getUnaryFactorGeneralKey(closestGeneralKey, *gmsfUnaryExpressionPtr->getUnaryMeasurementPtr())) {
    return;
  }
  gmsfUnaryExpressionPtr->generateExpressionForBasicImuStateInWorldFrameAtKey(closestGeneralKey);

  // B. Holistic Fusion: Optimize over fixed frame poses --------------------------------------------
  if (graphConfigPtr_->optimizeFixedFramePosesWrtWorld) {
    gmsfUnaryExpressionPtr->transformStateFromWorldToFixedFrame(gtsamExpressionTransformsKeys_, W_imuPropagatedState_);
  }

  //   C. Transform State to IMU Sensor Frame -----------------------------------------------------
  if (gmsfUnaryExpressionPtr->getUnaryMeasurementPtr()->sensorFrameName() != imuFrame_) {
    gmsfUnaryExpressionPtr->transformStateToSensorFrame();
  }

  // D. Extrinsic Calibration: Add correction to sensor pose -----------------------------------------
  if (graphConfigPtr_->optimizeExtrinsicSensorToSensorCorrectedOffset) {
    gmsfUnaryExpressionPtr->addExtrinsicCalibrationCorrection(gtsamExpressionTransformsKeys_);
  }

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
  }

  // Create Factor
  std::shared_ptr<gtsam::ExpressionFactor<GTSAM_MEASUREMENT_TYPE>> unaryExpressionFactorPtr;
  if (robustNormEnum == RobustNormEnum::None) {
    unaryExpressionFactorPtr = std::make_shared<gtsam::ExpressionFactor<GTSAM_MEASUREMENT_TYPE>>(
        noiseModel, gmsfUnaryExpressionPtr->getMeasurement(), gmsfUnaryExpressionPtr->getExpression());
  } else {
    unaryExpressionFactorPtr = std::make_shared<gtsam::ExpressionFactor<GTSAM_MEASUREMENT_TYPE>>(
        robustErrorFunction, gmsfUnaryExpressionPtr->getMeasurement(), gmsfUnaryExpressionPtr->getExpression());
  }

  // Operating on graph data
  const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
  // Add to graph
  addFactorToGraph_<const gtsam::ExpressionFactor<GTSAM_MEASUREMENT_TYPE>*>(unaryExpressionFactorPtr.get(),
                                                                            gmsfUnaryExpressionPtr->getTimestamp());
  // Write to timestamp map for fixed lag smoother
  for (const auto& key : unaryExpressionFactorPtr->keys()) {
    writeKeyToKeyTimeStampMap_(key, gmsfUnaryExpressionPtr->getTimestamp(), graphKeysTimestampsMapBufferPtr_);
  }
  // If one of the states was newly created, then add it to the values
  if (!gmsfUnaryExpressionPtr->getNewStateValues().empty()) {
    graphValuesBufferPtr_->insert(gmsfUnaryExpressionPtr->getNewStateValues());
  }
  // If new factors are there (due to newly generated factor or for regularization), add it to the graph
  if (!gmsfUnaryExpressionPtr->getNewPriorFactors().empty()) {
    factorGraphBufferPtr_->add(gmsfUnaryExpressionPtr->getNewPriorFactors());
  }

  // Print summary --------------------------------------
  if (graphConfigPtr_->verboseLevel > 0) {
    REGULAR_COUT << " Current propagated key " << propagatedStateKey_ << GREEN_START << ", expression factor of type "
                 << typeid(GTSAM_MEASUREMENT_TYPE).name() << " added to keys ";
    for (const auto& key : unaryExpressionFactorPtr->keys()) {
      std::cout << gtsam::Symbol(key) << ", ";
    }
    std::cout << COLOR_END << std::endl;
  }
}

// Private -----------------------------------------------------------------------------------------
template <class CHILDPTR>
void GraphManager::addFactorToGraph_(const gtsam::NoiseModelFactor* noiseModelFactorPtr) {
  factorGraphBufferPtr_->add(*dynamic_cast<CHILDPTR>(noiseModelFactorPtr));
}

template <class CHILDPTR>
void GraphManager::addFactorToGraph_(const gtsam::NoiseModelFactor* noiseModelFactorPtr, const double measurementTimestamp) {
  // Check Timestamp of Measurement on Delay
  if (timeToKeyBufferPtr_->getLatestTimestampInBuffer() - measurementTimestamp >
      graphConfigPtr_->realTimeSmootherLag - WORST_CASE_OPTIMIZATION_TIME) {
    REGULAR_COUT << RED_START
                 << " Measurement Delay is larger than the smootherLag - WORST_CASE_OPTIMIZATION_TIME, hence skipping this measurement."
                 << COLOR_END << std::endl;
  }
  // Add measurements
  return addFactorToGraph_<CHILDPTR>(noiseModelFactorPtr);
}

void GraphManager::writeKeyToKeyTimeStampMap_(const gtsam::Key& key, const double measurementTime,
                                              std::shared_ptr<std::map<gtsam::Key, double>> keyTimestampMapPtr) {
  (*keyTimestampMapPtr)[key] = measurementTime;
}

}  // namespace graph_msf
