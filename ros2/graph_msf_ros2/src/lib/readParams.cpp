/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
node_ file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of node_ package.
 */

// C++
#include <filesystem>

// Implementation
#include "graph_msf_ros2/GraphMsfRos2.h"

// Workspace
#include "graph_msf_ros2/ros/read_ros_params.h"

namespace graph_msf {

void GraphMsfRos2::readParams() {
  RCLCPP_INFO(node_->get_logger(), "GraphMsfRos2 - Reading parameters.");

  if (!graphConfigPtr_) {
    throw std::runtime_error("GraphMsfRos2: graphConfigPtr must be initialized.");
  }

  // Configuration
  // Sensor Parameters
  graphConfigPtr_->imuRate_ = tryGetParam<double>(node_, "sensor_params.imuRate");
  graphConfigPtr_->createStateEveryNthImuMeasurement_ =
      tryGetParam<int>(node_, "sensor_params.createStateEveryNthImuMeasurement");
  graphConfigPtr_->useImuSignalLowPassFilter_ = tryGetParam<bool>(node_, "sensor_params.useImuSignalLowPassFilter");
  graphConfigPtr_->imuLowPassFilterCutoffFreqHz_ =
      tryGetParam<double>(node_, "sensor_params.imuLowPassFilterCutoffFreq");
  graphConfigPtr_->maxSearchDeviation_ =
      1.0 / (graphConfigPtr_->imuRate_ / graphConfigPtr_->createStateEveryNthImuMeasurement_);
  graphConfigPtr_->imuBufferLength_ = tryGetParam<int>(node_, "sensor_params.imuBufferLength");
  graphConfigPtr_->imuTimeOffset_ = tryGetParam<double>(node_, "sensor_params.imuTimeOffset");

  // Initialization Params
  graphConfigPtr_->estimateGravityFromImuFlag_ =
      tryGetParam<bool>(node_, "initialization_params.estimateGravityFromImu");
  graphConfigPtr_->gravityMagnitude_ = tryGetParam<double>(node_, "initialization_params.gravityMagnitude");
  graphConfigPtr_->W_gravityVector_ = Eigen::Vector3d(0.0, 0.0, -1.0) * graphConfigPtr_->gravityMagnitude_;

  // Graph Params
  graphConfigPtr_->realTimeSmootherUseIsamFlag_ = tryGetParam<bool>(node_, "graph_params.realTimeSmootherUseIsam");
  graphConfigPtr_->realTimeSmootherLag_ = tryGetParam<double>(node_, "graph_params.realTimeSmootherLag");
  graphConfigPtr_->useAdditionalSlowBatchSmootherFlag_ =
      tryGetParam<bool>(node_, "graph_params.useAdditionalSlowBatchSmoother");
  graphConfigPtr_->slowBatchSmootherUseIsamFlag_ = tryGetParam<bool>(node_, "graph_params.slowBatchSmootherUseIsam");

  // Optimizer Config
  graphConfigPtr_->gaussNewtonWildfireThreshold_ =
      tryGetParam<double>(node_, "graph_params.gaussNewtonWildfireThreshold");
  graphConfigPtr_->minOptimizationFrequency_ = tryGetParam<double>(node_, "graph_params.minOptimizationFrequency");
  graphConfigPtr_->maxOptimizationFrequency_ = tryGetParam<double>(node_, "graph_params.maxOptimizationFrequency");
  graphConfigPtr_->additionalOptimizationIterations_ =
      tryGetParam<int>(node_, "graph_params.additionalOptimizationIterations");
  graphConfigPtr_->findUnusedFactorSlotsFlag_ = tryGetParam<bool>(node_, "graph_params.findUnusedFactorSlots");
  graphConfigPtr_->enableDetailedResultsFlag_ = tryGetParam<bool>(node_, "graph_params.enableDetailedResults");
  graphConfigPtr_->realTimeSmootherUseCholeskyFactorizationFlag_ =
      tryGetParam<bool>(node_, "graph_params.realTimeSmootherUseCholeskyFactorization");
  graphConfigPtr_->slowBatchSmootherUseCholeskyFactorizationFlag_ =
      tryGetParam<bool>(node_, "graph_params.slowBatchSmootherUseCholeskyFactorization");
  graphConfigPtr_->usingBiasForPreIntegrationFlag_ =
      tryGetParam<bool>(node_, "graph_params.usingBiasForPreIntegration");
  graphConfigPtr_->useWindowForMarginalsComputationFlag_ =
      tryGetParam<bool>(node_, "graph_params.useWindowForMarginalsComputation");
  graphConfigPtr_->windowSizeSecondsForMarginalsComputation_ =
      tryGetParam<double>(node_, "graph_params.windowSizeSecondsForMarginalsComputation");
  graphConfigPtr_->optimizeReferenceFramePosesWrtWorldFlag_ =
      tryGetParam<bool>(node_, "graph_params.optimizeReferenceFramePosesWrtWorld");
  graphConfigPtr_->optimizeExtrinsicSensorToSensorCorrectedOffsetFlag_ =
      tryGetParam<bool>(node_, "graph_params.optimizeExtrinsicSensorToSensorCorrectedOffset");

  // Alignment Parameters
  graphConfigPtr_->referenceFramePosesResetThreshold_ =
      tryGetParam<double>(node_, "graph_params.referenceFramePosesResetThreshold");
  graphConfigPtr_->centerMeasurementsAtKeyframePositionBeforeAlignmentFlag_ =
      tryGetParam<bool>(node_, "graph_params.centerMeasurementsAtKeyframePositionBeforeAlignment");
  graphConfigPtr_->createReferenceAlignmentKeyframeEveryNSeconds_ =
      tryGetParam<double>(node_, "graph_params.createReferenceAlignmentKeyframeEveryNSeconds");
//   graphConfigPtr_->centerReferenceFramesAtRobotPositionBeforeAlignment_ =
//       tryGetParam<bool>(node_, "graph_params.centerReferenceFramesAtRobotPositionBeforeAlignment");

  // Noise Parameters
  graphConfigPtr_->accNoiseDensity_ = tryGetParam<double>(node_, "noise_params.accNoiseDensity");
  graphConfigPtr_->integrationNoiseDensity_ = tryGetParam<double>(node_, "noise_params.integrationNoiseDensity");
  graphConfigPtr_->use2ndOrderCoriolisFlag_ = tryGetParam<bool>(node_, "noise_params.use2ndOrderCoriolis");
  graphConfigPtr_->gyroNoiseDensity_ = tryGetParam<double>(node_, "noise_params.gyrNoiseDensity");
  graphConfigPtr_->omegaCoriolis_ = tryGetParam<double>(node_, "noise_params.omegaCoriolis");
  graphConfigPtr_->accBiasRandomWalkNoiseDensity_ =
      tryGetParam<double>(node_, "noise_params.accBiasRandomWalkNoiseDensity");
  graphConfigPtr_->gyroBiasRandomWalkNoiseDensity_ =
      tryGetParam<double>(node_, "noise_params.gyrBiasRandomWalkNoiseDensity");
  graphConfigPtr_->biasAccOmegaInit_ = tryGetParam<double>(node_, "noise_params.biasAccOmegaInit");

  const double accBiasPrior = tryGetParam<double>(node_, "noise_params.accBiasPrior");
  graphConfigPtr_->accBiasPrior_ = Eigen::Vector3d(accBiasPrior, accBiasPrior, accBiasPrior);

  const double gyroBiasPrior = tryGetParam<double>(node_, "noise_params.gyrBiasPrior");
  graphConfigPtr_->gyroBiasPrior_ = Eigen::Vector3d(gyroBiasPrior, gyroBiasPrior, gyroBiasPrior);

  // Initial State
  graphConfigPtr_->initialPositionNoiseDensity_ =
      tryGetParam<double>(node_, "noise_params.initialPositionNoiseDensity");
  graphConfigPtr_->initialOrientationNoiseDensity_ =
      tryGetParam<double>(node_, "noise_params.initialOrientationNoiseDensity");
  graphConfigPtr_->initialVelocityNoiseDensity_ =
      tryGetParam<double>(node_, "noise_params.initialVelocityNoiseDensity");
  graphConfigPtr_->initialAccBiasNoiseDensity_ = tryGetParam<double>(node_, "noise_params.initialAccBiasNoiseDensity");
  graphConfigPtr_->initialGyroBiasNoiseDensity_ =
      tryGetParam<double>(node_, "noise_params.initialGyroBiasNoiseDensity");

  // Re-linearization
  graphConfigPtr_->positionReLinTh_ = tryGetParam<double>(node_, "relinearization_params.positionReLinTh");
  graphConfigPtr_->rotationReLinTh_ = tryGetParam<double>(node_, "relinearization_params.rotationReLinTh");
  graphConfigPtr_->velocityReLinTh_ = tryGetParam<double>(node_, "relinearization_params.velocityReLinTh");
  graphConfigPtr_->accBiasReLinTh_ = tryGetParam<double>(node_, "relinearization_params.accBiasReLinTh");
  graphConfigPtr_->gyroBiasReLinTh_ = tryGetParam<double>(node_, "relinearization_params.gyrBiasReLinTh");
  graphConfigPtr_->referenceFrameReLinTh_ = tryGetParam<double>(node_, "relinearization_params.referenceFrameReLinTh");
  graphConfigPtr_->calibrationReLinTh_ = tryGetParam<double>(node_, "relinearization_params.calibrationReLinTh");
  graphConfigPtr_->displacementReLinTh_ = tryGetParam<double>(node_, "relinearization_params.displacementReLinTh");
  graphConfigPtr_->landmarkReLinTh_ = tryGetParam<double>(node_, "relinearization_params.landmarkReLinTh");

  graphConfigPtr_->relinearizeSkip_ = tryGetParam<int>(node_, "relinearization_params.relinearizeSkip");
  graphConfigPtr_->enableRelinearizationFlag_ =
      tryGetParam<bool>(node_, "relinearization_params.enableRelinearization");
  graphConfigPtr_->evaluateNonlinearErrorFlag_ =
      tryGetParam<bool>(node_, "relinearization_params.evaluateNonlinearError");
  graphConfigPtr_->cacheLinearizedFactorsFlag_ =
      tryGetParam<bool>(node_, "relinearization_params.cacheLinearizedFactors");
  graphConfigPtr_->enablePartialRelinearizationCheckFlag_ =
      tryGetParam<bool>(node_, "relinearization_params.enablePartialRelinearizationCheck");

  // Common Parameters
  graphConfigPtr_->verboseLevel_ = tryGetParam<int>(node_, "common_params.verbosity");
  graphConfigPtr_->odomNotJumpAtStartFlag_ = tryGetParam<bool>(node_, "common_params.odomNotJumpAtStart");
  graphConfigPtr_->logRealTimeStateToMemoryFlag_ = tryGetParam<bool>(node_, "common_params.logRealTimeStateToMemory");
  graphConfigPtr_->logLatencyAndUpdateDurationToMemoryFlag_ =
      tryGetParam<bool>(node_, "common_params.logLatencyAndUpdateDurationToMemory");

  // Set frames
  staticTransformsPtr_->setWorldFrame(tryGetParam<std::string>(node_, "extrinsics.worldFrame"));
  staticTransformsPtr_->setOdomFrame(tryGetParam<std::string>(node_, "extrinsics.odomFrame"));
  staticTransformsPtr_->setImuFrame(tryGetParam<std::string>(node_, "extrinsics.imuFrame"));
  staticTransformsPtr_->setInitializationFrame(
      tryGetParam<std::string>(node_, "extrinsics.initializeZeroYawAndPositionOfFrame"));
  staticTransformsPtr_->setBaseLinkFrame(tryGetParam<std::string>(node_, "extrinsics.baseLinkFrame"));

  referenceFrameAlignedNameId = tryGetParam<std::string>(node_, "name_ids.referenceFrameAligned");
  sensorFrameCorrectedNameId = tryGetParam<std::string>(node_, "name_ids.sensorFrameCorrected");

  // Logging path
  if (graphConfigPtr_->useAdditionalSlowBatchSmootherFlag_) {
    optimizationResultLoggingPath = tryGetParam<std::string>(node_, "launch.optimizationResultLoggingPath");

    if (optimizationResultLoggingPath.back() != '/') {
      optimizationResultLoggingPath += "/";
    }

    if (!std::filesystem::exists(optimizationResultLoggingPath)) {
      std::filesystem::create_directories(optimizationResultLoggingPath);
    }
  }
}

}  // namespace graph_msf
