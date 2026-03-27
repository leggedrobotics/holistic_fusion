/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
this file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// C++
#include <filesystem>

// Implementation
#include "graph_msf_ros2/GraphMsfRos2.h"

// Workspace
#include "graph_msf_ros2/ros/read_ros_params.h"
// Constants
#include "graph_msf_ros2/constants.h"

namespace graph_msf {

void GraphMsfRos2::readParams() {
  RCLCPP_INFO(this->get_logger(), "GraphMsfRos2 - Reading parameters.");

  if (!graphConfigPtr_) {
    throw std::runtime_error("GraphMsfRos2: graphConfigPtr must be initialized.");
  }

  // Configuration
  // Sensor Parameters
  graphConfigPtr_->imuRate_ = tryGetParam<double>(this, "sensor_params.imuRate");
  graphConfigPtr_->createStateEveryNthImuMeasurement_ =
      tryGetParam<int>(this, "sensor_params.createStateEveryNthImuMeasurement");
  graphConfigPtr_->useImuSignalLowPassFilter_ = tryGetParam<bool>(this, "sensor_params.useImuSignalLowPassFilter");
  graphConfigPtr_->imuLowPassFilterCutoffFreqHz_ =
      tryGetParam<double>(this, "sensor_params.imuLowPassFilterCutoffFreq");
  graphConfigPtr_->maxSearchDeviation_ =
      1.0 / (graphConfigPtr_->imuRate_ / graphConfigPtr_->createStateEveryNthImuMeasurement_);
  graphConfigPtr_->imuBufferLength_ = tryGetParam<int>(this, "sensor_params.imuBufferLength");
  graphConfigPtr_->imuTimeOffset_ = tryGetParam<double>(this, "sensor_params.imuTimeOffset");
  graphConfigPtr_->isImuAccInG_ = tryGetParam<bool>(this, "sensor_params.isImuAccInG");

  // Initialization Params
  graphConfigPtr_->estimateGravityFromImuFlag_ =
      tryGetParam<bool>(this, "initialization_params.estimateGravityFromImu");
  graphConfigPtr_->gravityMagnitude_ = tryGetParam<double>(this, "initialization_params.gravityMagnitude");
  graphConfigPtr_->W_gravityVector_ = Eigen::Vector3d(0.0, 0.0, -1.0) * graphConfigPtr_->gravityMagnitude_;

  // Graph Params
  graphConfigPtr_->realTimeSmootherUseIsamFlag_ = tryGetParam<bool>(this, "graph_params.realTimeSmootherUseIsam");
  graphConfigPtr_->realTimeSmootherLag_ = tryGetParam<double>(this, "graph_params.realTimeSmootherLag");
  graphConfigPtr_->useAdditionalSlowBatchSmootherFlag_ =
      tryGetParam<bool>(this, "graph_params.useAdditionalSlowBatchSmoother");
  graphConfigPtr_->slowBatchSmootherUseIsamFlag_ = tryGetParam<bool>(this, "graph_params.slowBatchSmootherUseIsam");

  // Optimizer Config
  graphConfigPtr_->gaussNewtonWildfireThreshold_ =
      tryGetParam<double>(this, "graph_params.gaussNewtonWildfireThreshold");
  graphConfigPtr_->minOptimizationFrequency_ = tryGetParam<double>(this, "graph_params.minOptimizationFrequency");
  graphConfigPtr_->maxOptimizationFrequency_ = tryGetParam<double>(this, "graph_params.maxOptimizationFrequency");
  graphConfigPtr_->additionalOptimizationIterations_ =
      tryGetParam<int>(this, "graph_params.additionalOptimizationIterations");
  graphConfigPtr_->useAdaptiveAdditionalOptimizationIterationsFlag_ =
      tryGetParam<bool>(this, "graph_params.useAdaptiveAdditionalOptimizationIterations");
  graphConfigPtr_->adaptiveAdditionalOptimizationMinRelativeErrorImprovement_ =
      tryGetParam<double>(this, "graph_params.adaptiveAdditionalOptimizationMinRelativeErrorImprovement");
  graphConfigPtr_->printAdditionalOptimizationDiagnosticsFlag_ =
      tryGetParam<bool>(this, "graph_params.printAdditionalOptimizationDiagnostics");
  graphConfigPtr_->deferFutureUnaryMeasurementsFlag_ =
      tryGetParam<bool>(this, "graph_params.deferFutureUnaryMeasurements");
  graphConfigPtr_->maxDeferredUnaryFutureLeadSeconds_ =
      tryGetParam<double>(this, "graph_params.maxDeferredUnaryFutureLeadSeconds");
  graphConfigPtr_->maxDeferredUnaryMeasurementsInQueue_ =
      tryGetParam<int>(this, "graph_params.maxDeferredUnaryMeasurementsInQueue");
  graphConfigPtr_->findUnusedFactorSlotsFlag_ = tryGetParam<bool>(this, "graph_params.findUnusedFactorSlots");
  graphConfigPtr_->enableDetailedResultsFlag_ = tryGetParam<bool>(this, "graph_params.enableDetailedResults");
  graphConfigPtr_->realTimeSmootherUseCholeskyFactorizationFlag_ =
      tryGetParam<bool>(this, "graph_params.realTimeSmootherUseCholeskyFactorization");
  graphConfigPtr_->slowBatchSmootherUseCholeskyFactorizationFlag_ =
      tryGetParam<bool>(this, "graph_params.slowBatchSmootherUseCholeskyFactorization");
  graphConfigPtr_->usingBiasForPreIntegrationFlag_ =
      tryGetParam<bool>(this, "graph_params.usingBiasForPreIntegration");
  graphConfigPtr_->useWindowForMarginalsComputationFlag_ =
      tryGetParam<bool>(this, "graph_params.useWindowForMarginalsComputation");
  graphConfigPtr_->windowSizeSecondsForMarginalsComputation_ =
      tryGetParam<double>(this, "graph_params.windowSizeSecondsForMarginalsComputation");
  graphConfigPtr_->optimizeReferenceFramePosesWrtWorldFlag_ =
      tryGetParam<bool>(this, "graph_params.optimizeReferenceFramePosesWrtWorld");
  graphConfigPtr_->optimizeExtrinsicSensorToSensorCorrectedOffsetFlag_ =
      tryGetParam<bool>(this, "graph_params.optimizeExtrinsicSensorToSensorCorrectedOffset");

  // Alignment Parameters
  graphConfigPtr_->referenceFramePosesResetThreshold_ =
      tryGetParam<double>(this, "graph_params.referenceFramePosesResetThreshold");
  graphConfigPtr_->centerMeasurementsAtKeyframePositionBeforeAlignmentFlag_ =
      tryGetParam<bool>(this, "graph_params.centerMeasurementsAtKeyframePositionBeforeAlignment");
  graphConfigPtr_->createReferenceAlignmentKeyframeEveryNSeconds_ =
      tryGetParam<double>(this, "graph_params.createReferenceAlignmentKeyframeEveryNSeconds");
  if (graphConfigPtr_->optimizeReferenceFramePosesWrtWorldFlag_ &&
      graphConfigPtr_->createReferenceAlignmentKeyframeEveryNSeconds_ > graphConfigPtr_->realTimeSmootherLag_) {
    RCLCPP_WARN(
        this->get_logger(),
        "createReferenceAlignmentKeyframeEveryNSeconds (%.3fs) is larger than realTimeSmootherLag (%.3fs). "
        "Reference-alignment keys will regularly leave the online window before their successor is created, "
        "so delayed/resumed absolute measurements must rely on the old-key reactivation path.",
        graphConfigPtr_->createReferenceAlignmentKeyframeEveryNSeconds_, graphConfigPtr_->realTimeSmootherLag_);
  }

  // Noise Parameters
  graphConfigPtr_->accNoiseDensity_ = tryGetParam<double>(this, "noise_params.accNoiseDensity");
  graphConfigPtr_->integrationNoiseDensity_ = tryGetParam<double>(this, "noise_params.integrationNoiseDensity");
  graphConfigPtr_->use2ndOrderCoriolisFlag_ = tryGetParam<bool>(this, "noise_params.use2ndOrderCoriolis");
  graphConfigPtr_->gyroNoiseDensity_ = tryGetParam<double>(this, "noise_params.gyrNoiseDensity");
  graphConfigPtr_->omegaCoriolis_ = tryGetParam<double>(this, "noise_params.omegaCoriolis");
  graphConfigPtr_->accBiasRandomWalkNoiseDensity_ =
      tryGetParam<double>(this, "noise_params.accBiasRandomWalkNoiseDensity");
  graphConfigPtr_->gyroBiasRandomWalkNoiseDensity_ =
      tryGetParam<double>(this, "noise_params.gyrBiasRandomWalkNoiseDensity");
  graphConfigPtr_->biasAccOmegaInit_ = tryGetParam<double>(this, "noise_params.biasAccOmegaInit");

  const double accBiasPrior = tryGetParam<double>(this, "noise_params.accBiasPrior");
  graphConfigPtr_->accBiasPrior_ = Eigen::Vector3d(accBiasPrior, accBiasPrior, accBiasPrior);

  const double gyroBiasPrior = tryGetParam<double>(this, "noise_params.gyrBiasPrior");
  graphConfigPtr_->gyroBiasPrior_ = Eigen::Vector3d(gyroBiasPrior, gyroBiasPrior, gyroBiasPrior);

  // Initial State
  graphConfigPtr_->initialPositionNoiseDensity_ =
      tryGetParam<double>(this, "noise_params.initialPositionNoiseDensity");
  graphConfigPtr_->initialOrientationNoiseDensity_ =
      tryGetParam<double>(this, "noise_params.initialOrientationNoiseDensity");
  graphConfigPtr_->initialVelocityNoiseDensity_ =
      tryGetParam<double>(this, "noise_params.initialVelocityNoiseDensity");
  graphConfigPtr_->initialAccBiasNoiseDensity_ = tryGetParam<double>(this, "noise_params.initialAccBiasNoiseDensity");
  graphConfigPtr_->initialGyroBiasNoiseDensity_ =
      tryGetParam<double>(this, "noise_params.initialGyroBiasNoiseDensity");

  // Re-linearization
  graphConfigPtr_->positionReLinTh_ = tryGetParam<double>(this, "relinearization_params.positionReLinTh");
  graphConfigPtr_->rotationReLinTh_ = tryGetParam<double>(this, "relinearization_params.rotationReLinTh");
  graphConfigPtr_->velocityReLinTh_ = tryGetParam<double>(this, "relinearization_params.velocityReLinTh");
  graphConfigPtr_->accBiasReLinTh_ = tryGetParam<double>(this, "relinearization_params.accBiasReLinTh");
  graphConfigPtr_->gyroBiasReLinTh_ = tryGetParam<double>(this, "relinearization_params.gyrBiasReLinTh");
  graphConfigPtr_->referenceFrameReLinTh_ = tryGetParam<double>(this, "relinearization_params.referenceFrameReLinTh");
  graphConfigPtr_->calibrationReLinTh_ = tryGetParam<double>(this, "relinearization_params.calibrationReLinTh");
  graphConfigPtr_->displacementReLinTh_ = tryGetParam<double>(this, "relinearization_params.displacementReLinTh");
  graphConfigPtr_->landmarkReLinTh_ = tryGetParam<double>(this, "relinearization_params.landmarkReLinTh");

  graphConfigPtr_->relinearizeSkip_ = tryGetParam<int>(this, "relinearization_params.relinearizeSkip");
  graphConfigPtr_->enableRelinearizationFlag_ =
      tryGetParam<bool>(this, "relinearization_params.enableRelinearization");
  graphConfigPtr_->evaluateNonlinearErrorFlag_ =
      tryGetParam<bool>(this, "graph_params.evaluateNonlinearError");
  graphConfigPtr_->cacheLinearizedFactorsFlag_ =
      tryGetParam<bool>(this, "relinearization_params.cacheLinearizedFactors");
  graphConfigPtr_->enablePartialRelinearizationCheckFlag_ =
      tryGetParam<bool>(this, "relinearization_params.enablePartialRelinearizationCheck");

  if (graphConfigPtr_->useAdaptiveAdditionalOptimizationIterationsFlag_) {
    if (!graphConfigPtr_->realTimeSmootherUseIsamFlag_) {
      RCLCPP_WARN(
          this->get_logger(),
          "useAdaptiveAdditionalOptimizationIterations is enabled, but realTimeSmootherUseIsam is false. "
          "Adaptive stopping is only available for the iSAM2 real-time smoother and will fall back to the fixed "
          "additionalOptimizationIterations cap.");
    }
    if (!graphConfigPtr_->evaluateNonlinearErrorFlag_) {
      RCLCPP_WARN(
          this->get_logger(),
          "useAdaptiveAdditionalOptimizationIterations requires evaluateNonlinearError=true. "
          "Enabling evaluateNonlinearError automatically.");
      graphConfigPtr_->evaluateNonlinearErrorFlag_ = true;
    }
    if (graphConfigPtr_->additionalOptimizationIterations_ <= 0) {
      RCLCPP_WARN(
          this->get_logger(),
          "useAdaptiveAdditionalOptimizationIterations is enabled, but additionalOptimizationIterations is %d. "
          "No additional iterations will run until the cap is set above zero.",
          graphConfigPtr_->additionalOptimizationIterations_);
    }
    if (graphConfigPtr_->adaptiveAdditionalOptimizationMinRelativeErrorImprovement_ <= 0.0) {
      RCLCPP_WARN(
          this->get_logger(),
          "adaptiveAdditionalOptimizationMinRelativeErrorImprovement is %.3e. "
          "Non-positive values disable early stopping and keep the fixed additionalOptimizationIterations cap.",
          graphConfigPtr_->adaptiveAdditionalOptimizationMinRelativeErrorImprovement_);
    }
  }
  if (graphConfigPtr_->printAdditionalOptimizationDiagnosticsFlag_ &&
      !graphConfigPtr_->evaluateNonlinearErrorFlag_) {
    RCLCPP_WARN(
        this->get_logger(),
        "printAdditionalOptimizationDiagnostics requires evaluateNonlinearError=true. "
        "Enabling evaluateNonlinearError automatically.");
    graphConfigPtr_->evaluateNonlinearErrorFlag_ = true;
  }
  if (graphConfigPtr_->deferFutureUnaryMeasurementsFlag_ && graphConfigPtr_->maxDeferredUnaryMeasurementsInQueue_ <= 0) {
    RCLCPP_WARN(
        this->get_logger(),
        "deferFutureUnaryMeasurements is enabled, but maxDeferredUnaryMeasurementsInQueue is %d. "
        "Clamping it to 1.",
        graphConfigPtr_->maxDeferredUnaryMeasurementsInQueue_);
    graphConfigPtr_->maxDeferredUnaryMeasurementsInQueue_ = 1;
  }

  // Common Parameters
  graphConfigPtr_->verboseLevel_ = tryGetParam<int>(this, "common_params.verbosity");
  graphConfigPtr_->odomNotJumpAtStartFlag_ = tryGetParam<bool>(this, "common_params.odomNotJumpAtStart");
  graphConfigPtr_->logRealTimeStateToMemoryFlag_ = tryGetParam<bool>(this, "common_params.logRealTimeStateToMemory");
  graphConfigPtr_->logLatencyAndUpdateDurationToMemoryFlag_ =
      tryGetParam<bool>(this, "common_params.logLatencyAndUpdateDurationToMemory");

  // Set frames
  staticTransformsPtr_->setWorldFrame(tryGetParam<std::string>(this, "extrinsics.worldFrame"));
  staticTransformsPtr_->setOdomFrame(tryGetParam<std::string>(this, "extrinsics.odomFrame"));
  staticTransformsPtr_->setImuFrame(tryGetParam<std::string>(this, "extrinsics.imuFrame"));
  staticTransformsPtr_->setInitializationFrame(
      tryGetParam<std::string>(this, "extrinsics.initializeZeroYawAndPositionOfFrame"));
  staticTransformsPtr_->setBaseLinkFrame(tryGetParam<std::string>(this, "extrinsics.baseLinkFrame"));

  referenceFrameAlignedNameId = tryGetParam<std::string>(this, "name_ids.referenceFrameAligned");
  sensorFrameCorrectedNameId = tryGetParam<std::string>(this, "name_ids.sensorFrameCorrected");

  // Logging path
  if (graphConfigPtr_->useAdditionalSlowBatchSmootherFlag_) {
    optimizationResultLoggingPath = tryGetParam<std::string>(this, "launch.optimizationResultLoggingPath");

    if (optimizationResultLoggingPath.back() != '/') {
      optimizationResultLoggingPath += "/";
    }

    if (!std::filesystem::exists(optimizationResultLoggingPath)) {
      std::filesystem::create_directories(optimizationResultLoggingPath);
    }

    // Print the logging path
    std::cout << "-------------------------------------------------" << std::endl;
    RCLCPP_INFO(this->get_logger(), "Optimization results will be logged to: %s",
                optimizationResultLoggingPath.c_str());
    REGULAR_COUT << GREEN_START << " Optimization results will be logged to: "
                 << optimizationResultLoggingPath << COLOR_END << std::endl;
    std::cout << "-------------------------------------------------" << std::endl;
  }
}

}  // namespace graph_msf
