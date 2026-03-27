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

void GraphMsfRos2::declareRosParams() {
  // Sensor Params
  this->declare_parameter("sensor_params.imuRate", 0.0);
  this->declare_parameter("sensor_params.createStateEveryNthImuMeasurement", 25);
  this->declare_parameter("sensor_params.useImuSignalLowPassFilter", false);
  this->declare_parameter("sensor_params.imuLowPassFilterCutoffFreq", 0.0);
  this->declare_parameter("sensor_params.imuBufferLength", 800);
  this->declare_parameter("sensor_params.imuTimeOffset", 0.0);
  this->declare_parameter("sensor_params.isImuAccInG", false);  // If true, the IMU acceleration is in g, otherwise in m/s^2

  // Initialization Params
  this->declare_parameter("initialization_params.estimateGravityFromImu", false);
  this->declare_parameter("initialization_params.gravityMagnitude", 9.80665);

  // Graph Params
  this->declare_parameter("graph_params.realTimeSmootherUseIsam", false);
  this->declare_parameter("graph_params.realTimeSmootherLag", 0.0);
  this->declare_parameter("graph_params.useAdditionalSlowBatchSmoother", false);
  this->declare_parameter("graph_params.slowBatchSmootherUseIsam", false);
  this->declare_parameter("graph_params.gaussNewtonWildfireThreshold", 0.0);
  this->declare_parameter("graph_params.minOptimizationFrequency", 0.0);
  this->declare_parameter("graph_params.maxOptimizationFrequency", 0.0);
  this->declare_parameter("graph_params.additionalOptimizationIterations", 0);
  this->declare_parameter("graph_params.useAdaptiveAdditionalOptimizationIterations", false);
  this->declare_parameter("graph_params.adaptiveAdditionalOptimizationMinRelativeErrorImprovement", 0.0);
  this->declare_parameter("graph_params.printAdditionalOptimizationDiagnostics", false);
  this->declare_parameter("graph_params.deferFutureUnaryMeasurements", false);
  this->declare_parameter("graph_params.maxDeferredUnaryFutureLeadSeconds", 0.0);
  this->declare_parameter("graph_params.maxDeferredUnaryMeasurementsInQueue", 100);
  this->declare_parameter("graph_params.findUnusedFactorSlots", false);
  this->declare_parameter("graph_params.enableDetailedResults", false);
  this->declare_parameter("graph_params.realTimeSmootherUseCholeskyFactorization", false);
  this->declare_parameter("graph_params.slowBatchSmootherUseCholeskyFactorization", false);
  this->declare_parameter("graph_params.usingBiasForPreIntegration", false);
  this->declare_parameter("graph_params.optimizeReferenceFramePosesWrtWorld", false);
  this->declare_parameter("graph_params.optimizeExtrinsicSensorToSensorCorrectedOffset", false);
  this->declare_parameter("graph_params.referenceFramePosesResetThreshold", 0.0);
  this->declare_parameter("graph_params.centerMeasurementsAtKeyframePositionBeforeAlignment", false);

  this->declare_parameter("graph_params.useWindowForMarginalsComputation", false);
  this->declare_parameter("graph_params.windowSizeSecondsForMarginalsComputation", 0.0);
  this->declare_parameter("graph_params.createReferenceAlignmentKeyframeEveryNSeconds", 0.0);

  // Noise Params
  this->declare_parameter("noise_params.accNoiseDensity", 0.0);
  this->declare_parameter("noise_params.integrationNoiseDensity", 0.0);
  this->declare_parameter("noise_params.use2ndOrderCoriolis", false);
  this->declare_parameter("noise_params.gyrNoiseDensity", 0.0);
  this->declare_parameter("noise_params.omegaCoriolis", 0.0);
  this->declare_parameter("noise_params.accBiasRandomWalkNoiseDensity", 0.0);
  this->declare_parameter("noise_params.gyrBiasRandomWalkNoiseDensity", 0.0);
  this->declare_parameter("noise_params.biasAccOmegaInit", 0.0);
  this->declare_parameter("noise_params.accBiasPrior", 0.0);
  this->declare_parameter("noise_params.gyrBiasPrior", 0.0);
  this->declare_parameter("noise_params.initialPositionNoiseDensity", 0.0);
  this->declare_parameter("noise_params.initialOrientationNoiseDensity", 0.0);
  this->declare_parameter("noise_params.initialVelocityNoiseDensity", 0.0);
  this->declare_parameter("noise_params.initialAccBiasNoiseDensity", 0.0);
  this->declare_parameter("noise_params.initialGyroBiasNoiseDensity", 0.0);

  // Re-linearization Params
  this->declare_parameter("relinearization_params.positionReLinTh", 0.0);
  this->declare_parameter("relinearization_params.rotationReLinTh", 0.0);
  this->declare_parameter("relinearization_params.velocityReLinTh", 0.0);
  this->declare_parameter("relinearization_params.accBiasReLinTh", 0.0);
  this->declare_parameter("relinearization_params.gyrBiasReLinTh", 0.0);
  this->declare_parameter("relinearization_params.referenceFrameReLinTh", 0.0);
  this->declare_parameter("relinearization_params.calibrationReLinTh", 0.0);
  this->declare_parameter("relinearization_params.displacementReLinTh", 0.0);
  this->declare_parameter("relinearization_params.landmarkReLinTh", 0.0);
  this->declare_parameter("relinearization_params.relinearizeSkip", 0);
  this->declare_parameter("relinearization_params.enableRelinearization", false);
  this->declare_parameter("graph_params.evaluateNonlinearError", false);
  this->declare_parameter("relinearization_params.cacheLinearizedFactors", false);
  this->declare_parameter("relinearization_params.enablePartialRelinearizationCheck", false);

  // Common Params
  this->declare_parameter("common_params.verbosity", 0);
  this->declare_parameter("common_params.odomNotJumpAtStart", false);
  this->declare_parameter("common_params.logRealTimeStateToMemory", false);
  this->declare_parameter("common_params.logLatencyAndUpdateDurationToMemory", false);

  // Extrinsic frames
  this->declare_parameter("extrinsics.worldFrame", std::string(""));
  this->declare_parameter("extrinsics.odomFrame", std::string(""));
  this->declare_parameter("extrinsics.imuFrame", std::string(""));
  this->declare_parameter("extrinsics.initializeZeroYawAndPositionOfFrame", std::string(""));
  this->declare_parameter("extrinsics.baseLinkFrame", std::string(""));

  // Name IDs
  this->declare_parameter("name_ids.referenceFrameAligned", std::string(""));
  this->declare_parameter("name_ids.sensorFrameCorrected", std::string(""));

  // Logging path
  this->declare_parameter("launch.optimizationResultLoggingPath", std::string(""));
}

}  // namespace graph_msf
