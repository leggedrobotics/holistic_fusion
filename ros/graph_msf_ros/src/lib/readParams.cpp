/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Workspace
#include "graph_msf_ros/GraphMsfRos.h"
#include "graph_msf_ros/ros/read_ros_params.h"

namespace graph_msf {

void GraphMsfRos::readParams_(const ros::NodeHandle& privateNode) {
  std::cout << YELLOW_START << "GraphMsfRos" << GREEN_START << " Reading parameters." << COLOR_END << std::endl;

  if (!graphConfigPtr_) {
    throw std::runtime_error("GraphMsfRos: graphConfigPtr must be initialized.");
  }

  // Configuration
  // Sensor Parameters
  graphConfigPtr_->imuRate = tryGetParam<double>("sensor_params/imuRate", privateNode);
  graphConfigPtr_->createStateEveryNthImuMeasurement = tryGetParam<int>("sensor_params/createStateEveryNthImuMeasurement", privateNode);
  graphConfigPtr_->useImuSignalLowPassFilter = tryGetParam<bool>("sensor_params/useImuSignalLowPassFilter", privateNode);
  graphConfigPtr_->imuLowPassFilterCutoffFreqHz = tryGetParam<double>("sensor_params/imuLowPassFilterCutoffFreq", privateNode);
  graphConfigPtr_->maxSearchDeviation = 1.0 / (graphConfigPtr_->imuRate / graphConfigPtr_->createStateEveryNthImuMeasurement);
  graphConfigPtr_->imuBufferLength = tryGetParam<int>("sensor_params/imuBufferLength", privateNode);
  graphConfigPtr_->imuTimeOffset = tryGetParam<double>("sensor_params/imuTimeOffset", privateNode);

  // Initialization Params
  graphConfigPtr_->estimateGravityFromImuFlag = tryGetParam<bool>("initialization_params/estimateGravityFromImu", privateNode);
  graphConfigPtr_->gravityMagnitude = tryGetParam<double>("initialization_params/gravityMagnitude", privateNode);

  // Graph Params
  graphConfigPtr_->realTimeSmootherUseIsamFlag = tryGetParam<bool>("graph_params/realTimeSmootherUseIsam", privateNode);
  graphConfigPtr_->realTimeSmootherLag = tryGetParam<double>("graph_params/realTimeSmootherLag", privateNode);
  graphConfigPtr_->useAdditionalSlowBatchSmoother = tryGetParam<bool>("graph_params/useAdditionalSlowBatchSmoother", privateNode);
  graphConfigPtr_->slowBatchSmootherUseIsamFlag = tryGetParam<bool>("graph_params/slowBatchSmootherUseIsam", privateNode);
  // Optimizer Config
  graphConfigPtr_->gaussNewtonWildfireThreshold = tryGetParam<double>("graph_params/gaussNewtonWildfireThreshold", privateNode);
  graphConfigPtr_->maxOptimizationFrequency = tryGetParam<double>("graph_params/maxOptimizationFrequency", privateNode);
  graphConfigPtr_->additionalOptimizationIterations = tryGetParam<int>("graph_params/additionalOptimizationIterations", privateNode);
  graphConfigPtr_->findUnusedFactorSlotsFlag = tryGetParam<bool>("graph_params/findUnusedFactorSlots", privateNode);
  graphConfigPtr_->enableDetailedResultsFlag = tryGetParam<bool>("graph_params/enableDetailedResults", privateNode);
  graphConfigPtr_->usingCholeskyFactorizationFlag = tryGetParam<bool>("graph_params/usingCholeskyFactorization", privateNode);
  graphConfigPtr_->usingBiasForPreIntegrationFlag = tryGetParam<bool>("graph_params/usingBiasForPreIntegration", privateNode);
  graphConfigPtr_->optimizeFixedFramePosesWrtWorld = tryGetParam<bool>("graph_params/optimizeFixedFramePosesWrtWorld", privateNode);
  graphConfigPtr_->fixedFramePosesResetThreshold = tryGetParam<double>("graph_params/fixedFramePosesResetThreshold", privateNode);
  graphConfigPtr_->optimizeWithImuToSensorLeverArm = tryGetParam<bool>("graph_params/optimizeWithImuToSensorLeverArm", privateNode);
  graphConfigPtr_->optimizeExtrinsicSensorToSensorCorrectedOffset =
      tryGetParam<bool>("graph_params/optimizeExtrinsicSensorToSensorCorrectedOffset", privateNode);

  // Outlier Parameters
  graphConfigPtr_->poseMotionOutlierThresold = tryGetParam<double>("outlier_params/poseMotionOutlierThreshold", privateNode);

  // Noise Parameters
  /// Position
  graphConfigPtr_->accNoiseDensity = tryGetParam<double>("noise_params/accNoiseDensity", privateNode);
  graphConfigPtr_->integrationNoiseDensity = tryGetParam<double>("noise_params/integrationNoiseDensity", privateNode);
  graphConfigPtr_->use2ndOrderCoriolisFlag = tryGetParam<bool>("noise_params/use2ndOrderCoriolis", privateNode);
  /// Rotation
  graphConfigPtr_->gyroNoiseDensity = tryGetParam<double>("noise_params/gyrNoiseDensity", privateNode);
  graphConfigPtr_->omegaCoriolis = tryGetParam<double>("noise_params/omegaCoriolis", privateNode);
  /// Bias
  graphConfigPtr_->accBiasRandomWalkNoiseDensity = tryGetParam<double>("noise_params/accBiasRandomWalkNoiseDensity", privateNode);
  graphConfigPtr_->gyroBiasRandomWalkNoiseDensity = tryGetParam<double>("noise_params/gyrBiasRandomWalkNoiseDensity", privateNode);
  graphConfigPtr_->biasAccOmegaInit = tryGetParam<double>("noise_params/biasAccOmegaInit", privateNode);
  const double accBiasPrior = tryGetParam<double>("noise_params/accBiasPrior", privateNode);
  graphConfigPtr_->accBiasPrior = Eigen::Vector3d(accBiasPrior, accBiasPrior, accBiasPrior);
  const double gyroBiasPrior = tryGetParam<double>("noise_params/gyrBiasPrior", privateNode);
  graphConfigPtr_->gyroBiasPrior = Eigen::Vector3d(gyroBiasPrior, gyroBiasPrior, gyroBiasPrior);

  // Re-linearization
  /// Thresholds
  graphConfigPtr_->positionReLinTh = tryGetParam<double>("relinearization_params/positionReLinTh", privateNode);
  graphConfigPtr_->rotationReLinTh = tryGetParam<double>("relinearization_params/rotationReLinTh", privateNode);
  graphConfigPtr_->velocityReLinTh = tryGetParam<double>("relinearization_params/velocityReLinTh", privateNode);
  graphConfigPtr_->accBiasReLinTh = tryGetParam<double>("relinearization_params/accBiasReLinTh", privateNode);
  graphConfigPtr_->gyroBiasReLinTh = tryGetParam<double>("relinearization_params/gyrBiasReLinTh", privateNode);
  graphConfigPtr_->fixedFrameReLinTh = tryGetParam<double>("relinearization_params/fixedFrameReLinTh", privateNode);
  graphConfigPtr_->displacementReLinTh = tryGetParam<double>("relinearization_params/displacementReLinTh", privateNode);
  /// Others
  graphConfigPtr_->relinearizeSkip = tryGetParam<int>("relinearization_params/relinearizeSkip", privateNode);
  graphConfigPtr_->enableRelinearizationFlag = tryGetParam<bool>("relinearization_params/enableRelinearization", privateNode);
  graphConfigPtr_->evaluateNonlinearErrorFlag = tryGetParam<bool>("relinearization_params/evaluateNonlinearError", privateNode);
  graphConfigPtr_->cacheLinearizedFactorsFlag = tryGetParam<bool>("relinearization_params/cacheLinearizedFactors", privateNode);
  graphConfigPtr_->enablePartialRelinearizationCheckFlag =
      tryGetParam<bool>("relinearization_params/enablePartialRelinearizationCheck", privateNode);

  // Common Parameters
  graphConfigPtr_->verboseLevel = tryGetParam<int>("common_params/verbosity", privateNode);
  graphConfigPtr_->odomNotJumpAtStart = tryGetParam<bool>("common_params/odomNotJumpAtStart", privateNode);

  // Set frames
  /// World
  staticTransformsPtr_->setWorldFrame(tryGetParam<std::string>("extrinsics/worldFrame", privateNode));
  /// Odom
  staticTransformsPtr_->setOdomFrame(tryGetParam<std::string>("extrinsics/odomFrame", privateNode));
  /// IMU
  //// Cabin IMU
  staticTransformsPtr_->setImuFrame(tryGetParam<std::string>("extrinsics/imuFrame", privateNode));
  // Initialization Frame
  staticTransformsPtr_->setInitializationFrame(tryGetParam<std::string>("extrinsics/initializeZeroYawAndPositionOfFrame", privateNode));
  // Base frame
  staticTransformsPtr_->setBaseLinkFrame(tryGetParam<std::string>("extrinsics/baseLinkFrame", privateNode));
}

}  // namespace graph_msf
