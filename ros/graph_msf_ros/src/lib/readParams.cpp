/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// C++
#include <boost/filesystem.hpp>

// Implementation
#include "graph_msf_ros/GraphMsfRos.h"

// Workspace
#include "graph_msf_ros/ros/read_ros_params.h"

namespace graph_msf {

void GraphMsfRos::readParams_(const ros::NodeHandle& privateNode) {
  std::cout << YELLOW_START << "GraphMsfRos" << GREEN_START << " Reading parameters." << COLOR_END << std::endl;

  if (!graphConfigPtr_) {
    throw std::runtime_error("GraphMsfRos: graphConfigPtr must be initialized.");
  }

  // Configuration
  // Sensor Parameters
  graphConfigPtr_->imuRate_ = tryGetParam<double>("sensor_params/imuRate", privateNode);
  graphConfigPtr_->createStateEveryNthImuMeasurement_ = tryGetParam<int>("sensor_params/createStateEveryNthImuMeasurement", privateNode);
  graphConfigPtr_->useImuSignalLowPassFilter_ = tryGetParam<bool>("sensor_params/useImuSignalLowPassFilter", privateNode);
  graphConfigPtr_->imuLowPassFilterCutoffFreqHz_ = tryGetParam<double>("sensor_params/imuLowPassFilterCutoffFreq", privateNode);
  graphConfigPtr_->maxSearchDeviation_ = 1.0 / (graphConfigPtr_->imuRate_ / graphConfigPtr_->createStateEveryNthImuMeasurement_);
  graphConfigPtr_->imuBufferLength_ = tryGetParam<int>("sensor_params/imuBufferLength", privateNode);
  graphConfigPtr_->imuTimeOffset_ = tryGetParam<double>("sensor_params/imuTimeOffset", privateNode);

  // Initialization Params
  graphConfigPtr_->estimateGravityFromImuFlag_ = tryGetParam<bool>("initialization_params/estimateGravityFromImu", privateNode);
  graphConfigPtr_->gravityMagnitude_ = tryGetParam<double>("initialization_params/gravityMagnitude", privateNode);

  // Graph Params
  graphConfigPtr_->realTimeSmootherUseIsamFlag_ = tryGetParam<bool>("graph_params/realTimeSmootherUseIsam", privateNode);
  graphConfigPtr_->realTimeSmootherLag_ = tryGetParam<double>("graph_params/realTimeSmootherLag", privateNode);
  graphConfigPtr_->useAdditionalSlowBatchSmoother_ = tryGetParam<bool>("graph_params/useAdditionalSlowBatchSmoother", privateNode);
  graphConfigPtr_->slowBatchSmootherUseIsamFlag_ = tryGetParam<bool>("graph_params/slowBatchSmootherUseIsam", privateNode);
  // Optimizer Config
  graphConfigPtr_->gaussNewtonWildfireThreshold_ = tryGetParam<double>("graph_params/gaussNewtonWildfireThreshold", privateNode);
  graphConfigPtr_->minOptimizationFrequency_ = tryGetParam<double>("graph_params/minOptimizationFrequency", privateNode);
  graphConfigPtr_->maxOptimizationFrequency_ = tryGetParam<double>("graph_params/maxOptimizationFrequency", privateNode);
  graphConfigPtr_->additionalOptimizationIterations_ = tryGetParam<int>("graph_params/additionalOptimizationIterations", privateNode);
  graphConfigPtr_->findUnusedFactorSlotsFlag_ = tryGetParam<bool>("graph_params/findUnusedFactorSlots", privateNode);
  graphConfigPtr_->enableDetailedResultsFlag_ = tryGetParam<bool>("graph_params/enableDetailedResults", privateNode);
  graphConfigPtr_->usingCholeskyFactorizationFlag_ = tryGetParam<bool>("graph_params/usingCholeskyFactorization", privateNode);
  graphConfigPtr_->usingBiasForPreIntegrationFlag_ = tryGetParam<bool>("graph_params/usingBiasForPreIntegration", privateNode);
  graphConfigPtr_->optimizeFixedFramePosesWrtWorld_ = tryGetParam<bool>("graph_params/optimizeFixedFramePosesWrtWorld", privateNode);
  graphConfigPtr_->optimizeExtrinsicSensorToSensorCorrectedOffset_ =
      tryGetParam<bool>("graph_params/optimizeExtrinsicSensorToSensorCorrectedOffset", privateNode);
  // Alignment Parameters
  graphConfigPtr_->fixedFramePosesResetThreshold_ = tryGetParam<double>("graph_params/fixedFramePosesResetThreshold", privateNode);
  graphConfigPtr_->centerMeasurementsAtRobotPositionBeforeAlignment_ =
      tryGetParam<bool>("graph_params/centerMeasurementsAtRobotPositionBeforeAlignment", privateNode);

  // Noise Parameters
  /// IMU
  //// Position
  graphConfigPtr_->accNoiseDensity_ = tryGetParam<double>("noise_params/accNoiseDensity", privateNode);
  graphConfigPtr_->integrationNoiseDensity_ = tryGetParam<double>("noise_params/integrationNoiseDensity", privateNode);
  graphConfigPtr_->use2ndOrderCoriolisFlag_ = tryGetParam<bool>("noise_params/use2ndOrderCoriolis", privateNode);
  //// Rotation
  graphConfigPtr_->gyroNoiseDensity_ = tryGetParam<double>("noise_params/gyrNoiseDensity", privateNode);
  graphConfigPtr_->omegaCoriolis_ = tryGetParam<double>("noise_params/omegaCoriolis", privateNode);
  //// Bias
  graphConfigPtr_->accBiasRandomWalkNoiseDensity_ = tryGetParam<double>("noise_params/accBiasRandomWalkNoiseDensity", privateNode);
  graphConfigPtr_->gyroBiasRandomWalkNoiseDensity_ = tryGetParam<double>("noise_params/gyrBiasRandomWalkNoiseDensity", privateNode);
  graphConfigPtr_->biasAccOmegaInit_ = tryGetParam<double>("noise_params/biasAccOmegaInit", privateNode);
  const double accBiasPrior = tryGetParam<double>("noise_params/accBiasPrior", privateNode);
  graphConfigPtr_->accBiasPrior_ = Eigen::Vector3d(accBiasPrior, accBiasPrior, accBiasPrior);
  const double gyroBiasPrior = tryGetParam<double>("noise_params/gyrBiasPrior", privateNode);
  graphConfigPtr_->gyroBiasPrior_ = Eigen::Vector3d(gyroBiasPrior, gyroBiasPrior, gyroBiasPrior);
  // Initial State
  graphConfigPtr_->initialPositionNoiseDensity_ = tryGetParam<double>("noise_params/initialPositionNoiseDensity", privateNode);
  graphConfigPtr_->initialOrientationNoiseDensity_ = tryGetParam<double>("noise_params/initialOrientationNoiseDensity", privateNode);
  graphConfigPtr_->initialVelocityNoiseDensity_ = tryGetParam<double>("noise_params/initialVelocityNoiseDensity", privateNode);
  graphConfigPtr_->initialAccBiasNoiseDensity_ = tryGetParam<double>("noise_params/initialAccBiasNoiseDensity", privateNode);
  graphConfigPtr_->initialGyroBiasNoiseDensity_ = tryGetParam<double>("noise_params/initialGyroBiasNoiseDensity", privateNode);

  // Re-linearization
  /// Thresholds
  graphConfigPtr_->positionReLinTh_ = tryGetParam<double>("relinearization_params/positionReLinTh", privateNode);
  graphConfigPtr_->rotationReLinTh_ = tryGetParam<double>("relinearization_params/rotationReLinTh", privateNode);
  graphConfigPtr_->velocityReLinTh_ = tryGetParam<double>("relinearization_params/velocityReLinTh", privateNode);
  graphConfigPtr_->accBiasReLinTh_ = tryGetParam<double>("relinearization_params/accBiasReLinTh", privateNode);
  graphConfigPtr_->gyroBiasReLinTh_ = tryGetParam<double>("relinearization_params/gyrBiasReLinTh", privateNode);
  graphConfigPtr_->fixedFrameReLinTh_ = tryGetParam<double>("relinearization_params/fixedFrameReLinTh", privateNode);
  graphConfigPtr_->displacementReLinTh_ = tryGetParam<double>("relinearization_params/displacementReLinTh", privateNode);
  /// Others
  graphConfigPtr_->relinearizeSkip_ = tryGetParam<int>("relinearization_params/relinearizeSkip", privateNode);
  graphConfigPtr_->enableRelinearizationFlag_ = tryGetParam<bool>("relinearization_params/enableRelinearization", privateNode);
  graphConfigPtr_->evaluateNonlinearErrorFlag_ = tryGetParam<bool>("relinearization_params/evaluateNonlinearError", privateNode);
  graphConfigPtr_->cacheLinearizedFactorsFlag_ = tryGetParam<bool>("relinearization_params/cacheLinearizedFactors", privateNode);
  graphConfigPtr_->enablePartialRelinearizationCheckFlag_ =
      tryGetParam<bool>("relinearization_params/enablePartialRelinearizationCheck", privateNode);

  // Common Parameters
  graphConfigPtr_->verboseLevel_ = tryGetParam<int>("common_params/verbosity", privateNode);
  graphConfigPtr_->odomNotJumpAtStart_ = tryGetParam<bool>("common_params/odomNotJumpAtStart", privateNode);

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

  // Name IDs
  fixedFrameAlignedNameId_ = tryGetParam<std::string>("name_ids/fixedFrameAligned", privateNode);
  sensorFrameCorrectedNameId_ = tryGetParam<std::string>("name_ids/sensorFrameCorrected", privateNode);

  // Logging Path
  optimizationResultLoggingPath_ = tryGetParam<std::string>("launch/optimizationResultLoggingPath", privateNode);
  // Make sure the path ends with a slash
  if (optimizationResultLoggingPath_.back() != '/') {
    optimizationResultLoggingPath_ += "/";
  }
  // Create directory if it does not exist
  if (!boost::filesystem::exists(optimizationResultLoggingPath_)) {
    boost::filesystem::create_directories(optimizationResultLoggingPath_);
  }
}

}  // namespace graph_msf
