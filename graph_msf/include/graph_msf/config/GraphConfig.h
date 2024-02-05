/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPHCONFIG_H
#define GRAPHCONFIG_H

#include <Eigen/Eigen>

namespace graph_msf {

struct GraphConfig {
  GraphConfig() {}

  // General Config
  int verboseLevel = 0;
  bool reLocalizeWorldToMapAtStartFlag = false;

  // Sensor Config
  bool relocalizationAtStartFlag = true;

  // Sensor Params
  double imuRate = 100;
  bool useImuSignalLowPassFilter = true;
  double imuLowPassFilterCutoffFreqHz = 60;
  int imuBufferLength = 200;
  double imuTimeOffset = 0.0;

  // Gravity
  bool estimateGravityFromImuFlag = true;
  double gravityMagnitude = 9.81;
  Eigen::Vector3d W_gravityVector = Eigen::Vector3d(0.0, 0.0, -1) * gravityMagnitude;

  // Factor Graph
  bool realTimeSmootherUseIsamFlag = true;
  double realTimeSmootherLag = 1.5;
  bool useAdditionalSlowBatchSmoother = false;
  bool slowBatchSmootherUseIsamFlag = true;
  // Optimizer Config
  double gaussNewtonWildfireThreshold = 0.001;
  double maxOptimizationFrequency = 100;
  int additionalOptimizationIterations = 0;
  bool findUnusedFactorSlotsFlag = false;
  bool enableDetailedResultsFlag = false;
  bool usingCholeskyFactorizationFlag = true;
  bool usingBiasForPreIntegrationFlag = true;
  bool optimizeFixedFramePosesWrtWorld = true;
  double fixedFramePosesResetThreshold = 5.0;
  bool optimizeWithImuToSensorLeverArm = true;
  bool optimizeExtrinsicSensorToSensorCorrectedOffset = false;

  // Outlier Rejection
  double poseMotionOutlierThresold = 0.3;

  // Noise Params (Noise Amplitude Spectral Density)
  // Position
  double accNoiseDensity = 1e-3;  // [m/s^2/√Hz)]
  double integrationNoiseDensity = 1.0e-4;
  bool use2ndOrderCoriolisFlag = true;
  // Rotation
  double gyroNoiseDensity = 1e-4;  // [rad/s/√Hz]
  double omegaCoriolis = 0.0;
  // Bias
  double accBiasRandomWalkNoiseDensity = 1e-4;   // [m/s^3/√Hz]
  double gyroBiasRandomWalkNoiseDensity = 1e-5;  // [rad/s^2/√Hz]
  double biasAccOmegaInit = 0.0;
  Eigen::Vector3d accBiasPrior = Eigen::Vector3d(0.0, 0.0, 0.0);
  Eigen::Vector3d gyroBiasPrior = Eigen::Vector3d(0.0, 0.0, 0.0);
  // Preintegration

  // Relinearization
  // Thresholds
  double positionReLinTh = 1e-3;
  double rotationReLinTh = 1e-3;
  double velocityReLinTh = 1e-3;
  double accBiasReLinTh = 1e-3;
  double gyroBiasReLinTh = 1e-3;
  double fixedFrameReLinTh = 1e-3;
  double displacementReLinTh = 1e-3;
  // Flags
  int relinearizeSkip = 0;
  bool enableRelinearizationFlag = true;
  bool evaluateNonlinearErrorFlag = true;
  bool cacheLinearizedFactorsFlag = true;
  bool enablePartialRelinearizationCheckFlag = true;

  // Other
  double maxSearchDeviation = 0.01;
};

}  // namespace graph_msf

#endif  // GRAPHCONFIG_H
