/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_CONFIG_H
#define GRAPH_CONFIG_H

#include <Eigen/Eigen>

namespace graph_msf {

struct GraphConfig {
  GraphConfig() {}

  // General Config
  int verboseLevel_ = 0;
  bool odomNotJumpAtStartFlag_ = false;
  bool logRealTimeStateToMemoryFlag_ = false;

  // Sensor Params
  double imuRate_ = 100;
  int createStateEveryNthImuMeasurement_ = 1;
  bool useImuSignalLowPassFilter_ = true;
  double imuLowPassFilterCutoffFreqHz_ = 60;
  int imuBufferLength_ = 200;
  double imuTimeOffset_ = 0.0;

  // Gravity
  bool estimateGravityFromImuFlag_ = true;
  double gravityMagnitude_ = 9.81;
  Eigen::Vector3d W_gravityVector_ = Eigen::Vector3d(0.0, 0.0, -1) * gravityMagnitude_;

  // Factor Graph
  double realTimeSmootherLag_ = 1.5;
  bool realTimeSmootherUseIsamFlag_ = true;
  bool realTimeSmootherUseCholeskyFactorizationFlag_ = true;
  bool useAdditionalSlowBatchSmootherFlag_ = false;
  bool slowBatchSmootherUseIsamFlag_ = false;
  bool slowBatchSmootherUseCholeskyFactorizationFlag_ = false;
  // Optimizer Config
  double gaussNewtonWildfireThreshold_ = 0.001;
  double minOptimizationFrequency_ = 5;
  double maxOptimizationFrequency_ = 100;
  int additionalOptimizationIterations_ = 0;
  bool findUnusedFactorSlotsFlag_ = false;
  bool enableDetailedResultsFlag_ = false;
  bool usingBiasForPreIntegrationFlag_ = true;
  bool useWindowForMarginalsComputationFlag_ = true;
  double windowSizeSecondsForMarginalsComputation_ = 300.0;
  // Alignment Parameters
  bool optimizeReferenceFramePosesWrtWorldFlag_ = true;
  double referenceFramePosesResetThreshold_ = 0.5;
  bool centerMeasurementsAtKeyframePositionBeforeAlignmentFlag_ = false;
  double createReferenceAlignmentKeyframeEveryNSeconds_ = 10.0;  // [s]
  // Calibration
  bool optimizeExtrinsicSensorToSensorCorrectedOffsetFlag_ = false;

  // Noise Params (Noise Amplitude Spectral Density)
  // Position
  double accNoiseDensity_ = 1e-03;  // [m/s^2/√Hz)]
  double integrationNoiseDensity_ = 1.0e-04;
  bool use2ndOrderCoriolisFlag_ = true;
  // Rotation
  double gyroNoiseDensity_ = 1e-04;  // [rad/s/√Hz]
  double omegaCoriolis_ = 0.0;
  // Bias
  double accBiasRandomWalkNoiseDensity_ = 1e-04;   // [m/s^3/√Hz]
  double gyroBiasRandomWalkNoiseDensity_ = 1e-05;  // [rad/s^2/√Hz]
  double biasAccOmegaInit_ = 0.0;
  Eigen::Vector3d accBiasPrior_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  Eigen::Vector3d gyroBiasPrior_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  // Initial State
  double initialPositionNoiseDensity_ = 1e-01;
  double initialOrientationNoiseDensity_ = 1e-01;
  double initialVelocityNoiseDensity_ = 1e-01;
  double initialAccBiasNoiseDensity_ = 1e-01;
  double initialGyroBiasNoiseDensity_ = 1e-01;

  // Relinearization
  // Thresholds
  double positionReLinTh_ = 1e-3;
  double rotationReLinTh_ = 1e-3;
  double velocityReLinTh_ = 1e-3;
  double accBiasReLinTh_ = 1e-3;
  double gyroBiasReLinTh_ = 1e-3;
  double referenceFrameReLinTh_ = 1e-3;
  double calibrationReLinTh_ = 1e-3;
  double displacementReLinTh_ = 1e-3;
  double landmarkReLinTh_ = 1e-3;
  // Flags
  int relinearizeSkip_ = 0;
  bool enableRelinearizationFlag_ = true;
  bool evaluateNonlinearErrorFlag_ = true;
  bool cacheLinearizedFactorsFlag_ = true;
  bool enablePartialRelinearizationCheckFlag_ = true;

  // Other
  double maxSearchDeviation_ = 0.01;
};

}  // namespace graph_msf

#endif  // GRAPH_CONFIG_H
