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

  // Strings
  std::string imuGravityDirection = "up";

  // Sensor Config
  bool usingGnssFlag = true;
  bool usingLioFlag = true;

  // Sensor Params
  double imuRate = 100;
  int imuBufferLength = 200;
  double imuTimeOffset = 0.0;

  // Initialization
  bool estimateGravityFromImuFlag = true;

  // Factor Graph
  bool useIsamFlag = true;
  double smootherLag = 1.5;
  int additionalOptimizationIterations = 0;
  bool findUnusedFactorSlotsFlag = false;
  bool enableDetailedResultsFlag = false;
  bool usingFallbackGraphFlag = true;
  bool usingCholeskyFactorizationFlag = true;

  // Outlier Rejection
  double poseMotionOutlierThresold = 0.3;

  // Noise Params
  // Position
  double accNoiseDensity = 1e-8;
  double integrationNoiseDensity = 1.0;
  bool use2ndOrderCoriolisFlag = true;
  // Rotation
  double gyroNoiseDensity = 1e-8;
  double omegaCoriolis = 0.0;
  // Bias
  double accBiasRandomWalk = 1e-8;
  double gyroBiasRandomWalk = 1e-8;
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
