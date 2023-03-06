#include "graph_msf_ros/GraphMsfRos.h"

namespace graph_msf {

void GraphMsfRos::readParams_(const ros::NodeHandle& privateNode) {
  // Variables for parameter fetching
  double dParam;
  int iParam;
  bool bParam;
  std::string sParam;

  if (!graphConfigPtr_) {
    throw std::runtime_error("GraphMsfRos: graphConfigPtr must be initialized.");
  }

  // Configuration
  /// Using Gnss
  graphConfigPtr_->usingGnssFlag = tryGetParam<bool>("launch/usingGnss", privateNode);

  // Set frames
  /// World
  std::string frame = tryGetParam<std::string>("extrinsics/worldFrame", privateNode);
  staticTransformsPtr_->setWorldFrame(frame);
  /// Map
  frame = tryGetParam<std::string>("extrinsics/mapFrame", privateNode);
  staticTransformsPtr_->setMapFrame(frame);
  /// Odom
  frame = tryGetParam<std::string>("extrinsics/odomFrame", privateNode);
  staticTransformsPtr_->setOdomFrame(frame);
  /// IMU
  //// Cabin IMU
  frame = tryGetParam<std::string>("extrinsics/imuFrame", privateNode);
  staticTransformsPtr_->setImuFrame(frame);

  // IMU gravity definition
  graphConfigPtr_->imuGravityDirection = tryGetParam<std::string>("launch/imuGravityDirection", privateNode);

  // Sensor Parameters
  graphConfigPtr_->imuRate = tryGetParam<double>("sensor_params/imuRate", privateNode);
  graphConfigPtr_->maxSearchDeviation = 1.0 / graphConfigPtr_->imuRate;
  graphConfigPtr_->imuBufferLength = tryGetParam<int>("sensor_params/imuBufferLength", privateNode);
  graphConfigPtr_->imuTimeOffset = tryGetParam<double>("sensor_params/imuTimeOffset", privateNode);

  // Initialization Params
  graphConfigPtr_->estimateGravityFromImuFlag = tryGetParam<bool>("initialization_params/estimateGravityFromImu", privateNode);

  // Graph Params
  graphConfigPtr_->useIsamFlag = tryGetParam<bool>("graph_params/useIsam", privateNode);
  graphConfigPtr_->smootherLag = tryGetParam<double>("graph_params/smootherLag", privateNode);
  graphConfigPtr_->additionalOptimizationIterations = tryGetParam<int>("graph_params/additionalOptimizationIterations", privateNode);
  graphConfigPtr_->findUnusedFactorSlotsFlag = tryGetParam<bool>("graph_params/findUnusedFactorSlots", privateNode);
  graphConfigPtr_->enableDetailedResultsFlag = tryGetParam<bool>("graph_params/enableDetailedResults", privateNode);
  graphConfigPtr_->usingFallbackGraphFlag = tryGetParam<bool>("graph_params/usingFallbackGraph", privateNode);
  graphConfigPtr_->usingCholeskyFactorizationFlag = tryGetParam<bool>("graph_params/usingCholeskyFactorization", privateNode);

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
  graphConfigPtr_->accBiasRandomWalk = tryGetParam<double>("noise_params/accBiasRandomWalk", privateNode);
  graphConfigPtr_->gyroBiasRandomWalk = tryGetParam<double>("noise_params/gyrBiasRandomWalk", privateNode);
  graphConfigPtr_->biasAccOmegaInit = tryGetParam<double>("noise_params/biasAccOmegaInit", privateNode);
  const double accBiasPrior = tryGetParam<double>("noise_params/accBiasPrior", privateNode);
  graphConfigPtr_->accBiasPrior = Eigen::Vector3d(accBiasPrior, accBiasPrior, accBiasPrior);
  const double gyroBiasPrior = tryGetParam<double>("noise_params/gyrBiasPrior", privateNode);
  graphConfigPtr_->gyroBiasPrior = Eigen::Vector3d(gyroBiasPrior, gyroBiasPrior, gyroBiasPrior);

  // Re-linearization
  graphConfigPtr_->positionReLinTh = tryGetParam<double>("relinearization_params/positionReLinTh", privateNode);
  graphConfigPtr_->rotationReLinTh = tryGetParam<double>("relinearization_params/rotationReLinTh", privateNode);
  graphConfigPtr_->velocityReLinTh = tryGetParam<double>("relinearization_params/velocityReLinTh", privateNode);
  graphConfigPtr_->accBiasReLinTh = tryGetParam<double>("relinearization_params/accBiasReLinTh", privateNode);
  graphConfigPtr_->gyroBiasReLinTh = tryGetParam<double>("relinearization_params/gyrBiasReLinTh", privateNode);
  graphConfigPtr_->relinearizeSkip = tryGetParam<int>("relinearization_params/relinearizeSkip", privateNode);
  graphConfigPtr_->enableRelinearizationFlag = tryGetParam<bool>("relinearization_params/enableRelinearization", privateNode);
  graphConfigPtr_->evaluateNonlinearErrorFlag = tryGetParam<bool>("relinearization_params/evaluateNonlinearError", privateNode);
  graphConfigPtr_->cacheLinearizedFactorsFlag = tryGetParam<bool>("relinearization_params/cacheLinearizedFactors", privateNode);
  graphConfigPtr_->enablePartialRelinearizationCheckFlag =
      tryGetParam<bool>("relinearization_params/enablePartialRelinearizationCheck", privateNode);

  // Common Parameters
  graphConfigPtr_->verboseLevel = tryGetParam<int>("common_params/verbosity", privateNode);
  graphConfigPtr_->reLocalizeWorldToMapAtStartFlag = tryGetParam<bool>("common_params/reLocalizeWorldToMapAtStart", privateNode);
}

}  // namespace graph_msf
