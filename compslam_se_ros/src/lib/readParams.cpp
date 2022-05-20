#include "compslam_se_ros/CompslamEstimator.h"

namespace compslam_se {

void CompslamEstimator::readParams_(const ros::NodeHandle& privateNode) {
  // Variables for parameter fetching
  double dParam;
  int iParam;
  bool bParam;
  std::string sParam;

  if (!graphConfigPtr_) {
    throw std::runtime_error("CompslamSeInterface: graphConfigPtr must be initialized.");
  }

  // Set frames
  /// Map
  if (privateNode.getParam("extrinsics/mapFrame", sParam)) {
    ROS_INFO_STREAM("CompslamSe - Map frame set to: " << sParam);
    staticTransformsPtr_->setMapFrame(sParam);
  }
  /// Odom
  if (privateNode.getParam("extrinsics/odomFrame", sParam)) {
    ROS_INFO_STREAM("CompslamSe - Odom frame set to: " << sParam);
    staticTransformsPtr_->setOdomFrame(sParam);
  } else {
    ROS_WARN("CompslamSe - Odom frame not set");
  }

  /// IMU
  //// Cabin IMU
  if (privateNode.getParam("extrinsics/imuFrame", sParam)) {
    ROS_INFO_STREAM("CompslamSe - IMU frame for preintegrator and tf: " << sParam);
    staticTransformsPtr_->setImuFrame(sParam);
  } else
    ROS_WARN("CompslamSe - IMU Cabin frame not set for preintegrator");
  /// LiDAR frame
  if (privateNode.getParam("extrinsics/lidarFrame", sParam)) {
    ROS_INFO_STREAM("CompslamSe - LiDAR frame: " << sParam);
    staticTransformsPtr_->setLidarFrame(sParam);
  } else {
    ROS_WARN("CompslamSe - LiDAR frame not set");
  }

  /// Left GNSS frame
  if (privateNode.getParam("extrinsics/leftGnssFrame", sParam)) {
    ROS_INFO_STREAM("CompslamSe - left GNSS frame: " << sParam);
    staticTransformsPtr_->setLeftGnssFrame(sParam);
  } else {
    ROS_WARN("CompslamSe - left GNSS frame not set");
  }
  /// Right GNSS frame
  if (privateNode.getParam("extrinsics/rightGnssFrame", sParam)) {
    ROS_INFO_STREAM("CompslamSe - right GNSS frame: " << sParam);
    staticTransformsPtr_->setRightGnssFrame(sParam);
  } else {
    ROS_WARN("CompslamSe - right GNSS frame not set");
  }

  // IMU gravity definition
  if (privateNode.getParam("launch/imu_gravity_direction", sParam)) {
    ROS_INFO_STREAM("CompslamSe - gravity direction of IMU: " << sParam);
    graphConfigPtr_->imuGravityDirection = sParam;
  } else {
    ROS_ERROR("CompslamSe - gravity direction of imu not set");
    throw std::runtime_error("Rosparam 'launch/imu_gravity_direction' must be set.");
  }

  // Using Compslam
  if (privateNode.getParam("launch/using_compslam", bParam)) {
    ROS_INFO_STREAM("CompslamSe - using Compslam: " << bParam);
    graphConfigPtr_->usingLioFlag = bParam;
  } else {
    ROS_ERROR("CompslamSe - using Compslam not set.");
    throw std::runtime_error("Rosparam 'launch/using_compslam' must be set.");
  }

  // Factor Graph Parameters
  if (privateNode.getParam("sensor_params/imuRate", dParam)) {
    ROS_INFO_STREAM("CompslamSe - IMU rate for preintegrator: " << dParam);
    graphConfigPtr_->imuRate = dParam;
  }
  if (privateNode.getParam("sensor_params/imuBufferLength", iParam)) {
    ROS_INFO_STREAM("CompslamSe - IMU buffer length: " << iParam);
    graphConfigPtr_->imuBufferLength = iParam;
  }
  if (privateNode.getParam("sensor_params/lidarRate", dParam)) {
    ROS_INFO_STREAM("CompslamSe - LiDAR rate: " << dParam);
    lidarRate_ = dParam;
  }
  if (privateNode.getParam("sensor_params/gnssRate", dParam)) {
    ROS_INFO_STREAM("CompslamSe - GNSS rate: " << dParam);
    gnssLeftRate_ = dParam;
    gnssRightRate_ = dParam;
  }
  if (privateNode.getParam("sensor_params/imuTimeOffset", dParam)) {
    graphConfigPtr_->imuTimeOffset = dParam;
  }
  if (privateNode.getParam("graph_params/smootherLag", dParam)) {
    graphConfigPtr_->smootherLag = dParam;
  }
  if (privateNode.getParam("graph_params/additonalIterations", iParam)) {
    graphConfigPtr_->additionalIterations = iParam;
  }
  if (privateNode.getParam("graph_params/findUnusedFactorSlots", bParam)) {
    graphConfigPtr_->findUnusedFactorSlots = bParam;
  }
  if (privateNode.getParam("graph_params/enableDetailedResults", bParam)) {
    graphConfigPtr_->enableDetailedResults = bParam;
  }
  if (privateNode.getParam("graph_params/usingFallbackGraph", bParam)) {
    ROS_INFO_STREAM("CompslamSe - usingFallbackGraph: " << bParam);
    graphConfigPtr_->usingFallbackGraphFlag = bParam;
  }

  // Outlier Parameters
  if (privateNode.getParam("outlier_params/gnssOutlierThreshold", dParam)) {
    gnssOutlierThreshold_ = dParam;
  }

  // Noise Parameters
  /// Accelerometer
  if (privateNode.getParam("noise_params/accNoiseDensity", dParam)) {
    graphConfigPtr_->accNoiseDensity = dParam;
  }
  if (privateNode.getParam("noise_params/accBiasRandomWalk", dParam)) {
    graphConfigPtr_->accBiasRandomWalk = dParam;
  }
  if (privateNode.getParam("noise_params/accBiasPrior", dParam)) {
    graphConfigPtr_->accBiasPrior = Eigen::Vector3d(dParam, dParam, dParam);
  }
  /// Gyro
  if (privateNode.getParam("noise_params/gyrNoiseDensity", dParam)) {
    graphConfigPtr_->gyroNoiseDensity = dParam;
  }
  if (privateNode.getParam("noise_params/gyrBiasRandomWalk", dParam)) {
    graphConfigPtr_->gyroBiasRandomWalk = dParam;
  }
  if (privateNode.getParam("noise_params/gyrBiasPrior", dParam)) {
    graphConfigPtr_->gyroBiasPrior = Eigen::Vector3d(dParam, dParam, dParam);
  }
  /// Preintegration
  if (privateNode.getParam("noise_params/integrationNoiseDensity", dParam)) {
    graphConfigPtr_->integrationNoiseDensity = dParam;
  }
  if (privateNode.getParam("noise_params/biasAccOmegaPreint", dParam)) {
    graphConfigPtr_->biasAccOmegaPreint = dParam;
  }
  /// LiDAR
  std::vector<double> poseBetweenNoise;  // roll,pitch,yaw,x,y,z
  if (privateNode.getParam("noise_params/poseBetweenNoise", poseBetweenNoise)) {
    ROS_WARN_STREAM("Set pose between noise to " << poseBetweenNoise[0] << "," << poseBetweenNoise[1] << "," << poseBetweenNoise[2] << ","
                                                 << poseBetweenNoise[3] << "," << poseBetweenNoise[4] << "," << poseBetweenNoise[5]);
    poseBetweenNoise_ << poseBetweenNoise[0], poseBetweenNoise[1], poseBetweenNoise[2], poseBetweenNoise[3], poseBetweenNoise[4],
        poseBetweenNoise[5];
  } else {
    std::runtime_error("poseBetweenNoise needs to be set in config file.");
  }
  std::vector<double> poseUnaryNoise;  // roll,pitch,yaw,x,y,z
  if (privateNode.getParam("noise_params/poseUnaryNoise", poseUnaryNoise)) {
    ROS_WARN_STREAM("Set pose unary noise to " << poseUnaryNoise[0] << "," << poseUnaryNoise[1] << "," << poseUnaryNoise[2] << ","
                                               << poseUnaryNoise[3] << "," << poseUnaryNoise[4] << "," << poseUnaryNoise[5]);
    poseUnaryNoise_ << poseUnaryNoise[0], poseUnaryNoise[1], poseUnaryNoise[2], poseUnaryNoise[3], poseUnaryNoise[4], poseUnaryNoise[5];
  } else {
    std::runtime_error("poseUnaryNoise needs to be set in config file.");
  }
  /// GNSS
  if (privateNode.getParam("noise_params/gnssPositionUnaryNoise", dParam)) {
    ROS_WARN_STREAM("Set gnssPositionUnaryNoise to " << dParam);
    gnssPositionUnaryNoise_ = dParam;
  } else {
    std::runtime_error("gnssPositionUnaryNoise needs to be set in config file.");
  }
  if (privateNode.getParam("noise_params/gnssHeadingUnaryNoise", dParam)) {
    ROS_WARN_STREAM("Set gnssHeadingUnaryNoise to " << dParam);
    gnssHeadingUnaryNoise_ = dParam;
  } else {
    std::runtime_error("gnssHeadingUnaryNoise needs to be set in config file.");
  }

  // Relinearization
  if (privateNode.getParam("relinearization_params/positionReLinTh", dParam)) {
    graphConfigPtr_->positionReLinTh = dParam;
  }
  if (privateNode.getParam("relinearization_params/rotationReLinTh", dParam)) {
    graphConfigPtr_->rotationReLinTh = dParam;
  }
  if (privateNode.getParam("relinearization_params/velocityReLinTh", dParam)) {
    graphConfigPtr_->velocityReLinTh = dParam;
  }
  if (privateNode.getParam("relinearization_params/accBiasReLinTh", dParam)) {
    graphConfigPtr_->accBiasReLinTh = dParam;
  }
  if (privateNode.getParam("relinearization_params/gyrBiasReLinTh", dParam)) {
    graphConfigPtr_->gyroBiasReLinTh = dParam;
  }
  if (privateNode.getParam("relinearization_params/relinearizeSkip", iParam)) {
    graphConfigPtr_->relinearizeSkip = iParam;
  }
  if (privateNode.getParam("relinearization_params/enableRelinearization", bParam)) {
    graphConfigPtr_->enableRelinearization = bParam;
  }
  if (privateNode.getParam("relinearization_params/evaluateNonlinearError", bParam)) {
    graphConfigPtr_->evaluateNonlinearError = bParam;
  }
  if (privateNode.getParam("relinearization_params/cacheLinearizedFactors", bParam)) {
    graphConfigPtr_->cacheLinearizedFactors = bParam;
  }
  if (privateNode.getParam("relinearization_params/enablePartialRelinearizationCheck", bParam)) {
    graphConfigPtr_->enablePartialRelinearizationCheck = bParam;
  }

  // Common Parameters
  if (privateNode.getParam("common_params/verbosity", iParam)) {
    ROS_INFO("Set fg_filtering-Verbosity: %d", iParam);
    verboseLevel_ = iParam;
    graphConfigPtr_->verboseLevel = iParam;
  } else {
    verboseLevel_ = 0;
    graphConfigPtr_->verboseLevel = 0;
  }
  // Common Parameters
  if (privateNode.getParam("common_params/logPlots", bParam)) {
    ROS_INFO("Plotting of plots at end is set to: %d", bParam);
    logPlots_ = bParam;
  } else {
    logPlots_ = false;
  }

  // Using GNSS
  if (privateNode.getParam("launch/using_gps", bParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - using GNSS: " << bParam);
    graphConfigPtr_->usingGnssFlag = bParam;
  } else {
    ROS_ERROR("FactorGraphFiltering - using GNSS not set.");
    throw std::runtime_error("Rosparam 'launch/using_gps' must be set.");
  }
  // GNSS parameters
  if (graphConfigPtr_->usingGnssFlag) {
    if (privateNode.getParam("gnss/use_reference", bParam)) {
      ROS_INFO("Using the GNSS reference is set to: %d", bParam);
      gnssHandlerPtr_->usingGnssReferenceFlag = bParam;
    } else {
      std::runtime_error("Must set gnss/use_reference.");
    }
    if (privateNode.getParam("gnss/reference_latitude", dParam)) {
      ROS_INFO_STREAM("Reference latitude of the scene is given as: " << dParam);
      gnssHandlerPtr_->gnssReferenceLatitude = dParam;
    } else if (gnssHandlerPtr_->usingGnssReferenceFlag) {
      std::runtime_error("GNSS reference latitude must be provided.");
    }
    if (privateNode.getParam("gnss/reference_longitude", dParam)) {
      ROS_INFO_STREAM("Reference longitude of the scene is given as: " << dParam);
      gnssHandlerPtr_->gnssReferenceLongitude = dParam;
    } else if (gnssHandlerPtr_->usingGnssReferenceFlag) {
      std::runtime_error("GNSS reference longitude must be provided.");
    }
    if (privateNode.getParam("gnss/reference_altitude", dParam)) {
      ROS_INFO_STREAM("Reference altitude of the scene is given as: " << dParam);
      gnssHandlerPtr_->gnssReferenceAltitude = dParam;
    } else if (gnssHandlerPtr_->usingGnssReferenceFlag) {
      std::runtime_error("GNSS reference altitude must be provided.");
    }
    if (privateNode.getParam("gnss/reference_heading", dParam)) {
      ROS_INFO_STREAM("Reference heading of the scene is given as: " << dParam);
      gnssHandlerPtr_->gnssReferenceHeading = dParam;
    } else if (gnssHandlerPtr_->usingGnssReferenceFlag) {
      std::runtime_error("GNSS reference heading must be provided.");
    }
  }
}

}  // namespace compslam_se
