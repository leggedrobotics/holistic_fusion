/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "anymal_estimator_graph/AnymalEstimator.h"

// Project
#include "anymal_estimator_graph/AnymalStaticTransforms.h"

// Workspace
#include "anymal_estimator_graph/constants.h"
#include "graph_msf/interface/eigen_wrapped_gtsam_utils.h"
#include "graph_msf/interface/input_output.h"
#include "graph_msf/measurements/BinaryMeasurementXD.h"
#include "graph_msf/measurements/UnaryMeasurementXD.h"
#include "graph_msf/measurements/UnaryMeasurementXDAbsolute.h"
#include "graph_msf_ros/util/conversions.h"

namespace anymal_se {

// Constexpr from header
constexpr std::array<const char*, 4> AnymalEstimator::legNames_;

AnymalEstimator::AnymalEstimator(const std::shared_ptr<ros::NodeHandle>& privateNodePtr) : graph_msf::GraphMsfRos(privateNodePtr) {
  REGULAR_COUT << GREEN_START << " AnymalEstimatorGraph-Constructor called." << COLOR_END << std::endl;

  // Configurations ----------------------------
  // Static transforms
  staticTransformsPtr_ = std::make_shared<AnymalStaticTransforms>(privateNodePtr);

  // Set up
  AnymalEstimator::setup();
}

void AnymalEstimator::setup() {
  REGULAR_COUT << GREEN_START << " AnymalEstimator-Setup called." << COLOR_END << std::endl;

  // Read parameters ----------------------------
  AnymalEstimator::readParams(privateNode);

  // Super class
  GraphMsfRos::setup(staticTransformsPtr_);

  // Wait for static transforms ----------------------------
  staticTransformsPtr_->findTransformations();

  // Publishers ----------------------------
  AnymalEstimator::initializePublishers(privateNode);

  // Subscribers ----------------------------
  AnymalEstimator::initializeSubscribers(privateNode);

  // Messages ----------------------------
  AnymalEstimator::initializeMessages(privateNode);

  // Services ----------------------------
  AnymalEstimator::initializeServices_(privateNode);
}

void AnymalEstimator::initializePublishers(ros::NodeHandle& privateNode) {
  // Paths
  pubMeasMapLioPath_ = privateNode.advertise<nav_msgs::Path>("/graph_msf/measLiDAR_path_map_lidar", ROS_QUEUE_SIZE);
  pubMeasWorldGnssPath_ = privateNode.advertise<nav_msgs::Path>("/graph_msf/measGnss_path_world_gnss", ROS_QUEUE_SIZE);

  // Markers
  pubFootContactMarkers_ = privateNode.advertise<visualization_msgs::MarkerArray>("/graph_msf/foot_contact_markers", ROS_QUEUE_SIZE);
}

void AnymalEstimator::initializeSubscribers(ros::NodeHandle& privateNode) {
  // GNSS
  if (useGnssUnaryFlag_) {
    subGnssUnary_ = privateNode.subscribe<sensor_msgs::NavSatFix>("/gnss_topic", ROS_QUEUE_SIZE, &AnymalEstimator::gnssUnaryCallback_, this,
                                                                  ros::TransportHints().tcpNoDelay());
    REGULAR_COUT << " Initialized Gnss subscriber with topic: " << subGnssUnary_.getTopic() << std::endl;
  }

  // LiDAR Odometry
  // Unary
  if (useLioUnaryFlag_) {
    subLioUnary_ = privateNode.subscribe<nav_msgs::Odometry>("/lidar_odometry_topic", ROS_QUEUE_SIZE, &AnymalEstimator::lidarUnaryCallback_,
                                                             this, ros::TransportHints().tcpNoDelay());

    REGULAR_COUT << COLOR_END << " Initialized LiDAR Unary Factor Odometry subscriber with topic: " << subLioUnary_.getTopic() << std::endl;
  }
  // Between
  if (useLioBetweenFlag_) {
    subLioBetween_ = privateNode.subscribe<nav_msgs::Odometry>(
        "/lidar_odometry_topic", ROS_QUEUE_SIZE, &AnymalEstimator::lidarBetweenCallback_, this, ros::TransportHints().tcpNoDelay());
    REGULAR_COUT << COLOR_END << " Initialized LiDAR Odometry subscriber with topic: " << subLioBetween_.getTopic() << std::endl;
  }

  // Legged Odometry
  // Between
  if (useLeggedBetweenFlag_) {
    subLeggedBetween_ = privateNode.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
        "/legged_odometry_pose_topic", ROS_QUEUE_SIZE, &AnymalEstimator::leggedBetweenCallback_, this, ros::TransportHints().tcpNoDelay());
    REGULAR_COUT << COLOR_END << " Initialized Legged Odometry subscriber with topic: " << subLeggedBetween_.getTopic() << std::endl;
  }
  // Unary
  if (useLeggedVelocityUnaryFlag_) {
    subLeggedVelocityUnary_ =
        privateNode.subscribe<nav_msgs::Odometry>("/legged_odometry_odom_topic", ROS_QUEUE_SIZE,
                                                  &AnymalEstimator::leggedVelocityUnaryCallback_, this, ros::TransportHints().tcpNoDelay());
    REGULAR_COUT << COLOR_END
                 << " Initialized Legged Velocity Unary Factor Odometry subscriber with topic: " << subLeggedVelocityUnary_.getTopic()
                 << std::endl;
  }
  // Kinematics
  if (useLeggedKinematicsFlag_) {
    subLeggedKinematics_ = privateNode.subscribe<anymal_msgs::AnymalState>(
        "/anymal_state_topic", ROS_QUEUE_SIZE, &AnymalEstimator::leggedKinematicsCallback_, this, ros::TransportHints().tcpNoDelay());
    REGULAR_COUT << COLOR_END << " Initialized Legged Kinematics subscriber with topic: " << subLeggedKinematics_.getTopic() << std::endl;
  }
}

void AnymalEstimator::initializeMessages(ros::NodeHandle& privateNode) {
  // Path
  measLio_mapLidarPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  measGnss_worldGnssPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
}

void AnymalEstimator::initializeServices_(ros::NodeHandle& privateNode) {
  // Nothing
  // TODO: add soft reset of the graph for on-the-go re-init.
}

// Offline Optimization Service
bool AnymalEstimator::srvOfflineSmootherOptimizeCallback(graph_msf_ros_msgs::OfflineOptimizationTrigger::Request& req,
                                                         graph_msf_ros_msgs::OfflineOptimizationTrigger::Response& res) {
  // Call super class
  bool success = graph_msf::GraphMsfRos::srvOfflineSmootherOptimizeCallback(req, res);

  // if GNSS is used at all, also log the reference frame
  if (gnssCallbackCounter_ > 0) {
    // Get the reference coordinates of the GNSS handler
    const double referenceLatitude = gnssHandlerPtr_->getGnssReferenceLatitude();
    const double referenceLongitude = gnssHandlerPtr_->getGnssReferenceLongitude();
    const double referenceAltitude = gnssHandlerPtr_->getGnssReferenceAltitude();
    const double referenceHeading = gnssHandlerPtr_->getGnssReferenceHeading();
    REGULAR_COUT << " GNSS reference (latitude, longitude, altitude, heading): " << referenceLatitude << ", " << referenceLongitude << ", "
                 << referenceAltitude << ", " << referenceHeading << std::endl;

    // File streams
    std::map<std::string, std::ofstream> fileStreams;

    // Write T_totalStation_totalStationOld_ to file ----------------------------
    // Get time string by finding latest directory in optimizationResultLoggingPath
    std::string timeString = graph_msf::getLatestSubdirectory(optimizationResultLoggingPath);
    if (timeString.empty()) {
      REGULAR_COUT << " Could not find latest directory in " << optimizationResultLoggingPath << std::endl;
      return false;
    } else {
      REGULAR_COUT << " Found latest directory in " << optimizationResultLoggingPath << ": " << timeString << std::endl;
    }
    // Create file stream
    std::string transformIdentifier = "gnss_reference_lat_lon_alt";
    graph_msf::createLatLonAltCsvFileStream(fileStreams, optimizationResultLoggingPath, transformIdentifier, timeString);
    // Add to file
    graph_msf::writeLatLonAltToCsvFile(fileStreams, Eigen::Vector3d(referenceLatitude, referenceLongitude, referenceAltitude),
                                       transformIdentifier, ros::Time::now().toSec());
    REGULAR_COUT << " Wrote GNSS reference (latitude, longitude, altitude) coordinates to file." << std::endl;

    // If optional GNSS ENU coordinates are logged, write them to pose file
    if constexpr (logOptionalGnssEnuCoordinatesFlag_) {
      transformIdentifier = "gnss_enu_trajectory";
      graph_msf::createPose3CsvFileStream(fileStreams, optimizationResultLoggingPath, transformIdentifier, timeString, false);
      // Iterate over all optional GNSS ENU coordinates
      int index = 0;
      for (const auto& optionalGnssEnuCoordinates : optionalGnssEnuCoordinatesTrajectory_) {
        // Create Isometry3d with identity rotation
        Eigen::Isometry3d optionalGnssEnuCoordinatesIsometry = Eigen::Isometry3d::Identity();
        optionalGnssEnuCoordinatesIsometry.translation() = optionalGnssEnuCoordinates;
        // Add to file
        graph_msf::writePose3ToCsvFile(fileStreams, optionalGnssEnuCoordinatesIsometry, transformIdentifier,
                                       optionalGnssEnuCoordinatesTimeStamps_[index], false);
        index++;
      }
    }
  }

  // Return
  return success;
}

// Priority: 1
void AnymalEstimator::gnssUnaryCallback_(const sensor_msgs::NavSatFix::ConstPtr& gnssMsgPtr) {
  // Counter
  ++gnssCallbackCounter_;

  // Convert to Eigen
  Eigen::Vector3d gnssCoord = Eigen::Vector3d(gnssMsgPtr->latitude, gnssMsgPtr->longitude, gnssMsgPtr->altitude);
  Eigen::Vector3d estStdDevXYZ(sqrt(gnssMsgPtr->position_covariance[0]), sqrt(gnssMsgPtr->position_covariance[4]),
                               sqrt(gnssMsgPtr->position_covariance[8]));

  // Initialize GNSS Handler
  if (gnssCallbackCounter_ < NUM_GNSS_CALLBACKS_UNTIL_START) {  // Accumulate measurements
    // Wait until measurements got accumulated
    accumulatedGnssCoordinates_ += gnssCoord;
    if ((gnssCallbackCounter_ % 10) == 0) {
      REGULAR_COUT << " NOT ENOUGH GNSS MESSAGES ARRIVED!" << std::endl;
    }
    return;
  } else if (gnssCallbackCounter_ == NUM_GNSS_CALLBACKS_UNTIL_START) {  // Initialize GNSS Handler
    gnssHandlerPtr_->initHandler(accumulatedGnssCoordinates_ / NUM_GNSS_CALLBACKS_UNTIL_START);
    REGULAR_COUT << " GNSS Handler initialized." << std::endl;
    return;
  }

  // Convert to Cartesian Coordinates
  Eigen::Vector3d W_t_W_Gnss;
  // gnssHandlerPtr_->convertNavSatToPosition(gnssCoord, W_t_W_Gnss);
  gnssHandlerPtr_->convertNavSatToPositionLV03(gnssCoord, W_t_W_Gnss);
  std::string fixedFrame = staticTransformsPtr_->getWorldFrame();  // Alias
  // fixedFrame = "east_north_up";

  //  // For Debugging: Add Gaussian Noise with 0.1m std deviation
  //  // Random number generator
  //  std::default_random_engine generator(std::chrono::system_clock::now().time_since_epoch().count());
  //  // Add noise
  //  W_t_W_Gnss +=
  //      Eigen::Vector3d(std::normal_distribution<double>(0.0, 0.1)(generator), std::normal_distribution<double>(0.0, 0.1)(generator),
  //                      std::normal_distribution<double>(0.0, 0.1)(generator));
  //  // Add to assumed standard deviation (\sigma_{GNSS+noise} = \sqrt{\sigma_{GNSS}^2 + \sigma_{noise}^2})
  //  for (int i = 0; i < 3; ++i) {
  //    estStdDevXYZ(i) = sqrt(estStdDevXYZ(i) * estStdDevXYZ(i) + 0.1 * 0.1);
  //  }

  // Initial world yaw initialization options
  // Case 1: Initialization
  if (!areYawAndPositionInited()) {
    // a: Default
    double initYaw_W_Base{0.0};  // Default is 0 yaw
    // b: From file
    if (gnssHandlerPtr_->getUseYawInitialGuessFromFile()) {
      initYaw_W_Base = gnssHandlerPtr_->getGlobalYawDegFromFile() / 180.0 * M_PI;
    }
    // c: From alignment
    else if (gnssHandlerPtr_->getUseYawInitialGuessFromAlignment()) {
      // Adding the GNSS measurement
      trajectoryAlignmentHandler_->addR3Position(W_t_W_Gnss, gnssMsgPtr->header.stamp.toSec());
      // In radians
      Eigen::Isometry3d T_W_Base = Eigen::Isometry3d::Identity();
      if (!(trajectoryAlignmentHandler_->alignTrajectories(initYaw_W_Base, T_W_Base))) {
        if (gnssCallbackCounter_ % 10 == 0) {
          REGULAR_COUT << YELLOW_START << "Trajectory alignment not ready. Waiting for more motion." << COLOR_END << std::endl;
        }
        return;
      }
      REGULAR_COUT << GREEN_START << "Trajectory Alignment Successful. Obtained Yaw Value of T_W_Base (deg): " << COLOR_END
                   << 180.0 * initYaw_W_Base / M_PI << std::endl;
    }

    // Actual Initialization
    if (this->initYawAndPositionInWorld(initYaw_W_Base, W_t_W_Gnss,
                                        dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getBaseLinkFrame(),
                                        dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getGnssFrame())) {
      REGULAR_COUT << GREEN_START << " GNSS initialization of yaw and position successful." << std::endl;

    } else {
      REGULAR_COUT << RED_START << " GNSS initialization of yaw and position failed." << std::endl;
    }
  } else {  // Case 2: Already initialized --> Unary factor
    const std::string& gnssFrameName = dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getGnssFrame();  // Alias
    // Measurement
    graph_msf::UnaryMeasurementXDAbsolute<Eigen::Vector3d, 3> meas_W_t_W_Gnss(
        "GnssPosition", int(gnssRate_), gnssFrameName, gnssFrameName + sensorFrameCorrectedNameId, graph_msf::RobustNorm::None(),
        gnssMsgPtr->header.stamp.toSec(), 1.0, W_t_W_Gnss, estStdDevXYZ, fixedFrame, staticTransformsPtr_->getWorldFrame());
    this->addUnaryPosition3AbsoluteMeasurement(meas_W_t_W_Gnss);
  }

  // Add _gmsf to the frame
  if (fixedFrame != staticTransformsPtr_->getWorldFrame()) {
    fixedFrame += referenceFrameAlignedNameId;
  }

  /// Add GNSS to Path
  addToPathMsg(measGnss_worldGnssPathPtr_, fixedFrame, gnssMsgPtr->header.stamp, W_t_W_Gnss, graphConfigPtr_->imuBufferLength_ * 4);
  /// Publish path
  pubMeasWorldGnssPath_.publish(measGnss_worldGnssPathPtr_);

  // Potentially log the GNSS ENU coordinates
  if (logOptionalGnssEnuCoordinatesFlag_) {
    optionalGnssEnuCoordinatesTimeStamps_.push_back(gnssMsgPtr->header.stamp.toSec());
    optionalGnssEnuCoordinatesTrajectory_.push_back(W_t_W_Gnss);
  }
}

// Priority: 2
void AnymalEstimator::lidarUnaryCallback_(const nav_msgs::Odometry::ConstPtr& odomLidarPtr) {
  // Counter
  ++lidarUnaryCallbackCounter_;

  // Prepare Data
  Eigen::Isometry3d lio_T_M_Lk = Eigen::Isometry3d::Identity();
  graph_msf::odomMsgToEigen(*odomLidarPtr, lio_T_M_Lk.matrix());
  double lidarUnaryTimeK = odomLidarPtr->header.stamp.toSec();

  if (useGnssUnaryFlag_ && gnssHandlerPtr_->getUseYawInitialGuessFromAlignment()) {
    trajectoryAlignmentHandler_->addSe3Position(lio_T_M_Lk.translation(), lidarUnaryTimeK);
  }

  // Measurement
  const std::string& lioOdomFrameName = dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getLioOdometryFrame();  // alias
  graph_msf::UnaryMeasurementXDAbsolute<Eigen::Isometry3d, 6> unary6DMeasurement(
      "Lidar_unary_6D", int(lioOdometryRate_), lioOdomFrameName, lioOdomFrameName + sensorFrameCorrectedNameId,
      graph_msf::RobustNorm::None(), lidarUnaryTimeK, 1.0, lio_T_M_Lk, lioPoseUnaryNoise_, odomLidarPtr->header.frame_id,
      staticTransformsPtr_->getWorldFrame(), initialSe3AlignmentNoise_, lioSe3AlignmentRandomWalk_);

  if (lidarUnaryCallbackCounter_ <= 2) {
    return;
  } else if (!areYawAndPositionInited()) {  // Initializing if no GNSS
    if (!useGnssUnaryFlag_) {
      REGULAR_COUT << GREEN_START << " LiDAR odometry callback is setting global yaw, as it was not set so far." << COLOR_END << std::endl;
      this->initYawAndPosition(unary6DMeasurement);
    }
  } else {  // Already initialized --> unary factor
    this->addUnaryPose3AbsoluteMeasurement(unary6DMeasurement);
  }

  // Visualization ----------------------------
  // Add to path message
  addToPathMsg(measLio_mapLidarPathPtr_, odomLidarPtr->header.frame_id + referenceFrameAlignedNameId, odomLidarPtr->header.stamp,
               lio_T_M_Lk.translation(), graphConfigPtr_->imuBufferLength_ * 4);

  // Publish Path
  pubMeasMapLioPath_.publish(measLio_mapLidarPathPtr_);
}

// Priority: 3
void AnymalEstimator::lidarBetweenCallback_(const nav_msgs::Odometry::ConstPtr& odomLidarPtr) {
  if (!areRollAndPitchInited()) {
    return;
  }

  // Counter
  ++lidarBetweenCallbackCounter_;

  // Convert
  Eigen::Isometry3d lio_T_M_Lk = Eigen::Isometry3d::Identity();
  graph_msf::odomMsgToEigen(*odomLidarPtr, lio_T_M_Lk.matrix());
  // Get the time
  double lidarBetweenTimeK = odomLidarPtr->header.stamp.toSec();

  // At start
  if (lidarBetweenCallbackCounter_ == 0) {
    lio_T_M_Lkm1_ = lio_T_M_Lk;
    lidarBetweenTimeKm1_ = lidarBetweenTimeK;
  }

  // Add to trajectory aligner if needed.
  if (useGnssUnaryFlag_ && gnssHandlerPtr_->getUseYawInitialGuessFromAlignment()) {
    trajectoryAlignmentHandler_->addSe3Position(lio_T_M_Lk.translation(), lidarBetweenTimeK);
  }

  // Frame Name
  const std::string& lioOdomFrameName = dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getLioOdometryFrame();  // alias

  // State Machine
  if (lidarBetweenCallbackCounter_ <= 2) {
    return;
  } else if (!areYawAndPositionInited()) {  // Initializing
    if (!useGnssUnaryFlag_ && !useLioUnaryFlag_) {
      // Measurement
      graph_msf::UnaryMeasurementXD<Eigen::Isometry3d, 6> unary6DMeasurement(
          "Lidar_unary_6D", int(lioOdometryRate_), lioOdomFrameName, lioOdomFrameName + sensorFrameCorrectedNameId,
          graph_msf::RobustNorm::None(), lidarBetweenTimeK, 1.0, lio_T_M_Lk, lioPoseUnaryNoise_);
      // Add to graph
      REGULAR_COUT << GREEN_START << " LiDAR odometry callback is setting global yaw, as it was not set so far." << COLOR_END << std::endl;
      this->initYawAndPosition(unary6DMeasurement);
    }
  } else {  // Already initialized --> Between factor
    // Compute Delta
    const Eigen::Isometry3d T_Lkm1_Lk = lio_T_M_Lkm1_.inverse() * lio_T_M_Lk;
    // Create measurement
    graph_msf::BinaryMeasurementXD<Eigen::Isometry3d, 6> delta6DMeasurement(
        "Lidar_between_6D", int(lioOdometryRate_), lioOdomFrameName, lioOdomFrameName + sensorFrameCorrectedNameId,
        graph_msf::RobustNorm::None(), lidarBetweenTimeKm1_, lidarBetweenTimeK, T_Lkm1_Lk, lioPoseUnaryNoise_);
    // Add to graph
    this->addBinaryPose3Measurement(delta6DMeasurement);
  }
  // Provide for next iteration
  lio_T_M_Lkm1_ = lio_T_M_Lk;
  lidarBetweenTimeKm1_ = lidarBetweenTimeK;

  // Visualization ----------------------------
  // Add to path message
  addToPathMsg(measLio_mapLidarPathPtr_, staticTransformsPtr_->getWorldFrame(), odomLidarPtr->header.stamp, lio_T_M_Lk.translation(),
               graphConfigPtr_->imuBufferLength_ * 4);

  // Publish Path
  pubMeasMapLioPath_.publish(measLio_mapLidarPathPtr_);
}

// Priority: 4
void AnymalEstimator::leggedBetweenCallback_(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& leggedOdometryPoseKPtr) {
  if (!areRollAndPitchInited()) {
    return;
  }

  // Counter
  ++leggedOdometryPoseCallbackCounter_;

  // Eigen Type
  Eigen::Isometry3d T_O_Bl_k = Eigen::Isometry3d::Identity();
  graph_msf::geometryPoseToEigen(*leggedOdometryPoseKPtr, T_O_Bl_k.matrix());
  double legOdometryTimeK = leggedOdometryPoseKPtr->header.stamp.toSec();

  // At start
  if (leggedOdometryPoseCallbackCounter_ == 0) {
    T_O_Bl_km1_ = T_O_Bl_k;
    legOdometryTimeKm1_ = legOdometryTimeK;
    return;
  }

  // Frame Name
  const std::string& leggedOdometryFrameName =
      dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getLeggedOdometryFrame();  // alias

  // State Machine
  if (!areYawAndPositionInited()) {
    if (!useGnssUnaryFlag_ && !useLioUnaryFlag_ && !useLioBetweenFlag_) {
      // Measurement
      graph_msf::UnaryMeasurementXD<Eigen::Isometry3d, 6> unary6DMeasurement(
          "Leg_odometry_6D", int(leggedOdometryBetweenRate_), leggedOdometryFrameName, leggedOdometryFrameName + sensorFrameCorrectedNameId,
          graph_msf::RobustNorm::None(), legOdometryTimeK, 1.0, T_O_Bl_k, legPoseBetweenNoise_);
      // Add to graph
      REGULAR_COUT << GREEN_START << " Legged odometry between callback is setting global yaw, as it was not set so far." << COLOR_END
                   << std::endl;
      this->initYawAndPosition(unary6DMeasurement);
    }
  } else {
    // Only add every 40th measurement
    int measurementRate = static_cast<int>(leggedOdometryBetweenRate_) / leggedOdometryPoseDownsampleFactor_;
    // Check
    if ((leggedOdometryPoseCallbackCounter_ % leggedOdometryPoseDownsampleFactor_) == 0) {
      // Compute Delta
      const Eigen::Isometry3d T_Bkm1_Bk = T_O_Bl_km1_.inverse() * T_O_Bl_k;
      // Create measurement
      graph_msf::BinaryMeasurementXD<Eigen::Isometry3d, 6> delta6DMeasurement(
          "Leg_odometry_6D", measurementRate, leggedOdometryFrameName, leggedOdometryFrameName + sensorFrameCorrectedNameId,
          graph_msf::RobustNorm::None(), legOdometryTimeKm1_, legOdometryTimeK, T_Bkm1_Bk, legPoseBetweenNoise_);
      // Add to graph
      this->addBinaryPose3Measurement(delta6DMeasurement);

      // Prepare for next iteration
      T_O_Bl_km1_ = T_O_Bl_k;
      legOdometryTimeKm1_ = legOdometryTimeK;
    }
  }
}

// Priority: 5
void AnymalEstimator::leggedVelocityUnaryCallback_(const nav_msgs::Odometry ::ConstPtr& leggedOdometryKPtr) {
  if (!areRollAndPitchInited()) {
    return;
  }

  // Counter
  ++leggedOdometryOdomCallbackCounter_;

  if (!areYawAndPositionInited()) {
    if (!useGnssUnaryFlag_ && !useLioUnaryFlag_ && !useLioBetweenFlag_ && !useLeggedBetweenFlag_) {
      // Measurement
      graph_msf::UnaryMeasurementXD<Eigen::Isometry3d, 6> unary6DMeasurement(
          "Leg_odometry_6D", int(leggedOdometryVelocityRate_),
          dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getLeggedOdometryFrame(),
          dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getLeggedOdometryFrame() + sensorFrameCorrectedNameId,
          graph_msf::RobustNorm::None(), leggedOdometryKPtr->header.stamp.toSec(), 1.0, Eigen::Isometry3d::Identity(),
          legPoseBetweenNoise_);
      // Add to graph
      REGULAR_COUT << GREEN_START << " Legged odometry velocity callback is setting global yaw, as it was not set so far." << COLOR_END
                   << std::endl;
      this->initYawAndPosition(unary6DMeasurement);
    }
  } else {
    // Only add every nth measurement
    int measurementRate = static_cast<int>(leggedOdometryVelocityRate_) / leggedOdometryVelocityDownsampleFactor_;

    // Check
    if ((leggedOdometryOdomCallbackCounter_ % leggedOdometryVelocityDownsampleFactor_) == 0) {
      // Eigen Type
      Eigen::Vector3d legVelocity = Eigen::Vector3d(leggedOdometryKPtr->twist.twist.linear.x, leggedOdometryKPtr->twist.twist.linear.y,
                                                    leggedOdometryKPtr->twist.twist.linear.z);

      // Alias
      const std::string& leggedOdometryFrameName =
          dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getLeggedOdometryFrame();

      // Create the unary measurement
      graph_msf::UnaryMeasurementXD<Eigen::Vector3d, 3> legVelocityUnaryMeasurement(
          "LegVelocityUnary", measurementRate, leggedOdometryFrameName, leggedOdometryFrameName + sensorFrameCorrectedNameId,
          graph_msf::RobustNorm::None(), leggedOdometryKPtr->header.stamp.toSec(), 1.0, legVelocity, legVelocityUnaryNoise_);

      // Add to graph
      this->addUnaryVelocity3LocalMeasurement(legVelocityUnaryMeasurement);
    }
  }
}

void AnymalEstimator::leggedKinematicsCallback_(const anymal_msgs::AnymalState::ConstPtr& anymalStatePtr) {
  if (!areRollAndPitchInited()) {
    return;
  }

  // Counter
  ++leggedKinematicsCallbackCounter_;

  if (!areYawAndPositionInited()) {
    if (!useGnssUnaryFlag_ && !useLioUnaryFlag_ && !useLioBetweenFlag_ && !useLeggedBetweenFlag_ && !useLeggedVelocityUnaryFlag_) {
      // Measurement
      graph_msf::UnaryMeasurementXD<Eigen::Isometry3d, 6> unary6DMeasurement(
          "Leg_odometry_6D", int(leggedKinematicsRate_),
          dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getLeggedOdometryFrame(),
          dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getLeggedOdometryFrame() + sensorFrameCorrectedNameId,
          graph_msf::RobustNorm::None(), anymalStatePtr->header.stamp.toSec(), 1.0, Eigen::Isometry3d::Identity(),
          Eigen::Matrix<double, 6, 1>::Identity());
      // Add to graph
      REGULAR_COUT << GREEN_START << " Legged kinematics callback is setting global yaw, as it was not set so far." << COLOR_END
                   << std::endl;
      this->initYawAndPosition(unary6DMeasurement);
    }
  }
  // Normal Operation
  else {
    // Only add every nth measurement
    int measurementRate = static_cast<int>(leggedKinematicsRate_) / leggedKinematicsDownsampleFactor_;

    // Check whether it is time to add a measurement
    if ((leggedKinematicsCallbackCounter_ % leggedKinematicsDownsampleFactor_) == 0) {
      // Alias
      const std::string& leggedOdometryFrameName =
          dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getLeggedOdometryFrame();

      // Pose of the base in odom frame
      Eigen::Isometry3d T_O_B = Eigen::Isometry3d::Identity();
      graph_msf::geometryPoseToEigen(anymalStatePtr->pose.pose, T_O_B.matrix());

      // Create the unary measurement for each foot (loop unrolling with constexpr arrays) ----------------------------
      visualization_msgs::MarkerArray footContactMarkers = visualization_msgs::MarkerArray();
      // Loop
      for (int legIndex = 0; legIndex < legNames_.size(); ++legIndex) {
        // Get contact state
        bool footInContact = anymalStatePtr->contacts[legIndex].state;
        // Foot name as specified in the message
        const std::string legName = anymalStatePtr->contacts[legIndex].name;

        // Check whether leg is in contact ----------------------------
        if (footInContact) {
          // Increase debounce counter
          ++legInContactForNSteps_[legIndex];

          // Check whether leg was at least N times in contact to count it as in contact --> debouncing
          if (legInContactForNSteps_[legIndex] >= legInContactDebounceThreshold_) {
            // If it was not in contact before
            if (legInContactForNSteps_[legIndex] == legInContactDebounceThreshold_) {
              // Increase contact counter
              ++legTotalContactsCounter_[legIndex];
              // Print
              if (graphConfigPtr_->verboseLevel_ > 1) {
                REGULAR_COUT << " Leg " << legName << " came back into contact for the " << legTotalContactsCounter_[legIndex] << ". time."
                             << std::endl;
              }
            }

            // Add measurement ----------------------------
            // Position of foot in legged odometry frame
            Eigen::Vector3d O_t_O_foot =
                Eigen::Vector3d(anymalStatePtr->contacts[legIndex].position.x, anymalStatePtr->contacts[legIndex].position.y,
                                anymalStatePtr->contacts[legIndex].position.z);
            Eigen::Vector3d B_t_B_foot = T_O_B.inverse() * O_t_O_foot;

            // Create the unary measurement with contact counter
            std::string legIdentifier = legName;
            graph_msf::UnaryMeasurementXDLandmark<Eigen::Vector3d, 3> footContactPositionMeasurement(
                legIdentifier, measurementRate, leggedOdometryFrameName, leggedOdometryFrameName + sensorFrameCorrectedNameId,
                graph_msf::RobustNorm::None(), anymalStatePtr->header.stamp.toSec(), 1.0, B_t_B_foot, //graph_msf::RobustNorm::Huber(1)
                legKinematicsFootPositionUnaryNoise_, staticTransformsPtr_->getWorldFrame());

            // Add to graph
            this->addUnaryPosition3LandmarkMeasurement(footContactPositionMeasurement, legTotalContactsCounter_[legIndex]);

            // Visualize foot contact in RViz
            visualization_msgs::Marker footContactMarker;
            createContactMarker(leggedOdometryFrameName, anymalStatePtr->header.stamp, B_t_B_foot, "foot_contact", legIndex,
                                Eigen::Vector3d(0.0, 1.0, 0.0), footContactMarker);
            footContactMarkers.markers.push_back(footContactMarker);
          }
        }
        // Leg not in contact ----------------------------
        else if (legInContactForNSteps_[legIndex] > 0) {
          // Set to 0
          legInContactForNSteps_[legIndex] = 0;
          // Print
          if (graphConfigPtr_->verboseLevel_ > 1) {
            REGULAR_COUT << " Leg " << legName << " lost contact. " << std::endl;
          }
          // Remove marker if it lost contact
          visualization_msgs::Marker deleteMarker;
          deleteMarker.header.frame_id = leggedOdometryFrameName;
          deleteMarker.header.stamp = anymalStatePtr->header.stamp;
          deleteMarker.ns = "foot_contact";
          deleteMarker.id = legIndex;
          deleteMarker.action = visualization_msgs::Marker::DELETE;
          footContactMarkers.markers.push_back(deleteMarker);
        }
      }
      // Publish foot contact markers
      if (!footContactMarkers.markers.empty() && pubFootContactMarkers_.getNumSubscribers() > 0) {
        pubFootContactMarkers_.publish(footContactMarkers);
      }
    }
  }
}

}  // namespace anymal_se