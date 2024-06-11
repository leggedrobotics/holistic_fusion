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
#include "graph_msf/measurements/BinaryMeasurementXD.h"
#include "graph_msf/measurements/UnaryMeasurementXD.h"
#include "graph_msf_ros/util/conversions.h"

namespace anymal_se {

AnymalEstimator::AnymalEstimator(const std::shared_ptr<ros::NodeHandle>& privateNodePtr) : graph_msf::GraphMsfRos(privateNodePtr) {
  REGULAR_COUT << GREEN_START << " Initializing..." << COLOR_END << std::endl;

  // Configurations ----------------------------
  // Static transforms
  staticTransformsPtr_ = std::make_shared<AnymalStaticTransforms>(privateNodePtr);

  // Set up
  if (!AnymalEstimator::setup()) {
    REGULAR_COUT << COLOR_END << " Failed to set up." << std::endl;
    throw std::runtime_error("ANYmalEstimatorGraph failed to set up.");
  }

  REGULAR_COUT << GREEN_START << " Set up successfully." << COLOR_END << std::endl;
}

bool AnymalEstimator::setup() {
  REGULAR_COUT << GREEN_START << " Setting up." << COLOR_END << std::endl;

  // Super class
  if (not GraphMsfRos::setup()) {
    throw std::runtime_error("GraphMsfRos could not be initialized");
  }

  // Read parameters ----------------------------
  AnymalEstimator::readParams_(privateNode_);

  // Wait for static transforms ----------------------------
  staticTransformsPtr_->findTransformations();

  // Publishers ----------------------------
  AnymalEstimator::initializePublishers_(privateNode_);

  // Subscribers ----------------------------
  AnymalEstimator::initializeSubscribers_(privateNode_);

  // Messages ----------------------------
  AnymalEstimator::initializeMessages_(privateNode_);

  // Services ----------------------------
  AnymalEstimator::initializeServices_(privateNode_);

  // Wrap up ----------------------------
  REGULAR_COUT << GREEN_START << " Set up successfully." << COLOR_END << std::endl;

  return true;
}

void AnymalEstimator::initializePublishers_(ros::NodeHandle& privateNode) {
  // Paths
  pubMeasMapLioPath_ = privateNode_.advertise<nav_msgs::Path>("/graph_msf/measLiDAR_path_map_lidar", ROS_QUEUE_SIZE);
  pubMeasWorldGnssPath_ = privateNode_.advertise<nav_msgs::Path>("/graph_msf/measGnss_path_world_gnss", ROS_QUEUE_SIZE);
  pubReferenceNavSatFixCoordinates_ =
      privateNode_.advertise<sensor_msgs::NavSatFix>("/graph_msf/gps_reference_position", ROS_QUEUE_SIZE, true);
}

void AnymalEstimator::initializeSubscribers_(ros::NodeHandle& privateNode) {
  // GNSS
  if (useGnssUnaryFlag_) {
    subGnssUnary_ = privateNode_.subscribe<sensor_msgs::NavSatFix>("/gnss_topic", ROS_QUEUE_SIZE, &AnymalEstimator::gnssUnaryCallback_,
                                                                   this, ros::TransportHints().tcpNoDelay());
    REGULAR_COUT << " Initialized Gnss subscriber with topic: " << subGnssUnary_.getTopic() << std::endl;
  }

  // LiDAR Odometry
  // Unary
  if (useLioUnaryFlag_) {
    subLioUnary_ = privateNode_.subscribe<nav_msgs::Odometry>(
        "/lidar_odometry_topic", ROS_QUEUE_SIZE, &AnymalEstimator::lidarUnaryCallback_, this, ros::TransportHints().tcpNoDelay());

    REGULAR_COUT << COLOR_END << " Initialized LiDAR Unary Factor Odometry subscriber with topic: " << subLioUnary_.getTopic() << std::endl;
  }
  // Between
  if (useLioBetweenFlag_) {
    subLioBetween_ = privateNode_.subscribe<nav_msgs::Odometry>(
        "/lidar_odometry_topic", ROS_QUEUE_SIZE, &AnymalEstimator::lidarBetweenCallback_, this, ros::TransportHints().tcpNoDelay());
    REGULAR_COUT << COLOR_END << " Initialized LiDAR Odometry subscriber with topic: " << subLioBetween_.getTopic() << std::endl;
  }

  // Legged Odometry
  // Between
  if (useLeggedBetweenFlag_) {
    subLeggedBetween_ = privateNode_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
        "/legged_odometry_pose_topic", ROS_QUEUE_SIZE, &AnymalEstimator::leggedBetweenCallback_, this, ros::TransportHints().tcpNoDelay());
    REGULAR_COUT << COLOR_END << " Initialized Legged Odometry subscriber with topic: " << subLeggedBetween_.getTopic() << std::endl;
  }
  // Unary
  if (useLeggedVelocityUnaryFlag_) {
    subLeggedVelocityUnary_ = privateNode_.subscribe<nav_msgs::Odometry>("/legged_odometry_odom_topic", ROS_QUEUE_SIZE,
                                                                         &AnymalEstimator::leggedVelocityUnaryCallback_, this,
                                                                         ros::TransportHints().tcpNoDelay());
    REGULAR_COUT << COLOR_END
                 << " Initialized Legged Velocity Unary Factor Odometry subscriber with topic: " << subLeggedVelocityUnary_.getTopic()
                 << std::endl;
  }
}

void AnymalEstimator::initializeMessages_(ros::NodeHandle& privateNode) {
  // Path
  measLio_mapLidarPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  measGnss_worldGnssPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
}

void AnymalEstimator::initializeServices_(ros::NodeHandle& privateNode) {
  // TODO: add soft reset of the graph for on-the-go re-init.

  if (simulated_) {
    toggleSimulatedGPS_ = privateNode_.advertiseService("toggleGPS", &AnymalEstimator::toggleSimulatedGPSCallback, this);
    toggleLIO_ = privateNode_.advertiseService("toggleLIO", &AnymalEstimator::toggleLIOCallback, this);
  }
  return;
}

bool AnymalEstimator::toggleSimulatedGPSCallback(std_srvs::Trigger::Request& /*req*/, std_srvs::Trigger::Response& res) {
  if (useSimulatedGPS_) {
    useSimulatedGPS_ = false;
    REGULAR_COUT << "Simulated GPS is turned off." << std::endl;
  } else {
    useSimulatedGPS_ = true;
    REGULAR_COUT << "Simulated GPS is turned on." << std::endl;
  }
  res.success = true;
  return true;
}

bool AnymalEstimator::toggleLIOCallback(std_srvs::Trigger::Request& /*req*/, std_srvs::Trigger::Response& res) {
  if (useLIO_) {
    useLIO_ = false;
    REGULAR_COUT << "LIO input is turned off." << std::endl;
  } else {
    useLIO_ = true;
    REGULAR_COUT << "LIO input is turned on." << std::endl;
  }
  res.success = true;
  return true;
}

// Priority: 1
void AnymalEstimator::gnssUnaryCallback_(const sensor_msgs::NavSatFix::ConstPtr& gnssMsgPtr) {
  if (!useSimulatedGPS_ && simulated_) {
    return;
  }
  /*
  int8 STATUS_NO_FIX =  -1        # unable to fix position
  int8 STATUS_FIX =      0        # unaugmented fix
  int8 STATUS_SBAS_FIX = 1        # with satellite-based augmentation
  int8 STATUS_GBAS_FIX = 2        # with ground-based augmentation
  */

  // Refer to the piksi multi driver
  // https://github.com/ethz-asl/ethz_piksi_ros/blob/d0ed038078045696661b9bd7789305b7acd20bfa/piksi_multi_cpp/src/sbp_callback_handler/sbp_callback_handler_relay/ros_relays.cc#L35
  if (int(gnssMsgPtr->status.status) < int(sensor_msgs::NavSatStatus::STATUS_GBAS_FIX)) {
    std::cout << YELLOW_START << "AnymalEstimator" << COLOR_END << " GPS state is invalid. "
              << "Expected: " << int(sensor_msgs::NavSatStatus::STATUS_GBAS_FIX) << " Received: " << int(gnssMsgPtr->status.status)
              << std::endl;
    return;
  }

  // Convert to Eigen
  Eigen::Vector3d gnssCoord = Eigen::Vector3d(gnssMsgPtr->latitude, gnssMsgPtr->longitude, gnssMsgPtr->altitude);
  Eigen::Vector3d estStdDevXYZ(sqrt(gnssMsgPtr->position_covariance[0]), sqrt(gnssMsgPtr->position_covariance[4]),
                               sqrt(gnssMsgPtr->position_covariance[8]));

  if ((estStdDevXYZ[0] > 2.0) || (estStdDevXYZ[1] > 2.0)) {
    std::cout << YELLOW_START << "AnymalEstimator" << COLOR_END << " GPS is rejected due to high std deviation. "
              << "Expected: < 2.0 Received: " << estStdDevXYZ[0] << " and " << estStdDevXYZ[1] << " Received: " << std::endl;
    return;
  }

  // Counter
  ++gnssCallbackCounter_;

  // Initialize GNSS Handler
  if (gnssCallbackCounter_ < NUM_GNSS_CALLBACKS_UNTIL_START) {  // Accumulate measurements
    // Wait until measurements got accumulated
    accumulatedGnssCoordinates_ += gnssCoord;
    if (!(gnssCallbackCounter_ % 5)) {
      std::cout << YELLOW_START << "AnymalEstimator" << COLOR_END << " NOT ENOUGH GNSS MESSAGES ARRIVED! See: " << gnssCallbackCounter_
                << " / " << NUM_GNSS_CALLBACKS_UNTIL_START << std::endl;
    }
    return;
  } else if (gnssCallbackCounter_ == NUM_GNSS_CALLBACKS_UNTIL_START) {  // Initialize GNSS Handler
    gnssHandlerPtr_->initHandler(accumulatedGnssCoordinates_ / NUM_GNSS_CALLBACKS_UNTIL_START);

    sensor_msgs::NavSatFix referencefix;
    Eigen::Vector3d referenceFixValues = gnssHandlerPtr_->getGPSReference();
    referencefix.header = gnssMsgPtr->header;
    referencefix.latitude = referenceFixValues(0);
    referencefix.longitude = referenceFixValues(1);
    referencefix.altitude = referenceFixValues(2);
    pubReferenceNavSatFixCoordinates_.publish(referencefix);

    std::cout << YELLOW_START << "AnymalEstimator" << COLOR_END << " GNSS Handler initialized." << std::endl;
    return;
  }

  // Convert to Cartesian Coordinates
  Eigen::Vector3d W_t_W_Gnss;
  gnssHandlerPtr_->convertNavSatToPosition(gnssCoord, W_t_W_Gnss);
  std::string fixedFrame = staticTransformsPtr_->getWorldFrame();
  // fixedFrame = "east_north_up";

  /*Eigen::Vector3d diff = lastHealthyGPSmeasurement_ - W_t_W_Gnss;
  lastHealthyGPSmeasurement_ = W_t_W_Gnss;
  if (diff.norm() > 0.2) {
    std::cout << YELLOW_START << "AnymalEstimator" << COLOR_END << " GPS is jumped more than 20cm. Diff: " << diff.norm() << std::endl;
    return;
  }*/

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

  // Inital world yaw initialization options.
  if (!areYawAndPositionInited()) {  // 1: Initialization
    // 0: Default
    double initYaw_W_Base{0.0};  // Default is 0 yaw
    // 1: From file
    if (gnssHandlerPtr_->useYawInitialGuessFromFile_) {
      initYaw_W_Base = gnssHandlerPtr_->globalYawDegFromFile_ / 180.0 * M_PI;
    } else if (gnssHandlerPtr_->yawInitialGuessFromAlignment_) {  // 2: From alignment
      // Adding the GNSS measurement
      trajectoryAlignmentHandler_->addGnssPose(W_t_W_Gnss, gnssMsgPtr->header.stamp.toSec());
      // In radians.
      if (!(trajectoryAlignmentHandler_->alignTrajectories(initYaw_W_Base))) {
        return;
      }
      std::cout << GREEN_START << "Trajectory Alignment Successful. Obtained Yaw Value : " << COLOR_END << initYaw_W_Base << std::endl;
    }

    // Initialization
    if (not this->initYawAndPosition(initYaw_W_Base, W_t_W_Gnss, staticTransformsPtr_->getWorldFrame(),
                                     dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getBaseLinkFrame(),
                                     dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getGnssFrame())) {
      // Make clear that this was not successful
      REGULAR_COUT << RED_START << " GNSS initialization of yaw and position failed." << std::endl;
    } else {
      REGULAR_COUT << GREEN_START << " GNSS initialization of yaw and position successful." << std::endl;
    }
  } else {  // 2: Unary factor
    const std::string& gnssFrameName = dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getGnssFrame();  // Alias
    // Measurement
    graph_msf::UnaryMeasurementXD<Eigen::Vector3d, 3> meas_W_t_W_Gnss(
        "GnssPosition", int(gnssRate_), gnssFrameName, gnssFrameName + sensorFrameCorrectedNameId_, graph_msf::RobustNormEnum::None, 1.0,
        gnssMsgPtr->header.stamp.toSec(), fixedFrame, 2.0, W_t_W_Gnss, estStdDevXYZ);
    // graph_msf::GraphMsfInterface::addGnssPositionMeasurement_(meas_W_t_W_Gnss);
    this->addUnaryPosition3Measurement(meas_W_t_W_Gnss);
  }

  // Add _gmsf to the frame
  if (fixedFrame != staticTransformsPtr_->getWorldFrame()) {
    fixedFrame += fixedFrameAlignedNameId_;
  }

  /// Add GNSS to Path
  addToPathMsg(measGnss_worldGnssPathPtr_, fixedFrame, gnssMsgPtr->header.stamp, W_t_W_Gnss, graphConfigPtr_->imuBufferLength_ * 4);
  /// Publish path
  pubMeasWorldGnssPath_.publish(measGnss_worldGnssPathPtr_);
}

// Priority: 2
void AnymalEstimator::lidarUnaryCallback_(const nav_msgs::Odometry::ConstPtr& odomLidarPtr) {
  // In simulation the user can toggle LIO on / off.
  if (!useLIO_ && simulated_) {
    return;
  }

  // Currently we are not allowing LIO to initialize the global frame before GPS.
  // TODO, JN is devising a solution.
  if (useGnssUnaryFlag_ && !gnssHandlerPtr_->getGNSSstate()) {
    // if (!(lidarUnaryCallbackCounter_ % 10)) {
    REGULAR_COUT << YELLOW_START << " SLAM is available but GPS is not. Please give me GPS. (Throttled)" << COLOR_END << std::endl;
    //}
    return;
  }

  // Counter
  ++lidarUnaryCallbackCounter_;

  Eigen::Isometry3d lio_T_M_Lk = Eigen::Isometry3d::Identity();
  graph_msf::odomMsgToEigen(*odomLidarPtr, lio_T_M_Lk.matrix());

  // Transform to IMU frame
  double lidarUnaryTimeK = odomLidarPtr->header.stamp.toSec();

  if (useGnssUnaryFlag_ && gnssHandlerPtr_->yawInitialGuessFromAlignment_) {
    trajectoryAlignmentHandler_->addLidarPose(lio_T_M_Lk.translation(), lidarUnaryTimeK);
  }

  // Measurement
  const std::string& lioOdomFrameName = dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getLioOdometryFrame();  // alias
  graph_msf::UnaryMeasurementXD<Eigen::Isometry3d, 6> unary6DMeasurement(
      "Lidar_unary_6D", int(lioOdometryRate_), lioOdomFrameName, lioOdomFrameName + sensorFrameCorrectedNameId_,
      graph_msf::RobustNormEnum::None, 1.0, lidarUnaryTimeK, odomLidarPtr->header.frame_id, 1.0, lio_T_M_Lk, lioPoseUnaryNoise_);

  if (lidarUnaryCallbackCounter_ <= 2) {
    return;
  } else if (!areYawAndPositionInited()) {  // Initializing
    if (!useGnssUnaryFlag_ || (lidarUnaryCallbackCounter_ > NUM_GNSS_CALLBACKS_UNTIL_START)) {
      REGULAR_COUT << GREEN_START << " LiDAR odometry callback is setting global yaw, as it was not set so far." << COLOR_END << std::endl;
      this->initYawAndPosition(unary6DMeasurement);
    }
  } else {  // Already initialized --> unary factor
    this->addUnaryPoseMeasurement(unary6DMeasurement);
  }

  // Visualization ----------------------------
  // Add to path message
  addToPathMsg(measLio_mapLidarPathPtr_, odomLidarPtr->header.frame_id + fixedFrameAlignedNameId_, odomLidarPtr->header.stamp,
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
  if (useGnssUnaryFlag_ && gnssHandlerPtr_->yawInitialGuessFromAlignment_) {
    trajectoryAlignmentHandler_->addLidarPose(lio_T_M_Lk.translation(), lidarBetweenTimeK);
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
          "Lidar_unary_6D", int(lioOdometryRate_), lioOdomFrameName, lioOdomFrameName + sensorFrameCorrectedNameId_,
          graph_msf::RobustNormEnum::None, 1.0, lidarBetweenTimeK, odomLidarPtr->header.frame_id, 1.0, lio_T_M_Lk, lioPoseUnaryNoise_);
      // Add to graph
      REGULAR_COUT << GREEN_START << " LiDAR odometry callback is setting global yaw, as it was not set so far." << COLOR_END << std::endl;
      this->initYawAndPosition(unary6DMeasurement);
    }
  } else {  // Already initialized --> Between factor
    // Compute Delta
    const Eigen::Isometry3d T_Lkm1_Lk = lio_T_M_Lkm1_.inverse() * lio_T_M_Lk;
    // Create measurement
    graph_msf::BinaryMeasurementXD<Eigen::Isometry3d, 6> delta6DMeasurement(
        "Lidar_between_6D", int(lioOdometryRate_), lioOdomFrameName, lioOdomFrameName + sensorFrameCorrectedNameId_,
        graph_msf::RobustNormEnum::None, 1.0, lidarBetweenTimeKm1_, lidarBetweenTimeK, T_Lkm1_Lk, lioPoseUnaryNoise_);
    // Add to graph
    this->addBinaryPoseMeasurement(delta6DMeasurement);
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
  ++leggedOdometryCallbackCounter_;

  // Eigen Type
  Eigen::Isometry3d T_O_Bl_k = Eigen::Isometry3d::Identity();
  graph_msf::geometryPoseToEigen(*leggedOdometryPoseKPtr, T_O_Bl_k.matrix());
  double legOdometryTimeK = leggedOdometryPoseKPtr->header.stamp.toSec();

  // At start
  if (leggedOdometryCallbackCounter_ == 0) {
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
          "Leg_odometry_6D", int(leggedOdometryRate_), leggedOdometryFrameName, leggedOdometryFrameName + sensorFrameCorrectedNameId_,
          graph_msf::RobustNormEnum::None, 1.0, legOdometryTimeK, leggedOdometryPoseKPtr->header.frame_id, 1.0, T_O_Bl_k,
          legPoseBetweenNoise_);
      // Add to graph
      REGULAR_COUT << GREEN_START << " Legged odometry callback is setting global yaw, as it was not set so far." << COLOR_END << std::endl;
      this->initYawAndPosition(unary6DMeasurement);
    }
  } else {
    // Only add every 40th measurement
    int sampleRate = static_cast<int>(leggedOdometryRate_) / 40;
    if ((leggedOdometryCallbackCounter_ % 40) == 0) {
      // Compute Delta
      const Eigen::Isometry3d T_Bkm1_Bk = T_O_Bl_km1_.inverse() * T_O_Bl_k;
      // Create measurement
      graph_msf::BinaryMeasurementXD<Eigen::Isometry3d, 6> delta6DMeasurement(
          "Leg_odometry_6D", int(sampleRate), leggedOdometryFrameName, leggedOdometryFrameName + sensorFrameCorrectedNameId_,
          graph_msf::RobustNormEnum::None, 1.0, legOdometryTimeKm1_, legOdometryTimeK, T_Bkm1_Bk, legPoseBetweenNoise_);
      // Add to graph
      this->addBinaryPoseMeasurement(delta6DMeasurement);

      // Prepare for next iteration
      T_O_Bl_km1_ = T_O_Bl_k;
      legOdometryTimeKm1_ = legOdometryTimeK;
    }
  }
}

// Priority: 5
void AnymalEstimator::leggedVelocityUnaryCallback_(const nav_msgs::Odometry ::ConstPtr& leggedOdometryKPtr) {
  if (!areRollAndPitchInited() || !areYawAndPositionInited()) {
    return;
  }

  // Counter
  ++leggedOdometryOdomCallbackCounter_;

  // Eigen Type
  Eigen::Vector3d legVelocity = Eigen::Vector3d(leggedOdometryKPtr->twist.twist.linear.x, leggedOdometryKPtr->twist.twist.linear.y,
                                                leggedOdometryKPtr->twist.twist.linear.z);

  // Norm of the velocity
  double norm = legVelocity.norm();

  // Printout
  if (norm < 0.01 && leggedOdometryOdomCallbackCounter_ > 50) {
    std::cout << "Robot standing still." << std::endl;

    // Add zero velocity to the graph
    this->addZeroVelocityFactor(leggedOdometryKPtr->header.stamp.toSec(), legVelocityUnaryNoise_(0));
  } else {
    std::cout << "Robot walking." << std::endl;
  }
}

}  // namespace anymal_se