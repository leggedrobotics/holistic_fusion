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
  // Nothing
  // TODO: add soft reset of the graph for on-the-go re-init.
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
    if (!(gnssCallbackCounter_ % 10)) {
      std::cout << YELLOW_START << "AnymalEstimator" << COLOR_END << " NOT ENOUGH GNSS MESSAGES ARRIVED!" << std::endl;
    }
    return;
  } else if (gnssCallbackCounter_ == NUM_GNSS_CALLBACKS_UNTIL_START) {  // Initialize GNSS Handler
    gnssHandlerPtr_->initHandler(accumulatedGnssCoordinates_ / NUM_GNSS_CALLBACKS_UNTIL_START);
    std::cout << YELLOW_START << "AnymalEstimator" << COLOR_END << " GNSS Handler initialized." << std::endl;
    return;
  }

  // Convert to Cartesian Coordinates
  Eigen::Vector3d W_t_W_Gnss;
  gnssHandlerPtr_->convertNavSatToPosition(gnssCoord, W_t_W_Gnss);
  std::string fixedFrame = staticTransformsPtr_->getWorldFrame();
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

  // Inital world yaw initialization options
  // Case 1: Initialization
  if (!areYawAndPositionInited()) {
    // a: Default
    double initYaw_W_Base{0.0};  // Default is 0 yaw
    // b: From file
    if (gnssHandlerPtr_->useYawInitialGuessFromFile_) {
      initYaw_W_Base = gnssHandlerPtr_->globalYawDegFromFile_ / 180.0 * M_PI;
    } else if (gnssHandlerPtr_->yawInitialGuessFromAlignment_) {  // c: From alignment
      // Adding the GNSS measurement
      trajectoryAlignmentHandler_->addGnssPose(W_t_W_Gnss, gnssMsgPtr->header.stamp.toSec());
      // In radians.
      if (!(trajectoryAlignmentHandler_->alignTrajectories(initYaw_W_Base))) {
        if (gnssCallbackCounter_ % 10 == 0) {
          std::cout << YELLOW_START << "Trajectory alignment not ready. Waiting for more motion." << COLOR_END << std::endl;
        }
        return;
      }
      std::cout << GREEN_START << "Trajectory Alignment Successful. Obtained Yaw Value (deg): " << COLOR_END
                << 180.0 * initYaw_W_Base / M_PI << std::endl;
    }

    // Actual Initialization
    if (not this->initYawAndPosition(initYaw_W_Base, W_t_W_Gnss, staticTransformsPtr_->getWorldFrame(),
                                     dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getBaseLinkFrame(),
                                     dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getGnssFrame())) {
      // Make clear that this was not successful
      REGULAR_COUT << RED_START << " GNSS initialization of yaw and position failed." << std::endl;
    } else {
      REGULAR_COUT << GREEN_START << " GNSS initialization of yaw and position successful." << std::endl;
    }
  } else {  // Case 2: Already initialized --> Unary factor
    const std::string& gnssFrameName = dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getGnssFrame();  // Alias
    // Measurement
    graph_msf::UnaryMeasurementXD<Eigen::Vector3d, 3> meas_W_t_W_Gnss(
        "GnssPosition", int(gnssRate_), gnssFrameName, gnssFrameName + sensorFrameCorrectedNameId_, graph_msf::RobustNormEnum::None, 1.0,
        gnssMsgPtr->header.stamp.toSec(), fixedFrame, 1.0, initialSe3AlignmentNoise_, W_t_W_Gnss, estStdDevXYZ);
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
      graph_msf::RobustNormEnum::None, 1.0, lidarUnaryTimeK, odomLidarPtr->header.frame_id, 1.0, initialSe3AlignmentNoise_, lio_T_M_Lk,
      lioPoseUnaryNoise_);

  if (lidarUnaryCallbackCounter_ <= 2) {
    return;
  } else if (!areYawAndPositionInited()) {  // Initializing if no GNSS
    if (!useGnssUnaryFlag_) {
      REGULAR_COUT << GREEN_START << " LiDAR odometry callback is setting global yaw, as it was not set so far." << COLOR_END << std::endl;
      this->initYawAndPosition(unary6DMeasurement);
    }
  } else {  // Already initialized --> unary factor
    this->addUnaryPose3Measurement(unary6DMeasurement);
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
          graph_msf::RobustNormEnum::None, 1.0, lidarBetweenTimeK, odomLidarPtr->header.frame_id, 1.0, initialSe3AlignmentNoise_,
          lio_T_M_Lk, lioPoseUnaryNoise_);
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
          graph_msf::RobustNormEnum::None, 1.0, legOdometryTimeK, leggedOdometryPoseKPtr->header.frame_id, 1.0, initialSe3AlignmentNoise_,
          T_O_Bl_k, legPoseBetweenNoise_);
      // Add to graph
      REGULAR_COUT << GREEN_START << " Legged odometry callback is setting global yaw, as it was not set so far." << COLOR_END << std::endl;
      this->initYawAndPosition(unary6DMeasurement);
    }
  } else {
    // Only add every 40th measurement
    int measurementRate = static_cast<int>(leggedOdometryRate_) / 40;
    // Check
    if ((leggedOdometryCallbackCounter_ % 40) == 0) {
      // Compute Delta
      const Eigen::Isometry3d T_Bkm1_Bk = T_O_Bl_km1_.inverse() * T_O_Bl_k;
      // Create measurement
      graph_msf::BinaryMeasurementXD<Eigen::Isometry3d, 6> delta6DMeasurement(
          "Leg_odometry_6D", measurementRate, leggedOdometryFrameName, leggedOdometryFrameName + sensorFrameCorrectedNameId_,
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

  // Alias
  const std::string& legOdometryFrame = dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getLeggedOdometryFrame();

  // Create the unary measurement
  graph_msf::UnaryMeasurementXD<Eigen::Vector3d, 3> legVelocityUnaryMeasurement(
      "Leg_velocity_unary", int(leggedOdometryRate_), legOdometryFrame, legOdometryFrame + sensorFrameCorrectedNameId_,
      graph_msf::RobustNormEnum::None, 1.0, leggedOdometryKPtr->header.stamp.toSec(), leggedOdometryKPtr->header.frame_id, 1.0,
      initialSe3AlignmentNoise_, legVelocity, legVelocityUnaryNoise_);

  // Print Summary
  //  std::cout << "Legged Odometry Velocity: " << legVelocityUnaryMeasurement << std::endl;

  // Printout
  if (norm < 0.01 && leggedOdometryOdomCallbackCounter_ > 50) {
    std::cout << "Robot standing still." << std::endl;
    // Add zero velocity to the graph
    // this->addZeroVelocityFactor(leggedOdometryKPtr->header.stamp.toSec(), legVelocityUnaryNoise_(0));
  } else {
    std::cout << "Robot walking." << std::endl;
    // Add unary velocity to the graph
  }
}

}  // namespace anymal_se