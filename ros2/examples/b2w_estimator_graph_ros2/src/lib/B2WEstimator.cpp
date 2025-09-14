/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "b2w_estimator_graph_ros2/B2WEstimator.h"

// Project
#include "b2w_estimator_graph_ros2/B2WStaticTransforms.h"

// Workspace

#include "graph_msf/measurements/UnaryMeasurementXD.h"
#include "graph_msf/measurements/UnaryMeasurementXDAbsolute.h"
#include "graph_msf/measurements/BinaryMeasurementXD.h"
#include "graph_msf/measurements/UnaryMeasurementXD.h"
#include "graph_msf_ros2/util/conversions.h"
#include "b2w_estimator_graph_ros2/constants.h"

namespace b2w_se {

B2WEstimator::B2WEstimator(std::shared_ptr<rclcpp::Node>& node) : graph_msf::GraphMsfRos2(node) {
  REGULAR_COUT << GREEN_START << " B2WEstimator-Constructor called." << COLOR_END << std::endl;

  // Call setup after declaring parameters
  B2WEstimator::setup();
}

void B2WEstimator::setup() {
  REGULAR_COUT << GREEN_START << " B2WEstimator-Setup called." << COLOR_END << std::endl;

  // Boolean flags
  node_->declare_parameter("sensor_params.useGnss", false);
  node_->declare_parameter("sensor_params.useLioOdometry", false);
  node_->declare_parameter("sensor_params.useLioBetweenOdometry", false);
  node_->declare_parameter("sensor_params.useWheelOdometryBetween", false);
  node_->declare_parameter("sensor_params.useWheelLinearVelocities", false);
  node_->declare_parameter("sensor_params.useVioOdometry", false);


  // GNSS parameters
  node_->declare_parameter("gnss_params.initYaw", 90.0);
  node_->declare_parameter("gnss_params.useYawInitialGuessFromFile", false);
  node_->declare_parameter("gnss_params.yawInitialGuessFromAlignment", true);
  node_->declare_parameter("gnss_params.useGnssReference", false);
  node_->declare_parameter("gnss_params.referenceLatitude", 47.4084860363);
  node_->declare_parameter("gnss_params.referenceLongitude", 8.50435818058);
  node_->declare_parameter("gnss_params.referenceAltitude", 565.0);
  node_->declare_parameter("gnss_params.referenceHeading", 0.0);


  // Trajectory Alignment parameters
  node_->declare_parameter("trajectoryAlignment.gnssRate", 10.0); // [Hz], rate of gnss measurements
  node_->declare_parameter("trajectoryAlignment.lidarRate", 10.0); // [Hz], rate of lidar odometry
  node_->declare_parameter("trajectoryAlignment.minimumDistanceHeadingInit", 3.0); // [m], minimal length of trajectory to get yaw between GNSS and Lidar trajectory
  node_->declare_parameter("trajectoryAlignment.noMovementDistance", 0.1); // [m], if measurements are below this distance and in time range, robot is considered standing
  node_->declare_parameter("trajectoryAlignment.noMovementTime", 1.0); // [s], if measurements is time range and below distance, robot is considered standing
  
  // Sensor parameters (int)
  node_->declare_parameter("sensor_params.lioOdometryRate", 0);
  node_->declare_parameter("sensor_params.lioBetweenRate", 0);
  node_->declare_parameter("sensor_params.gnssRate", 0);
  
  node_->declare_parameter("sensor_params.lioBetweenOdometryRate", 0);
  node_->declare_parameter("sensor_params.wheelOdometryBetweenRate", 0);
  node_->declare_parameter("sensor_params.wheelLinearVelocitiesRate", 0);
  node_->declare_parameter("sensor_params.vioOdometryRate", 0);

  // Alignment parameters (vector of double)
  node_->declare_parameter("alignment_params.initialSe3AlignmentNoiseDensity", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  node_->declare_parameter("alignment_params.lioSe3AlignmentRandomWalk", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});


  // // Trajectory Alignment parameters (double)
  // node_->declare_parameter("trajectoryAlignment.gnssRate", 10);
  // node_->declare_parameter("trajectoryAlignment.lidarRate", 10);
  // node_->declare_parameter("trajectoryAlignment.minimumDistanceHeadingInit", 3.0);
  // node_->declare_parameter("trajectoryAlignment.noMovementDistance", 0.1);
  // node_->declare_parameter("trajectoryAlignment.noMovementTime", 1.0);

  // Noise parameters (vectors of double)
  node_->declare_parameter("noise_params.lioPoseUnaryNoiseDensity", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  node_->declare_parameter("noise_params.lioPoseBetweenNoiseDensity", std::vector<double>{0.0, 0.0, 0.0});
  node_->declare_parameter("noise_params.wheelPoseBetweenNoiseDensity", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  node_->declare_parameter("noise_params.wheelLinearVelocitiesNoiseDensity", std::vector<double>{0.0, 0.0, 0.0});
  node_->declare_parameter("noise_params.vioPoseBetweenNoiseDensity", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  node_->declare_parameter("noise_params.gnssPositionOutlierThreshold", 1.0);

  // Extrinsic frames (string)
  node_->declare_parameter("extrinsics.lidarOdometryFrame", std::string(""));
  node_->declare_parameter("extrinsics.betweenLidarOdometryFrame", std::string(""));
  node_->declare_parameter("extrinsics.gnssFrame", std::string(""));
  node_->declare_parameter("extrinsics.lidarBetweenFrame", std::string(""));
  node_->declare_parameter("extrinsics.wheelOdometryBetweenFrame", std::string(""));
  node_->declare_parameter("extrinsics.wheelLinearVelocityLeftFrame", std::string(""));
  node_->declare_parameter("extrinsics.wheelLinearVelocityRightFrame", std::string(""));
  node_->declare_parameter("extrinsics.vioOdometryFrame", std::string(""));

  // Wheel Radius (double)
  node_->declare_parameter("sensor_params.wheelRadius", 0.0);

  // Create B2WStaticTransforms
  staticTransformsPtr_ = std::make_shared<B2WStaticTransforms>(node_);


  REGULAR_COUT << RED_START << " PARAMETER READING NOT STARTED" << COLOR_END << std::endl;
  
  B2WEstimator::readParams();

  // Initialize ROS 2 publishers and subscribers
  B2WEstimator::initializePublishers();
  B2WEstimator::initializeSubscribers();
  B2WEstimator::initializeMessages();
  B2WEstimator::initializeServices();

  GraphMsfRos2::setup(staticTransformsPtr_);

  // Transforms --> query until returns true
  bool foundTransforms = false;
  double accumulatedSleepTime = 0.0;
  while (!foundTransforms) {
    foundTransforms = staticTransformsPtr_->findTransformations();
    // Sleep for 0.1 seconds to avoid busy waiting
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    accumulatedSleepTime += 0.1;
    
    // Print message every second
    if (accumulatedSleepTime >= 1.0) {
      REGULAR_COUT << "Waiting for transforms... " << static_cast<int>(accumulatedSleepTime) << " seconds elapsed" << COLOR_END << std::endl;
      accumulatedSleepTime = 0.0; // Reset the counter
    }
  }

  REGULAR_COUT << GREEN_START << " Set up successfully." << COLOR_END << std::endl;
}

void B2WEstimator::initializePublishers() {
  pubMeasMapLioLidarPath_ = node_->create_publisher<nav_msgs::msg::Path>("/graph_msf/measLiDAR_path_map_lidar", ROS_QUEUE_SIZE);
  pubMeasMapLioPath_ = node_->create_publisher<nav_msgs::msg::Path>("/graph_msf/measLiDAR_path_map_imu", ROS_QUEUE_SIZE);
  pubMeasMapVioPath_ = node_->create_publisher<nav_msgs::msg::Path>("/graph_msf/measVIO_path_map_imu", ROS_QUEUE_SIZE);

  if (useGnssFlag_) {
    pubMeasGNSSPath_ = node_->create_publisher<nav_msgs::msg::Path>("/graph_msf/measGNSS_path", ROS_QUEUE_SIZE);
    REGULAR_COUT << COLOR_END << " Initialized GNSS Path publisher" << std::endl;
  

    // GNSS - with latching behavior (TransientLocal durability)
    auto qos_latched = rclcpp::QoS(ROS_QUEUE_SIZE).transient_local();
    
    pubReferenceNavSatFixCoordinates_ = node_->create_publisher<sensor_msgs::msg::NavSatFix>(
      "/graph_msf/reference_gnss_position", qos_latched);
    // ENU origin - with latching behavior
    pubReferenceNavSatFixCoordinatesENU_ = 
      node_->create_publisher<sensor_msgs::msg::NavSatFix>("/graph_msf/reference_gnss_position_enu", qos_latched);

  }

}

void B2WEstimator::initializeSubscribers() {

  if (useGnssFlag_) {
    subGnssNavSatFix_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gnss_topic", ROS_QUEUE_SIZE, std::bind(&B2WEstimator::gnssNavSatFixCallback_, this, std::placeholders::_1));
    REGULAR_COUT << COLOR_END << " Initialized GNSS NavSatFix subscriber with topic: /gnss_topic" << std::endl;
  }

  if (useLioBetweenOdometryFlag_) {
    subLioBetweenOdometry_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/between_lidar_odometry_topic", ROS_QUEUE_SIZE, std::bind(&B2WEstimator::lidarBetweenOdometryCallback_, this, std::placeholders::_1));
    REGULAR_COUT << COLOR_END << " Initialized LiDAR Between Odometry subscriber with topic: /between_lidar_odometry_topic" << std::endl;
  }

  if (useLioOdometryFlag_) {
    subLioOdometry_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/lidar_odometry_topic", ROS_QUEUE_SIZE, std::bind(&B2WEstimator::lidarOdometryCallback_, this, std::placeholders::_1));
    REGULAR_COUT << COLOR_END << " Initialized LiDAR Odometry subscriber with topic: /lidar_odometry_topic" << std::endl;
  }

  if (useWheelOdometryBetweenFlag_) {
    subWheelOdometryBetween_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/wheel_odometry_topic", ROS_QUEUE_SIZE, std::bind(&B2WEstimator::wheelOdometryPoseCallback_, this, std::placeholders::_1));
    REGULAR_COUT << COLOR_END << " Initialized Wheel Odometry subscriber with topic: /wheel_odometry_topic" << std::endl;
  }

  if (useWheelLinearVelocitiesFlag_) {
    subWheelLinearVelocities_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/wheel_velocities_topic", ROS_QUEUE_SIZE, std::bind(&B2WEstimator::wheelLinearVelocitiesCallback_, this, std::placeholders::_1));
    REGULAR_COUT << COLOR_END << " Initialized Wheel Linear Velocities subscriber with topic: /wheel_velocities_topic" << std::endl;
  }

  if (useVioOdometryFlag_) {
    subVioOdometry_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/vio_odometry_topic", ROS_QUEUE_SIZE, std::bind(&B2WEstimator::vioOdometryCallback_, this, std::placeholders::_1));
    REGULAR_COUT << COLOR_END << " Initialized VIO Odometry subscriber with topic: /vio_odometry_topic" << std::endl;
  }
}

void B2WEstimator::initializeMessages() {
  measLio_mapLidarPathPtr_ = std::make_shared<nav_msgs::msg::Path>();
  measLio_mapImuPathPtr_ = std::make_shared<nav_msgs::msg::Path>();
  measVio_mapImuPathPtr_ = std::make_shared<nav_msgs::msg::Path>();
  if (useGnssFlag_) {
    measGnssPathPtr_ = std::make_shared<nav_msgs::msg::Path>();
  }
}

void B2WEstimator::initializeServices() {
  // Nothing for now
}

void B2WEstimator::lidarBetweenOdometryCallback_(const nav_msgs::msg::Odometry::ConstSharedPtr& lidarBetweenOdomPtr) {
  // REGULAR_COUT << RED_START << "Warning: Received LiDAR between odometry message, but handler is not implemented." << COLOR_END << std::endl;

  if (!areRollAndPitchInited()) {
    return;
  }

  // Counter
  ++lidarBetweenCallbackCounter_;

  // Convert
  Eigen::Isometry3d lio_T_M_Lk = Eigen::Isometry3d::Identity();
  graph_msf::odomMsgToEigen(*lidarBetweenOdomPtr, lio_T_M_Lk.matrix());

  // Get the time
  rclcpp::Time stamp(lidarBetweenOdomPtr->header.stamp);
  double lidarBetweenTimeK = stamp.seconds();

  // At start
  if (lidarBetweenCallbackCounter_ == 0) {
    lio_T_M_Lkm1_ = lio_T_M_Lk;
    lidarBetweenTimeKm1_ = lidarBetweenTimeK;
  }

  // Add to trajectory aligner if needed.
  if (useGnssFlag_ && gnssHandlerPtr_->getUseYawInitialGuessFromAlignment()) {
    trajectoryAlignmentHandler_->addSe3Position(lio_T_M_Lk.translation(), lidarBetweenTimeK);
  }

  
  // Frame Name
  const std::string& lioBetweenOdomFrameName = dynamic_cast<B2WStaticTransforms*>(staticTransformsPtr_.get())->getLidarBetweenFrame();  // alias
  const std::string& lioOdomFrameName = dynamic_cast<B2WStaticTransforms*>(staticTransformsPtr_.get())->getLioOdometryFrame();  // alias


  // State Machine
  if (lidarBetweenCallbackCounter_ <= 2) {
    return;
  } else if (!areYawAndPositionInited()) {  // Initializing
    if (!useGnssFlag_ && !useLioOdometryFlag_) {
      // Measurement
      graph_msf::UnaryMeasurementXD<Eigen::Isometry3d, 6> unary6DMeasurement(
          "Lidar_unary_6D", int(lioBetweenOdometryRate_), lioBetweenOdomFrameName, lioBetweenOdomFrameName + sensorFrameCorrectedNameId,
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
        "Lidar_between_6D", int(lioBetweenOdometryRate_), lioOdomFrameName, lioOdomFrameName + sensorFrameCorrectedNameId,
        graph_msf::RobustNorm::None(), lidarBetweenTimeKm1_, lidarBetweenTimeK, T_Lkm1_Lk, lioBetweenNoise_);
    // Add to graph
    this->addBinaryPose3Measurement(delta6DMeasurement);
  }
  // Provide for next iteration
  lio_T_M_Lkm1_ = lio_T_M_Lk;
  lidarBetweenTimeKm1_ = lidarBetweenTimeK;

  // Visualization ----------------------------
  // Add to path message
  addToPathMsg(measLio_mapLidarPathPtr_, staticTransformsPtr_->getWorldFrame(), lidarBetweenOdomPtr->header.stamp, lio_T_M_Lk.translation(),
               graphConfigPtr_->imuBufferLength_ * 4);

  // Publish Path
  pubMeasMapLioPath_->publish(*measLio_mapLidarPathPtr_);

}

void B2WEstimator::gnssNavSatFixCallback_(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& navSatFixPtr) {
  // Counter
  ++gnssCallbackCounter_;

  // Convert to Eigen
  Eigen::Vector3d gnssCoord = Eigen::Vector3d(navSatFixPtr->latitude, navSatFixPtr->longitude, navSatFixPtr->altitude);
  Eigen::Vector3d estStdDevXYZ(
      sqrt(navSatFixPtr->position_covariance[0]), 
      sqrt(navSatFixPtr->position_covariance[4]),
      sqrt(navSatFixPtr->position_covariance[8]));

  // Initialize GNSS Handler
  if (gnssCallbackCounter_ < NUM_GNSS_CALLBACKS_UNTIL_START) {  // Accumulate measurements
    // Wait until measurements got accumulated
    accumulatedGnssCoordinates_ += gnssCoord;
    if ((gnssCallbackCounter_ % 10) == 0) {
      REGULAR_COUT << " NOT ENOUGH GNSS MESSAGES ARRIVED!" << std::endl;
    }
    return;
  } else if (gnssCallbackCounter_ == NUM_GNSS_CALLBACKS_UNTIL_START) {  // Initialize GNSS Handler


    Eigen::Vector3d avgGnssCoord = accumulatedGnssCoordinates_ / NUM_GNSS_CALLBACKS_UNTIL_START;

    gnssHandlerPtr_->initHandler(avgGnssCoord);


    // Create and publish the reference coordinates for downstream applications
    auto referenceGNSSmsg = std::make_shared<sensor_msgs::msg::NavSatFix>();
    referenceGNSSmsg->header = navSatFixPtr->header;
    referenceGNSSmsg->latitude = gnssHandlerPtr_->getGnssReferenceLatitude();
    referenceGNSSmsg->longitude = gnssHandlerPtr_->getGnssReferenceLongitude();
    referenceGNSSmsg->altitude = gnssHandlerPtr_->getGnssReferenceAltitude();

    pubReferenceNavSatFixCoordinates_->publish(*referenceGNSSmsg);

    REGULAR_COUT << "\033[1;36m"
                 << "==================== GNSS REFERENCE ====================\n"
                 << " LATITUDE : " << referenceGNSSmsg->latitude << "\n"
                 << " LONGITUDE: " << referenceGNSSmsg->longitude << "\n"
                 << " ALTITUDE : " << referenceGNSSmsg->altitude << "\n"
                 << "=======================================================\n"
                 << "\033[0m";

    auto referenceGNSSmsgENU = std::make_shared<sensor_msgs::msg::NavSatFix>();
    Eigen::Vector3d originAsENU = Eigen::Vector3d::Zero();

    gnssHandlerPtr_->convertNavSatToPositionLV03(gnssCoord, originAsENU);

    referenceGNSSmsgENU->header = navSatFixPtr->header;
    referenceGNSSmsgENU->latitude = originAsENU(0);
    referenceGNSSmsgENU->longitude = originAsENU(1);
    referenceGNSSmsgENU->altitude = originAsENU(2);
    pubReferenceNavSatFixCoordinatesENU_->publish(*referenceGNSSmsgENU);

    REGULAR_COUT << "\033[1;36m"
                 << "==================== GNSS REFERENCE ENU ====================\n"
                 << " X : " << referenceGNSSmsgENU->latitude << "\n"
                 << " Y : " << referenceGNSSmsgENU->longitude << "\n"
                 << " Z : " << referenceGNSSmsgENU->altitude << "\n"
                 << "===========================================================\n"
                 << "\033[0m";


    REGULAR_COUT << " GNSS Handler initialized." << COLOR_END << std::endl;
    // return;
  }

  // Convert to Cartesian Coordinates
  Eigen::Vector3d W_t_W_Gnss;
  gnssHandlerPtr_->convertNavSatToPositionLV03(gnssCoord, W_t_W_Gnss);
  std::string fixedFrame = staticTransformsPtr_->getWorldFrame();  // Alias

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
      if (gnssCallbackCounter_ % 20 == 0) {
        REGULAR_COUT << YELLOW_START << " Adding GNSS measurement to trajectory alignment." << std::endl;
      }
      // Adding the GNSS measurement
      trajectoryAlignmentHandler_->addR3Position(W_t_W_Gnss, navSatFixPtr->header.stamp.sec + navSatFixPtr->header.stamp.nanosec * 1e-9);
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
                                        dynamic_cast<B2WStaticTransforms*>(staticTransformsPtr_.get())->getBaseLinkFrame(),
                                        dynamic_cast<B2WStaticTransforms*>(staticTransformsPtr_.get())->getGnssFrame())) {
      REGULAR_COUT << GREEN_START << " GNSS initialization of yaw and position successful." << COLOR_END << std::endl;
    } else {
      REGULAR_COUT << RED_START << " GNSS initialization of yaw and position failed." << COLOR_END << std::endl;
    }
  } else {  // Case 2: Already initialized --> Unary factor
    const std::string& gnssFrameName = dynamic_cast<B2WStaticTransforms*>(staticTransformsPtr_.get())->getGnssFrame();  // Alias
    double timestampSec = navSatFixPtr->header.stamp.sec + navSatFixPtr->header.stamp.nanosec * 1e-9;
    // Measurement
    graph_msf::UnaryMeasurementXDAbsolute<Eigen::Vector3d, 3> meas_W_t_W_Gnss(
        "GnssPosition", int(gnssRate_), gnssFrameName, gnssFrameName + sensorFrameCorrectedNameId, graph_msf::RobustNorm::Huber(3.0),
        timestampSec, gnssPositionOutlierThreshold_, W_t_W_Gnss, estStdDevXYZ, fixedFrame,
        staticTransformsPtr_->getWorldFrame());
    this->addUnaryPosition3AbsoluteMeasurement(meas_W_t_W_Gnss);
  }

  // Add _gmsf to the frame
  if (fixedFrame != staticTransformsPtr_->getWorldFrame()) {
    fixedFrame += referenceFrameAlignedNameId;
  }

  // Add GNSS to Path
  addToPathMsg(measGnssPathPtr_, fixedFrame, navSatFixPtr->header.stamp, W_t_W_Gnss, graphConfigPtr_->imuBufferLength_ * 4);
  // Publish path
  pubMeasGNSSPath_->publish(*measGnssPathPtr_);
}

void B2WEstimator::imuCallback(const sensor_msgs::msg::Imu::SharedPtr imuPtr) {
  const rclcpp::Time new_imu_timestamp{imuPtr->header.stamp};
//   const rclcpp::Time arrival_time = this->node_->get_clock()->now();
//   const rclcpp::Duration delay = arrival_time - new_imu_timestamp;
//   if (delay < rclcpp::Duration(0, 0)) {
//     REGULAR_COUT << RED_START << " IMU message arrived at " << std::fixed << delay.seconds() << " before the message "
//                 "timestamp. The messages should not be stamped with a time in the future." << COLOR_END << std::endl;
//   } else if (delay > rclcpp::Duration(0, 100000000)) {
//     REGULAR_COUT << RED_START << " IMU message arrival was delayed by " << std::fixed << delay.seconds() << "." << COLOR_END << std::endl;
//   } else {
//     REGULAR_COUT << " IMU message arrival was ok. Delay: " << delay.seconds() << "." << COLOR_END << std::endl;
//   }

  if (graph_msf::GraphMsf::areRollAndPitchInited() && !graph_msf::GraphMsf::areYawAndPositionInited() && !useLioOdometryFlag_ &&
      !useWheelOdometryBetweenFlag_ && !useWheelLinearVelocitiesFlag_ && !useVioOdometryFlag_) {
    REGULAR_COUT << RED_START << " IMU callback is setting global yaw and position, as no other odometry is available. Initializing..."
                 << COLOR_END << std::endl;

    graph_msf::UnaryMeasurementXD<Eigen::Isometry3d, 6> unary6DMeasurement(
        "IMU_init_6D", int(graphConfigPtr_->imuRate_), staticTransformsPtr_->getImuFrame(),
        staticTransformsPtr_->getImuFrame() + sensorFrameCorrectedNameId, graph_msf::RobustNorm::None(),
        imuPtr->header.stamp.sec + imuPtr->header.stamp.nanosec * 1e-9, 1.0, Eigen::Isometry3d::Identity(),
        Eigen::MatrixXd::Identity(6, 1));

    graph_msf::GraphMsf::initYawAndPosition(unary6DMeasurement);
    graph_msf::GraphMsf::pretendFirstMeasurementReceived();
  }
  // Remove if norm is larger than 100
  const double angular_velocity_norm = std::sqrt(imuPtr->angular_velocity.x * imuPtr->angular_velocity.x +
                imuPtr->angular_velocity.y * imuPtr->angular_velocity.y +
                imuPtr->angular_velocity.z * imuPtr->angular_velocity.z);
  const double linear_acceleration_norm = std::sqrt(imuPtr->linear_acceleration.x * imuPtr->linear_acceleration.x +
                imuPtr->linear_acceleration.y * imuPtr->linear_acceleration.y +
                imuPtr->linear_acceleration.z * imuPtr->linear_acceleration.z);
  if (angular_velocity_norm > 10) {
    ++num_imu_errors_;
    REGULAR_COUT << RED_START << " IMU angular velocity is larger than 10 rad/s, skipping this measurement. Total error count = " << num_imu_errors_ << COLOR_END << std::endl;
    return;
  } else if (linear_acceleration_norm > 100.0) {
    ++num_imu_errors_;
    REGULAR_COUT << RED_START << " IMU linear acceleration norm is larger than 100 m/s^2, skipping this measurement. Total error count = " << num_imu_errors_ << COLOR_END << std::endl;
    return;
  }
  // Check timestamps strictly increase
  if (new_imu_timestamp == last_imu_timestamp_) {
    ++num_imu_errors_;
    REGULAR_COUT << RED_START << " IMU timestamp " << new_imu_timestamp.seconds() << " was duplicated, skipping this measurement. Total error count = " << num_imu_errors_ << COLOR_END << std::endl;
    return;
  } else if (new_imu_timestamp < last_imu_timestamp_) {
    ++num_imu_errors_;
    REGULAR_COUT << RED_START << " IMU timestamp " << new_imu_timestamp.seconds() << " was before last included IMU measurement "
        " at time" << last_imu_timestamp_.seconds() << ", skipping this measurement. Total error count = " << num_imu_errors_ << COLOR_END << std::endl;
    return;
  }
  last_imu_timestamp_ = new_imu_timestamp;

  graph_msf::GraphMsfRos2::imuCallback(imuPtr);
}

void B2WEstimator::lidarOdometryCallback_(const nav_msgs::msg::Odometry::ConstSharedPtr& odomLidarPtr) {

  // Counter
  ++lidarUnaryCallbackCounter_;

  // static int lidarOdometryCallbackCounter__ = -1;
  static double lastLidarOdometryTimeK_ = 0.0;
  // static constexpr double lioOdometryRate_ = 10.0;  // Hz

  Eigen::Isometry3d lio_T_M_Lk;
  graph_msf::odomMsgToEigen(*odomLidarPtr, lio_T_M_Lk.matrix());

    // Check for NaN values in the transformation matrix
    bool hasNaN = false;
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        if (std::isnan(lio_T_M_Lk.matrix()(i, j))) {
          hasNaN = true;
        }
      }
    }

    // Print the full transformation matrix
    // REGULAR_COUT << "LiDAR odometry transformation matrix:" << std::endl;
    // REGULAR_COUT << lio_T_M_Lk.matrix() << std::endl;

    if (hasNaN) {
      // REGULAR_COUT << RED_START << " ERROR: LiDAR odometry contains NaN values! Skipping initialization." << COLOR_END << std::endl;
      return;
    }


  // Timestamp
  double lidarOdometryTimeK = odomLidarPtr->header.stamp.sec + odomLidarPtr->header.stamp.nanosec * 1e-9;

  if (useGnssFlag_ && gnssHandlerPtr_->getUseYawInitialGuessFromAlignment()) {
    trajectoryAlignmentHandler_->addSe3Position(lio_T_M_Lk.translation(), lidarOdometryTimeK);
  }

  // Check whether the callback rate is not exceeded
  if (lidarUnaryCallbackCounter_ >= 0 && lidarOdometryTimeK - lastLidarOdometryTimeK_ < (1.0 / lioOdometryRate_)) {
    return;  // Skip this callback if the rate is exceeded
  } else {
    lastLidarOdometryTimeK_ = lidarOdometryTimeK;  // Update the last timestamp
  }

  // Update the callback counter
  ++lidarUnaryCallbackCounter_;


  
  const std::string& lioOdometryFrame = dynamic_cast<B2WStaticTransforms*>(staticTransformsPtr_.get())->getLioOdometryFrame();

  // const std::string& lioOdomFrameName = dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getLioOdometryFrame();  // alias
  graph_msf::UnaryMeasurementXDAbsolute<Eigen::Isometry3d, 6> unary6DMeasurement(
      "Lidar_unary_6D", int(lioOdometryRate_), lioOdometryFrame, lioOdometryFrame + sensorFrameCorrectedNameId,
      graph_msf::RobustNorm::None(), lidarOdometryTimeK, 1.0, lio_T_M_Lk, lioPoseUnaryNoise_, odomLidarPtr->header.frame_id,
      staticTransformsPtr_->getWorldFrame(), initialSe3AlignmentNoise_, lioSe3AlignmentRandomWalk_);

  if (lidarUnaryCallbackCounter_ <= 2) {
    return;
  } else if (!areYawAndPositionInited()) {  // Initializing if no GNSS
    if (!useGnssFlag_) {
      REGULAR_COUT << GREEN_START << " LiDAR odometry callback is setting global yaw, as it was not set so far." << COLOR_END << std::endl;
      this->initYawAndPosition(unary6DMeasurement);
    }
  } else {  // Already initialized --> unary factor
    this->addUnaryPose3AbsoluteMeasurement(unary6DMeasurement);
  }

  addToPathMsg(measLio_mapImuPathPtr_, odomLidarPtr->header.frame_id  + referenceFrameAlignedNameId, odomLidarPtr->header.stamp,
               (lio_T_M_Lk * staticTransformsPtr_->rv_T_frame1_frame2(lioOdometryFrame, staticTransformsPtr_->getImuFrame()).matrix())
                   .block<3, 1>(0, 3),
               graphConfigPtr_->imuBufferLength_ * 4);

  addToPathMsg(measLio_mapLidarPathPtr_, odomLidarPtr->header.frame_id + referenceFrameAlignedNameId, odomLidarPtr->header.stamp,
               lio_T_M_Lk.translation(), graphConfigPtr_->imuBufferLength_ * 4);

  pubMeasMapLioLidarPath_->publish(*measLio_mapLidarPathPtr_);

  pubMeasMapLioPath_->publish(*measLio_mapImuPathPtr_);
}

void B2WEstimator::wheelOdometryPoseCallback_(const nav_msgs::msg::Odometry::ConstSharedPtr& wheelOdometryKPtr) {
  if (!areRollAndPitchInited()) {
    return;
  }

  ++wheelOdometryCallbackCounter_;

  Eigen::Isometry3d T_O_Bw_k;
  graph_msf::odomMsgToEigen(*wheelOdometryKPtr, T_O_Bw_k.matrix());
  double wheelOdometryTimeK = wheelOdometryKPtr->header.stamp.sec + wheelOdometryKPtr->header.stamp.nanosec * 1e-9;

  if (wheelOdometryCallbackCounter_ == 0) {
    T_O_Bw_km1_ = T_O_Bw_k;
    wheelOdometryTimeKm1_ = wheelOdometryTimeK;
    return;
  }

  const std::string& wheelOdometryFrame = dynamic_cast<B2WStaticTransforms*>(staticTransformsPtr_.get())->getWheelOdometryBetweenFrame();

  if (!areYawAndPositionInited()) {
    if (!useLioOdometryFlag_) {
      graph_msf::UnaryMeasurementXD<Eigen::Isometry3d, 6> unary6DMeasurement(
          "Lidar_unary_6D", int(wheelOdometryBetweenRate_), wheelOdometryFrame, wheelOdometryFrame + sensorFrameCorrectedNameId,
          graph_msf::RobustNorm::None(), wheelOdometryTimeK, 1.0, Eigen::Isometry3d::Identity(), Eigen::MatrixXd::Identity(6, 1));
      graph_msf::GraphMsf::initYawAndPosition(unary6DMeasurement);
    }
  } else if (wheelOdometryCallbackCounter_ % 5 == 0 && wheelOdometryCallbackCounter_ > 0) {
    Eigen::Isometry3d T_Bkm1_Bk = T_O_Bw_km1_.inverse() * T_O_Bw_k;
    graph_msf::BinaryMeasurementXD<Eigen::Isometry3d, 6> delta6DMeasurement(
        "Wheel_odometry_6D", int(wheelOdometryBetweenRate_ / 5), wheelOdometryFrame, wheelOdometryFrame + sensorFrameCorrectedNameId,
        graph_msf::RobustNorm::Tukey(1.0), wheelOdometryTimeKm1_, wheelOdometryTimeK, T_Bkm1_Bk, wheelPoseBetweenNoise_);
    this->addBinaryPose3Measurement(delta6DMeasurement);

    T_O_Bw_km1_ = T_O_Bw_k;
    wheelOdometryTimeKm1_ = wheelOdometryTimeK;
  }
}

void B2WEstimator::wheelLinearVelocitiesCallback_(const std_msgs::msg::Float64MultiArray::ConstSharedPtr& wheelsSpeedsPtr) {
  if (!areRollAndPitchInited()) {
    return;
  }

  const double timeK = wheelsSpeedsPtr->data[0];
  const double leftWheelSpeedRps = wheelsSpeedsPtr->data[1];
  const double rightWheelSpeedRps = wheelsSpeedsPtr->data[2];
  const double leftWheelSpeedMs = leftWheelSpeedRps * wheelRadiusMeter_;
  const double rightWheelSpeedMs = rightWheelSpeedRps * wheelRadiusMeter_;

  const std::string& wheelLinearVelocityLeftFrame =
      dynamic_cast<B2WStaticTransforms*>(staticTransformsPtr_.get())->getWheelLinearVelocityLeftFrame();
  const std::string& wheelLinearVelocityRightFrame =
      dynamic_cast<B2WStaticTransforms*>(staticTransformsPtr_.get())->getWheelLinearVelocityRightFrame();

  if (!areYawAndPositionInited()) {
    if (!useLioOdometryFlag_ && !useWheelOdometryBetweenFlag_) {
      graph_msf::UnaryMeasurementXD<Eigen::Isometry3d, 6> unary6DMeasurement(
          "Lidar_unary_6D", int(wheelLinearVelocitiesRate_), wheelLinearVelocityLeftFrame,
          wheelLinearVelocityLeftFrame + sensorFrameCorrectedNameId, graph_msf::RobustNorm::None(), timeK, 1.0,
          Eigen::Isometry3d::Identity(), Eigen::MatrixXd::Identity(6, 1));
      graph_msf::GraphMsf::initYawAndPosition(unary6DMeasurement);
    }
  } else {
    // graph_msf::UnaryMeasurementXD<Eigen::Vector3d, 3> leftWheelLinearVelocityMeasurement(
    //     "Wheel_linear_velocity_left", int(wheelLinearVelocitiesRate_), wheelLinearVelocityLeftFrame,
    //     wheelLinearVelocityLeftFrame + sensorFrameCorrectedNameId, graph_msf::RobustNorm::Tukey(1.0), timeK, 1.0,
    //     Eigen::Vector3d(leftWheelSpeedMs, 0.0, 0.0), wheelLinearVelocitiesNoise_);
    // this->addUnaryVelocity3LocalMeasurement(leftWheelLinearVelocityMeasurement);

    // graph_msf::UnaryMeasurementXD<Eigen::Vector3d, 3> rightWheelLinearVelocityMeasurement(
    //     "Wheel_linear_velocity_right", int(wheelLinearVelocitiesRate_), wheelLinearVelocityRightFrame,
    //     wheelLinearVelocityRightFrame + sensorFrameCorrectedNameId, graph_msf::RobustNorm::Tukey(1.0), timeK, 1.0,
    //     Eigen::Vector3d(rightWheelSpeedMs, 0.0, 0.0), wheelLinearVelocitiesNoise_);
    // this->addUnaryVelocity3LocalMeasurement(rightWheelLinearVelocityMeasurement);

    graph_msf::UnaryMeasurementXD<Eigen::Vector3d, 3> leftWheelLinearVelocityMeasurement(
        "Wheel_linear_velocity_left", int(wheelLinearVelocitiesRate_), wheelLinearVelocityLeftFrame,
        wheelLinearVelocityLeftFrame + sensorFrameCorrectedNameId, graph_msf::RobustNorm::None(), timeK, 1.0,
        Eigen::Vector3d(leftWheelSpeedMs, 0.0, 0.0), wheelLinearVelocitiesNoise_);
    this->addUnaryVelocity3LocalMeasurement(leftWheelLinearVelocityMeasurement);

    graph_msf::UnaryMeasurementXD<Eigen::Vector3d, 3> rightWheelLinearVelocityMeasurement(
        "Wheel_linear_velocity_right", int(wheelLinearVelocitiesRate_), wheelLinearVelocityRightFrame,
        wheelLinearVelocityRightFrame + sensorFrameCorrectedNameId, graph_msf::RobustNorm::None(), timeK, 1.0,
        Eigen::Vector3d(rightWheelSpeedMs, 0.0, 0.0), wheelLinearVelocitiesNoise_);
    this->addUnaryVelocity3LocalMeasurement(rightWheelLinearVelocityMeasurement);
  }
}

void B2WEstimator::vioOdometryCallback_(const nav_msgs::msg::Odometry::ConstSharedPtr& vioOdomPtr) {
  std::cout << "VIO odometry not yet stable enough for usage, disable flag." << std::endl;

  Eigen::Isometry3d vio_T_M_Ck;
  graph_msf::odomMsgToEigen(*vioOdomPtr, vio_T_M_Ck.matrix());

  addToPathMsg(measVio_mapImuPathPtr_, vioOdomPtr->header.frame_id, vioOdomPtr->header.stamp,
               (vio_T_M_Ck * staticTransformsPtr_
                                 ->rv_T_frame1_frame2(dynamic_cast<B2WStaticTransforms*>(staticTransformsPtr_.get())->getVioOdometryFrame(),
                                                      staticTransformsPtr_->getImuFrame())
                                 .matrix())
                   .block<3, 1>(0, 3),
               graphConfigPtr_->imuBufferLength_ * 4 * 10);

  pubMeasMapVioPath_->publish(*measVio_mapImuPathPtr_);
}

}  // namespace b2w_se
