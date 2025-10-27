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

B2WEstimator::B2WEstimator(const std::string& node_name,
                           const rclcpp::NodeOptions& options)
: graph_msf::GraphMsfRos2(node_name, options)
{
  RCLCPP_INFO(this->get_logger(), "B2WEstimator-Constructor called.");
  // setup();
}

void B2WEstimator::setup(const rclcpp::Node::SharedPtr& self) {
  REGULAR_COUT << GREEN_START << " B2WEstimator-Setup called." << COLOR_END << std::endl;

  // Boolean flags
  this->declare_parameter("sensor_params.useGnss", false);
  this->declare_parameter("sensor_params.useLioOdometry", false);
  this->declare_parameter("sensor_params.useLioBetweenOdometry", false);
  this->declare_parameter("sensor_params.useWheelOdometryBetween", false);
  this->declare_parameter("sensor_params.useWheelLinearVelocities", false);
  this->declare_parameter("sensor_params.useVioOdometry", false);


  // GNSS parameters
  this->declare_parameter("gnss_params.initYaw", 90.0);
  this->declare_parameter("gnss_params.useYawInitialGuessFromFile", false);
  this->declare_parameter("gnss_params.yawInitialGuessFromAlignment", true);
  this->declare_parameter("gnss_params.useGnssReference", false);
  this->declare_parameter("gnss_params.referenceLatitude", 47.4084860363);
  this->declare_parameter("gnss_params.referenceLongitude", 8.50435818058);
  this->declare_parameter("gnss_params.referenceAltitude", 565.0);
  this->declare_parameter("gnss_params.referenceHeading", 0.0);


  // Trajectory Alignment parameters
  this->declare_parameter("trajectoryAlignment.gnssRate", 10.0); // [Hz], rate of gnss measurements
  this->declare_parameter("trajectoryAlignment.lidarRate", 10.0); // [Hz], rate of lidar odometry
  this->declare_parameter("trajectoryAlignment.minimumDistanceHeadingInit", 3.0); // [m], minimal length of trajectory to get yaw between GNSS and Lidar trajectory
  this->declare_parameter("trajectoryAlignment.noMovementDistance", 0.1); // [m], if measurements are below this distance and in time range, robot is considered standing
  this->declare_parameter("trajectoryAlignment.noMovementTime", 1.0); // [s], if measurements is time range and below distance, robot is considered standing
  
  // Sensor parameters (int)
  this->declare_parameter("sensor_params.lioOdometryRate", 0);
  this->declare_parameter("sensor_params.lioBetweenRate", 0);
  this->declare_parameter("sensor_params.gnssRate", 0);
  
  this->declare_parameter("sensor_params.lioBetweenOdometryRate", 0);
  this->declare_parameter("sensor_params.wheelOdometryBetweenRate", 0);
  this->declare_parameter("sensor_params.wheelLinearVelocitiesRate", 0);
  this->declare_parameter("sensor_params.vioOdometryRate", 0);

  // Alignment parameters (vector of double)
  this->declare_parameter("alignment_params.initialSe3AlignmentNoiseDensity", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  this->declare_parameter("alignment_params.lioSe3AlignmentRandomWalk", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

  // Noise parameters (vectors of double)
  this->declare_parameter("noise_params.lioPoseUnaryNoiseDensity", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  this->declare_parameter("noise_params.lioPoseBetweenNoiseDensity", std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter("noise_params.wheelPoseBetweenNoiseDensity", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  this->declare_parameter("noise_params.wheelLinearVelocitiesNoiseDensity", std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter("noise_params.vioPoseBetweenNoiseDensity", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  this->declare_parameter("noise_params.gnssPositionOutlierThreshold", 1.0);

  // Extrinsic frames (string)
  this->declare_parameter("extrinsics.lidarOdometryFrame", std::string(""));
  this->declare_parameter("extrinsics.betweenLidarOdometryFrame", std::string(""));
  this->declare_parameter("extrinsics.gnssFrame", std::string(""));
  this->declare_parameter("extrinsics.lidarBetweenFrame", std::string(""));
  this->declare_parameter("extrinsics.wheelOdometryBetweenFrame", std::string(""));
  this->declare_parameter("extrinsics.wheelLinearVelocityLeftFrame", std::string(""));
  this->declare_parameter("extrinsics.wheelLinearVelocityRightFrame", std::string(""));
  this->declare_parameter("extrinsics.vioOdometryFrame", std::string(""));

  // Wheel Radius (double)
  this->declare_parameter("sensor_params.wheelRadius", 0.0);

  // Create B2WStaticTransforms
  // staticTransformsPtr_ = std::make_shared<B2WStaticTransforms>(this);
  staticTransformsPtr_ = std::make_shared<B2WStaticTransforms>(this->shared_from_this());


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
  // Non-time-critical publishers with best-effort and larger queue
  auto best_effort_qos = rclcpp::QoS(100)
                             .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                             .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE)
                             .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

  pubMeasMapLioLidarPath_ = this->create_publisher<nav_msgs::msg::Path>("/graph_msf/measLiDAR_path_map_lidar", best_effort_qos);
  pubMeasMapVioPath_ = this->create_publisher<nav_msgs::msg::Path>("/graph_msf/measVIO_path_map_imu", best_effort_qos);

  if (useGnssFlag_) {
    pubGnssPoseWithCov = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/graph_msf/gnss_pose_with_covariance", best_effort_qos);
    pubMeasGNSSPath_ = this->create_publisher<nav_msgs::msg::Path>("/graph_msf/measGNSS_path", best_effort_qos);
    REGULAR_COUT << COLOR_END << " Initialized GNSS Path publisher" << std::endl;
  

    // Create GNSS publishers with reliable QoS settings
    // - Reliability: RELIABLE ensures delivery of messages
    // - Durability: TRANSIENT_LOCAL (latching) keeps messages for late subscribers
    // - History: KEEP_LAST with depth 10 stores recent messages
    auto qos_reliable_latched = rclcpp::QoS(10)
      .transient_local()  // Latching behavior for late subscribers
      .reliable()         // Guaranteed delivery
      .keep_last(1);     // Store last 1 message

    pubStatus_ = this->create_publisher<std_msgs::msg::Bool>("/graph_msf/alignment_status", qos_reliable_latched);

    pubReferenceNavSatFixCoordinates_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
      "/graph_msf/reference_gnss_position", qos_reliable_latched);
    
    // ENU origin with same reliable QoS settings
    pubReferenceNavSatFixCoordinatesENU_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
      "/graph_msf/reference_gnss_position_enu", qos_reliable_latched);

  }

}


void B2WEstimator::initializeSubscribers() {
  // Per-stream groups so different sensors run concurrently on a MultiThreadedExecutor.
  auto cb_gnss  = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto cb_lidar = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto cb_wheel = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto cb_vio   = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions so_gnss;  so_gnss.callback_group  = cb_gnss;
  rclcpp::SubscriptionOptions so_lidar; so_lidar.callback_group = cb_lidar;
  rclcpp::SubscriptionOptions so_wheel; so_wheel.callback_group = cb_wheel;
  rclcpp::SubscriptionOptions so_vio;   so_vio.callback_group   = cb_vio;

  // Low-latency QoS: drop instead of backlog.
  const auto qosReliable = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
  const auto qos1 = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  const auto qos3 = rclcpp::QoS(rclcpp::KeepLast(3)).best_effort().durability_volatile();

  if (useGnssFlag_) {
    subGnssNavSatFix_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gnss_topic", qosReliable,
        std::bind(&B2WEstimator::gnssNavSatFixCallback_, this, std::placeholders::_1),
        so_gnss);
    REGULAR_COUT << COLOR_END << " Initialized GNSS NavSatFix subscriber with topic: /gnss_topic" << std::endl;
  }

  if (useLioOdometryFlag_) {
    subLioOdometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/lidar_odometry_topic", qosReliable,
        std::bind(&B2WEstimator::lidarOdometryCallback_, this, std::placeholders::_1),
        so_lidar);
    REGULAR_COUT << COLOR_END << " Initialized LiDAR Odometry subscriber with topic: /lidar_odometry_topic" << std::endl;
  }

  if (useLioBetweenOdometryFlag_) {
    subLioBetweenOdometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/between_lidar_odometry_topic", qos1,
        std::bind(&B2WEstimator::lidarBetweenOdometryCallback_, this, std::placeholders::_1),
        so_lidar);
    REGULAR_COUT << COLOR_END << " Initialized LiDAR Between Odometry subscriber with topic: /between_lidar_odometry_topic" << std::endl;
  }

  if (useWheelOdometryBetweenFlag_) {
    subWheelOdometryBetween_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/wheel_odometry_topic", qos1,
        std::bind(&B2WEstimator::wheelOdometryPoseCallback_, this, std::placeholders::_1),
        so_wheel);
    REGULAR_COUT << COLOR_END << " Initialized Wheel Odometry subscriber with topic: /wheel_odometry_topic" << std::endl;
  }

  if (useWheelLinearVelocitiesFlag_) {
    subWheelLinearVelocities_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/wheel_velocities_topic", qos1,
        std::bind(&B2WEstimator::wheelLinearVelocitiesCallback_, this, std::placeholders::_1),
        so_wheel);
    REGULAR_COUT << COLOR_END << " Initialized Wheel Linear Velocities subscriber with topic: /wheel_velocities_topic" << std::endl;
  }

  if (useVioOdometryFlag_) {
    subVioOdometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/vio_odometry_topic", qos3,  // small cushion for VIO bursts
        std::bind(&B2WEstimator::vioOdometryCallback_, this, std::placeholders::_1),
        so_vio);
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

  if (pubMeasMapLioPath_->get_subscription_count() > 0) {
    // Publish Path
    pubMeasMapLioPath_->publish(*measLio_mapLidarPathPtr_);
  }
}

void B2WEstimator::gnssNavSatFixCallback_(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& navSatFixPtr) {

  // 0) Fast validity checks
  // if (navSatFixPtr->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
  //   RCLCPP_WARN(this->get_logger(), "GNSS message has no fix. Skipping.");
  //   return;
  // }
  if (navSatFixPtr->position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                        "GNSS message has unknown covariance type. Skipping.");
    return;
  }
  if (navSatFixPtr->position_covariance[0] < 0.0) {  // driver sets -1 on invalid
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                        "GNSS message has invalid covariance. Skipping.");
    return;
  }
  if (!std::isfinite(navSatFixPtr->latitude) || !std::isfinite(navSatFixPtr->longitude) || !std::isfinite(navSatFixPtr->altitude)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                        "GNSS message contains non-finite values. Skipping.");
    return;
  }

  // Counter
  ++gnssCallbackCounter_;

  // Convert to Eigen
  Eigen::Vector3d gnssCoord = Eigen::Vector3d(navSatFixPtr->latitude, navSatFixPtr->longitude, navSatFixPtr->altitude);
  // Eigen::Vector3d estStdDevXYZ(
  //     sqrt(navSatFixPtr->position_covariance[0]), 
  //     sqrt(navSatFixPtr->position_covariance[4]),
  //     sqrt(navSatFixPtr->position_covariance[8]));

  //   Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> P_enu_map(navSatFixPtr->position_covariance.data());
  //   Eigen::Matrix3d P_lv03 = P_enu_map;

  Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>> Penu_map(navSatFixPtr->position_covariance.data());
  Eigen::Matrix3d P_enu = 0.5 * (Penu_map + Penu_map.transpose()); // symmetrize defensively

  Eigen::Matrix3d P_lv03 = graph_msf::gnss_cov::rotateCov_ENU_to_LV03(
    *gnssHandlerPtr_, navSatFixPtr->latitude, navSatFixPtr->longitude, navSatFixPtr->altitude, P_enu);


  // RCLCPP_INFO_STREAM(this->get_logger(), "P_enu (3x3):\n" << P_enu);
  // RCLCPP_INFO_STREAM(this->get_logger(), "P_lv03 (3x3):\n" << P_lv03);

  Eigen::Vector3d estStdDevXYZ(std::sqrt(P_lv03(0,0)),
                              std::sqrt(P_lv03(1,1)),
                              std::sqrt(P_lv03(2,2)));

  // bool ok = graph_msf::gnss_cov::selfCheckA(*gnssHandlerPtr_, gnssHandlerPtr_->getGnssReferenceLatitude(),
  //                     gnssHandlerPtr_->getGnssReferenceLongitude(),
  //                     gnssHandlerPtr_->getGnssReferenceAltitude());

  // if (!ok) {
  //   RCLCPP_WARN(this->get_logger(), "GNSS covariance Jacobian self-check failed; verify LV03 converter.");
  // }



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
    pubStatus_->publish(std_msgs::msg::Bool().set__data(false));
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
      pubStatus_->publish(std_msgs::msg::Bool().set__data(true));
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
        "GnssPosition", int(gnssRate_), gnssFrameName, gnssFrameName + sensorFrameCorrectedNameId, graph_msf::RobustNorm::None(),
        timestampSec, gnssPositionOutlierThreshold_, W_t_W_Gnss, estStdDevXYZ, fixedFrame,
        staticTransformsPtr_->getWorldFrame());
    this->addUnaryPosition3AbsoluteMeasurement(meas_W_t_W_Gnss);
  }

  // Add _gmsf to the frame
  if (fixedFrame != staticTransformsPtr_->getWorldFrame()) {
    fixedFrame += referenceFrameAlignedNameId;
  }

  {
    geometry_msgs::msg::PoseWithCovarianceStamped pwc;
    pwc.header.frame_id = fixedFrame;
    pwc.header.stamp = navSatFixPtr->header.stamp;
    pwc.pose.pose.position.x = W_t_W_Gnss.x();
    pwc.pose.pose.position.y = W_t_W_Gnss.y();
    pwc.pose.pose.position.z = W_t_W_Gnss.z();
    // Unknown orientation -> identity quaternion
    pwc.pose.pose.orientation.w = 1.0;
    pwc.pose.pose.orientation.x = 0.0;
    pwc.pose.pose.orientation.y = 0.0;
    pwc.pose.pose.orientation.z = 0.0;

    // Fill 6x6 covariance (row-major). Top-left 3x3 = P_lv03. Orientation variances set large (unknown).
    for (double &v : pwc.pose.covariance) v = 0.0;
    pwc.pose.covariance[0]  = P_lv03(0,0); pwc.pose.covariance[1]  = P_lv03(0,1); pwc.pose.covariance[2]  = P_lv03(0,2);
    pwc.pose.covariance[6]  = P_lv03(1,0); pwc.pose.covariance[7]  = P_lv03(1,1); pwc.pose.covariance[8]  = P_lv03(1,2);
    pwc.pose.covariance[12] = P_lv03(2,0); pwc.pose.covariance[13] = P_lv03(2,1); pwc.pose.covariance[14] = P_lv03(2,2);

    const double big_var = 1e6; // unknown roll/pitch/yaw
    pwc.pose.covariance[21] = big_var; // roll
    pwc.pose.covariance[28] = big_var; // pitch
    pwc.pose.covariance[35] = big_var; // yaw

    if (pubGnssPoseWithCov->get_subscription_count() > 0) {
      pubGnssPoseWithCov->publish(pwc);
    }
  }

  // Add GNSS to Path
  addToPathMsg(measGnssPathPtr_, fixedFrame, navSatFixPtr->header.stamp, W_t_W_Gnss, graphConfigPtr_->imuBufferLength_);
  // Publish path if there are subscribers
  if (pubMeasGNSSPath_->get_subscription_count() > 0) {
    pubMeasGNSSPath_->publish(*measGnssPathPtr_);
  }
}

void B2WEstimator::imuCallback(const sensor_msgs::msg::Imu::SharedPtr imuPtr) {
  const rclcpp::Time new_imu_timestamp{imuPtr->header.stamp};
//   const rclcpp::Time arrival_time = this->this->get_clock()->now();
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
  // // Remove if norm is larger than 100
  // const double angular_velocity_norm = std::sqrt(imuPtr->angular_velocity.x * imuPtr->angular_velocity.x +
  //               imuPtr->angular_velocity.y * imuPtr->angular_velocity.y +
  //               imuPtr->angular_velocity.z * imuPtr->angular_velocity.z);
  // const double linear_acceleration_norm = std::sqrt(imuPtr->linear_acceleration.x * imuPtr->linear_acceleration.x +
  //               imuPtr->linear_acceleration.y * imuPtr->linear_acceleration.y +
  //               imuPtr->linear_acceleration.z * imuPtr->linear_acceleration.z);
  // if (angular_velocity_norm > 10) {
  //   ++num_imu_errors_;
  //   REGULAR_COUT << RED_START << " IMU angular velocity is larger than 10 rad/s, skipping this measurement. Total error count = " << num_imu_errors_ << COLOR_END << std::endl;
  //   return;
  // } else if (linear_acceleration_norm > 100.0) {
  //   ++num_imu_errors_;
  //   REGULAR_COUT << RED_START << " IMU linear acceleration norm is larger than 100 m/s^2, skipping this measurement. Total error count = " << num_imu_errors_ << COLOR_END << std::endl;
  //   return;
  // }
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

  // // Check for NaN values in the transformation matrix
  // bool hasNaN = false;
  // for (int i = 0; i < 4; ++i) {
  //   for (int j = 0; j < 4; ++j) {
  //     if (std::isnan(lio_T_M_Lk.matrix()(i, j))) {
  //       hasNaN = true;
  //     }
  //   }
  // }

  // Print the full transformation matrix
  // REGULAR_COUT << "LiDAR odometry transformation matrix:" << std::endl;
  // REGULAR_COUT << lio_T_M_Lk.matrix() << std::endl;

  // if (hasNaN) {
  //   // REGULAR_COUT << RED_START << " ERROR: LiDAR odometry contains NaN values! Skipping initialization." << COLOR_END << std::endl;
  //   return;
  // }


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

  graph_msf::UnaryMeasurementXDAbsolute<Eigen::Isometry3d, 6> unary6DMeasurement(
      "Lidar_unary_6D", int(lioOdometryRate_), lioOdometryFrame, lioOdometryFrame + sensorFrameCorrectedNameId,
      graph_msf::RobustNorm::DCS(1.0), lidarOdometryTimeK, 1.0, lio_T_M_Lk, lioPoseUnaryNoise_, lioOdometryFrame,
      staticTransformsPtr_->getWorldFrame(), initialSe3AlignmentNoise_, lioSe3AlignmentRandomWalk_);

  if (lidarUnaryCallbackCounter_ <= 2) { //graph_msf::RobustNorm::DCS(2.0)
    return;
  } else if (!areYawAndPositionInited()) {  // Initializing if no GNSS
    if (!useGnssFlag_) {
      REGULAR_COUT << GREEN_START << " LiDAR odometry callback is setting global yaw, as it was not set so far." << COLOR_END << std::endl;
      this->initYawAndPosition(unary6DMeasurement);
    }
  } else {  // Already initialized --> unary factor
    this->addUnaryPose3AbsoluteMeasurement(unary6DMeasurement);
  }

  addToPathMsg(measLio_mapLidarPathPtr_, lioOdometryFrame + referenceFrameAlignedNameId, odomLidarPtr->header.stamp,
               lio_T_M_Lk.translation(), graphConfigPtr_->imuBufferLength_);

  if (pubMeasMapLioLidarPath_->get_subscription_count() > 0) {
  pubMeasMapLioLidarPath_->publish(*measLio_mapLidarPathPtr_);
  }
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
