#include "fg_filtering/FactorGraphFiltering.h"

namespace compslam_se {

// Public -----------------------------------------------------------
/// Constructor -----------
FactorGraphFiltering::FactorGraphFiltering() {
  std::cout << "\033[33mFactorGraphFiltering\033[0m FactorGraphFiltering instance created." << std::endl;
}

/// Setup ------------
bool FactorGraphFiltering::setup(ros::NodeHandle& node, ros::NodeHandle& privateNode) {
  /// Initialize transform container
  staticTransformsPtr_ = new StaticTransforms(privateNode);

  // Get ROS params
  readParams_(privateNode);

  // Get transformations from URDF
  staticTransformsPtr_->findTransformations();

  // Geodetic Converter
  // geodeticConverterLeft_ = geodetic_converter::GeodeticConverter();

  // Publishers
  /// advertise odometry topic
  pubOdometryCabin_ = node.advertise<nav_msgs::Odometry>("/fg_filtering/transform_odom_cabin", ROS_QUEUE_SIZE);
  pubOdometryLidar_ = node.advertise<nav_msgs::Odometry>("/fg_filtering/transform_odom_lidar", ROS_QUEUE_SIZE);
  pubWorldLidar_ = node.advertise<nav_msgs::Odometry>("/fg_filtering/transform_world_lidar", ROS_QUEUE_SIZE);
  pubWorldImu_ = node.advertise<nav_msgs::Odometry>("/fg_filtering/transform_world_imu", ROS_QUEUE_SIZE);
  pubOdomPath_ = node.advertise<nav_msgs::Path>("/fg_filtering/odom_path", ROS_QUEUE_SIZE);
  pubOptimizationPath_ = node.advertise<nav_msgs::Path>("/fg_filtering/optimization_path", ROS_QUEUE_SIZE);
  pubCompslamPath_ = node.advertise<nav_msgs::Path>("/fg_filtering/compslam_path", ROS_QUEUE_SIZE);
  pubLaserImuBias_ = node.advertise<sensor_msgs::Imu>("/fg_filtering/imu_bias", ROS_QUEUE_SIZE);
  pubLeftGnssPath_ = node.advertise<nav_msgs::Path>("/fg_filtering/gnss_path_left", ROS_QUEUE_SIZE);
  pubRightGnssPath_ = node.advertise<nav_msgs::Path>("/fg_filtering/gnss_path_right", ROS_QUEUE_SIZE);
  excavatorStatePublisher_ = node.advertise<m545_msgs::M545State>("/m545_state", ROS_QUEUE_SIZE);
  imuMultiplotPublisher_ = node.advertise<fg_filtering_log_msgs::ImuMultiplot>("/fg_filtering/imuMultiplot", ROS_QUEUE_SIZE);
  lidarMultiplotPublisher_ = node.advertise<fg_filtering_log_msgs::LidarMultiplot>("/fg_filtering/lidarMultiplot", ROS_QUEUE_SIZE);
  /// Messages
  odomPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  optimizationPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  compslamPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  leftGnssPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  rightGnssPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);

  // Signal logger
  signalLogger_.setup(node);
  // signalLoggerGnss_.setup(node);

  // Subscribers
  /// subscribe to remapped IMU topic
  subImuCabin_ = node.subscribe<sensor_msgs::Imu>("/imu_topic_cabin", ROS_QUEUE_SIZE, &FactorGraphFiltering::imuCabinCallback_, this,
                                                  ros::TransportHints().tcpNoDelay());
  std::cout << "\033[33mFactorGraphFiltering\033[0m Initialized IMU cabin subscriber." << std::endl;
  subImuBase_ = node.subscribe<sensor_msgs::Imu>("/imu_topic_base", ROS_QUEUE_SIZE, &FactorGraphFiltering::imuBaseCallback_, this,
                                                 ros::TransportHints().tcpNoDelay());
  std::cout << "\033[33mFactorGraphFiltering\033[0m Initialized IMU base subscriber." << std::endl;
  /// subscribe to remapped LiDAR odometry topic
  if (usingCompslamFlag_) {
    subLidarOdometry_ = node.subscribe<nav_msgs::Odometry>(
        "/lidar_odometry_topic", ROS_QUEUE_SIZE, &FactorGraphFiltering::lidarOdometryCallback_, this, ros::TransportHints().tcpNoDelay());
    std::cout << "\033[33mFactorGraphFiltering\033[0m Initialized LiDAR Odometry subscriber." << std::endl;
  }
  /// subscribe to gnss topics using ROS exact sync policy in a single callback
  if (usingGnssFlag_) {
    subGnssLeft_.subscribe(node, "/gnss_topic_left", ROS_QUEUE_SIZE);
    subGnssRight_.subscribe(node, "/gnss_topic_right", ROS_QUEUE_SIZE);
    gnssExactSyncPtr_.reset(
        new message_filters::Synchronizer<_gnssExactSyncPolicy>(_gnssExactSyncPolicy(ROS_QUEUE_SIZE), subGnssLeft_, subGnssRight_));
    gnssExactSyncPtr_->registerCallback(boost::bind(&FactorGraphFiltering::gnssCallback_, this, _1, _2));
    std::cout << "\033[33mFactorGraphFiltering\033[0m Initialized GNSS subscriber (for both GNSS topics)." << std::endl;
  }
  /// Subscribe to measurements
  subMeasurements_ = node.subscribe<m545_msgs::M545Measurements>(
      "/measurement_topic", ROS_QUEUE_SIZE, &FactorGraphFiltering::measurementsCallback_, this, ros::TransportHints().tcpNoDelay());
  std::cout << "\033[33mFactorGraphFiltering\033[0m Initialized Measurements subscriber." << std::endl;

  /// Initialize helper threads
  optimizeGraphThread_ = std::thread(&FactorGraphFiltering::optimizeGraph_, this);
  std::cout << "\033[33mFactorGraphFiltering\033[0m Initialized thread for optimizing the graph in parallel." << std::endl;

  // Services
  toggleGnssUsageService_ = node.advertiseService("fg_filtering/toggle_gnss_usage", &FactorGraphFiltering::toggleGnssFlag_, this);

  return true;
}

// Private ---------------------------------------------------------------
/// Callbacks -----------------------
void FactorGraphFiltering::imuCabinCallback_(const sensor_msgs::Imu::ConstPtr& imuMsgPtr) {
  // Static Members
  static gtsam::Pose3 T_O_Ikm1__;
  static bool startTimeSetFlag__ = false;
  static std::chrono::time_point<std::chrono::high_resolution_clock> timeAtStart__;
  usingGnssFlag_ = true;

  // Initialize timing
  if (!startTimeSetFlag__) {
    timeAtStart__ = std::chrono::high_resolution_clock::now();
    startTimeSetFlag__ = true;
  }

  // Look at current time and if no message has arrived skip GNSS Initialization
  std::chrono::time_point<std::chrono::high_resolution_clock> currentTime;
  currentTime = std::chrono::high_resolution_clock::now();
  unsigned int durationSinceStart = std::chrono::duration_cast<std::chrono::seconds>(currentTime - timeAtStart__).count();
  if (!initedGnssFlag_ && (durationSinceStart > 10 || !usingGnssFlag_)) {
    std::cout << YELLOW_START << "FactorGraphFiltering" << RED_START
              << " Waited for 10 seconds, not enough GNSS messages arrived. Initialize global yaw to 0." << COLOR_END << std::endl;
    yawR_W_C0_ = gtsam::Rot3::Yaw(0);
    initedGnssFlag_ = true;
  }

  // Set IMU time
  const ros::Time imuTimeKm1 = imuTimeKm1_;
  const ros::Time imuTimeK = imuMsgPtr->header.stamp;
  imuTimeK_ = imuMsgPtr->header.stamp;
  // Define variables for timing
  std::chrono::time_point<std::chrono::high_resolution_clock> startLoopTime;
  std::chrono::time_point<std::chrono::high_resolution_clock> endLoopTime;
  // Timing
  startLoopTime = std::chrono::high_resolution_clock::now();

  // Filter out imu messages with same time stamp
  if (imuTimeK == imuTimeKm1_) {
    ROS_WARN_STREAM("Imu time " << imuTimeK << " was repeated.");
    return;
  } else {
    imuTimeKm1_ = imuTimeK;
  }

  // Write measurement to vectors
  Eigen::Vector3d linearAcc(imuMsgPtr->linear_acceleration.x, imuMsgPtr->linear_acceleration.y, imuMsgPtr->linear_acceleration.z);
  Eigen::Vector3d angularVel(imuMsgPtr->angular_velocity.x, imuMsgPtr->angular_velocity.y, imuMsgPtr->angular_velocity.z);

  // If IMU not yet aligned
  if (!initedGnssFlag_ && usingGnssFlag_) {
    std::cout << YELLOW_START << "FactorGraphFiltering" << COLOR_END << " Waiting for GNSS to provide global yaw." << std::endl;
  }  // Align IMU if yaw from GPS is here
  else if (!imuAlignedFlag_) {
    // Add to buffer
    graphMgr_.addToIMUBuffer(imuTimeK.toSec(), linearAcc, angularVel);
    alignImu_(imuTimeK);
  }  // Notification that waiting for GNSS
     // Initialize graph at next iteration step
  else if (!initedGraphFlag_ && imuAlignedFlag_) {
    std::cout << YELLOW_START << "FactorGraphFiltering" << GREEN_START << " Initializing the graph..." << COLOR_END << std::endl;
    initGraph_(imuTimeK);
    std::cout << YELLOW_START << "FactorGraphFiltering" << GREEN_START << "...graph is initialized." << COLOR_END << std::endl;
  }
  // Add measurement to graph
  else if (initedGraphFlag_) {
    // Estimate
    gtsam::NavState T_O_Ik;
    Eigen::Vector3d correctedAngularVel;

    // Add IMU factor and get propagated state
    bool relocalizationFlag = false;
    gtsam::NavState T_W_Ik = graphMgr_.addImuFactorAndGetState(imuTimeK.toSec(), linearAcc, angularVel, relocalizationFlag);
    endLoopTime = std::chrono::high_resolution_clock::now();
    imuAttitudeRoll_ = T_W_Ik.pose().rotation().roll();
    imuAttitudePitch_ = T_W_Ik.pose().rotation().pitch();

    // Write information to global variable
    tf_T_W_Ik_.setData(pose3ToTf(T_W_Ik.pose()));
    tf_T_W_Ik_.stamp_ = imuTimeK;

    // If relocalization happens --> write to map->odom
    if (relocalizationFlag) {
      // Print
      std::cout << YELLOW_START << "FactorGraphFiltering" << GREEN_START << " Relocalization is needed. Publishing to map->odom."
                << std::endl;
      // For this computation step assume T_O_Ik ~ T_O_Ikm1
      tf::Transform tf_T_W_O = pose3ToTf(T_W_Ik.pose()) * pose3ToTf(T_O_Ikm1__).inverse();
      // T_W_O_ = T_W_Ik.pose() * T_O_Ikm1__.inverse();
      T_W_O_ = tfToPose3(tf_T_W_O);
      std::cout << YELLOW_START << "FactorGraphFiltering" << GREEN_START << " T_W_O_ is now " << T_W_O_ << std::endl;
      std::cout << YELLOW_START << "FactorGraphFiltering" << GREEN_START << " T_W_Ik: " << T_W_Ik.pose() << ", T_O_Ikm1__: " << T_O_Ikm1__
                << std::endl;
    }
    // Convert global pose to local odom-robot update
    T_O_Ik = gtsam::NavState(T_W_O_.inverse() * T_W_Ik.pose(), T_W_O_.inverse().rotation() * T_W_Ik.velocity());
    // Angular Velocity
    correctedAngularVel = graphMgr_.getIMUBias().correctGyroscope(angularVel);

    // Only at very beginning --> zero velocity
    if (lidarCallbackCounter_ < NUM_LIDAR_CALLBACKS_UNTIL_START) {
      T_O_Ik = gtsam::NavState(T_W_I0_, gtsam::Velocity3(0.0, 0.0, 0.0));
      correctedAngularVel = Eigen::Vector3d(0.0, 0.0, 0.0);
      //        if (gnssCovarianceViolatedFlag_ || !usingGnssFlag_) {
      //          // Add zero motion factor in the very beginning to make biases converge
      //          graphMgr_.addZeroMotionFactor(0.02, imuTimeKm1.toSec(), imuTimeK.toSec(), gtsam::Pose3::identity());
      //          {
      //            // Mutex for optimizeGraph Flag
      //            const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
      //            optimizeGraphFlag_ = true;
      //          }
      //        }
    }  // end lidar callback counter

    // Publish
    publishState_(imuTimeK, T_W_O_, T_O_Ik, correctedAngularVel);
    if (verboseLevel_ > 1) {
      std::cout << YELLOW_START << "FactorGraphFiltering" << COLOR_END << " Start of IMU callback until publishing took "
                << std::chrono::duration_cast<std::chrono::milliseconds>(endLoopTime - startLoopTime).count() << " ms." << std::endl;
    }
    T_O_Ikm1__ = T_O_Ik.pose();
    // Log
    signalLogger_.publishLogger(imuTimeK.sec, imuTimeK.nsec, T_W_O_ * T_O_Ik.pose(), T_O_Ik.pose(), T_O_Ik.velocity(),
                                std::chrono::duration_cast<std::chrono::microseconds>(endLoopTime - startLoopTime).count(),
                                graphMgr_.getIMUBias());

    // Plotting in rqt_multiplot
    fg_filtering_log_msgs::ImuMultiplot imuMultiplot;
    imuMultiplot.time_stamp = imuTimeK.toSec();
    imuMultiplot.roll_velocity = angularVel(0);
    imuMultiplot.pitch_velocity = angularVel(1);
    imuMultiplot.yaw_velocity = angularVel(2);
    imuMultiplotPublisher_.publish(imuMultiplot);

  }  // end inited graph flag condition
}

void FactorGraphFiltering::imuBaseCallback_(const sensor_msgs::Imu::ConstPtr& imuMsgPtr) {
  // Timing
  const ros::Time imuTimeK = imuMsgPtr->header.stamp;

  // Write measurement to vectors
  Eigen::Vector3d linearAcc(imuMsgPtr->linear_acceleration.x, imuMsgPtr->linear_acceleration.y, imuMsgPtr->linear_acceleration.z);
  Eigen::Vector3d angularVel(imuMsgPtr->angular_velocity.x, imuMsgPtr->angular_velocity.y, imuMsgPtr->angular_velocity.z);

  // Write to latest measurement variable
  {
    // Mutex for Base IMU measurements
    const std::lock_guard<std::mutex> optimizeGraphLock(accessImuBaseMutex_);
    latestImuBaseLinearAcc_ = linearAcc;
    latestImuBaseAngularVel_ = angularVel;
  }
}

void FactorGraphFiltering::lidarOdometryCallback_(const nav_msgs::Odometry::ConstPtr& odomLidarPtr) {
  // Static variables
  static bool lidarUnaryFactorInitialized__ = false;
  static tf::Transform tf_compslam_T_O_Ikm1__;
  static gtsam::Key lastDeltaMeasurementKey__;
  static tf::Transform tf_compslam_T_O_ILatestDelta__;
  static gtsam::Pose3 T_O_ILatestDelta_Graph__;

  // Output of compslam --> predicts absolute motion in lidar frame
  tf::Transform tf_compslam_T_O_Lk, tf_compslam_T_O_Ck;
  tf::Transform tf_compslam_T_O_Ik;
  odomMsgToTf(*odomLidarPtr, tf_compslam_T_O_Lk);
  ros::Time compslamTimeKm1;

  // Transform message to Imu frame
  tf_compslam_T_O_Ik = tf_compslam_T_O_Lk * staticTransformsPtr_->T_L_Ic();

  // Set initial compslam pose after third callback (because first compslam pose is wrong)
  ++lidarCallbackCounter_;
  if (lidarCallbackCounter_ < NUM_LIDAR_CALLBACKS_UNTIL_START) {
    compslamTimeK_ = odomLidarPtr->header.stamp + ros::Duration(imuTimeOffset_);
    tf_compslam_T_I0_O_ = tf_compslam_T_O_Ik.inverse();
    tf_compslam_T_O_ILatestDelta__ = tf_compslam_T_O_Ik;
    tf_compslam_T_O_Ikm1__ = tf_compslam_T_O_Ik;
    return;
  }
  // Set LiDAR time
  compslamTimeKm1 = compslamTimeK_;
  compslamTimeK_ = odomLidarPtr->header.stamp + ros::Duration(imuTimeOffset_);

  /// Delta pose
  gtsam::Pose3 T_Ikm1_Ik = computeDeltaPose(tf_compslam_T_O_Ikm1__, tf_compslam_T_O_Ik);

  if (initedGraphFlag_) {
    // Add LiDAR measurement as delta factor
    if (!addLidarUnaryFlag_) {
      /// Reset LiDAR Unary factor intiialization
      lidarUnaryFactorInitialized__ = false;
      // Write current compslam pose to latest delta pose
      tf_compslam_T_O_ILatestDelta__ = tf_compslam_T_O_Ik;
    }
    // Add lidar measurement as unary factor
    else {
      if (!lidarUnaryFactorInitialized__) {
        lidarUnaryFactorInitialized__ = true;
        // Calculate state still from globalGraph
        T_O_ILatestDelta_Graph__ = graphMgr_.calculateStateAtKey(lastDeltaMeasurementKey__).pose();
        std::cout << YELLOW_START << "FactorGraphFiltering" << GREEN_START " Initialized LiDAR unary factors." << COLOR_END << std::endl;
      }
      /// Delta pose
      gtsam::Pose3 T_ILatestDelta_Ik(computeDeltaPose(tf_compslam_T_O_ILatestDelta__, tf_compslam_T_O_Ik));
      gtsam::Pose3 T_O_Ik = T_O_ILatestDelta_Graph__ * T_ILatestDelta_Ik;
      graphMgr_.addPoseUnaryFactor(compslamTimeK_.toSec(), T_O_Ik);
    }
    // In any case: write the lidar odom delta to the graph
    lastDeltaMeasurementKey__ = graphMgr_.addPoseBetweenFactor(compslamTimeKm1.toSec(), compslamTimeK_.toSec(), T_Ikm1_Ik);
    {
      // Mutex for optimizeGraph Flag
      const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
      optimizeGraphFlag_ = true;
    }

    // Direct compslam estimate for base
    tf::Transform tf_compslam_T_I0_Ik = tf_compslam_T_I0_O_ * tf_compslam_T_O_Ik;
    tf_compslam_T_O_Ck = tf_T_W_I0_ * tf_compslam_T_I0_Ik * staticTransformsPtr_->T_Ic_C();

    // Visualization of Compslam pose
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.frame_id = staticTransformsPtr_->getMapFrame();
    poseStamped.header.stamp = odomLidarPtr->header.stamp;
    tf::poseTFToMsg(tf_compslam_T_O_Ck, poseStamped.pose);
    /// Path
    compslamPathPtr_->header.frame_id = staticTransformsPtr_->getMapFrame();
    compslamPathPtr_->header.stamp = odomLidarPtr->header.stamp;
    compslamPathPtr_->poses.push_back(poseStamped);
    /// Publish
    pubCompslamPath_.publish(compslamPathPtr_);

    // Set last pose for the next iteration
    tf_compslam_T_O_Ikm1__ = tf_compslam_T_O_Ik;
  }
  // Plotting in rqt_multiplot
  fg_filtering_log_msgs::LidarMultiplot lidarMultiplot;
  lidarMultiplot.time_stamp = compslamTimeK_.toSec();
  lidarMultiplot.delta_roll = 10.0 * T_Ikm1_Ik.rotation().roll();
  lidarMultiplot.delta_pitch = 10.0 * T_Ikm1_Ik.rotation().pitch();
  lidarMultiplot.delta_yaw = 10.0 * T_Ikm1_Ik.rotation().yaw();
  lidarMultiplotPublisher_.publish(lidarMultiplot);
}

void FactorGraphFiltering::gnssCallback_(const sensor_msgs::NavSatFix::ConstPtr& leftGnssMsgPtr,
                                         const sensor_msgs::NavSatFix::ConstPtr& rightGnssMsgPtr) {
  // Static method variables
  static gtsam::Point3 accumulatedLeftCoordinates__(0.0, 0.0, 0.0);
  static gtsam::Point3 accumulatedRightCoordinates__(0.0, 0.0, 0.0);
  static int gnssNotJumpingCounter__ = REQUIRED_GNSS_NUM_NOT_JUMPED;
  static gtsam::Point3 lastLeftPosition__;

  // Increase counter
  ++gnssCallbackCounter_;

  // Wait until measurements got accumulated
  if (gnssCallbackCounter_ <= NUM_GNSS_CALLBACKS_UNTIL_YAW_INIT) {
    accumulatedLeftCoordinates__ += gtsam::Point3(leftGnssMsgPtr->latitude, leftGnssMsgPtr->longitude, leftGnssMsgPtr->altitude);
    accumulatedRightCoordinates__ += gtsam::Point3(rightGnssMsgPtr->latitude, rightGnssMsgPtr->longitude, rightGnssMsgPtr->altitude);
    std::cout << YELLOW_START << "FactorGraphFiltering" << COLOR_END << "NOT ENOUGH GNSS MESSAGES ARRIVED!" << std::endl;
    return;
  }
  // Set reference
  else if (gnssCallbackCounter_ == NUM_GNSS_CALLBACKS_UNTIL_YAW_INIT + 1) {
    if (initedGnssFlag_) {
      std::runtime_error("Now enough GNSS messages have arrived for initializing. But yaw has already been intiialized.");
    }
    initGnss_(accumulatedLeftCoordinates__ / NUM_GNSS_CALLBACKS_UNTIL_YAW_INIT,
              accumulatedRightCoordinates__ / NUM_GNSS_CALLBACKS_UNTIL_YAW_INIT);
    // Convert to cartesian coordinates
    gtsam::Point3 rightDummyPosition;
    convertNavSatToPositions(leftGnssMsgPtr, rightGnssMsgPtr, lastLeftPosition__, rightDummyPosition);
    return;
  }
  // Check whether graph was already initialized, otherwise leave callback
  else if (!initedGraphFlag_) {
    return;
  }

  // Convert to cartesian coordinates
  gtsam::Point3 leftPosition, rightPosition;
  convertNavSatToPositions(leftGnssMsgPtr, rightGnssMsgPtr, leftPosition, rightPosition);

  // Read covariance
  bool gnssCovarianceViolatedFlag = leftGnssMsgPtr->position_covariance[0] > GNSS_COVARIANCE_VIOLATION_THRESHOLD ||
                                    leftGnssMsgPtr->position_covariance[4] > GNSS_COVARIANCE_VIOLATION_THRESHOLD ||
                                    leftGnssMsgPtr->position_covariance[8] > GNSS_COVARIANCE_VIOLATION_THRESHOLD;
  if (gnssCovarianceViolatedFlag && !gnssCovarianceViolatedFlag_) {
    std::cout << YELLOW_START << "FactorGraphFiltering" << RED_START << " GNSS measurments now ABSENT due to too big covariance."
              << std::endl;
    gnssCovarianceViolatedFlag_ = gnssCovarianceViolatedFlag;
  } else if (!gnssCovarianceViolatedFlag && gnssCovarianceViolatedFlag_) {
    std::cout << YELLOW_START << "FactorGraphFiltering" << GREEN_START << " GNSS returned. Low covariance." << std::endl;
    gnssCovarianceViolatedFlag_ = gnssCovarianceViolatedFlag;
  }

  // GNSS jumping?
  if ((lastLeftPosition__ - leftPosition).norm() < gnssOutlierThreshold_) {
    ++gnssNotJumpingCounter__;
    if (gnssNotJumpingCounter__ == REQUIRED_GNSS_NUM_NOT_JUMPED) {
      std::cout << YELLOW_START << "FactorGraphFiltering" << GREEN_START << " GNSS was not jumping recently. Jumping counter valid again."
                << std::endl;
    }
  } else {
    if (gnssNotJumpingCounter__ >= REQUIRED_GNSS_NUM_NOT_JUMPED) {
      std::cout << YELLOW_START << "FactorGraphFiltering" << RED_START << " GNSS was jumping: Distance is "
                << (lastLeftPosition__ - leftPosition).norm() << "m, larger than allowed " << gnssOutlierThreshold_
                << "m.  Reset outlier counter." << std::endl;
    }
    gnssNotJumpingCounter__ = 0;
  }
  lastLeftPosition__ = leftPosition;

  // Case: GNSS is good --> Write to graph and perform logic
  if (!gnssCovarianceViolatedFlag_ && usingGnssFlag_ && (gnssNotJumpingCounter__ >= REQUIRED_GNSS_NUM_NOT_JUMPED)) {
    // Position factor --> only use left GNSS
    gtsam::Point3 W_t_W_I = transformGnssPointToImuFrame_(leftPosition, tf_T_W_Ik_.getRotation());
    if (graphMgr_.getStateKey() == 0) {
      return;
    }
    graphMgr_.addGnssPositionUnaryFactor(leftGnssMsgPtr->header.stamp.toSec(), W_t_W_I);

    // Heading factor
    /// Get heading (assuming that connection between antennas is perpendicular to heading)
    gtsam::Point3 W_t_heading = getRobotHeading_(leftPosition, rightPosition);
    double yaw_W_C = computeYawFromHeadingVector_(W_t_heading);
    gtsam::Rot3 yawR_W_C = gtsam::Rot3::Yaw(yaw_W_C);
    gtsam::Rot3 yawR_W_I = yawR_W_C * tfToPose3(staticTransformsPtr_->T_C_Ic()).rotation();
    // std::cout << YELLOW_START << "FactorGraphFiltering" << COLOR_END << " Yaw that will be added as heading measurement "
    //          << 180 / M_PI * yawR_W_I.yaw() << "(deg)." << std::endl;
    // graphMgr_.addGnssHeadingUnaryFactor(leftGnssMsgPtr->header.stamp.toSec(), W_t_heading, yawR_W_I.yaw());

    // Unary factor
    gtsam::Rot3 R_W_I_approx = gtsam::Rot3::Ypr(yawR_W_I.yaw(), imuAttitudePitch_, imuAttitudeRoll_);
    gtsam::Pose3 T_W_I_approx = gtsam::Pose3(R_W_I_approx, W_t_W_I);
    // graphMgr_.addPoseUnaryFactor(leftGnssMsgPtr->header.stamp.toSec(), T_W_I_approx);

    {
      // Mutex for optimizeGraph Flag
      const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
      optimizeGraphFlag_ = true;
    }
    {
      // Mutex for changing LiDAR constraint mode
      const std::lock_guard<std::mutex> lidaryUnaryLock(lidarUnaryMutex_);
      addLidarUnaryFlag_ = false;
      graphMgr_.activateGlobalGraph();

      // signalLoggerGnss_.publishLogger(leftGnssMsgPtr->header.stamp.sec, leftGnssMsgPtr->header.stamp.nsec, W_t_W_I);
    }
  }
  // Case: GNSS is bad --> Do not write to graph, set flags for lidar unary factor to true
  else if (usingLidarUnaryFlag_) {
    graphMgr_.activateFallbackGraph();
    // Mutex for changing LiDAR constraint mode
    const std::lock_guard<std::mutex> lidarUnaryLock(lidarUnaryMutex_);
    addLidarUnaryFlag_ = true;
  }
  if (!usingGnssFlag_) {
    ROS_WARN("Received GNSS message, but usage is set to false.");
  }

  // Publish path
  /// Left
  //// Pose
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = staticTransformsPtr_->getMapFrame();
  pose.header.stamp = leftGnssMsgPtr->header.stamp;
  pose.pose.position.x = leftPosition(0);
  pose.pose.position.y = leftPosition(1);
  pose.pose.position.z = leftPosition(2);
  //// Path
  leftGnssPathPtr_->header.frame_id = staticTransformsPtr_->getMapFrame();
  leftGnssPathPtr_->header.stamp = leftGnssMsgPtr->header.stamp;
  leftGnssPathPtr_->poses.push_back(pose);
  pubLeftGnssPath_.publish(leftGnssPathPtr_);
  /// Right
  //// Pose
  pose.header.frame_id = staticTransformsPtr_->getMapFrame();
  pose.header.stamp = rightGnssMsgPtr->header.stamp;
  pose.pose.position.x = rightPosition(0);  // + tf_T_C_GR.getOrigin().x();
  pose.pose.position.y = rightPosition(1);  // + tf_T_C_GR.getOrigin().y();
  pose.pose.position.z = rightPosition(2);  // + tf_T_C_GR.getOrigin().z();
  //// Path
  rightGnssPathPtr_->header.frame_id = staticTransformsPtr_->getMapFrame();
  rightGnssPathPtr_->header.stamp = rightGnssMsgPtr->header.stamp;
  rightGnssPathPtr_->poses.push_back(pose);
  pubRightGnssPath_.publish(rightGnssPathPtr_);
}

void FactorGraphFiltering::measurementsCallback_(const m545_msgs::M545Measurements::ConstPtr& measurementsMsgPtr) {
  // Converting measurement message to measurement
  measurements_ = measurementConverter_.convert(*measurementsMsgPtr);
}

/// Worker Functions -----------------------
void FactorGraphFiltering::alignImu_(const ros::Time& imuTimeK) {
  gtsam::Rot3 imuAttitude;
  if (graphMgr_.estimateAttitudeFromImu(imuGravityDirection_, imuAttitude, gravityConstant_, graphMgr_.getInitGyrBiasReference())) {
    gtsam::Rot3 yawR_W_I0 = yawR_W_C0_ * tfToPose3(staticTransformsPtr_->T_C_Ic()).rotation();
    R_W_I0_ = gtsam::Rot3::Ypr(yawR_W_I0.yaw(), imuAttitude.pitch(), imuAttitude.roll());
    imuAttitudeRoll_ = imuAttitude.roll();
    imuAttitudePitch_ = imuAttitude.pitch();
    std::cout << "\033[33mFactorGraphFiltering\033[0m Attitude of IMU is initialized. Determined Gravity Magnitude: " << gravityConstant_
              << std::endl;
    std::cout << YELLOW_START << "FactorGraphFiltering"
              << GREEN_START " Yaw in IMU frame that is used for initializing the graph: " << 180 / M_PI * yawR_W_I0.yaw() << "(deg)."
              << COLOR_END << std::endl;
    std::cout << YELLOW_START << "FactorGraphFiltering" << GREEN_START
              << " Total initial IMU attitude is Yaw/Pitch/Roll(deg): " << R_W_I0_.ypr().transpose() * (180.0 / M_PI) << COLOR_END
              << std::endl;
    imuAlignedFlag_ = true;
  } else {
    std::cout << YELLOW_START << "FactorGraphFiltering" << COLOR_END << " NOT ENOUGH IMU MESSAGES TO INITIALIZE POSE. WAITNG FOR MORE..."
              << std::endl;
  }
}

void FactorGraphFiltering::initGnss_(const gtsam::Point3& leftGnssCoordinates, const gtsam::Point3& rightGnssCoordinates) {
  // Initialize GNSS converter
  if (usingGnssReferenceFlag_) {
    gnssSensor_.setReference(gnssReferenceLatitude_, gnssReferenceLongitude_, gnssReferenceAltitude_, gnssReferenceHeading_);
  } else {
    gnssSensor_.setReference(leftGnssCoordinates(0), leftGnssCoordinates(1), leftGnssCoordinates(2), 0.0);
  }

  // Get Positions
  gtsam::Point3 leftPosition, rightPosition;
  convertNavSatToPositions(leftGnssCoordinates, rightGnssCoordinates, leftPosition, rightPosition);

  // Get heading (assuming that connection between antennas is perpendicular to heading)
  gtsam::Point3 W_t_heading = getRobotHeading_(leftPosition, rightPosition);
  std::cout << YELLOW_START << "FactorGraphFiltering" << GREEN_START << " Heading read from the GNSS is the following: " << W_t_heading
            << std::endl;

  // Get initial global yaw
  double yaw_W_C0 = computeYawFromHeadingVector_(W_t_heading);
  yawR_W_C0_ = gtsam::Rot3::Yaw(yaw_W_C0);
  std::cout << YELLOW_START << "FactorGraphFiltering" << GREEN_START << " Initial global yaw of cabin is: " << 180 / M_PI * yaw_W_C0
            << std::endl;

  // Get initial GNSS position
  W_t_W_GnssL0_ = leftPosition;

  // Set flag to true
  initedGnssFlag_ = true;
}

// Graph initialization for roll & pitch from starting attitude, assume zero yaw
void FactorGraphFiltering::initGraph_(const ros::Time& timeStamp_k) {
  // Gravity
  graphMgr_.initImuIntegrators(gravityConstant_, imuGravityDirection_);
  // Initialize first node
  //// Initial orientation as quaternion
  tf::Quaternion tf_q_W_I0 = pose3ToTf(gtsam::Pose3(R_W_I0_, gtsam::Point3(0.0, 0.0, 0.0))).getRotation();
  /// Add initial IMU translation based on intial orientation
  T_W_I0_ = gtsam::Pose3(R_W_I0_, transformGnssPointToImuFrame_(W_t_W_GnssL0_, tf_q_W_I0));
  /// Initialize graph node
  graphMgr_.initPoseVelocityBiasGraph(timeStamp_k.toSec(), T_W_I0_);
  // Read initial pose from graph
  T_W_I0_ = gtsam::Pose3(graphMgr_.getGraphState().navState().pose().matrix());
  ROS_WARN_STREAM("INIT t(x,y,z): " << T_W_I0_.translation().transpose()
                                    << ", RPY(deg): " << T_W_I0_.rotation().rpy().transpose() * (180.0 / M_PI) << "\n");
  ROS_WARN_STREAM("Factor graph key of very first node: " << graphMgr_.getStateKey() << std::endl);
  // Write in tf member variable
  tf_T_W_I0_ = pose3ToTf(T_W_I0_);
  // Initialize global pose
  tf_T_W_Ik_.setData(tf_T_W_I0_);
  tf_T_W_Ik_.frame_id_ = staticTransformsPtr_->getMapFrame();
  tf_T_W_Ik_.stamp_ = timeStamp_k;
  // Set flag
  initedGraphFlag_ = true;
}

void FactorGraphFiltering::optimizeGraph_() {
  // Preallocation
  int numLidarFactors = 0;
  // Pose Stamped
  geometry_msgs::PoseStamped T_W_C;
  T_W_C.header.frame_id = staticTransformsPtr_->getMapFrame();
  // Transforms
  tf::Transform tf_T_W_I, tf_T_W_C;
  // Timing
  std::chrono::time_point<std::chrono::high_resolution_clock> startLoopTime;
  std::chrono::time_point<std::chrono::high_resolution_clock> endLoopTime;

  // While loop
  ROS_INFO("Thread for updating graph is ready.");
  while (ros::ok()) {
    bool optimizeGraphFlag = false;
    // Mutex for optimizeGraph Flag
    {
      // Lock
      const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
      if (optimizeGraphFlag_) {
        optimizeGraphFlag = optimizeGraphFlag_;
        optimizeGraphFlag_ = false;
      }
    }

    if (optimizeGraphFlag) {
      // Get result
      startLoopTime = std::chrono::high_resolution_clock::now();
      double currentTime;
      gtsam::NavState optimizedNavState = graphMgr_.updateGraphAndState(currentTime);
      endLoopTime = std::chrono::high_resolution_clock::now();

      if (verboseLevel_ > 0) {
        std::cout << YELLOW_START << "FactorGraphFiltering" << GREEN_START << " Whole optimization loop took "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(endLoopTime - startLoopTime).count() << " milliseconds."
                  << COLOR_END << std::endl;
      }
      // Transform pose
      gtsam::Pose3 T_W_I = optimizedNavState.pose();
      long seconds = long(currentTime);
      long nanoseconds = long((currentTime - seconds) * 1e9);
      signalLogger_.publishOptimizedPosition(seconds, nanoseconds, T_W_I);
      tf_T_W_I = pose3ToTf(T_W_I);
      tf_T_W_C = tf_T_W_I * staticTransformsPtr_->T_Ic_C();

      // Publish path of optimized poses
      /// Pose
      T_W_C.header.stamp = compslamTimeK_;
      tf::poseTFToMsg(tf_T_W_C, T_W_C.pose);
      /// Path
      optimizationPathPtr_->header.frame_id = staticTransformsPtr_->getMapFrame();
      optimizationPathPtr_->header.stamp = compslamTimeK_;
      optimizationPathPtr_->poses.push_back(T_W_C);
      /// Publish
      pubOptimizationPath_.publish(optimizationPathPtr_);

      // Publish IMU Bias after update of graph
      sensor_msgs::Imu imuBiasMsg;
      imuBiasMsg.header.frame_id = staticTransformsPtr_->getImuCabinFrame();
      imuBiasMsg.header.stamp = compslamTimeK_;
      imuBiasMsg.linear_acceleration.x = graphMgr_.getIMUBias().accelerometer()(0);
      imuBiasMsg.linear_acceleration.y = graphMgr_.getIMUBias().accelerometer()(1);
      imuBiasMsg.linear_acceleration.z = graphMgr_.getIMUBias().accelerometer()(2);
      imuBiasMsg.angular_velocity.x = graphMgr_.getIMUBias().gyroscope()(0);
      imuBiasMsg.angular_velocity.y = graphMgr_.getIMUBias().gyroscope()(1);
      imuBiasMsg.angular_velocity.z = graphMgr_.getIMUBias().gyroscope()(2);
      pubLaserImuBias_.publish(imuBiasMsg);
    }  // else just sleep for a short amount of time before polling again
    else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

void FactorGraphFiltering::publishState_(ros::Time imuTimeK, const gtsam::Pose3& T_W_O, const gtsam::NavState& T_O_Ik,
                                         const Eigen::Vector3d& I_w_W_I) {
  // Used transforms
  tf::Transform tf_T_I_C = staticTransformsPtr_->T_Ic_C();
  tf::Transform tf_T_I_L = staticTransformsPtr_->T_Ic_C() * staticTransformsPtr_->T_C_L();

  // Lookup chassis to cabin turn
  const double turnJointPosition = measurements_.actuatorStates_[m545_description::M545Topology::ActuatorEnum::TURN].position_;
  Eigen::AngleAxisd C_C_B(-turnJointPosition, Eigen::Vector3d(0.0, 0.0, 1.0));
  tf::Transform tf_T_C_B;
  tf_T_C_B = pose3ToTf(C_C_B.toRotationMatrix());
  tf_T_C_B.setOrigin(tf::Vector3(0.0, 0.0, -staticTransformsPtr_->BC_z_offset()));

  // Lookup / compute remaining relative transformations and rotations
  tf::Transform R_B_C = tf::Transform(tf_T_C_B.getRotation()).inverse();
  tf::Transform R_C_Ic = tf::Transform(staticTransformsPtr_->T_C_Ic().getRotation());
  tf::Transform R_B_Ib = tf::Transform(staticTransformsPtr_->T_B_Ib().getRotation());
  tf::Vector3 B_t_Ic_C = R_B_C * (R_C_Ic * staticTransformsPtr_->T_Ic_C().getOrigin());
  tf::Vector3 B_t_C_B = R_B_C * tf_T_C_B.getOrigin();

  // Pose
  /// From Eigen to TF
  tf::Transform tf_T_O_I = pose3ToTf(T_O_Ik.pose());
  tf::Transform tf_T_W_O = pose3ToTf(T_W_O);
  /// Transform
  tf::Transform tf_T_O_C = tf_T_O_I * tf_T_I_C;
  tf::Transform tf_T_O_L = tf_T_O_I * tf_T_I_L;
  tf::Transform tf_T_W_L = tf_T_W_O * tf_T_O_L;
  /// Get odom-->base_link transformation from odom-->cabin
  tf::Transform tf_T_O_B = tf_T_O_C * tf_T_C_B;

  // Angular Velocity
  tf::Vector3 C_w_W_Ic = R_C_Ic * tf::Vector3(I_w_W_I(0), I_w_W_I(1), I_w_W_I(2));
  tf::Vector3 B_w_W_Ic = R_B_C * C_w_W_Ic;
  /// Only along z-axis the angular velocity can be different to rotation in cabin frame
  tf::Vector3& B_w_W_B = B_w_W_Ic;  // alias
  B_w_W_B.setZ((R_B_Ib * tf::Vector3(latestImuBaseAngularVel_(0), latestImuBaseAngularVel_(1), latestImuBaseAngularVel_(2))).z());

  // Linear Velocity
  Eigen::Vector3d Ic_v_W_Ic = T_O_Ik.bodyVelocity();
  tf::Vector3 C_v_W_Ic = R_C_Ic * tf::Vector3(Ic_v_W_Ic(0), Ic_v_W_Ic(1), Ic_v_W_Ic(2));
  tf::Vector3 B_v_W_Ic = R_B_C * C_v_W_Ic;
  /// Compute velocity of base in world --> need "velocity composition rule"
  tf::Vector3 B_t_Ic_B = B_t_Ic_C + B_t_C_B;
  tf::Vector3 B_v_W_B = B_v_W_Ic + B_w_W_Ic.cross(B_t_Ic_B);
  tf::Transform R_W_B = tf::Transform(tf_T_O_B.getRotation());
  tf::Vector3 W_v_W_B = R_W_B * B_v_W_B;

  // Set m545_state
  /// Time and status
  std::chrono::steady_clock::time_point chronoTimeK = std::chrono::steady_clock::time_point(chrono::nanoseconds(imuTimeK.toNSec()));
  estExcavatorState_.setTime(chronoTimeK);
  estExcavatorState_.setSequence(measurements_.sequence_);
  estExcavatorState_.setStatus(excavator_model::ExcavatorState::Status::STATUS_OK);
  /// Joint States
  excavator_model::ActuatorConversions::jointStateFromActuatorState(measurements_, estExcavatorState_);
  /// Map to odom
  gtsam::Point3 W_t_W_O = T_W_O.translation();
  gtsam::Quaternion q_W_O = T_W_O.rotation().toQuaternion();
  estExcavatorState_.setMapToOdomTransform(kindr::HomTransformQuatD(
      kindr::Position3D(W_t_W_O.x(), W_t_W_O.y(), W_t_W_O.z()), kindr::RotationQuaternionD(q_W_O.w(), q_W_O.x(), q_W_O.y(), q_W_O.z())));
  /// Pose odom to base
  estExcavatorState_.setPositionWorldToBaseInWorldFrame(
      kindr::Position3D(tf_T_O_B.getOrigin().getX(), tf_T_O_B.getOrigin().getY(), tf_T_O_B.getOrigin().getZ()));
  estExcavatorState_.setOrientationBaseToWorld(kindr::RotationQuaternionPD(tf_T_O_B.getRotation().w(), tf_T_O_B.getRotation().x(),
                                                                           tf_T_O_B.getRotation().y(), tf_T_O_B.getRotation().z()));
  /// Angular Velocity
  estExcavatorState_.setAngularVelocityBaseInBaseFrame(kindr::LocalAngularVelocityD(B_w_W_B.x(), B_w_W_B.y(), B_w_W_B.z()));
  /// Linear Velocity
  estExcavatorState_.setLinearVelocityBaseInWorldFrame(kindr::Velocity3D(W_v_W_B.x(), W_v_W_B.y(), W_v_W_B.z()));
  /// Transform to ROS msg
  m545_msgs::M545State excavatorStateMsg;
  excavatorStateMsg = stateConverter_.convert(estExcavatorState_);
  excavatorStatePublisher_.publish(excavatorStateMsg);

  // Publish odometry message for odom->cabin with 100 Hz
  nav_msgs::OdometryPtr odomCabinMsgPtr(new nav_msgs::Odometry);
  odomCabinMsgPtr->header.frame_id = staticTransformsPtr_->getOdomFrame();
  odomCabinMsgPtr->child_frame_id = staticTransformsPtr_->getCabinFrame();
  odomCabinMsgPtr->header.stamp = imuTimeK;
  tf::poseTFToMsg(tf_T_O_C, odomCabinMsgPtr->pose.pose);
  pubOdometryCabin_.publish(odomCabinMsgPtr);

  // Publish odometry message for odom->lidar with 100 Hz
  nav_msgs::OdometryPtr odomLidarMsgPtr(new nav_msgs::Odometry);
  odomLidarMsgPtr->header.frame_id = staticTransformsPtr_->getOdomFrame();
  odomLidarMsgPtr->child_frame_id = staticTransformsPtr_->getLidarFrame();
  odomLidarMsgPtr->header.stamp = imuTimeK;
  tf::poseTFToMsg(tf_T_O_L, odomLidarMsgPtr->pose.pose);
  pubOdometryLidar_.publish(odomLidarMsgPtr);

  // Publish odometry message for map->lidar with 100 Hz
  nav_msgs::OdometryPtr worldLidarMsgPtr(new nav_msgs::Odometry);
  worldLidarMsgPtr->header.frame_id = staticTransformsPtr_->getMapFrame();
  worldLidarMsgPtr->child_frame_id = staticTransformsPtr_->getLidarFrame();
  worldLidarMsgPtr->header.stamp = imuTimeK;
  tf::poseTFToMsg(tf_T_W_L, worldLidarMsgPtr->pose.pose);
  pubWorldLidar_.publish(worldLidarMsgPtr);

  // Publish odometry message for map->imu with 100 Hz
  nav_msgs::OdometryPtr worldImuMsgPtr(new nav_msgs::Odometry);
  worldImuMsgPtr->header.frame_id = staticTransformsPtr_->getMapFrame();
  worldImuMsgPtr->child_frame_id = staticTransformsPtr_->getImuCabinFrame();
  worldImuMsgPtr->header.stamp = imuTimeK;
  tf::poseTFToMsg(tf_T_W_Ik_, worldImuMsgPtr->pose.pose);
  pubWorldImu_.publish(worldImuMsgPtr);

  // Publish path for cabin frame
  /// Pose
  geometry_msgs::PoseStamped poseStamped;
  poseStamped.header.frame_id = staticTransformsPtr_->getOdomFrame();
  poseStamped.header.stamp = imuTimeK;
  tf::poseTFToMsg(tf_T_O_C, poseStamped.pose);
  /// Path
  odomPathPtr_->header.frame_id = staticTransformsPtr_->getOdomFrame();
  odomPathPtr_->header.stamp = imuTimeK;
  odomPathPtr_->poses.push_back(poseStamped);
  /// Publish
  pubOdomPath_.publish(odomPathPtr_);
}

/// Utility -------------------------
void FactorGraphFiltering::convertNavSatToPositions(const sensor_msgs::NavSatFix::ConstPtr& leftGnssMsgPtr,
                                                    const sensor_msgs::NavSatFix::ConstPtr& rightGnssMsgPtr, gtsam::Point3& leftPosition,
                                                    gtsam::Point3& rightPosition) {
  /// Left
  leftPosition = gnssSensor_.gpsToCartesian(leftGnssMsgPtr->latitude, leftGnssMsgPtr->longitude, leftGnssMsgPtr->altitude);

  /// Right
  rightPosition = gnssSensor_.gpsToCartesian(rightGnssMsgPtr->latitude, rightGnssMsgPtr->longitude, rightGnssMsgPtr->altitude);
}

void FactorGraphFiltering::convertNavSatToPositions(const gtsam::Point3& leftGnssCoordinate, const gtsam::Point3& rightGnssCoordinate,
                                                    gtsam::Point3& leftPosition, gtsam::Point3& rightPosition) {
  /// Left
  leftPosition = gnssSensor_.gpsToCartesian(leftGnssCoordinate(0), leftGnssCoordinate(1), leftGnssCoordinate(2));

  /// Right
  rightPosition = gnssSensor_.gpsToCartesian(rightGnssCoordinate(0), rightGnssCoordinate(1), rightGnssCoordinate(2));
}

gtsam::Vector3 FactorGraphFiltering::transformGnssPointToImuFrame_(const gtsam::Point3& gnssPosition, const tf::Quaternion& tf_q_W_I) {
  /// Translation in robot frame
  tf::Vector3 tf_W_t_W_GnssL(gnssPosition(0), gnssPosition(1), gnssPosition(2));
  tf::Transform tf_T_GnssL_I = staticTransformsPtr_->T_GnssL_C() * staticTransformsPtr_->T_C_Ic();
  tf::Vector3 tf_GnssL_t_GnssL_I = tf_T_GnssL_I.getOrigin();
  /// Global rotation
  tf::Transform tf_T_I_GnssL = staticTransformsPtr_->T_Ic_C() * staticTransformsPtr_->T_C_GnssL();
  tf::Quaternion tf_q_W_GnssL = tf_q_W_I * tf_T_I_GnssL.getRotation();
  tf::Transform tf_R_W_GnssL = tf::Transform::getIdentity();
  tf_R_W_GnssL.setRotation(tf_q_W_GnssL);
  /// Translation in global frame
  tf::Vector3 tf_W_t_GnssL_I = tf_R_W_GnssL * tf_GnssL_t_GnssL_I;

  /// Shift observed GNSS position to IMU frame (instead of GNSS antenna)
  tf::Vector3 tf_W_t_W_I = tf_W_t_W_GnssL + tf_W_t_GnssL_I;

  return gtsam::Vector3(tf_W_t_W_I.x(), tf_W_t_W_I.y(), tf_W_t_W_I.z());
}

gtsam::Point3 FactorGraphFiltering::getRobotHeading_(const Eigen::Vector3d& leftPosition, const Eigen::Vector3d& rightPosition) {
  // Compute connecting unity vector
  gtsam::Point3 W_t_GnssR_GnssL = (leftPosition - rightPosition).normalized();
  W_t_GnssR_GnssL = W_t_GnssR_GnssL;
  // Compute forward pointing vector
  gtsam::Point3 zUnityVector(0.0, 0.0, -1.0);
  gtsam::Point3 W_t_heading = zUnityVector.cross(W_t_GnssR_GnssL).normalized();

  return W_t_heading;
}

double FactorGraphFiltering::computeYawFromHeadingVector_(const gtsam::Point3& headingVector) {
  double yaw = atan2(headingVector(1), headingVector(0));
  // Compute angle
  if (yaw > M_PI) {
    return yaw - (2 * M_PI);
  } else if (yaw < -M_PI) {
    return yaw + (2 * M_PI);
  } else {
    return yaw;
  }
}

bool FactorGraphFiltering::toggleGnssFlag_(std_srvs::Empty::Request& /*request*/, std_srvs::Empty::Response& /*response*/) {
  usingGnssFlag_ = !usingGnssFlag_;
  ROS_WARN_STREAM("GNSS usage was toggled to " << usingGnssFlag_);
  return true;
}

/// Commodity -----------------------
void FactorGraphFiltering::readParams_(const ros::NodeHandle& privateNode) {
  // Variables for parameter fetching
  double dParam;
  int iParam;
  bool bParam;
  std::string sParam;

  // Set frames
  /// Map
  if (privateNode.getParam("extrinsics/mapFrame", sParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - Map frame set to: " << sParam);
    staticTransformsPtr_->setMapFrame(sParam);
  }
  /// Odom
  if (privateNode.getParam("extrinsics/odomFrame", sParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - Odom frame set to: " << sParam);
    staticTransformsPtr_->setOdomFrame(sParam);
  } else {
    ROS_WARN("FactorGraphFiltering - Odom frame not set");
  }
  /// base_link
  if (privateNode.getParam("extrinsics/baseLinkFrame", sParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - base_link frame: " << sParam);
    staticTransformsPtr_->setBaseLinkFrame(sParam);
  } else
    ROS_WARN("FactorGraphFiltering - IMU frame not set for preintegrator");
  /// IMU
  //// Cabin IMU
  if (privateNode.getParam("extrinsics/imuCabinFrame", sParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - IMU Cabin frame for preintegrator and tf: " << sParam);
    staticTransformsPtr_->setImuCabinFrame(sParam);
  } else
    ROS_WARN("FactorGraphFiltering - IMU Cabin frame not set for preintegrator");
  if (privateNode.getParam("extrinsics/imuRooftopFrame", sParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - IMU Rooftop frame for preintegrator and tf: " << sParam);
    staticTransformsPtr_->setImuRooftopFrame(sParam);
  } else
    ROS_WARN("FactorGraphFiltering - IMU Rooftop frame not set for preintegrator");
  //// Base IMU
  if (privateNode.getParam("extrinsics/imuBaseFrame", sParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - IMU Base frame for state publishing: " << sParam);
    staticTransformsPtr_->setImuBaseFrame(sParam);
  } else
    ROS_WARN("FactorGraphFiltering - IMU base frame not set for state publishing");
  /// LiDAR frame
  if (privateNode.getParam("extrinsics/lidarFrame", sParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - LiDAR frame: " << sParam);
    staticTransformsPtr_->setLidarFrame(sParam);
  } else {
    ROS_WARN("FactorGraphFiltering - LiDAR frame not set");
  }
  /// Cabin frame
  if (privateNode.getParam("extrinsics/cabinFrame", sParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - cabin frame: " << sParam);
    staticTransformsPtr_->setCabinFrame(sParam);
  } else {
    ROS_WARN("FactorGraphFiltering - cabin frame not set");
  }
  /// Left GNSS frame
  if (privateNode.getParam("extrinsics/leftGnssFrame", sParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - left GNSS frame: " << sParam);
    staticTransformsPtr_->setLeftGnssFrame(sParam);
  } else {
    ROS_WARN("FactorGraphFiltering - left GNSS frame not set");
  }
  /// Right GNSS frame
  if (privateNode.getParam("extrinsics/rightGnssFrame", sParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - right GNSS frame: " << sParam);
    staticTransformsPtr_->setRightGnssFrame(sParam);
  } else {
    ROS_WARN("FactorGraphFiltering - right GNSS frame not set");
  }

  // IMU gravity definition
  if (privateNode.getParam("launch/imu_gravity_direction", sParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - gravity direction of IMU: " << sParam);
    setImuGravityDirection(sParam);
  } else {
    ROS_ERROR("FactorGraphFiltering - gravity direction of imu not set");
    throw std::runtime_error("Rosparam 'launch/imu_gravity_direction' must be set.");
  }

  // Using GNSS
  if (privateNode.getParam("launch/using_gps", bParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - using GNSS: " << bParam);
    usingGnssFlag_ = bParam;
  } else {
    ROS_ERROR("FactorGraphFiltering - using GNSS not set.");
    throw std::runtime_error("Rosparam 'launch/using_gps' must be set.");
  }

  // Using Compslam
  if (privateNode.getParam("launch/using_compslam", bParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - using Compslam: " << bParam);
    usingCompslamFlag_ = bParam;
  } else {
    ROS_ERROR("FactorGraphFiltering - using Compslam not set.");
    throw std::runtime_error("Rosparam 'launch/using_compslam' must be set.");
  }

  // Factor Graph Parameters
  if (privateNode.getParam("sensor_params/imuRate", dParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - IMU rate for preintegrator: " << dParam);
    graphMgr_.setImuRate(dParam);
  }
  if (privateNode.getParam("sensor_params/imuBufferLength", iParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - IMU buffer length: " << iParam);
    graphMgr_.setImuBufferLength(iParam);
  }
  if (privateNode.getParam("sensor_params/lidarRate", dParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - LiDAR rate: " << dParam);
    graphMgr_.setLidarRate(dParam);
  }
  if (privateNode.getParam("sensor_params/gnssRate", dParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - GNSS rate: " << dParam);
    graphMgr_.setGnssRate(dParam);
  }
  if (privateNode.getParam("sensor_params/imuTimeOffset", dParam)) {
    imuTimeOffset_ = dParam;
  }
  if (privateNode.getParam("graph_params/smootherLag", dParam)) {
    graphMgr_.setSmootherLag(dParam);
  }
  if (privateNode.getParam("graph_params/additonalIterations", iParam)) {
    graphMgr_.setIterations(iParam);
  }
  if (privateNode.getParam("graph_params/findUnusedFactorSlots", bParam)) {
    graphMgr_.getIsamParamsReference().findUnusedFactorSlots = bParam;
  }
  if (privateNode.getParam("graph_params/enableDetailedResults", bParam)) {
    graphMgr_.getIsamParamsReference().setEnableDetailedResults(bParam);
  }
  if (privateNode.getParam("graph_params/usingLidarUnary", bParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - useUnaryLidar: " << bParam);
    usingLidarUnaryFlag_ = bParam;
  }

  // Outlier Parameters
  if (privateNode.getParam("outlier_params/gnssOutlierThreshold", dParam)) {
    gnssOutlierThreshold_ = dParam;
  }
  //  if (privateNode.getParam("outlier_params/graphUpdateThreshold", dParam)) {
  //    graphUpdateThreshold_ = dParam;
  //  }
  // Noise Parameters
  /// Accelerometer
  if (privateNode.getParam("noise_params/accNoiseDensity", dParam)) {
    graphMgr_.setAccNoiseDensity(dParam);
  }
  if (privateNode.getParam("noise_params/accBiasRandomWalk", dParam)) {
    graphMgr_.setAccBiasRandomWalk(dParam);
  }
  if (privateNode.getParam("noise_params/accBiasPrior", dParam)) {
    graphMgr_.setAccBiasPrior(dParam);
  }
  /// Gyro
  if (privateNode.getParam("noise_params/gyrNoiseDensity", dParam)) {
    graphMgr_.setGyroNoiseDensity(dParam);
  }
  if (privateNode.getParam("noise_params/gyrBiasRandomWalk", dParam)) {
    graphMgr_.setGyrBiasRandomWalk(dParam);
  }
  if (privateNode.getParam("noise_params/gyrBiasPrior", dParam)) {
    graphMgr_.setGyrBiasPrior(Eigen::Vector3d(dParam, dParam, dParam));
  }
  /// Preintegration
  if (privateNode.getParam("noise_params/integrationNoiseDensity", dParam)) {
    graphMgr_.setIntegrationNoiseDensity(dParam);
  }
  if (privateNode.getParam("noise_params/biasAccOmegaPreint", dParam)) {
    graphMgr_.setBiasAccOmegaPreint(dParam);
  }
  /// LiDAR
  std::vector<double> poseBetweenNoise;  // roll,pitch,yaw,x,y,z
  if (privateNode.getParam("noise_params/poseBetweenNoise", poseBetweenNoise)) {
    ROS_WARN_STREAM("Set pose between noise to " << poseBetweenNoise[0] << "," << poseBetweenNoise[1] << "," << poseBetweenNoise[2] << ","
                                                 << poseBetweenNoise[3] << "," << poseBetweenNoise[4] << "," << poseBetweenNoise[5]);
    graphMgr_.setPoseBetweenNoise(poseBetweenNoise);
  } else {
    std::runtime_error("poseBetweenNoise needs to be set in config file.");
  }
  std::vector<double> poseUnaryNoise;  // roll,pitch,yaw,x,y,z
  if (privateNode.getParam("noise_params/poseUnaryNoise", poseUnaryNoise)) {
    ROS_WARN_STREAM("Set pose unary noise to " << poseUnaryNoise[0] << "," << poseUnaryNoise[1] << "," << poseUnaryNoise[2] << ","
                                               << poseUnaryNoise[3] << "," << poseUnaryNoise[4] << "," << poseUnaryNoise[5]);
    graphMgr_.setPoseUnaryNoise(poseUnaryNoise);
  } else {
    std::runtime_error("poseUnaryNoise needs to be set in config file.");
  }
  /// GNSS
  if (privateNode.getParam("noise_params/gnssPositionUnaryNoise", dParam)) {
    ROS_WARN_STREAM("Set gnssPositionUnaryNoise to " << dParam);
    graphMgr_.setGnssPositionUnaryNoise(dParam);
  } else {
    std::runtime_error("gnssPositionUnaryNoise needs to be set in config file.");
  }
  if (privateNode.getParam("noise_params/gnssHeadingUnaryNoise", dParam)) {
    ROS_WARN_STREAM("Set gnssHeadingUnaryNoise to " << dParam);
    graphMgr_.setGnssHeadingUnaryNoise(dParam);
  } else {
    std::runtime_error("gnssHeadingUnaryNoise needs to be set in config file.");
  }

  // Relinearization
  if (privateNode.getParam("relinearization_params/positionReLinTh", dParam)) {
    graphMgr_.setPositionReLinTh(dParam);
  }
  if (privateNode.getParam("relinearization_params/rotationReLinTh", dParam)) {
    graphMgr_.setRotationReLinTh(dParam);
  }
  if (privateNode.getParam("relinearization_params/velocityReLinTh", dParam)) {
    graphMgr_.setVelocityReLinTh(dParam);
  }
  if (privateNode.getParam("relinearization_params/accBiasReLinTh", dParam)) {
    graphMgr_.setAccBiasReLinTh(dParam);
  }
  if (privateNode.getParam("relinearization_params/gyrBiasReLinTh", dParam)) {
    graphMgr_.setGyrBiasReLinTh(dParam);
  }
  if (privateNode.getParam("relinearization_params/relinearizeSkip", iParam)) {
    graphMgr_.getIsamParamsReference().setRelinearizeSkip(iParam);
  }
  if (privateNode.getParam("relinearization_params/enableRelinearization", bParam)) {
    graphMgr_.getIsamParamsReference().setEnableRelinearization(bParam);
  }
  if (privateNode.getParam("relinearization_params/evaluateNonlinearError", bParam)) {
    graphMgr_.getIsamParamsReference().setEvaluateNonlinearError(bParam);
  }
  if (privateNode.getParam("relinearization_params/cacheLinearizedFactors", bParam)) {
    graphMgr_.getIsamParamsReference().setCacheLinearizedFactors(bParam);
  }
  if (privateNode.getParam("relinearization_params/enablePartialRelinearizationCheck", bParam)) {
    graphMgr_.getIsamParamsReference().setEnablePartialRelinearizationCheck(bParam);
  }

  // Common Parameters
  if (privateNode.getParam("common_params/verbosity", iParam)) {
    ROS_INFO("Set fg_filtering-Verbosity: %d", iParam);
    setVerboseLevel(iParam);
    graphMgr_.setVerboseLevel(iParam);
  } else {
    setVerboseLevel(0);
    graphMgr_.setVerboseLevel(0);
  }
  // Common Parameters
  if (privateNode.getParam("common_params/logPlots", bParam)) {
    ROS_INFO("Plotting of plots at end is set to: %d", bParam);
    logPlots_ = bParam;
  } else {
    logPlots_ = false;
  }

  // GNSS parameters
  if (privateNode.getParam("gnss/use_reference", bParam)) {
    ROS_INFO("Using the GNSS reference is set to: %d", bParam);
    usingGnssReferenceFlag_ = bParam;
  } else {
    std::runtime_error("Must set gnss/use_reference.");
  }
  if (privateNode.getParam("gnss/reference_latitude", dParam)) {
    ROS_INFO_STREAM("Reference latitude of the scene is given as: " << dParam);
    gnssReferenceLatitude_ = dParam;
  } else if (usingGnssReferenceFlag_) {
    std::runtime_error("GNSS reference latitude must be provided.");
  }
  if (privateNode.getParam("gnss/reference_longitude", dParam)) {
    ROS_INFO_STREAM("Reference longitude of the scene is given as: " << dParam);
    gnssReferenceLongitude_ = dParam;
  } else if (usingGnssReferenceFlag_) {
    std::runtime_error("GNSS reference longitude must be provided.");
  }
  if (privateNode.getParam("gnss/reference_altitude", dParam)) {
    ROS_INFO_STREAM("Reference altitude of the scene is given as: " << dParam);
    gnssReferenceAltitude_ = dParam;
  } else if (usingGnssReferenceFlag_) {
    std::runtime_error("GNSS reference altitude must be provided.");
  }
  if (privateNode.getParam("gnss/reference_heading", dParam)) {
    ROS_INFO_STREAM("Reference heading of the scene is given as: " << dParam);
    gnssReferenceHeading_ = dParam;
  } else if (usingGnssReferenceFlag_) {
    std::runtime_error("GNSS reference heading must be provided.");
  }
}

}  // namespace compslam_se
