#include "fg_filtering/CompslamSe.h"

namespace compslam_se {

// Public -----------------------------------------------------------
/// Constructor -----------
CompslamSe::CompslamSe() {
  std::cout << YELLOW_START << "FactorGraphFiltering" << GREEN_START << " Instance created." << COLOR_END << std::endl;
}

/// Setup ------------
bool CompslamSe::setup(ros::NodeHandle& node, ros::NodeHandle& privateNode, StaticTransforms* staticTransformsPtr) {
  std::cout << YELLOW_START << "FactorGraphFiltering" << GREEN_START << " Setting up." << COLOR_END << std::endl;
  // Get ROS params and set extrinsics
  staticTransformsPtr_ = staticTransformsPtr;
  if (staticTransformsPtr_) {
    readParams_(privateNode);
    staticTransformsPtr_->findTransformations();
  } else {
    throw std::runtime_error("CompslamSE: staticTransformsPtr must be set correctly by the inheriting class.");
  }

  // Publishers
  /// advertise odometry topic
  pubOptimizationPath_ = node.advertise<nav_msgs::Path>("/fg_filtering/optimization_path", ROS_QUEUE_SIZE);
  pubCompslamPath_ = node.advertise<nav_msgs::Path>("/fg_filtering/compslam_path", ROS_QUEUE_SIZE);
  pubLaserImuBias_ = node.advertise<sensor_msgs::Imu>("/fg_filtering/imu_bias", ROS_QUEUE_SIZE);
  pubLeftGnssPath_ = node.advertise<nav_msgs::Path>("/fg_filtering/gnss_path_left", ROS_QUEUE_SIZE);
  pubRightGnssPath_ = node.advertise<nav_msgs::Path>("/fg_filtering/gnss_path_right", ROS_QUEUE_SIZE);
  imuMultiplotPublisher_ = node.advertise<fg_filtering_log_msgs::ImuMultiplot>("/fg_filtering/imuMultiplot", ROS_QUEUE_SIZE);
  lidarMultiplotPublisher_ = node.advertise<fg_filtering_log_msgs::LidarMultiplot>("/fg_filtering/lidarMultiplot", ROS_QUEUE_SIZE);
  /// Messages
  optimizationPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  compslamPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  leftGnssPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  rightGnssPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);

  // Signal logger
  signalLogger_.setup(node);
  // signalLoggerGnss_.setup(node);

  /// Initialize helper threads
  optimizeGraphThread_ = std::thread(&CompslamSe::optimizeGraph_, this);
  std::cout << YELLOW_START << "FactorGraphFiltering" << COLOR_END << " Initialized thread for optimizing the graph in parallel."
            << std::endl;

  // Services
  toggleGnssUsageService_ = node.advertiseService("fg_filtering/toggle_gnss_usage", &CompslamSe::toggleGnssFlag_, this);

  std::cout << YELLOW_START << "FactorGraphFiltering" << GREEN_START << " Set up successfully." << COLOR_END << std::endl;
  return true;
}

// Private ---------------------------------------------------------------
/// Callbacks -----------------------
bool CompslamSe::addImuMeasurement(const Eigen::Vector3d& linearAcc, const Eigen::Vector3d& angularVel, const ros::Time& imuTimeK,
                                   InterfacePrediction*& predictionPtr) {
  // Static Members
  /// Prediction
  static Eigen::Matrix4d T_W_O__ = Eigen::Matrix4d::Identity();
  static Eigen::Matrix4d T_O_Ik__ = Eigen::Matrix4d::Identity();
  static Eigen::Vector3d I_v_W_I__ = Eigen::Vector3d();
  static Eigen::Vector3d I_w_W_I__ = Eigen::Vector3d();
  /// Other
  static Eigen::Matrix4d T_O_Ikm1__;
  static bool alignedImuFlag__ = false;
  static int imuCabinCallbackCounter__ = -1;

  // Increase counter
  ++imuCabinCallbackCounter__;

  // Set IMU time
  const ros::Time imuTimeKm1 = imuTimeKm1_;
  imuTimeK_ = imuTimeK;
  // Define variables for timing
  std::chrono::time_point<std::chrono::high_resolution_clock> startLoopTime;
  std::chrono::time_point<std::chrono::high_resolution_clock> endLoopTime;
  // Timing
  startLoopTime = std::chrono::high_resolution_clock::now();

  // Filter out imu messages with same time stamp
  if (imuTimeK == imuTimeKm1_) {
    ROS_WARN_STREAM("Imu time " << imuTimeK << " was repeated.");
    return false;
  } else {
    imuTimeKm1_ = imuTimeK;
  }

  // Loop variables
  gtsam::NavState T_W_Ik;
  bool relocalizationFlag = false;

  // If IMU not yet aligned
  if (!alignedImuFlag__) {
    // Add measurement to buffer
    graphMgr_.addToIMUBuffer(imuTimeK.toSec(), linearAcc, angularVel);
    // Add to buffer
    if (alignImu_(imuTimeK)) {
      gtsam::Rot3 yawR_W_C0 = gtsam::Rot3::Yaw(0.0);
      gtsam::Rot3 yawR_W_I0 = yawR_W_C0 * tfToPose3(staticTransformsPtr_->T_C_Ic()).rotation();
      gtsam::Rot3 R_W_I0 = gtsam::Rot3::Ypr(yawR_W_I0.yaw(), imuAttitudePitch_, imuAttitudeRoll_);
      T_O_Ik__ = gtsam::Pose3(R_W_I0, gtsam::Point3(0.0, 0.0, 0.0)).matrix();
      I_v_W_I__ = gtsam::Velocity3(0.0, 0.0, 0.0);
      I_w_W_I__ = Eigen::Vector3d(0.0, 0.0, 0.0);
      alignedImuFlag__ = true;
    } else if (imuCabinCallbackCounter__ % imuRate_ == 0) {
      // Add measurement to buffer
      graphMgr_.addToIMUBuffer(imuTimeK.toSec(), linearAcc, angularVel);
      std::cout << YELLOW_START << "FactorGraphFiltering" << COLOR_END << " NOT ENOUGH IMU MESSAGES TO INITIALIZE POSE. WAITING FOR MORE..."
                << std::endl;
    }
    return false;
  } else if (!foundInitialYawFlag_) {
    // Add measurement to buffer
    graphMgr_.addToIMUBuffer(imuTimeK.toSec(), linearAcc, angularVel);
    if (imuCabinCallbackCounter__ % imuRate_ == 0) {
      std::cout << YELLOW_START << "FactorGraphFiltering" << COLOR_END << " IMU callback waiting for initialization of global yaw."
                << std::endl;
    }
    T_W_Ik = gtsam::NavState(gtsam::Rot3(T_O_Ik__.block<3, 3>(0, 0)), T_O_Ik__.block<3, 1>(0, 3), I_v_W_I__);
  } else if (!initedGraphFlag_) {
    // Add measurement to buffer
    graphMgr_.addToIMUBuffer(imuTimeK.toSec(), linearAcc, angularVel);
    initGraph_(imuTimeK);
    std::cout << YELLOW_START << "FactorGraphFiltering" << GREEN_START << " ...graph is initialized." << COLOR_END << std::endl;
    T_W_Ik = gtsam::NavState(gtsam::Rot3(T_O_Ik__.block<3, 3>(0, 0)), T_O_Ik__.block<3, 1>(0, 3), I_v_W_I__);
  } else {
    // Add IMU factor and get propagated state
    T_W_Ik = graphMgr_.addImuFactorAndGetState(imuTimeK.toSec(), linearAcc, angularVel, relocalizationFlag);
    imuAttitudeRoll_ = T_W_Ik.pose().rotation().roll();
    imuAttitudePitch_ = T_W_Ik.pose().rotation().pitch();

    // If relocalization happens --> write to map->odom
    if (relocalizationFlag) {
      // Print
      std::cout << YELLOW_START << "FactorGraphFiltering" << GREEN_START << " Relocalization is needed. Publishing to map->odom."
                << std::endl;
      // For this computation step assume T_O_Ik ~ T_O_Ikm1
      T_W_O__ = (T_W_Ik.pose().matrix() * T_O_Ikm1__.inverse());
      tf::Transform tf_T_W_O = matrix4ToTf(T_W_O__);
    }
    // Estimate state
    if (lidarCallbackCounter_ > NUM_LIDAR_CALLBACKS_UNTIL_START) {
      T_O_Ik__ = T_W_O__.inverse() * T_W_Ik.pose().matrix();
      I_v_W_I__ = T_W_O__.block<3, 3>(0, 0).inverse() * T_W_Ik.velocity();
      I_w_W_I__ = graphMgr_.getIMUBias().correctGyroscope(angularVel);
    }
  }

  // Timing
  endLoopTime = std::chrono::high_resolution_clock::now();

  // Write information to global variable
  tf_T_W_Ik_.setData(pose3ToTf(T_W_Ik.pose()));
  tf_T_W_Ik_.stamp_ = imuTimeK;

  // Publish
  predictionPtr = new InterfacePrediction(T_W_O__, T_O_Ik__, I_v_W_I__, I_w_W_I__);

  // Write for next iteration
  T_O_Ikm1__ = T_O_Ik__;

  //  // Log
  //  signalLogger_.publishLogger(imuTimeK.sec, imuTimeK.nsec, gtsam::Pose3(T_W_O__ * T_O_Ik__), gtsam::Pose3(T_O_Ik__), I_v_W_I__,
  //                              std::chrono::duration_cast<std::chrono::microseconds>(endLoopTime - startLoopTime).count(),
  //                              graphMgr_.getIMUBias());
  //
  //  // Plotting in rqt_multiplot
  //  fg_filtering_log_msgs::ImuMultiplot imuMultiplot;
  //  imuMultiplot.time_stamp = imuTimeK.toSec();
  //  imuMultiplot.roll_velocity = angularVel(0);
  //  imuMultiplot.pitch_velocity = angularVel(1);
  //  imuMultiplot.yaw_velocity = angularVel(2);
  //  imuMultiplotPublisher_.publish(imuMultiplot);

  return true;
}

void CompslamSe::addOdometryMeasurement(const Eigen::Matrix4d& T_O_Lk, const ros::Time& odometryTimeK) {
  // Static variables
  static bool lidarUnaryFactorInitialized__ = false;
  static tf::Transform tf_compslam_T_O_Ikm1__;
  static gtsam::Key lastDeltaMeasurementKey__;
  static tf::Transform tf_compslam_T_O_Ij__;
  static gtsam::Pose3 T_O_Ij_Graph__;

  // Check whether global yaw was already provided (e.g. by GNSS)
  {
    // Locking
    const std::lock_guard<std::mutex> initGraphLock(initGraphMutex_);
    // Check
    if (!foundInitialYawFlag_) {
      globalAttitudeYaw_W_C0_ = 0.0;
      foundInitialYawFlag_ = true;
      std::cout << YELLOW_START << "FactorGraphFiltering" << GREEN_START
                << " LiDAR odometry callback is setting global cabin yaw to 0 (as it was not set so far)." << COLOR_END << std::endl;
    }
  }

  // Transform message to Imu frame
  tf::Transform tf_compslam_T_O_Lk = matrix4ToTf(T_O_Lk);
  const tf::Transform tf_compslam_T_O_Ik = tf_compslam_T_O_Lk * staticTransformsPtr_->T_L_Ic();
  const tf::Transform tf_compslam_T_I0_Ik = tf_compslam_T_I0_O_ * tf_compslam_T_O_Ik;
  const tf::Transform tf_compslam_T_O_Ck = tf_T_W_I0_ * tf_compslam_T_I0_Ik * staticTransformsPtr_->T_Ic_C();

  // Timing
  const ros::Time compslamTimeKm1 = compslamTimeK_;
  compslamTimeK_ = odometryTimeK + ros::Duration(imuTimeOffset_);

  // Set initial compslam pose after third callback (because first compslam pose is wrong)
  ++lidarCallbackCounter_;
  if (lidarCallbackCounter_ < NUM_LIDAR_CALLBACKS_UNTIL_START) {
    std::cout << YELLOW_START << "FactorGraphFiltering" << COLOR_END << " Waiting until enough LiDAR messages have arrived..." << std::endl;
    compslamTimeK_ = odometryTimeK + ros::Duration(imuTimeOffset_);
    tf_compslam_T_I0_O_ = tf_compslam_T_O_Ik.inverse();
    tf_compslam_T_O_Ij__ = tf_compslam_T_O_Ik;
    tf_compslam_T_O_Ikm1__ = tf_compslam_T_O_Ik;
    return;
  }

  if (initedGraphFlag_) {
    if (graphMgr_.globalGraphActiveFlag()) {
      /// Reset LiDAR Unary factor intiialization
      lidarUnaryFactorInitialized__ = false;
      // Write current compslam pose to latest delta pose
      tf_compslam_T_O_Ij__ = tf_compslam_T_O_Ik;
    }  //
    else if (graphMgr_.fallbackGraphActiveFlag()) {
      if (!lidarUnaryFactorInitialized__) {
        // Calculate state still from globalGraph
        T_O_Ij_Graph__ = graphMgr_.calculateStateAtKey(lastDeltaMeasurementKey__).pose();
        std::cout << YELLOW_START << "FactorGraphFiltering" << GREEN_START " Initialized LiDAR unary factors." << COLOR_END << std::endl;
        lidarUnaryFactorInitialized__ = true;
      }
      /// Delta pose
      gtsam::Pose3 T_Ij_Ik(computeDeltaPose(tf_compslam_T_O_Ij__, tf_compslam_T_O_Ik));
      gtsam::Pose3 T_O_Ik = T_O_Ij_Graph__ * T_Ij_Ik;
      graphMgr_.addPoseUnaryFactorToFallbackGraph(compslamTimeK_.toSec(), T_O_Ik);
    }
    // In any case: write the lidar odom delta to global graph
    /// Delta pose
    gtsam::Pose3 T_Ikm1_Ik = computeDeltaPose(tf_compslam_T_O_Ikm1__, tf_compslam_T_O_Ik);
    lastDeltaMeasurementKey__ = graphMgr_.addPoseBetweenFactorToGlobalGraph(compslamTimeKm1.toSec(), compslamTimeK_.toSec(), T_Ikm1_Ik);
    {
      // Mutex for optimizeGraph Flag
      const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
      optimizeGraphFlag_ = true;
    }

    // Visualization of Compslam pose
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.frame_id = staticTransformsPtr_->getMapFrame();
    poseStamped.header.stamp = odometryTimeK;
    tf::poseTFToMsg(tf_compslam_T_O_Ck, poseStamped.pose);
    /// Path
    compslamPathPtr_->header.frame_id = staticTransformsPtr_->getMapFrame();
    compslamPathPtr_->header.stamp = odometryTimeK;
    compslamPathPtr_->poses.push_back(poseStamped);
    /// Publish
    pubCompslamPath_.publish(compslamPathPtr_);

    // Set last pose for the next iteration
    tf_compslam_T_O_Ikm1__ = tf_compslam_T_O_Ik;

    // Plotting in rqt_multiplot
    fg_filtering_log_msgs::LidarMultiplot lidarMultiplot;
    lidarMultiplot.time_stamp = compslamTimeK_.toSec();
    lidarMultiplot.delta_roll = 10.0 * T_Ikm1_Ik.rotation().roll();
    lidarMultiplot.delta_pitch = 10.0 * T_Ikm1_Ik.rotation().pitch();
    lidarMultiplot.delta_yaw = 10.0 * T_Ikm1_Ik.rotation().yaw();
    lidarMultiplotPublisher_.publish(lidarMultiplot);
  }
}

void CompslamSe::addGnssMeasurements(const Eigen::Vector3d& leftGnssCoord, const Eigen::Vector3d& rightGnssCoord,
                                     const Eigen::Vector3d& covarianceXYZ, const ros::Time& gnssTimeK) {
  // Static method variables
  static gtsam::Point3 accumulatedLeftCoordinates__(0.0, 0.0, 0.0);
  static gtsam::Point3 accumulatedRightCoordinates__(0.0, 0.0, 0.0);
  static int gnssNotJumpingCounter__ = REQUIRED_GNSS_NUM_NOT_JUMPED;
  static gtsam::Point3 lastLeftPosition__;

  // Increase counter
  ++gnssCallbackCounter_;

  if (!usingGnssFlag_) {
    ROS_WARN("Received GNSS message, but usage is set to false.");
    if (usingFallbackGraphFlag_) {
      graphMgr_.activateFallbackGraph();
    }
    return;
  }

  // Wait until measurements got accumulated
  {
    // Locking
    const std::lock_guard<std::mutex> initGraphLock(initGraphMutex_);
    if (!foundInitialYawFlag_) {
      if (gnssCallbackCounter_ <= NUM_GNSS_CALLBACKS_UNTIL_YAW_INIT) {
        accumulatedLeftCoordinates__ += leftGnssCoord;
        accumulatedRightCoordinates__ += rightGnssCoord;
        std::cout << YELLOW_START << "FactorGraphFiltering" << COLOR_END << " NOT ENOUGH GNSS MESSAGES ARRIVED!" << std::endl;
        return;
      }
      // Set reference
      else if (gnssCallbackCounter_ == NUM_GNSS_CALLBACKS_UNTIL_YAW_INIT + 1) {
        initGnss_(accumulatedLeftCoordinates__ / NUM_GNSS_CALLBACKS_UNTIL_YAW_INIT,
                  accumulatedRightCoordinates__ / NUM_GNSS_CALLBACKS_UNTIL_YAW_INIT);
        // Convert to cartesian coordinates
        gtsam::Point3 rightDummyPosition;
        convertNavSatToPositions(leftGnssCoord, rightGnssCoord, lastLeftPosition__, rightDummyPosition);
        foundInitialYawFlag_ = true;
        return;
      }
    }
  }

  // Convert to cartesian coordinates
  gtsam::Point3 leftPosition, rightPosition;
  convertNavSatToPositions(leftGnssCoord, rightGnssCoord, leftPosition, rightPosition);

  // Read covariance
  bool gnssCovarianceViolatedFlag = covarianceXYZ(0) > GNSS_COVARIANCE_VIOLATION_THRESHOLD ||
                                    covarianceXYZ(1) > GNSS_COVARIANCE_VIOLATION_THRESHOLD ||
                                    covarianceXYZ(2) > GNSS_COVARIANCE_VIOLATION_THRESHOLD;
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
  if (!gnssCovarianceViolatedFlag_ && (gnssNotJumpingCounter__ >= REQUIRED_GNSS_NUM_NOT_JUMPED)) {
    // Position factor --> only use left GNSS
    gtsam::Point3 W_t_W_I = transformGnssPointToImuFrame_(leftPosition, tf_T_W_Ik_.getRotation());
    if (graphMgr_.getStateKey() == 0) {
      return;
    }
    graphMgr_.addGnssPositionUnaryFactor(gnssTimeK.toSec(), W_t_W_I);

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
    graphMgr_.activateGlobalGraph();
  }
  // Case: GNSS is bad --> Do not write to graph, set flags for lidar unary factor to true
  else if (usingFallbackGraphFlag_) {
    graphMgr_.activateFallbackGraph();
  }

  // Publish path
  /// Left
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = staticTransformsPtr_->getMapFrame();
  pose.header.stamp = gnssTimeK;
  pose.pose.position.x = leftPosition(0);
  pose.pose.position.y = leftPosition(1);
  pose.pose.position.z = leftPosition(2);
  leftGnssPathPtr_->header.frame_id = staticTransformsPtr_->getMapFrame();
  leftGnssPathPtr_->header.stamp = gnssTimeK;
  leftGnssPathPtr_->poses.push_back(pose);
  pubLeftGnssPath_.publish(leftGnssPathPtr_);
  /// Right
  pose.header.frame_id = staticTransformsPtr_->getMapFrame();
  pose.header.stamp = gnssTimeK;
  pose.pose.position.x = rightPosition(0);  // + tf_T_C_GR.getOrigin().x();
  pose.pose.position.y = rightPosition(1);  // + tf_T_C_GR.getOrigin().y();
  pose.pose.position.z = rightPosition(2);  // + tf_T_C_GR.getOrigin().z();
  rightGnssPathPtr_->header.frame_id = staticTransformsPtr_->getMapFrame();
  rightGnssPathPtr_->header.stamp = gnssTimeK;
  rightGnssPathPtr_->poses.push_back(pose);
  pubRightGnssPath_.publish(rightGnssPathPtr_);
}

/// Worker Functions -----------------------
bool CompslamSe::alignImu_(const ros::Time& imuTimeK) {
  gtsam::Rot3 imuAttitude;
  static int alignImuCounter__ = -1;
  ++alignImuCounter__;
  if (graphMgr_.estimateAttitudeFromImu(imuGravityDirection_, imuAttitude, gravityConstant_, graphMgr_.getInitGyrBiasReference())) {
    imuAttitudeRoll_ = imuAttitude.roll();
    imuAttitudePitch_ = imuAttitude.pitch();
    std::cout << YELLOW_START << "FactorGraphFiltering" << COLOR_END
              << " Attitude of IMU is initialized. Determined Gravity Magnitude: " << gravityConstant_ << std::endl;
    return true;
  } else {
    return false;
  }
}

void CompslamSe::initGnss_(const gtsam::Point3& leftGnssCoordinates, const gtsam::Point3& rightGnssCoordinates) {
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
  globalAttitudeYaw_W_C0_ = yaw_W_C0;
  std::cout << YELLOW_START << "FactorGraphFiltering" << GREEN_START << " Initial global yaw of cabin is: " << 180 / M_PI * yaw_W_C0
            << std::endl;

  // Get initial GNSS position
  W_t_W_GnssL0_ = leftPosition;
}

// Graph initialization for roll & pitch from starting attitude, assume zero yaw
void CompslamSe::initGraph_(const ros::Time& timeStamp_k) {
  // Calculate initial attitude;
  gtsam::Rot3 yawR_W_C0 = gtsam::Rot3::Yaw(globalAttitudeYaw_W_C0_);
  gtsam::Rot3 yawR_W_I0 = yawR_W_C0 * tfToPose3(staticTransformsPtr_->T_C_Ic()).rotation();
  gtsam::Rot3 R_W_I0 = gtsam::Rot3::Ypr(yawR_W_I0.yaw(), imuAttitudePitch_, imuAttitudeRoll_);

  std::cout << YELLOW_START << "FactorGraphFiltering" << GREEN_START
            << " Total initial IMU attitude is Yaw/Pitch/Roll(deg): " << R_W_I0.ypr().transpose() * (180.0 / M_PI) << COLOR_END
            << std::endl;

  // Gravity
  graphMgr_.initImuIntegrators(gravityConstant_, imuGravityDirection_);
  // Initialize first node
  //// Initial orientation as quaternion
  tf::Quaternion tf_q_W_I0 = pose3ToTf(gtsam::Pose3(R_W_I0, gtsam::Point3(0.0, 0.0, 0.0))).getRotation();
  /// Add initial IMU translation based on intial orientation
  gtsam::Pose3 T_W_I0;
  if (usingGnssFlag_) {
    T_W_I0 = gtsam::Pose3(R_W_I0, transformGnssPointToImuFrame_(W_t_W_GnssL0_, tf_q_W_I0));
  } else {
    T_W_I0 = gtsam::Pose3(R_W_I0, gtsam::Point3(0.0, 0.0, 0.0));
    std::cout << YELLOW_START << "FactorGraphFiltering" << COLOR_END << " Initialized position to 0,0,0 because no GNSS is present."
              << std::endl;
  }
  /// Initialize graph node
  graphMgr_.initPoseVelocityBiasGraph(timeStamp_k.toSec(), T_W_I0);
  if (!usingGnssFlag_ && usingFallbackGraphFlag_) {
    graphMgr_.activateFallbackGraph();
  }
  // Read initial pose from graph
  T_W_I0 = gtsam::Pose3(graphMgr_.getGraphState().navState().pose().matrix());
  std::cout << YELLOW_START << "FactorGraphFiltering " << GREEN_START << " INIT t(x,y,z): " << T_W_I0.translation().transpose()
            << ", RPY(deg): " << T_W_I0.rotation().rpy().transpose() * (180.0 / M_PI) << COLOR_END << std::endl;
  std::cout << YELLOW_START << "FactorGraphFiltering" << COLOR_END << " Factor graph key of very first node: " << graphMgr_.getStateKey()
            << std::endl;
  // Write in tf member variable
  tf_T_W_I0_ = pose3ToTf(T_W_I0);
  // Initialize global pose
  tf_T_W_Ik_.setData(tf_T_W_I0_);
  tf_T_W_Ik_.frame_id_ = staticTransformsPtr_->getMapFrame();
  tf_T_W_Ik_.stamp_ = timeStamp_k;
  // Set flag
  initedGraphFlag_ = true;
}

void CompslamSe::optimizeGraph_() {
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

/// Utility -------------------------
void CompslamSe::convertNavSatToPositions(const gtsam::Point3& leftGnssCoordinate, const gtsam::Point3& rightGnssCoordinate,
                                          gtsam::Point3& leftPosition, gtsam::Point3& rightPosition) {
  /// Left
  leftPosition = gnssSensor_.gpsToCartesian(leftGnssCoordinate(0), leftGnssCoordinate(1), leftGnssCoordinate(2));

  /// Right
  rightPosition = gnssSensor_.gpsToCartesian(rightGnssCoordinate(0), rightGnssCoordinate(1), rightGnssCoordinate(2));
}

gtsam::Vector3 CompslamSe::transformGnssPointToImuFrame_(const gtsam::Point3& gnssPosition, const tf::Quaternion& tf_q_W_I) {
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

gtsam::Point3 CompslamSe::getRobotHeading_(const Eigen::Vector3d& leftPosition, const Eigen::Vector3d& rightPosition) {
  // Compute connecting unity vector
  gtsam::Point3 W_t_GnssR_GnssL = (leftPosition - rightPosition).normalized();
  W_t_GnssR_GnssL = W_t_GnssR_GnssL;
  // Compute forward pointing vector
  gtsam::Point3 zUnityVector(0.0, 0.0, -1.0);
  gtsam::Point3 W_t_heading = zUnityVector.cross(W_t_GnssR_GnssL).normalized();

  return W_t_heading;
}

double CompslamSe::computeYawFromHeadingVector_(const gtsam::Point3& headingVector) {
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

bool CompslamSe::toggleGnssFlag_(std_srvs::Empty::Request& /*request*/, std_srvs::Empty::Response& /*response*/) {
  usingGnssFlag_ = !usingGnssFlag_;
  ROS_WARN_STREAM("GNSS usage was toggled to " << usingGnssFlag_);
  return true;
}

/// Commodity -----------------------
void CompslamSe::readParams_(const ros::NodeHandle& privateNode) {
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
    imuRate_ = int(dParam);
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
  if (privateNode.getParam("graph_params/usingFallbackGraph", bParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - usingFallbackGraph: " << bParam);
    usingFallbackGraphFlag_ = bParam;
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
