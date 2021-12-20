#include "compslam_se/CompslamSe.h"

namespace compslam_se {

// Public -----------------------------------------------------------
/// Constructor -----------
CompslamSe::CompslamSe() {
  std::cout << YELLOW_START << "CompslamSe" << GREEN_START << " Instance created." << COLOR_END << std::endl;
}

/// Setup ------------
bool CompslamSe::setup(ros::NodeHandle& node, ros::NodeHandle& privateNode, GraphConfig* graphConfigPtr,
                       StaticTransforms* staticTransformsPtr) {
  std::cout << YELLOW_START << "CompslamSe" << GREEN_START << " Setting up." << COLOR_END << std::endl;

  // Graph Config
  graphConfigPtr_ = graphConfigPtr;
  staticTransformsPtr_ = staticTransformsPtr;

  graphMgrPtr_ = new GraphManager(graphConfigPtr);

  // Configs
  graphMgrPtr_->getIsamParamsReference().findUnusedFactorSlots = graphConfigPtr_->findUnusedFactorSlots;
  graphMgrPtr_->getIsamParamsReference().setEnableDetailedResults(graphConfigPtr_->enableDetailedResults);
  graphMgrPtr_->getIsamParamsReference().setRelinearizeSkip(graphConfigPtr_->relinearizeSkip);
  graphMgrPtr_->getIsamParamsReference().setEnableRelinearization(graphConfigPtr_->enableRelinearization);
  graphMgrPtr_->getIsamParamsReference().setEvaluateNonlinearError(graphConfigPtr_->evaluateNonlinearError);
  graphMgrPtr_->getIsamParamsReference().setCacheLinearizedFactors(graphConfigPtr_->cacheLinearizedFactors);
  graphMgrPtr_->getIsamParamsReference().setEnablePartialRelinearizationCheck(graphConfigPtr_->enablePartialRelinearizationCheck);

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
  std::cout << YELLOW_START << "CompslamSe" << COLOR_END << " Initialized thread for optimizing the graph in parallel." << std::endl;

  std::cout << YELLOW_START << "CompslamSe" << GREEN_START << " Set up successfully." << COLOR_END << std::endl;
  return true;
}

bool CompslamSe::areYawAndPositionInited() {
  return foundInitialYawAndPositionFlag_;
}

bool CompslamSe::initYawAndPosition(const double yaw, const Eigen::Vector3d& position) {
  // Locking
  const std::lock_guard<std::mutex> initYawAndPositionLock(initYawAndPositionMutex_);
  if (!alignedImuFlag_) {
    std::cout << YELLOW_START << "CompslamSe" << RED_START << " Tried to set initial yaw, but initial attitude is not yet set." << COLOR_END
              << std::endl;
    return false;
  }
  if (!areYawAndPositionInited()) {
    globalAttitudeYaw_W_C0_ = yaw;
    std::cout << YELLOW_START << "CompslamSe" << GREEN_START << " Initial yaw has been set." << COLOR_END << std::endl;
    gtsam::Rot3 yawR_W_C0 = gtsam::Rot3::Yaw(globalAttitudeYaw_W_C0_);
    gtsam::Rot3 yawR_W_I0 = yawR_W_C0 * tfToPose3(staticTransformsPtr_->T_C_Ic()).rotation();
    gtsam::Rot3 R_W_I0 = gtsam::Rot3::Ypr(yawR_W_I0.yaw(), imuAttitudePitch_, imuAttitudeRoll_);
    tf::Quaternion tf_q_W_I0 = pose3ToTf(gtsam::Pose3(R_W_I0, gtsam::Point3(0.0, 0.0, 0.0))).getRotation();
    globalPosition_W_I0_ = transformGnssPointToImuFrame_(position, tf_q_W_I0);
    foundInitialYawAndPositionFlag_ = true;
    return true;
  } else {
    std::cout << YELLOW_START << "CompslamSe" << RED_START << " Tried to set initial yaw, but it has been set before." << COLOR_END
              << std::endl;
    return false;
  }
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
  if (!alignedImuFlag_) {
    // Add measurement to buffer
    graphMgrPtr_->addToIMUBuffer(imuTimeK.toSec(), linearAcc, angularVel);
    // Add to buffer
    if (alignImu_(imuTimeK)) {
      gtsam::Rot3 yawR_W_C0 = gtsam::Rot3::Yaw(0.0);
      gtsam::Rot3 yawR_W_I0 = yawR_W_C0 * tfToPose3(staticTransformsPtr_->T_C_Ic()).rotation();
      gtsam::Rot3 R_W_I0 = gtsam::Rot3::Ypr(yawR_W_I0.yaw(), imuAttitudePitch_, imuAttitudeRoll_);
      T_O_Ik__ = gtsam::Pose3(R_W_I0, gtsam::Point3(0.0, 0.0, 0.0)).matrix();
      I_v_W_I__ = gtsam::Velocity3(0.0, 0.0, 0.0);
      I_w_W_I__ = Eigen::Vector3d(0.0, 0.0, 0.0);
      alignedImuFlag_ = true;
    } else if (imuCabinCallbackCounter__ % int(graphConfigPtr_->imuRate) == 0) {
      // Add measurement to buffer
      graphMgrPtr_->addToIMUBuffer(imuTimeK.toSec(), linearAcc, angularVel);
      std::cout << YELLOW_START << "CompslamSe" << COLOR_END << " NOT ENOUGH IMU MESSAGES TO INITIALIZE POSE. WAITING FOR MORE..."
                << std::endl;
    }
    return false;
  } else if (!foundInitialYawAndPositionFlag_) {
    // Add measurement to buffer
    graphMgrPtr_->addToIMUBuffer(imuTimeK.toSec(), linearAcc, angularVel);
    if (imuCabinCallbackCounter__ % int(graphConfigPtr_->imuRate) == 0) {
      std::cout << YELLOW_START << "CompslamSe" << COLOR_END << " IMU callback waiting for initialization of global yaw." << std::endl;
    }
    T_W_Ik = gtsam::NavState(gtsam::Rot3(T_O_Ik__.block<3, 3>(0, 0)), T_O_Ik__.block<3, 1>(0, 3), I_v_W_I__);
  } else if (!initedGraphFlag_) {
    // Add measurement to buffer
    graphMgrPtr_->addToIMUBuffer(imuTimeK.toSec(), linearAcc, angularVel);
    initGraph_(imuTimeK);
    std::cout << YELLOW_START << "CompslamSe" << GREEN_START << " ...graph is initialized." << COLOR_END << std::endl;
    T_W_Ik = gtsam::NavState(gtsam::Rot3(T_O_Ik__.block<3, 3>(0, 0)), T_O_Ik__.block<3, 1>(0, 3), I_v_W_I__);
  } else {
    // Add IMU factor and get propagated state
    T_W_Ik = graphMgrPtr_->addImuFactorAndGetState(imuTimeK.toSec(), linearAcc, angularVel, relocalizationFlag);
    imuAttitudeRoll_ = T_W_Ik.pose().rotation().roll();
    imuAttitudePitch_ = T_W_Ik.pose().rotation().pitch();

    // If relocalization happens --> write to map->odom
    if (relocalizationFlag) {
      // Print
      std::cout << YELLOW_START << "CompslamSe" << GREEN_START << " Relocalization is needed. Publishing to map->odom." << std::endl;
      // For this computation step assume T_O_Ik ~ T_O_Ikm1
      T_W_O__ = (T_W_Ik.pose().matrix() * T_O_Ikm1__.inverse());
      tf::Transform tf_T_W_O = matrix4ToTf(T_W_O__);
    }
    // Estimate state
    if (lidarCallbackCounter_ > NUM_LIDAR_CALLBACKS_UNTIL_START) {
      T_O_Ik__ = T_W_O__.inverse() * T_W_Ik.pose().matrix();
      I_v_W_I__ = T_W_O__.block<3, 3>(0, 0).inverse() * T_W_Ik.velocity();
      I_w_W_I__ = graphMgrPtr_->getIMUBias().correctGyroscope(angularVel);
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

  return true;
}

void CompslamSe::addOdometryMeasurement(const Eigen::Matrix4d& T_O_Lk, const double rate, const std::vector<double>& poseBetweenNoise,
                                        const ros::Time& odometryTimeK) {
  // Static variables
  static bool lidarUnaryFactorInitialized__ = false;
  static tf::Transform tf_compslam_T_O_Ikm1__;
  static gtsam::Key lastDeltaMeasurementKey__;
  static tf::Transform tf_compslam_T_O_Ij__;
  static gtsam::Pose3 T_O_Ij_Graph__;

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
    std::cout << YELLOW_START << "CompslamSe" << COLOR_END << " Waiting until enough LiDAR messages have arrived..." << std::endl;
    compslamTimeK_ = odometryTimeK + ros::Duration(imuTimeOffset_);
    tf_compslam_T_I0_O_ = tf_compslam_T_O_Ik.inverse();
    tf_compslam_T_O_Ij__ = tf_compslam_T_O_Ik;
    tf_compslam_T_O_Ikm1__ = tf_compslam_T_O_Ik;
    return;
  }

  if (initedGraphFlag_) {
    if (graphMgrPtr_->globalGraphActiveFlag()) {
      /// Reset LiDAR Unary factor intiialization
      lidarUnaryFactorInitialized__ = false;
      // Write current compslam pose to latest delta pose
      tf_compslam_T_O_Ij__ = tf_compslam_T_O_Ik;
    }  //
    else if (graphMgrPtr_->fallbackGraphActiveFlag()) {
      if (!lidarUnaryFactorInitialized__) {
        // Calculate state still from globalGraph
        T_O_Ij_Graph__ = graphMgrPtr_->calculateStateAtKey(lastDeltaMeasurementKey__).pose();
        std::cout << YELLOW_START << "CompslamSe" << GREEN_START " Initialized LiDAR unary factors." << COLOR_END << std::endl;
        lidarUnaryFactorInitialized__ = true;
      }
      /// Delta pose
      gtsam::Pose3 T_Ij_Ik(computeDeltaPose(tf_compslam_T_O_Ij__, tf_compslam_T_O_Ik));
      gtsam::Pose3 T_O_Ik = T_O_Ij_Graph__ * T_Ij_Ik;
      graphMgrPtr_->addPoseUnaryFactorToFallbackGraph(compslamTimeK_.toSec(), rate, poseBetweenNoise, T_O_Ik);
    }
    // In any case: write the lidar odom delta to global graph
    /// Delta pose
    gtsam::Pose3 T_Ikm1_Ik = computeDeltaPose(tf_compslam_T_O_Ikm1__, tf_compslam_T_O_Ik);
    lastDeltaMeasurementKey__ =
        graphMgrPtr_->addPoseBetweenFactorToGlobalGraph(compslamTimeKm1.toSec(), compslamTimeK_.toSec(), rate, poseBetweenNoise, T_Ikm1_Ik);
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

void CompslamSe::addGnssPositionMeasurement(const Eigen::Vector3d& position, const Eigen::Vector3d& lastPosition,
                                            const Eigen::Vector3d& covarianceXYZ, const ros::Time& gnssTimeK, const double rate,
                                            double positionUnaryNoise) {
  static int gnssNotJumpingCounter__ = 0;

  // Read covariance
  bool gnssCovarianceViolatedFlag = covarianceXYZ(0) > GNSS_COVARIANCE_VIOLATION_THRESHOLD ||
                                    covarianceXYZ(1) > GNSS_COVARIANCE_VIOLATION_THRESHOLD ||
                                    covarianceXYZ(2) > GNSS_COVARIANCE_VIOLATION_THRESHOLD;
  if (gnssCovarianceViolatedFlag && !gnssCovarianceViolatedFlag_) {
    std::cout << YELLOW_START << "CompslamSe" << RED_START << " GNSS measurments now ABSENT due to too big covariance." << std::endl;
    gnssCovarianceViolatedFlag_ = gnssCovarianceViolatedFlag;
  } else if (!gnssCovarianceViolatedFlag && gnssCovarianceViolatedFlag_) {
    std::cout << YELLOW_START << "CompslamSe" << GREEN_START << " GNSS returned. Low covariance." << std::endl;
    gnssCovarianceViolatedFlag_ = gnssCovarianceViolatedFlag;
  }

  // GNSS jumping?
  if ((lastPosition - position).norm() < 1.0) {  // gnssOutlierThreshold_) {
    ++gnssNotJumpingCounter__;
    if (gnssNotJumpingCounter__ == REQUIRED_GNSS_NUM_NOT_JUMPED) {
      std::cout << YELLOW_START << "CompslamSe" << GREEN_START << " GNSS was not jumping recently. Jumping counter valid again."
                << std::endl;
    }
  } else {
    if (gnssNotJumpingCounter__ >= REQUIRED_GNSS_NUM_NOT_JUMPED) {
      std::cout << YELLOW_START << "CompslamSe" << RED_START << " GNSS was jumping: Distance is " << (lastPosition - position).norm()
                << "m, larger than allowed " << 1.0  // gnssOutlierThreshold_
                << "m.  Reset outlier counter." << std::endl;
    }
    gnssNotJumpingCounter__ = 0;
  }

  // Case: GNSS is good --> Write to graph and perform logic
  if (!gnssCovarianceViolatedFlag_ && (gnssNotJumpingCounter__ >= REQUIRED_GNSS_NUM_NOT_JUMPED)) {
    // Position factor --> only use left GNSS
    gtsam::Point3 W_t_W_I = transformGnssPointToImuFrame_(position, tf_T_W_Ik_.getRotation());
    if (graphMgrPtr_->getStateKey() == 0) {
      return;
    }
    graphMgrPtr_->addGnssPositionUnaryFactor(gnssTimeK.toSec(), rate, positionUnaryNoise, W_t_W_I);
    {
      // Mutex for optimizeGraph Flag
      const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
      optimizeGraphFlag_ = true;
    }
    graphMgrPtr_->activateGlobalGraph();
  }
  // Case: GNSS is bad --> Do not write to graph, set flags for odometry unary factor to true
  else if (usingFallbackGraphFlag_) {
    graphMgrPtr_->activateFallbackGraph();
  }

  // Publish path
  /// Left
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = staticTransformsPtr_->getMapFrame();
  pose.header.stamp = gnssTimeK;
  pose.pose.position.x = position(0);
  pose.pose.position.y = position(1);
  pose.pose.position.z = position(2);
  leftGnssPathPtr_->header.frame_id = staticTransformsPtr_->getMapFrame();
  leftGnssPathPtr_->header.stamp = gnssTimeK;
  leftGnssPathPtr_->poses.push_back(pose);
  pubLeftGnssPath_.publish(leftGnssPathPtr_);
  /// Right
  pose.header.frame_id = staticTransformsPtr_->getMapFrame();
  pose.header.stamp = gnssTimeK;
  pose.pose.position.x = position(0);  // + tf_T_C_GR.getOrigin().x();
  pose.pose.position.y = position(1);  // + tf_T_C_GR.getOrigin().y();
  pose.pose.position.z = position(2);  // + tf_T_C_GR.getOrigin().z();
  rightGnssPathPtr_->header.frame_id = staticTransformsPtr_->getMapFrame();
  rightGnssPathPtr_->header.stamp = gnssTimeK;
  rightGnssPathPtr_->poses.push_back(pose);
  pubRightGnssPath_.publish(rightGnssPathPtr_);
}

// void CompslamSe::addGnssYawMeasurement(const double yaw, const double lastYaw,
//                                            const Eigen::Vector3d& covarianceYaw, const ros::Time& gnssTimeK, const double rate,
//                                            double yawUnaryNoise) {
//  // Case: GNSS is good --> Write to graph and perform logic
//  if (!gnssCovarianceViolatedFlag_ && (gnssNotJumpingCounter__ >= REQUIRED_GNSS_NUM_NOT_JUMPED)) {
//    // Position factor --> only use left GNSS
//    gtsam::Point3 W_t_W_I = transformGnssPointToImuFrame_(leftPosition, tf_T_W_Ik_.getRotation());
//    if (graphMgrPtr_->getStateKey() == 0) {
//      return;
//    }
//    graphMgrPtr_->addGnssPositionUnaryFactor(gnssTimeK.toSec(), rate, positionUnaryNoise, W_t_W_I);
//
//    // Heading factor
//    /// Get heading (assuming that connection between antennas is perpendicular to heading)
//    gtsam::Point3 W_t_heading = getRobotHeading_(leftPosition, rightPosition);
//    double yaw_W_C = computeYawFromHeadingVector_(W_t_heading);
//    gtsam::Rot3 yawR_W_C = gtsam::Rot3::Yaw(yaw_W_C);
//    gtsam::Rot3 yawR_W_I = yawR_W_C * tfToPose3(staticTransformsPtr_->T_C_Ic()).rotation();
//
//    // Unary factor
//    gtsam::Rot3 R_W_I_approx = gtsam::Rot3::Ypr(yawR_W_I.yaw(), imuAttitudePitch_, imuAttitudeRoll_);
//    gtsam::Pose3 T_W_I_approx = gtsam::Pose3(R_W_I_approx, W_t_W_I);
//    // graphMgrPtr_->addPoseUnaryFactor(leftGnssMsgPtr->header.stamp.toSec(), T_W_I_approx);
//
//    {
//      // Mutex for optimizeGraph Flag
//      const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
//      optimizeGraphFlag_ = true;
//    }
//    graphMgrPtr_->activateGlobalGraph();
//  }
//    // Case: GNSS is bad --> Do not write to graph, set flags for odometry unary factor to true
//  else if (usingFallbackGraphFlag_) {
//    graphMgrPtr_->activateFallbackGraph();
//  }
//
//}

/// Worker Functions -----------------------
bool CompslamSe::alignImu_(const ros::Time& imuTimeK) {
  gtsam::Rot3 imuAttitude;
  static int alignImuCounter__ = -1;
  ++alignImuCounter__;
  if (graphMgrPtr_->estimateAttitudeFromImu(graphConfigPtr_->imuGravityDirection, imuAttitude, gravityConstant_,
                                            graphMgrPtr_->getInitGyrBiasReference())) {
    imuAttitudeRoll_ = imuAttitude.roll();
    imuAttitudePitch_ = imuAttitude.pitch();
    std::cout << YELLOW_START << "CompslamSe" << COLOR_END
              << " Attitude of IMU is initialized. Determined Gravity Magnitude: " << gravityConstant_ << std::endl;
    return true;
  } else {
    return false;
  }
}

// Graph initialization for roll & pitch from starting attitude, assume zero yaw
void CompslamSe::initGraph_(const ros::Time& timeStamp_k) {
  // Calculate initial attitude;
  gtsam::Rot3 yawR_W_C0 = gtsam::Rot3::Yaw(globalAttitudeYaw_W_C0_);
  gtsam::Rot3 yawR_W_I0 = yawR_W_C0 * tfToPose3(staticTransformsPtr_->T_C_Ic()).rotation();
  gtsam::Rot3 R_W_I0 = gtsam::Rot3::Ypr(yawR_W_I0.yaw(), imuAttitudePitch_, imuAttitudeRoll_);

  std::cout << YELLOW_START << "CompslamSe" << GREEN_START
            << " Total initial IMU attitude is Yaw/Pitch/Roll(deg): " << R_W_I0.ypr().transpose() * (180.0 / M_PI) << COLOR_END
            << std::endl;

  // Gravity
  graphMgrPtr_->initImuIntegrators(gravityConstant_, graphConfigPtr_->imuGravityDirection);
  // Initialize first node
  //// Initial orientation as quaternion
  tf::Quaternion tf_q_W_I0 = pose3ToTf(gtsam::Pose3(R_W_I0, gtsam::Point3(0.0, 0.0, 0.0))).getRotation();
  /// Add initial IMU translation based on intial orientation
  gtsam::Pose3 T_W_I0;

  T_W_I0 = gtsam::Pose3(R_W_I0, globalPosition_W_I0_);

  /// Initialize graph node
  graphMgrPtr_->initPoseVelocityBiasGraph(timeStamp_k.toSec(), T_W_I0);
  if (false && usingFallbackGraphFlag_) {
    graphMgrPtr_->activateFallbackGraph();
  }
  // Read initial pose from graph
  T_W_I0 = gtsam::Pose3(graphMgrPtr_->getGraphState().navState().pose().matrix());
  std::cout << YELLOW_START << "CompslamSe " << GREEN_START << " INIT t(x,y,z): " << T_W_I0.translation().transpose()
            << ", RPY(deg): " << T_W_I0.rotation().rpy().transpose() * (180.0 / M_PI) << COLOR_END << std::endl;
  std::cout << YELLOW_START << "CompslamSe" << COLOR_END << " Factor graph key of very first node: " << graphMgrPtr_->getStateKey()
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
      gtsam::NavState optimizedNavState = graphMgrPtr_->updateGraphAndState(currentTime);
      endLoopTime = std::chrono::high_resolution_clock::now();

      if (graphConfigPtr_->verboseLevel > 0) {
        std::cout << YELLOW_START << "CompslamSe" << GREEN_START << " Whole optimization loop took "
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
      imuBiasMsg.linear_acceleration.x = graphMgrPtr_->getIMUBias().accelerometer()(0);
      imuBiasMsg.linear_acceleration.y = graphMgrPtr_->getIMUBias().accelerometer()(1);
      imuBiasMsg.linear_acceleration.z = graphMgrPtr_->getIMUBias().accelerometer()(2);
      imuBiasMsg.angular_velocity.x = graphMgrPtr_->getIMUBias().gyroscope()(0);
      imuBiasMsg.angular_velocity.y = graphMgrPtr_->getIMUBias().gyroscope()(1);
      imuBiasMsg.angular_velocity.z = graphMgrPtr_->getIMUBias().gyroscope()(2);
      pubLaserImuBias_.publish(imuBiasMsg);
    }  // else just sleep for a short amount of time before polling again
    else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
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

}  // namespace compslam_se
