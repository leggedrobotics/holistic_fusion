#include "compslam_se/CompslamSe.h"

namespace compslam_se {

// Public -----------------------------------------------------------
/// Constructor -----------
CompslamSe::CompslamSe() {
  std::cout << BLUE_START << "CompslamSe" << GREEN_START << " Instance created." << COLOR_END << std::endl;
}

/// Setup ------------
bool CompslamSe::setup(ros::NodeHandle& node, GraphConfig* graphConfigPtr, StaticTransforms* staticTransformsPtr) {
  std::cout << BLUE_START << "CompslamSe" << GREEN_START << " Setting up." << COLOR_END << std::endl;

  // Graph Config
  graphConfigPtr_ = graphConfigPtr;
  staticTransformsPtr_ = staticTransformsPtr;

  graphMgrPtr_ = new GraphManager(graphConfigPtr);

  // Configs
  graphMgrPtr_->getIsamParamsReference().findUnusedFactorSlots = graphConfigPtr_->findUnusedFactorSlots;
  graphMgrPtr_->getIsamParamsReference().enableDetailedResults = graphConfigPtr_->enableDetailedResults;
  graphMgrPtr_->getIsamParamsReference().relinearizeSkip = graphConfigPtr_->relinearizeSkip;
  graphMgrPtr_->getIsamParamsReference().enableRelinearization = graphConfigPtr_->enableRelinearization;
  graphMgrPtr_->getIsamParamsReference().evaluateNonlinearError = graphConfigPtr_->evaluateNonlinearError;
  graphMgrPtr_->getIsamParamsReference().cacheLinearizedFactors = graphConfigPtr_->cacheLinearizedFactors;
  graphMgrPtr_->getIsamParamsReference().enablePartialRelinearizationCheck = graphConfigPtr_->enablePartialRelinearizationCheck;

  // Publishers
  /// advertise odometry topic
  pubOptimizationPath_ = node.advertise<nav_msgs::Path>("/fg_filtering/optimization_path", ROS_QUEUE_SIZE);
  pubCompslamPath_ = node.advertise<nav_msgs::Path>("/fg_filtering/compslam_path", ROS_QUEUE_SIZE);
  pubLaserImuBias_ = node.advertise<sensor_msgs::Imu>("/fg_filtering/imu_bias", ROS_QUEUE_SIZE);
  pubLeftGnssPath_ = node.advertise<nav_msgs::Path>("/fg_filtering/gnss_path_left", ROS_QUEUE_SIZE);
  pubRightGnssPath_ = node.advertise<nav_msgs::Path>("/fg_filtering/gnss_path_right", ROS_QUEUE_SIZE);
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
  std::cout << BLUE_START << "CompslamSe" << COLOR_END << " Initialized thread for optimizing the graph in parallel." << std::endl;

  std::cout << BLUE_START << "CompslamSe" << GREEN_START << " Set up successfully." << COLOR_END << std::endl;
  return true;
}

bool CompslamSe::areYawAndPositionInited() {
  return foundInitialYawAndPositionFlag_;
}

void CompslamSe::activateFallbackGraph() {
  if (graphConfigPtr_->usingFallbackGraphFlag) {
    graphMgrPtr_->activateFallbackGraph();
  } else {
    std::cout << BLUE_START << "CompslamSe" << RED_START << " Not activating fallback graph, disabled in config." << COLOR_END
              << std::endl;
  }
}

bool CompslamSe::initYawAndPosition(const double globAttitude_W_I0, const Eigen::Vector3d& t_W_GnssL) {
  // Locking
  const std::lock_guard<std::mutex> initYawAndPositionLock(initYawAndPositionMutex_);
  if (!alignedImuFlag_) {
    std::cout << BLUE_START << "CompslamSe" << RED_START << " Tried to set initial yaw, but initial attitude is not yet set." << COLOR_END
              << std::endl;
    return false;
  }
  if (!areYawAndPositionInited()) {
    std::cout << BLUE_START << "CompslamSe" << GREEN_START << " Initial global yaw of has been set to " << globAttitude_W_I0 << "."
              << COLOR_END << std::endl;
    gtsam::Rot3 yawR_W_I0 = gtsam::Rot3::Yaw(globAttitude_W_I0_);
    // gtsam::Rot3 yawR_W_I0 = yawR_W_I0 * tfToPose3(staticTransformsPtr_->T_C_Ic()).rotation();
    gtsam::Rot3 R_W_I0 = gtsam::Rot3::Ypr(yawR_W_I0.yaw(), imuAttitudePitch_, imuAttitudeRoll_);
    tf::Quaternion tf_q_W_I0 = pose3ToTf(gtsam::Pose3(R_W_I0, gtsam::Point3(0.0, 0.0, 0.0))).getRotation();
    // Set Member Variables
    globPosition_W_I0_ = transformLeftGnssPointToImuFrame_(t_W_GnssL, tf_q_W_I0);
    globAttitude_W_I0_ = globAttitude_W_I0;
    foundInitialYawAndPositionFlag_ = true;
    return true;
  } else {
    std::cout << BLUE_START << "CompslamSe" << RED_START << " Tried to set initial yaw, but it has been set before." << COLOR_END
              << std::endl;
    return false;
  }
}

bool CompslamSe::initYawAndPosition(Eigen::Matrix4d T_O_I) {
  gtsam::Pose3 T_O_Ik(T_O_I);

  const std::lock_guard<std::mutex> initYawAndPositionLock(initYawAndPositionMutex_);
  if (!alignedImuFlag_) {
    std::cout << BLUE_START << "CompslamSe" << RED_START << " Tried to set initial yaw, but initial attitude is not yet set." << COLOR_END
              << std::endl;
    return false;
  }
  if (!areYawAndPositionInited()) {
    globPosition_W_I0_ = T_O_Ik.translation();
    globAttitude_W_I0_ = T_O_Ik.rotation().yaw();
    foundInitialYawAndPositionFlag_ = true;
    return true;
  } else {
    std::cout << BLUE_START << "CompslamSe" << RED_START << " Tried to set initial yaw, but it has been set before." << COLOR_END
              << std::endl;
    return false;
  }
}

// Private ---------------------------------------------------------------
/// Callbacks -----------------------
bool CompslamSe::addImuMeasurement(const Eigen::Vector3d& linearAcc, const Eigen::Vector3d& angularVel, const ros::Time& imuTimeK,
                                   std::shared_ptr<InterfacePrediction>& predictionPtr) {
  // Static Members
  /// Prediction
  static Eigen::Matrix4d T_W_O__ = Eigen::Matrix4d::Identity();
  static Eigen::Matrix4d T_O_Ik__ = Eigen::Matrix4d::Identity();
  static Eigen::Vector3d I_v_W_I__ = Eigen::Vector3d();
  static Eigen::Vector3d I_w_W_I__ = Eigen::Vector3d();
  /// Other
  static Eigen::Matrix4d T_O_Ikm1__;
  static int imuCabinCallbackCounter__ = -1;
  static int count_imu_to_graph = 0;

  // Increase counter
  ++imuCabinCallbackCounter__;

  // Set IMU time
  ros::Time imuTimeKm1 = imuTimeKm1_;
  imuTimeK_ = imuTimeK;

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

  // Add measurement to buffer
  graphMgrPtr_->addToIMUBuffer(imuTimeK_.toSec(), linearAcc, angularVel);

  // If IMU not yet gravity aligned
  if (!alignedImuFlag_) {
    // Try to align
    if (alignImu_(imuTimeK)) {
      gtsam::Rot3 R_W_I0 = gtsam::Rot3::Ypr(0.0, imuAttitudePitch_, imuAttitudeRoll_);
      T_O_Ik__ = gtsam::Pose3(R_W_I0, gtsam::Point3(0.0, 0.0, 0.0)).matrix();
      I_v_W_I__ = gtsam::Velocity3(0.0, 0.0, 0.0);
      I_w_W_I__ = Eigen::Vector3d(0.0, 0.0, 0.0);
      alignedImuFlag_ = true;
    }
    // Otherwise
    else if (imuCabinCallbackCounter__ % int(graphConfigPtr_->imuRate) == 0) {
      std::cout << BLUE_START << "CompslamSe" << COLOR_END << " NOT ENOUGH IMU MESSAGES TO INITIALIZE POSE. WAITING FOR MORE..."
                << std::endl;
    }
    return false;
  }
  // Gravity Aligned but not yaw-aligned
  else if (!foundInitialYawAndPositionFlag_) {
    T_W_Ik = gtsam::NavState(gtsam::Rot3(T_O_Ik__.block<3, 3>(0, 0)), T_O_Ik__.block<3, 1>(0, 3), I_v_W_I__);
    if (imuCabinCallbackCounter__ % int(graphConfigPtr_->imuRate) == 0) {
      std::cout << BLUE_START << "CompslamSe" << COLOR_END << " IMU callback waiting for initialization of global yaw." << std::endl;
    }
  }
  // Initialization
  else if (!initedGraphFlag_) {
    initGraph_(imuTimeK);
    if (!graphConfigPtr_->usingGnssFlag) {
      // TODO
      activateFallbackGraph();
    }
    std::cout << BLUE_START << "CompslamSe" << GREEN_START << " ...graph is initialized." << COLOR_END << std::endl;
    T_W_Ik = gtsam::NavState(gtsam::Rot3(T_O_Ik__.block<3, 3>(0, 0)), T_O_Ik__.block<3, 1>(0, 3), I_v_W_I__);
  }
  // Normal operation
  else {
    // Add IMU factor and get propagated state
    T_W_Ik = graphMgrPtr_->addImuFactorAndGetState(imuTimeK.toSec(), linearAcc, angularVel, relocalizationFlag);
    imuAttitudeRoll_ = T_W_Ik.pose().rotation().roll();
    imuAttitudePitch_ = T_W_Ik.pose().rotation().pitch();

    // If relocalization happens --> write to map->odom
    if (relocalizationFlag) {
      // Print
      std::cout << BLUE_START << "CompslamSe" << GREEN_START << " Relocalization is needed. Publishing to map->odom." << COLOR_END
                << std::endl;
      // For this computation step assume T_O_Ik ~ T_O_Ikm1
      T_W_O__ = (T_W_Ik.pose().matrix() * T_O_Ikm1__.inverse());
      tf::Transform tf_T_W_O = matrix4ToTf(T_W_O__);
    }
    // Estimate state
    if (receivedOdometryFlag_) {
      T_O_Ik__ = T_W_O__.inverse() * T_W_Ik.pose().matrix();
      I_v_W_I__ = T_W_O__.block<3, 3>(0, 0).inverse() * T_W_Ik.velocity();
      I_w_W_I__ = graphMgrPtr_->getIMUBias().correctGyroscope(angularVel);
    }
  }

  // Write information to global variable
  tf_T_W_Ik_.setData(pose3ToTf(T_W_Ik.pose()));
  tf_T_W_Ik_.stamp_ = imuTimeK;

  // Publish
  predictionPtr = std::make_shared<InterfacePrediction>(T_W_O__, T_O_Ik__, I_v_W_I__, I_w_W_I__);

  // Write for next iteration
  T_O_Ikm1__ = T_O_Ik__;

  count_imu_to_graph++;

  // optimize graph at least every 100 IMU messages
  if( count_imu_to_graph > 100 )
  {
    // Mutex for optimizeGraph Flag
    const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
    optimizeGraphFlag_ = true;
    count_imu_to_graph = 0;
  }


  return true;
}

void CompslamSe::addOdometryMeasurement(const DeltaMeasurement6D& delta) {}  // TODO

void CompslamSe::addUnaryPoseMeasurement(const UnaryMeasurement6D& unary) {
  gtsam::Pose3 T_O_Ik(unary.measurementPose);

  if (initedGraphFlag_) {
    graphMgrPtr_->addPoseUnaryFactorToGlobalGraph(unary.time, unary.rate, unary.measurementNoise, T_O_Ik);
    graphMgrPtr_->addPoseUnaryFactorToFallbackGraph(unary.time, unary.rate, unary.measurementNoise, T_O_Ik);

    // Optimize
    {
      // Mutex for optimizeGraph Flag
      const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
      optimizeGraphFlag_ = true;
    }
  }
  if (!receivedOdometryFlag_) {
    receivedOdometryFlag_ = true;
  }
}

void CompslamSe::addOdometryMeasurement(const UnaryMeasurement6D& odometryKm1, const UnaryMeasurement6D& odometryK,
                                        const Eigen::Matrix<double, 6, 1>& poseBetweenNoise) {
  // Static variables
  static bool lidarUnaryFactorInitialized__ = false;
  static Eigen::Matrix4d T_O_Ij__;
  static gtsam::Pose3 T_O_Ij_Graph__;
  static gtsam::Key lastDeltaMeasurementKey__;

  // Start
  if (initedGraphFlag_) {
    // Create Pseudo Unary Factor
    if (graphMgrPtr_->globalGraphActiveFlag()) {
      /// Reset LiDAR Unary factor intiialization
      lidarUnaryFactorInitialized__ = false;
      // Write current compslam pose to latest delta pose
      T_O_Ij__ = odometryK.measurementPose;
    }  //
    else if (graphMgrPtr_->fallbackGraphActiveFlag()) {
      if (!lidarUnaryFactorInitialized__) {
        // Calculate state still from globalGraph
        std::cout << lastDeltaMeasurementKey__ << std::endl;
        T_O_Ij_Graph__ = graphMgrPtr_->calculateStateAtKey(lastDeltaMeasurementKey__).pose();
        std::cout << BLUE_START << "CompslamSe" << GREEN_START " Initialized LiDAR unary factors." << COLOR_END << std::endl;
        lidarUnaryFactorInitialized__ = true;
      }
      /// Delta pose
      gtsam::Pose3 T_Ij_Ik(T_O_Ij__.inverse() * odometryK.measurementPose);
      gtsam::Pose3 T_O_Ik = T_O_Ij_Graph__ * T_Ij_Ik;
      graphMgrPtr_->addPoseUnaryFactorToFallbackGraph(odometryK.time, odometryK.rate, odometryK.measurementNoise, T_O_Ik);
    }
    /// Delta pose
    Eigen::Matrix4d T_Ikm1_Ik = odometryKm1.measurementPose.inverse() * odometryK.measurementPose;
    lastDeltaMeasurementKey__ = graphMgrPtr_->addPoseBetweenFactorToGlobalGraph(odometryKm1.time, odometryK.time, odometryK.rate,
                                                                                poseBetweenNoise, gtsam::Pose3(T_Ikm1_Ik));

    // Optimize
    {
      // Mutex for optimizeGraph Flag
      const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
      optimizeGraphFlag_ = true;
    }
  }
  if (!receivedOdometryFlag_) {
    receivedOdometryFlag_ = true;
  }
}

void CompslamSe::addGnssPositionMeasurement(const Eigen::Vector3d& position, const Eigen::Vector3d& lastPosition,
                                            const Eigen::Vector3d& covarianceXYZ, const ros::Time& gnssTimeK, const double rate,
                                            double positionUnaryNoise, double covarianceXYZ_violation_threshold) {
  // Read covariance
  bool gnssCovarianceViolatedFlag = covarianceXYZ(0) > covarianceXYZ_violation_threshold ||
                                    covarianceXYZ(1) > covarianceXYZ_violation_threshold ||
                                    covarianceXYZ(2) > covarianceXYZ_violation_threshold;
  if (gnssCovarianceViolatedFlag && !gnssCovarianceViolatedFlag_) {
    std::cout << BLUE_START << "CompslamSe" << RED_START << " GNSS measurments now ABSENT due to too big covariance." << std::endl;
    gnssCovarianceViolatedFlag_ = gnssCovarianceViolatedFlag;
  } else if (!gnssCovarianceViolatedFlag && gnssCovarianceViolatedFlag_) {
    std::cout << BLUE_START << "CompslamSe" << GREEN_START << " GNSS returned. Low covariance." << std::endl;
    gnssCovarianceViolatedFlag_ = gnssCovarianceViolatedFlag;
  }

  // GNSS jumping?
  if ((lastPosition - position).norm() < 1.0) {  // gnssOutlierThreshold_) {
    ++gnssNotJumpingCounter_;
    if (gnssNotJumpingCounter_ == REQUIRED_GNSS_NUM_NOT_JUMPED) {
      std::cout << BLUE_START << "CompslamSe" << GREEN_START << " GNSS was not jumping recently. Jumping counter valid again."
                << std::endl;
    }
  } else {
    if (gnssNotJumpingCounter_ >= REQUIRED_GNSS_NUM_NOT_JUMPED) {
      std::cout << BLUE_START << "CompslamSe" << RED_START << " GNSS was jumping: Distance is " << (lastPosition - position).norm()
                << "m, larger than allowed " << 1.0  // gnssOutlierThreshold_
                << "m.  Reset outlier counter." << std::endl;
    }
    gnssNotJumpingCounter_ = 0;
  }

  // Case: GNSS is good --> Write to graph and perform logic
  if (!gnssCovarianceViolatedFlag_ && (gnssNotJumpingCounter_ >= REQUIRED_GNSS_NUM_NOT_JUMPED)) {
    // Position factor --> only use left GNSS
    gtsam::Point3 W_t_W_I = transformLeftGnssPointToImuFrame_(position, tf_T_W_Ik_.getRotation());
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
  
  if (!receivedOdometryFlag_) {
    receivedOdometryFlag_ = true;
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

void CompslamSe::addGnssHeadingMeasurement(const double attitude_W_I, const ros::Time& gnssTimeK, const double rate, double headingUnaryNoise) {
  //  gtsam::Rot3 yawR_W_C = gtsam::Rot3::Yaw(yaw_W_C);
  //  gtsam::Rot3 yawR_W_I = yawR_W_C * tfToPose3(staticTransformsPtr_->T_C_Ic()).rotation();
  //  double yaw_W_I = yawR_W_I.yaw();
  if (!gnssCovarianceViolatedFlag_ && (gnssNotJumpingCounter_ >= REQUIRED_GNSS_NUM_NOT_JUMPED)) {
    graphMgrPtr_->addGnssHeadingUnaryFactor(gnssTimeK.toSec(), rate, headingUnaryNoise, attitude_W_I);
    {
      // Mutex for optimizeGraph Flag
      const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
      optimizeGraphFlag_ = true;
    }
  }
}

void CompslamSe::addWheelOdometryMeasurement(const ros::Time& woTimeK, const double rate, const std::vector<double>& woSpeedNoise, const gtsam::Vector3& linearVel, const gtsam::Vector3& angularVel) {
  // transform from cabine frame to IMU frame
  gtsam::Matrix3 IcWOSkew = gtsam::skewSymmetric(angularVel);
  gtsam::Vector3 T_C_I = tfToPose3(staticTransformsPtr_->T_C_I()).translation();
  gtsam::Rot3 R_C_Ic = gtsam::Pose3(graphMgrPtr_->getGraphState().navState().pose().matrix()).rotation();
  gtsam::Vector3 linearVelIMU = R_C_Ic * (linearVel + IcWOSkew * T_C_I);

  graphMgrPtr_->addWheelOdometryVelocityFactor(woTimeK.toSec(), rate, woSpeedNoise, linearVelIMU, angularVel);
  
  {
    // Mutex for optimizeGraph Flag
    const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
    optimizeGraphFlag_ = true;
  }

}

/// Worker Functions -----------------------
bool CompslamSe::alignImu_(const ros::Time& imuTimeK) {
  gtsam::Rot3 imuAttitude;
  static int alignImuCounter__ = -1;
  ++alignImuCounter__;
  if (graphMgrPtr_->estimateAttitudeFromImu(graphConfigPtr_->imuGravityDirection, imuAttitude, gravityConstant_,
                                            graphMgrPtr_->getInitGyrBiasReference())) {
    imuAttitudeRoll_ = imuAttitude.roll();
    imuAttitudePitch_ = imuAttitude.pitch();
    std::cout << BLUE_START << "CompslamSe" << COLOR_END
              << " Attitude of IMU is initialized. Determined Gravity Magnitude: " << gravityConstant_ << std::endl;
    return true;
  } else {
    return false;
  }
}

// Graph initialization for roll & pitch from starting attitude, assume zero yaw
void CompslamSe::initGraph_(const ros::Time& timeStamp_k) {
  // Calculate initial attitude;
  gtsam::Rot3 yawR_W_I0 = gtsam::Rot3::Yaw(globAttitude_W_I0_);
  // gtsam::Rot3 yawR_W_I0 = yawR_W_C0 * tfToPose3(staticTransformsPtr_->T_C_Ic()).rotation();
  gtsam::Rot3 R_W_I0 = gtsam::Rot3::Ypr(yawR_W_I0.yaw(), imuAttitudePitch_, imuAttitudeRoll_);

  std::cout << BLUE_START << "CompslamSe" << GREEN_START
            << " Total initial IMU attitude is Yaw/Pitch/Roll(deg): " << R_W_I0.ypr().transpose() * (180.0 / M_PI) << COLOR_END
            << std::endl;

  // Gravity
  graphMgrPtr_->initImuIntegrators(gravityConstant_, graphConfigPtr_->imuGravityDirection);
  // Initialize first node
  //// Initial orientation as quaternion
  tf::Quaternion tf_q_W_I0 = pose3ToTf(gtsam::Pose3(R_W_I0, gtsam::Point3(0.0, 0.0, 0.0))).getRotation();
  /// Add initial IMU translation based on intial orientation
  gtsam::Pose3 T_W_I0;

  T_W_I0 = gtsam::Pose3(R_W_I0, globPosition_W_I0_);

  /// Initialize graph node
  graphMgrPtr_->initPoseVelocityBiasGraph(timeStamp_k.toSec(), T_W_I0);
  if (graphConfigPtr_->usingGnssFlag && usingFallbackGraphFlag_) {
    graphMgrPtr_->activateFallbackGraph();
  }
  // Read initial pose from graph
  T_W_I0 = gtsam::Pose3(graphMgrPtr_->getGraphState().navState().pose().matrix());
  std::cout << BLUE_START << "CompslamSe " << GREEN_START << " INIT t(x,y,z): " << T_W_I0.translation().transpose()
            << ", RPY(deg): " << T_W_I0.rotation().rpy().transpose() * (180.0 / M_PI) << COLOR_END << std::endl;
  std::cout << BLUE_START << "CompslamSe" << COLOR_END << " Factor graph key of very first node: " << graphMgrPtr_->getStateKey()
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
        std::cout << BLUE_START << "CompslamSe" << GREEN_START << " Whole optimization loop took "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(endLoopTime - startLoopTime).count() << " milliseconds."
                  << COLOR_END << std::endl;
      }
      // Transform pose
      gtsam::Pose3 T_W_I = optimizedNavState.pose();
      long seconds = long(currentTime);
      long nanoseconds = long((currentTime - seconds) * 1e9);
      signalLogger_.publishOptimizedPosition(seconds, nanoseconds, T_W_I);
      tf_T_W_I = pose3ToTf(T_W_I);

      // tf_T_W_C = tf_T_W_I * staticTransformsPtr_->T_Ic_C();
      //      // Publish path of optimized poses
      //      /// Pose
      //      T_W_C.header.stamp = compslamTimeK_;
      //      tf::poseTFToMsg(tf_T_W_C, T_W_C.pose);
      //      /// Path
      //      optimizationPathPtr_->header.frame_id = staticTransformsPtr_->getMapFrame();
      //      optimizationPathPtr_->header.stamp = compslamTimeK_;
      //      optimizationPathPtr_->poses.push_back(T_W_C);
      //      /// Publish
      //      pubOptimizationPath_.publish(optimizationPathPtr_);

      // Publish IMU Bias after update of graph
      sensor_msgs::Imu imuBiasMsg;
      imuBiasMsg.header.frame_id = staticTransformsPtr_->getImuFrame();
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

gtsam::Vector3 CompslamSe::transformLeftGnssPointToImuFrame_(const gtsam::Point3& t_W_GnssL, const tf::Quaternion& tf_q_W_I) {
  /// Translation in robot frame
  tf::Vector3 tf_W_t_W_GnssL(t_W_GnssL(0), t_W_GnssL(1), t_W_GnssL(2));
  tf::Transform tf_T_GnssL_I = staticTransformsPtr_->T_GnssL_I();
  tf::Vector3 tf_GnssL_t_GnssL_I = tf_T_GnssL_I.getOrigin();
  /// Global rotation
  tf::Transform tf_T_I_GnssL = staticTransformsPtr_->T_GnssL_I().inverse();
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
