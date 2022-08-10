#include "compslam_se/CompslamSe.h"

namespace compslam_se {

// Public -----------------------------------------------------------
/// Constructor -----------
CompslamSe::CompslamSe() {
  std::cout << YELLOW_START << "CompslamSe" << GREEN_START << " Instance created." << COLOR_END << std::endl;
}

/// Setup ------------
bool CompslamSe::setup(GraphConfig* graphConfigPtr, StaticTransforms* staticTransformsPtr) {
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

  /// Initialize helper threads
  optimizeGraphThread_ = std::thread(&CompslamSe::optimizeGraph_, this);
  std::cout << YELLOW_START << "CompslamSe" << COLOR_END << " Initialized thread for optimizing the graph in parallel." << std::endl;

  std::cout << YELLOW_START << "CompslamSe" << GREEN_START << " Set up successfully." << COLOR_END << std::endl;
  return true;
}

bool CompslamSe::yawAndPositionInited() {
  return foundInitialYawAndPositionFlag_;
}

void CompslamSe::activateFallbackGraph() {
  if (graphConfigPtr_->usingFallbackGraphFlag) {
    graphMgrPtr_->activateFallbackGraph();
  } else {
    std::cout << YELLOW_START << "CompslamSe" << RED_START << " Not activating fallback graph, disabled in config." << COLOR_END
              << std::endl;
  }
}

bool CompslamSe::initYawAndPosition(const double yaw_W_frame1, const std::string& frame1, const Eigen::Vector3d& W_t_W_frame2,
                                    const std::string& frame2) {
  assert(frame1 == staticTransformsPtr_->getCabinFrame() || frame1 == staticTransformsPtr_->getLidarFrame());
  assert(frame2 == staticTransformsPtr_->getLeftGnssFrame());  // frame will be ignored if gnss usage is set to false

  // Transform yaw to imu frame
  gtsam::Rot3 yawR_W_frame1 = gtsam::Rot3::Yaw(yaw_W_frame1);
  gtsam::Rot3 yawR_W_I0;
  if (frame1 == staticTransformsPtr_->getCabinFrame()) {
    std::cout << YELLOW_START << "CompslamSe" << GREEN_START << " Setting yaw in CABIN frame." << COLOR_END << std::endl;
    yawR_W_I0 = yawR_W_frame1 * gtsam::Pose3(staticTransformsPtr_->T_C_I()).rotation();
  } else if (frame1 == staticTransformsPtr_->getLidarFrame()) {
    yawR_W_I0 = yawR_W_frame1 * gtsam::Pose3(staticTransformsPtr_->T_L_I()).rotation();
    std::cout << YELLOW_START << "CompslamSe" << GREEN_START << " Setting yaw in LiDAR frame." << COLOR_END << std::endl;
  }

  // Locking
  const std::lock_guard<std::mutex> initYawAndPositionLock(initYawAndPositionMutex_);
  if (not alignedImuFlag_) {
    std::cout << YELLOW_START << "CompslamSe" << RED_START << " Tried to set initial yaw, but initial attitude is not yet set." << COLOR_END
              << std::endl;
    return false;
  } else if (not yawAndPositionInited()) {
    yaw_W_I0_ = yawR_W_I0.yaw();
    std::cout << YELLOW_START << "CompslamSe" << GREEN_START << " Initial global yaw of imu in world has been set to (deg) "
              << 180.0 * yaw_W_I0_ / M_PI << "." << COLOR_END << std::endl;

    gtsam::Rot3 R_W_I0 = gtsam::Rot3::Ypr(yawR_W_I0.yaw(), imuAttitudePitch_, imuAttitudeRoll_);
    // Set Member Variables
    // TODO replace with actual frame setting
    if (graphConfigPtr_->usingGnssFlag) {
      W_t_W_I0_ = transformLeftGnssPointToImuFrame_(W_t_W_frame2, R_W_I0);
    } else {
      W_t_W_I0_ = W_t_W_frame2;
    }
    foundInitialYawAndPositionFlag_ = true;
    return true;
  } else {
    std::cout << YELLOW_START << "CompslamSe" << RED_START << " Tried to set initial yaw, but it has been set before." << COLOR_END
              << std::endl;
    return false;
  }
}

bool CompslamSe::initYawAndPosition(Eigen::Matrix4d T_O_I) {
  gtsam::Pose3 T_O_Ik(T_O_I);

  const std::lock_guard<std::mutex> initYawAndPositionLock(initYawAndPositionMutex_);
  if (!alignedImuFlag_) {
    std::cout << YELLOW_START << "CompslamSe" << RED_START << " Tried to set initial yaw, but initial attitude is not yet set." << COLOR_END
              << std::endl;
    return false;
  }
  if (not yawAndPositionInited()) {
    W_t_W_I0_ = T_O_Ik.translation();
    yaw_W_I0_ = T_O_Ik.rotation().yaw();
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
bool CompslamSe::addImuMeasurement(const Eigen::Vector3d& linearAcc, const Eigen::Vector3d& angularVel, const double imuTimeK,
                                   std::shared_ptr<InterfacePrediction>& predictionPtr) {
  // Static Members
  static gtsam::Pose3 T_W_I_km1__;
  static int imuCabinCallbackCounter__ = -1;

  // Increase counter
  ++imuCabinCallbackCounter__;

  double imuTimeKm1 = imuTimeK_;
  imuTimeK_ = imuTimeK;

  // Filter out imu messages with same time stamp
  if (std::abs(imuTimeK_ - imuTimeKm1) < 1e-8) {
    std::cout << YELLOW_START << " CompslamSe" << RED_START << " Imu time " << std::setprecision(14) << imuTimeK << " was repeated."
              << COLOR_END << std::endl;
    return false;
  }

  // Loop variables
  bool relocalizationFlag = false;

  // Add measurement to buffer
  graphMgrPtr_->addToIMUBuffer(imuTimeK, linearAcc, angularVel);

  // Variable of odometry
  gtsam::Pose3 T_O_Ik;

  // If IMU not yet gravity aligned
  if (!alignedImuFlag_) {  // Not yet gravity aligned
    // Try to align
    if (alignImu_()) {
      gtsam::Rot3 R_W_I0 = gtsam::Rot3::Ypr(0.0, imuAttitudePitch_, imuAttitudeRoll_);
      T_W_Ik_ = gtsam::Pose3(R_W_I0, gtsam::Point3(0.0, 0.0, 0.0));
      T_W_O_ = gtsam::Pose3();
      I_v_W_I_ = gtsam::Vector3(0.0, 0.0, 0.0);
      I_w_W_I_ = gtsam::Vector3(0.0, 0.0, 0.0);
      alignedImuFlag_ = true;
    } else if (imuCabinCallbackCounter__ % int(graphConfigPtr_->imuRate) == 0) {
      std::cout << YELLOW_START << "CompslamSe" << COLOR_END << " NOT ENOUGH IMU MESSAGES TO INITIALIZE POSE. WAITING FOR MORE..."
                << std::endl;
      return false;
    }
  } else if (not yawAndPositionInited()) {  // Gravity Aligned but not yaw-aligned
    if (imuCabinCallbackCounter__ % int(graphConfigPtr_->imuRate) == 0) {
      std::cout << YELLOW_START << "CompslamSe" << COLOR_END << " IMU callback waiting for initialization of global yaw." << std::endl;
    }
  } else if (!initedGraphFlag_) {  // Initialization
    initGraph_(imuTimeK);
    if (!graphConfigPtr_->usingGnssFlag) {
      // TODO
      activateFallbackGraph();
    }
    std::cout << YELLOW_START << "CompslamSe" << GREEN_START << " ...graph is initialized." << COLOR_END << std::endl;
    // Do nothing
  } else {  // Normal operation
    // Add IMU factor and get propagated state
    gtsam::NavState T_W_Ik_nav = graphMgrPtr_->addImuFactorAndGetState(imuTimeK, linearAcc, angularVel, relocalizationFlag);

    // Assign poses and velocities ---------------------------------------------------
    T_W_Ik_ = T_W_Ik_nav.pose();
    I_v_W_I_ = T_W_Ik_nav.bodyVelocity();
    I_w_W_I_ = graphMgrPtr_->getIMUBias().correctGyroscope(angularVel);
    imuAttitudeRoll_ = T_W_Ik_.rotation().roll();
    imuAttitudePitch_ = T_W_Ik_.rotation().pitch();

    // If relocalization happens --> write to map->odom ------------------------------------
    if (relocalizationFlag) {
      // Print
      std::cout << YELLOW_START << "CompslamSe" << GREEN_START << " Relocalization is needed. Publishing to map->odom." << COLOR_END
                << std::endl;
      // For this computation step assume T_O_Ik ~ T_O_Ikm1
      gtsam::Pose3 T_I_O_km1 = T_W_I_km1__.inverse() * T_W_O_;
      T_W_O_ = T_W_Ik_ * T_I_O_km1;
    }

    // Convert to odom frame ----------------------------------------------------------------
    T_O_Ik = T_W_O_.inverse() * T_W_Ik_;

    // Estimate state
    if (receivedOdometryFlag_) {
    }
  }

  // Publish
  predictionPtr =
      std::make_shared<InterfacePrediction>(T_W_O_.matrix(), T_O_Ik.matrix(), Eigen::Vector3d(I_v_W_I_), Eigen::Vector3d(I_w_W_I_));

  // Write for next iteration
  T_W_I_km1__ = T_W_Ik_;
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
  static Eigen::Matrix4d T_I0_Ij__;
  static gtsam::Pose3 T_O_Ij_Graph__;
  static gtsam::Key lastDeltaMeasurementKey__;

  // Start
  if (initedGraphFlag_) {
    // Create Pseudo Unary Factor
    if (graphMgrPtr_->globalGraphActiveFlag()) {
      /// Reset LiDAR Unary factor intiialization
      lidarUnaryFactorInitialized__ = false;
      // Write current compslam pose to latest delta pose
      T_I0_Ij__ = odometryK.measurementPose;
    }  //
    else if (graphMgrPtr_->fallbackGraphActiveFlag()) {
      if (!lidarUnaryFactorInitialized__) {
        // Calculate state still from globalGraph
        std::cout << lastDeltaMeasurementKey__ << std::endl;
        T_O_Ij_Graph__ = graphMgrPtr_->calculateStateAtKey(lastDeltaMeasurementKey__).pose();
        std::cout << YELLOW_START << "CompslamSe" << GREEN_START " Initialized LiDAR unary factors." << COLOR_END << std::endl;
        lidarUnaryFactorInitialized__ = true;
      }
      /// Delta pose
      gtsam::Pose3 T_Ij_Ik(T_I0_Ij__.inverse() * odometryK.measurementPose);
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

void CompslamSe::addGnssPositionMeasurement(const Eigen::Vector3d& W_t_W_frame, const Eigen::Vector3d& W_t_W_frame_km1,
                                            const Eigen::Vector3d& covarianceXYZ, const double gnssTimeK, const double rate,
                                            double positionUnaryNoise) {
  // Read covariance
  bool gnssCovarianceViolatedFlag = covarianceXYZ(0) > GNSS_COVARIANCE_VIOLATION_THRESHOLD ||
                                    covarianceXYZ(1) > GNSS_COVARIANCE_VIOLATION_THRESHOLD ||
                                    covarianceXYZ(2) > GNSS_COVARIANCE_VIOLATION_THRESHOLD;
  if (gnssCovarianceViolatedFlag && !gnssCovarianceViolatedFlag_) {
    std::cout << YELLOW_START << "CompslamSe" << RED_START << " Gnss measurments now ABSENT due to too big covariance." << std::endl;
    gnssCovarianceViolatedFlag_ = gnssCovarianceViolatedFlag;
  } else if (!gnssCovarianceViolatedFlag && gnssCovarianceViolatedFlag_) {
    std::cout << YELLOW_START << "CompslamSe" << GREEN_START << " Gnss returned. Low covariance." << std::endl;
    gnssCovarianceViolatedFlag_ = gnssCovarianceViolatedFlag;
  }

  // Gnss jumping?
  double jumpingDistance = (W_t_W_frame_km1 - W_t_W_frame).norm();
  if (jumpingDistance < 1.0) {  // gnssOutlierThreshold_) {
    ++gnssNotJumpingCounter_;
    if (gnssNotJumpingCounter_ == REQUIRED_GNSS_NUM_NOT_JUMPED) {
      std::cout << YELLOW_START << "CompslamSe" << GREEN_START << " Gnss was not jumping recently. Jumping counter valid again."
                << std::endl;
    }
  } else {
    if (gnssNotJumpingCounter_ >= REQUIRED_GNSS_NUM_NOT_JUMPED) {
      std::cout << YELLOW_START << "CompslamSe" << RED_START << " Gnss was jumping: Distance is " << jumpingDistance
                << "m, larger than allowed " << 1.0  // gnssOutlierThreshold_
                << "m.  Reset outlier counter." << std::endl;
    }
    gnssNotJumpingCounter_ = 0;
  }

  // Case: Gnss is good --> Write to graph and perform logic
  if (!gnssCovarianceViolatedFlag_ && (gnssNotJumpingCounter_ >= REQUIRED_GNSS_NUM_NOT_JUMPED)) {
    // Position factor --> only use left Gnss
    gtsam::Point3 W_t_W_I = transformLeftGnssPointToImuFrame_(W_t_W_frame, T_W_Ik_.rotation());
    if (graphMgrPtr_->getStateKey() == 0) {
      return;
    }
    graphMgrPtr_->addGnssPositionUnaryFactor(gnssTimeK, rate, positionUnaryNoise, W_t_W_I);
    {
      // Mutex for optimizeGraph Flag
      const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
      optimizeGraphFlag_ = true;
    }
    graphMgrPtr_->activateGlobalGraph();
  }
  // Case: Gnss is bad --> Do not write to graph, set flags for odometry unary factor to true
  else if (usingFallbackGraphFlag_) {
    graphMgrPtr_->activateFallbackGraph();
  }
}

void CompslamSe::addGnssHeadingMeasurement(const double yaw_W_frame, const std::string& frameName, const double gnssTimeK,
                                           const double rate, double headingUnaryNoise) {
  // Transform yaw to imu frame
  gtsam::Rot3 yawR_W_frame = gtsam::Rot3::Yaw(yaw_W_frame);
  gtsam::Rot3 yawR_W_I0 = yawR_W_frame * gtsam::Pose3(staticTransformsPtr_->T_C_I()).rotation();

  //  gtsam::Rot3 yawR_W_C = gtsam::Rot3::Yaw(yaw_W_C);
  //  gtsam::Rot3 yawR_W_I = yawR_W_C * tfToPose3(staticTransformsPtr_->T_C_Ic()).rotation();
  //  double yaw_W_I = yawR_W_I.yaw();
  if (!gnssCovarianceViolatedFlag_ && (gnssNotJumpingCounter_ >= REQUIRED_GNSS_NUM_NOT_JUMPED)) {
    graphMgrPtr_->addGnssHeadingUnaryFactor(gnssTimeK, rate, headingUnaryNoise, yawR_W_I0.yaw());
    {
      // Mutex for optimizeGraph Flag
      const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
      optimizeGraphFlag_ = true;
    }
  }
}

/// Worker Functions -----------------------
bool CompslamSe::alignImu_() {
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
void CompslamSe::initGraph_(const double timeStamp_k) {
  // Calculate initial attitude;
  gtsam::Rot3 yawR_W_I0 = gtsam::Rot3::Yaw(yaw_W_I0_);
  // gtsam::Rot3 yawR_W_I0 = yawR_W_C0 * tfToPose3(staticTransformsPtr_->T_C_Ic()).rotation();
  gtsam::Rot3 R_W_I0 = gtsam::Rot3::Ypr(yawR_W_I0.yaw(), imuAttitudePitch_, imuAttitudeRoll_);

  std::cout << YELLOW_START << "CompslamSe" << GREEN_START
            << " Total initial IMU attitude is Yaw/Pitch/Roll(deg): " << R_W_I0.ypr().transpose() * (180.0 / M_PI) << COLOR_END
            << std::endl;

  // Gravity
  graphMgrPtr_->initImuIntegrators(gravityConstant_, graphConfigPtr_->imuGravityDirection);
  /// Add initial IMU translation based on intial orientation
  gtsam::Pose3 T_W_I0;

  T_W_I0 = gtsam::Pose3(R_W_I0, W_t_W_I0_);

  /// Initialize graph node
  graphMgrPtr_->initPoseVelocityBiasGraph(timeStamp_k, T_W_I0);
  if (graphConfigPtr_->usingGnssFlag && usingFallbackGraphFlag_) {
    graphMgrPtr_->activateFallbackGraph();
  }
  // Read initial pose from graph
  T_W_I0 = gtsam::Pose3(graphMgrPtr_->getGraphState().navState().pose().matrix());
  std::cout << YELLOW_START << "CompslamSe " << GREEN_START << " INIT t(x,y,z): " << T_W_I0.translation().transpose()
            << ", RPY(deg): " << T_W_I0.rotation().rpy().transpose() * (180.0 / M_PI) << COLOR_END << std::endl;
  std::cout << YELLOW_START << "CompslamSe" << COLOR_END << " Factor graph key of very first node: " << graphMgrPtr_->getStateKey()
            << std::endl;
  // Write in tf member variable
  T_W_I0_ = T_W_I0;
  // Initialize global pose
  T_W_Ik_ = T_W_I0_;
  imuTimeK_ = timeStamp_k;
  // Set flag
  initedGraphFlag_ = true;
}

void CompslamSe::optimizeGraph_() {
  // Timing
  std::chrono::time_point<std::chrono::high_resolution_clock> startLoopTime;
  std::chrono::time_point<std::chrono::high_resolution_clock> endLoopTime;

  // While loop
  std::cout << YELLOW_START << "CompslamSe" << COLOR_END << " Thread for updating graph is ready." << std::endl;
  while (true) {
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
      // TODO proper datatype
      T_W_I_opt_ = optimizedNavState.pose();
      optTime_ = currentTime;

      long seconds = long(currentTime);
      long nanoseconds = long((currentTime - seconds) * 1e9);

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
      //      sensor_msgs::Imu imuBiasMsg;
      //      imuBiasMsg.header.frame_id = staticTransformsPtr_->getImuFrame();
      //      imuBiasMsg.header.stamp = ros::Time(compslamTimeK_);
      //      imuBiasMsg.linear_acceleration.x = graphMgrPtr_->getIMUBias().accelerometer()(0);
      //      imuBiasMsg.linear_acceleration.y = graphMgrPtr_->getIMUBias().accelerometer()(1);
      //      imuBiasMsg.linear_acceleration.z = graphMgrPtr_->getIMUBias().accelerometer()(2);
      //      imuBiasMsg.angular_velocity.x = graphMgrPtr_->getIMUBias().gyroscope()(0);
      //      imuBiasMsg.angular_velocity.y = graphMgrPtr_->getIMUBias().gyroscope()(1);
      //      imuBiasMsg.angular_velocity.z = graphMgrPtr_->getIMUBias().gyroscope()(2);
      //      pubLaserImuBias_.publish(imuBiasMsg);
    }  // else just sleep for a short amount of time before polling again
    else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

gtsam::Vector3 CompslamSe::transformLeftGnssPointToImuFrame_(const gtsam::Point3& W_t_W_GnssL, const gtsam::Rot3& R_W_I) {
  // Static transforms
  Eigen::Matrix4d T_I_GnssL = staticTransformsPtr_->T_GnssL_I().inverse();
  Eigen::Vector3d GnssL_t_GnssL_I = staticTransformsPtr_->T_GnssL_I().block<3, 1>(0, 3);

  /// Global rotation
  gtsam::Rot3 R_W_GnssL = R_W_I * gtsam::Pose3(T_I_GnssL).rotation();

  /// Translation in global frame
  Eigen::Vector3d W_t_GnssL_I = R_W_GnssL * GnssL_t_GnssL_I;

  /// Shift observed Gnss position to IMU frame (instead of Gnss antenna)
  gtsam::Vector3 W_t_W_I = W_t_W_GnssL + W_t_GnssL_I;
  return W_t_W_I;
}

}  // namespace compslam_se
