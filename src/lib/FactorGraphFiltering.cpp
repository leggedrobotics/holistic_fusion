#include "fg_filtering/FactorGraphFiltering.h"

namespace fg_filtering {

// Public -----------------------------------------------------------
/// Constructor -----------
FactorGraphFiltering::FactorGraphFiltering() {
  ROS_INFO("FactorGraphFiltering instance created.");
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
  geodeticConverterLeft_ = geodetic_converter::GeodeticConverter();

  // Publishers
  /// advertise odometry topic
  pubOdometry_ = node.advertise<nav_msgs::Odometry>("/fg_filtering/transform_odom_base", ROS_QUEUE_SIZE);
  pubOdomPath_ = node.advertise<nav_msgs::Path>("/fg_filtering/odom_path", ROS_QUEUE_SIZE);
  pubOptimizationPath_ = node.advertise<nav_msgs::Path>("/fg_filtering/optimization_path", ROS_QUEUE_SIZE);
  pubCompslamPath_ = node.advertise<nav_msgs::Path>("/fg_filtering/compslam_path", ROS_QUEUE_SIZE);
  pubLaserImuBias_ = node.advertise<sensor_msgs::Imu>("/fg_filtering/imu_bias", ROS_QUEUE_SIZE);
  pubLeftGnssPath_ = node.advertise<nav_msgs::Path>("/fg_filtering/gnss_path_left", ROS_QUEUE_SIZE);
  pubRightGnssPath_ = node.advertise<nav_msgs::Path>("/fg_filtering/gnss_path_right", ROS_QUEUE_SIZE);
  excavatorStatePublisher_ = node.advertise<m545_msgs::M545State>("/m545_state", ROS_QUEUE_SIZE);
  /// Messages
  odomPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  optimizationPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  compslamPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  leftGnssPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  rightGnssPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);

  // Signal logger
  signalLogger_.setup(node);

  // Subscribers
  /// subscribe to remapped IMU topic
  subImuCabin_ = node.subscribe<sensor_msgs::Imu>("/imu_topic_cabin", ROS_QUEUE_SIZE, &FactorGraphFiltering::imuCabinCallback_, this,
                                                  ros::TransportHints().tcpNoDelay());
  ROS_INFO("Initialized IMU cabin subscriber.");
  subImuBase_ = node.subscribe<sensor_msgs::Imu>("/imu_topic_base", ROS_QUEUE_SIZE, &FactorGraphFiltering::imuBaseCallback_, this,
                                                 ros::TransportHints().tcpNoDelay());
  ROS_INFO("Initialized IMU base subscriber.");
  /// subscribe to remapped LiDAR odometry topic
  if (usingCompslamFlag_) {
    subLidarOdometry_ = node.subscribe<nav_msgs::Odometry>(
        "/lidar_odometry_topic", ROS_QUEUE_SIZE, &FactorGraphFiltering::lidarOdometryCallback_, this, ros::TransportHints().tcpNoDelay());
    ROS_INFO("Initialized LiDAR Odometry subscriber.");
  }
  /// subscribe to gnss topics using ROS exact sync policy in a single callback
  if (usingGnssFlag_) {
    subGnssLeft_.subscribe(node, "/gnss_topic_left", ROS_QUEUE_SIZE);
    subGnssRight_.subscribe(node, "/gnss_topic_right", ROS_QUEUE_SIZE);
    gnssExactSyncPtr_.reset(
        new message_filters::Synchronizer<_gnssExactSyncPolicy>(_gnssExactSyncPolicy(ROS_QUEUE_SIZE), subGnssLeft_, subGnssRight_));
    gnssExactSyncPtr_->registerCallback(boost::bind(&FactorGraphFiltering::gnssCallback_, this, _1, _2));
    ROS_INFO("Initialized GNSS subscriber (for both GNSS topics).");
  }
  /// Subscribe to measurements
  subMeasurements_ = node.subscribe<m545_msgs::M545Measurements>(
      "/measurement_topic", ROS_QUEUE_SIZE, &FactorGraphFiltering::measurementsCallback_, this, ros::TransportHints().tcpNoDelay());
  ROS_INFO("Initialized Measurements subscriber.");

  /// Initialize helper threads
  optimizeGraphThread_ = std::thread(&FactorGraphFiltering::optimizeGraph_, this);
  ROS_INFO("Initialized thread for optimizing the graph in parallel.");

  // Services
  toggleGnssUsageService_ = node.advertiseService("fg_filtering/toggle_gnss_usage", &FactorGraphFiltering::toggleGnssFlag_, this);

  return true;
}

// Private ---------------------------------------------------------------
/// Callbacks -----------------------
void FactorGraphFiltering::imuCabinCallback_(const sensor_msgs::Imu::ConstPtr& imuMsgPtr) {
  // Set IMU time
  const ros::Time imuTimeKm1 = imuTimeKm1_;
  const ros::Time imuTimeK = imuMsgPtr->header.stamp;
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

  // Add to buffer
  graphMgr_.addToIMUBuffer(imuTimeK.toSec(), linearAcc, angularVel);

  // If IMU not yet aligned
  if (!imuAlignedFlag_ && (initedGnssFlag_ || !usingGnssFlag_)) {
    alignImu_(imuTimeK);
  }  // Notification that waiting for GNSS
  else if (!initedGnssFlag_ && usingGnssFlag_) {
    ROS_INFO("Waiting for GNSS to provide global yaw.");
  }  // Initialize graph at next iteration step
  else if (imuAlignedFlag_ && !initedGraphFlag_) {
    ROS_WARN("Initializing the graph...");
    initGraph_(imuTimeK);
    ROS_WARN("...graph is initialized.");
  }
  // Add measurement to graph
  else if (initedGraphFlag_) {
    // Add IMU factor and get propagated state
    gtsam::NavState T_O_Ik = graphMgr_.addImuFactorAndGetState(imuTimeK.toSec());
    // Write information to global variable
    pose3ToTF(T_O_Ik.pose(), tf_T_O_Ik_);
    tf_T_O_Ik_.frame_id_ = staticTransformsPtr_->getOdomFrame();
    tf_T_O_Ik_.stamp_ = imuTimeK;
    // Publish state
    if (lidarCallbackCounter_ > NUM_LIDAR_CALLBACKS_UNTIL_START) {
      Eigen::Vector3d correctedAngularVel = graphMgr_.getIMUBias().correctGyroscope(angularVel);
      // Publish
      publishState_(imuTimeK, T_O_Ik, correctedAngularVel);
      // Log
      signalLogger_.publishLogger(T_O_Ik.pose(), graphMgr_.getIMUBias());
    } else {
      // Add zero motion factor in the very beginning to make biases converge
      graphMgr_.addZeroMotionFactor(0.02, imuTimeKm1.toSec(), imuTimeK.toSec(), gtsam::Pose3::identity());
      {
        // Mutex for optimizeGraph Flag
        const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
        optimizeGraphFlag_ = true;
      }
      // Publish zero motion state
      publishState_(imuTimeK, gtsam::NavState(initialImuPose_, gtsam::Velocity3(0.0, 0.0, 0.0)), Eigen::Vector3d(0.0, 0.0, 0.0));
    }
  }
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
  static bool pseudoUnaryFactorInitialized__ = false;
  static tf::Transform tf_T_O_Ikm1_Compslam__;

  // Output of compslam --> predicts absolute motion in lidar frame
  tf::Transform tf_T_O_Lk_Compslam, tf_T_O_Ck_Compslam;
  tf::Transform tf_T_O_Ik_Compslam;
  odomMsgToTF(*odomLidarPtr, tf_T_O_Lk_Compslam);
  ros::Time compslamTimeKm1;

  // Transform message to Imu frame
  tf_T_O_Ik_Compslam = tf_T_O_Lk_Compslam * staticTransformsPtr_->T_L_Ic();

  // Set initial compslam pose after third callback (because first compslam pose is wrong)
  ++lidarCallbackCounter_;
  if (lidarCallbackCounter_ < NUM_LIDAR_CALLBACKS_UNTIL_START) {
    compslamTimeK_ = odomLidarPtr->header.stamp;
    tf_T_I0_O_Compslam_ = tf_T_O_Ik_Compslam.inverse();
    return;
  }
  // Set LiDAR time
  compslamTimeKm1 = compslamTimeK_;
  compslamTimeK_ = odomLidarPtr->header.stamp;

  if (initedGraphFlag_) {
    if (!gnssAbsentFlag_) {
      /// Delta pose
      Eigen::Matrix4d T_Ikm1_Ik = computeDeltaPose(tf_T_O_Ikm1_Compslam__, tf_T_O_Ik_Compslam);
      gtsam::Pose3 lidarDeltaPose(T_Ikm1_Ik);
      // Write the lidar odom delta to the graph
      graphMgr_.addPoseBetweenFactor(lidarDeltaPose, compslamTimeKm1.toSec(), compslamTimeK_.toSec());

      {
        // Mutex for optimizeGraph Flag
        const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
        optimizeGraphFlag_ = true;
      }

      // Direct compslam estimate for base
      tf::Transform tf_T_I0_Ik_Compslam = tf_T_I0_O_Compslam_ * tf_T_O_Ik_Compslam;
      tf_T_O_Ck_Compslam = tf_T_O_I0_ * tf_T_I0_Ik_Compslam * staticTransformsPtr_->T_Ic_C();

      // Visualization of Compslam pose
      geometry_msgs::PoseStamped poseStamped;
      poseStamped.header.frame_id = staticTransformsPtr_->getOdomFrame();
      poseStamped.header.stamp = odomLidarPtr->header.stamp;
      tf::poseTFToMsg(tf_T_O_Ck_Compslam, poseStamped.pose);
      /// Path
      compslamPathPtr_->header.frame_id = staticTransformsPtr_->getOdomFrame();
      compslamPathPtr_->header.stamp = odomLidarPtr->header.stamp;
      compslamPathPtr_->poses.push_back(poseStamped);
      /// Publish
      pubCompslamPath_.publish(compslamPathPtr_);
    }
    // Set last pose for the next iteration
    tf_T_O_Ikm1_Compslam__ = tf_T_O_Ik_Compslam;
  }
}

void FactorGraphFiltering::gnssCallback_(const sensor_msgs::NavSatFix::ConstPtr& leftGnssMsgPtr,
                                         const sensor_msgs::NavSatFix::ConstPtr& rightGnssMsgPtr) {
  // Check whether GNSS should be used
  if (!usingGnssFlag_) {
    ROS_WARN("Received GNSS message, but usage is set to false.");
    return;
  }

  // First callback --> set position to zero
  if (firstGnssCallbackFlag_) {
    initGnss_(leftGnssMsgPtr, rightGnssMsgPtr);
    firstGnssCallbackFlag_ = false;
    return;
  }
  // Convert ros messages
  gtsam::Point3 leftPosition, rightPosition;
  convertNavSatToPositions(leftGnssMsgPtr, rightGnssMsgPtr, leftPosition, rightPosition);

  // Write to graph
  /// Read covariance
  bool covarianceViolated = leftGnssMsgPtr->position_covariance[0] > 1.0 || leftGnssMsgPtr->position_covariance[4] > 1.0 ||
                            leftGnssMsgPtr->position_covariance[8] > 1.0;
  /// Check whether covariance is okay, otherwise return
  if (initedGraphFlag_ && !covarianceViolated) {
    // Position factor --> only use left GNSS
    gtsam::Point3 W_t_W_I = transformGnssPointToImuFrame_(leftPosition);
    graphMgr_.addGnssPositionUnaryFactor(leftGnssMsgPtr->header.stamp.toSec(), W_t_W_I);

    // Heading factor
    /// Get heading (assuming that connection between antennas is perpendicular to heading)
    gtsam::Point3 W_t_heading = getRobotHeading_(leftPosition, rightPosition);

    // Modify graph
    // graphMgr_.addGnssHeadingUnaryFactor(leftGnssPtr->header.stamp.toSec(), W_t_heading, computeYawFromHeading_(W_t_heading));

    {
      // Mutex for optimizeGraph Flag
      const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
      optimizeGraphFlag_ = true;
    }
    {
      // Mutex for changing LiDAR constraint mode
      const std::lock_guard<std::mutex> gnssAnchorLock(gnssAnchorMutex_);
      gnssAbsentFlag_ = false;
    }
  } else if (covarianceViolated) {
    {
      // Mutex for changing LiDAR constraint mode
      const std::lock_guard<std::mutex> gnssAnchorLock(gnssAnchorMutex_);
      gnssAbsentFlag_ = false;
    }
    ROS_ERROR_STREAM("Covariance is too big, not using GNSS estimate.");
  }

  // Publish path
  /// Left
  //// Pose
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = staticTransformsPtr_->getOdomFrame();
  pose.header.stamp = leftGnssMsgPtr->header.stamp;
  pose.pose.position.x = leftPosition(0);  //+ tf_T_C_GL.getOrigin().x();
  pose.pose.position.y = leftPosition(1);  //+ tf_T_C_GL.getOrigin().y();
  pose.pose.position.z = leftPosition(2);  //+ tf_T_C_GL.getOrigin().z();
  //// Path
  leftGnssPathPtr_->header.frame_id = staticTransformsPtr_->getOdomFrame();
  leftGnssPathPtr_->header.stamp = leftGnssMsgPtr->header.stamp;
  leftGnssPathPtr_->poses.push_back(pose);
  pubLeftGnssPath_.publish(leftGnssPathPtr_);
  /// Right
  //// Pose
  pose.header.frame_id = staticTransformsPtr_->getOdomFrame();
  pose.header.stamp = rightGnssMsgPtr->header.stamp;
  pose.pose.position.x = rightPosition(0);  // + tf_T_C_GR.getOrigin().x();
  pose.pose.position.y = rightPosition(1);  // + tf_T_C_GR.getOrigin().y();
  pose.pose.position.z = rightPosition(2);  // + tf_T_C_GR.getOrigin().z();
  //// Path
  rightGnssPathPtr_->header.frame_id = staticTransformsPtr_->getOdomFrame();
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
  gtsam::Rot3 imu_attitude;
  if (graphMgr_.estimateAttitudeFromImu(imuTimeK.toSec(), imuGravityDirection_, imu_attitude, gravityConstant_,
                                        graphMgr_.getInitGyrBiasReference())) {
    initialImuAttitude_ = gtsam::Rot3::Ypr(initialGlobalYaw_, imu_attitude.pitch(), imu_attitude.roll());  // IMU yaw to zero
    ROS_WARN_STREAM("\033[33mFG_FILTERING\033[0mAttitude of IMU is initialized. Determined Gravity Magnitude: " << gravityConstant_);
    imuAlignedFlag_ = true;
  } else {
    ROS_INFO_STREAM("\033[33mFG_FILTERING\033[0m NOT ENOUGH IMU MESSAGES TO INITIALIZE POSE. WAITNG FOR MORE...\n");
  }
}

void FactorGraphFiltering::initGnss_(const sensor_msgs::NavSatFix::ConstPtr& leftGnssMsgPtr,
                                     const sensor_msgs::NavSatFix::ConstPtr& rightGnssMsgPtr) {
  // Initialize left converter
  geodeticConverterLeft_.initialiseReference(leftGnssMsgPtr->latitude, leftGnssMsgPtr->longitude, leftGnssMsgPtr->altitude);
  if (geodeticConverterLeft_.isInitialised()) {
    ROS_INFO("Left GNSS position was initialized.");
  } else {
    ROS_ERROR("Left GNSS position could not be initialized.");
  }

  // Get Positions
  gtsam::Point3 leftPosition, rightPosition;
  convertNavSatToPositions(leftGnssMsgPtr, rightGnssMsgPtr, leftPosition, rightPosition);

  // Get heading (assuming that connection between antennas is perpendicular to heading)
  gtsam::Point3 W_t_heading = getRobotHeading_(leftPosition, rightPosition);
  ROS_INFO_STREAM("Heading read from the GNSS is the following: " << W_t_heading);

  // Get initial global yaw
  initialGlobalYaw_ = computeYawFromHeading_(W_t_heading);
  gtsam::Rot3 yawRotationMatrix = gtsam::Rot3::Yaw(initialGlobalYaw_);
  ROS_INFO_STREAM("Initial global yaw is: " << 180 / M_PI * initialGlobalYaw_);

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
  /// Initial rotation known from IMU attitude and GNSS yaw
  initialImuPose_ = gtsam::Pose3(initialImuAttitude_, gtsam::Point3(0.0, 0.0, 0.0));
  //// Need to set tf_T_0_Ik_ because function transfromGnssPointToImuFrame makes use of its rotational part --> change later on
  pose3ToTF(initialImuPose_, tf_T_O_Ik_);
  /// Add initial IMU translation based on intial orientation
  initialImuPose_ = gtsam::Pose3(initialImuPose_.rotation(), transformGnssPointToImuFrame_(W_t_W_GnssL0_));
  /// Initialize graph node
  graphMgr_.initPoseVelocityBiasGraph(timeStamp_k.toSec(), initialImuPose_);
  // Read initial pose from graph
  initialImuPose_ = gtsam::Pose3(graphMgr_.getGraphState().navState().pose().matrix());
  ROS_WARN_STREAM("INIT t(x,y,z): " << initialImuPose_.translation().transpose()
                                    << ", RPY(deg): " << initialImuPose_.rotation().rpy().transpose() * (180.0 / M_PI) << "\n");
  ROS_WARN_STREAM("Factor graph key of very first node: " << graphMgr_.getStateKey() << std::endl);
  // Write in tf member variable
  pose3ToTF(initialImuPose_, tf_T_O_I0_);
  // Initialize global pose
  tf_T_O_Ik_.setData(tf_T_O_I0_);
  tf_T_O_Ik_.frame_id_ = staticTransformsPtr_->getOdomFrame();
  tf_T_O_Ik_.stamp_ = timeStamp_k;
  // Set flag
  initedGraphFlag_ = true;

  // Case that no GNSS is existent --> mark graph as initialized
  if (!usingGnssFlag_) {
    initedGraphFlag_ = true;
  }
}

void FactorGraphFiltering::optimizeGraph_() {
  // Preallocation
  int numLidarFactors = 0;
  // Pose Stamped
  geometry_msgs::PoseStamped poseStamped;
  poseStamped.header.frame_id = staticTransformsPtr_->getOdomFrame();
  // Transforms
  tf::Transform tf_T_O_I, tf_T_O_C;
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
      gtsam::NavState optimizedNavState = graphMgr_.updateGraphAndState();
      endLoopTime = std::chrono::high_resolution_clock::now();

      if (verboseLevel_ > 1) {
        ROS_INFO_STREAM("\033[92mWhole optimization loop took "
                        << std::chrono::duration_cast<std::chrono::milliseconds>(endLoopTime - startLoopTime).count()
                        << " milliseconds.\033[0m");
      }
      // Transform pose
      gtsam::Pose3 T_O_I = optimizedNavState.pose();
      pose3ToTF(T_O_I, tf_T_O_I);
      tf_T_O_C = tf_T_O_I * staticTransformsPtr_->T_Ic_C();

      // Publish path of optimized poses
      /// Pose
      poseStamped.header.stamp = compslamTimeK_;
      tf::poseTFToMsg(tf_T_O_C, poseStamped.pose);
      /// Path
      optimizationPathPtr_->header.frame_id = staticTransformsPtr_->getOdomFrame();
      optimizationPathPtr_->header.stamp = compslamTimeK_;
      optimizationPathPtr_->poses.push_back(poseStamped);
      /// Publish
      pubOptimizationPath_.publish(optimizationPathPtr_);

      // Publish IMU Bias after update of graph
      sensor_msgs::Imu imuBiasMsg;
      imuBiasMsg.header.frame_id = "/fg_odometry_imu";
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

void FactorGraphFiltering::publishState_(ros::Time imuTimeK, const gtsam::NavState& T_O_Ik, const Eigen::Vector3d& I_w_W_I) {
  // Used transforms
  tf::Transform tf_T_I_C;
  tf_T_I_C = staticTransformsPtr_->T_Ic_C();

  // Lookup chassis to cabin turn
  const double turnJointPosition = measurements_.actuatorStates_[m545_description::M545Topology::ActuatorEnum::TURN].position_;
  Eigen::AngleAxisd C_C_B(-turnJointPosition, Eigen::Vector3d(0.0, 0.0, 1.0));
  tf::Transform tf_T_C_B;
  pose3ToTF(C_C_B.toRotationMatrix(), tf_T_C_B);
  tf_T_C_B.setOrigin(tf::Vector3(0.0, 0.0, -staticTransformsPtr_->BC_z_offset()));

  // Lookup / compute remaining relative transformations and rotations
  tf::Transform R_B_C = tf::Transform(tf_T_C_B.getRotation()).inverse();
  tf::Transform R_C_Ic = tf::Transform(staticTransformsPtr_->T_C_Ic().getRotation());
  tf::Transform R_B_Ib = tf::Transform(staticTransformsPtr_->T_B_Ib().getRotation());
  tf::Vector3 B_t_Ic_C = R_B_C * (R_C_Ic * staticTransformsPtr_->T_Ic_C().getOrigin());
  tf::Vector3 B_t_C_B = R_B_C * tf_T_C_B.getOrigin();

  // Pose
  /// From Eigen to TF
  gtsam::Pose3 T_O_I = T_O_Ik.pose();
  tf::Transform tf_T_O_I;
  pose3ToTF(T_O_I, tf_T_O_I);
  /// Transform
  tf::Transform tf_T_O_C = tf_T_O_I * tf_T_I_C;
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
  ROS_WARN_STREAM("Body velocity of IMU in IMU frame: " << Ic_v_W_Ic);
  tf::Vector3 C_v_W_Ic = R_C_Ic * tf::Vector3(Ic_v_W_Ic(0), Ic_v_W_Ic(1), Ic_v_W_Ic(2));
  tf::Vector3 B_v_W_Ic = R_B_C * C_v_W_Ic;
  /// Compute velocity of base in world --> need "velocity composition rule"
  tf::Vector3 B_t_Ic_B = B_t_Ic_C + B_t_C_B;
  tf::Vector3 B_v_W_B = B_v_W_Ic + B_w_W_Ic.cross(B_t_Ic_B);
  ROS_WARN_STREAM("Body velocity of base in base frame: " << B_v_W_B.x() << ", " << B_v_W_B.y() << ", " << B_v_W_B.z());
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
  /// Pose
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
  nav_msgs::OdometryPtr odomBaseMsgPtr(new nav_msgs::Odometry);
  odomBaseMsgPtr->header.frame_id = staticTransformsPtr_->getOdomFrame();
  odomBaseMsgPtr->child_frame_id = staticTransformsPtr_->getCabinFrame();
  odomBaseMsgPtr->header.stamp = imuTimeK;
  tf::poseTFToMsg(tf_T_O_C, odomBaseMsgPtr->pose.pose);
  pubOdometry_.publish(odomBaseMsgPtr);
  ;
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
  std::unique_ptr<double> leftEastPtr = std::make_unique<double>();
  std::unique_ptr<double> leftNorthPtr = std::make_unique<double>();
  std::unique_ptr<double> leftUpPtr = std::make_unique<double>();
  geodeticConverterLeft_.geodetic2Enu(leftGnssMsgPtr->latitude, leftGnssMsgPtr->longitude, leftGnssMsgPtr->altitude, leftEastPtr.get(),
                                      leftNorthPtr.get(), leftUpPtr.get());
  leftPosition = gtsam::Point3(*leftEastPtr, *leftNorthPtr, *leftUpPtr);
  /// Right
  auto rightEastPtr = std::make_unique<double>();
  auto rightNorthPtr = std::make_unique<double>();
  auto rightUpPtr = std::make_unique<double>();
  geodeticConverterLeft_.geodetic2Enu(rightGnssMsgPtr->latitude, rightGnssMsgPtr->longitude, rightGnssMsgPtr->altitude, rightEastPtr.get(),
                                      rightNorthPtr.get(), rightUpPtr.get());
  rightPosition = gtsam::Point3(*rightEastPtr, *rightNorthPtr, *rightUpPtr);
}

gtsam::Vector3 FactorGraphFiltering::transformGnssPointToImuFrame_(const gtsam::Point3& gnssPosition) {
  /// Translation in robot frame
  tf::Vector3 tf_W_t_W_GnssL(gnssPosition(0), gnssPosition(1), gnssPosition(2));
  tf::Transform tf_T_GnssL_I = staticTransformsPtr_->T_GnssL_C() * staticTransformsPtr_->T_C_Ic();
  tf::Vector3 tf_GnssL_t_GnssL_I = tf_T_GnssL_I.getOrigin();
  /// Global rotation
  tf::Transform tf_T_I_GnssL = staticTransformsPtr_->T_Ic_C() * staticTransformsPtr_->T_C_GnssL();
  tf::Quaternion tf_q_W_GnssL = (tf_T_O_Ik_ * tf_T_I_GnssL).getRotation();
  tf::Transform tf_R_W_GnssL = tf::Transform::getIdentity();
  tf_R_W_GnssL.setRotation(tf_q_W_GnssL);
  /// Translation in global frame
  tf::Vector3 tf_W_t_GnssL_I = tf_R_W_GnssL * tf_GnssL_t_GnssL_I;

  /// Shift observed GNSS position to IMU frame (instead of GNSS antenna)
  tf::Vector3 tf_W_t_W_I = tf_W_t_W_GnssL + tf_W_t_GnssL_I;

  /// Modify graph
  gtsam::Vector3 W_t_W_I = {tf_W_t_W_I.x(), tf_W_t_W_I.y(), tf_W_t_W_I.z()};
  return W_t_W_I;
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

double FactorGraphFiltering::computeYawFromHeading_(const gtsam::Point3& headingVector) {
  double yaw = M_PI / 2.0 + atan2(headingVector(1), headingVector(0));
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

  // Factor Graph
  if (privateNode.getParam("graph_params/imuRate", dParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - IMU rate for preintegrator: " << dParam);
    graphMgr_.setImuRate(dParam);
  }
  if (privateNode.getParam("graph_params/lidarRate", dParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - LiDAR rate: " << dParam);
    graphMgr_.setLidarRate(dParam);
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
    ROS_WARN_STREAM("Set pose noise to " << poseBetweenNoise[0] << "," << poseBetweenNoise[1] << "," << poseBetweenNoise[2] << ","
                                         << poseBetweenNoise[3] << "," << poseBetweenNoise[4] << "," << poseBetweenNoise[5]);
    graphMgr_.setPoseBetweenNoise(poseBetweenNoise);
  } else {
    std::runtime_error("poseBetweenNoise needs to be set in config file.");
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
}

}  // end namespace fg_filtering
