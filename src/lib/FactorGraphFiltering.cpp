#include "fg_filtering/FactorGraphFiltering.h"

namespace fg_filtering {

using std::asin;
using std::atan2;
using std::cos;
using std::fabs;
using std::pow;
using std::sin;
using std::sqrt;

FactorGraphFiltering::FactorGraphFiltering(float scanPeriod) {
  ROS_INFO("FactorGraphFiltering instance created.");
}

// Setup ----------------------------------------------------------------------------------------
bool FactorGraphFiltering::setup(ros::NodeHandle& node, ros::NodeHandle& privateNode) {
  // Variables for parameter fetching
  double dParam;
  int iParam;
  bool bParam;
  std::string sParam;

  // Set frames
  /// Initialize containers
  staticTransformsPtr_ = new StaticTransforms(privateNode);
  /// Odom
  if (privateNode.getParam("odomFrame", sParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - Odom frame set to: " << sParam);
    staticTransformsPtr_->setOdomFrame(sParam);
  } else {
    ROS_WARN("FactorGraphFiltering - Odom frame not set");
  }
  /// base_link
  if (privateNode.getParam("baseLinkFrame", sParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - base_link frame: " << sParam);
    staticTransformsPtr_->setBaseLinkFrame(sParam);
  } else
    ROS_WARN("FactorGraphFiltering - IMU frame not set for preintegrator");
  /// IMU
  if (privateNode.getParam("imuCabinFrame", sParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - IMU Cabin frame for preintegrator and tf: " << sParam);
    staticTransformsPtr_->setImuCabinFrame(sParam);
  } else
    ROS_WARN("FactorGraphFiltering - IMU Cabin frame not set for preintegrator");
  if (privateNode.getParam("imuRooftopFrame", sParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - IMU Rooftop frame for preintegrator and tf: " << sParam);
    staticTransformsPtr_->setImuRooftopFrame(sParam);
  } else
    ROS_WARN("FactorGraphFiltering - IMU Rooftop frame not set for preintegrator");
  /// LiDAR frame
  if (privateNode.getParam("lidarFrame", sParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - LiDAR frame: " << sParam);
    staticTransformsPtr_->setLidarFrame(sParam);
  } else {
    ROS_WARN("FactorGraphFiltering - LiDAR frame not set");
  }
  /// Cabin frame
  if (privateNode.getParam("cabinFrame", sParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - cabin frame: " << sParam);
    staticTransformsPtr_->setCabinFrame(sParam);
  } else {
    ROS_WARN("FactorGraphFiltering - cabin frame not set");
  }
  /// Left GNSS frame
  if (privateNode.getParam("leftGnssFrame", sParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - left GNSS frame: " << sParam);
    staticTransformsPtr_->setLeftGnssFrame(sParam);
  } else {
    ROS_WARN("FactorGraphFiltering - left GNSS frame not set");
  }
  /// Right GNSS frame
  if (privateNode.getParam("rightGnssFrame", sParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - right GNSS frame: " << sParam);
    staticTransformsPtr_->setRightGnssFrame(sParam);
  } else {
    ROS_WARN("FactorGraphFiltering - right GNSS frame not set");
  }

  // IMU gravity definition
  if (privateNode.getParam("imu_gravity_direction", sParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - gravity direction of IMU: " << sParam);
    setImuGravityDirection(sParam);
  } else {
    ROS_ERROR("FactorGraphFiltering - gravity direction of imu not set");
    throw std::runtime_error("Rosparam 'imu_gravity_direction' must be set.");
  }

  // Timing
  if (privateNode.getParam("imuRate", dParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - IMU rate for preintegrator: " << dParam);
    graphMgr_.setImuRate(dParam);
  }
  if (privateNode.getParam("lidarRate", dParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - LiDAR rate: " << dParam);
    graphMgr_.setLidarRate(dParam);
  }

  // Get transformations from URDF
  staticTransformsPtr_->findTransformations();

  // Geodetic Converter
  geodeticConverterLeft_ = geodetic_converter::GeodeticConverter();
  geodeticConverterRight_ = geodetic_converter::GeodeticConverter();

  // Factor Graph
  if (privateNode.getParam("accNoiseDensity", dParam)) {
    graphMgr_.setAccNoiseDensity(dParam);
  }
  if (privateNode.getParam("accBiasRandomWalk", dParam)) {
    graphMgr_.setAccBiasRandomWalk(dParam);
  }
  if (privateNode.getParam("accBiasPrior", dParam)) {
    graphMgr_.setAccBiasPrior(dParam);
  }
  if (privateNode.getParam("gyrNoiseDensity", dParam)) {
    graphMgr_.setGyroNoiseDensity(dParam);
  }
  if (privateNode.getParam("gyrBiasRandomWalk", dParam)) {
    graphMgr_.setGyrBiasRandomWalk(dParam);
  }
  if (privateNode.getParam("integrationNoiseDensity", dParam)) {
    graphMgr_.setIntegrationNoiseDensity(dParam);
  }
  if (privateNode.getParam("biasAccOmegaPreint", dParam)) {
    graphMgr_.setBiasAccOmegaPreint(dParam);
  }
  if (privateNode.getParam("gyrBiasPrior", dParam)) {
    graphMgr_.setGyrBiasPrior(Eigen::Vector3d(dParam, dParam, dParam));
  }
  if (privateNode.getParam("smootherLag", dParam)) {
    graphMgr_.setSmootherLag(dParam);
  }
  if (privateNode.getParam("additonalIterations", iParam)) {
    graphMgr_.setIterations(iParam);
  }
  if (privateNode.getParam("positionReLinTh", dParam)) {
    graphMgr_.setPositionReLinTh(dParam);
  }
  if (privateNode.getParam("rotationReLinTh", dParam)) {
    graphMgr_.setRotationReLinTh(dParam);
  }
  if (privateNode.getParam("velocityReLinTh", dParam)) {
    graphMgr_.setVelocityReLinTh(dParam);
  }
  if (privateNode.getParam("accBiasReLinTh", dParam)) {
    graphMgr_.setAccBiasReLinTh(dParam);
  }
  if (privateNode.getParam("gyrBiasReLinTh", dParam)) {
    graphMgr_.setGyrBiasReLinTh(dParam);
  }
  if (privateNode.getParam("relinearizeSkip", iParam)) {
    graphMgr_.getIsamParamsReference().setRelinearizeSkip(iParam);
  }
  if (privateNode.getParam("enableRelinearization", bParam)) {
    graphMgr_.getIsamParamsReference().setEnableRelinearization(bParam);
  }
  if (privateNode.getParam("evaluateNonlinearError", bParam)) {
    graphMgr_.getIsamParamsReference().setEvaluateNonlinearError(bParam);
  }
  if (privateNode.getParam("cacheLinearizedFactors", bParam)) {
    graphMgr_.getIsamParamsReference().setCacheLinearizedFactors(bParam);
  }
  if (privateNode.getParam("findUnusedFactorSlots", bParam)) {
    graphMgr_.getIsamParamsReference().findUnusedFactorSlots = bParam;
  }
  if (privateNode.getParam("enablePartialRelinearizationCheck", bParam)) {
    graphMgr_.getIsamParamsReference().setEnablePartialRelinearizationCheck(bParam);
  }
  if (privateNode.getParam("enableDetailedResults", bParam)) {
    graphMgr_.getIsamParamsReference().setEnableDetailedResults(bParam);
  }
  std::vector<double> poseBetweenNoise;  // roll,pitch,yaw,x,y,z
  if (privateNode.getParam("poseBetweenNoise", poseBetweenNoise)) {
    ROS_WARN_STREAM("Set pose noise to " << poseBetweenNoise[0] << "," << poseBetweenNoise[1] << "," << poseBetweenNoise[2] << ","
                                         << poseBetweenNoise[3] << "," << poseBetweenNoise[4] << "," << poseBetweenNoise[5]);
    graphMgr_.setPoseBetweenNoise(poseBetweenNoise);
  } else {
    std::runtime_error("poseBetweenNoise needs to be set in config file.");
  }
  double gnssUnaryNoise;
  if (privateNode.getParam("gnssUnaryNoise", gnssUnaryNoise)) {
    ROS_WARN_STREAM("Set pose noise to " << gnssUnaryNoise);
    graphMgr_.setGnssUnaryNoise(gnssUnaryNoise);
  } else {
    std::runtime_error("gnssUnaryNoise needs to be set in config file.");
  }

  // Verbosity
  iParam = 0;
  if (privateNode.getParam("Verbosity", iParam)) {
    ROS_INFO("Set fg_filtering-Verbosity: %d", iParam);
    setVerboseLevel(iParam);
    graphMgr_.setVerboseLevel(iParam);
  } else {
    setVerboseLevel(0);
    graphMgr_.setVerboseLevel(0);
  }

  // Publishers
  /// advertise odometry topic
  pubOdometry_ = privateNode.advertise<nav_msgs::Odometry>("/fg_filtering/transform_odom_base", 100);
  pubOdomPath_ = node.advertise<nav_msgs::Path>("/fg_filtering/odom_path", 100);
  pubOptimizationPath_ = node.advertise<nav_msgs::Path>("/fg_filtering/optimization_path", 100);
  pubCompslamPath_ = node.advertise<nav_msgs::Path>("/fg_filtering/compslam_path", 100);
  pubLaserImuBias_ = node.advertise<sensor_msgs::Imu>("/fg_filtering/imu_bias", 100);
  pubLeftGnssPath_ = node.advertise<nav_msgs::Path>("/fg_filtering/gnss_path_left", 100);
  pubRightGnssPath_ = node.advertise<nav_msgs::Path>("/fg_filtering/gnss_path_right", 100);
  excavatorStatePublisher_ = node.advertise<m545_msgs::M545State>("/m545_state", 100);
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
  subImu_ =
      node.subscribe<sensor_msgs::Imu>("/imu_topic", 100, &FactorGraphFiltering::imuCallback, this, ros::TransportHints().tcpNoDelay());
  ROS_INFO("Initialized IMU subscriber.");
  /// subscribe to remapped LiDAR odometry topic
  subLidarOdometry_ = node.subscribe<nav_msgs::Odometry>("/lidar_odometry_topic", 1000, &FactorGraphFiltering::lidarOdometryCallback, this,
                                                         ros::TransportHints().tcpNoDelay());
  ROS_INFO("Initialized LiDAR Odometry subscriber.");
  /// subscribe to gnss topics using ROS exact sync policy in a single callback
  subGnssLeft_.subscribe(node, "/gnss_topic_left", 100);
  subGnssRight_.subscribe(node, "/gnss_topic_right", 100);
  gnssExactSyncPtr_.reset(new message_filters::Synchronizer<_gnssExactSyncPolicy>(_gnssExactSyncPolicy(100), subGnssLeft_, subGnssRight_));
  gnssExactSyncPtr_->registerCallback(boost::bind(&FactorGraphFiltering::gnssCallback, this, _1, _2));
  ROS_INFO("Initialized GNSS subscriber (for both GNSS topics).");
  /// Subscribe to measurements
  subMeasurements_ = node.subscribe<m545_msgs::M545Measurements>("/measurement_topic", 100, &FactorGraphFiltering::measurementsCallback,
                                                                 this, ros::TransportHints().tcpNoDelay());
  ROS_INFO("Initialized Measurements subscriber.");

  /// Initialize helper threads
  optimizeGraphThread_ = std::thread(&FactorGraphFiltering::optimizeGraph, this);
  ROS_INFO("Initialized thread for optimizing the graph in parallel.");

  return true;
}

// ROS helper functions ---------------------------------------------------------------

// Estimation dependant functions --------------------------------------------------------------
void FactorGraphFiltering::alignImu(const ros::Time& imuTimeK) {
  gtsam::Rot3 imu_attitude;
  if (graphMgr_.estimateAttitudeFromImu(imuTimeK.toSec(), imuGravityDirection_, imu_attitude, gravityConstant_,
                                        graphMgr_.getInitGyrBiasReference())) {
    zeroYawImuAttitude_ = gtsam::Rot3::Ypr(0.0, imu_attitude.pitch(), imu_attitude.roll());  // IMU yaw to zero
    ROS_WARN_STREAM("\033[33mFG_FILTERING\033[0mAttitude of IMU is initialized. Determined Gravity Magnitude: " << gravityConstant_);
    imuAligned_ = true;
  } else {
    ROS_INFO_STREAM("\033[33mFG_FILTERING\033[0m NOT ENOUGH IMU MESSAGES TO INITIALIZE POSE. WAITNG FOR MORE...\n");
  }
}

void FactorGraphFiltering::initGNSS(const sensor_msgs::NavSatFix::ConstPtr& leftGnssPtr,
                                    const sensor_msgs::NavSatFix::ConstPtr& rightGnssPtr) {
  /// Left
  geodeticConverterLeft_.initialiseReference(leftGnssPtr->latitude, leftGnssPtr->longitude, leftGnssPtr->altitude);
  firstGnssCallback_ = false;
  if (geodeticConverterLeft_.isInitialised()) {
    ROS_INFO("Left GNSS position was initialized.");
  } else {
    ROS_ERROR("Left GNSS position could not be initialized.");
  }
  /// Right
  geodeticConverterRight_.initialiseReference(rightGnssPtr->latitude, rightGnssPtr->longitude, rightGnssPtr->altitude);
  firstGnssCallback_ = false;
  if (geodeticConverterRight_.isInitialised()) {
    ROS_INFO("Right GNSS position was initialized.");
  } else {
    ROS_ERROR("Right GNSS position could not be initialized.");
  }
}

// Graph initialization from starting attitude
void FactorGraphFiltering::initGraph(const ros::Time& timeStamp_k) {
  //  // Initial ros time
  //  initialTime_ = ros::Time(0);
  //  ROS_WARN_STREAM("Initial ros time is set to " << initialTime_.toSec() << ". Time of factor graph is relative to this time.");

  // Gravity
  graphMgr_.initImuIntegrators(gravityConstant_, imuGravityDirection_);
  // Initialize first node
  graphMgr_.initPoseVelocityBiasGraph(timeStamp_k.toSec(), gtsam::Pose3(zeroYawImuAttitude_, gtsam::Point3()));
  gtsam::Pose3 initialImuPose(graphMgr_.getGraphState().navState().pose().matrix());
  ROS_WARN_STREAM("INIT t(x,y,z): " << initialImuPose.translation().transpose()
                                    << ", RPY(deg): " << initialImuPose.rotation().rpy().transpose() * (180.0 / M_PI) << "\n");
  ROS_WARN_STREAM("Factor graph key of very first node: " << graphMgr_.getStateKey() << std::endl);
  // Write in tf member variable
  pose3ToTF(initialImuPose, tf_T_O_I0_);
  // Initialize global pose
  pose3ToTF(initialImuPose, tf_T_O_Ik_);
  tf_T_O_Ik_.frame_id_ = staticTransformsPtr_->getOdomFrame();
  tf_T_O_Ik_.stamp_ = timeStamp_k;
  // Set flag
  graphInited_ = true;
}

// Callbacks ------------------------------------------------------------------------
void FactorGraphFiltering::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_ptr) {
  // Set IMU time
  const ros::Time imuTimeKm1 = imuTimeKm1_;
  const ros::Time imuTimeK = imu_ptr->header.stamp;
  // Filter out imu messages with same time stamp
  if (imuTimeK == imuTimeKm1_) {
    ROS_WARN_STREAM("Imu time " << imuTimeK << " was repeated.");
    return;
  } else {
    imuTimeKm1_ = imuTimeK;
  }

  // Add to buffer
  graphMgr_.addToIMUBuffer(imuTimeK.toSec(), imu_ptr->linear_acceleration.x, imu_ptr->linear_acceleration.y, imu_ptr->linear_acceleration.z,
                           imu_ptr->angular_velocity.x, imu_ptr->angular_velocity.y, imu_ptr->angular_velocity.z);

  // If IMU not yet aligned
  if (!imuAligned_) {
    alignImu(imuTimeK);
  }  // Initialize graph at next iteration step
  else if (imuAligned_ && !graphInited_) {
    ROS_WARN("Initializing the graph...");
    initGraph(imuTimeK);
    ROS_WARN("...graph is initialized.");
  }
  // Add measurement to graph but don't optimize it
  else if (graphInited_) {
    // Add IMU factor and get propagated state
    gtsam::NavState T_O_Ik = graphMgr_.addImuFactorAndGetState(imuTimeK.toSec());
    // Write information to global variable
    pose3ToTF(T_O_Ik.pose(), tf_T_O_Ik_);
    tf_T_O_Ik_.frame_id_ = staticTransformsPtr_->getOdomFrame();
    tf_T_O_Ik_.stamp_ = imuTimeK;
    // Publish state
    if (graphOptimizedAtLeastOnce_) {
      // Publish
      publishState(T_O_Ik, imuTimeK);
      // Log
      signalLogger_.publishLogger(T_O_Ik.pose(), graphMgr_.getIMUBias());
    } else {
      // Add zero motion factor in the very beginning to make biases converge
      graphMgr_.addZeroMotionFactor(0.02, imuTimeKm1.toSec(), imuTimeK.toSec(), gtsam::Pose3::identity());
      // Publish zero motion state
      publishState(gtsam::NavState(gtsam::Pose3(zeroYawImuAttitude_, gtsam::Point3()), gtsam::Velocity3(0.0, 0.0, 0.0)), imuTimeK);
    }
  }
}

void FactorGraphFiltering::lidarOdometryCallback(const nav_msgs::Odometry::ConstPtr& odomLidarPtr) {
  // Output of compslam --> predicts absolute motion in lidar frame
  tf::Transform tf_T_O_Lk_Compslam, tf_T_O_Ck_Compslam;
  tf::StampedTransform tf_T_O_Ik_Compslam;
  odomMsgToTF(*odomLidarPtr, tf_T_O_Lk_Compslam);
  ros::Time compslamTimeKm1;

  // Transform message to Imu frame
  tf_T_O_Ik_Compslam.setData(tf_T_O_Lk_Compslam * staticTransformsPtr_->T_L_I());
  tf_T_O_Ik_Compslam.stamp_ = odomLidarPtr->header.stamp;

  // Set initial compslam pose after third callback (because first compslam pose is wrong)
  ++lidarCallbackCounter_;
  if (lidarCallbackCounter_ <= 3) {
    compslamTimeK_ = odomLidarPtr->header.stamp;
    tf_T_I0_O_Compslam_ = tf_T_O_Ik_Compslam.inverse();
    return;
  }
  // Set LiDAR time
  compslamTimeKm1 = compslamTimeK_;
  compslamTimeK_ = odomLidarPtr->header.stamp;

  if (graphInited_) {
    /// Delta pose
    Eigen::Matrix4d T_Ikm1_Ik = computeDeltaPose(tf_T_O_Ikm1_Compslam_, tf_T_O_Ik_Compslam);
    gtsam::Pose3 lidarDeltaPose(T_Ikm1_Ik);
    // Write the lidar odom delta to the graph
    graphMgr_.addPoseBetweenFactor(lidarDeltaPose, compslamTimeKm1.toSec(), compslamTimeK_.toSec());
    // Mutex for optimizeGraph Flag
    {
      // Lock
      const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
      optimizeGraph_ = true;
    }

    // Direct compslam estimate for base
    tf::Transform tf_T_I0_Ik_Compslam = tf_T_I0_O_Compslam_ * tf_T_O_Ik_Compslam;
    tf_T_O_Ck_Compslam = tf_T_O_I0_ * tf_T_I0_Ik_Compslam * staticTransformsPtr_->T_I_C();

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
  tf_T_O_Ikm1_Compslam_ = tf_T_O_Ik_Compslam;
}

void FactorGraphFiltering::gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& leftGnssPtr,
                                        const sensor_msgs::NavSatFix::ConstPtr& rightGnssPtr) {
  // First callback --> set position to zero
  if (firstGnssCallback_) {
    initGNSS(leftGnssPtr, rightGnssPtr);
    return;
  }
  /// Left
  std::unique_ptr<double> leftEastPtr = std::make_unique<double>();
  std::unique_ptr<double> leftNorthPtr = std::make_unique<double>();
  std::unique_ptr<double> leftUpPtr = std::make_unique<double>();
  geodeticConverterLeft_.geodetic2Enu(leftGnssPtr->latitude, leftGnssPtr->longitude, leftGnssPtr->altitude, leftEastPtr.get(),
                                      leftNorthPtr.get(), leftUpPtr.get());
  /// Right
  auto rightEastPtr = std::make_unique<double>();
  auto rightNorthPtr = std::make_unique<double>();
  auto rightUpPtr = std::make_unique<double>();
  geodeticConverterLeft_.geodetic2Enu(rightGnssPtr->latitude, rightGnssPtr->longitude, rightGnssPtr->altitude, rightEastPtr.get(),
                                      rightNorthPtr.get(), rightUpPtr.get());

  // Write to graph
  // Check whether covariance is okay, otherwise return
  bool covarianceViolated =
      leftGnssPtr->position_covariance[0] > 1.0 || leftGnssPtr->position_covariance[4] > 1.0 || leftGnssPtr->position_covariance[8] > 1.0;
  if (graphInited_ && !covarianceViolated) {
    // Translation in robot frame
    tf::Vector3 tf_W_t_W_GnssL(*leftEastPtr, *leftNorthPtr, *leftUpPtr);
    tf::Transform tf_T_GnssL_I = staticTransformsPtr_->T_GnssL_C() * staticTransformsPtr_->T_C_I();
    tf::Vector3 tf_GnssL_t_GnssL_I = tf_T_GnssL_I.getOrigin();
    // Global rotation
    tf::Transform tf_T_I_GnssL = staticTransformsPtr_->T_I_C() * staticTransformsPtr_->T_C_GnssL();
    tf::Quaternion tf_q_W_GnssL = (tf_T_O_Ik_ * tf_T_I_GnssL).getRotation();
    tf::Transform tf_R_W_GnssL = tf::Transform::getIdentity();
    tf_R_W_GnssL.setRotation(tf_q_W_GnssL);
    // Translation in global frame
    tf::Vector3 tf_W_t_GnssL_I = tf_R_W_GnssL * tf_GnssL_t_GnssL_I;

    // Shift observed GNSS position to IMU frame (instead of GNSS antenna)
    tf::Vector3 tf_W_t_W_I = tf_W_t_W_GnssL + tf_W_t_GnssL_I;

    // Modify graph
    gtsam::Vector3 W_t_W_I = {tf_W_t_W_I.x(), tf_W_t_W_I.y(), tf_W_t_W_I.z()};
    graphMgr_.addGnssUnaryFactor(leftGnssPtr->header.stamp.toSec(), W_t_W_I);
  } else if (covarianceViolated) {
    ROS_ERROR_STREAM("Covariance is too big, not using GNSS estimate.");
  }

  // Publish path
  /// Left
  //// Pose
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = staticTransformsPtr_->getOdomFrame();
  pose.header.stamp = leftGnssPtr->header.stamp;
  pose.pose.position.x = *leftEastPtr;   //+ tf_T_C_GL.getOrigin().x();
  pose.pose.position.y = *leftNorthPtr;  //+ tf_T_C_GL.getOrigin().y();
  pose.pose.position.z = *leftUpPtr;     //+ tf_T_C_GL.getOrigin().z();
  //// Path
  leftGnssPathPtr_->header.frame_id = staticTransformsPtr_->getOdomFrame();
  leftGnssPathPtr_->header.stamp = leftGnssPtr->header.stamp;
  leftGnssPathPtr_->poses.push_back(pose);
  pubLeftGnssPath_.publish(leftGnssPathPtr_);
  /// Right
  //// Pose
  pose.header.frame_id = staticTransformsPtr_->getOdomFrame();
  pose.header.stamp = rightGnssPtr->header.stamp;
  pose.pose.position.x = *rightEastPtr;   // + tf_T_C_GR.getOrigin().x();
  pose.pose.position.y = *rightNorthPtr;  // + tf_T_C_GR.getOrigin().y();
  pose.pose.position.z = *rightUpPtr;     // + tf_T_C_GR.getOrigin().z();
  //// Path
  rightGnssPathPtr_->header.frame_id = staticTransformsPtr_->getOdomFrame();
  rightGnssPathPtr_->header.stamp = rightGnssPtr->header.stamp;
  rightGnssPathPtr_->poses.push_back(pose);
  pubRightGnssPath_.publish(rightGnssPathPtr_);
}

void FactorGraphFiltering::measurementsCallback(const m545_msgs::M545Measurements::ConstPtr& measurementsMsgPtr) {
  // Converting measurement message to measurement
  measurements_ = measurementConverter_.convert(*measurementsMsgPtr);
}

// update graph ----------------------------------------------------------------------------------------
void FactorGraphFiltering::optimizeGraph() {
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
    bool optimizeGraph = false;
    // Mutex for optimizeGraph Flag
    {
      // Lock
      const std::lock_guard<std::mutex> optimizeGraphLock(optimizeGraphMutex_);
      if (optimizeGraph_) {
        optimizeGraph = optimizeGraph_;
        optimizeGraph_ = false;
      }
    }

    if (optimizeGraph) {
      // Get result
      startLoopTime = std::chrono::high_resolution_clock::now();
      gtsam::NavState optimizedNavState = graphMgr_.updateGraphAndState();
      endLoopTime = std::chrono::high_resolution_clock::now();
      if (!graphOptimizedAtLeastOnce_) {
        graphOptimizedAtLeastOnce_ = true;
      }
      if (verboseLevel_ > 1) {
        ROS_INFO_STREAM("\033[92mWhole optimization loop took "
                        << std::chrono::duration_cast<std::chrono::milliseconds>(endLoopTime - startLoopTime).count()
                        << " milliseconds.\033[0m");
      }
      // Transform pose
      gtsam::Pose3 T_O_I = optimizedNavState.pose();
      pose3ToTF(T_O_I, tf_T_O_I);
      tf_T_O_C = tf_T_O_I * staticTransformsPtr_->T_I_C();

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

void FactorGraphFiltering::publishState(const gtsam::NavState& T_O_Ik, ros::Time imuTimeK) {
  // Used transforms
  tf::Transform tf_T_O_C, tf_T_I_C, tf_T_C_B, tf_T_O_I;
  tf_T_I_C = staticTransformsPtr_->T_I_C();
  // tf_T_CB.setData(staticTransformsPtr_->T_CB());

  // Lookup chassis to cabin turn
  const double turnJointPosition = measurements_.actuatorStates_[m545_description::M545Topology::ActuatorEnum::TURN].position_;
  Eigen::AngleAxisd C_CB(-turnJointPosition, Eigen::Vector3d(0.0, 0.0, 1.0));
  pose3ToTF(C_CB.toRotationMatrix(), tf_T_C_B);
  tf_T_C_B.setOrigin(tf::Vector3(0.0, 0.0, -staticTransformsPtr_->BC_z_offset()));

  // From Eigen to TF
  gtsam::Pose3 T_O_I = T_O_Ik.pose();
  pose3ToTF(T_O_I, tf_T_O_I);
  // Transform
  // Only publish state if already some lidar constraints in graph
  tf_T_O_C = tf_T_O_I * tf_T_I_C;
  // Get odom-->base_link transformation from odom-->cabin
  tf::Transform tf_T_O_B = tf_T_O_C * tf_T_C_B;

  // m545_state
  excavator_model::ActuatorConversions::jointStateFromActuatorState(measurements_, estExcavatorState_);
  //_estExcavatorState.setAngularVelocityBaseInBaseFrame(...);
  estExcavatorState_.setLinearVelocityBaseInWorldFrame(kindr::Velocity3D(0.0, 0.0, 0.0));
  estExcavatorState_.setPositionWorldToBaseInWorldFrame(
      kindr::Position3D(tf_T_O_B.getOrigin().getX(), tf_T_O_B.getOrigin().getY(), tf_T_O_B.getOrigin().getZ()));
  estExcavatorState_.setOrientationBaseToWorld(kindr::RotationQuaternionPD(tf_T_O_B.getRotation().w(), tf_T_O_B.getRotation().x(),
                                                                           tf_T_O_B.getRotation().y(), tf_T_O_B.getRotation().z()));
  std::chrono::steady_clock::time_point chronoTimeK = std::chrono::steady_clock::time_point(chrono::nanoseconds(imuTimeK.toNSec()));
  estExcavatorState_.setTime(chronoTimeK);
  estExcavatorState_.setSequence(measurements_.sequence_);
  estExcavatorState_.setStatus(excavator_model::ExcavatorState::Status::STATUS_OK);
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
}  // end namespace fg_filtering
