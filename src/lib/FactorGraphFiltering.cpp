#include "fg_filtering/FactorGraphFiltering.h"

namespace fg_filtering {

using std::asin;
using std::atan2;
using std::cos;
using std::fabs;
using std::pow;
using std::sin;
using std::sqrt;

FactorGraphFiltering::FactorGraphFiltering(float scanPeriod) { ROS_INFO("FactorGraphFiltering instance created."); }

// setup ----------------------------------------------------------------------------------------
bool FactorGraphFiltering::setup(ros::NodeHandle& node, ros::NodeHandle& privateNode) {
  // Variables for parameter fetching
  double dParam;
  float fParam;
  int iParam;
  bool bParam;
  std::string sParam;

  // Set frames
  /// Odom
  if (privateNode.getParam("odomFrame", sParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - Odom frame set to: " << sParam);
    setOdomFrame(sParam);
    _tf_T_OC.frame_id_ = sParam;
    _tf_T_OB.frame_id_ = sParam;
  } else {
    ROS_WARN("FactorGraphFiltering - Odom frame not set");
  }
  /// base_link
  if (privateNode.getParam("baseLinkFrame", sParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - base_link frame: " << sParam);
    setBaseLinkFrame(sParam);
    _tf_T_OB.child_frame_id_ = sParam;
  } else
    ROS_WARN("FactorGraphFiltering - IMU frame not set for preintegrator");
  /// IMU
  if (privateNode.getParam("imuFrame", sParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - IMU frame for preintegrator and tf: " << sParam);
    setImuFrame(sParam);
  } else
    ROS_WARN("FactorGraphFiltering - IMU frame not set for preintegrator");
  /// LiDAR frame
  if (privateNode.getParam("lidarFrame", sParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - LiDAR frame: " << sParam);
    setLidarFrame(sParam);
  } else {
    ROS_WARN("FactorGraphFiltering - LiDAR frame not set");
  }
  /// Cabin frame
  if (privateNode.getParam("cabinFrame", sParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - cabin frame: " << sParam);
    setCabinFrame(sParam);
    _tf_T_OC.child_frame_id_ = sParam;
  } else {
    ROS_WARN("FactorGraphFiltering - cabin frame not set");
  }
  /// Left GNSS frame
  if (privateNode.getParam("leftGnssFrame", sParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - left GNSS frame: " << sParam);
    setLeftGnssFrame(sParam);
  } else {
    ROS_WARN("FactorGraphFiltering - left GNSS frame not set");
  }
  /// Right GNSS frame
  if (privateNode.getParam("rightGnssFrame", sParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - right GNSS frame: " << sParam);
    setRightGnssFrame(sParam);
  } else {
    ROS_WARN("FactorGraphFiltering - right GNSS frame not set");
  }

  // Timing
  if (privateNode.getParam("imuTimeOffset", dParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - IMU time offset[s]: " << dParam);
    setImuTimeOffset(dParam);
  }
  if (privateNode.getParam("imuRate", dParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - IMU rate for preintegrator: " << dParam);
    _graphMgr.setImuRate(dParam);
  }
  if (privateNode.getParam("ioRatio", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("Invalid ioRatio parameter: %d (expected > 0)", iParam);
      return false;
    } else {
      _ioRatio = iParam;
      ROS_INFO("Set ioRatio: %d", iParam);
    }
  }

  // Geodetic Converter
  _geodeticConverterLeft = geodetic_converter::GeodeticConverter();
  _geodeticConverterRight = geodetic_converter::GeodeticConverter();

  // Factor Graph
  if (privateNode.getParam("accNoiseDensity", dParam)) _graphMgr.setAccNoiseDensity(dParam);
  if (privateNode.getParam("accBiasRandomWalk", dParam)) _graphMgr.setAccBiasRandomWalk(dParam);
  if (privateNode.getParam("accBiasPrior", dParam)) _graphMgr.setAccBiasPrior(dParam);
  if (privateNode.getParam("gyrNoiseDensity", dParam)) _graphMgr.setGyroNoiseDensity(dParam);
  if (privateNode.getParam("gyrBiasRandomWalk", dParam)) _graphMgr.setGyrBiasRandomWalk(dParam);
  if (privateNode.getParam("gyrBiasPrior", dParam)) _graphMgr.setGyrBiasPrior(dParam);
  if (privateNode.getParam("smootherLag", dParam)) _graphMgr.setSmootherLag(dParam);
  if (privateNode.getParam("additonalIterations", iParam)) _graphMgr.setIterations(iParam);
  if (privateNode.getParam("positionReLinTh", dParam)) _graphMgr.setPositionReLinTh(dParam);
  if (privateNode.getParam("rotationReLinTh", dParam)) _graphMgr.setRotationReLinTh(dParam);
  if (privateNode.getParam("velocityReLinTh", dParam)) _graphMgr.setVelocityReLinTh(dParam);
  if (privateNode.getParam("accBiasReLinTh", dParam)) _graphMgr.setAccBiasReLinTh(dParam);
  if (privateNode.getParam("gyrBiasReLinTh", dParam)) _graphMgr.setGyrBiasReLinTh(dParam);
  if (privateNode.getParam("relinearizeSkip", iParam)) _graphMgr._isamParams.setRelinearizeSkip(iParam);
  if (privateNode.getParam("enableRelinearization", bParam)) _graphMgr._isamParams.setEnableRelinearization(bParam);
  if (privateNode.getParam("evaluateNonlinearError", bParam)) _graphMgr._isamParams.setEvaluateNonlinearError(bParam);
  if (privateNode.getParam("cacheLinearizedFactors", bParam)) _graphMgr._isamParams.setCacheLinearizedFactors(bParam);
  if (privateNode.getParam("findUnusedFactorSlots", bParam)) _graphMgr._isamParams.findUnusedFactorSlots = bParam;
  if (privateNode.getParam("enablePartialRelinearizationCheck", bParam))
    _graphMgr._isamParams.setEnablePartialRelinearizationCheck(bParam);
  if (privateNode.getParam("enableDetailedResults", bParam)) _graphMgr._isamParams.setEnableDetailedResults(bParam);
  std::vector<double> poseNoise{0, 0, 0, 0, 0, 0};  // roll,pitch,yaw,x,y,z
  if (privateNode.getParam("poseBetweenNoise", poseNoise)) {
    _graphMgr.setPoseNoise(poseNoise);
  }

  // Verbosity
  iParam = 0;
  if (privateNode.getParam("Verbosity", iParam)) {
    ROS_INFO("Set fg_filtering-Verbosity: %d", iParam);
    setVerboseLevel(iParam);
  } else
    setVerboseLevel(0);

  // Publishers
  /// advertise odometry topic
  _pubOdometry = privateNode.advertise<nav_msgs::Odometry>("/fg_filtering/transform_odom_base", 100);
  _pubLaserImuBias = node.advertise<sensor_msgs::Imu>("/fg_filtering/imu_bias", 100);
  _pubOdomPath = node.advertise<nav_msgs::Path>("/fg_filtering/odom_path", 100);
  _pubCompslamPath = node.advertise<nav_msgs::Path>("/fg_filtering/compslam_path", 5);
  _pubLeftGnssPath = node.advertise<nav_msgs::Path>("/fg_filtering/gnss_path_left", 20);
  _pubRightGnssPath = node.advertise<nav_msgs::Path>("/fg_filtering/gnss_path_right", 20);
  /// Messages
  _odomPathPtr = nav_msgs::PathPtr(new nav_msgs::Path);
  _compslamPathPtr = nav_msgs::PathPtr(new nav_msgs::Path);
  _leftGnssPathPtr = nav_msgs::PathPtr(new nav_msgs::Path);
  _rightGnssPathPtr = nav_msgs::PathPtr(new nav_msgs::Path);

  // Signal logger
  _signalLogger.setup(node);

  // Subscribers
  /// subscribe to remapped IMU topic
  _subImu = node.subscribe<sensor_msgs::Imu>("/imu_topic", 100, &FactorGraphFiltering::imuCallback, this,
                                             ros::TransportHints().tcpNoDelay());
  ROS_INFO("Initialized IMU subscriber.");
  /// subscribe to remapped LiDAR odometry topic
  _subLidarOdometry =
      node.subscribe<nav_msgs::Odometry>("/lidar_odometry_topic", 10, &FactorGraphFiltering::lidarOdometryCallback,
                                         this, ros::TransportHints().tcpNoDelay());
  ROS_INFO("Initialized LiDAR Odometry subscriber.");

  // subscribe to gnss topics using ROS exact sync policy in a single callback
  _subGnssLeft.subscribe(node, "/gnss_topic_left", 20);
  _subGnssRight.subscribe(node, "/gnss_topic_right", 20);
  _gnssExactSyncPtr.reset(
      new message_filters::Synchronizer<_gnssExactSyncPolicy>(_gnssExactSyncPolicy(20), _subGnssLeft, _subGnssRight));
  _gnssExactSyncPtr->registerCallback(boost::bind(&FactorGraphFiltering::gnssCallback, this, _1, _2));

  // Initialize helper threads
  _updateGraphThread = std::thread(&FactorGraphFiltering::updateGraph, this);

  return true;
}

void FactorGraphFiltering::alignImu(const double imuTime_k) {
  gtsam::Rot3 imu_attitude;
  if (_graphMgr.estimateAttitudeFromImu(imuTime_k, imu_attitude, _gravityConstant)) {
    _zeroYawIMUattitude = gtsam::Rot3::Ypr(0.0, imu_attitude.pitch(), imu_attitude.roll());  // IMU yaw to zero
    _imuAligned = true;
    ROS_WARN_STREAM("\033[33mFG_FILTERING\033[0mAttitude of IMU is initialized. Determined Gravity Magnitude: "
                    << _gravityConstant);
  } else {
    ROS_INFO_STREAM("\033[33mFG_FILTERING\033[0m NOT ENOUGH IMU MESSAGES TO INITIALIZE POSE. WAITNG FOR MORE...\n");
  }
}

void FactorGraphFiltering::initGNSS(const sensor_msgs::NavSatFix::ConstPtr& leftGnssPtr,
                                    const sensor_msgs::NavSatFix::ConstPtr& rightGnssPtr) {
  /// Left
  _geodeticConverterLeft.initialiseReference(leftGnssPtr->latitude, leftGnssPtr->longitude, leftGnssPtr->altitude);
  _firstGnssCallback = false;
  if (_geodeticConverterLeft.isInitialised()) {
    ROS_INFO("Left GNSS position was initialized.");
  } else {
    ROS_ERROR("Left GNSS position could not be initialized.");
  }
  /// Right
  _geodeticConverterRight.initialiseReference(rightGnssPtr->latitude, rightGnssPtr->longitude, rightGnssPtr->altitude);
  _firstGnssCallback = false;
  if (_geodeticConverterRight.isInitialised()) {
    ROS_INFO("Right GNSS position was initialized.");
  } else {
    ROS_ERROR("Right GNSS position could not be initialized.");
  }
}

// Graph initialization from starting attitude
void FactorGraphFiltering::initGraph(const nav_msgs::Odometry::ConstPtr& odomLidarPtr) {
  // Gravity
  _graphMgr.initImuIntegrators(_gravityConstant);
  // Set starting time and key to the first node
  _compslamTime_k = odomLidarPtr->header.stamp;
  // Initialize first node
  _graphMgr.initPoseVelocityBiasGraph(_compslamTime_k.toSec(), gtsam::Pose3(_zeroYawIMUattitude, gtsam::Point3()));
  // Print Initialization
  ROS_WARN("Graph is initialized with first pose.");
  gtsam::Pose3 initialImuPose(_graphMgr.getGraphState().navState().pose().matrix());
  ROS_WARN_STREAM("INIT t(x,y,z): " << initialImuPose.translation().transpose() << ", RPY(deg): "
                                    << initialImuPose.rotation().rpy().transpose() * (180.0 / M_PI) << "\n");
  ROS_WARN_STREAM("Factor graph key of very first node: " << _graphMgr.getStateKey() << std::endl);
  // Write in tf member variable
  pose3ToTF(initialImuPose, _tf_initialImuPose);
}

// imuCallback ----------------------------------------------------------------------------------------
void FactorGraphFiltering::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_ptr) {
  // Set IMU time
  ros::Time imuTime_k = imu_ptr->header.stamp;

  // Filter out imu messages with same time stamp
  if (imuTime_k == _imuTime_km1) {
    ROS_WARN_STREAM("Imu time " << imuTime_k << " was repeated.");
    return;
  }
  else {
    _imuTime_km1 = imuTime_k;
  }

  // Add to buffer
  _graphMgr.addToIMUBuffer(imuTime_k.toSec(), imu_ptr->linear_acceleration.x, imu_ptr->linear_acceleration.y,
                           imu_ptr->linear_acceleration.z, imu_ptr->angular_velocity.x, imu_ptr->angular_velocity.y,
                           imu_ptr->angular_velocity.z);

  // If IMU not yet aligned
  if (!_imuAligned) {
    alignImu(imuTime_k.toSec());
  }  // Add measurement to graph but don't optimize it
  else if (_graphInited) {
    // Add IMU factor and get propagated state
    gtsam::NavState currentState = _graphMgr.addImuFactorAndGetState(imuTime_k.toSec());
    // Publish current state at imu frequency
    publishState(currentState, imuTime_k);
    _signalLogger.publishLogger(currentState.pose());
  }
}

// lidarOdometryCallback ----------------------------------------------------------------------------------------
void FactorGraphFiltering::lidarOdometryCallback(const nav_msgs::Odometry::ConstPtr& odomLidarPtr) {
  // Output of compslam --> predicts absolute motion in lidar frame
  tf::Transform tf_T_OL_Compslam_k;
  odomMsgToTF(*odomLidarPtr, tf_T_OL_Compslam_k);
  ros::Time compslamTime_km1;

  // Lookup transformations
  tf::StampedTransform tf_T_LC;
  _tfListener.lookupTransform(_lidarFrame, _cabinFrame, ros::Time(0), tf_T_LC);
  tf::StampedTransform tf_T_CL;
  _tfListener.lookupTransform(_lidarFrame, _cabinFrame, ros::Time(0), tf_T_CL);
  tf::StampedTransform tf_T_LI;
  _tfListener.lookupTransform(_lidarFrame, _imuFrame, ros::Time(0), tf_T_LI);
  tf::StampedTransform tf_T_IC;
  _tfListener.lookupTransform(_imuFrame, _cabinFrame, ros::Time(0), tf_T_IC);
  tf::StampedTransform tf_T_CI;
  _tfListener.lookupTransform(_cabinFrame, _imuFrame, ros::Time(0), tf_T_CI);

  // Transform message to Imu frame
  tf::StampedTransform tf_T_OI_Compslam_k;
  tf_T_OI_Compslam_k.setData(tf_T_OL_Compslam_k * tf_T_LI);
  tf_T_OI_Compslam_k.stamp_ = odomLidarPtr->header.stamp;

  // Set initial compslam pose
  if (_firstLidarOdomCallback) {
    _tf_T_OI_init_inv = tf_T_OI_Compslam_k.inverse();
    _firstLidarOdomCallback = false;
  }

  // Compslam - Wait with writing to graph until IMU attitude is determined
  if (_imuAligned) {
    // Initialize graph (if not happened already)
    if (!_graphInited) {
      initGraph(odomLidarPtr);
      _graphInited = true;
    }  // Wait one iteration to be able to compute delta pose
    if (_firstScanCallback) {
      _firstScanCallback = false;
    }  // Else: Get Delta pose from Compslam
    else {
      // Set LiDAR time
      compslamTime_km1 = _compslamTime_k;
      _compslamTime_k = odomLidarPtr->header.stamp;
      /// Delta pose
      Eigen::Matrix4d I_T_km1_k = computeDeltaPose(_tf_T_OI_Compslam_km1, tf_T_OI_Compslam_k);
      // Write to delta pose --> mutex such that time and delta pose always correspond to each other
      gtsam::Pose3 lidarDeltaPose(I_T_km1_k);
      // Write the lidar odom delta to the graph
      _graphMgr.addPoseBetweenFactor(lidarDeltaPose, compslamTime_km1.toSec(), _compslamTime_k.toSec());
      // Mutex for optimizeGraph Flag
      {
        // Lock
        const std::lock_guard<std::mutex> optimizeGraphLock(_optimizeGraphMutex);
        _optimizeGraph = true;
      }

      // Direct compslam estimate for base
      _tf_T_OC_Compslam.setData(tf_T_CI * _tf_initialImuPose * _tf_T_OI_init_inv * tf_T_OI_Compslam_k * tf_T_IC);
      _tf_T_OC_Compslam.stamp_ = odomLidarPtr->header.stamp;

      // Visualization of Compslam pose
      geometry_msgs::PoseStamped poseStamped;
      poseStamped.header.frame_id = _odomFrame;
      poseStamped.header.stamp = _tf_T_OC_Compslam.stamp_;
      tf::poseTFToMsg(_tf_T_OC_Compslam, poseStamped.pose);
      /// Path
      _compslamPathPtr->header.frame_id = _odomFrame;
      _compslamPathPtr->header.stamp = _tf_T_OC_Compslam.stamp_;
      _compslamPathPtr->poses.push_back(poseStamped);
      /// Publish
      _pubCompslamPath.publish(_compslamPathPtr);
    }
    // Set last pose for the next iteration
    _tf_T_OI_Compslam_km1 = tf_T_OI_Compslam_k;
  }
}

void FactorGraphFiltering::gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& leftGnssPtr,
                                        const sensor_msgs::NavSatFix::ConstPtr& rightGnssPtr) {
  // First callback --> set position to zero
  if (_firstGnssCallback) {
    initGNSS(leftGnssPtr, rightGnssPtr);

    // Later callbacks
  } else {
    /// Left
    auto leftEastPtr = std::make_unique<double>();
    auto leftNorthPtr = std::make_unique<double>();
    auto leftUpPtr = std::make_unique<double>();
    _geodeticConverterLeft.geodetic2Enu(leftGnssPtr->latitude, leftGnssPtr->longitude, leftGnssPtr->altitude,
                                        leftEastPtr.get(), leftNorthPtr.get(), leftUpPtr.get());
    /// Right
    auto rightEastPtr = std::make_unique<double>();
    auto rightNorthPtr = std::make_unique<double>();
    auto rightUpPtr = std::make_unique<double>();
    _geodeticConverterRight.geodetic2Enu(leftGnssPtr->latitude, leftGnssPtr->longitude, leftGnssPtr->altitude,
                                         rightEastPtr.get(), rightNorthPtr.get(), rightUpPtr.get());
    // Publish path
    /// Left
    //// Pose
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = _odomFrame;
    pose.header.stamp = leftGnssPtr->header.stamp;
    pose.pose.position.x = *leftEastPtr;   //+ tf_T_C_GL.getOrigin().x();
    pose.pose.position.y = *leftNorthPtr;  //+ tf_T_C_GL.getOrigin().y();
    pose.pose.position.z = *leftUpPtr;     //+ tf_T_C_GL.getOrigin().z();
    //// Path
    _leftGnssPathPtr->header.frame_id = _odomFrame;
    _leftGnssPathPtr->header.stamp = leftGnssPtr->header.stamp;
    _leftGnssPathPtr->poses.push_back(pose);
    _pubLeftGnssPath.publish(_leftGnssPathPtr);
    /// Right
    //// Pose
    pose.header.frame_id = _odomFrame;
    pose.header.stamp = rightGnssPtr->header.stamp;
    pose.pose.position.x = *rightEastPtr;   // + tf_T_C_GR.getOrigin().x();
    pose.pose.position.y = *rightNorthPtr;  // + tf_T_C_GR.getOrigin().y();
    pose.pose.position.z = *rightUpPtr;     // + tf_T_C_GR.getOrigin().z();
    //// Path
    _rightGnssPathPtr->header.frame_id = _odomFrame;
    _rightGnssPathPtr->header.stamp = rightGnssPtr->header.stamp;
    _rightGnssPathPtr->poses.push_back(pose);
    _pubRightGnssPath.publish(_rightGnssPathPtr);
  }
}

// update graph ----------------------------------------------------------------------------------------
void FactorGraphFiltering::updateGraph() {
  // Preallocation
  int numLidarFactors = 0;
  std::chrono::time_point<std::chrono::high_resolution_clock> startLoopTime;
  std::chrono::time_point<std::chrono::high_resolution_clock> endLoopTime;
  double imuTime_k;
  tf::StampedTransform tf_T_CB;
  ros::Rate loopRate(100);

  ROS_INFO("Thread for updating graph is ready.");
  while (ros::ok()) {
    bool optimizeGraph = false;
    // Mutex for optimizeGraph Flag
    {
      // Lock
      const std::lock_guard<std::mutex> optimizeGraphLock(_optimizeGraphMutex);
      if (_optimizeGraph) {
        optimizeGraph = _optimizeGraph;
        _optimizeGraph = false;
      }
    }

    if (optimizeGraph) {
      startLoopTime = std::chrono::high_resolution_clock::now();
      _graphMgr.updateGraphAndState();
      endLoopTime = std::chrono::high_resolution_clock::now();
      ROS_WARN_STREAM("Iteration for writing into graph took "
                      << std::chrono::duration_cast<std::chrono::milliseconds>(endLoopTime - startLoopTime).count()
                      << " milliseconds.");
      // Publish IMU Bias after update of graph
      sensor_msgs::Imu imuBiasMsg;
      imuBiasMsg.header.frame_id = "/fg_odometry_imu";
      imuBiasMsg.header.stamp = _tf_T_OC.stamp_;
      imuBiasMsg.linear_acceleration.x = _graphMgr.getIMUBias().accelerometer()(0);
      imuBiasMsg.linear_acceleration.y = _graphMgr.getIMUBias().accelerometer()(1);
      imuBiasMsg.linear_acceleration.z = _graphMgr.getIMUBias().accelerometer()(2);
      imuBiasMsg.angular_velocity.x = _graphMgr.getIMUBias().gyroscope()(0);
      imuBiasMsg.angular_velocity.y = _graphMgr.getIMUBias().gyroscope()(1);
      imuBiasMsg.angular_velocity.z = _graphMgr.getIMUBias().gyroscope()(2);
      _pubLaserImuBias.publish(imuBiasMsg);
    }  // else just sleep for a short amount of time before polling again
    else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

void FactorGraphFiltering::publishState(gtsam::NavState currentState, ros::Time imuTime_k) {
  // Lookup transforms
  tf::StampedTransform tf_T_CI, tf_T_IC, tf_T_CB;
  _tfListener.lookupTransform(_cabinFrame, _imuFrame, imuTime_k, tf_T_CI);
  _tfListener.lookupTransform(_imuFrame, _cabinFrame, imuTime_k, tf_T_IC);

  // From Eigen to TF
  gtsam::Pose3 I_T_rel = currentState.pose();
  tf::Transform I_tf_T_rel;
  pose3ToTF(I_T_rel, I_tf_T_rel);
  tf::Transform tf_T_OC;
  // Transform
  tf_T_OC = tf_T_CI * I_tf_T_rel * tf_T_IC;
  _tf_T_OC.setData(tf_T_OC);
  _tf_T_OC.stamp_ = imuTime_k;
  // Get odom-->base_link transformation from odom-->imu
  _tfListener.lookupTransform(_cabinFrame, _baseLinkFrame, ros::Time(0), tf_T_CB);  // TODO
  _tf_T_OB.setData(_tf_T_OC * tf_T_CB);
  _tf_T_OB.stamp_ = imuTime_k;

  // Publish odometry message for odom->base with 100 Hz
  nav_msgs::OdometryPtr odomBaseMsgPtr(new nav_msgs::Odometry);
  odomBaseMsgPtr->header.frame_id = _tf_T_OB.frame_id_;
  odomBaseMsgPtr->child_frame_id = _tf_T_OB.child_frame_id_;
  odomBaseMsgPtr->header.stamp = imuTime_k;
  tf::poseTFToMsg(_tf_T_OB, odomBaseMsgPtr->pose.pose);
  _pubOdometry.publish(odomBaseMsgPtr);
  // Publish to Tf tree
  _tfBroadcaster.sendTransform(_tf_T_OB);
  // Publish path for cabin frame
  /// Pose
  geometry_msgs::PoseStamped poseStamped;
  poseStamped.header.frame_id = _odomFrame;
  poseStamped.header.stamp = imuTime_k;
  tf::poseTFToMsg(_tf_T_OC, poseStamped.pose);
  /// Path
  _odomPathPtr->header.frame_id = _odomFrame;
  _odomPathPtr->header.stamp = imuTime_k;
  _odomPathPtr->poses.push_back(poseStamped);
  /// Publish
  _pubOdomPath.publish(_odomPathPtr);
}

}  // end namespace fg_filtering
