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
    _imuBuffer.setImuRate(dParam);
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
  if (privateNode.getParam("relinearizeSkip", iParam)) _graphMgr._params.setRelinearizeSkip(iParam);
  if (privateNode.getParam("enableRelinearization", bParam)) _graphMgr._params.setEnableRelinearization(bParam);
  if (privateNode.getParam("evaluateNonlinearError", bParam)) _graphMgr._params.setEvaluateNonlinearError(bParam);
  if (privateNode.getParam("cacheLinearizedFactors", bParam)) _graphMgr._params.setCacheLinearizedFactors(bParam);
  if (privateNode.getParam("findUnusedFactorSlots", bParam)) _graphMgr._params.findUnusedFactorSlots = bParam;
  if (privateNode.getParam("enablePartialRelinearizationCheck", bParam))
    _graphMgr._params.setEnablePartialRelinearizationCheck(bParam);
  if (privateNode.getParam("enableDetailedResults", bParam)) _graphMgr._params.setEnableDetailedResults(bParam);
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
  _pubOdometry = privateNode.advertise<nav_msgs::Odometry>("/transform_odom_base", 5);
  _pubLaserImuBias = node.advertise<sensor_msgs::Imu>("/fg_imu_bias", 20);
  _pubOdomPath = node.advertise<nav_msgs::Path>("/odom_path", 100);
  _pubCompslamPath = node.advertise<nav_msgs::Path>("/compslam_path", 5);
  _pubLeftGnssPath = node.advertise<nav_msgs::Path>("/gnss_path_left", 20);
  _pubRightGnssPath = node.advertise<nav_msgs::Path>("/gnss_path_right", 20);
  /// Messages
  _odomPathPtr = nav_msgs::PathPtr(new nav_msgs::Path);
  _compslamPathPtr = nav_msgs::PathPtr(new nav_msgs::Path);
  _leftGnssPathPtr = nav_msgs::PathPtr(new nav_msgs::Path);
  _rightGnssPathPtr = nav_msgs::PathPtr(new nav_msgs::Path);

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
  _publishOdometryAndTFThread = std::thread(&FactorGraphFiltering::publishOdometryAndTF, this);
  _writeToGraphThread = std::thread(&FactorGraphFiltering::writeToGraph, this);

  return true;
}

void FactorGraphFiltering::alignImu() {
  gtsam::Rot3 imu_attitude;
  if (_imuBuffer.estimateAttitudeFromImu(_imuTime_k.toSec(), imu_attitude, _gravityConstant)) {
    _zeroYawIMUattitude = gtsam::Rot3::Ypr(0.0, imu_attitude.pitch(), imu_attitude.roll());  // IMU yaw to zero
    _imuAligned = true;
    ROS_WARN_STREAM("Attitude of IMU is initialized. Determined Gravity Magnitude: " << _gravityConstant);
  } else {
    std::cout << "\033[33mFG_FILTERING\033[0m NOT ENOUGH IMU MESSAGES TO INITIALIZE POSE.\n";
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
  _graphMgr.initImuIntegrator(_gravityConstant);
  // Set starting time and key to the first node
  _compslamTime_k = odomLidarPtr->header.stamp;
  _lidarKey_k = _graphMgr._state.key();
  _imuKey_k = _lidarKey_k;
  // Initialize first node
  _graphMgr.initPoseVelocityBiasGraph(_compslamTime_k.toSec(), gtsam::Pose3(_zeroYawIMUattitude, gtsam::Point3()));
  // Print Initialization
  ROS_WARN("Graph is initialized with first pose.");
  gtsam::Pose3 initialImuPose(_graphMgr._state.navState().pose().matrix());
  ROS_WARN_STREAM("INIT t(x,y,z): " << initialImuPose.translation().transpose() << ", RPY(deg): "
                                    << initialImuPose.rotation().rpy().transpose() * (180.0 / M_PI) << "\n");
  ROS_WARN_STREAM("Factor graph key of very first node: " << _lidarKey_k << std::endl);
  // Write in tf member variable
  pose3ToTF(initialImuPose, _tf_initialImuPose);
}

// imuCallback ----------------------------------------------------------------------------------------
void FactorGraphFiltering::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_ptr) {
  // Write to imu measurement and to buffer --> mutex
  std::lock_guard<std::mutex>* lockPtr = new std::lock_guard<std::mutex>(_imuMeasurementMutex);

  // Add to buffer
  _imuBuffer.addToIMUBuffer(imu_ptr->header.stamp.toSec(), imu_ptr->linear_acceleration.x,
                            imu_ptr->linear_acceleration.y, imu_ptr->linear_acceleration.z, imu_ptr->angular_velocity.x,
                            imu_ptr->angular_velocity.y, imu_ptr->angular_velocity.z);

  // Set IMU time
  _imuTime_k = imu_ptr->header.stamp;
  // If IMU not yet aligned
  if (!_imuAligned) {
    alignImu();
  }  // Else: set flag that new measurement arrived
  else {
    _newImuMeasurement = true;
  }
  // Release mutex
  delete lockPtr;
}

// lidarOdometryCallback ----------------------------------------------------------------------------------------
void FactorGraphFiltering::lidarOdometryCallback(const nav_msgs::Odometry::ConstPtr& odomLidarPtr) {
  // Output of compslam --> predicts absolute motion in lidar frame
  tf::Transform tf_T_OL_Compslam_k;
  odomMsgToTF(*odomLidarPtr, tf_T_OL_Compslam_k);

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

  // Transform message Imu frame
  tf::StampedTransform tf_T_OI_Compslam_k;
  tf_T_OI_Compslam_k.setData(tf_T_OL_Compslam_k * tf_T_LI);
  tf_T_OI_Compslam_k.stamp_ = odomLidarPtr->header.stamp;

  // Initialize compslam pose
  if (_firstLidarOdomCallback) {
    _tf_T_OI_init_inv = tf_T_OI_Compslam_k.inverse();
    _firstLidarOdomCallback = false;
  }

  // Compslam - Wait with graph manipulation until IMU attitude is determined
  if (_imuAligned) {
    // Initialize graph (if not happened already)
    if (!_graphInited) {
      initGraph(odomLidarPtr);
      _graphInited = true;
    }  // Wait one iteration
    else if (_firstScanCallback) {
      _firstScanCallback = false;
    }  // Else: Get Delta pose from Compslam
    else if (!_firstScanCallback) {
      /// Delta pose
      Eigen::Matrix4d I_T_km1_k = computeDeltaPose(_tf_T_OI_Compslam_km1, tf_T_OI_Compslam_k);
      // Write to delta pose --> mutex such that time and delta pose always correspond to each other
      std::lock_guard<std::mutex>* lockDeltaPosePtr = new std::lock_guard<std::mutex>(_lidarDeltaPoseMutex);
      _lidarDeltaPose = gtsam::Pose3(I_T_km1_k);
      // Set LiDAR time
      _compslamTime_km1 = _compslamTime_k;
      _compslamTime_k = odomLidarPtr->header.stamp;
      // Set Flag
      _newLidarDeltaPose = true;
      delete lockDeltaPosePtr;
      // Direct compslam estimate for base
      _tf_T_OC_Compslam.setData(tf_T_CI * _tf_initialImuPose * _tf_T_OI_init_inv * tf_T_OI_Compslam_k * tf_T_IC);
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
    //// Publish
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
    //// Publish
    _pubRightGnssPath.publish(_rightGnssPathPtr);
  }
}

// write to graph ----------------------------------------------------------------------------------------
void FactorGraphFiltering::writeToGraph() {
  int numLidarFactors = 0;
  std::chrono::time_point<std::chrono::system_clock> startLoopTime;
  std::chrono::time_point<std::chrono::system_clock> endLoopTime;
  ROS_INFO("Thread for adding factors to graph is ready.");
  while (ros::ok()) {
    startLoopTime = std::chrono::system_clock::now();
    // If graph not yet initialized
    if (!_graphInited) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    // Else, add measurements to graph
    else if (_newImuMeasurement) {
      // Always first add new Imu measurement
      // Write to delta pose --> mutex such that time and delta pose always correspond to each other
      std::lock_guard<std::mutex>* imuLockPtr = new std::lock_guard<std::mutex>(_imuMeasurementMutex);
      gtsam::Key imuKey_km1 = _imuKey_k;
      _imuKey_k = _graphMgr.newStateKey();

      // Write to key buffer
      _imuBuffer.addToKeyBuffer(_imuTime_k.toSec(), _imuKey_k);

      // Create IMU Map
      IMUMap imuMeas;
      bool success = _imuBuffer.getLastTwoMeasurements(imuMeas);
      if (success) {
        _graphMgr.addImuFactor(imuKey_km1, _imuKey_k, imuMeas);
      } else {
        ROS_WARN("FG Filtering - IMU factor not added to graph");
      }

      // If a LiDAR delta pose is there, add this as well
      if (_newLidarDeltaPose) {
        // Mutex to have reliable correspondences and always distinct values for last and current
        std::lock_guard<std::mutex>* lidarLockPtr = new std::lock_guard<std::mutex>(_lidarDeltaPoseMutex);

        // Find closest lidar key in existing graph
        IMUMapItr lidarMapItr;
        _imuBuffer.getClosestIMUBufferIteratorToTime(_compslamTime_k.toSec(), lidarMapItr);
        std::cout << "Found time stamp is: " << lidarMapItr->first << std::endl;
        gtsam::Key closestLidarKey;
        _imuBuffer.getCorrespondingGtsamKey(lidarMapItr->first, closestLidarKey);

        // Create new key
        gtsam::Key lidarKey_km1 = _lidarKey_k;
        _lidarKey_k = closestLidarKey;

        std::cout << "Last IMU key: " << imuKey_km1 << ", current IMU key: " << _imuKey_k << std::endl;
        std::cout << "Adding the " << ++numLidarFactors << "th LiDAR factor. "
                  << "Last LiDAR Key: " << lidarKey_km1 << ", current LiDAR Key: " << _lidarKey_k << std::endl;
        std::cout << "Delta Pose: " << _lidarDeltaPose << std::endl;

        // Write the lidar odom delta to the graph
        _graphMgr.addPoseBetweenFactor(lidarKey_km1, _lidarKey_k, _lidarDeltaPose);
        _newLidarDeltaPose = false;

        // Release mutex
        delete lidarLockPtr;
      }

      // Update graph
      _graphMgr.updateGraphAndState(_imuTime_k.toSec(), _imuKey_k);
      _newImuMeasurement = false;
      delete imuLockPtr;

      // Lookup transforms
      tf::StampedTransform tfT_CI, tfT_IC;
      _tfListener.lookupTransform(_cabinFrame, _imuFrame, _imuTime_k, tfT_CI);
      _tfListener.lookupTransform(_imuFrame, _cabinFrame, _imuTime_k, tfT_IC);

      // From Eigen to TF
      gtsam::Pose3 I_T_rel = _graphMgr._state.navState().pose();
      tf::Transform I_tfT_rel;
      pose3ToTF(I_T_rel, I_tfT_rel);
      tf::Transform tfT_OC;
      // Transform
      tfT_OC = tfT_CI * I_tfT_rel * tfT_IC;
      _tf_T_OC.setData(tfT_OC);
      _tf_T_OC.stamp_ = _imuTime_k;
    }
    endLoopTime = std::chrono::system_clock::now();
    // ROS_WARN_STREAM("FactorGraphUpdate took "
    //                << std::chrono::duration_cast<std::chrono::milliseconds>(endLoopTime - startLoopTime).count()
    //                << " milliseconds.");
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

// publish result ----------------------------------------------------------------------------------------
void FactorGraphFiltering::publishOdometryAndTF() {
  ROS_INFO("Thread for publishing the odometry is initialized.");
  // Timing
  std::chrono::time_point<std::chrono::system_clock> startLoopTime;
  std::chrono::time_point<std::chrono::system_clock> endLoopTime;
  // Time stamps
  ros::Time factorGraphStamp_k;
  ros::Time factorGraphStamp_km1;
  ros::Time compslamStamp_k;
  ros::Time compslamStamp_km1;
  // Variables
  tf::StampedTransform tf_T_CB;

  // Main loop of thread
  while (ros::ok()) {
    // Starting time of the iteration
    startLoopTime = std::chrono::system_clock::now();
    factorGraphStamp_k = _tf_T_OC.stamp_;
    compslamStamp_k = _compslamTime_k;

    // Check whether system is initialized --> set flag
    if (!_systemInited && _imuAligned && _graphInited) {
      _systemInited = true;
      ROS_WARN("System fully initialized, start publishing.");
      // Initialize timing variables
      factorGraphStamp_km1 = factorGraphStamp_k;
      compslamStamp_km1 = compslamStamp_k;
    }
    // Actual publishing
    else {
      if (_systemInited && factorGraphStamp_k != factorGraphStamp_km1) {
        // Get odom-->base_link transformation from odom-->imu
        _tfListener.lookupTransform(_cabinFrame, _baseLinkFrame, ros::Time(0), tf_T_CB);
        _tf_T_OB.setData(_tf_T_OC * tf_T_CB);
        _tf_T_OB.stamp_ = _tf_T_OC.stamp_;

        // Publish to Tf tree
        _tfBroadcaster.sendTransform(_tf_T_OB);

        // Publish odometry message for odom -> base frame
        nav_msgs::OdometryPtr odomBaseMsgPtr(new nav_msgs::Odometry);
        odomBaseMsgPtr->header.frame_id = _tf_T_OB.frame_id_;
        odomBaseMsgPtr->child_frame_id = _tf_T_OB.child_frame_id_;
        odomBaseMsgPtr->header.stamp = _tf_T_OB.stamp_;
        tf::poseTFToMsg(_tf_T_OB, odomBaseMsgPtr->pose.pose);
        _pubOdometry.publish(odomBaseMsgPtr);

        // Publish path for cabin frame
        /// Pose
        geometry_msgs::PoseStamped poseStamped;
        poseStamped.header.frame_id = _odomFrame;
        poseStamped.header.stamp = _tf_T_OC.stamp_;
        tf::poseTFToMsg(_tf_T_OC, poseStamped.pose);
        /// Path
        _odomPathPtr->header.frame_id = _odomFrame;
        _odomPathPtr->header.stamp = _tf_T_OC.stamp_;
        _odomPathPtr->poses.push_back(poseStamped);
        /// Publish
        _pubOdomPath.publish(_odomPathPtr);

        // IMU Bias
        if (_pubLaserImuBias.getNumSubscribers() > 0) {
          sensor_msgs::Imu imuBiasMsg;
          imuBiasMsg.header.frame_id = "/fg_odometry_imu";
          imuBiasMsg.header.stamp = _tf_T_OC.stamp_;
          imuBiasMsg.linear_acceleration.x = graphIMUBias().accelerometer()(0);
          imuBiasMsg.linear_acceleration.y = graphIMUBias().accelerometer()(1);
          imuBiasMsg.linear_acceleration.z = graphIMUBias().accelerometer()(2);
          imuBiasMsg.angular_velocity.x = graphIMUBias().gyroscope()(0);
          imuBiasMsg.angular_velocity.y = graphIMUBias().gyroscope()(1);
          imuBiasMsg.angular_velocity.z = graphIMUBias().gyroscope()(2);
          _pubLaserImuBias.publish(imuBiasMsg);
        }

        // Update time
        factorGraphStamp_km1 = factorGraphStamp_k;
      }
      // Visualization of Compslam pose
      if (_systemInited && compslamStamp_k != compslamStamp_km1) {
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
        // Update time
        compslamStamp_km1 = compslamStamp_k;
      }
    }

    // Time to exactly 100 Hz
    endLoopTime = std::chrono::system_clock::now();
    // ROS_WARN_STREAM("Odometry publishing took "
    //                 << std::chrono::duration_cast<std::chrono::milliseconds>(endLoopTime - startLoopTime).count()
    //                 << " milliseconds.");
    // std::this_thread::sleep_for(std::chrono::milliseconds(10) - (endLoopTime - startLoopTime));
  }
}

}  // end namespace fg_filtering
