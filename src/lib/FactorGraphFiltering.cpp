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

  // Motion parameters
  /// Zero Motion Factor
  if (privateNode.getParam("zeroMotionDetection", bParam)) {
    setZeroMotionDetection(bParam);
    if (bParam)
      ROS_INFO_STREAM(
          "FactorGraphFiltering - Zero Motion Detection ENABLED - Zero Velocity and Zero Delta Pose Factors will be "
          "added");
    else
      ROS_INFO_STREAM("FactorGraphFiltering - Zero Motion Detection DISABLED");
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

// imuCallback ----------------------------------------------------------------------------------------
void FactorGraphFiltering::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_ptr) {
  // Write to imu measurement and to buffer --> mutex
  std::lock_guard<std::mutex>* lockPtr = new std::lock_guard<std::mutex>(_imuMeasurementMutex);

  // Add to buffer
  _imuBuffer.addToIMUBuffer(imu_ptr->header.stamp.toSec(), imu_ptr->linear_acceleration.x,
                            imu_ptr->linear_acceleration.y, imu_ptr->linear_acceleration.z, imu_ptr->angular_velocity.x,
                            imu_ptr->angular_velocity.y, imu_ptr->angular_velocity.z);

  // Set IMU time
  _currentImuTime = imu_ptr->header.stamp;
  if (_graphInited) {
    // Set flag
    _newImuMeasurement = true;
  }
  // Release mutex
  delete lockPtr;
}

// lidarOdometryCallback ----------------------------------------------------------------------------------------
void FactorGraphFiltering::lidarOdometryCallback(const nav_msgs::Odometry::ConstPtr& odomLidarPtr) {
  // Get Message
  tf::Quaternion tf_q_OL;
  tf::quaternionMsgToTF(odomLidarPtr->pose.pose.orientation, tf_q_OL);
  tf::Vector3 tf_t_OL = tf::Vector3(odomLidarPtr->pose.pose.position.x, odomLidarPtr->pose.pose.position.y,
                                    odomLidarPtr->pose.pose.position.z);
  // Output of compslam --> translate up s.t. cabin is in center
  tf::StampedTransform tf_T_LC;
  _tfListener.lookupTransform(_lidarFrame, _cabinFrame, ros::Time(0), tf_T_LC);
  tf::StampedTransform tf_T_CL;
  _tfListener.lookupTransform(_lidarFrame, _cabinFrame, ros::Time(0), tf_T_CL);
  tf::Transform tf_T_OL_Compslam;
  tf_T_OL_Compslam.setRotation(tf_q_OL);
  tf_T_OL_Compslam.setOrigin(tf_t_OL);
  // tf_T_OL_Compslam = tf_T_CL * tf_T_OL_Compslam * tf_T_LC;

  // Transform message Imu frame
  tf::StampedTransform tf_T_LI;
  _tfListener.lookupTransform(_lidarFrame, _imuFrame, ros::Time(0), tf_T_LI);
  tf::StampedTransform tf_T_OI_Compslam;
  tf_T_OI_Compslam.setData(tf_T_OL_Compslam * tf_T_LI);
  tf_T_OI_Compslam.stamp_ = odomLidarPtr->header.stamp;

  // Compute delta pose
  /// Wait with graph manipulation until IMU attitude is determined
  if (_imuAligned) {
    // Initialize graph (if not happened already)
    if (!_graphInited) {
      // Set starting time and key to the first node
      _currentCompslamTime = odomLidarPtr->header.stamp;
      _currentLidarKey = _graphMgr._state.key();
      _currentImuKey = _currentLidarKey;

      // Initialize first node
      _graphMgr.initPoseVelocityBiasGraph(_currentCompslamTime.toSec(),
                                          gtsam::Pose3(_zeroYawIMUattitude, gtsam::Point3()));

      // Print Initialization
      ROS_WARN("Graph is initialized with first pose.");
      gtsam::Pose3 imuPose(_graphMgr._state.navState().pose().matrix());
      std::cout << "INIT t(x,y,z): " << imuPose.translation().transpose()
                << ", RPY(deg): " << imuPose.rotation().rpy().transpose() * (180.0 / M_PI) << "\n";
      std::cout << "Factor graph key of very first node: " << _currentLidarKey << std::endl;

      // Set flag
      _graphInited = true;
    }
    // Add delta pose for Odom->Imu to graph
    else if (!_firstScanCallback) {
      // Determine delta pose from compslam
      Eigen::Matrix4d T_OI_last = Eigen::MatrixXd::Identity(4, 4);
      Eigen::Matrix4d T_OI_current = Eigen::MatrixXd::Identity(4, 4);
      Eigen::Matrix4d T_OI_last_inv = Eigen::MatrixXd::Identity(4, 4);
      Eigen::Matrix4d I_T_rel = Eigen::MatrixXd::Identity(4, 4);
      T_OI_last.block<4, 1>(0, 3) =
          Eigen::Vector4d(_tf_T_OI_CompslamLast.getOrigin().x(), _tf_T_OI_CompslamLast.getOrigin().y(),
                          _tf_T_OI_CompslamLast.getOrigin().z(), 1.0);
      T_OI_last.block<3, 3>(0, 0) =
          Eigen::Quaterniond(_tf_T_OI_CompslamLast.getRotation().w(), _tf_T_OI_CompslamLast.getRotation().x(),
                             _tf_T_OI_CompslamLast.getRotation().y(), _tf_T_OI_CompslamLast.getRotation().z())
              .toRotationMatrix();
      T_OI_current.block<4, 1>(0, 3) = Eigen::Vector4d(
          tf_T_OI_Compslam.getOrigin().x(), tf_T_OI_Compslam.getOrigin().y(), tf_T_OI_Compslam.getOrigin().z(), 1.0);
      T_OI_current.block<3, 3>(0, 0) =
          Eigen::Quaterniond(tf_T_OI_Compslam.getRotation().w(), tf_T_OI_Compslam.getRotation().x(),
                             tf_T_OI_Compslam.getRotation().y(), tf_T_OI_Compslam.getRotation().z())
              .toRotationMatrix();
      invertHomogenousMatrix(T_OI_last, T_OI_last_inv);
      I_T_rel = T_OI_last_inv * T_OI_current;

      // Write to delta pose --> mutex such that time and delta pose always correspond to each other
      std::lock_guard<std::mutex>* lockDeltaPosePtr = new std::lock_guard<std::mutex>(_lidarDeltaPoseMutex);
      // Set to new delta pose
      _lidarDeltaPose = gtsam::Pose3(I_T_rel);
      // Set LiDAR time
      _lastCompslamTime = _currentCompslamTime;
      _currentCompslamTime = odomLidarPtr->header.stamp;
      // Set Flag
      _newLidarDeltaPose = true;
      delete lockDeltaPosePtr;

      // Preliminary until output comes from graph
      tf::StampedTransform tf_T_IC;
      _tfListener.lookupTransform(_imuFrame, _cabinFrame, ros::Time(0), tf_T_IC);
      _tf_T_OC_Compslam.setData(tf_T_OI_Compslam * tf_T_IC);
    }
  }
  if (_firstScanCallback) {
    _firstScanCallback = false;
  }

  // Set last pose for the next iteration
  _tf_T_OI_CompslamLast = tf_T_OI_Compslam;
}

void FactorGraphFiltering::gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& leftGnssPtr,
                                        const sensor_msgs::NavSatFix::ConstPtr& rightGnssPtr) {
  // First callback --> set position to zero
  if (_firstGnssCallback) {
    /// Left
    _geodeticConverterLeft.initialiseReference(leftGnssPtr->latitude, leftGnssPtr->longitude, leftGnssPtr->altitude);
    _firstGnssCallback = false;
    if (_geodeticConverterLeft.isInitialised()) {
      ROS_INFO("Left GNSS position was initialized.");
    } else {
      ROS_ERROR("Left GNSS position could not be initialized.");
    }
    /// Right
    _geodeticConverterRight.initialiseReference(rightGnssPtr->latitude, rightGnssPtr->longitude,
                                                rightGnssPtr->altitude);
    _firstGnssCallback = false;
    if (_geodeticConverterRight.isInitialised()) {
      ROS_INFO("Right GNSS position was initialized.");
    } else {
      ROS_ERROR("Right GNSS position could not be initialized.");
    }
    // Later callbacks
  } else {
    /// Left
    auto leftEastPtr = std::make_unique<double>();
    auto leftNorthPtr = std::make_unique<double>();
    auto leftUpPtr = std::make_unique<double>();
    _geodeticConverterLeft.geodetic2Enu(leftGnssPtr->latitude, leftGnssPtr->longitude, leftGnssPtr->altitude,
                                        leftEastPtr.get(), leftNorthPtr.get(), leftUpPtr.get());
    // tf::StampedTransform tf_T_C_GL;
    //_tfListener.lookupTransform(_cabinFrame, _leftGnssFrame, ros::Time(0), tf_T_C_GL);
    /// Right
    auto rightEastPtr = std::make_unique<double>();
    auto rightNorthPtr = std::make_unique<double>();
    auto rightUpPtr = std::make_unique<double>();
    _geodeticConverterRight.geodetic2Enu(leftGnssPtr->latitude, leftGnssPtr->longitude, leftGnssPtr->altitude,
                                         rightEastPtr.get(), rightNorthPtr.get(), rightUpPtr.get());
    // tf::StampedTransform tf_T_C_GR;
    //_tfListener.lookupTransform(_cabinFrame, _rightGnssFrame, ros::Time(0), tf_T_C_GR);

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
  ROS_INFO("Thread for adding IMU factors is initialized.");
  while (ros::ok()) {
    startLoopTime = std::chrono::system_clock::now();
    // If imu not yet aligned, try to align it until this works
    if (!_imuAligned) {
      double g;
      gtsam::Rot3 imu_attitude;
      if (_imuBuffer.estimateAttitudeFromImu(_currentImuTime.toSec(), imu_attitude, g)) {
        // Update graph
        _graphMgr.initImuIntegrator(g);
        _zeroYawIMUattitude = gtsam::Rot3::Ypr(0.0, imu_attitude.pitch(), imu_attitude.roll());  // Set IMU yaw to
                                                                                                 // zero
        // IMU initialized
        _imuAligned = true;
        ROS_WARN_STREAM("Attitude of IMU is initialized. Determined Gravity Magnitude: " << g);
        std::cout << "IMU attitude: " << _zeroYawIMUattitude << std::endl;
      }
      // Wait a bit for topics to be published
      else {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      }
    }
    // Else, add measurements to graph
    else if (_newImuMeasurement && _graphInited) {
      // Always first add new Imu measurement
      // Write to delta pose --> mutex such that time and delta pose always correspond to each other
      std::lock_guard<std::mutex>* imuLockPtr = new std::lock_guard<std::mutex>(_imuMeasurementMutex);
      gtsam::Key lastImuKey = _currentImuKey;
      _currentImuKey = _graphMgr.newStateKey();

      // Write to key buffer
      _imuBuffer.addToKeyBuffer(_currentImuTime.toSec(), _currentImuKey);

      // Create IMU Map
      IMUMap imuMeas;
      bool success = _imuBuffer.getLastTwoMeasurements(imuMeas);
      // std::cout << "Map: ";
      // print_map(imuMeas);
      // std::cout << "dT between IMU measurements: " << (--imuMeas.end())->first - (--(--imuMeas.end()))->first
      //          << std::endl;
      if (success) {
        _graphMgr.addImuFactor(lastImuKey, _currentImuKey, imuMeas);
      } else {
        ROS_WARN("FG Filtering - IMU factor not added to graph");
      }

      // If a LiDAR delta pose is there, add this as well
      if (_newLidarDeltaPose) {
        // Mutex to have reliable correspondences and always distinct values for last and current
        std::lock_guard<std::mutex>* lidarLockPtr = new std::lock_guard<std::mutex>(_lidarDeltaPoseMutex);

        // Find closest lidar key in existing graph
        IMUMapItr lidarMapItr;
        _imuBuffer.getClosestIMUBufferIteratorToTime(_currentCompslamTime.toSec(), lidarMapItr);
        std::cout << "Found time stamp is: " << lidarMapItr->first << std::endl;
        gtsam::Key closestLidarKey;
        _imuBuffer.getCorrespondingGtsamKey(lidarMapItr->first, closestLidarKey);

        // Create new key
        gtsam::Key lastLidarKey = _currentLidarKey;
        _currentLidarKey = closestLidarKey;

        std::cout << "Last IMU key: " << lastImuKey << ", current IMU key: " << _currentImuKey << std::endl;
        std::cout << "Adding the " << ++numLidarFactors << "th LiDAR factor. "
                  << "Last LiDAR Key: " << lastLidarKey << ", current LiDAR Key: " << _currentLidarKey << std::endl;
        std::cout << "Delta Pose: " << _lidarDeltaPose << std::endl;

        // Write the lidar odom delta to the graph
        _graphMgr.addPoseBetweenFactor(lastLidarKey, _currentLidarKey, _lidarDeltaPose);
        _newLidarDeltaPose = false;

        // Release mutex
        delete lidarLockPtr;
      }

      // Update graph
      /// Add constraints in the beginning before the first update
      // if (numLidarFactors > 3) {
      _graphMgr.updateGraphAndState(_currentImuTime.toSec(), _currentImuKey);
      //}
      _newImuMeasurement = false;
      delete imuLockPtr;
      // Write output to corresponding member variable for advertisement
      gtsam::Pose3 I_T_rel = _graphMgr._state.navState().pose();
      // Transform relative transformation in IMU frame to T_OI
      tf::StampedTransform tfT_CI, tfT_IC;
      _tfListener.lookupTransform(_cabinFrame, _imuFrame, _currentImuTime, tfT_CI);
      _tfListener.lookupTransform(_imuFrame, _cabinFrame, _currentImuTime, tfT_IC);
      tf::StampedTransform I_tfT_rel, tfT_OC;
      Eigen::Quaterniond I_r_rel = I_T_rel.rotation().toQuaternion();
      I_tfT_rel.setRotation(tf::Quaternion(I_r_rel.x(), I_r_rel.y(), I_r_rel.z(), I_r_rel.w()));
      I_tfT_rel.setOrigin(tf::Vector3(I_T_rel.x(), I_T_rel.y(), I_T_rel.z()));
      tfT_OC.setData(tfT_CI * I_tfT_rel * tfT_IC);

      _tf_T_OC.setData(tfT_OC);
      _tf_T_OC.stamp_ = _currentImuTime;
    }
    endLoopTime = std::chrono::system_clock::now();
    //ROS_WARN_STREAM("FactorGraphUpdate took "
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
  ros::Time currentFactorGraphStamp;
  ros::Time lastFactorGraphStamp;
  ros::Time currentCompslamStamp;
  ros::Time lastCompslamStamp;
  // Variables
  tf::StampedTransform tf_T_CB;

  // Main loop of thread
  while (ros::ok()) {
    // Starting time of the iteration
    startLoopTime = std::chrono::system_clock::now();
    currentFactorGraphStamp = _tf_T_OC.stamp_;
    currentCompslamStamp = _currentCompslamTime;

    // Check whether system is initialized --> set flag
    if (!_systemInited && _imuAligned && _graphInited) {
      _systemInited = true;
      ROS_WARN("System fully initialized, start publishing.");
      // Initialize timing variables
      lastFactorGraphStamp = currentFactorGraphStamp;
      lastCompslamStamp = currentCompslamStamp;
    }
    // Actual publishing
    else {
      if (_systemInited && currentFactorGraphStamp != lastFactorGraphStamp) {
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
        lastFactorGraphStamp = currentFactorGraphStamp;
      }
      // Visualization of Compslam pose
      if (_systemInited && currentCompslamStamp != lastCompslamStamp) {
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
        lastCompslamStamp = currentCompslamStamp;
      }
    }
    
    // Time to exactly 100 Hz
    endLoopTime = std::chrono::system_clock::now();
    // ROS_WARN_STREAM("Odometry publishing took "
    //                 << std::chrono::duration_cast<std::chrono::milliseconds>(endLoopTime - startLoopTime).count()
    //                 << " milliseconds.");
    //std::this_thread::sleep_for(std::chrono::milliseconds(10) - (endLoopTime - startLoopTime));
  }
}

}  // end namespace fg_filtering
