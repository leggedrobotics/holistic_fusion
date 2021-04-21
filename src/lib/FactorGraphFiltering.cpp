#include "fg_filtering/FactorGraphFiltering.h"

#include <pcl/filters/filter.h>

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
    _tf_T_OI.frame_id_ = sParam;
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
    _tf_T_OI.child_frame_id_ = sParam;
  } else
    ROS_WARN("FactorGraphFiltering - IMU frame not set for preintegrator");
  /// LiDAR frame
  if (privateNode.getParam("lidarFrame", sParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - LiDAR frame: " << sParam);
    setLidarFrame(sParam);
  } else {
    ROS_WARN("FactorGraphFiltering - LiDAR frame not set");
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
    ROS_INFO("Set loamVerbosity: %d", iParam);
    setVerboseLevel(iParam);
  } else
    setVerboseLevel(0);

  // Publishers
  /// advertise odometry topic
  _pubOdometry = privateNode.advertise<nav_msgs::Odometry>("/transform_odom_base", 5);
  _pubLaserImuBias = node.advertise<sensor_msgs::Imu>("/fg_imu_bias", 20);

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
  tf::Transform tf_T_OL_Compslam;
  tf_T_OL_Compslam.setRotation(tf_q_OL);
  tf_T_OL_Compslam.setOrigin(tf_t_OL);

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
      _currentLidarTime = odomLidarPtr->header.stamp;
      _currentLidarKey = _graphMgr._state.key();
      _currentImuKey = _currentLidarKey;

      // Initialize first node
      _graphMgr.initPoseVelocityBiasGraph(_currentLidarTime.toSec(),
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
      Eigen::Matrix4d T_last, T_last_inv, T_current, T_rel;
      T_last.block<3, 1>(0, 3) =
          Eigen::Vector3d(_tf_T_OI_CompslamLast.getOrigin().x(), _tf_T_OI_CompslamLast.getOrigin().y(),
                          _tf_T_OI_CompslamLast.getOrigin().z());
      T_last.block<3, 3>(0, 0) =
          Eigen::Quaterniond(_tf_T_OI_CompslamLast.getRotation().w(), _tf_T_OI_CompslamLast.getRotation().x(),
                             _tf_T_OI_CompslamLast.getRotation().y(), _tf_T_OI_CompslamLast.getRotation().z())
              .toRotationMatrix();
      T_current.block<3, 1>(0, 3) = Eigen::Vector3d(tf_T_OI_Compslam.getOrigin().x(), tf_T_OI_Compslam.getOrigin().y(),
                                                    tf_T_OI_Compslam.getOrigin().z());
      T_current.block<3, 3>(0, 0) =
          Eigen::Quaterniond(tf_T_OI_Compslam.getRotation().w(), tf_T_OI_Compslam.getRotation().x(),
                             tf_T_OI_Compslam.getRotation().y(), tf_T_OI_Compslam.getRotation().z())
              .toRotationMatrix();
      loam::invertHomogenousMatrix(T_last, T_last_inv);
      T_rel = T_last_inv * T_current;

      // Write to delta pose --> mutex such that time and delta pose always correspond to each other
      std::lock_guard<std::mutex>* lockPtr = new std::lock_guard<std::mutex>(_lidarDeltaPoseMutex);
      // Set to new delta pose
      _lidarDeltaPose = gtsam::Pose3(T_rel);
      // Set LiDAR time
      _lastLidarTime = _currentLidarTime;
      _currentLidarTime = odomLidarPtr->header.stamp;
      // Set Flag
      _newLidarDeltaPose = true;
      delete lockPtr;

      // Preliminary until output comes from graph
      // Important
      //_tf_T_OI = tf_T_OI_Compslam;
    }
  }
  if (_firstScanCallback) {
    _firstScanCallback = false;
  }

  // Set last pose for the next iteration
  _tf_T_OI_CompslamLast = tf_T_OI_Compslam;
}

// write to graph ----------------------------------------------------------------------------------------
void FactorGraphFiltering::writeToGraph() {
  ROS_INFO("Thread for adding IMU factors is initialized.");
  while (ros::ok()) {
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
      // std::cout << "Delay of LiDAR measurement[s]: " << _currentImuTime.toSec() - _currentLidarTime.toSec()
      //          << std::endl;
      gtsam::Key lastImuKey = _currentImuKey;
      _currentImuKey = _graphMgr.newStateKey();
      std::cout << "Lidar key: " << _currentLidarKey << ", last IMU key: " << lastImuKey
                << ", current IMU key: " << _currentImuKey << std::endl;

      // Write to key buffer
      _imuBuffer.addToKeyBuffer(_currentImuTime.toSec(), _currentImuKey);

      // Create IMU Map
      loam::IMUMap imuMeas;
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
        loam::IMUMapItr lidarMapItr;
        _imuBuffer.getClosestIMUBufferIteratorToTime(_currentLidarTime.toSec(), lidarMapItr);
        std::cout << "Found time stamp is: " << lidarMapItr->first << std::endl;
        gtsam::Key closestLidarKey; 
        _imuBuffer.getCorrespondingGtsamKey(lidarMapItr->first, closestLidarKey);

        // Create new key
        gtsam::Key lastLidarKey = _currentLidarKey;
        _currentLidarKey = closestLidarKey;

        std::cout << "Last LiDAR Key: " << lastLidarKey << ", ";
        std::cout << "Current LiDAR Key: " << _currentLidarKey << std::endl;

        // Write the lidar odom delta to the graph
        _graphMgr.addPoseBetweenFactor(lastLidarKey, _currentLidarKey, _lidarDeltaPose);
        _newLidarDeltaPose = false;

        // Release mutex
        delete lidarLockPtr;
      }

      // Update graph
      _graphMgr.updateGraphAndState(_currentImuTime.toSec(), _currentImuKey);
      _newImuMeasurement = false;
      delete imuLockPtr;
      // Write output to corresponding member variable for advertisement
      gtsam::Pose3 navState = _graphMgr._state.navState().pose();
      Eigen::Quaterniond quat = navState.rotation().toQuaternion();
      _tf_T_OI.setRotation(tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()));
      _tf_T_OI.setOrigin(tf::Vector3(navState.x(), navState.y(), navState.z()));
      _tf_T_OI.stamp_ = _currentImuTime;
    }
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
  ros::Time currentStamp;
  ros::Time lastStamp;
  // Variables
  tf::StampedTransform tf_T_IB;

  // Main loop of thread
  while (ros::ok()) {
    // Starting time of the iteration
    startLoopTime = std::chrono::system_clock::now();
    currentStamp = _tf_T_OI.stamp_;

    // Check whether system is initialized --> set flag
    if (!_systemInited && _imuAligned && _graphInited) {
      _systemInited = true;
      ROS_WARN("System fully initialized, start publishing.");
    }

    // Actual publishing
    if (_systemInited && currentStamp != lastStamp) {
      // Get odom-->base_link transformation from odom-->imu
      _tfListener.lookupTransform(_imuFrame, _baseLinkFrame, ros::Time(0), tf_T_IB);
      _tf_T_OB.setData(_tf_T_OI * tf_T_IB);
      _tf_T_OB.stamp_ = _tf_T_OI.stamp_;

      // Publish to Tf tree
      _tfBroadcaster.sendTransform(_tf_T_OB);
      // ROS_INFO("TF broadcasted.");

      nav_msgs::OdometryPtr odomBaseMsgPtr(new nav_msgs::Odometry);
      odomBaseMsgPtr->header.frame_id = _tf_T_OB.frame_id_;
      odomBaseMsgPtr->child_frame_id = _tf_T_OB.child_frame_id_;
      odomBaseMsgPtr->header.stamp = _tf_T_OB.stamp_;
      tf::poseTFToMsg(_tf_T_OB, odomBaseMsgPtr->pose.pose);
      _pubOdometry.publish(odomBaseMsgPtr);

      // IMU Bias
      if (_pubLaserImuBias.getNumSubscribers() > 0) {
        sensor_msgs::Imu imuBiasMsg;
        imuBiasMsg.header.frame_id = "/fg_odometry_imu";
        imuBiasMsg.header.stamp = _tf_T_OI.stamp_;
        imuBiasMsg.linear_acceleration.x = graphIMUBias().linear_acceleration.x;
        imuBiasMsg.linear_acceleration.y = graphIMUBias().linear_acceleration.y;
        imuBiasMsg.linear_acceleration.z = graphIMUBias().linear_acceleration.z;
        imuBiasMsg.angular_velocity.x = graphIMUBias().angular_velocity.x;
        imuBiasMsg.angular_velocity.y = graphIMUBias().angular_velocity.y;
        imuBiasMsg.angular_velocity.z = graphIMUBias().angular_velocity.z;
        _pubLaserImuBias.publish(imuBiasMsg);
      }
    }
    lastStamp = currentStamp;
    // Time to exactly 100 Hz
    endLoopTime = std::chrono::system_clock::now();
    std::this_thread::sleep_for(std::chrono::milliseconds(10) - (endLoopTime - startLoopTime));
  }
}

}  // end namespace fg_filtering
