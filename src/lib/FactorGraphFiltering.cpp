#include "fg_filtering/FactorGraphFiltering.h"

#include <pcl/filters/filter.h>

#include "fg_filtering/common.h"


namespace fg_filtering {

using std::asin;
using std::atan2;
using std::cos;
using std::fabs;
using std::pow;
using std::sin;
using std::sqrt;

FactorGraphFiltering::FactorGraphFiltering(float scanPeriod)
{
  ROS_INFO("FactorGraphFiltering instance created.");
}

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
    _tfT_OdomImu.frame_id_ = sParam;
    _tfT_OdomBase.frame_id_ = sParam;
  } else {
    ROS_WARN("FactorGraphFiltering - Odom frame not set");
  }
  /// base_link
  if (privateNode.getParam("baseLinkFrame", sParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - base_link frame: " << sParam);
    setBaseLinkFrame(sParam);
    _tfT_OdomBase.child_frame_id_ = sParam;
  } else
    ROS_WARN("FactorGraphFiltering - IMU frame not set for preintegrator");
  /// IMU
  if (privateNode.getParam("imuFrame", sParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - IMU frame for preintegrator and tf: " << sParam);
    setImuFrame(sParam);
    _tfT_OdomImu.child_frame_id_ = sParam;
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
  if (privateNode.getParam("accNoiseDensity", dParam))
    _graphMgr.setAccNoiseDensity(dParam);
  if (privateNode.getParam("accBiasRandomWalk", dParam))
    _graphMgr.setAccBiasRandomWalk(dParam);
  if (privateNode.getParam("accBiasPrior", dParam))
    _graphMgr.setAccBiasPrior(dParam);
  if (privateNode.getParam("gyrNoiseDensity", dParam))
    _graphMgr.setGyroNoiseDensity(dParam);
  if (privateNode.getParam("gyrBiasRandomWalk", dParam))
    _graphMgr.setGyrBiasRandomWalk(dParam);
  if (privateNode.getParam("gyrBiasPrior", dParam))
    _graphMgr.setGyrBiasPrior(dParam);
  if (privateNode.getParam("smootherLag", dParam))
    _graphMgr.setSmootherLag(dParam);
  if (privateNode.getParam("additonalIterations", iParam))
    _graphMgr.setIterations(iParam);
  if (privateNode.getParam("positionReLinTh", dParam))
    _graphMgr.setPositionReLinTh(dParam);
  if (privateNode.getParam("rotationReLinTh", dParam))
    _graphMgr.setRotationReLinTh(dParam);
  if (privateNode.getParam("velocityReLinTh", dParam))
    _graphMgr.setVelocityReLinTh(dParam);
  if (privateNode.getParam("accBiasReLinTh", dParam))
    _graphMgr.setAccBiasReLinTh(dParam);
  if (privateNode.getParam("gyrBiasReLinTh", dParam))
    _graphMgr.setGyrBiasReLinTh(dParam);
  if (privateNode.getParam("relinearizeSkip", iParam))
    _graphMgr._params.setRelinearizeSkip(iParam);
  if (privateNode.getParam("enableRelinearization", bParam))
    _graphMgr._params.setEnableRelinearization(bParam);
  if (privateNode.getParam("evaluateNonlinearError", bParam))
    _graphMgr._params.setEvaluateNonlinearError(bParam);
  if (privateNode.getParam("cacheLinearizedFactors", bParam))
    _graphMgr._params.setCacheLinearizedFactors(bParam);
  if (privateNode.getParam("findUnusedFactorSlots", bParam))
    _graphMgr._params.findUnusedFactorSlots = bParam;
  if (privateNode.getParam("enablePartialRelinearizationCheck", bParam))
    _graphMgr._params.setEnablePartialRelinearizationCheck(bParam);
  if (privateNode.getParam("enableDetailedResults", bParam))
    _graphMgr._params.setEnableDetailedResults(bParam);
  std::vector<double> poseNoise{0, 0, 0, 0, 0, 0};  //roll,pitch,yaw,x,y,z
  if (privateNode.getParam("poseBetweenNoise", poseNoise)){
    _graphMgr.setPoseNoise(poseNoise);
  }

  // Motion parameters
  /// Zero Motion Factor
  if (privateNode.getParam("zeroMotionDetection", bParam)) {
    setZeroMotionDetection(bParam);
    if (bParam)
      ROS_INFO_STREAM("FactorGraphFiltering - Zero Motion Detection ENABLED - Zero Velocity and Zero Delta Pose Factors will be added");
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
  _subImu = node.subscribe<sensor_msgs::Imu>("/imu_topic", 100, &FactorGraphFiltering::imuCallback, this, ros::TransportHints().tcpNoDelay());
  ROS_INFO("Initialized IMU subscriber.");
  /// subscribe to remapped LiDAR odometry topic
  _subLidarOdometry = node.subscribe<nav_msgs::Odometry>("/lidar_odometry_topic", 10, &FactorGraphFiltering::lidarOdometryCallback, this, ros::TransportHints().tcpNoDelay());
  ROS_INFO("Initialized LiDAR Odometry subscriber.");

  return true;
}

// imuCallback ----------------------------------------------------------------------------------------
void FactorGraphFiltering::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_ptr) {
  //ROS_INFO("Reading IMU message.");
  //Add to buffer
  _imuBuffer.addToIMUBuffer(imu_ptr->header.stamp.toSec(),
                            imu_ptr->linear_acceleration.x, imu_ptr->linear_acceleration.y, imu_ptr->linear_acceleration.z,
                            imu_ptr->angular_velocity.x, imu_ptr->angular_velocity.y, imu_ptr->angular_velocity.z);
}
// lidarOdometryCallback ----------------------------------------------------------------------------------------
void FactorGraphFiltering::lidarOdometryCallback(const nav_msgs::Odometry::ConstPtr& odomLidarPtr) {
  // Temporary Variables
  tf::Quaternion tfQuaternion;
  tf::StampedTransform tfT_LidarImu;
  tf::quaternionMsgToTF(odomLidarPtr->pose.pose.orientation, tfQuaternion);
  tf::Vector3 tfOrigin = tf::Vector3(odomLidarPtr->pose.pose.position.x, odomLidarPtr->pose.pose.position.y, odomLidarPtr->pose.pose.position.z);
  
  // Write current factor graph prediction to tf-Transformation
  /// MUTEX!
  _tfT_OdomCompslam.setRotation(tfQuaternion);
  _tfT_OdomCompslam.setOrigin(tfOrigin);
  _tfT_OdomLidar.setData(_tfT_OdomCompslam);
  _tfListener.lookupTransform(_lidarFrame, _imuFrame, ros::Time(0), tfT_LidarImu);
  _tfT_OdomImu.setData(_tfT_OdomLidar*tfT_LidarImu);
  _tfT_OdomImu.stamp_ = odomLidarPtr->header.stamp;

  // Add delta pose for Odom->Imu to graph
  // here the code will follow

  // Publish the prediction
  publishOdometryAndTF();
}

// process ----------------------------------------------------------------------------------------
void FactorGraphFiltering::process() {
  ROS_INFO("Processing...");
  //publishResult();
}

// publish result ----------------------------------------------------------------------------------------
void FactorGraphFiltering::publishOdometryAndTF() {
  // Acquire MUTEX! for _tfT_OdomIMU, s.t. all transformations correspond to same time

  
  // Get odom-->base_link transformation from odom-->imu
  //ROS_INFO_STREAM("Current ROS time: " << ros::Time::now());
  //ROS_INFO_STREAM("Current lidar odometry timestamp: " << _tfT_OdomImu.stamp_);
  _tfListener.lookupTransform(_imuFrame, _baseLinkFrame, ros::Time(0), _tfT_ImuBase); //_tfT_OdomImu.stamp_, _tfT_ImuBase);
  _tfT_OdomBase.setData(_tfT_OdomImu*_tfT_ImuBase);
  _tfT_OdomBase.stamp_ = _tfT_OdomImu.stamp_;

  // Publish to Tf tree
  _tfBroadcaster.sendTransform(_tfT_OdomBase);
  //ROS_INFO("TF broadcasted.");

  nav_msgs::OdometryPtr odomBaseMsgPtr(new nav_msgs::Odometry);
  odomBaseMsgPtr->header.frame_id = _tfT_OdomBase.frame_id_;
  odomBaseMsgPtr->child_frame_id = _tfT_OdomBase.child_frame_id_;
  odomBaseMsgPtr->header.stamp = _tfT_OdomBase.stamp_;
  tf::poseTFToMsg(_tfT_OdomBase, odomBaseMsgPtr->pose.pose);
  _pubOdometry.publish(odomBaseMsgPtr);

  //IMU Bias
  if (_pubLaserImuBias.getNumSubscribers() > 0) {
    sensor_msgs::Imu imuBiasMsg;
    imuBiasMsg.header.frame_id = "/fg_odometry_imu";
    imuBiasMsg.header.stamp = _tfT_OdomImu.stamp_;
    imuBiasMsg.linear_acceleration.x = graphIMUBias().linear_acceleration.x;
    imuBiasMsg.linear_acceleration.y = graphIMUBias().linear_acceleration.y;
    imuBiasMsg.linear_acceleration.z = graphIMUBias().linear_acceleration.z;
    imuBiasMsg.angular_velocity.x = graphIMUBias().angular_velocity.x;
    imuBiasMsg.angular_velocity.y = graphIMUBias().angular_velocity.y;
    imuBiasMsg.angular_velocity.z = graphIMUBias().angular_velocity.z;
    _pubLaserImuBias.publish(imuBiasMsg);
  }
}

}  // end namespace fg_filtering
