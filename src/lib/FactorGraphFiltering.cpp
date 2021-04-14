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
  /// LiDAR frame
  if (privateNode.getParam("lidarFrame", sParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - LiDAR frame: " << sParam);
    setLidarFrame(sParam);
  } else {
    ROS_WARN("FactorGraphFiltering - LiDAR frame not set");
  }
  /// IMU
  if (privateNode.getParam("imuFrame", sParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - IMU frame for preintegrator and tf: " << sParam);
    setImuFrame(sParam);
    _odometryTrans.frame_id_ = sParam;
  } else
    ROS_WARN("FactorGraphFiltering - IMU frame not set for preintegrator");
  /// Odom
  if (privateNode.getParam("odomFrame", sParam)) {
    ROS_INFO_STREAM("FactorGraphFiltering - Odom frame set to: " << sParam);
    setOdomFrame(sParam);
    _odometryTrans.child_frame_id_ = sParam;
  } else {
    ROS_WARN("FactorGraphFiltering - Odom frame not set");
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
  _pubOdometry = node.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 5);
  _pubLaserImuBias = node.advertise<sensor_msgs::Imu>("/laser_odom_imu_bias", 20);

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
void FactorGraphFiltering::lidarOdometryCallback(const nav_msgs::Odometry::ConstPtr& lidar_odometry_ptr) {
  //ROS_INFO("Reading lidar odometry message.");
  //Add to buffer
  _lidarOdometryBuffer.addToLidarOdometryBuffer();

  // Publish to tf
  _odometryTrans.stamp_ = lidar_odometry_ptr->header.stamp;
  tf::Quaternion tfQuaternion;
  tf::quaternionMsgToTF(lidar_odometry_ptr->pose.pose.orientation, tfQuaternion);
  _odometryTrans.setRotation(tfQuaternion);
  _odometryTrans.setOrigin(tf::Vector3(lidar_odometry_ptr->pose.pose.position.x, lidar_odometry_ptr->pose.pose.position.y, lidar_odometry_ptr->pose.pose.position.z));
  _tfBroadcaster.sendTransform(_odometryTrans);
  ROS_INFO("TF broadcasted.");
}

// process ----------------------------------------------------------------------------------------
void FactorGraphFiltering::process() {
  ROS_INFO("Processing...");
  //publishResult();
}

// publish result ----------------------------------------------------------------------------------------
void FactorGraphFiltering::publishResult() {
  // publish odometry transformations
  geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(transformSum().rot_z.rad(),
                                                                              -transformSum().rot_x.rad(),
                                                                              -transformSum().rot_y.rad());

  // initialize odometry and odometry tf messages
  nav_msgs::OdometryPtr odometryMsgPtr(new nav_msgs::Odometry);
  odometryMsgPtr->header.frame_id = _odometryTrans.frame_id_;
  odometryMsgPtr->child_frame_id = _odometryTrans.child_frame_id_;
  odometryMsgPtr->header.stamp = _timeUpdate;
  odometryMsgPtr->pose.pose.orientation.x = -geoQuat.y;
  odometryMsgPtr->pose.pose.orientation.y = -geoQuat.z;
  odometryMsgPtr->pose.pose.orientation.z = geoQuat.x;
  odometryMsgPtr->pose.pose.orientation.w = geoQuat.w;
  odometryMsgPtr->pose.pose.position.x = transformSum().pos.x();
  odometryMsgPtr->pose.pose.position.y = transformSum().pos.y();
  odometryMsgPtr->pose.pose.position.z = transformSum().pos.z();
  _pubOdometry.publish(odometryMsgPtr);

  _odometryTrans.stamp_ = _timeUpdate;
  _odometryTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
  _odometryTrans.setOrigin(tf::Vector3(transformSum().pos.x(), transformSum().pos.y(), transformSum().pos.z()));
  _tfBroadcaster.sendTransform(_odometryTrans);

  //IMU Bias
  if (_pubLaserImuBias.getNumSubscribers() > 0) {
    sensor_msgs::Imu imuBiasMsg;
    imuBiasMsg.header.frame_id = "/fg_odometry_imu";
    imuBiasMsg.header.stamp = _timeUpdate;
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
