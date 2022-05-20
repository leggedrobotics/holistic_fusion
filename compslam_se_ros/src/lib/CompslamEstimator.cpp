#include "compslam_se_ros/CompslamEstimator.h"

namespace compslam_se {

CompslamEstimator::CompslamEstimator(ros::NodeHandle& node, ros::NodeHandle& privateNode) {
  std::cout << YELLOW_START << "CompslamEstimator" << GREEN_START << " Setting up." << COLOR_END << std::endl;

  // Configurations
  graphConfigPtr_ = new compslam_se::GraphConfig();
  staticTransformsPtr_ = new StaticTransformsTf(privateNode);

  // Get ROS params and set extrinsics
  readParams_(privateNode);
  staticTransformsPtr_->findTransformations();

  bool success = compslam_se::CompslamSeInterface::setup_(node);

  // Publishers
  pubOdometryImu_ = node.advertise<nav_msgs::Odometry>("/compslam_se/transform_odom_imu", ROS_QUEUE_SIZE);
  pubOdometryLidar_ = node.advertise<nav_msgs::Odometry>("/compslam_se/transform_odom_lidar", ROS_QUEUE_SIZE);
  pubWorldLidar_ = node.advertise<nav_msgs::Odometry>("/compslam_se/transform_world_lidar", ROS_QUEUE_SIZE);
  pubWorldImu_ = node.advertise<nav_msgs::Odometry>("/compslam_se/transform_world_imu", ROS_QUEUE_SIZE);
  pubOdomPath_ = node.advertise<nav_msgs::Path>("/compslam_se/odom_lidar_path", ROS_QUEUE_SIZE);

  // Messages
  odomPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);

  // Subscribers
  /// Remapped IMU topic
  subImu_ = node.subscribe<sensor_msgs::Imu>("/imu_topic", ROS_QUEUE_SIZE, &CompslamEstimator::imuCabinCallback_, this,
                                             ros::TransportHints().tcpNoDelay());
  std::cout << YELLOW_START << "CompslamEstimator" << COLOR_END << " Initialized IMU cabin subscriber." << std::endl;
  /// Remapped LiDAR odometry topic
  subLidarOdometry_ = node.subscribe<nav_msgs::Odometry>(
      "/lidar_odometry_topic", ROS_QUEUE_SIZE, &CompslamEstimator::lidarOdometryCallback_, this, ros::TransportHints().tcpNoDelay());
  std::cout << YELLOW_START << "CompslamEstimator" << COLOR_END << " Initialized LiDAR Odometry subscriber." << std::endl;
  // GNSS usage
  if (!graphConfigPtr_->usingGnssFlag) {
    std::cout << YELLOW_START << "CompslamEstimator" << GREEN_START
              << " GNSS usage is set to false. Hence, lidar unary factors will be activated after graph initialization." << COLOR_END
              << std::endl;
  }

  std::cout << YELLOW_START << "CompslamEstimator" << GREEN_START << " Set up successfully." << COLOR_END << std::endl;
}

void CompslamEstimator::imuCabinCallback_(const sensor_msgs::Imu::ConstPtr& imuMsgPtr) {
  static bool firstCallbackFlag__ = true;
  static ros::Time lastTime__;
  if (firstCallbackFlag__) {
    lastTime__ = imuMsgPtr->header.stamp;
    firstCallbackFlag__ = false;
    startTime_ = std::chrono::high_resolution_clock::now();
  }
  currentTime_ = std::chrono::high_resolution_clock::now();

  // std::cout << ros::Duration(imuMsgPtr->header.stamp.toSec() - lastTime__.toSec()).toSec() << std::endl;

  Eigen::Vector3d linearAcc(imuMsgPtr->linear_acceleration.x, imuMsgPtr->linear_acceleration.y, imuMsgPtr->linear_acceleration.z);
  Eigen::Vector3d angularVel(imuMsgPtr->angular_velocity.x, imuMsgPtr->angular_velocity.y, imuMsgPtr->angular_velocity.z);
  compslam_se::CompslamSeInterface::addImuMeasurement_(linearAcc, angularVel, imuMsgPtr->header.stamp);

  // Last time
  lastTime__ = imuMsgPtr->header.stamp;
}

void CompslamEstimator::lidarOdometryCallback_(const nav_msgs::Odometry::ConstPtr& odomLidarPtr) {
  // Static members
  static Eigen::Matrix4d compslam_T_I0_O__;
  static int lidarCallbackCounter__ = -1;
  static std::shared_ptr<compslam_se::UnaryMeasurement6D> odometryKm1Ptr__;

  // Counter
  ++lidarCallbackCounter__;

  tf::Transform tf_compslam_T_O_Lk;
  compslam_se::odomMsgToTf(*odomLidarPtr, tf_compslam_T_O_Lk);
  // Transform message to Imu frame
  const tf::Transform tf_compslam_T_O_Ik = tf_compslam_T_O_Lk * staticTransformsPtr_->T_L_I();
  Eigen::Matrix4d compslam_T_O_Ik;
  compslam_se::tfToMatrix4(tf_compslam_T_O_Ik, compslam_T_O_Ik);

  // Compute Pose Between Transformation
  const Eigen::Matrix4d compslam_T_I0_Ik = compslam_T_I0_O__ * compslam_T_O_Ik;

  // Create Measurements
  double timeK = odomLidarPtr->header.stamp.toSec();
  std::shared_ptr<compslam_se::UnaryMeasurement6D> poseBetweenKPtr =
      std::make_unique<compslam_se::UnaryMeasurement6D>("Lidar 6D", int(lidarRate_), timeK, compslam_T_I0_Ik, poseUnaryNoise_);
  std::shared_ptr<compslam_se::UnaryMeasurement6D> poseUnaryKPtr =
      std::make_unique<compslam_se::UnaryMeasurement6D>("Lidar 6D", int(lidarRate_), timeK, compslam_T_O_Ik, poseUnaryNoise_);

  // Set initial criterion
  if (lidarCallbackCounter__ == 0) {
    compslam_T_I0_O__ = compslam_T_O_Ik.inverse();
    return;
  } else if (lidarCallbackCounter__ == 1) {
    odometryKm1Ptr__ = poseBetweenKPtr;
    return;
  }

  // compslam_se::CompslamSeInterface::addOdometryMeasurement_(*odometryKm1Ptr__, *odometryKPtr, poseBetweenNoise_);
  compslam_se::CompslamSeInterface::addUnaryPoseMeasurement_(*poseUnaryKPtr);

  if (!areYawAndPositionInited_() && (!graphConfigPtr_->usingGnssFlag || (secondsSinceStart_() > 15))) {
    std::cout << YELLOW_START << "CompslamEstimator" << GREEN_START
              << " LiDAR odometry callback is setting global cabin position and yaw to LiDAR Pose, as it was not set so far." << COLOR_END
              << std::endl;
    compslam_se::CompslamSeInterface::initYawAndPosition_(poseUnaryKPtr->measurementPose);
  }

  // Wrap up iteration
  odometryKm1Ptr__ = poseBetweenKPtr;
}

void CompslamEstimator::imuBaseCallback_(const sensor_msgs::Imu::ConstPtr& imuMsgPtr) {
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

void CompslamEstimator::publishState_(ros::Time imuTimeK, const Eigen::Matrix4d& T_W_O, const Eigen::Matrix4d& T_O_Ik,
                                      const Eigen::Vector3d& Ic_v_W_Ic, const Eigen::Vector3d& I_w_W_I) {
  // Used transforms
  tf::Transform tf_T_I_L = staticTransformsPtr_->T_L_I().inverse();

  // Pose
  /// From Eigen to TF
  tf::Transform tf_T_O_I = compslam_se::matrix4ToTf(T_O_Ik);
  tf::Transform tf_T_W_O = compslam_se::matrix4ToTf(T_W_O);
  tf::Transform tf_T_O_L = tf_T_O_I * tf_T_I_L;
  tf::Transform tf_T_W_L = tf_T_W_O * tf_T_O_L;

  // Publish odometry message for odom->lidar at 100 Hz
  nav_msgs::OdometryPtr odomLidarMsgPtr(new nav_msgs::Odometry);
  odomLidarMsgPtr->header.frame_id = staticTransformsPtr_->getOdomFrame();
  odomLidarMsgPtr->child_frame_id = staticTransformsPtr_->getLidarFrame();
  odomLidarMsgPtr->header.stamp = imuTimeK;
  tf::poseTFToMsg(tf_T_O_L, odomLidarMsgPtr->pose.pose);
  pubOdometryLidar_.publish(odomLidarMsgPtr);

  // Publish path for IMU frame
  /// Pose
  //  geometry_msgs::PoseStamped poseStamped;
  //  poseStamped.header.frame_id = staticTransformsPtr_->getOdomFrame();
  //  poseStamped.header.stamp = imuTimeK;
  //  tf::poseTFToMsg(tf_T_O_L, poseStamped.pose);
  //  /// Path
  //  odomPathPtr_->header.frame_id = staticTransformsPtr_->getOdomFrame();
  //  odomPathPtr_->header.stamp = imuTimeK;
  //  odomPathPtr_->poses.push_back(poseStamped);
  //  /// Publish
  //  pubOdomPath_.publish(odomPathPtr_);

  // Publish odometry message for map->lidar with 100 Hz
  nav_msgs::OdometryPtr worldLidarMsgPtr(new nav_msgs::Odometry);
  worldLidarMsgPtr->header.frame_id = staticTransformsPtr_->getMapFrame();
  worldLidarMsgPtr->child_frame_id = staticTransformsPtr_->getLidarFrame();
  worldLidarMsgPtr->header.stamp = imuTimeK;
  tf::poseTFToMsg(tf_T_W_L, worldLidarMsgPtr->pose.pose);
  pubWorldLidar_.publish(worldLidarMsgPtr);

  // Publish odometry message for map->imu with 100 Hz
  nav_msgs::OdometryPtr worldImuMsgPtr(new nav_msgs::Odometry);
  worldImuMsgPtr->header.frame_id = staticTransformsPtr_->getMapFrame();
  worldImuMsgPtr->child_frame_id = staticTransformsPtr_->getImuFrame();
  worldImuMsgPtr->header.stamp = imuTimeK;
  tf::Pose tf_T_W_Ik = compslam_se::matrix4ToTf(T_W_O * T_O_Ik);
  tf::poseTFToMsg(tf_T_W_Ik, worldImuMsgPtr->pose.pose);
  pubWorldImu_.publish(worldImuMsgPtr);
}

long CompslamEstimator::secondsSinceStart_() {
  return std::chrono::duration_cast<std::chrono::seconds>(currentTime_ - startTime_).count();
}

}  // namespace compslam_se