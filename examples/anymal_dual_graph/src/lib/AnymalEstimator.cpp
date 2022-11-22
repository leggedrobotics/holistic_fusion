/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "anymal_dual_graph/AnymalEstimator.h"

// Project
#include "anymal_dual_graph/AnymalStaticTransforms.h"

// Workspace
#include "graph_msf/measurements/BinaryMeasurement6D.h"
#include "graph_msf/measurements/UnaryMeasurement1D.h"
#include "graph_msf/measurements/UnaryMeasurement3D.h"
#include "graph_msf/measurements/UnaryMeasurement6D.h"
#include "graph_msf_ros/util/conversions.h"

namespace anymal_se {

AnymalEstimator::AnymalEstimator(std::shared_ptr<ros::NodeHandle> privateNodePtr) : privateNode_(*privateNodePtr) {
  std::cout << YELLOW_START << "AnymalEstimator" << GREEN_START << " Setting up." << COLOR_END << std::endl;

  // Configurations ----------------------------
  graphConfigPtr_ = std::make_shared<graph_msf::GraphConfig>();
  staticTransformsPtr_ = std::make_shared<AnymalStaticTransforms>(privateNodePtr);
  gnssHandlerPtr_ = std::make_shared<graph_msf::GnssHandler>();
  trajectoryAlignmentHandlerPtr_ = std::make_shared<graph_msf::TrajectoryAlignmentHandler>();

  // Get ROS params and set extrinsics
  readParams_(privateNode_);
  staticTransformsPtr_->findTransformations();

  if (not graph_msf::GraphMsfInterface::setup_()) {
    throw std::runtime_error("CompslamSeInterface could not be initiallized");
  }

  // Publishers ----------------------------
  initializePublishers_(privateNodePtr);

  // Subscribers ----------------------------
  initializeSubscribers_(privateNodePtr);

  // Server ----------------------------
  initializeServers_(privateNodePtr);

  // Messages ----------------------------
  initializeMessages_(privateNodePtr);

  initialized_ = false;
  trajectoryAlignmentHandlerPtr_->initHandler();

  std::cout << YELLOW_START << "CompslamEstimator" << GREEN_START << " Set up successfully." << COLOR_END << std::endl;
}

void AnymalEstimator::initializePublishers_(std::shared_ptr<ros::NodeHandle>& privateNodePtr) {
  // Odometry
  pubEstOdomImu_ = privateNode_.advertise<nav_msgs::Odometry>("/graph_msf/odometry_odom_imu", ROS_QUEUE_SIZE);
  pubEstMapImu_ = privateNode_.advertise<nav_msgs::Odometry>("/graph_msf/odometry_map_imu", ROS_QUEUE_SIZE);
  // Paths
  pubEstOdomImuPath_ = privateNode_.advertise<nav_msgs::Path>("/graph_msf/est_path_odom_imu", ROS_QUEUE_SIZE);
  pubEstMapImuPath_ = privateNode_.advertise<nav_msgs::Path>("/graph_msf/est_path_map_imu", ROS_QUEUE_SIZE);
  pubMeasMapGnssPath_ = privateNode_.advertise<nav_msgs::Path>("/graph_msf/meas_path_map_gnss", ROS_QUEUE_SIZE);
  pubMeasMapLidarPath_ = privateNode_.advertise<nav_msgs::Path>("/graph_msf/meas_path_map_Lidar", ROS_QUEUE_SIZE);
}

void AnymalEstimator::initializeSubscribers_(std::shared_ptr<ros::NodeHandle>& privateNodePtr) {
  // Imu
  subImu_ = privateNode_.subscribe<sensor_msgs::Imu>("/imu_topic", ROS_QUEUE_SIZE, &AnymalEstimator::imuCallback_, this,
                                                     ros::TransportHints().tcpNoDelay());
  std::cout << YELLOW_START << "CompslamEstimator" << COLOR_END << " Initialized IMU cabin subscriber with topic: " << subImu_.getTopic()
            << std::endl;

  // LiDAR Odometry
  subLidarOdometry_ = privateNode_.subscribe<nav_msgs::Odometry>(
      "/lidar_odometry_topic", ROS_QUEUE_SIZE, &AnymalEstimator::lidarOdometryCallback_, this, ros::TransportHints().tcpNoDelay());
  std::cout << YELLOW_START << "CompslamEstimator" << COLOR_END
            << " Initialized LiDAR Odometry subscriber with topic: " << subLidarOdometry_.getTopic() << std::endl;

  // GNSS
  if (graphConfigPtr_->usingGnssFlag) {
    subGnss_ = privateNode_.subscribe<sensor_msgs::NavSatFix>("/gnss_topic", ROS_QUEUE_SIZE, &AnymalEstimator::gnssCallback_, this,
                                                              ros::TransportHints().tcpNoDelay());
    std::cout << YELLOW_START << "FactorGraphFiltering" << COLOR_END << " Initialized Gnss subscriber with topic: " << subGnss_.getTopic()
              << std::endl;
  } else {
    std::cout << YELLOW_START << "CompslamEstimator" << GREEN_START
              << " Gnss usage is set to false. Hence, lidar unary factors will be activated after graph initialization." << COLOR_END
              << std::endl;
  }
}

void AnymalEstimator::initializeServers_(std::shared_ptr<ros::NodeHandle>& privateNodePtr) {
  serverTransformGnssToEnu_ =
      privateNode_.advertiseService("/gnss_coordinates_to_enu_topic", &AnymalEstimator::gnssCoordinatesToENUCallback_, this);
}

void AnymalEstimator::initializeMessages_(std::shared_ptr<ros::NodeHandle>& privateNodePtr) {
  // Odometry
  odomImuMsgPtr_ = nav_msgs::OdometryPtr(new nav_msgs::Odometry);
  mapImuMsgPtr_ = nav_msgs::OdometryPtr(new nav_msgs::Odometry);
  // Path
  estOdomImuPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  estMapImuPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  measMapGnssPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  measMapLidarPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
}

bool AnymalEstimator::gnssCoordinatesToENUCallback_(graph_msf_msgs::GetPathInEnu::Request& req,
                                                    graph_msf_msgs::GetPathInEnu::Response& res) {
  nav_msgs::PathPtr enuPathPtr = nav_msgs::PathPtr(new nav_msgs::Path);
  for (auto& coordinate : req.wgs84Coordinates) {
    Eigen::Vector3d enuCoordinate;
    Eigen::Vector3d gnssCoordinate = Eigen::Vector3d(coordinate.latitude, coordinate.longitude, coordinate.altitude);
    gnssHandlerPtr_->convertNavSatToPosition(gnssCoordinate, enuCoordinate);

    addToPathMsg(enuPathPtr, dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getMapFrame(), ros::Time::now(),
                 enuCoordinate, std::numeric_limits<int>::max());
  }
  res.gnssEnuPath = *enuPathPtr;

  return true;
}

void AnymalEstimator::imuCallback_(const sensor_msgs::Imu::ConstPtr& imuMsgPtr) {
  static bool firstCallbackFlag__ = true;
  if (firstCallbackFlag__) {
    firstCallbackFlag__ = false;
  }
  Eigen::Vector3d linearAcc(imuMsgPtr->linear_acceleration.x, imuMsgPtr->linear_acceleration.y, imuMsgPtr->linear_acceleration.z);
  Eigen::Vector3d angularVel(imuMsgPtr->angular_velocity.x, imuMsgPtr->angular_velocity.y, imuMsgPtr->angular_velocity.z);
  graph_msf::GraphMsfInterface::addImuMeasurementAndPublishState_(linearAcc, angularVel, imuMsgPtr->header.stamp.toSec());
}

void AnymalEstimator::lidarOdometryCallback_(const nav_msgs::Odometry::ConstPtr& odomLidarPtr) {
  // Static members
  static int odometryCallbackCounter__ = -1;
  static std::shared_ptr<graph_msf::UnaryMeasurement6D> odometryKm1Ptr__;

  // Counter
  ++odometryCallbackCounter__;

  Eigen::Matrix4d compslam_T_Wl_Lk;
  graph_msf::odomMsgToEigen(*odomLidarPtr, compslam_T_Wl_Lk);

  if (not initialized_ && graphConfigPtr_->usingGnssFlag) {
    trajectoryAlignmentHandlerPtr_->addLidarPose(compslam_T_Wl_Lk.block<3, 1>(0, 3), odomLidarPtr->header.stamp.toSec());
    --odometryCallbackCounter__;
    return;
  }

  // Transform to IMU frame
  double timeK = odomLidarPtr->header.stamp.toSec();

  // Measurement
  std::shared_ptr<graph_msf::UnaryMeasurement6D> odometryKPtr;
  // Create pseudo unary factors
  if (graphConfigPtr_->usingGnssFlag) {
    odometryKPtr = std::make_unique<graph_msf::UnaryMeasurement6D>(
        "Lidar 6D", dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getLidarFrame(), int(lidarRate_), timeK,
        compslam_T_Wl_Lk, poseUnaryNoise_);
    if (odometryCallbackCounter__ > 0) {
      graph_msf::GraphMsfInterface::addDualOdometryMeasurement_(*odometryKm1Ptr__, *odometryKPtr, poseBetweenNoise_);
    }
  } else {  // real unary factors
    odometryKPtr = std::make_unique<graph_msf::UnaryMeasurement6D>(
        "Lidar 6D", dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getLidarFrame(), int(lidarRate_), timeK,
        compslam_T_Wl_Lk, poseUnaryNoise_);
    graph_msf::GraphMsfInterface::addUnaryPoseMeasurement_(*odometryKPtr);
  }

  if (!areYawAndPositionInited_() && (!graphConfigPtr_->usingGnssFlag || secondsSinceStart_() > 15)) {
    std::cout << YELLOW_START << "AnymalEstimator" << GREEN_START
              << " LiDAR odometry callback is setting global cabin yaw to 0, as it was not set so far." << COLOR_END << std::endl;
    graph_msf::GraphMsfInterface::initYawAndPosition_(compslam_T_Wl_Lk,
                                                      dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getLidarFrame());
  }

  // Wrap up iteration
  odometryKm1Ptr__ = odometryKPtr;

  // Add to path message
  addToPathMsg(measMapLidarPathPtr_, dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getMapFrame(),
               odomLidarPtr->header.stamp, compslam_T_Wl_Lk.block<3, 1>(0, 3), graphConfigPtr_->imuBufferLength * 4);

  // Publish Path
  pubMeasMapLidarPath_.publish(measMapLidarPathPtr_);
}

void AnymalEstimator::gnssCallback_(const sensor_msgs::NavSatFix::ConstPtr& gnssMsgPtr) {
  // Static method variables
  static Eigen::Vector3d accumulatedCoordinates__(0.0, 0.0, 0.0);
  static Eigen::Vector3d W_t_W_Gnss_km1__;
  static int gnssCallbackCounter__ = 0;
  Eigen::Vector3d W_t_W_Gnss;

  // Start
  ++gnssCallbackCounter__;
  Eigen::Vector3d gnssCoord = Eigen::Vector3d(gnssMsgPtr->latitude, gnssMsgPtr->longitude, gnssMsgPtr->altitude);
  Eigen::Vector3d estCovarianceXYZ(gnssMsgPtr->position_covariance[0], gnssMsgPtr->position_covariance[4],
                                   gnssMsgPtr->position_covariance[8]);

  if (!graphConfigPtr_->usingGnssFlag) {
    ROS_WARN("Received Gnss message, but usage is set to false.");
    graph_msf::GraphMsfInterface::activateFallbackGraph();
    return;
  } else if (not initialized_) {
    if (gnssCallbackCounter__ < NUM_GNSS_CALLBACKS_UNTIL_START + 1) {
      // Wait until measurements got accumulated
      accumulatedCoordinates__ += gnssCoord;
      if (!(gnssCallbackCounter__ % 10)) {
        std::cout << YELLOW_START << "AnymalEstimator" << COLOR_END << " NOT ENOUGH Gnss MESSAGES ARRIVED!" << std::endl;
      }
      return;
    } else if (gnssCallbackCounter__ == NUM_GNSS_CALLBACKS_UNTIL_START + 1) {
      gnssHandlerPtr_->initHandler(accumulatedCoordinates__ / NUM_GNSS_CALLBACKS_UNTIL_START);
      ++gnssCallbackCounter__;
    }
    // Convert to cartesian coordinates
    gnssHandlerPtr_->convertNavSatToPosition(gnssCoord, W_t_W_Gnss);
    trajectoryAlignmentHandlerPtr_->addGnssPose(W_t_W_Gnss, gnssMsgPtr->header.stamp.toSec());
    double initYawEnuLidar;
    if (!trajectoryAlignmentHandlerPtr_->alignTrajectories(initYawEnuLidar)) {
      --gnssCallbackCounter__;
      return;
    }
    // Rotate to IMU frame
    Eigen::Matrix3d R_L_I = staticTransformsPtr_
                                ->lv_T_frame1_frame2(dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getLidarFrame(),
                                                     dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getImuFrame())
                                .block<3, 3>(0, 0);
    Eigen::Vector3d eulerAngles = R_L_I.eulerAngles(2, 1, 0).reverse();
    double initYaw = initYawEnuLidar;  //+ eulerAngles(0);
    gnssHandlerPtr_->setInitYaw(initYaw);
    initialized_ = true;
    gnssHandlerPtr_->initHandler(gnssHandlerPtr_->getInitYaw());
  }

  // Convert to cartesian coordinates
  gnssHandlerPtr_->convertNavSatToPosition(gnssCoord, W_t_W_Gnss);

  // Initialization
  if (gnssCallbackCounter__ == NUM_GNSS_CALLBACKS_UNTIL_START + 2) {
    if (not graph_msf::GraphMsfInterface::initYawAndPosition_(
            gnssHandlerPtr_->getInitYaw(), dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getLidarFrame(), W_t_W_Gnss,
            dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getGnssFrame())) {
      // Decrease counter if not successfully initialized
      --gnssCallbackCounter__;
    }
  } else {
    graph_msf::UnaryMeasurement3D meas_W_t_W_Gnss(
        "Gnss", dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getGnssFrame(), gnssRate_,
        gnssMsgPtr->header.stamp.toSec(), W_t_W_Gnss,
        Eigen::Vector3d(gnssPositionUnaryNoise_, gnssPositionUnaryNoise_, gnssPositionUnaryNoise_));
    // graph_msf::GraphMsfInterface::addGnssPositionMeasurement_(meas_W_t_W_Gnss);
    graph_msf::GraphMsfInterface::addDualGnssPositionMeasurement_(meas_W_t_W_Gnss, W_t_W_Gnss_km1__, estCovarianceXYZ);
  }
  W_t_W_Gnss_km1__ = W_t_W_Gnss;

  /// Add GNSS to Path
  addToPathMsg(measMapGnssPathPtr_, dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getMapFrame(),
               gnssMsgPtr->header.stamp, W_t_W_Gnss, graphConfigPtr_->imuBufferLength * 4);
  /// Publish path
  pubMeasMapGnssPath_.publish(measMapGnssPathPtr_);
}

void AnymalEstimator::publishState_(const double imuTimeK, const Eigen::Matrix4d& T_W_O, const Eigen::Matrix4d& T_O_Ik,
                                    const Eigen::Vector3d& Ic_v_W_Ic, const Eigen::Vector3d& I_w_W_I) {
  // Used transforms
  Eigen::Matrix4d T_I_L = staticTransformsPtr_
                              ->rv_T_frame1_frame2(staticTransformsPtr_.get()->getImuFrame(),
                                                   dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getLidarFrame())
                              .inverse();

  // Pose
  Eigen::Matrix4d T_O_L = T_O_Ik * T_I_L;
  Eigen::Matrix4d T_W_L = T_W_O * T_O_L;

  // Publish odometry message for odom->imu with 100 Hz
  addToOdometryMsg(odomImuMsgPtr_, dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getOdomFrame(),
                   dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getImuFrame(), ros::Time(imuTimeK), T_O_Ik, Ic_v_W_Ic,
                   I_w_W_I);
  pubEstOdomImu_.publish(odomImuMsgPtr_);
  addToPathMsg(estOdomImuPathPtr_, dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getOdomFrame(), ros::Time(imuTimeK),
               T_O_Ik.block<3, 1>(0, 3), graphConfigPtr_->imuBufferLength * 20);
  pubEstOdomImuPath_.publish(estOdomImuPathPtr_);
  // Publish odometry message for map->imu with 100 Hz
  addToOdometryMsg(mapImuMsgPtr_, dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getMapFrame(),
                   dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getImuFrame(), ros::Time(imuTimeK), T_W_O * T_O_Ik,
                   Ic_v_W_Ic, I_w_W_I);
  pubEstMapImu_.publish(mapImuMsgPtr_);
  addToPathMsg(estMapImuPathPtr_, dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getMapFrame(), ros::Time(imuTimeK),
               (T_W_O * T_O_Ik).block<3, 1>(0, 3), graphConfigPtr_->imuBufferLength * 20);
  pubEstMapImuPath_.publish(estMapImuPathPtr_);

  // Convert Measurements

  // Publish to TF
  // W_O
  static tf::Transform transform_W_O;
  transform_W_O.setOrigin(tf::Vector3(T_W_O(0, 3), T_W_O(1, 3), T_W_O(2, 3)));
  Eigen::Quaterniond q_W_O(T_W_O.block<3, 3>(0, 0));
  transform_W_O.setRotation(tf::Quaternion(q_W_O.x(), q_W_O.y(), q_W_O.z(), q_W_O.w()));
  tfBroadcaster_.sendTransform(tf::StampedTransform(transform_W_O, ros::Time(imuTimeK),
                                                    dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getMapFrame(),
                                                    dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getOdomFrame()));
  // I->B
  static tf::StampedTransform transform_I_B;
  tfListener_.waitForTransform(staticTransformsPtr_->getImuFrame(),
                               dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getBaseLinkFrame(), ros::Time(0),
                               ros::Duration(0.1));
  listener_.lookupTransform(staticTransformsPtr_->getImuFrame(),
                            dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getBaseLinkFrame(), ros::Time(0),
                            transform_I_B);

  // Publish T_O_B
  static tf::Transform transform_O_I;
  transform_O_I.setOrigin(tf::Vector3(T_O_Ik(0, 3), T_O_Ik(1, 3), T_O_Ik(2, 3)));
  Eigen::Quaterniond q_O_I(T_O_Ik.block<3, 3>(0, 0));
  transform_O_I.setRotation(tf::Quaternion(q_O_I.x(), q_O_I.y(), q_O_I.z(), q_O_I.w()));
  // tfBroadcaster_.sendTransform(tf::StampedTransform(transform_O_I * transform_I_B, ros::Time(imuTimeK),
  //                                                   dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getOdomFrame(),
  //                                                   dynamic_cast<AnymalStaticTransforms*>(staticTransformsPtr_.get())->getBaseLinkFrame()));
}

}  // namespace anymal_se