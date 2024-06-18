/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "graph_msf_ros/GraphMsfRos.h"

// ROS
#include <tf/tf.h>

// Workspace
#include "graph_msf_ros/constants.h"
#include "graph_msf_ros/util/conversions.h"

namespace graph_msf {

GraphMsfRos::GraphMsfRos(const std::shared_ptr<ros::NodeHandle>& privateNodePtr) : GraphMsf(), privateNode_(*privateNodePtr) {
  REGULAR_COUT << GREEN_START << " Initializing." << COLOR_END << std::endl;

  // Configurations ----------------------------
  graphConfigPtr_ = std::make_shared<GraphConfig>();
  staticTransformsPtr_ = std::make_shared<StaticTransforms>();

  // Wrap up
  REGULAR_COUT << GREEN_START << " Initialized." << COLOR_END << std::endl;
}

bool GraphMsfRos::setup() {
  REGULAR_COUT << GREEN_START << " Setting up." << COLOR_END << std::endl;

  // Read parameters ----------------------------
  GraphMsfRos::readParams_(privateNode_);

  // Super class
  if (not graph_msf::GraphMsf::setup()) {
    throw std::runtime_error("GraphMsfRos could not be initialized");
  }

  // Publishers ----------------------------
  GraphMsfRos::initializePublishers_(privateNode_);

  // Subscribers ----------------------------
  GraphMsfRos::initializeSubscribers_(privateNode_);

  // Messages ----------------------------
  GraphMsfRos::initializeMessages_(privateNode_);

  // Services ----------------------------
  GraphMsfRos::initializeServices_(privateNode_);

  // Time
  startTime_ = std::chrono::high_resolution_clock::now();

  // Wrap up
  REGULAR_COUT << GREEN_START << " Set up successfully." << COLOR_END << std::endl;

  return true;
}

void GraphMsfRos::initializePublishers_(ros::NodeHandle& privateNode) {
  REGULAR_COUT << GREEN_START << " Initializing publishers." << COLOR_END << std::endl;
  // Odometry
  pubEstOdomImu_ = privateNode.advertise<nav_msgs::Odometry>("/graph_msf/est_odometry_odom_imu", ROS_QUEUE_SIZE);
  pubEstMapImu_ = privateNode.advertise<nav_msgs::Odometry>("/graph_msf/est_odometry_map_imu", ROS_QUEUE_SIZE);
  pubEstWorldImu_ = privateNode.advertise<nav_msgs::Odometry>("/graph_msf/est_odometry_world_imu", ROS_QUEUE_SIZE);
  pubOptWorldImu_ = privateNode.advertise<nav_msgs::Odometry>("/graph_msf/opt_odometry_world_imu", ROS_QUEUE_SIZE);
  // Vector3 Variances
  pubEstWorldPosVariance_ = privateNode.advertise<geometry_msgs::Vector3Stamped>("/graph_msf/est_world_pos_variance", ROS_QUEUE_SIZE);
  pubEstWorldRotVariance_ = privateNode.advertise<geometry_msgs::Vector3Stamped>("/graph_msf/est_world_rot_variance", ROS_QUEUE_SIZE);
  // Paths
  pubEstOdomImuPath_ = privateNode.advertise<nav_msgs::Path>("/graph_msf/est_path_odom_imu", ROS_QUEUE_SIZE);
  pubEstWorldImuPath_ = privateNode.advertise<nav_msgs::Path>("/graph_msf/est_path_world_imu", ROS_QUEUE_SIZE);
  pubOptWorldImuPath_ = privateNode.advertise<nav_msgs::Path>("/graph_msf/opt_path_world_imu", ROS_QUEUE_SIZE);
  // Imu Bias
  pubAccelBias_ = privateNode.advertise<geometry_msgs::Vector3Stamped>("/graph_msf/accel_bias", ROS_QUEUE_SIZE);
  pubGyroBias_ = privateNode.advertise<geometry_msgs::Vector3Stamped>("/graph_msf/gyro_bias", ROS_QUEUE_SIZE);
  // Added Imu Measurements
  pubAddedImuMeas_ = privateNode.advertise<sensor_msgs::Imu>("/graph_msf/added_imu_meas", ROS_QUEUE_SIZE);
}

void GraphMsfRos::initializeSubscribers_(ros::NodeHandle& privateNode) {
  REGULAR_COUT << GREEN_START << " Initializing subscribers." << COLOR_END << std::endl;
  // Imu
  subImu_ = privateNode.subscribe<sensor_msgs::Imu>("/imu_topic", ROS_QUEUE_SIZE, &GraphMsfRos::imuCallback_, this,
                                                    ros::TransportHints().tcpNoDelay());
  std::cout << YELLOW_START << "GraphMsfRos" << COLOR_END << " Initialized IMU cabin subscriber with topic: " << subImu_.getTopic()
            << std::endl;
}

void GraphMsfRos::initializeMessages_(ros::NodeHandle& privateNode) {
  REGULAR_COUT << GREEN_START << " Initializing messages." << COLOR_END << std::endl;
  // Odometry
  estOdomImuMsgPtr_ = boost::make_shared<nav_msgs::Odometry>();
  estMapImuMsgPtr_ = boost::make_shared<nav_msgs::Odometry>();
  estWorldImuMsgPtr_ = boost::make_shared<nav_msgs::Odometry>();
  optWorldImuMsgPtr_ = boost::make_shared<nav_msgs::Odometry>();
  // Vector3 Variances
  estWorldPosVarianceMsgPtr_ = boost::make_shared<geometry_msgs::Vector3Stamped>();
  estWorldRotVarianceMsgPtr_ = boost::make_shared<geometry_msgs::Vector3Stamped>();
  // Path
  estOdomImuPathPtr_ = boost::make_shared<nav_msgs::Path>();
  estWorldImuPathPtr_ = boost::make_shared<nav_msgs::Path>();
  optWorldImuPathPtr_ = boost::make_shared<nav_msgs::Path>();
  // Imu Bias
  accelBiasMsgPtr_ = boost::make_shared<geometry_msgs::Vector3Stamped>();
  gyroBiasMsgPtr_ = boost::make_shared<geometry_msgs::Vector3Stamped>();
}

void GraphMsfRos::initializeServices_(ros::NodeHandle& privateNode) {
  // Trigger offline smoother optimization
  srvSmootherOptimize_ =
      privateNode.advertiseService("/graph_msf/trigger_offline_optimization", &GraphMsfRos::srvOfflineSmootherOptimizeCallback_, this);
}

void GraphMsfRos::imuCallback_(const sensor_msgs::Imu::ConstPtr& imuMsgPtr) {
  // Convert to Eigen
  Eigen::Vector3d linearAcc(imuMsgPtr->linear_acceleration.x, imuMsgPtr->linear_acceleration.y, imuMsgPtr->linear_acceleration.z);
  Eigen::Vector3d angularVel(imuMsgPtr->angular_velocity.x, imuMsgPtr->angular_velocity.y, imuMsgPtr->angular_velocity.z);
  Eigen::Matrix<double, 6, 1> addedImuMeasurements;  // accel, gyro

  // Create pointer for carrying state
  std::shared_ptr<SafeIntegratedNavState> preIntegratedNavStatePtr = nullptr;
  std::shared_ptr<SafeNavStateWithCovarianceAndBias> optimizedStateWithCovarianceAndBiasPtr = nullptr;

  // Add measurement and get state
  if (this->addImuMeasurementAndGetState(linearAcc, angularVel, imuMsgPtr->header.stamp.toSec(), preIntegratedNavStatePtr,
                                         optimizedStateWithCovarianceAndBiasPtr, addedImuMeasurements)) {
    // Encountered Delay
    if (ros::Time::now() - ros::Time(preIntegratedNavStatePtr->getTimeK()) > ros::Duration(0.5)) {
      std::cout << RED_START << "GraphMsfRos" << COLOR_END << " Encountered delay of "
                << (ros::Time::now() - ros::Time(preIntegratedNavStatePtr->getTimeK())).toSec() << " seconds." << std::endl;
    }

    // Publish Odometry
    this->publishState_(preIntegratedNavStatePtr, optimizedStateWithCovarianceAndBiasPtr);

    // Publish Filtered Imu Measurements
    //    this->publishAddedImuMeas_(addedImuMeasurements, imuMsgPtr->header.stamp);
  } else if (isGraphInited()) {
    REGULAR_COUT << RED_START << " Could not add IMU measurement." << COLOR_END << std::endl;
  }
}

bool GraphMsfRos::srvOfflineSmootherOptimizeCallback_(graph_msf_ros::OfflineOptimizationTrigger::Request& req,
                                                      graph_msf_ros::OfflineOptimizationTrigger::Response& res) {
  // Max Iterations from service call
  int maxIterations = req.max_optimization_iterations;

  // Trigger offline smoother optimization and create response
  if (GraphMsf::optimizeSlowBatchSmoother(maxIterations, optimizationResultLoggingPath_)) {
    res.success = true;
    res.message = "Optimization successful.";
  } else {
    res.success = false;
    res.message = "Optimization failed.";
  }
  return true;
}

void GraphMsfRos::addToPathMsg(const nav_msgs::PathPtr& pathPtr, const std::string& fixedFrameName, const ros::Time& stamp,
                               const Eigen::Vector3d& t, const int maxBufferLength) {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = fixedFrameName;
  pose.header.stamp = stamp;
  pose.pose.position.x = t(0);
  pose.pose.position.y = t(1);
  pose.pose.position.z = t(2);
  pathPtr->header.frame_id = fixedFrameName;
  pathPtr->header.stamp = stamp;
  pathPtr->poses.push_back(pose);
  if (pathPtr->poses.size() > maxBufferLength) {
    pathPtr->poses.erase(pathPtr->poses.begin());
  }
}

void GraphMsfRos::addToOdometryMsg(const nav_msgs::OdometryPtr& msgPtr, const std::string& fixedFrame, const std::string& movingFrame,
                                   const ros::Time& stamp, const Eigen::Isometry3d& T, const Eigen::Vector3d& F_v_W_F,
                                   const Eigen::Vector3d& F_w_W_F, const Eigen::Matrix<double, 6, 6>& poseCovariance,
                                   const Eigen::Matrix<double, 6, 6>& twistCovariance) {
  msgPtr->header.frame_id = fixedFrame;
  msgPtr->child_frame_id = movingFrame;
  msgPtr->header.stamp = stamp;
  tf::poseTFToMsg(isometry3ToTf(T), msgPtr->pose.pose);
  msgPtr->twist.twist.linear.x = F_v_W_F(0);
  msgPtr->twist.twist.linear.y = F_v_W_F(1);
  msgPtr->twist.twist.linear.z = F_v_W_F(2);
  msgPtr->twist.twist.angular.x = F_w_W_F(0);
  msgPtr->twist.twist.angular.y = F_w_W_F(1);
  msgPtr->twist.twist.angular.z = F_w_W_F(2);

  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      msgPtr->pose.covariance[6 * i + j] = poseCovariance(i, j);
      msgPtr->twist.covariance[6 * i + j] = twistCovariance(i, j);
    }
  }
}

void GraphMsfRos::addToPoseWithCovarianceStampedMsg(const geometry_msgs::PoseWithCovarianceStampedPtr& msgPtr, const std::string& frameName,
                                                    const ros::Time& stamp, const Eigen::Isometry3d& T,
                                                    const Eigen::Matrix<double, 6, 6>& transformCovariance) {
  msgPtr->header.frame_id = frameName;
  msgPtr->header.stamp = stamp;
  tf::poseTFToMsg(isometry3ToTf(T), msgPtr->pose.pose);
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      msgPtr->pose.covariance[6 * i + j] = transformCovariance(i, j);
    }
  }
}

void GraphMsfRos::extractCovariancesFromOptimizedState(
    Eigen::Matrix<double, 6, 6>& poseCovarianceRos, Eigen::Matrix<double, 6, 6>& twistCovarianceRos,
    const std::shared_ptr<graph_msf::SafeNavStateWithCovarianceAndBias>& optimizedStateWithCovarianceAndBiasPtr) {
  // Extract covariances from optimized state
  if (optimizedStateWithCovarianceAndBiasPtr != nullptr) {
    poseCovarianceRos =
        graph_msf::convertCovarianceGtsamConventionToRosConvention(optimizedStateWithCovarianceAndBiasPtr->getPoseCovariance());
    twistCovarianceRos.block<3, 3>(0, 0) = optimizedStateWithCovarianceAndBiasPtr->getVelocityCovariance();
  } else {
    poseCovarianceRos.setZero();
    twistCovarianceRos.setZero();
  }
}

long GraphMsfRos::secondsSinceStart_() {
  currentTime_ = std::chrono::high_resolution_clock::now();
  return std::chrono::duration_cast<std::chrono::seconds>(currentTime_ - startTime_).count();
}

// Publish state ---------------------------------------------------------------
// Higher Level Functions
void GraphMsfRos::publishState_(
    const std::shared_ptr<graph_msf::SafeIntegratedNavState>& integratedNavStatePtr,
    const std::shared_ptr<graph_msf::SafeNavStateWithCovarianceAndBias>& optimizedStateWithCovarianceAndBiasPtr) {
  // Covariances
  Eigen::Matrix<double, 6, 6> poseCovarianceRos, twistCovarianceRos;
  extractCovariancesFromOptimizedState(poseCovarianceRos, twistCovarianceRos, optimizedStateWithCovarianceAndBiasPtr);

  // Variances (only digonal elements
  Eigen::Vector3d positionVarianceRos = poseCovarianceRos.block<3, 3>(0, 0).diagonal();
  Eigen::Vector3d orientationVarianceRos = poseCovarianceRos.block<3, 3>(3, 3).diagonal();

  // Alias
  const Eigen::Isometry3d& T_O_Ik = integratedNavStatePtr->getT_O_Ik_gravityAligned();  // alias
  const double& timeK = integratedNavStatePtr->getTimeK();                              // alias

  // Publish non-time critical data in a separate thread
  std::thread publishNonTimeCriticalDataThread(&GraphMsfRos::publishNonTimeCriticalData_, this, T_O_Ik, timeK, poseCovarianceRos,
                                               twistCovarianceRos, positionVarianceRos, orientationVarianceRos, integratedNavStatePtr,
                                               optimizedStateWithCovarianceAndBiasPtr);
  publishNonTimeCriticalDataThread.detach();
}

// Copy the arguments in order to be thread safe
void GraphMsfRos::publishNonTimeCriticalData_(
    const Eigen::Isometry3d T_O_Ik, const double timeK, const Eigen::Matrix<double, 6, 6> poseCovarianceRos,
    const Eigen::Matrix<double, 6, 6> twistCovarianceRos, const Eigen::Vector3d positionVarianceRos,
    const Eigen::Vector3d orientationVarianceRos, const std::shared_ptr<const graph_msf::SafeIntegratedNavState> integratedNavStatePtr,
    const std::shared_ptr<const graph_msf::SafeNavStateWithCovarianceAndBias> optimizedStateWithCovarianceAndBiasPtr) {
  // Mutex for not overloading ROS
  std::lock_guard<std::mutex> lock(rosPublisherMutex_);

  // Odometry messages
  publishImuOdoms_(integratedNavStatePtr, poseCovarianceRos, twistCovarianceRos);

  // Publish to TF
  // B_O
  Eigen::Isometry3d T_B_Ok =
      staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getBaseLinkFrame(), staticTransformsPtr_->getImuFrame()) *
      T_O_Ik.inverse();
  publishTfTreeTransform_(staticTransformsPtr_->getBaseLinkFrame(), staticTransformsPtr_->getOdomFrame(), timeK, T_B_Ok);
  // O_W
  Eigen::Isometry3d T_O_W = integratedNavStatePtr->getT_W_O().inverse();
  publishTfTreeTransform_(staticTransformsPtr_->getOdomFrame(), staticTransformsPtr_->getWorldFrame(), timeK, T_O_W);

  // Publish Variances
  publishDiagVarianceVectors(positionVarianceRos, orientationVarianceRos, timeK);

  // Publish paths
  publishImuPaths_(integratedNavStatePtr);

  // Optimized estimate ----------------------
  publishOptimizedStateAndBias_(optimizedStateWithCovarianceAndBiasPtr, poseCovarianceRos, twistCovarianceRos);
}

void GraphMsfRos::publishOptimizedStateAndBias_(
    const std::shared_ptr<const graph_msf::SafeNavStateWithCovarianceAndBias> optimizedStateWithCovarianceAndBiasPtr,
    const Eigen::Matrix<double, 6, 6>& poseCovarianceRos, const Eigen::Matrix<double, 6, 6>& twistCovarianceRos) {
  if (optimizedStateWithCovarianceAndBiasPtr != nullptr &&
      optimizedStateWithCovarianceAndBiasPtr->getTimeK() - lastOptimizedStateTimestamp_ > 1e-03) {
    // Time of this optimized state
    lastOptimizedStateTimestamp_ = optimizedStateWithCovarianceAndBiasPtr->getTimeK();

    // Odometry messages
    // world->imu
    if (pubOptWorldImu_.getNumSubscribers() > 0) {
      addToOdometryMsg(optWorldImuMsgPtr_, staticTransformsPtr_->getWorldFrame(), staticTransformsPtr_->getImuFrame(),
                       ros::Time(optimizedStateWithCovarianceAndBiasPtr->getTimeK()), optimizedStateWithCovarianceAndBiasPtr->getT_W_Ik(),
                       optimizedStateWithCovarianceAndBiasPtr->getI_v_W_I(), optimizedStateWithCovarianceAndBiasPtr->getI_w_W_I(),
                       poseCovarianceRos, twistCovarianceRos);
      pubOptWorldImu_.publish(optWorldImuMsgPtr_);
    }

    // Path
    // world->imu
    addToPathMsg(optWorldImuPathPtr_, staticTransformsPtr_->getWorldFrame(), ros::Time(optimizedStateWithCovarianceAndBiasPtr->getTimeK()),
                 optimizedStateWithCovarianceAndBiasPtr->getT_W_Ik().translation(), graphConfigPtr_->imuBufferLength_ * 20);
    if (pubOptWorldImuPath_.getNumSubscribers() > 0) {
      pubOptWorldImuPath_.publish(optWorldImuPathPtr_);
    }

    // Biases
    // Publish accel bias
    if (pubAccelBias_.getNumSubscribers() > 0) {
      Eigen::Vector3d accelBias = optimizedStateWithCovarianceAndBiasPtr->getAccelerometerBias();
      accelBiasMsgPtr_->header.stamp = ros::Time(optimizedStateWithCovarianceAndBiasPtr->getTimeK());
      accelBiasMsgPtr_->header.frame_id = staticTransformsPtr_->getImuFrame();
      accelBiasMsgPtr_->vector.x = accelBias(0);
      accelBiasMsgPtr_->vector.y = accelBias(1);
      accelBiasMsgPtr_->vector.z = accelBias(2);
      pubAccelBias_.publish(accelBiasMsgPtr_);
    }
    // Publish gyro bias
    if (pubGyroBias_.getNumSubscribers() > 0) {
      Eigen::Vector3d gyroBias = optimizedStateWithCovarianceAndBiasPtr->getGyroscopeBias();
      gyroBiasMsgPtr_->header.stamp = ros::Time(optimizedStateWithCovarianceAndBiasPtr->getTimeK());
      gyroBiasMsgPtr_->header.frame_id = staticTransformsPtr_->getImuFrame();
      gyroBiasMsgPtr_->vector.x = gyroBias(0);
      gyroBiasMsgPtr_->vector.y = gyroBias(1);
      gyroBiasMsgPtr_->vector.z = gyroBias(2);
      pubGyroBias_.publish(gyroBiasMsgPtr_);
    }

    // TFs in Optimized State
    for (const auto& transformIterator : optimizedStateWithCovarianceAndBiasPtr->getFixedFrameTransforms().getTransformsMap()) {
      // Case 1: Holistic transformation --> includes world frame
      if (transformIterator.first.second == staticTransformsPtr_->getWorldFrame()) {
        // A. Get transform
        const Eigen::Isometry3d& T_M_W = transformIterator.second;
        const Eigen::Isometry3d T_M_Ik = T_M_W * optimizedStateWithCovarianceAndBiasPtr->getT_W_Ik();
        const std::string& mapFrameName = transformIterator.first.first;
        const std::string& worldFrameName = transformIterator.first.second;
        if (graphConfigPtr_->verboseLevel_ >= 2) {
          std::cout << "Transformation from " << mapFrameName << " to " << worldFrameName << std::endl;
          std::cout << "Uncertainty: " << std::endl
                    << optimizedStateWithCovarianceAndBiasPtr->getFixedFrameTransformsCovariance().rv_T_frame1_frame2(mapFrameName,
                                                                                                                      worldFrameName)
                    << std::endl;
        }
        // B. Publish Odometry for Map->imu
        if (pubEstMapImu_.getNumSubscribers() > 0) {
          addToOdometryMsg(estMapImuMsgPtr_, mapFrameName + fixedFrameAlignedNameId_, staticTransformsPtr_->getImuFrame(),
                           ros::Time(optimizedStateWithCovarianceAndBiasPtr->getTimeK()), T_M_Ik,
                           optimizedStateWithCovarianceAndBiasPtr->getI_v_W_I(), optimizedStateWithCovarianceAndBiasPtr->getI_w_W_I(),
                           poseCovarianceRos, twistCovarianceRos);
          pubEstMapImu_.publish(estMapImuMsgPtr_);
        }
        // C. Publish TransformStamped for Aligned Frames
        std::string transformTopic = "/graph_msf/transform_" + worldFrameName + "_to_" + mapFrameName + fixedFrameAlignedNameId_;
        geometry_msgs::PoseWithCovarianceStampedPtr poseWithCovarianceStampedMsgPtr =
            boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();
        addToPoseWithCovarianceStampedMsg(
            poseWithCovarianceStampedMsgPtr, staticTransformsPtr_->getWorldFrame(),
            ros::Time(optimizedStateWithCovarianceAndBiasPtr->getTimeK()), T_M_W.inverse(),
            optimizedStateWithCovarianceAndBiasPtr->getFixedFrameTransformsCovariance().rv_T_frame1_frame2(mapFrameName, worldFrameName));
        // Check whether publisher already exists
        if (pubPoseStampedByTopicMap_.find(transformTopic) == pubPoseStampedByTopicMap_.end()) {
          pubPoseStampedByTopicMap_[transformTopic] = privateNode_.advertise<geometry_msgs::PoseWithCovarianceStamped>(transformTopic, 1);
          REGULAR_COUT << GREEN_START << " Initialized publisher for " << transformTopic << COLOR_END << std::endl;
        }
        if (pubPoseStampedByTopicMap_[transformTopic].getNumSubscribers() > 0) {
          pubPoseStampedByTopicMap_[transformTopic].publish(poseWithCovarianceStampedMsgPtr);
        }

        // D. Publish TF Tree --> everything children of world
        publishTfTreeTransform_(worldFrameName, mapFrameName + fixedFrameAlignedNameId_, optimizedStateWithCovarianceAndBiasPtr->getTimeK(),
                                T_M_W.inverse());

      } else {  // Case 2: Calibration transformation --> does not include world frame
        const std::string& sensorFrameName = transformIterator.first.first;
        const std::string& sensorFrameNameCorrected = sensorFrameName + sensorFrameCorrectedNameId_;  // transformIterator.first.second;
        const Eigen::Isometry3d& T_sensor_sensorCorrected = transformIterator.second;
        // A. Publish TransformStamped for Corrected Frames
        std::string transformTopic = "/graph_msf/transform_" + sensorFrameName + "_to_" + sensorFrameNameCorrected;
        geometry_msgs::PoseWithCovarianceStampedPtr poseWithCovarianceStampedMsgPtr =
            boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();
        addToPoseWithCovarianceStampedMsg(poseWithCovarianceStampedMsgPtr, sensorFrameName,
                                          ros::Time(optimizedStateWithCovarianceAndBiasPtr->getTimeK()), T_sensor_sensorCorrected,
                                          optimizedStateWithCovarianceAndBiasPtr->getFixedFrameTransformsCovariance().rv_T_frame1_frame2(
                                              sensorFrameName, transformIterator.first.second));
        // Check whether publisher already exists
        if (pubPoseStampedByTopicMap_.find(transformTopic) == pubPoseStampedByTopicMap_.end()) {
          pubPoseStampedByTopicMap_[transformTopic] = privateNode_.advertise<geometry_msgs::PoseWithCovarianceStamped>(transformTopic, 1);
          REGULAR_COUT << GREEN_START << " Initialized publisher for " << transformTopic << COLOR_END << std::endl;
        }
        if (pubPoseStampedByTopicMap_[transformTopic].getNumSubscribers() > 0) {
          pubPoseStampedByTopicMap_[transformTopic].publish(poseWithCovarianceStampedMsgPtr);
        }

        // D. Publish TF Tree --> as children of sensor frame
        publishTfTreeTransform_(sensorFrameName, sensorFrameNameCorrected, optimizedStateWithCovarianceAndBiasPtr->getTimeK(),
                                T_sensor_sensorCorrected);
      }
    }
  }
}

// Lower Level Functions
void GraphMsfRos::publishTfTreeTransform_(const std::string& parentFrameName, const std::string& childFrameName, const double timeStamp,
                                          const Eigen::Isometry3d& T_frame_childFrame) {
  tf::Transform transform_frame_childFrame = isometry3ToTf(T_frame_childFrame);
  tfBroadcaster_.sendTransform(tf::StampedTransform(transform_frame_childFrame, ros::Time(timeStamp), parentFrameName, childFrameName));
}

void GraphMsfRos::publishImuOdoms_(const std::shared_ptr<const graph_msf::SafeIntegratedNavState>& preIntegratedNavStatePtr,
                                   const Eigen::Matrix<double, 6, 6>& poseCovarianceRos,
                                   const Eigen::Matrix<double, 6, 6>& twistCovarianceRos) const {
  // Odom->imu with 100 Hz
  if (pubEstOdomImu_.getNumSubscribers() > 0) {
    addToOdometryMsg(estOdomImuMsgPtr_, staticTransformsPtr_->getOdomFrame(), staticTransformsPtr_->getImuFrame(),
                     ros::Time(preIntegratedNavStatePtr->getTimeK()), preIntegratedNavStatePtr->getT_O_Ik_gravityAligned(),
                     preIntegratedNavStatePtr->getI_v_W_I(), preIntegratedNavStatePtr->getI_w_W_I(), poseCovarianceRos, twistCovarianceRos);
    pubEstOdomImu_.publish(estOdomImuMsgPtr_);
  }
  // World->imu with 100 Hz
  if (pubEstWorldImu_.getNumSubscribers() > 0) {
    addToOdometryMsg(estWorldImuMsgPtr_, staticTransformsPtr_->getWorldFrame(), staticTransformsPtr_->getImuFrame(),
                     ros::Time(preIntegratedNavStatePtr->getTimeK()), preIntegratedNavStatePtr->getT_W_Ik(),
                     preIntegratedNavStatePtr->getI_v_W_I(), preIntegratedNavStatePtr->getI_w_W_I(), poseCovarianceRos, twistCovarianceRos);
    pubEstWorldImu_.publish(estWorldImuMsgPtr_);
  }
}

void GraphMsfRos::publishDiagVarianceVectors(const Eigen::Vector3d& posVarianceRos, const Eigen::Vector3d& rotVarianceRos,
                                             const double timeStamp) const {
  // World Position Variance
  if (pubEstWorldPosVariance_.getNumSubscribers() > 0) {
    estWorldPosVarianceMsgPtr_->header.stamp = ros::Time(timeStamp);
    estWorldPosVarianceMsgPtr_->header.frame_id = staticTransformsPtr_->getWorldFrame();
    estWorldPosVarianceMsgPtr_->vector.x = posVarianceRos(0);
    estWorldPosVarianceMsgPtr_->vector.y = posVarianceRos(1);
    estWorldPosVarianceMsgPtr_->vector.z = posVarianceRos(2);
    pubEstWorldPosVariance_.publish(estWorldPosVarianceMsgPtr_);
  }
  // World Rotation Variance
  if (pubEstWorldRotVariance_.getNumSubscribers() > 0) {
    estWorldRotVarianceMsgPtr_->header.stamp = ros::Time(timeStamp);
    estWorldRotVarianceMsgPtr_->header.frame_id = staticTransformsPtr_->getWorldFrame();
    estWorldRotVarianceMsgPtr_->vector.x = rotVarianceRos(0);
    estWorldRotVarianceMsgPtr_->vector.y = rotVarianceRos(1);
    estWorldRotVarianceMsgPtr_->vector.z = rotVarianceRos(2);
    pubEstWorldRotVariance_.publish(estWorldRotVarianceMsgPtr_);
  }
}

void GraphMsfRos::publishImuPaths_(const std::shared_ptr<const graph_msf::SafeIntegratedNavState>& navStatePtr) const {
  // odom->imu
  addToPathMsg(estOdomImuPathPtr_, staticTransformsPtr_->getOdomFrame(), ros::Time(navStatePtr->getTimeK()),
               navStatePtr->getT_O_Ik_gravityAligned().translation(), graphConfigPtr_->imuBufferLength_ * 20);
  if (pubEstOdomImuPath_.getNumSubscribers() > 0) {
    pubEstOdomImuPath_.publish(estOdomImuPathPtr_);
  }
  // world->imu
  addToPathMsg(estWorldImuPathPtr_, staticTransformsPtr_->getWorldFrame(), ros::Time(navStatePtr->getTimeK()),
               (navStatePtr->getT_W_Ik()).translation(), graphConfigPtr_->imuBufferLength_ * 20);
  if (pubEstWorldImuPath_.getNumSubscribers() > 0) {
    pubEstWorldImuPath_.publish(estWorldImuPathPtr_);
  }
}

void GraphMsfRos::publishAddedImuMeas_(const Eigen::Matrix<double, 6, 1>& addedImuMeas, const ros::Time& stamp) const {
  // Publish added imu measurement
  if (pubAddedImuMeas_.getNumSubscribers() > 0) {
    sensor_msgs::Imu addedImuMeasMsg;
    addedImuMeasMsg.header.stamp = stamp;
    addedImuMeasMsg.header.frame_id = staticTransformsPtr_->getImuFrame();
    addedImuMeasMsg.linear_acceleration.x = addedImuMeas(0);
    addedImuMeasMsg.linear_acceleration.y = addedImuMeas(1);
    addedImuMeasMsg.linear_acceleration.z = addedImuMeas(2);
    addedImuMeasMsg.angular_velocity.x = addedImuMeas(3);
    addedImuMeasMsg.angular_velocity.y = addedImuMeas(4);
    addedImuMeasMsg.angular_velocity.z = addedImuMeas(5);
    pubAddedImuMeas_.publish(addedImuMeasMsg);
  }
}

}  // namespace graph_msf
