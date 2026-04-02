/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "anymal_estimator_graph/AnymalEstimator.h"

// Project
#include "anymal_estimator_graph/AnymalStaticTransforms.h"

// Workspace
#include "anymal_estimator_graph/constants.h"
#include "graph_msf/interface/eigen_wrapped_gtsam_utils.h"
#include "graph_msf/interface/input_output.h"
#include "graph_msf/measurements/BinaryMeasurementXD.h"
#include "graph_msf/measurements/UnaryMeasurementXD.h"
#include "graph_msf/measurements/UnaryMeasurementXDAbsolute.h"
#include "graph_msf_ros/util/conversions.h"

// std
#include <cmath>
#include <iomanip>

namespace anymal_se {

// Constexpr from header
constexpr std::array<const char*, 4> AnymalEstimator::legNames_;

int AnymalEstimator::legIndexFromName_(const std::string& legName) {
  for (std::size_t legIndex = 0; legIndex < legNames_.size(); ++legIndex) {
    if (legName == legNames_[legIndex]) {
      return static_cast<int>(legIndex);
    }
  }
  return -1;
}

bool AnymalEstimator::getClosestLioPose_(double t, Eigen::Isometry3d& T_M_B_out, double* best_dt_out) const {
  std::lock_guard<std::mutex> lioPoseBufLock(lioPoseBufMutex_);
  if (lioPoseBuf_.empty()) {
    if (best_dt_out) {
      *best_dt_out = std::numeric_limits<double>::infinity();
    }
    return false;
  }

  auto bestIt = lioPoseBuf_.begin();
  double bestDt = std::abs(bestIt->first - t);

  for (auto it = lioPoseBuf_.begin(); it != lioPoseBuf_.end(); ++it) {
    const double dt = std::abs(it->first - t);
    if (dt < bestDt) {
      bestDt = dt;
      bestIt = it;
    }
  }

  if (best_dt_out) {
    *best_dt_out = bestDt;
  }
  if (bestDt > kInitSyncMaxDt) {
    return false;
  }

  T_M_B_out = bestIt->second;
  return true;
}

AnymalEstimator::AnymalEstimator(const std::shared_ptr<ros::NodeHandle>& privateNodePtr) : graph_msf::GraphMsfRos(privateNodePtr) {
  REGULAR_COUT << GREEN_START << " AnymalEstimatorGraph-Constructor called." << COLOR_END << std::endl;

  // Configurations ----------------------------
  // Static transforms
  anymalStaticTransformsPtr_ = std::make_shared<AnymalStaticTransforms>(privateNodePtr);
  staticTransformsPtr_ = anymalStaticTransformsPtr_;

  // Set up
  AnymalEstimator::setup();
}

void AnymalEstimator::setup() {
  REGULAR_COUT << GREEN_START << " AnymalEstimator-Setup called." << COLOR_END << std::endl;
  GraphMsfRos::setup(staticTransformsPtr_);
}

void AnymalEstimator::cacheFrames() {
  GraphMsfRos::cacheFrames();

  if (!anymalStaticTransformsPtr_) {
    throw std::runtime_error("AnymalEstimator: typed static transforms are not initialized.");
  }

  worldFrame_ = staticTransformsPtr_->getWorldFrame();
  baseLinkFrame_ = anymalStaticTransformsPtr_->getBaseLinkFrame();
  gnssFrame_ = anymalStaticTransformsPtr_->getGnssFrame();
  lioOdometryFrame_ = anymalStaticTransformsPtr_->getLioOdometryFrame();
  leggedOdometryFrame_ = anymalStaticTransformsPtr_->getLeggedOdometryFrame();
  vioOdometryFrame_ = anymalStaticTransformsPtr_->getVioOdometryFrame();

  // Cache the static GNSS lever arm once. The callbacks only need the translation, and the
  // transform is immutable after TF resolution.
  if (useGnssUnaryFlag_ && !gnssFrame_.empty()) {
    t_B_G_cached_ = staticTransformsPtr_->rv_T_frame1_frame2(baseLinkFrame_, gnssFrame_).translation();
  }
}

void AnymalEstimator::initializePublishers(ros::NodeHandle& privateNode) {
  graph_msf::GraphMsfRos::initializePublishers(privateNode);

  // Paths
  pubMeasMapLioPath_ = privateNode.advertise<nav_msgs::Path>("/graph_msf/measLiDAR_path_map_lidar", ROS_QUEUE_SIZE);
  pubMeasWorldGnssPath_ = privateNode.advertise<nav_msgs::Path>("/graph_msf/measGnss_path_world_gnss", ROS_QUEUE_SIZE);
  pubMeasMapVioPath_ = privateNode.advertise<nav_msgs::Path>("/graph_msf/measVIO_path_map_vio", ROS_QUEUE_SIZE);
  pubMeasMapVioBetweenPath_ = privateNode.advertise<nav_msgs::Path>("/graph_msf/measVIO_Between_path_map_vio", ROS_QUEUE_SIZE);

  if (useGnssUnaryFlag_) {
    pubGnssPoseWithCov_ =
        privateNode.advertise<geometry_msgs::PoseWithCovarianceStamped>("/graph_msf/gnss_pose_with_covariance", ROS_QUEUE_SIZE);
    // Latch the status and reference topics so late subscribers receive the latest initialization state.
    pubStatus_ = privateNode.advertise<std_msgs::Bool>("/graph_msf/alignment_status", 1, true);
    pubReferenceNavSatFixCoordinates_ =
        privateNode.advertise<sensor_msgs::NavSatFix>("/graph_msf/reference_gnss_position", 1, true);
    pubReferenceNavSatFixCoordinatesENU_ =
        privateNode.advertise<sensor_msgs::NavSatFix>("/graph_msf/reference_gnss_position_enu", 1, true);
  }

  // Markers
  pubFootContactMarkers_ = privateNode.advertise<visualization_msgs::MarkerArray>("/graph_msf/foot_contact_markers", ROS_QUEUE_SIZE);
}

void AnymalEstimator::initializeSubscribers(ros::NodeHandle& privateNode) {
  graph_msf::GraphMsfRos::initializeSubscribers(privateNode);

  // GNSS
  if (useGnssUnaryFlag_) {
    subGnssUnary_ = privateNode.subscribe<sensor_msgs::NavSatFix>("/gnss_topic", ROS_QUEUE_SIZE, &AnymalEstimator::gnssUnaryCallback_, this,
                                                                  ros::TransportHints().tcpNoDelay());
    REGULAR_COUT << " Initialized Gnss subscriber with topic: " << subGnssUnary_.getTopic() << std::endl;
  }

  // LiDAR Odometry
  // Unary
  if (useLioUnaryFlag_) {
    subLioUnary_ = privateNode.subscribe<nav_msgs::Odometry>("/lidar_odometry_topic", ROS_QUEUE_SIZE, &AnymalEstimator::lidarUnaryCallback_,
                                                             this, ros::TransportHints().tcpNoDelay());

    REGULAR_COUT << COLOR_END << " Initialized LiDAR Unary Factor Odometry subscriber with topic: " << subLioUnary_.getTopic() << std::endl;
  }
  // Between
  if (useLioBetweenFlag_) {
    subLioBetween_ = privateNode.subscribe<nav_msgs::Odometry>(
        "/lidar_odometry_topic", ROS_QUEUE_SIZE, &AnymalEstimator::lidarBetweenCallback_, this, ros::TransportHints().tcpNoDelay());
    REGULAR_COUT << COLOR_END << " Initialized LiDAR Odometry subscriber with topic: " << subLioBetween_.getTopic() << std::endl;
  }

  // VIO
  if (useVioOdometryFlag_) {
    subVioOdometry_ = privateNode.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
        "/vio_odometry_topic", ROS_QUEUE_SIZE, &AnymalEstimator::vioUnaryCallback_, this, ros::TransportHints().tcpNoDelay());
    REGULAR_COUT << COLOR_END << " Initialized VIO Odometry subscriber with topic: " << subVioOdometry_.getTopic() << std::endl;
  }
  if (useVioOdometryBetweenFlag_) {
    subVioBetween_ = privateNode.subscribe<nav_msgs::Odometry>(
        "/vio_odometry_between_topic", ROS_QUEUE_SIZE, &AnymalEstimator::vioBetweenCallback_, this, ros::TransportHints().tcpNoDelay());
    REGULAR_COUT << COLOR_END << " Initialized VIO Between Odometry subscriber with topic: " << subVioBetween_.getTopic() << std::endl;
  }

  // Legged Odometry
  // Between
  if (useLeggedBetweenFlag_) {
    subLeggedBetween_ = privateNode.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
        "/legged_odometry_pose_topic", ROS_QUEUE_SIZE, &AnymalEstimator::leggedBetweenCallback_, this, ros::TransportHints().tcpNoDelay());
    REGULAR_COUT << COLOR_END << " Initialized Legged Odometry subscriber with topic: " << subLeggedBetween_.getTopic() << std::endl;
  }
  // Unary
  if (useLeggedVelocityUnaryFlag_) {
    subLeggedVelocityUnary_ =
        privateNode.subscribe<nav_msgs::Odometry>("/legged_odometry_odom_topic", ROS_QUEUE_SIZE,
                                                  &AnymalEstimator::leggedVelocityUnaryCallback_, this, ros::TransportHints().tcpNoDelay());
    REGULAR_COUT << COLOR_END
                 << " Initialized Legged Velocity Unary Factor Odometry subscriber with topic: " << subLeggedVelocityUnary_.getTopic()
                 << std::endl;
  }
  // Kinematics
  if (useLeggedKinematicsFlag_) {
    subLeggedKinematics_ = privateNode.subscribe<graph_msf_anymal_msgs::AnymalState>(
        "/anymal_state_topic", ROS_QUEUE_SIZE, &AnymalEstimator::leggedKinematicsCallback_, this, ros::TransportHints().tcpNoDelay());
    REGULAR_COUT << COLOR_END << " Initialized Legged Kinematics subscriber with topic: " << subLeggedKinematics_.getTopic() << std::endl;
  }
}

void AnymalEstimator::initializeMessages(ros::NodeHandle& privateNode) {
  graph_msf::GraphMsfRos::initializeMessages(privateNode);

  // Path
  measLio_mapLidarPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  measGnss_worldGnssPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  measVio_mapCameraPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  measVioBetween_mapCameraPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
}

void AnymalEstimator::initializeServices(ros::NodeHandle& privateNode) {
  graph_msf::GraphMsfRos::initializeServices(privateNode);

  // Nothing
  // TODO: add soft reset of the graph for on-the-go re-init.
}

// Offline Optimization Service
bool AnymalEstimator::srvOfflineSmootherOptimizeCallback(graph_msf_ros_msgs::OfflineOptimizationTrigger::Request& req,
                                                         graph_msf_ros_msgs::OfflineOptimizationTrigger::Response& res) {
  const std::lock_guard<std::mutex> estimatorLock(estimatorMutex_);

  // Call super class
  bool success = graph_msf::GraphMsfRos::srvOfflineSmootherOptimizeCallback(req, res);

  // if GNSS is used at all, also log the reference frame
  if (gnssCallbackCounter_ > 0) {
    // Get the reference coordinates of the GNSS handler
    const double referenceLatitude = gnssHandlerPtr_->getGnssReferenceLatitude();
    const double referenceLongitude = gnssHandlerPtr_->getGnssReferenceLongitude();
    const double referenceAltitude = gnssHandlerPtr_->getGnssReferenceAltitude();
    const double referenceHeading = gnssHandlerPtr_->getGnssReferenceHeading();
    REGULAR_COUT << " GNSS reference (latitude, longitude, altitude, heading): " << referenceLatitude << ", " << referenceLongitude << ", "
                 << referenceAltitude << ", " << referenceHeading << std::endl;

    // File streams
    std::map<std::string, std::ofstream> fileStreams;

    // Write T_totalStation_totalStationOld_ to file ----------------------------
    // Get time string by finding latest directory in optimizationResultLoggingPath
    std::string timeString = graph_msf::getLatestSubdirectory(optimizationResultLoggingPath);
    if (timeString.empty()) {
      REGULAR_COUT << " Could not find latest directory in " << optimizationResultLoggingPath << std::endl;
      return false;
    } else {
      REGULAR_COUT << " Found latest directory in " << optimizationResultLoggingPath << ": " << timeString << std::endl;
    }
    // Create file stream
    std::string transformIdentifier = "gnss_reference_lat_lon_alt";
    graph_msf::createLatLonAltCsvFileStream(fileStreams, optimizationResultLoggingPath, transformIdentifier, timeString);
    // Add to file
    graph_msf::writeLatLonAltToCsvFile(fileStreams, Eigen::Vector3d(referenceLatitude, referenceLongitude, referenceAltitude),
                                       transformIdentifier, ros::Time::now().toSec());
    REGULAR_COUT << " Wrote GNSS reference (latitude, longitude, altitude) coordinates to file." << std::endl;

    // If optional GNSS ENU coordinates are logged, write them to pose file
    if constexpr (logOptionalGnssEnuCoordinatesFlag_) {
      transformIdentifier = "gnss_enu_trajectory";
      graph_msf::createPose3CsvFileStream(fileStreams, optimizationResultLoggingPath, transformIdentifier, timeString, false);
      // Iterate over all optional GNSS ENU coordinates
      int index = 0;
      for (const auto& optionalGnssEnuCoordinates : optionalGnssEnuCoordinatesTrajectory_) {
        // Create Isometry3d with identity rotation
        Eigen::Isometry3d optionalGnssEnuCoordinatesIsometry = Eigen::Isometry3d::Identity();
        optionalGnssEnuCoordinatesIsometry.translation() = optionalGnssEnuCoordinates;
        // Add to file
        graph_msf::writePose3ToCsvFile(fileStreams, optionalGnssEnuCoordinatesIsometry, transformIdentifier,
                                       optionalGnssEnuCoordinatesTimeStamps_[index], false);
        index++;
      }
    }
  }

  // Return
  return success;
}

// Priority: 1
void AnymalEstimator::gnssUnaryCallback_(const sensor_msgs::NavSatFix::ConstPtr& gnssMsgPtr) {
  const std::lock_guard<std::mutex> estimatorLock(estimatorMutex_);

  // Fast validity checks mirror the ROS 2 reference before we let GNSS drive initialization
  // or add absolute factors into the graph.
  const auto status = gnssMsgPtr->status.status;
  if (status != sensor_msgs::NavSatStatus::STATUS_FIX && status != sensor_msgs::NavSatStatus::STATUS_SBAS_FIX &&
      status != sensor_msgs::NavSatStatus::STATUS_GBAS_FIX) {
    ROS_WARN_THROTTLE(0.5, "GNSS message has invalid fix status (%d). Expected FIX/SBAS_FIX/GBAS_FIX. Skipping.", status);
    return;
  }
  if (gnssMsgPtr->position_covariance_type != sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN) {
    ROS_WARN_THROTTLE(0.5, "GNSS message has unknown covariance type. Skipping.");
    return;
  }
  if (gnssMsgPtr->position_covariance[0] < 0.0) {
    ROS_WARN_THROTTLE(0.5, "GNSS message has invalid covariance. Skipping.");
    return;
  }
  if (!std::isfinite(gnssMsgPtr->latitude) || !std::isfinite(gnssMsgPtr->longitude) || !std::isfinite(gnssMsgPtr->altitude)) {
    ROS_WARN_THROTTLE(0.5, "GNSS message contains non-finite values. Skipping.");
    return;
  }

  ++gnssCallbackCounter_;

  const double timeK = gnssMsgPtr->header.stamp.toSec();
  Eigen::Vector3d gnssCoord(gnssMsgPtr->latitude, gnssMsgPtr->longitude, gnssMsgPtr->altitude);

  // The ROS 2 reference rotates the full covariance from ENU into LV03 before building the
  // absolute GNSS factor. Reuse the same path here instead of only reading the covariance diagonal.
  Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> PenuMap(gnssMsgPtr->position_covariance.data());
  Eigen::Matrix3d P_enu = 0.5 * (PenuMap + PenuMap.transpose());
  Eigen::Matrix3d P_lv03 =
      graph_msf::gnss_cov::rotateCov_ENU_to_LV03(*gnssHandlerPtr_, gnssMsgPtr->latitude, gnssMsgPtr->longitude,
                                                 gnssMsgPtr->altitude, P_enu);
  Eigen::Vector3d estStdDevXYZ(std::sqrt(P_lv03(0, 0)), std::sqrt(P_lv03(1, 1)), std::sqrt(P_lv03(2, 2)));

  // Accumulate the first N messages so the GNSS handler can choose a stable reference.
  // Unlike the old ROS 1 code, keep processing the N-th message after initialization so
  // the bootstrap behavior matches the ROS 2 reference.
  if (gnssCallbackCounter_ < NUM_GNSS_CALLBACKS_UNTIL_START) {
    accumulatedGnssCoordinates_ += gnssCoord;
    if ((gnssCallbackCounter_ % 10) == 0) {
      REGULAR_COUT << " NOT ENOUGH GNSS MESSAGES ARRIVED!" << std::endl;
    }
    return;
  } else if (gnssCallbackCounter_ == NUM_GNSS_CALLBACKS_UNTIL_START) {
    const Eigen::Vector3d avgGnssCoord = accumulatedGnssCoordinates_ / NUM_GNSS_CALLBACKS_UNTIL_START;
    gnssHandlerPtr_->initHandler(avgGnssCoord);

    sensor_msgs::NavSatFix referenceGnssMsg;
    referenceGnssMsg.header = gnssMsgPtr->header;
    referenceGnssMsg.latitude = gnssHandlerPtr_->getGnssReferenceLatitude();
    referenceGnssMsg.longitude = gnssHandlerPtr_->getGnssReferenceLongitude();
    referenceGnssMsg.altitude = gnssHandlerPtr_->getGnssReferenceAltitude();
    pubReferenceNavSatFixCoordinates_.publish(referenceGnssMsg);

    REGULAR_COUT << "\033[1;36m"
                 << "==================== GNSS REFERENCE ====================\n"
                 << " LATITUDE : " << referenceGnssMsg.latitude << "\n"
                 << " LONGITUDE: " << referenceGnssMsg.longitude << "\n"
                 << " ALTITUDE : " << referenceGnssMsg.altitude << "\n"
                 << "=======================================================\n"
                 << "\033[0m";

    sensor_msgs::NavSatFix referenceGnssMsgEnu;
    Eigen::Vector3d originAsEnu = Eigen::Vector3d::Zero();
    gnssHandlerPtr_->convertNavSatToPositionLV03(gnssCoord, originAsEnu);
    referenceGnssMsgEnu.header = gnssMsgPtr->header;
    referenceGnssMsgEnu.latitude = originAsEnu(0);
    referenceGnssMsgEnu.longitude = originAsEnu(1);
    referenceGnssMsgEnu.altitude = originAsEnu(2);
    pubReferenceNavSatFixCoordinatesENU_.publish(referenceGnssMsgEnu);

    REGULAR_COUT << "\033[1;36m"
                 << "==================== GNSS REFERENCE ENU ====================\n"
                 << " X : " << referenceGnssMsgEnu.latitude << "\n"
                 << " Y : " << referenceGnssMsgEnu.longitude << "\n"
                 << " Z : " << referenceGnssMsgEnu.altitude << "\n"
                 << "===========================================================\n"
                 << "\033[0m";

    REGULAR_COUT << " GNSS Handler initialized." << COLOR_END << std::endl;

    std_msgs::Bool statusMsg;
    statusMsg.data = false;
    pubStatus_.publish(statusMsg);
  }

  Eigen::Vector3d W_t_W_Gnss = Eigen::Vector3d::Zero();
  gnssHandlerPtr_->convertNavSatToPositionLV03(gnssCoord, W_t_W_Gnss);
  std::string fixedFrame = worldFrame_;

  if (!areYawAndPositionInited()) {
    const std::string& baseFrame = baseLinkFrame_;
    const std::string& gnssFrame = gnssFrame_;
    const Eigen::Vector3d t_B_G = t_B_G_cached_;

    double initYaw_W_Base = 0.0;
    bool have_R_W_B_full = false;
    Eigen::Matrix3d R_W_B_full = Eigen::Matrix3d::Identity();

    if (gnssHandlerPtr_->getUseYawInitialGuessFromFile()) {
      initYaw_W_Base = gnssHandlerPtr_->getGlobalYawDegFromFile() / 180.0 * M_PI;
    } else if (gnssHandlerPtr_->getUseYawInitialGuessFromAlignment()) {
      if (gnssCallbackCounter_ % 20 == 0) {
        REGULAR_COUT << YELLOW_START << " Adding GNSS measurement to trajectory alignment." << COLOR_END << std::endl;
      }

      trajectoryAlignmentHandler_->addR3Position(W_t_W_Gnss, timeK);

      double yaw_W_M = 0.0;
      Eigen::Isometry3d T_W_M = Eigen::Isometry3d::Identity();
      if (!trajectoryAlignmentHandler_->alignTrajectories(yaw_W_M, T_W_M)) {
        if (gnssCallbackCounter_ % 10 == 0) {
          REGULAR_COUT << YELLOW_START << "Trajectory alignment not ready. Waiting for more motion." << COLOR_END << std::endl;
        }
        return;
      }

      // The ROS 2 reference computes the GNSS initialization yaw from the synchronized LiDAR pose
      // buffer rather than assuming the LiDAR body frame and GNSS antenna frame are co-located.
      Eigen::Isometry3d T_M_B_t0 = Eigen::Isometry3d::Identity();
      double bestDt = 0.0;
      if (!getClosestLioPose_(timeK, T_M_B_t0, &bestDt)) {
        if (!std::isfinite(bestDt)) {
          REGULAR_COUT << YELLOW_START
                       << "Trajectory alignment ready, but LiDAR pose buffer is empty (cannot compute yaw(W<-B))."
                       << COLOR_END << std::endl;
        } else {
          REGULAR_COUT << YELLOW_START
                       << "Trajectory alignment ready, but no sufficiently time-synced LiDAR pose for yaw(W<-B). best_dt="
                       << std::fixed << std::setprecision(3) << bestDt << " s" << COLOR_END << std::endl;
        }
        return;
      }

      R_W_B_full = T_W_M.rotation() * T_M_B_t0.rotation();
      have_R_W_B_full = true;

      const Eigen::Vector3d x_W = R_W_B_full.col(0);
      initYaw_W_Base = std::atan2(x_W.y(), x_W.x());

      std_msgs::Bool statusMsg;
      statusMsg.data = true;
      pubStatus_.publish(statusMsg);

      REGULAR_COUT << GREEN_START << "Trajectory Alignment Successful. "
                   << "yaw(W<-M) [deg]=" << (180.0 * yaw_W_M / M_PI) << "  yaw(W<-B) [deg]="
                   << (180.0 * initYaw_W_Base / M_PI) << COLOR_END << std::endl;
    }

    Eigen::Matrix3d R_W_B_forLever = Eigen::AngleAxisd(initYaw_W_Base, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    if (have_R_W_B_full) {
      R_W_B_forLever = R_W_B_full;
    }
    const Eigen::Vector3d W_t_W_Base = W_t_W_Gnss - R_W_B_forLever * t_B_G;

    if (this->initYawAndPositionInWorld(initYaw_W_Base, W_t_W_Base, baseFrame, baseFrame)) {
      REGULAR_COUT << GREEN_START << " GNSS initialization of yaw and position successful." << COLOR_END << std::endl;
    } else {
      REGULAR_COUT << RED_START << " GNSS initialization of yaw and position failed." << COLOR_END << std::endl;
    }
  } else {
    const std::string& gnssFrameName = gnssFrame_;
    graph_msf::UnaryMeasurementXDAbsolute<Eigen::Vector3d, 3> meas_W_t_W_Gnss(
        "GnssPosition", int(gnssRate_), gnssFrameName, gnssFrameName + sensorFrameCorrectedNameId, graph_msf::RobustNorm::Huber(0.25),
        timeK, gnssPositionOutlierThreshold_, W_t_W_Gnss, estStdDevXYZ, fixedFrame, worldFrame_);
    this->addUnaryPosition3AbsoluteMeasurement(meas_W_t_W_Gnss);
  }

  if (fixedFrame != worldFrame_) {
    fixedFrame += referenceFrameAlignedNameId;
  }

  // Publish the rotated GNSS covariance in the aligned frame so downstream debugging in ROS 1
  // sees the same information that the ROS 2 reference publishes.
  if (pubGnssPoseWithCov_.getNumSubscribers() > 0) {
    geometry_msgs::PoseWithCovarianceStamped gnssPoseWithCovMsg;
    gnssPoseWithCovMsg.header.frame_id = fixedFrame;
    gnssPoseWithCovMsg.header.stamp = gnssMsgPtr->header.stamp;
    gnssPoseWithCovMsg.pose.pose.position.x = W_t_W_Gnss.x();
    gnssPoseWithCovMsg.pose.pose.position.y = W_t_W_Gnss.y();
    gnssPoseWithCovMsg.pose.pose.position.z = W_t_W_Gnss.z();
    gnssPoseWithCovMsg.pose.pose.orientation.w = 1.0;
    gnssPoseWithCovMsg.pose.pose.orientation.x = 0.0;
    gnssPoseWithCovMsg.pose.pose.orientation.y = 0.0;
    gnssPoseWithCovMsg.pose.pose.orientation.z = 0.0;

    for (double& covarianceEntry : gnssPoseWithCovMsg.pose.covariance) {
      covarianceEntry = 0.0;
    }
    gnssPoseWithCovMsg.pose.covariance[0] = P_lv03(0, 0);
    gnssPoseWithCovMsg.pose.covariance[1] = P_lv03(0, 1);
    gnssPoseWithCovMsg.pose.covariance[2] = P_lv03(0, 2);
    gnssPoseWithCovMsg.pose.covariance[6] = P_lv03(1, 0);
    gnssPoseWithCovMsg.pose.covariance[7] = P_lv03(1, 1);
    gnssPoseWithCovMsg.pose.covariance[8] = P_lv03(1, 2);
    gnssPoseWithCovMsg.pose.covariance[12] = P_lv03(2, 0);
    gnssPoseWithCovMsg.pose.covariance[13] = P_lv03(2, 1);
    gnssPoseWithCovMsg.pose.covariance[14] = P_lv03(2, 2);

    const double bigOrientationVariance = 1e6;
    gnssPoseWithCovMsg.pose.covariance[21] = bigOrientationVariance;
    gnssPoseWithCovMsg.pose.covariance[28] = bigOrientationVariance;
    gnssPoseWithCovMsg.pose.covariance[35] = bigOrientationVariance;

    pubGnssPoseWithCov_.publish(gnssPoseWithCovMsg);
  }

  if (pubMeasWorldGnssPath_.getNumSubscribers() > 0) {
    addToPathMsg(measGnss_worldGnssPathPtr_, fixedFrame, gnssMsgPtr->header.stamp, W_t_W_Gnss,
                 static_cast<int>(graphConfigPtr_->imuBufferLength_ / 2.0));
    pubMeasWorldGnssPath_.publish(measGnss_worldGnssPathPtr_);
  }

  // Potentially log the GNSS ENU coordinates
  if (logOptionalGnssEnuCoordinatesFlag_) {
    optionalGnssEnuCoordinatesTimeStamps_.push_back(gnssMsgPtr->header.stamp.toSec());
    optionalGnssEnuCoordinatesTrajectory_.push_back(W_t_W_Gnss);
  }
}

// Priority: 2
void AnymalEstimator::lidarUnaryCallback_(const nav_msgs::Odometry::ConstPtr& odomLidarPtr) {
  const std::lock_guard<std::mutex> estimatorLock(estimatorMutex_);

  const double lidarUnaryTimeK = odomLidarPtr->header.stamp.toSec();

  // Mirror the ROS 2 LiDAR-unary rate gate so the absolute pose factor stream is sampled the same way.
  if (lioOdometryRate_ > 0.0 && lastLidarUnaryTime_ > 0.0) {
    const double dt = lidarUnaryTimeK - lastLidarUnaryTime_;
    if (dt > 0.0 && dt < (1.0 / lioOdometryRate_)) {
      return;
    }
  }
  lastLidarUnaryTime_ = lidarUnaryTimeK;
  ++lidarUnaryCallbackCounter_;

  // Prepare Data
  Eigen::Isometry3d lio_T_M_Lk = Eigen::Isometry3d::Identity();
  graph_msf::odomMsgToEigen(*odomLidarPtr, lio_T_M_Lk.matrix());

  // Keep a short LiDAR pose history so GNSS alignment can reconstruct the body orientation the same
  // way as the ROS 2 reference does.
  {
    std::lock_guard<std::mutex> lioPoseBufLock(lioPoseBufMutex_);
    lioPoseBuf_.emplace_back(lidarUnaryTimeK, lio_T_M_Lk);

    const double tMin = lidarUnaryTimeK - kLioBufKeepSec;
    while (!lioPoseBuf_.empty() && lioPoseBuf_.front().first < tMin) {
      lioPoseBuf_.pop_front();
    }
  }

  const std::string& lioOdomFrameName = lioOdometryFrame_;
  const std::string& lioFixedFrame = odomLidarPtr->header.frame_id;

  if (useGnssUnaryFlag_ && gnssHandlerPtr_->getUseYawInitialGuessFromAlignment()) {
    const std::string& baseFrame = odomLidarPtr->child_frame_id.empty() ? baseLinkFrame_ : odomLidarPtr->child_frame_id;

    // Feed the aligner with the GNSS antenna trajectory reconstructed from the LiDAR odometry pose
    // and the cached GNSS lever arm, exactly as in the ROS 2 reference.
    const Eigen::Vector3d t_B_G =
        (baseFrame == baseLinkFrame_) ? t_B_G_cached_ : staticTransformsPtr_->rv_T_frame1_frame2(baseFrame, gnssFrame_).translation();
    const Eigen::Vector3d p_G_in_M = lio_T_M_Lk.translation() + lio_T_M_Lk.rotation() * t_B_G;
    trajectoryAlignmentHandler_->addSe3Position(p_G_in_M, lidarUnaryTimeK);
  }

  if (lioOdomFrameName != odomLidarPtr->child_frame_id) {
    REGULAR_COUT << RED_START << "====================================================================\n"
                 << "ERROR: LIDAR ODOMETRY FRAME MISMATCH!\n"
                 << "Expected frame: " << lioOdomFrameName << "\n"
                 << "Odometry message child_frame_id: " << odomLidarPtr->child_frame_id << "\n"
                 << "====================================================================\n"
                 << COLOR_END << std::endl;
  }
  if (lioFixedFrame.empty()) {
    ROS_WARN_THROTTLE(1.0, "LiDAR odometry header.frame_id is empty. Absolute pose unary will be ill-defined.");
  } else if (lioFixedFrame == lioOdomFrameName) {
    ROS_WARN_THROTTLE(1.0,
                      "LiDAR absolute pose uses the sensor/body frame as fixed frame (%s). Expected a map/odom-like reference frame.",
                      lioFixedFrame.c_str());
  }

  graph_msf::UnaryMeasurementXDAbsolute<Eigen::Isometry3d, 6> unary6DMeasurement(
      "Lidar_unary_6D", int(lioOdometryRate_), lioOdomFrameName, lioOdomFrameName + sensorFrameCorrectedNameId,
      graph_msf::RobustNorm::DCS(1.0), lidarUnaryTimeK, 1.0, lio_T_M_Lk, lioPoseUnaryNoise_, lioFixedFrame, worldFrame_,
      initialSe3AlignmentNoise_, lioSe3AlignmentRandomWalk_, graph_msf::AbsoluteUnaryAlignmentRecoveryPolicy::RestartFromPrior);

  if (lidarUnaryCallbackCounter_ <= 2) {
    return;
  } else if (!areYawAndPositionInited()) {
    if (!useGnssUnaryFlag_) {
      REGULAR_COUT << GREEN_START << " LiDAR odometry callback is setting global yaw, as it was not set so far." << COLOR_END << std::endl;
      this->initYawAndPosition(unary6DMeasurement);
    }
  } else {
    this->addUnaryPose3AbsoluteMeasurement(unary6DMeasurement);
  }

  if (pubMeasMapLioPath_.getNumSubscribers() > 0 && areYawAndPositionInited()) {
    addToPathMsg(measLio_mapLidarPathPtr_, lioFixedFrame + referenceFrameAlignedNameId, odomLidarPtr->header.stamp, lio_T_M_Lk.translation(),
                 static_cast<int>(graphConfigPtr_->imuBufferLength_ / 2.0));
    pubMeasMapLioPath_.publish(measLio_mapLidarPathPtr_);
  }
}

// Priority: 3
void AnymalEstimator::lidarBetweenCallback_(const nav_msgs::Odometry::ConstPtr& odomLidarPtr) {
  const std::lock_guard<std::mutex> estimatorLock(estimatorMutex_);

  if (!areRollAndPitchInited()) {
    return;
  }

  // Counter
  ++lidarBetweenCallbackCounter_;

  // Convert
  Eigen::Isometry3d lio_T_M_Lk = Eigen::Isometry3d::Identity();
  graph_msf::odomMsgToEigen(*odomLidarPtr, lio_T_M_Lk.matrix());
  // Get the time
  double lidarBetweenTimeK = odomLidarPtr->header.stamp.toSec();

  // At start
  if (lidarBetweenCallbackCounter_ == 0) {
    lio_T_M_Lkm1_ = lio_T_M_Lk;
    lidarBetweenTimeKm1_ = lidarBetweenTimeK;
  }

  // Add to trajectory aligner if needed.
  if (useGnssUnaryFlag_ && gnssHandlerPtr_->getUseYawInitialGuessFromAlignment()) {
    trajectoryAlignmentHandler_->addSe3Position(lio_T_M_Lk.translation(), lidarBetweenTimeK);
  }

  // Frame Name
  const std::string& lioOdomFrameName = lioOdometryFrame_;

  // State Machine
  if (lidarBetweenCallbackCounter_ <= 2) {
    return;
  } else if (!areYawAndPositionInited()) {  // Initializing
    if (!useGnssUnaryFlag_ && !useLioUnaryFlag_) {
      // Measurement
      graph_msf::UnaryMeasurementXD<Eigen::Isometry3d, 6> unary6DMeasurement(
          "Lidar_unary_6D", int(lioOdometryRate_), lioOdomFrameName, lioOdomFrameName + sensorFrameCorrectedNameId,
          graph_msf::RobustNorm::None(), lidarBetweenTimeK, 1.0, lio_T_M_Lk, lioPoseUnaryNoise_);
      // Add to graph
      REGULAR_COUT << GREEN_START << " LiDAR odometry callback is setting global yaw, as it was not set so far." << COLOR_END << std::endl;
      this->initYawAndPosition(unary6DMeasurement);
    }
  } else {  // Already initialized --> Between factor
    // Compute Delta
    const Eigen::Isometry3d T_Lkm1_Lk = lio_T_M_Lkm1_.inverse() * lio_T_M_Lk;
    // Create measurement
    graph_msf::BinaryMeasurementXD<Eigen::Isometry3d, 6> delta6DMeasurement(
        "Lidar_between_6D", int(lioOdometryRate_), lioOdomFrameName, lioOdomFrameName + sensorFrameCorrectedNameId,
        graph_msf::RobustNorm::None(), lidarBetweenTimeKm1_, lidarBetweenTimeK, T_Lkm1_Lk, lioPoseBetweenNoise_);
    // Add to graph
    this->addBinaryPose3Measurement(delta6DMeasurement);
  }
  // Provide for next iteration
  lio_T_M_Lkm1_ = lio_T_M_Lk;
  lidarBetweenTimeKm1_ = lidarBetweenTimeK;

  // Visualization ----------------------------
  // Add to path message
  addToPathMsg(measLio_mapLidarPathPtr_, worldFrame_, odomLidarPtr->header.stamp, lio_T_M_Lk.translation(), graphConfigPtr_->imuBufferLength_ * 4);

  // Publish Path
  pubMeasMapLioPath_.publish(measLio_mapLidarPathPtr_);
}

void AnymalEstimator::vioUnaryCallback_(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& vioOdomPtr) {
  const std::lock_guard<std::mutex> estimatorLock(estimatorMutex_);

  if (!areRollAndPitchInited()) {
    return;
  }

  const double timeK = vioOdomPtr->header.stamp.toSec();

  // Sample the absolute VIO stream the same way as the ROS 2 reference so the unary factor
  // cadence is comparable across wrappers.
  if (vioOdometryRate_ > 0.0 && lastVioUnaryTime_ > 0.0) {
    const double dt = timeK - lastVioUnaryTime_;
    if (dt > 0.0 && dt < (1.0 / vioOdometryRate_)) {
      return;
    }
  }
  lastVioUnaryTime_ = timeK;
  ++vioUnaryCallbackCounter_;

  Eigen::Matrix<double, 6, 1> vioCovariance = vioPoseUnaryNoise_;

  Eigen::Isometry3d vio_T_M_Ck = Eigen::Isometry3d::Identity();
  graph_msf::geometryPoseToEigen(*vioOdomPtr, vio_T_M_Ck.matrix());

  const std::string& vioOdometryFrame = vioOdometryFrame_;
  const std::string& vioFixedFrame = vioOdomPtr->header.frame_id;
  if (vioFixedFrame.empty()) {
    ROS_WARN_THROTTLE(1.0, "VIO header.frame_id is empty. Absolute pose unary will be ill-defined.");
  } else if (vioFixedFrame == vioOdometryFrame) {
    ROS_WARN_THROTTLE(1.0,
                      "VIO absolute pose uses the sensor/body frame as fixed frame (%s). Expected a map/odom-like reference frame.",
                      vioFixedFrame.c_str());
  }

  graph_msf::UnaryMeasurementXDAbsolute<Eigen::Isometry3d, 6> unary6DMeasurement(
      "Vio_unary_6D", int(vioOdometryRate_), vioOdometryFrame, vioOdometryFrame + sensorFrameCorrectedNameId,
      graph_msf::RobustNorm::DCS(2.0), timeK, 1.0, vio_T_M_Ck, vioCovariance, vioFixedFrame, worldFrame_, initialSe3AlignmentNoise_,
      vioSe3AlignmentRandomWalk_, graph_msf::AbsoluteUnaryAlignmentRecoveryPolicy::RestartFromPrior);

  if (vioUnaryCallbackCounter_ <= 2) {
    // Skip the first few samples to mirror the ROS 2 bootstrap behavior.
  } else if (!areYawAndPositionInited()) {
    if (!useGnssUnaryFlag_ && !useLioUnaryFlag_) {
      REGULAR_COUT << GREEN_START << " VIO odometry callback is setting global yaw, as it was not set so far." << COLOR_END << std::endl;
      this->initYawAndPosition(unary6DMeasurement);
    }
  } else {
    this->addUnaryPose3AbsoluteMeasurement(unary6DMeasurement);
  }

  if (pubMeasMapVioPath_.getNumSubscribers() > 0 && areYawAndPositionInited()) {
    addToPathMsg(measVio_mapCameraPathPtr_, vioFixedFrame + referenceFrameAlignedNameId, vioOdomPtr->header.stamp, vio_T_M_Ck.translation(),
                 static_cast<int>(graphConfigPtr_->imuBufferLength_ / 2.0));
    pubMeasMapVioPath_.publish(measVio_mapCameraPathPtr_);
  }
}

void AnymalEstimator::vioBetweenCallback_(const nav_msgs::Odometry::ConstPtr& vioOdomPtr) {
  const std::lock_guard<std::mutex> estimatorLock(estimatorMutex_);

  if (!areRollAndPitchInited()) {
    return;
  }

  const double timeK = vioOdomPtr->header.stamp.toSec();

  // Rate-gate the relative VIO stream to match the ROS 2 behavior and keep the between-factor
  // density predictable when the upstream odometry publishes faster than requested.
  if (vioOdometryBetweenRate_ > 0.0 && lastVioBetweenTime_ > 0.0) {
    const double dt = timeK - lastVioBetweenTime_;
    if (dt > 0.0 && dt < (1.0 / vioOdometryBetweenRate_)) {
      return;
    }
  }
  lastVioBetweenTime_ = timeK;
  ++vioBetweenCallbackCounter_;

  Eigen::Matrix<double, 6, 1> vioCovariance = vioPoseBetweenNoise_;
  if (vioOdomPtr->pose.covariance.size() == 36) {
    Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> covarianceMap(vioOdomPtr->pose.covariance.data());
    // The graph expects [roll pitch yaw x y z] variances. nav_msgs/Odometry stores position first.
    vioCovariance << covarianceMap(3, 3), covarianceMap(4, 4), covarianceMap(5, 5), covarianceMap(0, 0), covarianceMap(1, 1),
        covarianceMap(2, 2);
  } else {
    ROS_WARN_THROTTLE(0.5, "VIO odometry message does not contain a 6x6 covariance. Using default between covariance.");
  }

  Eigen::Isometry3d vio_T_M_Ck = Eigen::Isometry3d::Identity();
  graph_msf::odomMsgToEigen(*vioOdomPtr, vio_T_M_Ck.matrix());

  const std::string& vioOdometryFrame = vioOdometryFrame_;
  if (vioOdometryFrame != vioOdomPtr->child_frame_id) {
    REGULAR_COUT << RED_START << "====================================================================\n"
                 << "ERROR: VIO ODOMETRY FRAME MISMATCH!\n"
                 << "Expected frame: " << vioOdometryFrame << "\n"
                 << "Odometry message child_frame_id: " << vioOdomPtr->child_frame_id << "\n"
                 << "====================================================================\n"
                 << COLOR_END << std::endl;
  }

  if (vioBetweenCallbackCounter_ <= 2) {
    vio_T_M_Ckm1_ = vio_T_M_Ck;
    vioBetweenTimeKm1_ = timeK;
    return;
  }

  if (!areYawAndPositionInited()) {
    vio_T_M_Ckm1_ = vio_T_M_Ck;
    vioBetweenTimeKm1_ = timeK;
    return;
  }

  if (vioBetweenTimeKm1_ > 0.0 && timeK > vioBetweenTimeKm1_) {
    const Eigen::Isometry3d T_Ckm1_Ck = vio_T_M_Ckm1_.inverse() * vio_T_M_Ck;

    graph_msf::BinaryMeasurementXD<Eigen::Isometry3d, 6> delta6DMeasurement(
        "Vio_between_6D", int(vioOdometryBetweenRate_), vioOdometryFrame, vioOdometryFrame + sensorFrameCorrectedNameId,
        graph_msf::RobustNorm::DCS(2.0), vioBetweenTimeKm1_, timeK, T_Ckm1_Ck, vioCovariance);
    this->addBinaryPose3Measurement(delta6DMeasurement);
  } else {
    REGULAR_COUT << RED_START << "[VIO Between] ERROR: invalid time ordering for between factor. "
                 << "vioTimeKm1=" << std::fixed << std::setprecision(9) << vioBetweenTimeKm1_ << ", timeK=" << timeK
                 << ", dt=" << (timeK - vioBetweenTimeKm1_) << COLOR_END << std::endl;
  }

  vio_T_M_Ckm1_ = vio_T_M_Ck;
  vioBetweenTimeKm1_ = timeK;

  if (pubMeasMapVioBetweenPath_.getNumSubscribers() > 0) {
    addToPathMsg(measVioBetween_mapCameraPathPtr_, vioOdometryFrame + referenceFrameAlignedNameId, vioOdomPtr->header.stamp,
                 vio_T_M_Ck.translation(), static_cast<int>(graphConfigPtr_->imuBufferLength_ / 2.0));
    pubMeasMapVioBetweenPath_.publish(measVioBetween_mapCameraPathPtr_);
  }
}

// Priority: 4
void AnymalEstimator::leggedBetweenCallback_(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& leggedOdometryPoseKPtr) {
  const std::lock_guard<std::mutex> estimatorLock(estimatorMutex_);

  if (!areRollAndPitchInited()) {
    return;
  }

  // Counter
  ++leggedOdometryPoseCallbackCounter_;

  // Eigen Type
  Eigen::Isometry3d T_O_Bl_k = Eigen::Isometry3d::Identity();
  graph_msf::geometryPoseToEigen(*leggedOdometryPoseKPtr, T_O_Bl_k.matrix());
  double legOdometryTimeK = leggedOdometryPoseKPtr->header.stamp.toSec();

  // At start
  if (leggedOdometryPoseCallbackCounter_ == 0) {
    T_O_Bl_km1_ = T_O_Bl_k;
    legOdometryTimeKm1_ = legOdometryTimeK;
    return;
  }

  // Frame Name
  const std::string& leggedOdometryFrameName = leggedOdometryFrame_;

  // State Machine
  if (!areYawAndPositionInited()) {
    // Keep the legged fallback from stealing initialization when one of the ROS 2-aligned
    // absolute sensors is explicitly enabled to own global pose bootstrap.
    if (!useGnssUnaryFlag_ && !useLioUnaryFlag_ && !useLioBetweenFlag_ && !useVioOdometryFlag_) {
      // Measurement
      graph_msf::UnaryMeasurementXD<Eigen::Isometry3d, 6> unary6DMeasurement(
          "Leg_odometry_6D", int(leggedOdometryBetweenRate_), leggedOdometryFrameName, leggedOdometryFrameName + sensorFrameCorrectedNameId,
          graph_msf::RobustNorm::None(), legOdometryTimeK, 1.0, T_O_Bl_k, legPoseBetweenNoise_);
      // Add to graph
      REGULAR_COUT << GREEN_START << " Legged odometry between callback is setting global yaw, as it was not set so far." << COLOR_END
                   << std::endl;
      this->initYawAndPosition(unary6DMeasurement);
    }
  } else {
    // Only add every 40th measurement
    int measurementRate = static_cast<int>(leggedOdometryBetweenRate_) / leggedOdometryPoseDownsampleFactor_;
    // Check
    if ((leggedOdometryPoseCallbackCounter_ % leggedOdometryPoseDownsampleFactor_) == 0) {
      // Compute Delta
      const Eigen::Isometry3d T_Bkm1_Bk = T_O_Bl_km1_.inverse() * T_O_Bl_k;
      // Create measurement
      graph_msf::BinaryMeasurementXD<Eigen::Isometry3d, 6> delta6DMeasurement(
          "Leg_odometry_6D", measurementRate, leggedOdometryFrameName, leggedOdometryFrameName + sensorFrameCorrectedNameId,
          graph_msf::RobustNorm::None(), legOdometryTimeKm1_, legOdometryTimeK, T_Bkm1_Bk, legPoseBetweenNoise_);
      // Add to graph
      this->addBinaryPose3Measurement(delta6DMeasurement);

      // Prepare for next iteration
      T_O_Bl_km1_ = T_O_Bl_k;
      legOdometryTimeKm1_ = legOdometryTimeK;
    }
  }
}

// Priority: 5
void AnymalEstimator::leggedVelocityUnaryCallback_(const nav_msgs::Odometry ::ConstPtr& leggedOdometryKPtr) {
  const std::lock_guard<std::mutex> estimatorLock(estimatorMutex_);

  if (!areRollAndPitchInited()) {
    return;
  }

  // Counter
  ++leggedOdometryOdomCallbackCounter_;

  if (!areYawAndPositionInited()) {
    // Preserve the old legged fallback, but only when no GNSS, LiDAR unary, or VIO unary
    // sensor is supposed to provide the global initialization path.
    if (!useGnssUnaryFlag_ && !useLioUnaryFlag_ && !useLioBetweenFlag_ && !useVioOdometryFlag_ && !useLeggedBetweenFlag_) {
      // Measurement
      graph_msf::UnaryMeasurementXD<Eigen::Isometry3d, 6> unary6DMeasurement(
          "Leg_odometry_6D", int(leggedOdometryVelocityRate_), leggedOdometryFrame_, leggedOdometryFrame_ + sensorFrameCorrectedNameId,
          graph_msf::RobustNorm::None(), leggedOdometryKPtr->header.stamp.toSec(), 1.0, Eigen::Isometry3d::Identity(), legPoseBetweenNoise_);
      // Add to graph
      REGULAR_COUT << GREEN_START << " Legged odometry velocity callback is setting global yaw, as it was not set so far." << COLOR_END
                   << std::endl;
      this->initYawAndPosition(unary6DMeasurement);
    }
  } else {
    // Only add every nth measurement
    int measurementRate = static_cast<int>(leggedOdometryVelocityRate_) / leggedOdometryVelocityDownsampleFactor_;

    // Check
    if ((leggedOdometryOdomCallbackCounter_ % leggedOdometryVelocityDownsampleFactor_) == 0) {
      // Eigen Type
      Eigen::Vector3d legVelocity = Eigen::Vector3d(leggedOdometryKPtr->twist.twist.linear.x, leggedOdometryKPtr->twist.twist.linear.y,
                                                    leggedOdometryKPtr->twist.twist.linear.z);

      // Alias
      const std::string& leggedOdometryFrameName = leggedOdometryFrame_;

      // Create the unary measurement
      graph_msf::UnaryMeasurementXD<Eigen::Vector3d, 3> legVelocityUnaryMeasurement(
          "LegVelocityUnary", measurementRate, leggedOdometryFrameName, leggedOdometryFrameName + sensorFrameCorrectedNameId,
          graph_msf::RobustNorm::None(), leggedOdometryKPtr->header.stamp.toSec(), 1.0, legVelocity, legVelocityUnaryNoise_);

      // Add to graph
      this->addUnaryVelocity3LocalMeasurement(legVelocityUnaryMeasurement);
    }
  }
}

void AnymalEstimator::leggedKinematicsCallback_(const graph_msf_anymal_msgs::AnymalState::ConstPtr& anymalStatePtr) {
  const std::lock_guard<std::mutex> estimatorLock(estimatorMutex_);

  if (!areRollAndPitchInited()) {
    return;
  }

  // Counter
  ++leggedKinematicsCallbackCounter_;

  if (!areYawAndPositionInited()) {
    // The kinematics-only bootstrap should remain the last fallback after the ROS 2-aligned
    // absolute sensors and the existing legged odometry fallbacks have all been ruled out.
    if (!useGnssUnaryFlag_ && !useLioUnaryFlag_ && !useLioBetweenFlag_ && !useVioOdometryFlag_ && !useLeggedBetweenFlag_ &&
        !useLeggedVelocityUnaryFlag_) {
      // Measurement
      graph_msf::UnaryMeasurementXD<Eigen::Isometry3d, 6> unary6DMeasurement(
          "Leg_odometry_6D", int(leggedKinematicsRate_), leggedOdometryFrame_, leggedOdometryFrame_ + sensorFrameCorrectedNameId,
          graph_msf::RobustNorm::None(), anymalStatePtr->header.stamp.toSec(), 1.0, Eigen::Isometry3d::Identity(),
          Eigen::Matrix<double, 6, 1>::Identity());
      // Add to graph
      REGULAR_COUT << GREEN_START << " Legged kinematics callback is setting global yaw, as it was not set so far." << COLOR_END
                   << std::endl;
      this->initYawAndPosition(unary6DMeasurement);
    }
  }
  // Normal Operation
  else {
    // Only add every nth measurement
    int measurementRate = static_cast<int>(leggedKinematicsRate_) / leggedKinematicsDownsampleFactor_;

    // Check whether it is time to add a measurement
    if ((leggedKinematicsCallbackCounter_ % leggedKinematicsDownsampleFactor_) == 0) {
      // Alias
      const std::string& leggedOdometryFrameName = leggedOdometryFrame_;

      // Pose of the base in odom frame
      Eigen::Isometry3d T_O_B = Eigen::Isometry3d::Identity();
      graph_msf::geometryPoseToEigen(anymalStatePtr->pose.pose, T_O_B.matrix());

      // Create the unary measurement for each foot (loop unrolling with constexpr arrays) ----------------------------
      visualization_msgs::MarkerArray footContactMarkers = visualization_msgs::MarkerArray();
      std::array<bool, legNames_.size()> seenLegs = {false, false, false, false};

      // Loop over reported contacts and map them to the estimator's canonical leg order.
      for (const auto& contact : anymalStatePtr->contacts) {
        const int legIndex = legIndexFromName_(contact.name);
        if (legIndex < 0 || seenLegs[legIndex]) {
          continue;
        }
        seenLegs[legIndex] = true;

        // Get contact state
        const bool footInContact = contact.state;
        // Foot name as specified in the message
        const std::string& legName = contact.name;

        // Check whether leg is in contact ----------------------------
        if (footInContact) {
          // Increase debounce counter
          ++legInContactForNSteps_[legIndex];

          // Check whether leg was at least N times in contact to count it as in contact --> debouncing
          if (legInContactForNSteps_[legIndex] >= legInContactDebounceThreshold_) {
            // If it was not in contact before
            if (legInContactForNSteps_[legIndex] == legInContactDebounceThreshold_) {
              // Increase contact counter
              ++legTotalContactsCounter_[legIndex];
              // Print
              if (graphConfigPtr_->verboseLevel_ > 1) {
                REGULAR_COUT << " Leg " << legName << " came back into contact for the " << legTotalContactsCounter_[legIndex] << ". time."
                             << std::endl;
              }
            }

            // Add measurement ----------------------------
            // Position of foot in legged odometry frame
            Eigen::Vector3d O_t_O_foot = Eigen::Vector3d(contact.position.x, contact.position.y, contact.position.z);
            Eigen::Vector3d B_t_B_foot = T_O_B.inverse() * O_t_O_foot;

            // Create the unary measurement with contact counter
            std::string legIdentifier = legName;
            graph_msf::UnaryMeasurementXDLandmark<Eigen::Vector3d, 3> footContactPositionMeasurement(
                legIdentifier, measurementRate, leggedOdometryFrameName, leggedOdometryFrameName + sensorFrameCorrectedNameId,
                graph_msf::RobustNorm::None(), anymalStatePtr->header.stamp.toSec(), 1.0, B_t_B_foot,  // graph_msf::RobustNorm::Huber(1)
                legKinematicsFootPositionUnaryNoise_, worldFrame_);

            // Add to graph
            this->addUnaryPosition3LandmarkMeasurement(footContactPositionMeasurement, legTotalContactsCounter_[legIndex]);

            // Visualize foot contact in RViz
            visualization_msgs::Marker footContactMarker;
            createContactMarker(leggedOdometryFrameName, anymalStatePtr->header.stamp, B_t_B_foot, "foot_contact", legIndex,
                                Eigen::Vector3d(0.0, 1.0, 0.0), footContactMarker);
            footContactMarkers.markers.push_back(footContactMarker);
          }
        }
        // Leg not in contact ----------------------------
        else if (legInContactForNSteps_[legIndex] > 0) {
          // Set to 0
          legInContactForNSteps_[legIndex] = 0;
          // Print
          if (graphConfigPtr_->verboseLevel_ > 1) {
            REGULAR_COUT << " Leg " << legName << " lost contact. " << std::endl;
          }
          // Remove marker if it lost contact
          visualization_msgs::Marker deleteMarker;
          deleteMarker.header.frame_id = leggedOdometryFrameName;
          deleteMarker.header.stamp = anymalStatePtr->header.stamp;
          deleteMarker.ns = "foot_contact";
          deleteMarker.id = legIndex;
          deleteMarker.action = visualization_msgs::Marker::DELETE;
          footContactMarkers.markers.push_back(deleteMarker);
        }
      }

      for (std::size_t legIndex = 0; legIndex < legNames_.size(); ++legIndex) {
        if (seenLegs[legIndex] || legInContactForNSteps_[legIndex] <= 0) {
          continue;
        }

        legInContactForNSteps_[legIndex] = 0;

        visualization_msgs::Marker deleteMarker;
        deleteMarker.header.frame_id = leggedOdometryFrameName;
        deleteMarker.header.stamp = anymalStatePtr->header.stamp;
        deleteMarker.ns = "foot_contact";
        deleteMarker.id = static_cast<int>(legIndex);
        deleteMarker.action = visualization_msgs::Marker::DELETE;
        footContactMarkers.markers.push_back(deleteMarker);
      }

      // Publish foot contact markers
      if (!footContactMarkers.markers.empty() && pubFootContactMarkers_.getNumSubscribers() > 0) {
        pubFootContactMarkers_.publish(footContactMarkers);
      }
    }
  }
}

}  // namespace anymal_se
