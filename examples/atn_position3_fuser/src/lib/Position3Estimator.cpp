/*
Copyright 2024 by Julian Nubert & Timon Mathis, Robotic Systems Lab & Autonomous Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "atn_position3_fuser/Position3Estimator.h"

// Project
#include "atn_position3_fuser/Position3StaticTransforms.h"
#include "atn_position3_fuser/constants.h"

// Workspace
#include "graph_msf_ros/util/conversions.h"
#include "graph_msf/interface/eigen_wrapped_gtsam_utils.h"


namespace position3_se {

Position3Estimator::Position3Estimator(std::shared_ptr<ros::NodeHandle> privateNodePtr) : graph_msf::GraphMsfRos(privateNodePtr) {
  REGULAR_COUT << GREEN_START << " Position3Estimator-Constructor called." << COLOR_END << std::endl;

  // Configurations ----------------------------
  // Static Transforms
  staticTransformsPtr_ = std::make_shared<Position3StaticTransforms>(
      privateNodePtr, constexprUsePrismPositionUnaryFlag_, constexprUseGnssPositionUnaryFlag_ || constexprUseGnssOfflinePoseUnaryFlag_);

  // GNSS Handler
  gnssHandlerPtr_ = std::make_shared<graph_msf::GnssHandler>();

  // Alignment Handler
  trajectoryAlignmentHandler_ = std::make_shared<graph_msf::TrajectoryAlignmentHandler>();

  // Setup
  Position3Estimator::setup();
}

//---------------------------------------------------------------
void Position3Estimator::setup() {
  REGULAR_COUT << GREEN_START << " Position3Estimator-Setup called." << COLOR_END << std::endl;

  // Read parameters ----------------------------
  Position3Estimator::readParams(privateNode);

  // Super class
  GraphMsfRos::setup(staticTransformsPtr_);

  // Publishers ----------------------------
  Position3Estimator::initializePublishers(privateNode);

  // Subscribers ----------------------------
  Position3Estimator::initializeSubscribers(privateNode);

  // Messages ----------------------------
  Position3Estimator::initializeMessages(privateNode);

  // Services ----------------------------
  Position3Estimator::initializeServices(privateNode);

  // Static Transforms
  staticTransformsPtr_->findTransformations();

  // Wrap up ----------------------------
  REGULAR_COUT << GREEN_START << " Set up successfully." << COLOR_END << std::endl;
}

//---------------------------------------------------------------
void Position3Estimator::initializePublishers(ros::NodeHandle& privateNode) {
  // Status
  REGULAR_COUT << GREEN_START << " Initializing Publishers..." << COLOR_END << std::endl;

  // Paths
  if constexpr (constexprUsePrismPositionUnaryFlag_) {
    pubMeasWorldPrismPositionPath_ = privateNode.advertise<nav_msgs::Path>("/graph_msf/measPosition_path_world_prism", ROS_QUEUE_SIZE);
  }
  if constexpr (constexprUseGnssPositionUnaryFlag_) {
    pubMeasWorldGnssPositionPath_ = privateNode.advertise<nav_msgs::Path>("/graph_msf/measPosition_path_world_gnss", ROS_QUEUE_SIZE);
  }
  if constexpr (constexprUseGnssOfflinePoseUnaryFlag_) {
    pubMeasWorldGnssOfflinePosePath_ =
        privateNode.advertise<nav_msgs::Path>("/graph_msf/measPosition_path_world_gnss_offline_pose", ROS_QUEUE_SIZE);
  }
}

void Position3Estimator::initializeSubscribers(ros::NodeHandle& privateNode) {
  // Prism Position
  if constexpr (constexprUsePrismPositionUnaryFlag_) {
    subPrismPosition_ = privateNode.subscribe<geometry_msgs::PointStamped>(
        "/prism_position_topic", ROS_QUEUE_SIZE, &Position3Estimator::prismPositionCallback_, this, ros::TransportHints().tcpNoDelay());
  }
  // GNSS
  if constexpr (constexprUseGnssPositionUnaryFlag_) {
    subGnssPosition_ = privateNode.subscribe<sensor_msgs::NavSatFix>(
        "/gnss_position_topic", ROS_QUEUE_SIZE, &Position3Estimator::gnssPositionCallback_, this, ros::TransportHints().tcpNoDelay());
  }
  // GNSS Offline Pose
  if constexpr (constexprUseGnssOfflinePoseUnaryFlag_) {
    subGnssOfflinePose_ =
        privateNode.subscribe<nav_msgs::Odometry>("/gnss_offline_pose_topic", ROS_QUEUE_SIZE, &Position3Estimator::gnssOfflinePoseCallback_,
                                                  this, ros::TransportHints().tcpNoDelay());
  }

  // Log
  std::cout << YELLOW_START << "FactorGraphFiltering" << COLOR_END << " Initialized Position subscriber (on position_topic)." << std::endl;
}

//---------------------------------------------------------------
void Position3Estimator::initializeMessages(ros::NodeHandle& privateNode) {
  // Status
  REGULAR_COUT << GREEN_START << " Initializing Messages..." << COLOR_END << std::endl;

  // Paths
  measPosition_worldPrismPositionPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  measPosition_worldGnssPositionPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  measPosition_worldGnssOfflinePosePathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
}

// --------------------------------------------------------------
void Position3Estimator::initializeServices(ros::NodeHandle& privateNode) {
  // Trigger Prism Unary
  if constexpr (constexprUsePrismPositionUnaryFlag_) {
    srvTogglePrismPositionUnary_ =
        privateNode.advertiseService("/atn_position3_fuser_node/togglePrismUnary", &Position3Estimator::srvTogglePrismUnaryCallback_, this);
  }
  // Trigger GNSS Unary
  if constexpr (constexprUseGnssPositionUnaryFlag_) {
    srvToggleGnssPositionUnary_ =
        privateNode.advertiseService("/atn_position3_fuser_node/toggleGnssUnary", &Position3Estimator::srvToggleGnssUnaryCallback_, this);
  }
  // Trigger GNSS Offline Pose Unary
  if constexpr (constexprUseGnssOfflinePoseUnaryFlag_) {
    srvToggleGnssOfflinePoseUnary_ = privateNode.advertiseService("/atn_position3_fuser_node/toggleGnssOfflinePoseUnary",
                                                                  &Position3Estimator::srvToggleGnssOfflinePoseUnaryCallback_, this);
  }
}

//---------------------------------------------------------------
void Position3Estimator::prismPositionCallback_(const geometry_msgs::PointStamped::ConstPtr& leicaPositionPtr) {
  // Counter
  prismPositionCallbackCounter_++;

  // Debug: Skip the first 40 seconds
  //  const int numSkipSteps = 40 * prismPositionRate_;
  //  if (prismPositionCallbackCounter_ < numSkipSteps) {
  //    if ((prismPositionCallbackCounter_ % 100) == 0) {
  //      REGULAR_COUT << " Skipping prism position measurement. Counter: " << prismPositionCallbackCounter_ << std::endl;
  //    }
  //    return;
  //  }

  // Translate to Eigen
  Eigen::Vector3d positionMeas = Eigen::Vector3d(leicaPositionPtr->point.x, leicaPositionPtr->point.y, leicaPositionPtr->point.z);
  Eigen::Vector3d positionCovarianceXYZ(prismPositionMeasUnaryNoise_, prismPositionMeasUnaryNoise_,
                                        prismPositionMeasUnaryNoise_);

  // Case: First call
  if (prismPositionCallbackCounter_ == 1) {
    REGULAR_COUT << " First prism position measurement received. Setting initial position to " << positionMeas.transpose() << std::endl;
    // Set initial position
    initialPrismPosition_ = positionMeas;
  }

  // Case: Not moved enough --> check if moved enough in meanwhile, only relevant if not initialized by GNSS
  if (!initializedByGnssFlag_ && !prismMovedEnoughFlag_) {
    if ((positionMeas - initialPrismPosition_).norm() > PRISM_MOVED_ENOUGH_THRESHOLD) {
      prismMovedEnoughFlag_ = true;
      REGULAR_COUT << " Prism moved enough from initial position of " << initialPrismPosition_.transpose() << " to "
                   << positionMeas.transpose() << std::endl;
    }
  }

  // Case: Graph was not initialized by this callback, but instead by GNSS
  if (initializedByGnssFlag_ && !alignedPrismAndGnssFlag_) {
    trajectoryAlignmentHandler_->addR3Position(positionMeas, leicaPositionPtr->header.stamp.toSec());
    // In radians
    double yaw{0};
    if (!(trajectoryAlignmentHandler_->alignTrajectories(yaw, T_enu_totalStation_))) {
      if (prismPositionCallbackCounter_ % 10 == 0) {
        REGULAR_COUT << " Graph was initialized by GNSS, hence trying to align. Trajectory alignment not ready. Waiting for more motion."
                     << std::endl;
      }
    } else {
      T_enu_totalStation_ = T_enu_totalStation_.inverse();
      // Only keep yaw of the orientation part of the transformation
      REGULAR_COUT << GREEN_START << " ENU to Total Station Transformation: " << COLOR_END << T_enu_totalStation_.matrix() << std::endl;
      graph_msf::inPlaceRemoveRollPitch(T_enu_totalStation_);
      REGULAR_COUT << GREEN_START << " ENU to Total Station Transformation after removing roll and pitch: " << COLOR_END << T_enu_totalStation_.matrix() << std::endl;
      // Prepare for using this transformation in the graph
      alignedPrismAndGnssFlag_ = true;
    }
    return;  // Do not add unary measurement this round
  }

  // State Machine
  if (!areYawAndPositionInited() && areRollAndPitchInited()) {
    // Try to initialize yaw and position if not done already
    if (this->initYawAndPositionInWorld(
            0.0, positionMeas, staticTransformsPtr_->getBaseLinkFrame(),
            dynamic_cast<Position3StaticTransforms*>(staticTransformsPtr_.get())->getPrismPositionMeasFrame())) {
      REGULAR_COUT << " Set yaw and position successfully." << std::endl;
    } else {
      REGULAR_COUT << " Could not set yaw and position." << std::endl;
    }
  }
  // Else if active
  else if (usePrismPositionUnaryFlag_) {
    const std::string& positionMeasFrame =
        dynamic_cast<Position3StaticTransforms*>(staticTransformsPtr_.get())->getPrismPositionMeasFrame();
    std::string fixedFrame = staticTransformsPtr_->getWorldFrame();
    // Consider whether graph was initialized by GNSS or not ----------------------------
    //    if (initializedByGnssFlag_) {
    //      fixedFrame = leicaPositionPtr->header.frame_id;
    //    } else {
    //      fixedFrame = staticTransformsPtr_->getWorldFrame();
    //    }
    // Potentially transform measurement to start ENU frame (if world origin was set to coincide with GNSS)
    if (alignedPrismAndGnssFlag_) {
      positionMeas = T_enu_totalStation_ * positionMeas;
    }

    // Already initialized --> add position measurement to graph ----------------------------
    graph_msf::UnaryMeasurementXDAbsolute<Eigen::Vector3d, 3> meas_W_t_W_P(
        "LeicaPosition", int(prismPositionRate_), positionMeasFrame, positionMeasFrame + sensorFrameCorrectedNameId,
        graph_msf::RobustNorm::None(), leicaPositionPtr->header.stamp.toSec(), POS_COVARIANCE_VIOLATION_THRESHOLD, positionMeas,
        positionCovarianceXYZ, fixedFrame, staticTransformsPtr_->getWorldFrame(), initialSe3AlignmentNoise_, prismSe3AlignmentRandomWalk_);
    this->addUnaryPosition3AbsoluteMeasurement(meas_W_t_W_P);
  }
  // Else if not active
  else if ((prismPositionCallbackCounter_ % 10) == 0) {
    REGULAR_COUT << " Prism unary measurement is turned off." << std::endl;
  }

  // Visualizations
  addToPathMsg(measPosition_worldPrismPositionPathPtr_, staticTransformsPtr_->getWorldFrame(), leicaPositionPtr->header.stamp, positionMeas,
               graphConfigPtr_->imuBufferLength_ * 4);
  pubMeasWorldPrismPositionPath_.publish(measPosition_worldPrismPositionPathPtr_);
}

//---------------------------------------------------------------
void Position3Estimator::gnssPositionCallback_(const sensor_msgs::NavSatFix::ConstPtr& gnssPositionPtr) {
  // Counter
  gnssPositionCallbackCounter_++;

  // If prism has not moved enough, do not add GNSS measurements
  if (!prismMovedEnoughFlag_ && !initializedByGnssFlag_) {
    if ((gnssPositionCallbackCounter_ % 100) == 0) {
      REGULAR_COUT << " PRISM HAS NOT MOVED ENOUGH YET! Not adding GNSS measurements." << std::endl;
    }
    //    return;
  }

  // Translate to Eigen
  Eigen::Vector3d gnssCoord = Eigen::Vector3d(gnssPositionPtr->latitude, gnssPositionPtr->longitude, gnssPositionPtr->altitude);
  Eigen::Vector3d estStdDevXYZ(sqrt(gnssPositionPtr->position_covariance[0]), sqrt(gnssPositionPtr->position_covariance[4]),
                               sqrt(gnssPositionPtr->position_covariance[8]));

  // Initialize GNSS Handler
  if (gnssPositionCallbackCounter_ < NUM_GNSS_CALLBACKS_UNTIL_START) {  // Accumulate measurements
    // Wait until measurements got accumulated
    accumulatedGnssCoordinates_ += gnssCoord;
    if ((gnssPositionCallbackCounter_ % 10) == 0) {
      REGULAR_COUT << " NOT ENOUGH GNSS MESSAGES ARRIVED!" << std::endl;
    }
    return;
  } else if (gnssPositionCallbackCounter_ == NUM_GNSS_CALLBACKS_UNTIL_START) {  // Initialize GNSS Handler
    gnssHandlerPtr_->initHandler(accumulatedGnssCoordinates_ / NUM_GNSS_CALLBACKS_UNTIL_START);
    REGULAR_COUT << " GNSS Handler initialized." << std::endl;
    return;
  }

  // Convert to Cartesian Coordinates
  Eigen::Vector3d W_t_W_Gnss;
  gnssHandlerPtr_->convertNavSatToPosition(gnssCoord, W_t_W_Gnss);
  // std::string fixedFrame = staticTransformsPtr_->getWorldFrame();
  std::string fixedFrame = gnssPositionPtr->header.frame_id;

  // Regular Operation
  // Initialize if needed and no prism is used
  if (!areYawAndPositionInited() && areRollAndPitchInited() && !constexprUsePrismPositionUnaryFlag_) {
    // Try to initialize yaw and position if not done already
    if (this->initYawAndPositionInWorld(0.0, W_t_W_Gnss, staticTransformsPtr_->getBaseLinkFrame(),
                                        dynamic_cast<Position3StaticTransforms*>(staticTransformsPtr_.get())->getGnssPositionMeasFrame())) {
      REGULAR_COUT << " GNSS set yaw and position successfully, as there is no prism." << std::endl;
      initializedByGnssFlag_ = true;
    } else {
      REGULAR_COUT << " Could not set yaw and position." << std::endl;
    }
  }
  // Else if inited or prism is used
  else if (areRollAndPitchInited() && useGnssPositionUnaryFlag_) {
    const std::string& positionMeasFrame = dynamic_cast<Position3StaticTransforms*>(staticTransformsPtr_.get())->getGnssPositionMeasFrame();
    // Already initialized --> add position measurement to graph
    graph_msf::UnaryMeasurementXDAbsolute<Eigen::Vector3d, 3> meas_W_t_W_P(
        "GnssPosition", int(gnssPositionRate_), positionMeasFrame, positionMeasFrame + sensorFrameCorrectedNameId,
        graph_msf::RobustNorm::None(), gnssPositionPtr->header.stamp.toSec(), POS_COVARIANCE_VIOLATION_THRESHOLD, W_t_W_Gnss, estStdDevXYZ,
        fixedFrame, staticTransformsPtr_->getWorldFrame(), initialSe3AlignmentNoise_, gnssSe3AlignmentRandomWalk_);
    this->addUnaryPosition3AbsoluteMeasurement(meas_W_t_W_P);
  }
  // Status
  else if ((gnssPositionCallbackCounter_ % 10) == 0) {
    REGULAR_COUT << " GNSS unary measurement is turned off or roll/pitch not yet inited." << std::endl;
  }

  // Adjust frame name
  if (fixedFrame != staticTransformsPtr_->getWorldFrame()) {
    fixedFrame += referenceFrameAlignedNameId;
  }

  // Visualizations
  addToPathMsg(measPosition_worldGnssPositionPathPtr_, fixedFrame, gnssPositionPtr->header.stamp, W_t_W_Gnss,
               graphConfigPtr_->imuBufferLength_ * 4);
  // Publish Path
  pubMeasWorldGnssPositionPath_.publish(measPosition_worldGnssPositionPathPtr_);
}

//---------------------------------------------------------------
void Position3Estimator::gnssOfflinePoseCallback_(const nav_msgs::Odometry::ConstPtr& gnssOfflinePosePtr) {
  // Counter
  gnssOfflinePoseCallbackCounter_++;

  // If prism has not moved enough, do not add GNSS measurements
  bool addToOnlineSmootherFlag = true;
  if (!prismMovedEnoughFlag_ && !initializedByGnssFlag_) {
    if ((gnssOfflinePoseCallbackCounter_ % 100) == 0) {
      REGULAR_COUT << " PRISM HAS NOT MOVED ENOUGH YET! Not adding GNSS measurements to online graph." << std::endl;
    }
    addToOnlineSmootherFlag = false;
  }

  // Prepare Data
  Eigen::Isometry3d T_ENU_Gk = Eigen::Isometry3d::Identity();
  graph_msf::odomMsgToEigen(*gnssOfflinePosePtr, T_ENU_Gk.matrix());
  double gnssUnaryTimeK = gnssOfflinePosePtr->header.stamp.toSec();

  // If initialized by gnss --> make sure we can do the alignment
  if (initializedByGnssFlag_ && !alignedPrismAndGnssFlag_) {
    trajectoryAlignmentHandler_->addSe3Position(T_ENU_Gk.translation(), gnssOfflinePosePtr->header.stamp.toSec());
  }

  // Get Uncertainty from message
  auto estGnssOfflinePoseMeasUnaryNoiseList = gnssOfflinePosePtr->pose.covariance;
  Eigen::Matrix<double, 6, 1> estGnssOfflinePoseMeasUnaryNoise;
  estGnssOfflinePoseMeasUnaryNoise << sqrt(estGnssOfflinePoseMeasUnaryNoiseList[0]), sqrt(estGnssOfflinePoseMeasUnaryNoiseList[7]),
      sqrt(estGnssOfflinePoseMeasUnaryNoiseList[14]), sqrt(estGnssOfflinePoseMeasUnaryNoiseList[21]),
      sqrt(estGnssOfflinePoseMeasUnaryNoiseList[28]), sqrt(estGnssOfflinePoseMeasUnaryNoiseList[35]);

  // Fixed Frame
  std::string fixedFrame = gnssOfflinePosePtr->header.frame_id;

  // Measurement
  const std::string& gnssFrameName =
      dynamic_cast<Position3StaticTransforms*>(staticTransformsPtr_.get())->getGnssOfflinePoseMeasFrame();  // alias
  graph_msf::UnaryMeasurementXDAbsolute<Eigen::Isometry3d, 6> unary6DMeasurement(
      "Lidar_unary_6D", int(gnssOfflinePoseRate_), gnssFrameName, gnssFrameName + sensorFrameCorrectedNameId, graph_msf::RobustNorm::None(),
      gnssUnaryTimeK, 1.0, T_ENU_Gk, gnssOfflinePoseMeasUnaryNoise_, fixedFrame, staticTransformsPtr_->getWorldFrame(),
      initialSe3AlignmentNoise_, gnssSe3AlignmentRandomWalk_);

  // Wait until enough measurements have arrived
  if (gnssOfflinePoseCallbackCounter_ <= 2) {
    return;
  }
  // Initialize if needed and no prism is used
  else if (!areYawAndPositionInited() && areRollAndPitchInited() &&
           (!usePrismPositionUnaryFlag_ ||
            gnssOfflinePoseCallbackCounter_ > NUM_GNSS_CALLBACKS_UNTIL_YAW_INIT)) {  // Initializing if no GNSS
    REGULAR_COUT << RED_START << " GNSS offline odometry callback is setting global yaw, as it was not set so far." << COLOR_END
                 << std::endl;
    this->initYawAndPosition(unary6DMeasurement);
    initializedByGnssFlag_ = true;
  }
  // Otherwise just add measurement
  else {  // Already initialized --> unary factor
    this->addUnaryPose3AbsoluteMeasurement(unary6DMeasurement, addToOnlineSmootherFlag);
  }

  // Visualization ----------------------------
  // Add to path message
  addToPathMsg(measPosition_worldGnssOfflinePosePathPtr_, fixedFrame, gnssOfflinePosePtr->header.stamp, T_ENU_Gk.translation(),
               graphConfigPtr_->imuBufferLength_ * 4);

  // Publish Path
  pubMeasWorldGnssOfflinePosePath_.publish(measPosition_worldGnssOfflinePosePathPtr_);
}

//---------------------------------------------------------------
bool Position3Estimator::srvTogglePrismUnaryCallback_(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  // Toggle Flag
  usePrismPositionUnaryFlag_ = !usePrismPositionUnaryFlag_;

  // Response
  res.success = true;
  res.message = "Prism unary measurement toggled " + std::string(usePrismPositionUnaryFlag_ ? "on" : "off") + ".";

  return true;
}

//---------------------------------------------------------------
bool Position3Estimator::srvToggleGnssUnaryCallback_(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  // Toggle Flag
  useGnssPositionUnaryFlag_ = !useGnssPositionUnaryFlag_;

  // Response
  res.success = true;
  res.message = "GNSS unary measurement toggled " + std::string(useGnssPositionUnaryFlag_ ? "on" : "off") + ".";

  return true;
}

//---------------------------------------------------------------
bool Position3Estimator::srvToggleGnssOfflinePoseUnaryCallback_(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  // Toggle Flag
  useGnssOfflinePoseUnaryFlag_ = !useGnssOfflinePoseUnaryFlag_;

  // Response
  res.success = true;
  res.message = "GNSS offline pose unary measurement toggled " + std::string(useGnssOfflinePoseUnaryFlag_ ? "on" : "off") + ".";

  return true;
}

}  // namespace position3_se