#ifndef FG_FILTERING_H
#define FG_FILTERING_H

// C++
#include <mutex>
#include <stdexcept>
#include <string_view>
#include <thread>

// Package
#include "StaticTransforms.h"
#include "compslam_se/GraphManager.hpp"
#include "compslam_se/InterfacePrediction.h"
#include "compslam_se/config/GraphConfig.h"
#include "compslam_se/geometry/math_utils.h"
#include "compslam_se/measurements/DeltaMeasurement6D.h"
#include "compslam_se/measurements/UnaryMeasurement6D.h"

// Defined macros
#define ROS_QUEUE_SIZE 100
#define REQUIRED_GNSS_NUM_NOT_JUMPED 20
#define GNSS_COVARIANCE_VIOLATION_THRESHOLD 0.1  // 10000
#define GREEN_START "\033[92m"
#define YELLOW_START "\033[33m"
#define COLOR_END "\033[0m"

namespace compslam_se {

/** \brief Implementation of the factor graph based filtering.
 *
 */
class CompslamSe {
 public:
  // Constructor
  CompslamSe();
  // Destructor
  ~CompslamSe(){};

  // Setup
  bool setup(GraphConfig* graphConfigPtr, StaticTransforms* staticTransformsPtr);

  // Required Initialization
  bool initYawAndPosition(const double yaw_W_frame1, const std::string& frame1, const Eigen::Vector3d& t_W_frame2,
                          const std::string& frame2);
  bool initYawAndPosition(Eigen::Matrix4d T_O_I);
  bool yawAndPositionInited();

  // Graph Manipulation
  void activateFallbackGraph();

  // Adderfunctions
  /// Return
  bool addImuMeasurement(const Eigen::Vector3d& linearAcc, const Eigen::Vector3d& angularVel, const double imuTimeK,
                         std::shared_ptr<InterfacePrediction>& predictionPtr);
  /// No return
  void addOdometryMeasurement(const DeltaMeasurement6D& delta);
  void addUnaryPoseMeasurement(const UnaryMeasurement6D& unary);
  void addOdometryMeasurement(const UnaryMeasurement6D& odometryKm1, const UnaryMeasurement6D& odometryK,
                              const Eigen::Matrix<double, 6, 1>& poseBetweenNoise);
  void addGnssPositionMeasurement(const Eigen::Vector3d& position, const Eigen::Vector3d& lastPosition,
                                  const Eigen::Vector3d& covarianceXYZ, const double gnssTimeK, const double rate,
                                  double positionUnaryNoise);
  void addGnssHeadingMeasurement(const double yaw_W_frame, const std::string& frameName, const double gnssTimeK, const double rate,
                                 double headingUnaryNoise);

  // Getters
  bool getLogPlots() { return logPlots_; }
  void getLatestOptimizedState(Eigen::Matrix4d& optState, double& time) {
    time = optTime_;
    optState = T_W_I_opt_.matrix();
  }

 protected:
  // Methods -------------
  /// Worker functions
  //// Set Imu Attitude
  bool alignImu_();
  //// Initialize the graph
  void initGraph_(const double timeStamp_k);
  //// Updating the factor graph
  void optimizeGraph_();

  /// Utility functions
  //// Geometric transformation to IMU in world frame
  gtsam::Vector3 transformLeftGnssPointToImuFrame_(const gtsam::Point3& t_W_GnssL, const gtsam::Rot3& R_W_I);
  //// Get the robot heading from the two Gnss positions
  static gtsam::Point3 getRobotHeading_(const Eigen::Vector3d& leftPosition, const Eigen::Vector3d& rightPosition);

  // Threads
  std::thread optimizeGraphThread_;  /// Thread 5: Update of the graph as soon as new lidar measurement has arrived

  // Mutex
  std::mutex initYawAndPositionMutex_;
  std::mutex optimizeGraphMutex_;

  // Factor graph
  GraphManager* graphMgrPtr_ = NULL;

  // Graph Config
  GraphConfig* graphConfigPtr_ = NULL;
  StaticTransforms* staticTransformsPtr_ = NULL;

  /// Flags
  //// Configuration
  bool usingFallbackGraphFlag_ = true;

  //// Initialization
  bool alignedImuFlag_ = false;
  bool foundInitialYawAndPositionFlag_ = false;
  bool initedGraphFlag_ = false;
  bool receivedOdometryFlag_ = false;
  //// During operation
  bool optimizeGraphFlag_ = false;
  bool gnssCovarianceViolatedFlag_ = false;

  /// Times
  double imuTimeKm1_;
  double imuTimeOffset_ = 0.0;

  /// Transformations with timestamps
  /// Pose
  gtsam::Pose3 T_W_Ik_;
  gtsam::Pose3 T_W_O_;
  /// Velocity
  gtsam::Vector3 I_v_W_I_;
  gtsam::Vector3 I_w_W_I_;
  /// Timestamp
  double imuTimeK_;
  /// Other
  gtsam::Pose3 T_W_I0_;  // Initial IMU pose (in graph)
  gtsam::Pose3 T_W_I_opt_;
  double optTime_;

  /// Attitudes
  double gravityConstant_ = 9.81;  // Will be overwritten
  double yaw_W_I0_;
  gtsam::Vector3 W_t_W_I0_;
  double imuAttitudePitch_;
  double imuAttitudeRoll_;

  /// Counter
  long gnssCallbackCounter_ = 0;
  int gnssNotJumpingCounter_ = 0;

  /// Logging
  bool logPlots_ = false;
};

}  // end namespace compslam_se

#endif  // FG_FILTERING_H
