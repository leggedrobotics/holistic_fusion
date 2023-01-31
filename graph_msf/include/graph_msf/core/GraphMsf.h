/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_H
#define GRAPH_MSF_H

// C++
#include <mutex>
#include <stdexcept>
#include <string_view>
#include <thread>

// Package
#include "graph_msf/config/GraphConfig.h"
#include "graph_msf/core/StaticTransforms.h"
#include "graph_msf/interface/NavState.h"
#include "graph_msf/measurements/BinaryMeasurement6D.h"
#include "graph_msf/measurements/UnaryMeasurement1D.h"
#include "graph_msf/measurements/UnaryMeasurement3D.h"
#include "graph_msf/measurements/UnaryMeasurement6D.h"

// Defined macros
#define ROS_QUEUE_SIZE 100
#define REQUIRED_GNSS_NUM_NOT_JUMPED 40          // 2*singleGnssJumping = 2*20 = 40
#define GNSS_COVARIANCE_VIOLATION_THRESHOLD 0.2  // 10000
#define GREEN_START "\033[92m"
#define YELLOW_START "\033[33m"
#define RED_START "\033[31m"
#define COLOR_END "\033[0m"

namespace graph_msf {

// Forward Declarations
class GraphManager;

// Actual Class
class GraphMsf {
 public:  // Interface
  // Constructor
  GraphMsf();
  // Destructor
  ~GraphMsf(){};

  // Setup
  bool setup();

  // Required Initialization
  bool initYawAndPosition(const double yaw_W_frame1, const std::string& frame1, const Eigen::Vector3d& W_t_W_frame2,
                          const std::string& frame2);
  bool initYawAndPosition(const Eigen::Matrix4d& T_O_frame, const std::string& frameName);
  bool initYawAndPosition(Eigen::Matrix4d T_O_I);
  bool areYawAndPositionInited();

  // Graph Manipulation
  void activateFallbackGraph();

  // Adder functions
  /// Return
  bool addImuMeasurement(const Eigen::Vector3d& linearAcc, const Eigen::Vector3d& angularVel, const double imuTimeK,
                         std::shared_ptr<NavState> returnPreIntegratedNavStatePtr);
  /// No return
  void addOdometryMeasurement(const BinaryMeasurement6D& delta);
  void addUnaryPoseMeasurement(const UnaryMeasurement6D& unary);
  void addDualOdometryMeasurement(const UnaryMeasurement6D& odometryKm1, const UnaryMeasurement6D& odometryK,
                                  const Eigen::Matrix<double, 6, 1>& poseBetweenNoise);
  void addDualGnssPositionMeasurement(const UnaryMeasurement3D& W_t_W_frame, const Eigen::Vector3d& lastPosition,
                                      const Eigen::Vector3d& estCovarianceXYZ, const bool attemptGraphSwitching, const bool addedYawBefore);
  void addGnssPositionMeasurement(const UnaryMeasurement3D& W_t_W_frame);
  void addGnssHeadingMeasurement(const UnaryMeasurement1D& yaw_W_frame);

  // Getters
  inline NavState getLatestPreintegratedNavState() {
    if (preIntegratedNavStatePtr_ != nullptr)
      return *preIntegratedNavStatePtr_;
    else
      throw std::runtime_error("GraphMsf::getLatestPreintegratedNavState: preIntegratedNavStatePtr_ is NULL");
    ;
  }
  inline NavStateWithCovarianceAndBias getLatestOptimizedNavStateWithCovarianceAndBias() {
    if (optimizedNavStateWithCovariancePtr_ != nullptr)
      return *optimizedNavStateWithCovariancePtr_;
    else
      throw std::runtime_error("GraphMsf::getLatestOptimizedNavStateWithCovariance: optimizedNavStateWithCovariancePtr is NULL");
    ;
  }

  bool getNormalOperationFlag() const { return normalOperationFlag_; }

 protected:  // Functions -------------
  /// Worker functions
  //// Set Imu Attitude
  bool alignImu_(double& imuAttitudeRoll, double& imuAttitudePitch);
  //// Initialize the graph
  void initGraph_(const double timeStamp_k);
  //// Updating the factor graph
  void optimizeGraph_();

  /// Utility functions
  //// Geometric transformation to IMU in world frame
  Eigen::Vector3d W_t_W_Frame1_to_W_t_W_Frame2_(const Eigen::Vector3d& W_t_W_frame1, const std::string& frame1, const std::string& frame2,
                                                const Eigen::Matrix3d& R_W_frame2);

  // Graph Config
  std::shared_ptr<GraphConfig> graphConfigPtr_ = nullptr;

  // Extrinsics
  std::shared_ptr<StaticTransforms> staticTransformsPtr_ = nullptr;

 private:  // Variables -------------
  // Threads
  std::thread optimizeGraphThread_;  /// Thread 5: Update of the graph as soon as new lidar measurement has arrived

  // Mutex
  std::mutex initYawAndPositionMutex_;
  std::mutex optimizeGraphMutex_;

  // Factor graph
  std::shared_ptr<GraphManager> graphMgrPtr_ = nullptr;

  /// Flags
  //// Initialization
  bool alignedImuFlag_ = false;
  bool foundInitialYawAndPositionFlag_ = false;
  bool initedGraphFlag_ = false;
  bool receivedOdometryFlag_ = false;
  //// During operation
  bool optimizeGraphFlag_ = false;
  bool gnssCovarianceViolatedFlag_ = false;
  bool normalOperationFlag_ = false;

  /// State Containers
  // Preintegrated NavState
  std::shared_ptr<NavState> preIntegratedNavStatePtr_ = NULL;
  // Optimized NavState with Covariance
  std::shared_ptr<NavStateWithCovarianceAndBias> optimizedNavStateWithCovariancePtr_ = NULL;
  /// Yaw
  double lastGnssYaw_W_I;

  /// Gravity
  double gravityConstant_ = 9.81;  // Will be overwritten

  // Counter
  long gnssNotJumpingCounter_ = REQUIRED_GNSS_NUM_NOT_JUMPED;
  long imuCabinCallbackCounter_ = 0;
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_H
