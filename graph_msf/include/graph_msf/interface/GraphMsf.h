/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MSF_H
#define GRAPH_MSF_H

// C++
#include <mutex>
#include <stdexcept>
#include <thread>

// Package
#include "graph_msf/config/GraphConfig.h"
#include "graph_msf/config/StaticTransforms.h"
#include "graph_msf/imu/ImuBuffer.hpp"
#include "graph_msf/interface/NavState.h"
#include "graph_msf/measurements/BinaryMeasurementXD.h"
#include "graph_msf/measurements/UnaryMeasurementXD.h"

// Defined macros
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
  virtual ~GraphMsf() = default;
  // Setup
  virtual bool setup();

  // Initialization Interface
  bool initYawAndPosition(const double yaw_fixedFrame_frame1, const Eigen::Vector3d& fixedFrame_t_fixedFrame_frame2,
                          const std::string& fixedFrame, const std::string& frame1, const std::string& frame2);
  bool initYawAndPosition(const UnaryMeasurementXD<Eigen::Isometry3d, 6>& unary6DMeasurement);
  bool areYawAndPositionInited();
  bool areRollAndPitchInited();

  // Trigger offline smoother optimization
  bool optimizeSlowBatchSmoother();

  // Adder functions
  /// Return
  bool addImuMeasurementAndGetState(const Eigen::Vector3d& linearAcc, const Eigen::Vector3d& angularVel, const double imuTimeK,
                                    std::shared_ptr<SafeIntegratedNavState>& returnPreIntegratedNavStatePtr,
                                    std::shared_ptr<SafeNavStateWithCovarianceAndBias>& returnOptimizedStateWithCovarianceAndBiasPtr,
                                    Eigen::Matrix<double, 6, 1>& returnAddedImuMeasurements);
  /// No return
  void addOdometryMeasurement(const BinaryMeasurementXD<Eigen::Isometry3d, 6>& delta);
  void addUnaryPoseMeasurement(const UnaryMeasurementXD<Eigen::Isometry3d, 6>& unary);
  bool addPositionMeasurement(UnaryMeasurementXD<Eigen::Vector3d, 3>& W_t_W_frame);
  bool addHeadingMeasurement(const UnaryMeasurementXD<double, 1>& yaw_W_frame);
  bool addZeroMotionFactor(double timeKm1, double timeK);

  bool getNormalOperationFlag() const { return normalOperationFlag_; }

 protected:
  // Methods -------------
  /// Worker functions
  //// Set Imu Attitude
  bool alignImu_(double& imuAttitudeRoll, double& imuAttitudePitch);
  //// Initialize the graph
  void initGraph_(const double timeStamp_k);
  //// Updating the factor graph
  void optimizeGraph_();
  //// GNSS Violation
  template <int DIM>
  bool isCovarianceViolated_(const Eigen::Matrix<double, DIM, 1>& gnssCovarianceXYZ, const double covarianceViolationThreshold);

  /// Utility functions
  //// Geometric transformation to IMU in world frame
  Eigen::Vector3d W_t_W_Frame1_to_W_t_W_Frame2_(const Eigen::Vector3d& W_t_W_frame1, const std::string& frame1, const std::string& frame2,
                                                const Eigen::Matrix3d& R_W_frame2);

  // Graph Config
  std::shared_ptr<GraphConfig> graphConfigPtr_ = nullptr;

  // Extrinsics
  std::shared_ptr<StaticTransforms> staticTransformsPtr_ = nullptr;

  // Initialization
  void pretendFirstMeasurementReceived();

 private:  // Variables -------------
  // Threads
  std::thread optimizeGraphThread_;  /// Thread 5: Update of the graph as soon as
                                     /// new lidar measurement has arrived

  // Mutex
  std::mutex initYawAndPositionMutex_;
  std::mutex optimizeGraphMutex_;

  // Factor graph
  std::shared_ptr<GraphManager> graphMgrPtr_ = nullptr;
  // Imu Buffer
  std::shared_ptr<graph_msf::ImuBuffer> imuBufferPtr_;

  /// Flags
  //// Initialization
  bool alignedImuFlag_ = false;
  bool foundInitialYawAndPositionFlag_ = false;
  bool initedGraphFlag_ = false;
  bool validFirstMeasurementReceivedFlag_ = false;
  //// During operation
  bool optimizeGraphFlag_ = false;
  bool normalOperationFlag_ = false;

  /// State Containers
  // Preintegrated NavState
  std::shared_ptr<SafeIntegratedNavState> preIntegratedNavStatePtr_ = nullptr;

  // Counter
  long imuCallbackCounter_ = 0;
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_H
