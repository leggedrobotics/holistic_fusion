/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
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

  // Trigger offline smoother optimization
  bool optimizeSlowBatchSmoother(int maxIterations, const std::string& savePath);

  // Getter functions
  bool areYawAndPositionInited() const;
  bool areRollAndPitchInited() const;
  bool isGraphInited() const;
  bool getNormalOperationFlag() const { return normalOperationFlag_; }

  // Adder functions
  /// Main: IMU
  bool addImuMeasurementAndGetState(const Eigen::Vector3d& linearAcc, const Eigen::Vector3d& angularVel, const double imuTimeK,
                                    std::shared_ptr<SafeIntegratedNavState>& returnPreIntegratedNavStatePtr,
                                    std::shared_ptr<SafeNavStateWithCovarianceAndBias>& returnOptimizedStateWithCovarianceAndBiasPtr,
                                    Eigen::Matrix<double, 6, 1>& returnAddedImuMeasurements);
  /// Unary Measurements
  void addUnaryPose3Measurement(const UnaryMeasurementXD<Eigen::Isometry3d, 6>& F_T_F_S);
  void addUnaryPosition3Measurement(UnaryMeasurementXD<Eigen::Vector3d, 3>& F_t_F_S);
  void addUnaryVelocity3Measurement(UnaryMeasurementXD<Eigen::Vector3d, 3>& F_v_F_S);
  void addUnaryVelocity3BodyMeasurement(UnaryMeasurementXD<Eigen::Vector3d, 3>& S_v_F_S);
  bool addUnaryRollMeasurement(const UnaryMeasurementXD<double, 1>& roll_F_S);
  bool addUnaryPitchMeasurement(const UnaryMeasurementXD<double, 1>& pitch_F_S);
  bool addUnaryYawMeasurement(const UnaryMeasurementXD<double, 1>& yaw_F_S);

  /// Binary Measurements
  void addBinaryPoseMeasurement(const BinaryMeasurementXD<Eigen::Isometry3d, 6>& delta);

  /// Ambiguous Measurements
  bool addZeroMotionFactor(double timeKm1, double timeK, double noiseDensity);
  bool addZeroVelocityFactor(double timeK, double noiseDensity);

 protected:
  // Methods -------------
  /// Worker functions
  //// Set Imu Attitude
  bool alignImu_(double& imuAttitudeRoll, double& imuAttitudePitch);
  //// Initialize the graph
  void initGraph_(const double timeStamp_k);
  //// Updating the factor graph
  void optimizeGraph_();

  /// Convenience functions
  template <int DIM>
  bool isCovarianceViolated_(const Eigen::Matrix<double, DIM, 1>& gnssCovarianceXYZ, const double covarianceViolationThreshold);

  /// Utility functions
  //// Geometric transformation to IMU in world frame
  Eigen::Vector3d W_t_W_Frame1_to_W_t_W_Frame2_(const Eigen::Vector3d& W_t_W_frame1, const std::string& frame1, const std::string& frame2,
                                                const Eigen::Matrix3d& R_W_frame2);

  // Initialization
  void pretendFirstMeasurementReceived();

  // Members
  // Graph Config
  std::shared_ptr<GraphConfig> graphConfigPtr_ = nullptr;
  // Extrinsics
  std::shared_ptr<StaticTransforms> staticTransformsPtr_ = nullptr;

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
