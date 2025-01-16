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
#include <unordered_set>

// Package
#include "graph_msf/config/GraphConfig.h"
#include "graph_msf/config/StaticTransforms.h"
#include "graph_msf/imu/ImuBuffer.hpp"
#include "graph_msf/interface/NavState.h"
#include "graph_msf/measurements/BinaryMeasurementXD.h"
#include "graph_msf/measurements/UnaryMeasurementXDAbsolute.h"
#include "graph_msf/measurements/UnaryMeasurementXDLandmark.h"

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
  void setup(const std::shared_ptr<GraphConfig> graphConfigPtr, const std::shared_ptr<StaticTransforms> staticTransformsPtr);

  // Initialization Interface
  bool initYawAndPositionInWorld(const double yaw_W_frame1, const Eigen::Vector3d& W_t_W_frame2, const std::string& frame1,
                                 const std::string& frame2);
  bool initYawAndPosition(const UnaryMeasurementXD<Eigen::Isometry3d, 6>& unary6DMeasurement);

  // Trigger offline smoother optimization
  bool optimizeSlowBatchSmoother(int maxIterations, const std::string& savePath, const bool saveCovarianceFlag);

  // Logging of the real-time states
  bool logRealTimeStates(const std::string& savePath);

  // Getter functions
  bool areYawAndPositionInited() const;
  bool areRollAndPitchInited() const;
  bool isGraphInited() const;
  bool getNormalOperationFlag() const { return normalOperationFlag_; }

  // Main: IMU
  bool addCoreImuMeasurementAndGetState(const Eigen::Vector3d& linearAcc, const Eigen::Vector3d& angularVel, const double imuTimeK,
                                        std::shared_ptr<SafeIntegratedNavState>& returnPreIntegratedNavStatePtr,
                                        std::shared_ptr<SafeNavStateWithCovarianceAndBias>& returnOptimizedStateWithCovarianceAndBiasPtr,
                                        Eigen::Matrix<double, 6, 1>& returnAddedImuMeasurements);

  // Pure Virtual Methods
  /// Unary Measurements
  //// Absolute Measurements
  virtual void addUnaryPose3AbsoluteMeasurement(const UnaryMeasurementXDAbsolute<Eigen::Isometry3d, 6>& R_T_R_S,
                                                const bool addToOnlineSmootherFlag = true) = 0;
  virtual void addUnaryPosition3AbsoluteMeasurement(UnaryMeasurementXDAbsolute<Eigen::Vector3d, 3>& R_t_R_S) = 0;
  virtual void addUnaryVelocity3AbsoluteMeasurement(UnaryMeasurementXDAbsolute<Eigen::Vector3d, 3>& R_v_R_S) = 0;
  virtual void addUnaryRollAbsoluteMeasurement(const UnaryMeasurementXDAbsolute<double, 1>& roll_R_S) = 0;
  virtual void addUnaryPitchAbsoluteMeasurement(const UnaryMeasurementXDAbsolute<double, 1>& pitch_R_S) = 0;
  virtual void addUnaryYawAbsoluteMeasurement(const UnaryMeasurementXDAbsolute<double, 1>& yaw_R_S) = 0;
  //// Local Measurements
  virtual void addUnaryVelocity3LocalMeasurement(UnaryMeasurementXD<Eigen::Vector3d, 3>& S_v_R_S) = 0;

  /// Landmark Measurements
  virtual void addUnaryPosition3LandmarkMeasurement(UnaryMeasurementXDLandmark<Eigen::Vector3d, 3>& S_t_S_L,
                                                    const int landmarkCreationCounter) = 0;
  virtual void addUnaryBearing3LandmarkMeasurement(UnaryMeasurementXDLandmark<Eigen::Vector3d, 3>& S_bearing_S_L) = 0;

  /// Binary Measurements
  virtual void addBinaryPose3Measurement(const BinaryMeasurementXD<Eigen::Isometry3d, 6>& delta) = 0;

  // Ambiguous Measurements
  bool addZeroMotionFactor(double timeKm1, double timeK, double noiseDensity);
  bool addZeroVelocityFactor(double timeK, double noiseDensity);

 protected:
  // Methods -------------
  /// Convenience functions
  template <int DIM>
  bool isCovarianceViolated_(const Eigen::Matrix<double, DIM, 1>& covariance, const double covarianceViolationThreshold) {
    if (covarianceViolationThreshold > 0.0) {
      for (int i = 0; i < DIM; i++) {
        if (covariance(i) > covarianceViolationThreshold) {
          return true;
        }
      }
    }
    return false;
  }

  // Initialization
  void pretendFirstMeasurementReceived();

  // Check whether measurement violated covariance, if yes, add to set and print once, if not, remove from set and print that returned
  bool checkAndPrintCovarianceViolation_(const std::string& measurementName, const bool violatedFlag);

  // Members
  // Graph Config
  std::shared_ptr<GraphConfig> graphConfigPtr_ = nullptr;
  // Extrinsics
  std::shared_ptr<StaticTransforms> staticTransformsPtr_ = nullptr;

  // Factor graph
  std::shared_ptr<GraphManager> graphMgrPtr_ = nullptr;
  // Imu Buffer
  std::shared_ptr<graph_msf::ImuBuffer> coreImuBufferPtr_;

  // Mutex
  std::mutex optimizeGraphMutex_;

  /// Flags
  //// Initialization
  bool alignedImuFlag_ = false;
  bool foundInitialYawAndPositionFlag_ = false;
  bool initedGraphFlag_ = false;
  bool validFirstMeasurementReceivedFlag_ = false;
  //// During operation
  bool optimizeGraphFlag_ = false;
  bool normalOperationFlag_ = false;

  // Counter
  long imuCallbackCounter_ = 0;

  // Set of measurements that have violated the covariance
  std::unordered_set<std::string> measurementsWithViolatedCovariance_;

 private:
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

  // Threads
  std::thread optimizeGraphThread_;  /// Thread 5: Update of the graph as soon as
                                     /// new lidar measurement has arrived

  // Mutex
  std::mutex initYawAndPositionMutex_;

  /// State Containers
  std::shared_ptr<SafeIntegratedNavState> preIntegratedNavStatePtr_ = nullptr;
};

}  // namespace graph_msf

#endif  // GRAPH_MSF_H
