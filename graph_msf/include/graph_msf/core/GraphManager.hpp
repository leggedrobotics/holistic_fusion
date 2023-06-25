/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MANAGER_HPP_
#define GRAPH_MANAGER_HPP_

#define GREEN_START "\033[92m"
#define YELLOW_START "\033[33m"
#define RED_START "\033[31m"
#define COLOR_END "\033[0m"

// C++
#include <chrono>
#include <mutex>
#include <vector>

// GTSAM
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearISAM.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

// Factors
#include <gtsam/navigation/CombinedImuFactor.h>

// Package
#include "graph_msf/config/GraphConfig.h"
#include "graph_msf/core/GraphState.hpp"
#include "graph_msf/factors/HeadingFactor.h"
#include "graph_msf/frontend/NavState.h"
#include "graph_msf/imu/ImuBuffer.hpp"
#include "graph_msf/measurements/UnaryMeasurement6D.h"

namespace graph_msf {

class GraphManager {
 public:
  GraphManager(std::shared_ptr<GraphConfig> graphConfigPtr);
  ~GraphManager(){};

  // Change Graph
  bool initImuIntegrators(const double g);
  bool initPoseVelocityBiasGraph(const double ts, const gtsam::Pose3& init_pose);
  gtsam::NavState addImuFactorAndGetState(const double imuTimeK, const Eigen::Vector3d& linearAcc, const Eigen::Vector3d& angularVel,
                                          bool& relocalizationFlag);
  gtsam::Key addPoseBetweenFactorToGlobalGraph(const double lidarTimeKm1, const double lidarTimeK, const double rate,
                                               const Eigen::Matrix<double, 6, 1>& poseBetweenNoise, const gtsam::Pose3& pose,
                                               const std::string& measurementType);
  void addPoseUnaryFactorToGlobalGraph(const double lidarTimeK, const double rate, const Eigen::Matrix<double, 6, 1>& poseUnaryNoise,
                                       const gtsam::Pose3& unaryPose);
  void addPoseUnaryFactorToFallbackGraph(const double lidarTimeK, const double rate, const Eigen::Matrix<double, 6, 1>& poseUnaryNoise,
                                         const gtsam::Pose3& pose);
  void addGnssPositionUnaryFactor(double gnssTime, const double rate, const Eigen::Vector3d& gnssPositionUnaryNoise,
                                  const gtsam::Vector3& position);
  void addGnssHeadingUnaryFactor(double gnssTime, const double rate, const double gnssHeadingUnaryNoise, const double measuredYaw);
  bool addZeroMotionFactor(double maxTimestampDistance, double timeKm1, double timeK, const gtsam::Pose3 pose);

  // Graph selection
  void activateGlobalGraph(const gtsam::Vector3& imuPosition, const gtsam::Rot3& imuRotation, const double measurementTime);
  void activateFallbackGraph();

  // Update graph and get new state
  SafeNavStateWithCovarianceAndBias updateActiveGraphAndGetState(double& currentTime);

  // Compute state at specific key
  gtsam::NavState calculateActiveStateAtKey(bool& computeSuccessfulFlag, const gtsam::Key& key);

  // IMU Buffer interface
  /// Estimate attitude from IMU
  inline bool estimateAttitudeFromImu(gtsam::Rot3& initAttitude, double& gravityMagnitude, Eigen::Vector3d& gyrBias) {
    return imuBufferPtr_->estimateAttitudeFromImu(initAttitude, gravityMagnitude, gyrBias);
  }
  /// Add to IMU buffer
  inline Eigen::Matrix<double, 6, 1> addToIMUBuffer(double ts, const Eigen::Vector3d& linearAcc, const Eigen::Vector3d& angularVel) {
    return imuBufferPtr_->addToImuBuffer(ts, linearAcc, angularVel);
  }

  // Accessors
  /// Getters
  Eigen::Vector3d& getInitAccBiasReference() { return graphConfigPtr_->accBiasPrior; }
  Eigen::Vector3d& getInitGyrBiasReference() { return graphConfigPtr_->gyroBiasPrior; }

  //  auto iterations() const { return additonalIterations_; }
  const State& getOptimizedGraphState() { return optimizedGraphState_; }
  const gtsam::Key getPropagatedStateKey() { return propagatedStateKey_; }
  const gtsam::imuBias::ConstantBias getOptimizedImuBias() { return optimizedGraphState_.imuBias(); }
  gtsam::ISAM2Params& getIsamParamsReference() { return isamParams_; }

  // Status
  bool globalGraphActiveFlag() {
    const std::lock_guard<std::mutex> swappingActiveGraphLock(swappingActiveGraphMutex_);
    return activeSmootherPtr_ == globalSmootherPtr_;
  }
  bool fallbackGraphActiveFlag() {
    const std::lock_guard<std::mutex> swappingActiveGraphLock(swappingActiveGraphMutex_);
    return activeSmootherPtr_ == fallbackSmootherPtr_;  //&& numOptimizationsSinceGraphSwitching_ >= 1;
  }

 protected:
  // Calculate state at key for graph
  static gtsam::NavState calculateNavStateAtKey(bool& computeSuccessfulFlag,
                                                const std::shared_ptr<gtsam::IncrementalFixedLagSmoother> graphPtr,
                                                const std::shared_ptr<GraphConfig>& graphConfigPtr, const gtsam::Key& key,
                                                const char* callingFunctionName);

 private:
  // Methods
  template <class CHILDPTR>
  bool addFactorToGraph_(std::shared_ptr<gtsam::NonlinearFactorGraph> modifiedGraphPtr, const gtsam::NoiseModelFactor* noiseModelFactorPtr);
  template <class CHILDPTR>
  bool addFactorToGraph_(std::shared_ptr<gtsam::NonlinearFactorGraph> modifiedGraphPtr, const gtsam::NoiseModelFactor* noiseModelFactorPtr,
                         const double measurementTimestamp);
  template <class CHILDPTR>
  bool addFactorSafelyToGraph_(std::shared_ptr<gtsam::NonlinearFactorGraph> modifiedGraphPtr,
                               const gtsam::NoiseModelFactor* noiseModelFactorPtr, const double measurementTimestamp);
  /// Update IMU integrators
  void updateImuIntegrators_(const TimeToImuMap& imuMeas);

  // Add Factors for a smoother
  static void addFactorsToSmootherAndOptimize(std::shared_ptr<gtsam::IncrementalFixedLagSmoother> smootherPtr,
                                              const gtsam::NonlinearFactorGraph& newGraphFactors, const gtsam::Values& newGraphValues,
                                              const std::map<gtsam::Key, double>& newGraphKeysTimestampsMap,
                                              const std::shared_ptr<GraphConfig>& graphConfigPtr, const int additionalIterations);
  /// Find graph keys for timestamps
  bool findGraphKeys_(double maxTimestampDistance, double timeKm1, double timeK, gtsam::Key& keyKm1, gtsam::Key& keyK,
                      const std::string& name = "lidar");
  /// Generate new key
  const auto newPropagatedStateKey_() { return ++propagatedStateKey_; }
  /// Associate timestamp to each 'value key', e.g. for graph key 0, value keys (x0,v0,b0) need to be associated
  inline void writeValueKeysToKeyTimeStampMap_(const gtsam::Values& values, const double measurementTime,
                                               std::shared_ptr<std::map<gtsam::Key, double>> keyTimestampMapPtr) {
    for (const auto& value : values) {
      (*keyTimestampMapPtr)[value.key] = measurementTime;
    }
  }

  // Objects
  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> imuParamsPtr_;
  std::shared_ptr<gtsam::imuBias::ConstantBias> imuBiasPriorPtr_;
  State optimizedGraphState_;
  gtsam::ISAM2Params isamParams_;

  // Graphs
  std::shared_ptr<gtsam::IncrementalFixedLagSmoother> globalSmootherPtr_;
  std::shared_ptr<gtsam::IncrementalFixedLagSmoother> fallbackSmootherPtr_;
  /// Data buffers
  std::shared_ptr<gtsam::NonlinearFactorGraph> globalFactorsBufferPtr_;
  std::shared_ptr<gtsam::NonlinearFactorGraph> fallbackFactorsBufferPtr_;
  // Values map
  std::shared_ptr<gtsam::Values> globalGraphValuesBufferPtr_;
  std::shared_ptr<gtsam::Values> fallbackGraphValuesBufferPtr_;
  // Timestampmaps
  std::shared_ptr<std::map<gtsam::Key, double>> globalGraphKeysTimestampsMapBufferPtr_;
  std::shared_ptr<std::map<gtsam::Key, double>> fallbackGraphKeysTimestampsMapBufferPtr_;

  /// Counter
  int numOptimizationsSinceGraphSwitching_ = 0;
  bool sentRelocalizationCommandAlready_ = true;

  /// Buffer Preintegrator
  std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> globalImuBufferPreintegratorPtr_;
  std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> fallbackImuBufferPreintegratorPtr_;
  /// Step Preintegrator
  std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> imuStepPreintegratorPtr_;
  /// IMU Buffer
  std::shared_ptr<graph_msf::ImuBuffer> imuBufferPtr_;  // Need to get rid of this
  gtsam::Vector6 lastImuVector_;

  /// Config
  std::shared_ptr<graph_msf::GraphConfig> graphConfigPtr_ = NULL;
  /// Graph names
  std::vector<std::string> graphNames_{"globalGraph", "fallbackGraph"};
  /// Selector
  std::shared_ptr<gtsam::IncrementalFixedLagSmoother> activeSmootherPtr_ = globalSmootherPtr_;
  std::shared_ptr<gtsam::NonlinearFactorGraph> activeFactorsBufferPtr_ = globalFactorsBufferPtr_;
  std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> activeImuBufferPreintegratorPtr_ = globalImuBufferPreintegratorPtr_;
  std::shared_ptr<gtsam::Values> activeGraphValuesBufferPtr_ = globalGraphValuesBufferPtr_;
  std::shared_ptr<std::map<gtsam::Key, double>> activeGraphKeysTimestampsMapBufferPtr_ = globalGraphKeysTimestampsMapBufferPtr_;

  // Member variables
  /// Mutex
  std::mutex operateOnGraphDataMutex_;
  std::mutex activelyUsingActiveGraphMutex_;
  std::mutex swappingActiveGraphMutex_;
  /// Propagated state (at IMU frequency)
  gtsam::NavState imuPropagatedState_;

  /// Factor Graph
  gtsam::Key propagatedStateKey_ = 0;  // Current state key
  double propagatedStateTime_;
};
}  // namespace graph_msf

#endif  // GRAPH_MANAGER_HPP_