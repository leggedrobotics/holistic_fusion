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

// Package
#include "graph_msf/config/GraphConfig.h"
#include "graph_msf/core/GraphState.hpp"
#include "graph_msf/core/Optimizer.h"
#include "graph_msf/core/TimeGraphKeyBuffer.h"
#include "graph_msf/core/TransformsExpressionKeys.h"
#include "graph_msf/factors/HeadingFactor.h"
#include "graph_msf/imu/ImuBuffer.hpp"
#include "graph_msf/interface/NavState.h"
#include "graph_msf/measurements/UnaryMeasurementXD.h"

namespace graph_msf {

// Actual Class
class GraphManager {
 public:
  GraphManager(std::shared_ptr<GraphConfig> graphConfigPtr, const std::string& imuFrame, const std::string& worldFrame);
  ~GraphManager(){};

  // Initialization Interface ---------------------------------------------------
  bool initImuIntegrators(const double g);
  bool initPoseVelocityBiasGraph(const double ts, const gtsam::Pose3& init_pose);

  // IMU at the core -----------------------------------------------------------
  void addImuFactorAndGetState(SafeIntegratedNavState& changedPreIntegratedNavStatePtr,
                               std::shared_ptr<SafeNavStateWithCovarianceAndBias>& newOptimizedNavStatePtr,
                               const std::shared_ptr<ImuBuffer> imuBufferPtr, const double imuTimeK);

  // All other measurements -----------------------------------------------------
  // Unary commodity methods
  bool addUnaryFactorToReturnedKey(gtsam::Key& returnedKey, const graph_msf::UnaryMeasurement& unaryMeasurement);
  // Unary Meta Method
  typedef gtsam::Key (*F)(std::uint64_t);
  template <class MEASUREMENT_TYPE, int NOISE_DIM, class FACTOR_TYPE, F SYMBOL_SHORTHAND>
  void addUnaryFactorInImuFrame(const MEASUREMENT_TYPE& unaryMeasurement, const Eigen::Matrix<double, NOISE_DIM, 1>& unaryNoiseDensity,
                                const double measurementTime);
  template <class MEASUREMENT_TYPE, int NOISE_DIM, class EXPRESSION>
  void addUnaryExpressionFactor(const MEASUREMENT_TYPE& unaryMeasurement, const Eigen::Matrix<double, NOISE_DIM, 1>& unaryNoiseDensity,
                                const EXPRESSION& unaryExpression, const double measurementTime, const gtsam::Values& newStateValues,
                                std::vector<gtsam::PriorFactor<MEASUREMENT_TYPE>>& priorFactors);
  // Unary Specializations
  void addPoseUnaryFactor(const UnaryMeasurementXD<Eigen::Isometry3d, 6>& unary6DMeasurement, const Eigen::Isometry3d& T_sensorFrame_imu);
  void addVelocityUnaryFactor(const gtsam::Vector3& velocity, const Eigen::Matrix<double, 3, 1>& velocityUnaryNoiseDensity,
                              const double lidarTimeK);
  void addPositionUnaryFactor(const UnaryMeasurementXD<Eigen::Vector3d, 3>& unaryPositionMeasurement,
                              const std::optional<Eigen::Vector3d>& I_t_I_sensorFrame = std::nullopt);
  void addHeadingUnaryFactor(const double measuredYaw, const Eigen::Matrix<double, 1, 1>& gnssHeadingUnaryNoiseDensity,
                             const double gnssTime);

  // Between
  gtsam::Key addPoseBetweenFactor(const gtsam::Pose3& deltaPose, const Eigen::Matrix<double, 6, 1>& poseBetweenNoiseDensity,
                                  const double lidarTimeKm1, const double lidarTimeK, const double rate);

  // Update of graph  ----------------------------------------------------------
  void updateGraph();

  // Comfort functions ---------------------------------------------------------
  gtsam::NavState calculateStateAtKey(bool& computeSuccessfulFlag, const gtsam::Key& key);

  // Accessors
  /// Getters
  Eigen::Vector3d& getInitAccBiasReference() { return graphConfigPtr_->accBiasPrior; }
  Eigen::Vector3d& getInitGyrBiasReference() { return graphConfigPtr_->gyroBiasPrior; }

  //  auto iterations() const { return additonalIterations_; }
  const GraphState& getOptimizedGraphState() { return optimizedGraphState_; }
  const gtsam::Key getPropagatedStateKey() { return propagatedStateKey_; }

 protected:
  // Calculate state at key for graph
  static gtsam::NavState calculateNavStateAtKey(bool& computeSuccessfulFlag, const std::shared_ptr<graph_msf::Optimizer> graphPtr,
                                                const gtsam::Key& key, const char* callingFunctionName);

 private:
  // Methods
  template <class CHILDPTR>
  void addFactorToGraph_(const gtsam::NoiseModelFactor* noiseModelFactorPtr);
  template <class CHILDPTR>
  void addFactorToGraph_(const gtsam::NoiseModelFactor* noiseModelFactorPtr, const double measurementTimestamp);
  template <class CHILDPTR>
  void addFactorSafelyToGraph_(const gtsam::NoiseModelFactor* noiseModelFactorPtr, const double measurementTimestamp);
  /// Update IMU integrators
  void updateImuIntegrators_(const TimeToImuMap& imuMeas);

  // Add Factors for a smoother
  static bool addFactorsToSmootherAndOptimize(std::shared_ptr<graph_msf::Optimizer> smootherPtr,
                                              const gtsam::NonlinearFactorGraph& newGraphFactors, const gtsam::Values& newGraphValues,
                                              const std::map<gtsam::Key, double>& newGraphKeysTimestampsMap,
                                              const std::shared_ptr<GraphConfig>& graphConfigPtr, const int additionalIterations);
  /// Find graph keys for timestamps
  bool findGraphKeys_(gtsam::Key& closestKeyKm1, gtsam::Key& closestKeyK, double& keyTimeStampDistance, const double maxTimestampDistance,
                      const double timeKm1, const double timeK, const std::string& name);
  /// Generate new key
  const uint64_t newPropagatedStateKey_() { return ++propagatedStateKey_; }
  /// Associate timestamp to each 'value key', e.g. for graph key 0, value keys (x0,v0,b0) need to be associated
  inline void writeKeyToKeyTimeStampMap_(const gtsam::Key& key, const double measurementTime,
                                         std::shared_ptr<std::map<gtsam::Key, double>> keyTimestampMapPtr);

  void writeValueKeysToKeyTimeStampMap_(const gtsam::Values& values, const double measurementTime,
                                        std::shared_ptr<std::map<gtsam::Key, double>> keyTimestampMapPtr);

  // Buffers
  std::shared_ptr<TimeGraphKeyBuffer> timeToKeyBufferPtr_;

  // Optimization Transformations
  std::string imuFrame_;
  std::string worldFrame_;
  TransformsExpressionKeys gtsamExpressionTransformsKeys_;
  TransformsDictionary<Eigen::Isometry3d> resultFixedFrameTransformations_;
  TransformsDictionary<Eigen::Matrix<double, 6, 6>> resultFixedFrameTransformationsCovariance_;

  // Objects
  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> imuParamsPtr_;
  std::shared_ptr<gtsam::imuBias::ConstantBias> imuBiasPriorPtr_;
  graph_msf::GraphState optimizedGraphState_;
  /// Propagated state (at IMU frequency)
  gtsam::NavState W_imuPropagatedState_ = gtsam::NavState(gtsam::Pose3(), gtsam::Vector3(0, 0, 0));
  gtsam::NavState O_imuPropagatedState_ = gtsam::NavState(gtsam::Pose3(), gtsam::Vector3(0, 0, 0));
  Eigen::Isometry3d T_W_O_ = Eigen::Isometry3d::Identity();  // Current state pose, depending on whether propagated state jumps or not
  gtsam::Key propagatedStateKey_ = 0;                        // Current state key
  double propagatedStateTime_ = 0.0;                         // Current state time
  gtsam::Vector3 currentAngularVelocity_ = gtsam::Vector3(0, 0, 0);

  // Optimizer
  std::shared_ptr<graph_msf::Optimizer> optimizerPtr_;
  /// Data buffer
  std::shared_ptr<gtsam::NonlinearFactorGraph> factorGraphBufferPtr_;
  // Values map
  std::shared_ptr<gtsam::Values> graphValuesBufferPtr_;
  // Keys timestamp map
  std::shared_ptr<std::map<gtsam::Key, double>> graphKeysTimestampsMapBufferPtr_;

  // Preintegration
  /// Step Preintegrator
  std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> imuStepPreintegratorPtr_;
  /// Buffer Preintegrator
  std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> imuBufferPreintegratorPtr_;

  /// IMU Buffer
  gtsam::Vector6 lastImuVector_;

  /// Config
  std::shared_ptr<graph_msf::GraphConfig> graphConfigPtr_ = nullptr;

  // Member variables
  /// Mutex
  std::mutex operateOnGraphDataMutex_;
};
}  // namespace graph_msf

#endif  // GRAPH_MANAGER_HPP_