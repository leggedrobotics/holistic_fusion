/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GRAPH_MANAGER_HPP_
#define GRAPH_MANAGER_HPP_

// C++
#include <chrono>
#include <mutex>
#include <vector>

// GTSAM
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/slam/expressions.h>

// Package
#include "graph_msf/config/GraphConfig.h"
#include "graph_msf/core/GraphState.hpp"
#include "graph_msf/core/TimeGraphKeyBuffer.h"
#include "graph_msf/core/TransformsExpressionKeys.h"
#include "graph_msf/core/optimizer/OptimizerBase.h"
#include "graph_msf/imu/ImuBuffer.hpp"
#include "graph_msf/interface/NavState.h"
#include "graph_msf/measurements/Measurement.h"
#include "graph_msf/measurements/UnaryMeasurementXD.h"

// General Unary Factor Interface
#include "graph_msf/factors/gmsf_expression/GmsfUnaryExpression.h"

// General Binary Factor Interface
// TODO: add binary factor interface

namespace graph_msf {

// Actual Class
class GraphManager {
 public:
  GraphManager(std::shared_ptr<GraphConfig> graphConfigPtr, std::string imuFrame, std::string worldFrame);
  ~GraphManager() {
    std::cout << YELLOW_START << "GraphMSF: GraphManager" << GREEN_START << " Destructor called." << COLOR_END << std::endl;
    if (graphConfigPtr_->useAdditionalSlowBatchSmoother_) {
      std::cout << YELLOW_START << "GraphMSF: GraphManager" << COLOR_END
                << " Additional slow batch smoother was built up. Next time the optimization of it can be called before shutting down (if "
                   "not done already)."
                << std::endl;
    }
  };

  // Initialization Interface ---------------------------------------------------
  bool initImuIntegrators(double gravityValue);
  bool initPoseVelocityBiasGraph(double timeStamp, const gtsam::Pose3& T_W_I0, const gtsam::Pose3& T_O_I0);

  // IMU at the core -----------------------------------------------------------
  void addImuFactorAndGetState(SafeIntegratedNavState& returnPreIntegratedNavState,
                               std::shared_ptr<SafeNavStateWithCovarianceAndBias>& newOptimizedNavStatePtr,
                               const std::shared_ptr<ImuBuffer>& imuBufferPtr, double imuTimeK, bool createNewStateFlag);

  // All other measurements -----------------------------------------------------

  // Unary commodity methods --> Key Lookup
  bool getUnaryFactorGeneralKey(gtsam::Key& returnedKey, const UnaryMeasurement& unaryMeasurement);

  // Unary Meta Method --> classic GTSAM Factors
  typedef gtsam::Key (*F)(std::uint64_t);
  template <class MEASUREMENT_TYPE, int NOISE_DIM, class FACTOR_TYPE, F SYMBOL_SHORTHAND>
  void addUnaryFactorInImuFrame(const MEASUREMENT_TYPE& unaryMeasurement, const Eigen::Matrix<double, NOISE_DIM, 1>& unaryNoiseDensity,
                                double measurementTime);

  // GMSF Holistic Graph Factors with Extrinsic Calibration ------------------------
  template <class GTSAM_MEASUREMENT_TYPE>
  void addUnaryGmsfExpressionFactor(const std::shared_ptr<GmsfUnaryExpression<GTSAM_MEASUREMENT_TYPE>>& gmsfUnaryExpressionPtr);

  // Robust Norm Aware Between Factor
  gtsam::Key addPoseBetweenFactor(const gtsam::Pose3& deltaPose, const Eigen::Matrix<double, 6, 1>& poseBetweenNoiseDensity,
                                  double lidarTimeKm1, double lidarTimeK, double rate, const RobustNormEnum& robustNormEnum,
                                  const double robustNormConstant);

  // Update of graph  ----------------------------------------------------------
  // Real-time Graph Update
  void updateGraph();

  // Slow Graph Update (if desired)
  bool optimizeSlowBatchSmoother(int maxIterations, const std::string& savePath);

  // Save Variables to File
  static void saveOptimizedValuesToFile(const gtsam::Values& optimizedValues, const std::map<gtsam::Key, double>& keyTimestampMap,
                                        const std::string& savePath);

  // Save Optimized Graph to G2O Format
  static void saveOptimizedGraphToG2o(const OptimizerBase& optimizedGraph, const gtsam::Values& optimizedValues,
                                      const std::string& saveFileName);

  // Comfort functions ---------------------------------------------------------
  gtsam::NavState calculateStateAtKey(bool& computeSuccessfulFlag, const gtsam::Key& key);

  // Accessors
  /// Getters
  Eigen::Vector3d& getInitAccBiasReference() { return graphConfigPtr_->accBiasPrior_; }
  Eigen::Vector3d& getInitGyrBiasReference() { return graphConfigPtr_->gyroBiasPrior_; }

  //  auto iterations() const { return additonalIterations_; }
  const GraphState& getOptimizedGraphState() { return optimizedGraphState_; }
  const gtsam::Key getPropagatedStateKey() { return propagatedStateKey_; }

 protected:
  // Calculate state at key for graph
  static gtsam::NavState calculateNavStateAtKey(bool& computeSuccessfulFlag, std::shared_ptr<graph_msf::OptimizerBase> graphPtr,
                                                const gtsam::Key& key, const char* callingFunctionName);

 private:
  // Methods
  template <class CHILDPTR>
  bool addFactorToGraph_(const gtsam::NoiseModelFactor* noiseModelFactorPtr);
  template <class CHILDPTR>
  bool addFactorToGraph_(const gtsam::NoiseModelFactor* noiseModelFactorPtr, double measurementTimestamp,
                         const std::string& measurementName);
  template <class CHILDPTR>
  bool addFactorSafelyToGraph_(const gtsam::NoiseModelFactor* noiseModelFactorPtr, double measurementTimestamp);
  /// Update IMU integrators
  void updateImuIntegrators_(const TimeToImuMap& imuMeas);

  // Add Factors for a smoother
  bool addFactorsToSmootherAndOptimize(const gtsam::NonlinearFactorGraph& newGraphFactors, const gtsam::Values& newGraphValues,
                                       const std::map<gtsam::Key, double>& newGraphKeysTimestampsMap,
                                       const std::shared_ptr<GraphConfig>& graphConfigPtr, int additionalIterations);
  /// Find graph keys for timestamps
  bool findGraphKeys_(gtsam::Key& closestKeyKm1, gtsam::Key& closestKeyK, double& keyTimeStampDistance, double maxTimestampDistance,
                      double timeKm1, double timeK, const std::string& name);
  /// Generate new key
  const uint64_t newPropagatedStateKey_() { return ++propagatedStateKey_; }
  /// Associate timestamp to each 'value key', e.g. for graph key 0, value keys (x0,v0,b0) need to be associated
  inline void writeKeyToKeyTimeStampMap_(const gtsam::Key& key, double measurementTime,
                                         std::shared_ptr<std::map<gtsam::Key, double>> keyTimestampMapPtr);

  void writeValueKeysToKeyTimeStampMap_(const gtsam::Values& values, double measurementTime,
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
  double lastOptimizedStateTime_ = 0.0;                      // Last optimized state time
  gtsam::Vector3 currentAngularVelocity_ = gtsam::Vector3(0, 0, 0);

  // Optimizer(s)
  std::shared_ptr<OptimizerBase> rtOptimizerPtr_;
  std::shared_ptr<OptimizerBase> batchOptimizerPtr_;
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
  std::mutex optimizationRunningMutex_;
};

}  // namespace graph_msf

// Template Implementations
#include "graph_msf/core/GraphManager.inl"

#endif  // GRAPH_MANAGER_HPP_