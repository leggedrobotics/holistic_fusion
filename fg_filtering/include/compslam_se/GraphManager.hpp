#ifndef GRAPH_MANAGER_HPP_
#define GRAPH_MANAGER_HPP_

// C++
#include <chrono>
#include <mutex>
#include <vector>

// ROS
#include <rosconsole/macros_generated.h>

// GTSAM
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearISAM.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

// Factors
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include "compslam_se/factors/HeadingFactor.h"

// Package
#include "compslam_se/GraphState.hpp"
#include "compslam_se/ImuBuffer.hpp"
#include "compslam_se/config/GraphConfig.h"

namespace compslam_se {

#define GREEN_START "\033[92m"
#define YELLOW_START "\033[33m"
#define RED_START "\033[31m"
#define COLOR_END "\033[0m"

class GraphManager {
 public:
  GraphManager(GraphConfig* graphConfigPtr);
  ~GraphManager(){};

  // Change Graph
  bool initImuIntegrators(const double g, const std::string& imuGravityDirection);
  bool initPoseVelocityBiasGraph(const double ts, const gtsam::Pose3& init_pose);
  gtsam::NavState addImuFactorAndGetState(const double imuTimeK, const Eigen::Vector3d& linearAcc, const Eigen::Vector3d& angularVel,
                                          bool& relocalizationFlag);
  gtsam::Key addPoseBetweenFactorToGlobalGraph(const double lidarTimeKm1, const double lidarTimeK, const double rate,
                                               const Eigen::Matrix<double, 6, 1>& poseBetweenNoise, const gtsam::Pose3& pose);
  void addPoseUnaryFactorToFallbackGraph(const double lidarTimeK, const double rate, const Eigen::Matrix<double, 6, 1>& poseUnaryNoise,
                                         const gtsam::Pose3& pose);
  void addGnssPositionUnaryFactor(double gnssTime, const double rate, const double gnssPositionUnaryNoise, const gtsam::Vector3& position);
  void addGnssHeadingUnaryFactor(double gnssTime, const double rate, const double gnssHeadingUnaryNoise, const double measuredYaw);
  bool addZeroMotionFactor(double maxTimestampDistance, double timeKm1, double timeK, const gtsam::Pose3 pose);
  bool addGravityRollPitchFactor(const gtsam::Key key, const gtsam::Rot3 imuAttitude);

  // Graph selection
  void activateGlobalGraph();
  void activateFallbackGraph();

  // Update graph and get new state
  gtsam::NavState updateGraphAndState(double& currentTime);

  // Compute state at specific key
  gtsam::NavState calculateStateAtKey(const gtsam::Key& key);

  // IMU Buffer interface
  /// Estimate attitude from IMU
  inline bool estimateAttitudeFromImu(const std::string& imuGravityDirection, gtsam::Rot3& initAttitude, double& gravityMagnitude,
                                      Eigen::Vector3d& gyrBias) {
    return imuBuffer_.estimateAttitudeFromImu(imuGravityDirection, initAttitude, gravityMagnitude, gyrBias);
  }
  /// Add to IMU buffer
  inline void addToIMUBuffer(double ts, const Eigen::Vector3d& linearAcc, const Eigen::Vector3d& angularVel) {
    imuBuffer_.addToIMUBuffer(ts, linearAcc(0), linearAcc(1), linearAcc(2), angularVel(0), angularVel(1), angularVel(2));
  }

  // Accessors
  /// Getters
  Eigen::Vector3d& getInitGyrBiasReference() { return graphConfigPtr_->gyroBiasPrior; }
  //  auto iterations() const { return additonalIterations_; }
  const State& getGraphState() { return graphState_; }
  const gtsam::Key getStateKey() { return stateKey_; }
  const gtsam::imuBias::ConstantBias getIMUBias() { return graphState_.imuBias(); }
  gtsam::ISAM2Params& getIsamParamsReference() { return isamParams_; }

  // Status
  bool globalGraphActiveFlag() {
    const std::lock_guard<std::mutex> consistentActiveGraphLock(consistentActiveGraphMutex_);
    return activeGraphPtr_ == globalGraphPtr_;
  }
  bool fallbackGraphActiveFlag() {
    const std::lock_guard<std::mutex> consistentActiveGraphLock(consistentActiveGraphMutex_);
    return activeGraphPtr_ == fallbackGraphPtr_;  //&& numOptimizationsSinceGraphSwitching_ >= 1;
  }

 private:
  // Methods
  /// Update IMU integrators
  void updateImuIntegrators_(const TimeToImuMap& imuMeas);
  /// Find graph keys for timestamps
  bool findGraphKeys_(double maxTimestampDistance, double timeKm1, double timeK, gtsam::Key& keyKm1, gtsam::Key& keyK,
                      const std::string& name = "lidar");
  /// Generate new key
  const auto newStateKey_() { return ++stateKey_; }
  /// Associate timestamp to each 'value key', e.g. for graph key 0, value keys (x0,v0,b0) need to be associated
  inline void writeValueKeysToKeyTimeStampMap_(const gtsam::Values& values, const double measurementTime,
                                               std::map<gtsam::Key, double>& keyTimestampMap) {
    for (const auto& value : values) {
      keyTimestampMap[value.key] = measurementTime;
    }
  }

  // Objects
  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> imuParamsPtr_;
  std::shared_ptr<gtsam::imuBias::ConstantBias> imuBiasPriorPtr_;
  State graphState_;
  gtsam::ISAM2Params isamParams_;

  // Graphs
  std::shared_ptr<gtsam::IncrementalFixedLagSmoother> globalGraphPtr_;
  std::shared_ptr<gtsam::IncrementalFixedLagSmoother> fallbackGraphPtr_;
  /// Data buffers
  gtsam::NonlinearFactorGraph globalFactorsBuffer_;
  gtsam::NonlinearFactorGraph fallbackFactorsBuffer_;
  /// Config
  GraphConfig* graphConfigPtr_ = NULL;
  /// Graph names
  std::vector<std::string> graphNames_{"globalGraph", "fallbackGraph"};
  /// Selector
  std::shared_ptr<gtsam::IncrementalFixedLagSmoother> activeGraphPtr_ = globalGraphPtr_;  // std::shared_ptr<gtsam::ISAM2> mainGraphPtr_;
  gtsam::NonlinearFactorGraph& activeFactorsBuffer_ = globalFactorsBuffer_;
  /// Counter
  int numOptimizationsSinceGraphSwitching_ = 0;
  bool sentRelocalizationCommandAlready_ = true;

  // Values and timestamp map --> same for both graphs
  gtsam::Values graphValuesBuffer_;
  std::map<gtsam::Key, double> graphKeysTimestampsMapBuffer_;
  /// Buffer Preintegrator
  std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> imuBufferPreintegratorPtr_;
  /// Step Preintegrator
  std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> imuStepPreintegratorPtr_;
  /// IMU Buffer
  ImuBuffer imuBuffer_;  // Need to get rid of this
  gtsam::Vector6 lastImuVector_;
  bool firstImuCallback_ = true;

  // Member variables
  /// Mutex
  std::mutex operateOnGraphDataMutex_;
  std::mutex consistentActiveGraphMutex_;
  /// Propagated state (at IMU frequency)
  gtsam::NavState imuPropagatedState_;

  /// Factor Graph
  gtsam::Key stateKey_ = 0;  // Current state key
  double stateTime_;
};
}  // namespace compslam_se

#endif