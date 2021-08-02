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
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include "fg_filtering/GraphState.hpp"
#include "fg_filtering/ImuManager.hpp"

namespace fg_filtering {
class GraphManager {
 public:
  // Constructor / Destructor
  GraphManager(){};
  ~GraphManager(){};

  // Initialize Factor graph with Pose,Velocity and Bias states
  bool initPoseVelocityBiasGraph(const double ts, const gtsam::Pose3& init_pose);

  // Initialize IMU integrator
  bool initImuIntegrators(const double g, const std::string& imuGravityDirection);

  // Add IMU factor to graph
  gtsam::NavState addImuFactorAndGetState(const double imuTime_k);

  // Add a pose between factor
  void addPoseBetweenFactor(const gtsam::Pose3& pose, const double lidarTime_km1, const double lidarTime_k);

  // Add a pose unary factor
  void addPoseUnaryFactor(const gtsam::Key old_key, const gtsam::Key new_key, const gtsam::Pose3& pose);

  // Check if zero motion has occured and add zero pose/velocity factor
  bool addZeroMotionFactor(const gtsam::Key old_key, const gtsam::Key new_key, const gtsam::Pose3 pose);

  // Add gravity-aligned roll/ptich when no motion
  bool addGravityRollPitchFactor(const gtsam::Key key, const gtsam::Rot3 imu_attitude);

  // Update graph and get new state
  gtsam::NavState updateGraphAndState();
  // Associate timestamp to each 'value key', e.g. for graph key 0, value keys (x0,v0,b0) need to be associated
  void valuesToKeyTimeStampMap(const gtsam::Values& values, const double ts, std::map<gtsam::Key, double>& key_timestamp_map) {
    for (const auto& value : values) key_timestamp_map[value.key] = ts;
  }

  // IMU Buffer interface
  /// Estimate attitude from IMU
  inline bool estimateAttitudeFromImu(const double init_ts, const std::string& imuGravityDirection, gtsam::Rot3& initAttitude,
                                      double& gravityMagnitude) {
    return imuBuffer_.estimateAttitudeFromImu(init_ts, imuGravityDirection, initAttitude, gravityMagnitude);
  }
  /// Add to IMU buffer
  inline void addToIMUBuffer(double ts, double accX, double accY, double accZ, double gyrX, double gyrY, double gyrZ) {
    imuBuffer_.addToIMUBuffer(ts, accX, accY, accZ, gyrX, gyrY, gyrZ);
  }

  // Accessors - Setters
  void setAccNoiseDensity(double val) { accNoiseDensity_ = val; }
  void setAccBiasRandomWalk(double val) { accBiasRandomWalk_ = val; }
  void setAccBiasPrior(double val) { accBiasPrior_ = val; }
  void setGyroNoiseDensity(double val) { gyrNoiseDensity_ = val; }
  void setGyrBiasRandomWalk(double val) { gyrBiasRandomWalk_ = val; }
  void setIntegrationNoiseDensity(double val) { integrationNoiseDensity_ = val; }
  void setBiasAccOmegaPreint(double val) { biasAccOmegaPreint_ = val; }
  void setGyrBiasPrior(double val) { gyrBiasPrior_ = val; }
  void setSmootherLag(double val) { smootherLag_ = val; }
  void setIterations(int val) { additonalIterations_ = val; }
  void setPositionReLinTh(double val) { posReLinTh_ = val; }
  void setRotationReLinTh(double val) { rotReLinTh_ = val; }
  void setVelocityReLinTh(double val) { velReLinTh_ = val; }
  void setAccBiasReLinTh(double val) { accBiasReLinTh_ = val; }
  void setGyrBiasReLinTh(double val) { gyrBiasReLinTh_ = val; }
  void setPoseNoise(const std::vector<double>& v) { poseNoise_ = v; }
  void setImuRate(double d) { imuBuffer_.setImuRate(d); }
  // Accessors - Getters
  auto iterations() const { return additonalIterations_; }
  const fg_filtering::State& getGraphState() { return graphState_; }
  const auto getStateKey() { return stateKey_; }
  const auto getIMUBias() { return graphState_.imuBias(); }
  gtsam::ISAM2Params& getIsamParamsReference() { return isamParams_; }

 private:
  // Methods
  /// Update IMU integrator with new measurements - Resets bias
  void updateImuIntegrators_(const IMUMap& imuMeas);
  const auto newStateKey() { return ++stateKey_; }

  // Objects
  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> imuParamsPtr_;
  std::shared_ptr<gtsam::imuBias::ConstantBias> imuBiasPriorPtr_;
  std::shared_ptr<gtsam::IncrementalFixedLagSmoother> mainGraphPtr_;
  fg_filtering::State graphState_;
  gtsam::ISAM2Params isamParams_;
  /// Data buffers for callbacks to add information via member functions
  gtsam::NonlinearFactorGraph newGraphFactors_;
  gtsam::Values newGraphValues_;
  /// Buffer Preintegrator
  std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> imuBufferPreintegratorPtr_;
  /// Step Preintegrator
  std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> imuStepPreintegratorPtr_;
  /// IMU Buffer
  ImuManager imuBuffer_;

  // Member variables
  /// Mutex
  std::mutex operateOnGraphDataMutex_;
  /// Propagated state (at IMU frequency)
  gtsam::NavState imuPropagatedState_;
  /// IMU Preintegration
  double accNoiseDensity_;          // continuous-time "Covariance" of accelerometer
  double accBiasRandomWalk_;        // continuous-time "Covariance" describing accelerometer bias random walk
  double accBiasPrior_;             // prior/starting value of accelerometer bias
  double gyrNoiseDensity_;          // continuous-time "Covariance" of gyroscope measurements
  double gyrBiasRandomWalk_;        // continuous-time "Covariance" describing gyroscope bias random walk
  double integrationNoiseDensity_;  // "Covariance" describing
  double biasAccOmegaPreint_;       // Describing error of bias for preintegration
  double gyrBiasPrior_;             // prior/starting value of gyroscope bias
  /// Factor Graph
  gtsam::Key stateKey_ = 0;  // Current state key
  double stateTime_;
  double smootherLag_ = 1;       // Lag of fixed lag smoother[seconds]
  int additonalIterations_ = 1;  // Additional iterations of graph optimizer after update with new factors
  double posReLinTh_ = 0.05;     // Position linearization threshold
  double rotReLinTh_ = 0.05;     // Rotation linearization threshold
  double velReLinTh_ = 0.1;      // Linear Velocity linearization threshold
  double accBiasReLinTh_ = 0.1;  // Accelerometer bias linearization threshold
  double gyrBiasReLinTh_ = 0.1;  // Gyroscope bias linearization threshold
  /// Pose Between Factor
  std::vector<double> poseNoise_{0.02, 0.02, 0.02, 0.05, 0.05, 0.05};  // ORDER RPY(rad) - XYZ(meters)
  /// Zero Velocity Factor
  double zeroMotionTh_ = 0.01;              // Zero motion threshold meters
  double minDetections_ = 10;               // Number of consective zero motions detected before zero motion factors are added
  double detectionCount_ = minDetections_;  // Assumption: Robot starts at rest so initially zero motion is enabled
};
}  // namespace fg_filtering

#endif