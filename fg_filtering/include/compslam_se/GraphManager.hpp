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

// Catkin workspace
#include "compslam_se/GraphState.hpp"
#include "compslam_se/ImuBuffer.hpp"

namespace compslam_se {

#define GREEN_START "\033[92m"
#define YELLOW_START "\033[33m"
#define RED_START "\033[31m"
#define COLOR_END "\033[0m"

class GraphManager {
 public:
  GraphManager(){};
  ~GraphManager(){};

  // Change Graph
  bool initImuIntegrators(const double g, const std::string& imuGravityDirection);
  bool initPoseVelocityBiasGraph(const double ts, const gtsam::Pose3& init_pose);
  gtsam::NavState addImuFactorAndGetState(const double imuTimeK, const Eigen::Vector3d& linearAcc, const Eigen::Vector3d& angularVel,
                                          bool& relocalizationFlag);
  gtsam::Key addPoseBetweenFactorToGlobalGraph(const double lidarTimeKm1, const double lidarTimeK, const gtsam::Pose3& pose);
  void addPoseUnaryFactorToFallbackGraph(const double lidarTimeK, const gtsam::Pose3& pose);
  void addGnssPositionUnaryFactor(double gnssTime, const gtsam::Vector3& position);
  void addGnssHeadingUnaryFactor(double gnssTime, const gtsam::Vector3& heading, double measuredYaw);
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
  /// Setters
  inline void setAccNoiseDensity(double val) { accNoiseDensity_ = val; }
  inline void setAccBiasRandomWalk(double val) { accBiasRandomWalk_ = val; }
  inline void setAccBiasPrior(double val) { accBiasPrior_ = val; }
  inline void setGyroNoiseDensity(double val) { gyrNoiseDensity_ = val; }
  inline void setGyrBiasRandomWalk(double val) { gyrBiasRandomWalk_ = val; }
  inline void setIntegrationNoiseDensity(double val) { integrationNoiseDensity_ = val; }
  inline void setBiasAccOmegaPreint(double val) { biasAccOmegaPreint_ = val; }
  inline void setGyrBiasPrior(Eigen::Vector3d gyrBiasPrior) { gyrBiasPrior_ = gyrBiasPrior; }
  inline void setSmootherLag(double val) { smootherLag_ = val; }
  inline void setIterations(int val) { additonalIterations_ = val; }
  inline void setPositionReLinTh(double val) { posReLinTh_ = val; }
  inline void setRotationReLinTh(double val) { rotReLinTh_ = val; }
  inline void setVelocityReLinTh(double val) { velReLinTh_ = val; }
  inline void setAccBiasReLinTh(double val) { accBiasReLinTh_ = val; }
  inline void setGyrBiasReLinTh(double val) { gyrBiasReLinTh_ = val; }
  inline void setPoseBetweenNoise(const std::vector<double>& v) { poseBetweenNoise_ = v; }
  inline void setPoseUnaryNoise(const std::vector<double>& v) { poseUnaryNoise_ = v; }
  inline void setGnssPositionUnaryNoise(double v) { gnssPositionUnaryNoise_ = v; }
  inline void setGnssHeadingUnaryNoise(double v) { gnssHeadingUnaryNoise_ = v; }
  inline void setImuRate(double d) { imuBuffer_.setImuRate(d); }
  inline void setImuBufferLength(int i) { imuBuffer_.setImuBufferLength(i); }
  inline void setLidarRate(double d) { lidarRate_ = d; }
  inline void setGnssRate(double d) { gnssRate_ = d; }
  inline void setVerboseLevel(int verbose) {
    verboseLevel_ = verbose;
    imuBuffer_.setVerboseLevel(verbose);
  }
  /// Getters
  Eigen::Vector3d& getInitGyrBiasReference() { return gyrBiasPrior_; }
  auto iterations() const { return additonalIterations_; }
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
    return activeGraphPtr_ == fallbackGraphPtr_;
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
  /// IMU Preintegration
  double accNoiseDensity_;                      // continuous-time "Covariance" of accelerometer
  double accBiasRandomWalk_;                    // continuous-time "Covariance" describing accelerometer bias random walk
  double accBiasPrior_;                         // prior/starting value of accelerometer bias
  double gyrNoiseDensity_;                      // continuous-time "Covariance" of gyroscope measurements
  double gyrBiasRandomWalk_;                    // continuous-time "Covariance" describing gyroscope bias random walk
  double integrationNoiseDensity_;              // "Covariance" describing
  double biasAccOmegaPreint_;                   // Describing error of bias for preintegration
  gtsam::Vector3 gyrBiasPrior_{0.0, 0.0, 0.0};  // prior/starting value of gyroscope bias
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
  /// LiDAR
  std::vector<double> poseBetweenNoise_;  // ORDER RPY(rad) - XYZ(meters)
  std::vector<double> poseUnaryNoise_;    // ORDER RPY(rad) - XYZ(meters)
  /// GNSS Unary Factor Noise
  double gnssPositionUnaryNoise_;
  double gnssHeadingUnaryNoise_;
  /// Zero Velocity Factor
  double zeroMotionTh_ = 0.01;  // Zero motion threshold meters
  // Timing
  double lidarRate_ = 10.0;
  double gnssRate_ = 20.0;
  // Verbose
  int verboseLevel_ = 0;
};
}  // namespace compslam_se

#endif