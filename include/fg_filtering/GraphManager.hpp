#ifndef GRAPH_MANAGER_HPP_
#define GRAPH_MANAGER_HPP_

//C++
#include <chrono>
#include <vector>

//GTSAM
#define SLOW_BUT_CORRECT_BETWEENFACTOR  // increases accuracy in handling rotations
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include "fg_filtering/GraphState.hpp"
#include "fg_filtering/ImuManager.hpp"

using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X;  // Pose3 (R,t)

namespace fg_filtering {
class GraphManager {
 public:
  //Constructor / Destructor
  GraphManager(){};
  ~GraphManager(){};

  //Initialize Factor graph with Pose,Velocity and Bias states
  bool initPoseVelocityBiasGraph(const double ts, const gtsam::Pose3 init_pose);

  //Initialize IMU integrator
  bool initImuIntegrator(const double g);

  //Update IMU integrator with new measurements - Resets bias
  bool updateImuIntegrator(const IMUMap& imu_meas);

  //Add IMU factor to graph
  bool addImuFactor(const gtsam::Key old_key, const gtsam::Key new_key, const IMUMap& imu_meas);

  //Add a pose between factor
  void addPoseBetweenFactor(const gtsam::Key old_key, const gtsam::Key new_key, const gtsam::Pose3& pose);

  //Update graph and get new state
  void updateGraphAndState(const double new_ts, const gtsam::Key new_key, const bool debug = false);

  //Check if zero motion has occured and add zero pose/velocity factor
  bool zeroMotionFactor(const gtsam::Key old_key, const gtsam::Key new_key, const gtsam::Pose3 pose);

  //Add gravity-aligned roll/ptich when no motion
  bool addGravityRollPitchFactor(const gtsam::Key key, const gtsam::Rot3 imu_attitude);

  //Associate timestamp to each 'value key', e.g. for graph key 0, value keys (x0,v0,b0) need to be associated
  void valuesToKeyTimeStampMap(const gtsam::Values& values, const double ts, std::map<gtsam::Key, double>& key_timestamp_map) {
    for (const auto& value : values)
      key_timestamp_map[value.key] = ts;
  }

  //Accessors - Setters
  void setAccNoiseDensity(double val) { _accNoiseDensity = val; }
  void setAccBiasRandomWalk(double val) { _accBiasRandomWalk = val; }
  void setAccBiasPrior(double val) { _accBiasPrior = val; }
  void setGyroNoiseDensity(double val) { _gyrNoiseDensity = val; }
  void setGyrBiasRandomWalk(double val) { _gyrBiasRandomWalk = val; }
  void setGyrBiasPrior(double val) { _gyrBiasPrior = val; }
  void setSmootherLag(double val) { _smootherLag = val; }
  void setIterations(int val) { _additonalIterations = val; }
  void setPositionReLinTh(double val) { _posReLinTh = val; }
  void setRotationReLinTh(double val) { _rotReLinTh = val; }
  void setVelocityReLinTh(double val) { _velReLinTh = val; }
  void setAccBiasReLinTh(double val) { _accBiasReLinTh = val; }
  void setGyrBiasReLinTh(double val) { _gyrBiasReLinTh = val; }
  void setPoseNoise(const std::vector<double>& v) {_poseNoise = v;}
  //Accessors - Getters
  auto iterations() const { return _additonalIterations; }
  const auto newStateKey() { return ++_state_key; }

  //Objects
  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> _imuParams;
  std::shared_ptr<gtsam::imuBias::ConstantBias> _imuBiasPrior;
  std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> _imuPreintegrator;
  gtsam::ISAM2Params _params;
  gtsam::NonlinearFactorGraph _newFactors;
  std::shared_ptr<gtsam::IncrementalFixedLagSmoother> _graph;
  State _state;

 private:
  //Member variables
  // IMU Preintegration
  double _accNoiseDensity;    // continuous-time "Covariance" of accelerometer
  double _accBiasRandomWalk;  // continuous-time "Covariance" describing accelerometer bias random walk
  double _accBiasPrior;       // prior/starting value of accelerometer bias
  double _gyrNoiseDensity;    // continuous-time "Covariance" of gyroscope measurements
  double _gyrBiasRandomWalk;  // continuous-time "Covariance" describing gyroscope bias random walk
  double _gyrBiasPrior;       // prior/starting value of gyroscope bias
  //Factor Graph
  gtsam::Key _state_key = 0;     //Current state key
  double _smootherLag = 1;       //Lag of fixed lag smoother[seconds]
  int _additonalIterations = 1;  //Additional iterations of graph optimizer after update with new factors
  double _posReLinTh = 0.05;     //Position linearization threshold
  double _rotReLinTh = 0.05;     //Rotation linearization threshold
  double _velReLinTh = 0.1;      //Linear Velocity linearization threshold
  double _accBiasReLinTh = 0.1;  //Accelerometer bias linearization threshold
  double _gyrBiasReLinTh = 0.1;  //Gyroscope bias linearization threshold
  //Pose Between Factor
  std::vector<double> _poseNoise{0.02, 0.02, 0.02, 0.05, 0.05, 0.05};  //ORDER RPY(rad) - XYZ(meters)
  //Zero Velocity Factor
  double _zeroMotionTh = 0.01;              //Zero motion threshold meters
  double _minDetections = 10;               //Number of consective zero motions detected before zero motion factors are added
  double _detectionCount = _minDetections;  //Assumption: Robot starts at rest so initially zero motion is enabled
};
}  //namespace fg_filtering

#endif