#include "fg_filtering/GraphManager.hpp"

namespace fg_filtering {

bool GraphManager::initPoseVelocityBiasGraph(const double ts, const gtsam::Pose3 init_pose) {
  // Set graph relinearization thresholds - must be lower case letters, check:gtsam::symbol_shorthand
  gtsam::FastMap<char, gtsam::Vector> relinTh;
  relinTh['x'] =
      (gtsam::Vector(6) << _rotReLinTh, _rotReLinTh, _rotReLinTh, _posReLinTh, _posReLinTh, _posReLinTh).finished();
  relinTh['v'] = (gtsam::Vector(3) << _velReLinTh, _velReLinTh, _velReLinTh).finished();
  relinTh['b'] = (gtsam::Vector(6) << _accBiasReLinTh, _accBiasReLinTh, _accBiasReLinTh, _gyrBiasReLinTh,
                  _gyrBiasReLinTh, _gyrBiasReLinTh)
                     .finished();
  _params.relinearizeThreshold = relinTh;
  _params.factorization = gtsam::ISAM2Params::QR;  // CHOLESKY:Fast but non-stable //QR:Slower but more stable in poorly
                                                   // conditioned problems

  // Create Prior factor and Initialize factor graph
  // Prior factor noise
  auto poseNoise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-6, 1e-6, 1e-6).finished());  // rad,rad,rad,m, m, m
  auto velocityNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e-3);         // m/s
  auto biasNoise =
      gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3).finished());
  // Create prior factors
  _newFactors.resize(0);
  _newFactors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      X(_state_key), init_pose,
      poseNoise);  // POSE - PriorFactor format is (key,value,matrix) value is same type as type of PriorFactor
  _newFactors.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(_state_key), gtsam::Vector3(0, 0, 0),
                                                                 velocityNoise);  // VELOCITY
  _newFactors.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(_state_key), *_imuBiasPrior,
                                                                               biasNoise);  // BIAS
  // Initial estimate
  gtsam::Values estimate;
  estimate.insert(X(_state_key), init_pose);
  estimate.insert(V(_state_key), gtsam::Vector3(0, 0, 0));
  estimate.insert(B(_state_key), *_imuBiasPrior);

  // Initialize factor graph
  _graph = std::make_shared<gtsam::IncrementalFixedLagSmoother>(_smootherLag, _params);
  _graph->params().print("Factor Graph Parameters:");
  std::cout << "Pose Between Factor Noise - RPY(rad): " << _poseNoise[0] << "," << _poseNoise[1] << "," << _poseNoise[2]
            << ", XYZ(m): " << _poseNoise[3] << "," << _poseNoise[4] << "," << _poseNoise[5] << std::endl;

  // Add prior factor to graph and update
  std::map<gtsam::Key, double> keyTimestampMap;
  valuesToKeyTimeStampMap(estimate, ts, keyTimestampMap);
  _graph->update(_newFactors, estimate, keyTimestampMap);

  // Reset
  _newFactors.resize(0);

  // Update Current State
  _state.updateNavStateAndBias(_state_key, ts, gtsam::NavState(init_pose, gtsam::Vector3(0, 0, 0)), *_imuBiasPrior);

  // //DEBUG
  // gtsam::Values result = _graph->calculateEstimate();
  // result.print("Prior Factor Result:");
  // auto& ns = _state.navState();
  // std::cout << "Init t(x,y,z): " << ns.position().transpose()<< ", RPY(deg): " << ns.attitude().rpy().transpose() *
  // (180.0 / M_PI) << ", v(x,y,z): " << ns.velocity().transpose()  << "\n";
  // //DEBUG

  return true;
}

bool GraphManager::initImuIntegrator(const double g) {
  // Initialize IMU Preintegrator
  _imuParams = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedD(g);
  _imuParams->accelerometerCovariance = gtsam::Matrix33::Identity(3, 3) * _accNoiseDensity;
  _imuParams->biasAccCovariance = gtsam::Matrix33::Identity(3, 3) * _accBiasRandomWalk;
  _imuParams->gyroscopeCovariance = gtsam::Matrix33::Identity(3, 3) * _gyrNoiseDensity;
  _imuParams->biasOmegaCovariance = gtsam::Matrix33::Identity(3, 3) * _gyrBiasRandomWalk;
  _imuParams->integrationCovariance =
      gtsam::Matrix33::Identity(3, 3) * 1.0e-8;  // error committed in integrating position from velocities
  // _imuParams->biasAccOmegaInt = gtsam::Matrix66::Identity(6, 6) * 1.0e-5; // error in the bias used for
  // preintegration
  gtsam::Vector3 acc_bias_prior(_accBiasPrior, _accBiasPrior, _accBiasPrior);
  gtsam::Vector3 gyr_bias_prior(_gyrBiasPrior, _gyrBiasPrior, _gyrBiasPrior);
  _imuBiasPrior = std::make_shared<gtsam::imuBias::ConstantBias>(acc_bias_prior, gyr_bias_prior);
  _imuPreintegrator = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(_imuParams, *_imuBiasPrior);
  _imuParams->print("IMU Preintegration Parameters:");
  return true;
}

bool GraphManager::updateImuIntegrator(const IMUMap& imu_meas) {
  if (imu_meas.size() < 2) {
    std::cout << "updateImuIntegrator --- Received less than 2 IMU messages --- No Preintegration done" << std::endl;
    return false;
  }

  // Reset IMU Pre-integration
  _imuPreintegrator->resetIntegrationAndSetBias(_state.imuBias());

  // Start integrating with imu_meas.begin()+1 meas to calculate dt //NOTE imu_meas.begin() meas was integrated before
  auto curr_itr = imu_meas.begin();
  auto prev_itr = curr_itr;
  ++curr_itr;

  // Calculate dt and integrate IMU measurements
  size_t count = 0;
  for (; curr_itr != imu_meas.end(); ++curr_itr, ++prev_itr) {
    double dt = curr_itr->first - prev_itr->first;
    _imuPreintegrator->integrateMeasurement(curr_itr->second.head<3>(),  // acc
                                            curr_itr->second.tail<3>(),  // gyro
                                            dt);                         // delta t
    ++count;
  }

  return true;
}

bool GraphManager::addImuFactor(const gtsam::Key old_key, const gtsam::Key new_key, const IMUMap& imu_meas) {
  // Update IMU preintegrator
  bool success = updateImuIntegrator(imu_meas);
  if (!success) return success;

  // Create IMU Factor
  gtsam::CombinedImuFactor imu_factor(X(old_key), V(old_key), X(new_key), V(new_key), B(old_key), B(new_key),
                                      *_imuPreintegrator);

  // Add factor
  _newFactors.add(imu_factor);

  return success;
}

void GraphManager::addPoseBetweenFactor(const gtsam::Key old_key, const gtsam::Key new_key, const gtsam::Pose3& pose) {
  // Create Pose BetweenFactor and add
  auto poseNoiseModel = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << _poseNoise[0], _poseNoise[1], _poseNoise[2], _poseNoise[3], _poseNoise[4], _poseNoise[5])
          .finished());  // rad,rad,rad,m,m,m
  gtsam::BetweenFactor<gtsam::Pose3> poseBetweenFactor(X(old_key), X(new_key), pose, poseNoiseModel);
  _newFactors.add(poseBetweenFactor);
}

void GraphManager::updateGraphAndState(const double new_ts, const gtsam::Key new_key, const bool debug /*= false*/) {
  auto t1 = std::chrono::high_resolution_clock::now();
  // Create propogated state estimate using IMU
  gtsam::NavState imuPropogatedState = _imuPreintegrator->predict(_state.navState(), _state.imuBias());
  gtsam::Values estimate;
  estimate.insert(X(new_key), imuPropogatedState.pose());
  estimate.insert(V(new_key), imuPropogatedState.velocity());
  estimate.insert(B(new_key), _state.imuBias());

  // Update graph
  std::map<gtsam::Key, double> keyTimestampMap;
  valuesToKeyTimeStampMap(estimate, new_ts, keyTimestampMap);
  _graph->update(_newFactors, estimate, keyTimestampMap);
  _newFactors.resize(0);

  // Additional iterations
  for (size_t itr = 0; itr < _additonalIterations; ++itr) _graph->update();

  // Update State
  auto result = _graph->calculateEstimate();
  _state.updateNavStateAndBias(
      new_key, new_ts, gtsam::NavState(result.at<gtsam::Pose3>(X(new_key)), result.at<gtsam::Vector3>(V(new_key))),
      result.at<gtsam::imuBias::ConstantBias>(B(new_key)));
  auto t2 = std::chrono::high_resolution_clock::now();

  // Debug
  if (debug) {
    auto& ns = _state.navState();
    std::cout << "Optimized t(x,y,z): " << ns.position().transpose()
              << ", RPY(deg): " << ns.attitude().rpy().transpose() * (180.0 / M_PI)
              << ", time(ms): " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
              << std::endl;  // lag:1s->0-1ms, lag:5s->1-2ms
  }
  // Debug
}

bool GraphManager::zeroMotionFactor(const gtsam::Key old_key, const gtsam::Key new_key, const gtsam::Pose3 pose) {
  // Check external motion
  if (pose.translation().norm() > _zeroMotionTh) {
    _detectionCount = 0;
    return false;
  }

  // Check IMU motion
  gtsam::NavState imuPropogatedState = _imuPreintegrator->predict(gtsam::NavState(), _state.imuBias());
  if (imuPropogatedState.position().norm() > _zeroMotionTh) {
    _detectionCount = 0;
    return false;
  }

  // Check consective zero-motion detections
  ++_detectionCount;
  if (_detectionCount < _minDetections) return false;

  // Add Zero Pose Factor
  _newFactors.add(gtsam::BetweenFactor<gtsam::Pose3>(X(old_key), X(new_key), gtsam::Pose3::identity(),
                                                     gtsam::noiseModel::Isotropic::Sigma(6, 1e-3)));
  // Add Zero Velocity Factor
  _newFactors.add(gtsam::PriorFactor<gtsam::Vector3>(V(old_key), gtsam::Vector3::Zero(),
                                                     gtsam::noiseModel::Isotropic::Sigma(3, 1e-3)));

  // Re-detect Zero Motion
  _detectionCount = _minDetections / 2;

  // DEBUG
  std::cout << "Zero Motion Factor added between: " << old_key << "/" << new_key << std::endl;
  // DEBUG

  return true;
}

bool GraphManager::addGravityRollPitchFactor(const gtsam::Key key, const gtsam::Rot3 imu_attitude) {
  static auto imuEstNoise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 1e-6, 1e-6, 1e+10, 1e+10, 1e+10, 1e+10).finished());  // rad,rad,rad,m, m, m
  _newFactors.add(
      gtsam::PriorFactor<gtsam::Pose3>(X(key), gtsam::Pose3(imu_attitude, gtsam::Point3::Zero()), imuEstNoise));
  return true;
}

}  // namespace fg_filtering