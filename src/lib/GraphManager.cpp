#include "fg_filtering/GraphManager.hpp"

namespace fg_filtering {

// Public --------------------------------------------------------------------

bool GraphManager::initPoseVelocityBiasGraph(const double ts, const gtsam::Pose3 init_pose) {
  // Set graph relinearization thresholds - must be lower case letters, check:gtsam::symbol_shorthand
  gtsam::FastMap<char, gtsam::Vector> relinTh;
  relinTh['x'] =
      (gtsam::Vector(6) << _rotReLinTh, _rotReLinTh, _rotReLinTh, _posReLinTh, _posReLinTh, _posReLinTh).finished();
  relinTh['v'] = (gtsam::Vector(3) << _velReLinTh, _velReLinTh, _velReLinTh).finished();
  relinTh['b'] = (gtsam::Vector(6) << _accBiasReLinTh, _accBiasReLinTh, _accBiasReLinTh, _gyrBiasReLinTh,
                  _gyrBiasReLinTh, _gyrBiasReLinTh)
                     .finished();
  _isamParams.relinearizeThreshold = relinTh;
  _isamParams.factorization = gtsam::ISAM2Params::QR;  // CHOLESKY:Fast but non-stable //QR:Slower but more stable in
                                                       // poorly conditioned problems

  // Create Prior factor and Initialize factor graph
  // Prior factor noise
  auto poseNoise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-6, 1e-6, 1e-6).finished());  // rad,rad,rad,m, m, m
  auto velocityNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e-3);         // m/s
  auto biasNoise =
      gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3).finished());
  // Create prior factors
  _newGraphFactors.resize(0);
  _newGraphFactors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      X(_stateKey), init_pose,
      poseNoise);  // POSE - PriorFactor format is (key,value,matrix) value is same type as type of PriorFactor
  _newGraphFactors.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(_stateKey), gtsam::Vector3(0, 0, 0),
                                                                      velocityNoise);  // VELOCITY
  _newGraphFactors.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(_stateKey), *_imuBiasPrior,
                                                                                    biasNoise);  // BIAS
  // Initial estimate
  gtsam::Values estimate;
  estimate.insert(X(_stateKey), init_pose);
  estimate.insert(V(_stateKey), gtsam::Vector3(0, 0, 0));
  estimate.insert(B(_stateKey), *_imuBiasPrior);

  // Initialize factor graph
  _mainGraph = std::make_shared<gtsam::IncrementalFixedLagSmoother>(_smootherLag, _isamParams);
  _mainGraph->params().print("Factor Graph Parameters:");
  std::cout << "Pose Between Factor Noise - RPY(rad): " << _poseNoise[0] << "," << _poseNoise[1] << "," << _poseNoise[2]
            << ", XYZ(m): " << _poseNoise[3] << "," << _poseNoise[4] << "," << _poseNoise[5] << std::endl;

  // Add prior factor to graph and update
  std::map<gtsam::Key, double> keyTimestampMap;
  valuesToKeyTimeStampMap(estimate, ts, keyTimestampMap);
  _mainGraph->update(_newGraphFactors, estimate, keyTimestampMap);

  // Reset
  _newGraphFactors.resize(0);

  // Update Current State
  _graphState.updateNavStateAndBias(_stateKey, ts, gtsam::NavState(init_pose, gtsam::Vector3(0, 0, 0)), *_imuBiasPrior);

  return true;
}

bool GraphManager::initImuIntegrators(const double g) {
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

  // Init preintegrators
  _imuBufferPreintegrator = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(_imuParams, *_imuBiasPrior);
  _imuStepPreintegrator = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(_imuParams, *_imuBiasPrior);
  _imuParams->print("IMU Preintegration Parameters:");
  return true;
}

gtsam::NavState GraphManager::addImuFactorAndGetState(const double imuTime_k) {
  // Operating on graph data --> acquire mutex during whole method
  const std::lock_guard<std::mutex> operateOnGraphDataLock(_operateOnGraphDataMutex);

  // Get new key
  gtsam::Key oldKey = _stateKey;
  gtsam::Key newKey = newStateKey();
  // Write current time
  _stateTime = imuTime_k;

  // Add to key buffer
  _imuBuffer.addToKeyBuffer(imuTime_k, newKey);

  // Get last two measurements from buffer
  IMUMap imuMeas;
  _imuBuffer.getLastTwoMeasurements(imuMeas);

  // Update IMU preintegrator
  bool success = _updateImuIntegrators(imuMeas);

  // Create and add IMU Factor
  gtsam::CombinedImuFactor imuFactor(X(oldKey), V(oldKey), X(newKey), V(newKey), B(oldKey), B(newKey),
                                     *_imuBufferPreintegrator);
  _newGraphFactors.add(imuFactor);
  // Predict propagated state
  ROS_INFO_STREAM("Propagated state (key " << oldKey << ") before prediction: " << _imuPropogatedState.pose().translation());
  _imuPropogatedState = _imuStepPreintegrator->predict(_imuPropogatedState, _graphState.imuBias());
  ROS_INFO_STREAM("Propagated state (key " << newKey << ") after prediction prediction: " << _imuPropogatedState.pose().translation());
  ROS_INFO("----------------------------");
  // Add IMU Value
  _newGraphValues.insert(X(newKey), _imuPropogatedState.pose());
  _newGraphValues.insert(V(newKey), _imuPropogatedState.velocity());
  _newGraphValues.insert(B(newKey), _graphState.imuBias());

  // Return copy of propagated state (for publishing)
  return _imuPropogatedState;
}

void GraphManager::addPoseBetweenFactor(const gtsam::Pose3& pose, const double lidarTime_km1,
                                        const double lidarTime_k) {
  // Operating on graph data --> acquire mutex during whole method
  const std::lock_guard<std::mutex> operateOnGraphDataLock(_operateOnGraphDataMutex);

  // Find closest lidar keys in existing graph
  IMUMapItr lidarMapItr_km1, lidarMapItr_k;
  _imuBuffer.getClosestIMUBufferIteratorToTime(lidarTime_km1, lidarMapItr_km1);
  _imuBuffer.getClosestIMUBufferIteratorToTime(lidarTime_k, lidarMapItr_k);
  gtsam::Key key_km1, key_k;
  _imuBuffer.getCorrespondingGtsamKey(lidarMapItr_km1->first, key_km1);
  _imuBuffer.getCorrespondingGtsamKey(lidarMapItr_k->first, key_k);
  std::cout << "Found time stamps are: " << lidarMapItr_km1->first << " and " << lidarMapItr_k->first << std::endl;
  std::cout << "Current key: " << _stateKey << ", found lidar keys are: "
            << key_km1 << " and " << key_k << std::endl;
  gtsam::Key closestLidarKey_km1, closestLidarKey_k;
  _imuBuffer.getCorrespondingGtsamKey(lidarMapItr_km1->first, closestLidarKey_km1);
  _imuBuffer.getCorrespondingGtsamKey(lidarMapItr_k->first, closestLidarKey_k);

  // Create Pose BetweenFactor and add
  auto poseNoiseModel = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << _poseNoise[0], _poseNoise[1], _poseNoise[2], _poseNoise[3], _poseNoise[4], _poseNoise[5])
          .finished());  // rad,rad,rad,m,m,m
  gtsam::BetweenFactor<gtsam::Pose3> poseBetweenFactor(X(closestLidarKey_km1), X(closestLidarKey_k), pose,
                                                       poseNoiseModel);
  _newGraphFactors.add(poseBetweenFactor);
}

void addPoseUnaryFactor(const gtsam::Key old_key, const gtsam::Key new_key, const gtsam::Pose3& pose) {
  // TODO: TO BE IMPLEMENTED
}

bool GraphManager::addZeroMotionFactor(const gtsam::Key old_key, const gtsam::Key new_key, const gtsam::Pose3 pose) {
  // Check external motion
  if (pose.translation().norm() > _zeroMotionTh) {
    _detectionCount = 0;
    return false;
  }

  // Check IMU motion
  gtsam::NavState imuPropogatedState = _imuStepPreintegrator->predict(gtsam::NavState(), _graphState.imuBias());
  if (imuPropogatedState.position().norm() > _zeroMotionTh) {
    _detectionCount = 0;
    return false;
  }

  // Check consective zero-motion detections
  ++_detectionCount;
  if (_detectionCount < _minDetections) return false;

  // Add Zero Pose Factor
  _newGraphFactors.add(gtsam::BetweenFactor<gtsam::Pose3>(X(old_key), X(new_key), gtsam::Pose3::identity(),
                                                          gtsam::noiseModel::Isotropic::Sigma(6, 1e-3)));
  // Add Zero Velocity Factor
  _newGraphFactors.add(gtsam::PriorFactor<gtsam::Vector3>(V(old_key), gtsam::Vector3::Zero(),
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
  _newGraphFactors.add(
      gtsam::PriorFactor<gtsam::Pose3>(X(key), gtsam::Pose3(imu_attitude, gtsam::Point3::Zero()), imuEstNoise));
  return true;
}

void GraphManager::updateGraphAndState() {
  gtsam::NonlinearFactorGraph newGraphFactors;
  gtsam::Values newGraphValues;
  gtsam::Key currentKey;
  double currentTime;
  // Mutex Block 1 -----------------
  {
    // Lock
    const std::lock_guard<std::mutex> operateOnGraphDataLock(_operateOnGraphDataMutex);
    // Get copy of factors and values
    newGraphFactors = _newGraphFactors;
    newGraphValues = _newGraphValues;
    // Empty graph buffers
    _newGraphFactors.resize(0);
    _newGraphValues.clear();
    // Empty Buffer Preintegrator --> everything missed during the update will be in here
    _imuBufferPreintegrator->resetIntegrationAndSetBias(_graphState.imuBias());
    // Get current key and time
    currentKey = _stateKey;
    currentTime = _stateTime;
  }
  // Graph Update (time consuming) -------------------
  std::map<gtsam::Key, double> keyTimestampMap;
  valuesToKeyTimeStampMap(newGraphValues, currentTime, keyTimestampMap);
  _mainGraph->update(newGraphFactors, newGraphValues, keyTimestampMap);
  // Additional iterations
  for (size_t itr = 0; itr < _additonalIterations; ++itr) _mainGraph->update();
  // Compute result
  auto result = _mainGraph->calculateEstimate();
  // Mutex block 2 ------------------
  {
    // Lock
    const std::lock_guard<std::mutex> operateOnGraphDataLock(_operateOnGraphDataMutex);
    // Update Graph State
    _graphState.updateNavStateAndBias(
        currentKey, currentTime,
        gtsam::NavState(result.at<gtsam::Pose3>(X(currentKey)), result.at<gtsam::Vector3>(V(currentKey))),
        result.at<gtsam::imuBias::ConstantBias>(B(currentKey)));
    // Predict from solution to obtain refined propagated state
    ROS_WARN_STREAM("Graph state (key " << currentKey << ") before prediction: " << _graphState.navState().pose().translation());
    _imuPropogatedState = _imuBufferPreintegrator->predict(_graphState.navState(), _graphState.imuBias());
    ROS_WARN_STREAM("Propagated state (key " << _stateKey << ") after prediction: " << _imuPropogatedState.pose().translation());
  }
}

// Private --------------------------------------------------------------------

bool GraphManager::_updateImuIntegrators(const IMUMap& imuMeas) {
  if (imuMeas.size() < 2) {
    std::cout << "_updateImuIntegrators --- Received less than 2 IMU messages --- No Preintegration done" << std::endl;
    return false;
  }

  // Reset IMU Step Preintegration
  _imuStepPreintegrator->resetIntegrationAndSetBias(_graphState.imuBias());

  // Start integrating with imu_meas.begin()+1 meas to calculate dt, imu_meas.begin() meas was integrated before
  auto currItr = imuMeas.begin();
  auto prevItr = currItr;
  ++currItr;

  // Calculate dt and integrate IMU measurements for both preintegrators
  size_t count = 0;
  for (; currItr != imuMeas.end(); ++currItr, ++prevItr) {
    double dt = currItr->first - prevItr->first;
    ROS_INFO_STREAM("IMU acceleration: " << currItr->second.head<3>());
    _imuStepPreintegrator->integrateMeasurement(currItr->second.head<3>(),    // acc
                                                currItr->second.tail<3>(),    // gyro
                                                dt);                          // delta t
    _imuBufferPreintegrator->integrateMeasurement(currItr->second.head<3>(),  // acc
                                                  currItr->second.tail<3>(),  // gyro
                                                  dt);
    ++count;
  }

  return true;
}

}  // namespace fg_filtering