#include "fg_filtering/GraphManager.hpp"

namespace fg_filtering {

// Public --------------------------------------------------------------------

bool GraphManager::initPoseVelocityBiasGraph(const double timeStep, const gtsam::Pose3& init_pose) {
  // Set graph relinearization thresholds - must be lower case letters, check:gtsam::symbol_shorthand
  gtsam::FastMap<char, gtsam::Vector> relinTh;
  relinTh['x'] = (gtsam::Vector(6) << _rotReLinTh, _rotReLinTh, _rotReLinTh, _posReLinTh, _posReLinTh, _posReLinTh).finished();
  relinTh['v'] = (gtsam::Vector(3) << _velReLinTh, _velReLinTh, _velReLinTh).finished();
  relinTh['b'] =
      (gtsam::Vector(6) << _accBiasReLinTh, _accBiasReLinTh, _accBiasReLinTh, _gyrBiasReLinTh, _gyrBiasReLinTh, _gyrBiasReLinTh).finished();
  _isamParams.relinearizeThreshold = relinTh;
  _isamParams.factorization = gtsam::ISAM2Params::QR;  // CHOLESKY:Fast but non-stable //QR:Slower but more stable in
                                                       // poorly conditioned problems

  // Create Prior factor and Initialize factor graph
  // Prior factor noise
  auto poseNoise =
      gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-6, 1e-6, 1e-6).finished());  // rad,rad,rad,m, m, m
  auto velocityNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e-3);                                             // m/s
  auto biasNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3).finished());
  // Create prior factors
  _newGraphFactors.resize(0);
  _newGraphFactors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      X(_stateKey), init_pose,
      poseNoise);  // POSE - PriorFactor format is (key,value,matrix) value is same type as type of PriorFactor
  _newGraphFactors.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(_stateKey), gtsam::Vector3(0, 0, 0),
                                                                      velocityNoise);  // VELOCITY
  _newGraphFactors.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(_stateKey), *_imuBiasPriorPtr,
                                                                                    biasNoise);  // BIAS
  // Initial estimate
  gtsam::Values estimate;
  estimate.insert(X(_stateKey), init_pose);
  estimate.insert(V(_stateKey), gtsam::Vector3(0, 0, 0));
  estimate.insert(B(_stateKey), *_imuBiasPriorPtr);

  // Initialize factor graph
  _mainGraphPtr = std::make_shared<gtsam::IncrementalFixedLagSmoother>(_smootherLag, _isamParams);
  _mainGraphPtr->params().print("Factor Graph Parameters:");
  std::cout << "Pose Between Factor Noise - RPY(rad): " << _poseNoise[0] << "," << _poseNoise[1] << "," << _poseNoise[2]
            << ", XYZ(m): " << _poseNoise[3] << "," << _poseNoise[4] << "," << _poseNoise[5] << std::endl;

  // Add prior factor to graph and update
  std::map<gtsam::Key, double> keyTimestampMap;
  valuesToKeyTimeStampMap(estimate, timeStep, keyTimestampMap);
  _mainGraphPtr->update(_newGraphFactors, estimate, keyTimestampMap);

  // Reset
  _newGraphFactors.resize(0);

  // Update Current State
  _graphState.updateNavStateAndBias(_stateKey, timeStep, gtsam::NavState(init_pose, gtsam::Vector3(0, 0, 0)), *_imuBiasPriorPtr);
  _imuPropagatedState = gtsam::NavState(init_pose, gtsam::Vector3(0, 0, 0));
  return true;
}

bool GraphManager::initImuIntegrators(const double g, const std::string& imuGravityDirection) {
  // Initialize IMU Preintegrator
  if (imuGravityDirection == "up") {
    _imuParamsPtr = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(g);  // ROS convention
  } else if (imuGravityDirection == "down") {
    _imuParamsPtr = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedD(g);
  } else {
    throw std::runtime_error("Gravity direction must be either 'up' or 'down'.");
  }

  _imuParamsPtr->accelerometerCovariance = gtsam::Matrix33::Identity(3, 3) * _accNoiseDensity;
  _imuParamsPtr->biasAccCovariance = gtsam::Matrix33::Identity(3, 3) * _accBiasRandomWalk;
  _imuParamsPtr->gyroscopeCovariance = gtsam::Matrix33::Identity(3, 3) * _gyrNoiseDensity;
  _imuParamsPtr->biasOmegaCovariance = gtsam::Matrix33::Identity(3, 3) * _gyrBiasRandomWalk;
  _imuParamsPtr->integrationCovariance =
      gtsam::Matrix33::Identity(3, 3) * integrationNoiseDensity_;  // error committed in integrating position from velocities
  _imuParamsPtr->biasAccOmegaInt = gtsam::Matrix66::Identity(6, 6) * biasAccOmegaPreint_;  // covariance of bias used for preintegration
  gtsam::Vector3 acc_bias_prior(_accBiasPrior, _accBiasPrior, _accBiasPrior);
  gtsam::Vector3 gyr_bias_prior(_gyrBiasPrior, _gyrBiasPrior, _gyrBiasPrior);
  _imuBiasPriorPtr = std::make_shared<gtsam::imuBias::ConstantBias>(acc_bias_prior, gyr_bias_prior);

  // Init preintegrators
  _imuBufferPreintegratorPtr = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(_imuParamsPtr, *_imuBiasPriorPtr);
  _imuStepPreintegratorPtr = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(_imuParamsPtr, *_imuBiasPriorPtr);
  _imuParamsPtr->print("IMU Preintegration Parameters:");
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
  _updateImuIntegrators(imuMeas);

  // Create and add IMU Factor
  gtsam::CombinedImuFactor imuFactor(X(oldKey), V(oldKey), X(newKey), V(newKey), B(oldKey), B(newKey), *_imuStepPreintegratorPtr);
  _newGraphFactors.add(imuFactor);
  // Predict propagated state
  // ROS_INFO_STREAM("Propagated state (key " << oldKey
  //                                         << ") before prediction: " << _imuPropagatedState.pose().translation());
  _imuPropagatedState = _imuStepPreintegratorPtr->predict(_imuPropagatedState, _graphState.imuBias());
  // ROS_INFO_STREAM("Propagated state (key "
  //                 << newKey << ") after prediction prediction: " << _imuPropagatedState.pose().translation());
  // ROS_INFO("----------------------------");

  // Add to IMU pose buffer
  _imuBuffer.addImuPoseToBuffer(imuTime_k, _imuPropagatedState.pose());

  // Add IMU Value
  _newGraphValues.insert(X(newKey), _imuPropagatedState.pose());
  _newGraphValues.insert(V(newKey), _imuPropagatedState.velocity());
  _newGraphValues.insert(B(newKey), _graphState.imuBias());

  // Return copy of propagated state (for publishing)
  return _imuPropagatedState;
}

void GraphManager::addPoseBetweenFactor(const gtsam::Pose3& pose, const double lidarTime_km1, const double lidarTime_k) {
  // Operating on graph data --> acquire mutex during whole method
  const std::lock_guard<std::mutex> operateOnGraphDataLock(_operateOnGraphDataMutex);

  // Find closest lidar keys in existing graph
  IMUMapItr lidarMapItr_km1, lidarMapItr_k;
  _imuBuffer.getClosestIMUBufferIteratorToTime(lidarTime_km1, lidarMapItr_km1);
  _imuBuffer.getClosestIMUBufferIteratorToTime(lidarTime_k, lidarMapItr_k);
  gtsam::Key closestLidarKey_km1, closestLidarKey_k;
  _imuBuffer.getCorrespondingGtsamKey(lidarTime_km1, closestLidarKey_km1);
  _imuBuffer.getCorrespondingGtsamKey(lidarTime_k, closestLidarKey_k);
  // ROS_INFO_STREAM("Found time stamps are: " << lidarMapItr_km1->first << " and " << lidarMapItr_k->first);
  ROS_INFO_STREAM("Current key: " << _stateKey << ", found lidar keys are: " << closestLidarKey_km1 << " and " << closestLidarKey_k);
  // ROS_WARN_STREAM("LiDAR delta pose: " << pose);
  gtsam::Pose3 imuPose_km1, imuPose_k;
  //_imuBuffer.getCorrespondingIMUGraphPose(lidarTime_km1, imuPose_km1);
  //_imuBuffer.getCorrespondingIMUGraphPose(lidarTime_k, imuPose_k);
  // ROS_WARN_STREAM("Delta pose in graph from IMU: " << imuPose_km1.inverse() * imuPose_k);

  // Create Pose BetweenFactor and add
  auto poseNoiseModel = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << _poseNoise[0], _poseNoise[1], _poseNoise[2], _poseNoise[3], _poseNoise[4], _poseNoise[5])
          .finished());  // rad,rad,rad,m,m,m
  gtsam::BetweenFactor<gtsam::Pose3> poseBetweenFactor(X(closestLidarKey_km1), X(closestLidarKey_k), pose, poseNoiseModel);
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
  gtsam::NavState imuPropagatedState = _imuStepPreintegratorPtr->predict(gtsam::NavState(), _graphState.imuBias());
  if (imuPropagatedState.position().norm() > _zeroMotionTh) {
    _detectionCount = 0;
    return false;
  }

  // Check consective zero-motion detections
  ++_detectionCount;
  if (_detectionCount < _minDetections) return false;

  // Add Zero Pose Factor
  _newGraphFactors.add(
      gtsam::BetweenFactor<gtsam::Pose3>(X(old_key), X(new_key), gtsam::Pose3::identity(), gtsam::noiseModel::Isotropic::Sigma(6, 1e-3)));
  // Add Zero Velocity Factor
  _newGraphFactors.add(
      gtsam::PriorFactor<gtsam::Vector3>(V(old_key), gtsam::Vector3::Zero(), gtsam::noiseModel::Isotropic::Sigma(3, 1e-3)));

  // Re-detect Zero Motion
  _detectionCount = _minDetections / 2;

  // DEBUG
  std::cout << "Zero Motion Factor added between: " << old_key << "/" << new_key << std::endl;
  // DEBUG

  return true;
}

bool GraphManager::addGravityRollPitchFactor(const gtsam::Key key, const gtsam::Rot3 imu_attitude) {
  static auto imuEstNoise =
      gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-6, 1e-6, 1e+10, 1e+10, 1e+10, 1e+10).finished());  // rad,rad,rad,m, m, m
  _newGraphFactors.add(gtsam::PriorFactor<gtsam::Pose3>(X(key), gtsam::Pose3(imu_attitude, gtsam::Point3::Zero()), imuEstNoise));
  return true;
}

gtsam::NavState GraphManager::updateGraphAndState() {
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
    _imuBufferPreintegratorPtr->resetIntegrationAndSetBias(_graphState.imuBias());
    // Get current key and time
    currentKey = _stateKey;
    currentTime = _stateTime;
  }
  // Graph Update (time consuming) -------------------
  std::map<gtsam::Key, double> keyTimestampMap;
  valuesToKeyTimeStampMap(newGraphValues, currentTime, keyTimestampMap);
  _mainGraphPtr->update(newGraphFactors, newGraphValues, keyTimestampMap);
  // Additional iterations
  for (size_t itr = 0; itr < _additonalIterations; ++itr) _mainGraphPtr->update();
  // Compute result
  auto result = _mainGraphPtr->calculateEstimate();
  // Mutex block 2 ------------------
  gtsam::NavState navState = gtsam::NavState(result.at<gtsam::Pose3>(X(currentKey)), result.at<gtsam::Vector3>(V(currentKey)));
  {
    // Lock
    const std::lock_guard<std::mutex> operateOnGraphDataLock(_operateOnGraphDataMutex);
    // Update Graph State
    _graphState.updateNavStateAndBias(currentKey, currentTime, navState, result.at<gtsam::imuBias::ConstantBias>(B(currentKey)));
    // Predict from solution to obtain refined propagated state
    _imuPropagatedState = _imuBufferPreintegratorPtr->predict(_graphState.navState(), _graphState.imuBias());
  }
  return navState;
}

// Private --------------------------------------------------------------------

void GraphManager::_updateImuIntegrators(const IMUMap& imuMeas) {
  if (imuMeas.size() < 2) {
    std::cout << "_updateImuIntegrators --- Received less than 2 IMU messages --- No Preintegration done" << std::endl;
    return;
  }

  // Reset IMU Step Preintegration
  _imuStepPreintegratorPtr->resetIntegrationAndSetBias(_graphState.imuBias());

  // Start integrating with imu_meas.begin()+1 meas to calculate dt, imu_meas.begin() meas was integrated before
  auto currItr = imuMeas.begin();
  auto prevItr = currItr;
  ++currItr;

  // Calculate dt and integrate IMU measurements for both preintegrators
  size_t count = 0;
  for (; currItr != imuMeas.end(); ++currItr, ++prevItr) {
    double dt = currItr->first - prevItr->first;
    _imuStepPreintegratorPtr->integrateMeasurement(currItr->second.head<3>(),    // acc
                                                   currItr->second.tail<3>(),    // gyro
                                                   dt);                          // delta t
    _imuBufferPreintegratorPtr->integrateMeasurement(currItr->second.head<3>(),  // acc
                                                     currItr->second.tail<3>(),  // gyro
                                                     dt);
    ++count;
  }
}

}  // namespace fg_filtering