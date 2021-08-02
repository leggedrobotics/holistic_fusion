#include "fg_filtering/GraphManager.hpp"

namespace fg_filtering {

// Public --------------------------------------------------------------------

bool GraphManager::initPoseVelocityBiasGraph(const double timeStep, const gtsam::Pose3& init_pose) {
  // Set graph relinearization thresholds - must be lower case letters, check:gtsam::symbol_shorthand
  gtsam::FastMap<char, gtsam::Vector> relinTh;
  relinTh['x'] = (gtsam::Vector(6) << rotReLinTh_, rotReLinTh_, rotReLinTh_, posReLinTh_, posReLinTh_, posReLinTh_).finished();
  relinTh['v'] = (gtsam::Vector(3) << velReLinTh_, velReLinTh_, velReLinTh_).finished();
  relinTh['b'] =
      (gtsam::Vector(6) << accBiasReLinTh_, accBiasReLinTh_, accBiasReLinTh_, gyrBiasReLinTh_, gyrBiasReLinTh_, gyrBiasReLinTh_).finished();
  isamParams_.relinearizeThreshold = relinTh;
  isamParams_.factorization = gtsam::ISAM2Params::QR;  // CHOLESKY:Fast but non-stable //QR:Slower but more stable in
                                                       // poorly conditioned problems

  // Create Prior factor and Initialize factor graph
  // Prior factor noise
  auto poseNoise =
      gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-6, 1e-6, 1e-6).finished());  // rad,rad,rad,m, m, m
  auto velocityNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e-3);                                             // m/s
  auto biasNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3).finished());
  // Create prior factors
  newGraphFactors_.resize(0);
  newGraphFactors_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      gtsam::symbol_shorthand::X(stateKey_), init_pose,
      poseNoise);  // POSE - PriorFactor format is (key,value,matrix) value is same type as type of PriorFactor
  newGraphFactors_.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(gtsam::symbol_shorthand::V(stateKey_), gtsam::Vector3(0, 0, 0),
                                                                      velocityNoise);  // VELOCITY
  newGraphFactors_.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(gtsam::symbol_shorthand::B(stateKey_),
                                                                                    *imuBiasPriorPtr_,
                                                                                    biasNoise);  // BIAS
  // Initial estimate
  gtsam::Values estimate;
  estimate.insert(gtsam::symbol_shorthand::X(stateKey_), init_pose);
  estimate.insert(gtsam::symbol_shorthand::V(stateKey_), gtsam::Vector3(0, 0, 0));
  estimate.insert(gtsam::symbol_shorthand::B(stateKey_), *imuBiasPriorPtr_);

  // Initialize factor graph
  mainGraphPtr_ = std::make_shared<gtsam::IncrementalFixedLagSmoother>(smootherLag_, isamParams_);
  mainGraphPtr_->params().print("Factor Graph Parameters:");
  std::cout << "Pose Between Factor Noise - RPY(rad): " << poseNoise_[0] << "," << poseNoise_[1] << "," << poseNoise_[2]
            << ", XYZ(m): " << poseNoise_[3] << "," << poseNoise_[4] << "," << poseNoise_[5] << std::endl;

  // Add prior factor to graph and update
  std::map<gtsam::Key, double> keyTimestampMap;
  valuesToKeyTimeStampMap(estimate, timeStep, keyTimestampMap);
  mainGraphPtr_->update(newGraphFactors_, estimate, keyTimestampMap);

  // Reset
  newGraphFactors_.resize(0);

  // Update Current State
  graphState_.updateNavStateAndBias(stateKey_, timeStep, gtsam::NavState(init_pose, gtsam::Vector3(0, 0, 0)), *imuBiasPriorPtr_);
  imuPropagatedState_ = gtsam::NavState(init_pose, gtsam::Vector3(0, 0, 0));
  return true;
}

bool GraphManager::initImuIntegrators(const double g, const std::string& imuGravityDirection) {
  // Gravity direction definition
  if (imuGravityDirection == "up") {
    imuParamsPtr_ = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(g);  // ROS convention
  } else if (imuGravityDirection == "down") {
    imuParamsPtr_ = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedD(g);
  } else {
    throw std::runtime_error("Gravity direction must be either 'up' or 'down'.");
  }
  // Set noise and bias parameters
  imuParamsPtr_->accelerometerCovariance = gtsam::Matrix33::Identity(3, 3) * accNoiseDensity_;
  imuParamsPtr_->biasAccCovariance = gtsam::Matrix33::Identity(3, 3) * accBiasRandomWalk_;
  imuParamsPtr_->gyroscopeCovariance = gtsam::Matrix33::Identity(3, 3) * gyrNoiseDensity_;
  imuParamsPtr_->biasOmegaCovariance = gtsam::Matrix33::Identity(3, 3) * gyrBiasRandomWalk_;
  imuParamsPtr_->integrationCovariance =
      gtsam::Matrix33::Identity(3, 3) * integrationNoiseDensity_;  // error committed in integrating position from velocities
  imuParamsPtr_->biasAccOmegaInt = gtsam::Matrix66::Identity(6, 6) * biasAccOmegaPreint_;  // covariance of bias used for preintegration
  gtsam::Vector3 accBiasPrior(accBiasPrior_, accBiasPrior_, accBiasPrior_);
  gtsam::Vector3 gyrBiasPrior(gyrBiasPrior_, gyrBiasPrior_, gyrBiasPrior_);
  imuBiasPriorPtr_ = std::make_shared<gtsam::imuBias::ConstantBias>(accBiasPrior, gyrBiasPrior);

  // Init preintegrators
  imuBufferPreintegratorPtr_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(imuParamsPtr_, *imuBiasPriorPtr_);
  imuStepPreintegratorPtr_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(imuParamsPtr_, *imuBiasPriorPtr_);
  imuParamsPtr_->print("IMU Preintegration Parameters:");
  return true;
}

gtsam::NavState GraphManager::addImuFactorAndGetState(const double imuTime_k) {
  // Operating on graph data --> acquire mutex during whole method
  const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);

  // Get new key
  gtsam::Key oldKey = stateKey_;
  gtsam::Key newKey = newStateKey();
  // Write current time
  stateTime_ = imuTime_k;

  // Add to key buffer
  imuBuffer_.addToKeyBuffer(imuTime_k, newKey);

  // Get last two measurements from buffer
  IMUMap imuMeas;
  imuBuffer_.getLastTwoMeasurements(imuMeas);

  // Update IMU preintegrator
  updateImuIntegrators_(imuMeas);
  // Predict propagated state
  imuPropagatedState_ = imuStepPreintegratorPtr_->predict(imuPropagatedState_, graphState_.imuBias());
  // Add to IMU pose buffer
  imuBuffer_.addImuPoseToBuffer(imuTime_k, imuPropagatedState_.pose());

  // Create and add IMU Factor
  gtsam::CombinedImuFactor imuFactor(gtsam::symbol_shorthand::X(oldKey), gtsam::symbol_shorthand::V(oldKey),
                                     gtsam::symbol_shorthand::X(newKey), gtsam::symbol_shorthand::V(newKey),
                                     gtsam::symbol_shorthand::B(oldKey), gtsam::symbol_shorthand::B(newKey), *imuStepPreintegratorPtr_);
  newGraphFactors_.add(imuFactor);
  // Add IMU Value
  newGraphValues_.insert(gtsam::symbol_shorthand::X(newKey), imuPropagatedState_.pose());
  newGraphValues_.insert(gtsam::symbol_shorthand::V(newKey), imuPropagatedState_.velocity());
  newGraphValues_.insert(gtsam::symbol_shorthand::B(newKey), graphState_.imuBias());

  // Return copy of propagated state (for publishing)
  return imuPropagatedState_;
}

void GraphManager::addPoseBetweenFactor(const gtsam::Pose3& pose, const double lidarTimeKm1, const double lidarTimeK) {
  // Operating on graph data --> acquire mutex during whole method
  const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);

  // Check
  if (lidarTimeKm1 > lidarTimeK) {
    throw std::runtime_error("Time at time step k-1 must be smaller than time at time step k.");
  }

  // Find closest lidar keys in existing graph
  gtsam::Key closestLidarKeyKm1, closestLidarKeyK;
  double closestGraphTimeKm1, closestGraphTimeK;
  imuBuffer_.getClosestKeyAndTimestamp(lidarTimeKm1, closestGraphTimeKm1, closestLidarKeyKm1);
  imuBuffer_.getClosestKeyAndTimestamp(lidarTimeK, closestGraphTimeK, closestLidarKeyK);
  ROS_INFO_STREAM("Current key: " << stateKey_ << ", found lidar keys are: " << closestLidarKeyKm1 << " and " << closestLidarKeyK);
  ROS_WARN_STREAM("Lidar time at k-1: " << lidarTimeKm1 << ", found LiDAR time: " << closestGraphTimeKm1);
  ROS_WARN_STREAM("Lidar time at k: " << lidarTimeK << ", found LiDAR time: " << closestGraphTimeK);

  // Check search result and potentially warn user
  double maxSearchDeviation = 1 / (2 * imuBuffer_.getImuRate());
  double lowerTimeDeviation = std::abs(lidarTimeKm1 - closestGraphTimeKm1);
  if (lowerTimeDeviation > maxSearchDeviation) {
    ROS_ERROR_STREAM("Time deviation at key " << closestLidarKeyKm1 << " is " << lowerTimeDeviation
                                              << " which is larger than the maximum time deviation of " << maxSearchDeviation);
  }
  double upperTimeDeviation = std::abs(lidarTimeK - closestGraphTimeK);
  if (upperTimeDeviation > maxSearchDeviation) {
    ROS_ERROR_STREAM("Time deviation at key " << closestLidarKeyK << " is " << upperTimeDeviation
                                              << " which is larger than the maximum time deviation of " << maxSearchDeviation);
  }
  double maxLidarTimestampDistance = 1.0 / lidarRate_ + 2.0 * maxSearchDeviation;
  double lidarTimestampDistance = std::abs(closestGraphTimeK - closestGraphTimeKm1);
  if (lidarTimestampDistance > maxLidarTimestampDistance) {
    ROS_ERROR_STREAM("Distance of LiDAR timestamps is too big. Real lidar timestamps are  "
                     << lidarTimeKm1 << " and " << lidarTimeK << " which is larger than the maximum allowed distance of "
                     << maxLidarTimestampDistance);
  }

  // Create Pose BetweenFactor and add
  auto poseNoiseModel = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << poseNoise_[0], poseNoise_[1], poseNoise_[2], poseNoise_[3], poseNoise_[4], poseNoise_[5])
          .finished());  // rad,rad,rad,m,m,m
  gtsam::BetweenFactor<gtsam::Pose3> poseBetweenFactor(gtsam::symbol_shorthand::X(closestLidarKeyKm1),
                                                       gtsam::symbol_shorthand::X(closestLidarKeyK), pose, poseNoiseModel);
  newGraphFactors_.add(poseBetweenFactor);
}

void addPoseUnaryFactor(const gtsam::Key old_key, const gtsam::Key new_key, const gtsam::Pose3& pose) {
  // TODO: TO BE IMPLEMENTED
}

bool GraphManager::addZeroMotionFactor(const gtsam::Key old_key, const gtsam::Key new_key, const gtsam::Pose3 pose) {
  // Check external motion
  if (pose.translation().norm() > zeroMotionTh_) {
    detectionCount_ = 0;
    return false;
  }

  // Check IMU motion
  gtsam::NavState imuPropagatedState = imuStepPreintegratorPtr_->predict(gtsam::NavState(), graphState_.imuBias());
  if (imuPropagatedState.position().norm() > zeroMotionTh_) {
    detectionCount_ = 0;
    return false;
  }

  // Check consective zero-motion detections
  ++detectionCount_;
  if (detectionCount_ < minDetections_) return false;

  // Add Zero Pose Factor
  newGraphFactors_.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::symbol_shorthand::X(old_key), gtsam::symbol_shorthand::X(new_key),
                                                          gtsam::Pose3::identity(), gtsam::noiseModel::Isotropic::Sigma(6, 1e-3)));
  // Add Zero Velocity Factor
  newGraphFactors_.add(gtsam::PriorFactor<gtsam::Vector3>(gtsam::symbol_shorthand::V(old_key), gtsam::Vector3::Zero(),
                                                          gtsam::noiseModel::Isotropic::Sigma(3, 1e-3)));

  // Re-detect Zero Motion
  detectionCount_ = minDetections_ / 2;

  // DEBUG
  std::cout << "Zero Motion Factor added between: " << old_key << "/" << new_key << std::endl;
  // DEBUG

  return true;
}

bool GraphManager::addGravityRollPitchFactor(const gtsam::Key key, const gtsam::Rot3 imu_attitude) {
  static auto imuEstNoise =
      gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-6, 1e-6, 1e+10, 1e+10, 1e+10, 1e+10).finished());  // rad,rad,rad,m, m, m
  newGraphFactors_.add(
      gtsam::PriorFactor<gtsam::Pose3>(gtsam::symbol_shorthand::X(key), gtsam::Pose3(imu_attitude, gtsam::Point3::Zero()), imuEstNoise));
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
    const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
    // Get copy of factors and values
    newGraphFactors = newGraphFactors_;
    newGraphValues = newGraphValues_;
    // Empty graph buffers
    newGraphFactors_.resize(0);
    newGraphValues_.clear();
    // Empty Buffer Preintegrator --> everything missed during the update will be in here
    imuBufferPreintegratorPtr_->resetIntegrationAndSetBias(graphState_.imuBias());
    // Get current key and time
    currentKey = stateKey_;
    currentTime = stateTime_;
  }
  // Graph Update (time consuming) -------------------
  std::map<gtsam::Key, double> keyTimestampMap;
  valuesToKeyTimeStampMap(newGraphValues, currentTime, keyTimestampMap);
  mainGraphPtr_->update(newGraphFactors, newGraphValues, keyTimestampMap);
  // Additional iterations
  for (size_t itr = 0; itr < additonalIterations_; ++itr) {
    mainGraphPtr_->update();
  }
  // Compute result
  auto result = mainGraphPtr_->calculateEstimate();
  // Mutex block 2 ------------------
  gtsam::NavState navState = gtsam::NavState(result.at<gtsam::Pose3>(gtsam::symbol_shorthand::X(currentKey)),
                                             result.at<gtsam::Vector3>(gtsam::symbol_shorthand::V(currentKey)));
  {
    // Lock
    const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
    // Update Graph State
    graphState_.updateNavStateAndBias(currentKey, currentTime, navState,
                                      result.at<gtsam::imuBias::ConstantBias>(gtsam::symbol_shorthand::B(currentKey)));
    // Predict from solution to obtain refined propagated state
    imuPropagatedState_ = imuBufferPreintegratorPtr_->predict(graphState_.navState(), graphState_.imuBias());
  }
  return navState;
}

// Private --------------------------------------------------------------------

void GraphManager::updateImuIntegrators_(const IMUMap& imuMeas) {
  if (imuMeas.size() < 2) {
    std::cout << "_updateImuIntegrators --- Received less than 2 IMU messages --- No Preintegration done" << std::endl;
    return;
  }

  // Reset IMU Step Preintegration
  imuStepPreintegratorPtr_->resetIntegrationAndSetBias(graphState_.imuBias());

  // Start integrating with imu_meas.begin()+1 meas to calculate dt, imu_meas.begin() meas was integrated before
  auto currItr = imuMeas.begin();
  auto prevItr = currItr;

  // Calculate dt and integrate IMU measurements for both preintegrators
  for (++currItr; currItr != imuMeas.end(); ++currItr, ++prevItr) {
    double dt = currItr->first - prevItr->first;
    imuStepPreintegratorPtr_->integrateMeasurement(currItr->second.head<3>(),    // acc
                                                   currItr->second.tail<3>(),    // gyro
                                                   dt);                          // delta t
    imuBufferPreintegratorPtr_->integrateMeasurement(currItr->second.head<3>(),  // acc
                                                     currItr->second.tail<3>(),  // gyro
                                                     dt);
  }
}

}  // namespace fg_filtering