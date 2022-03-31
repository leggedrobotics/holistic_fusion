#include "compslam_se/GraphManager.hpp"

namespace compslam_se {

// Public --------------------------------------------------------------------

GraphManager::GraphManager(GraphConfig* graphConfigPtr) : graphConfigPtr_(graphConfigPtr) {
  imuBuffer_.setImuRate(graphConfigPtr_->imuRate);
  imuBuffer_.setImuBufferLength(graphConfigPtr_->imuBufferLength);
  imuBuffer_.setVerboseLevel(graphConfigPtr_->verboseLevel);
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
  imuParamsPtr_->accelerometerCovariance = gtsam::Matrix33::Identity(3, 3) * graphConfigPtr_->accNoiseDensity;
  imuParamsPtr_->biasAccCovariance = gtsam::Matrix33::Identity(3, 3) * graphConfigPtr_->accBiasRandomWalk;
  imuParamsPtr_->gyroscopeCovariance = gtsam::Matrix33::Identity(3, 3) * graphConfigPtr_->gyroNoiseDensity;
  imuParamsPtr_->biasOmegaCovariance = gtsam::Matrix33::Identity(3, 3) * graphConfigPtr_->gyroBiasRandomWalk;
  imuParamsPtr_->integrationCovariance =
      gtsam::Matrix33::Identity(3, 3) *
      graphConfigPtr_->integrationNoiseDensity;  // error committed in integrating position from velocities
  imuParamsPtr_->biasAccOmegaInt =
      gtsam::Matrix66::Identity(6, 6) * graphConfigPtr_->biasAccOmegaPreint;  // covariance of bias used for preintegration
  gtsam::Vector3 accBiasPrior(graphConfigPtr_->accBiasPrior);
  // Use previously defined prior for gyro
  imuBiasPriorPtr_ = std::make_shared<gtsam::imuBias::ConstantBias>(graphConfigPtr_->accBiasPrior, graphConfigPtr_->gyroBiasPrior);

  // Init preintegrators
  imuBufferPreintegratorPtr_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(imuParamsPtr_, *imuBiasPriorPtr_);
  imuStepPreintegratorPtr_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(imuParamsPtr_, *imuBiasPriorPtr_);
  imuParamsPtr_->print("IMU Preintegration Parameters:");
  return true;
}

bool GraphManager::initPoseVelocityBiasGraph(const double timeStep, const gtsam::Pose3& initialPose) {
  // Set graph relinearization thresholds - must be lower case letters, check:gtsam::symbol_shorthand
  gtsam::FastMap<char, gtsam::Vector> relinTh;
  relinTh['x'] = (gtsam::Vector(6) << graphConfigPtr_->rotationReLinTh, graphConfigPtr_->rotationReLinTh, graphConfigPtr_->rotationReLinTh,
                  graphConfigPtr_->positionReLinTh, graphConfigPtr_->positionReLinTh, graphConfigPtr_->positionReLinTh)
                     .finished();
  relinTh['v'] =
      (gtsam::Vector(3) << graphConfigPtr_->velocityReLinTh, graphConfigPtr_->velocityReLinTh, graphConfigPtr_->velocityReLinTh).finished();
  relinTh['b'] = (gtsam::Vector(6) << graphConfigPtr_->accBiasReLinTh, graphConfigPtr_->accBiasReLinTh, graphConfigPtr_->accBiasReLinTh,
                  graphConfigPtr_->gyroBiasReLinTh, graphConfigPtr_->gyroBiasReLinTh, graphConfigPtr_->gyroBiasReLinTh)
                     .finished();
  isamParams_.relinearizeThreshold = relinTh;
  isamParams_.factorization = gtsam::ISAM2Params::CHOLESKY;  // CHOLESKY:Fast but non-stable //QR:Slower but more stable in
                                                             // poorly conditioned problems

  // Create Prior factor and Initialize factor graph
  /// Prior factor noise
  auto priorPoseNoise =
      gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-6, 1e-6, 1e-6).finished());  // rad,rad,rad,m, m, m
  auto priorVelocityNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e-3);                                        // m/s
  auto priorBiasNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3).finished());

  // Initial estimate
  gtsam::Values valuesEstimate;
  std::cout << YELLOW_START << "GraphManager" << COLOR_END << " Initial Pose: " << initialPose << std::endl;
  valuesEstimate.insert(gtsam::symbol_shorthand::X(stateKey_), initialPose);
  valuesEstimate.insert(gtsam::symbol_shorthand::V(stateKey_), gtsam::Vector3(0, 0, 0));
  valuesEstimate.insert(gtsam::symbol_shorthand::B(stateKey_), *imuBiasPriorPtr_);
  /// Timestamp mapping for incremental fixed lag smoother
  std::map<gtsam::Key, double> priorKeyTimestampMap;
  writeValueKeysToKeyTimeStampMap_(valuesEstimate, timeStep, priorKeyTimestampMap);

  // Initialize both graphs
  /// Initial global factors
  globalFactorsBuffer_.resize(0);
  globalFactorsBuffer_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      gtsam::symbol_shorthand::X(stateKey_), initialPose,
      priorPoseNoise);  // POSE - PriorFactor format is (key,value,matrix) value is same type as type of PriorFactor
  globalFactorsBuffer_.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(gtsam::symbol_shorthand::V(stateKey_), gtsam::Vector3(0, 0, 0),
                                                                          priorVelocityNoise);  // VELOCITY
  globalFactorsBuffer_.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(gtsam::symbol_shorthand::B(stateKey_),
                                                                                        *imuBiasPriorPtr_,
                                                                                        priorBiasNoise);  // BIAS
  /// Initial fallback factors
  fallbackFactorsBuffer_.resize(0);
  fallbackFactorsBuffer_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      gtsam::symbol_shorthand::X(stateKey_), initialPose,
      priorPoseNoise);  // POSE - PriorFactor format is (key,value,matrix) value is same type as type of PriorFactor
  fallbackFactorsBuffer_.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(gtsam::symbol_shorthand::V(stateKey_), gtsam::Vector3(0, 0, 0),
                                                                            priorVelocityNoise);  // VELOCITY
  fallbackFactorsBuffer_.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(gtsam::symbol_shorthand::B(stateKey_),
                                                                                          *imuBiasPriorPtr_,
                                                                                          priorBiasNoise);  // BIAS

  // Initialize global graph
  globalGraphPtr_ = std::make_shared<gtsam::IncrementalFixedLagSmoother>(graphConfigPtr_->smootherLag,
                                                                         isamParams_);  // std::make_shared<gtsam::ISAM2>(isamParams_);
  globalGraphPtr_->params().print("Factor Graph Parameters of global graph.");
  /// Add prior factor to graph and update
  globalGraphPtr_->update(globalFactorsBuffer_, valuesEstimate, priorKeyTimestampMap);
  /// Reset
  globalFactorsBuffer_.resize(0);
  // Initialize fallback graph
  fallbackGraphPtr_ = std::make_shared<gtsam::IncrementalFixedLagSmoother>(graphConfigPtr_->smootherLag,
                                                                           isamParams_);  // std::make_shared<gtsam::ISAM2>(isamParams_);
  globalGraphPtr_->params().print("Factor Graph Parameters of fallback graph.");
  /// Add prior factor to graph and update
  fallbackGraphPtr_->update(fallbackFactorsBuffer_, valuesEstimate, priorKeyTimestampMap);
  /// Reset
  fallbackFactorsBuffer_.resize(0);

  // Set active graph to global graph in the beginning
  activeGraphPtr_ = globalGraphPtr_;

  // Update Current State
  graphState_.updateNavStateAndBias(stateKey_, timeStep, gtsam::NavState(initialPose, gtsam::Vector3(0, 0, 0)), *imuBiasPriorPtr_);
  imuPropagatedState_ = gtsam::NavState(initialPose, gtsam::Vector3(0, 0, 0));
  return true;
}

gtsam::NavState GraphManager::addImuFactorAndGetState(const double imuTimeK, const Eigen::Vector3d& linearAcc,
                                                      const Eigen::Vector3d& angularVel, bool& relocalizationFlag) {
  // Write current time
  stateTime_ = imuTimeK;

  {
    // Operating on graph data --> acquire mutex
    const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);

    // Get new key
    gtsam::Key oldKey = stateKey_;
    gtsam::Key newKey = newStateKey_();

    // Buffers
    /// Add to measurement buffer
    addToIMUBuffer(imuTimeK, linearAcc, angularVel);
    /// Add to key buffer
    imuBuffer_.addToKeyBuffer(imuTimeK, newKey);

    // Get last two measurements from buffer to determine dt
    TimeToImuMap imuMeas;
    imuBuffer_.getLastTwoMeasurements(imuMeas);

    // Update IMU preintegrator
    updateImuIntegrators_(imuMeas);
    // Predict propagated state
    imuPropagatedState_ = imuStepPreintegratorPtr_->predict(imuPropagatedState_, graphState_.imuBias());

    // Add IMU Factor to graph
    gtsam::CombinedImuFactor imuFactor(gtsam::symbol_shorthand::X(oldKey), gtsam::symbol_shorthand::V(oldKey),
                                       gtsam::symbol_shorthand::X(newKey), gtsam::symbol_shorthand::V(newKey),
                                       gtsam::symbol_shorthand::B(oldKey), gtsam::symbol_shorthand::B(newKey), *imuStepPreintegratorPtr_);
    globalFactorsBuffer_.add(imuFactor);
    fallbackFactorsBuffer_.add(imuFactor);

    // Add IMU values
    gtsam::Values valuesEstimate;
    valuesEstimate.insert(gtsam::symbol_shorthand::X(newKey), imuPropagatedState_.pose());
    valuesEstimate.insert(gtsam::symbol_shorthand::V(newKey), imuPropagatedState_.velocity());
    valuesEstimate.insert(gtsam::symbol_shorthand::B(newKey), graphState_.imuBias());
    graphValuesBuffer_.insert(valuesEstimate);

    // Add timestamp for fixed lag smoother
    writeValueKeysToKeyTimeStampMap_(valuesEstimate, imuTimeK, graphKeysTimestampsMapBuffer_);

    // Relocalization Command
    if (!sentRelocalizationCommandAlready_ && numOptimizationsSinceGraphSwitching_ == 1) {
      relocalizationFlag = true;
      sentRelocalizationCommandAlready_ = true;
    } else {
      relocalizationFlag = false;
    }

    // Return copy of propagated state (for publishing)
    return imuPropagatedState_;
  }
}

gtsam::Key GraphManager::addPoseBetweenFactorToGlobalGraph(const double lidarTimeKm1, const double lidarTimeK, const double rate,
                                                           const Eigen::Matrix<double, 6, 1>& poseBetweenNoise, const gtsam::Pose3& pose) {
  // Find corresponding keys in graph
  double maxSearchDeviation = 1 / (2 * imuBuffer_.getImuRate());
  maxSearchDeviation += 0.1 * maxSearchDeviation;
  double maxLidarTimestampDistance = 1.0 / rate + 2.0 * maxSearchDeviation;
  gtsam::Key closestLidarKeyKm1, closestLidarKeyK;

  if (!findGraphKeys_(maxLidarTimestampDistance, lidarTimeKm1, lidarTimeK, closestLidarKeyKm1, closestLidarKeyK, "lidar delta")) {
    std::cerr << YELLOW_START << "FG-GraphManager" << RED_START << " Current key: " << stateKey_
              << " , PoseBetween factor not added to graph at key " << closestLidarKeyK << COLOR_END << std::endl;
    // return closestLidarKeyK;
  }

  // Create noise model
  assert(poseBetweenNoise.size() == 6);
  auto noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << poseBetweenNoise(0), poseBetweenNoise(1), poseBetweenNoise(2),
                                                    poseBetweenNoise(3), poseBetweenNoise(4), poseBetweenNoise(5))
                                                       .finished());  // rad,rad,rad,m,m,m
  auto errorFunction = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.5), noise);

  // Create pose between factor and add it
  gtsam::BetweenFactor<gtsam::Pose3> poseBetweenFactor(gtsam::symbol_shorthand::X(closestLidarKeyKm1),
                                                       gtsam::symbol_shorthand::X(closestLidarKeyK), pose, errorFunction);

  // Write to graph
  {
    // Operating on graph data --> acquire mutex
    const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
    globalFactorsBuffer_.add(poseBetweenFactor);
  }

  // Print summary
  if (graphConfigPtr_->verboseLevel > 1) {
    std::cout << YELLOW_START << "FG-GraphManager" << COLOR_END << " Current key: " << stateKey_ << "," << GREEN_START
              << " LiDAR PoseBetween factor added between key " << closestLidarKeyKm1 << " and key " << closestLidarKeyK << COLOR_END
              << std::endl;
  }

  return closestLidarKeyK;
}

void GraphManager::addPoseUnaryFactorToFallbackGraph(const double lidarTimeK, const double rate,
                                                     const Eigen::Matrix<double, 6, 1>& poseUnaryNoise, const gtsam::Pose3& unaryPose) {
  // Find closest key in existing graph
  double closestGraphTime;
  gtsam::Key closestKey;
  double maxSearchDeviation = 1 / (2 * rate);
  maxSearchDeviation += 0.1 * maxSearchDeviation;
  {
    // Looking up from IMU buffer --> acquire mutex (otherwise values for key might not be set)
    const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
    if (!imuBuffer_.getClosestKeyAndTimestamp("lidar unary", maxSearchDeviation, lidarTimeK, closestGraphTime, closestKey)) {
      std::cerr << YELLOW_START << "FG-GraphManager" << RED_START << " Not adding lidar unary constraint to graph." << std::endl;
      // return;
    }
  }

  assert(poseUnaryNoise.size() == 6);
  auto noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << poseUnaryNoise(0), poseUnaryNoise(1), poseUnaryNoise(2), poseUnaryNoise(3), poseUnaryNoise(4), poseUnaryNoise(5))
          .finished());  // rad,rad,rad,x,y,z
  auto errorFunction = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.5), noise);

  // Unary factor
  gtsam::PriorFactor<gtsam::Pose3> poseUnaryFactor(gtsam::symbol_shorthand::X(closestKey), unaryPose, errorFunction);

  // Write to graph
  {
    // Operating on graph data --> acquire mutex
    const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
    fallbackFactorsBuffer_.add(poseUnaryFactor);
  }

  // Print summary
  if (graphConfigPtr_->verboseLevel > 0) {
    std::cout << YELLOW_START << "FG-GraphManager" << COLOR_END << " Current key " << stateKey_ << GREEN_START
              << " LiDAR unary factor added to key " << closestKey << COLOR_END << std::endl;
  }
}

void GraphManager::addGnssPositionUnaryFactor(double gnssTimeK, const double rate, const double gnssPositionUnaryNoise,
                                              const gtsam::Vector3& position) {
  // Find closest key in existing graph
  double closestGraphTime;
  gtsam::Key closestKey;
  double maxSearchDeviation = 1 / (2 * rate);
  maxSearchDeviation += 0.1 * maxSearchDeviation;
  {
    // Looking up from IMU buffer --> acquire mutex (otherwise values for key might not be set)
    const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
    if (!imuBuffer_.getClosestKeyAndTimestamp("Gnss Unary", maxSearchDeviation, gnssTimeK, closestGraphTime, closestKey)) {
      std::cerr << YELLOW_START << "FG-GraphManager" << RED_START << " Not adding gnss unary constraint to graph." << std::endl;
      // return;
    }
  }

  // Create noise model
  auto noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(3) << gnssPositionUnaryNoise, gnssPositionUnaryNoise, gnssPositionUnaryNoise).finished());  // rad,rad,rad,m,m,m
  auto tukeyErrorFunction = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(0.5), noise);

  // Create unary factor and add it
  gtsam::GPSFactor gnssPositionUnaryFactor(gtsam::symbol_shorthand::X(closestKey), position, tukeyErrorFunction);

  // Write to graph
  {
    // Operating on graph data --> acquire mutex
    const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
    globalFactorsBuffer_.add(gnssPositionUnaryFactor);
    fallbackFactorsBuffer_.add(gnssPositionUnaryFactor);
  }

  // Print summary
  if (graphConfigPtr_->verboseLevel > 1) {
    std::cout << YELLOW_START << "FG-GraphManager" << COLOR_END << " Current key " << stateKey_ << GREEN_START
              << ", GNSS position factor added to key " << closestKey << COLOR_END << std::endl;
  }
}

void GraphManager::addGnssHeadingUnaryFactor(double gnssTimeK, const double rate, const double gnssHeadingUnaryNoise,
                                             const double measuredYaw) {
  // Print information
  if (graphConfigPtr_->verboseLevel > 2) {
    std::cout << YELLOW_START << "FG-GraphManager" << COLOR_END << " Current key " << stateKey_ << std::setprecision(14)
              << ", GNSS yaw measurement at time stamp " << gnssTimeK << " is: " << measuredYaw << std::endl;
  }

  // Find closest key in existing graph
  double closestGraphTime;
  gtsam::Key closestKey;
  double maxSearchDeviation = 1 / (2 * rate);
  maxSearchDeviation += 0.1 * maxSearchDeviation;
  {
    // Looking up from IMU buffer --> acquire mutex (otherwise values for key might not be set)
    const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
    if (!imuBuffer_.getClosestKeyAndTimestamp("Gnss Heading", maxSearchDeviation, gnssTimeK, closestGraphTime, closestKey)) {
      std::cerr << YELLOW_START << "FG-GraphManager" << RED_START << " Not adding gnss heading constraint to graph." << std::endl;
      // return;
    }
  }

  // Create noise model
  //  auto gnssHeadingUnaryNoise =
  //      gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(1) << gnssHeadingUnaryNoise_).finished());  // rad,rad,rad,m,m,m
  auto noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(1) << gnssHeadingUnaryNoise).finished());  // rad,rad,rad,m,m,m
  auto tukeyErrorFunction = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(0.5), noise);

  HeadingFactor gnssHeadingUnaryFactor(gtsam::symbol_shorthand::X(closestKey), measuredYaw, tukeyErrorFunction);

  // Write to graph
  {
    // Operating on graph data --> acquire mutex
    const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
    globalFactorsBuffer_.add(gnssHeadingUnaryFactor);
    fallbackFactorsBuffer_.add(gnssHeadingUnaryFactor);
  }

  // Print summary
  if (graphConfigPtr_->verboseLevel > 0) {
    std::cout << YELLOW_START << "FG-GraphManager" << COLOR_END << " Current key " << stateKey_ << GREEN_START
              << " GNSS heading factor is added to key " << closestKey << COLOR_END << std::endl;
  }
}

bool GraphManager::addZeroMotionFactor(double maxTimestampDistance, double timeKm1, double timeK, const gtsam::Pose3 pose) {
  // Check external motion
  if (pose.translation().norm() > graphConfigPtr_->zeroMotionTh) {
    std::cout << YELLOW_START << "FG-GraphManager" << COLOR_END << " Current key " << stateKey_
              << ", Not adding zero motion factor due to too big motion." << std::endl;
    return false;
  }

  // Find corresponding keys in graph
  gtsam::Key closestKeyKm1, closestKeyK;
  if (!findGraphKeys_(maxTimestampDistance, timeKm1, timeK, closestKeyKm1, closestKeyK, "zero motion factor")) {
    return false;
  }

  // Global
  /// Operating on graph data --> acquire mutex during whole method
  const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
  /// Add Zero Pose Factor
  globalFactorsBuffer_.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::symbol_shorthand::X(closestKeyKm1),
                                                              gtsam::symbol_shorthand::X(closestKeyK), gtsam::Pose3::identity(),
                                                              gtsam::noiseModel::Isotropic::Sigma(6, 1e-3)));
  /// Add Zero Velocity Factor
  globalFactorsBuffer_.add(gtsam::PriorFactor<gtsam::Vector3>(gtsam::symbol_shorthand::V(closestKeyKm1), gtsam::Vector3::Zero(),
                                                              gtsam::noiseModel::Isotropic::Sigma(3, 1e-3)));
  // Fallback
  /// Add Zero Pose Factor
  fallbackFactorsBuffer_.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::symbol_shorthand::X(closestKeyKm1),
                                                                gtsam::symbol_shorthand::X(closestKeyK), gtsam::Pose3::identity(),
                                                                gtsam::noiseModel::Isotropic::Sigma(6, 1e-3)));
  /// Add Zero Velocity Factor
  fallbackFactorsBuffer_.add(gtsam::PriorFactor<gtsam::Vector3>(gtsam::symbol_shorthand::V(closestKeyKm1), gtsam::Vector3::Zero(),
                                                                gtsam::noiseModel::Isotropic::Sigma(3, 1e-3)));

  if (graphConfigPtr_->verboseLevel > 0) {
    std::cout << YELLOW_START << "FG-GraphManager" << COLOR_END << " Current key " << stateKey_
              << GREEN_START " Zero Motion Factor added between keys " << closestKeyKm1 << " and " << closestKeyK << COLOR_END << std::endl;
  }
  return true;
}

bool GraphManager::addGravityRollPitchFactor(const gtsam::Key key, const gtsam::Rot3 imu_attitude) {
  static auto imuEstNoise =
      gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-6, 1e-6, 1e+10, 1e+10, 1e+10, 1e+10).finished());  // rad,rad,rad,m, m, m
  globalFactorsBuffer_.add(
      gtsam::PriorFactor<gtsam::Pose3>(gtsam::symbol_shorthand::X(key), gtsam::Pose3(imu_attitude, gtsam::Point3::Zero()), imuEstNoise));
  fallbackFactorsBuffer_.add(
      gtsam::PriorFactor<gtsam::Pose3>(gtsam::symbol_shorthand::X(key), gtsam::Pose3(imu_attitude, gtsam::Point3::Zero()), imuEstNoise));
  return true;
}

void GraphManager::activateGlobalGraph() {
  // Mutex, such s.t. the used graph is consistent
  const std::lock_guard<std::mutex> consistentActiveGraphLock(consistentActiveGraphMutex_);
  if (activeGraphPtr_ != globalGraphPtr_) {
    *fallbackGraphPtr_ = *globalGraphPtr_;
    std::cout << YELLOW_START << "GraphManager" << GREEN_START << " Reset fallback graph to global graph." << COLOR_END << std::endl;
    activeGraphPtr_ = globalGraphPtr_;
    std::cout << YELLOW_START << "GraphManager" << GREEN_START << " Activated global graph pointer." << COLOR_END << std::endl;
    // Reset counter
    numOptimizationsSinceGraphSwitching_ = 0;
    sentRelocalizationCommandAlready_ = false;
  }
}

void GraphManager::activateFallbackGraph() {
  // Mutex, such s.t. the used graph is consistent
  const std::lock_guard<std::mutex> consistentActiveGraphLock(consistentActiveGraphMutex_);
  if (activeGraphPtr_ != fallbackGraphPtr_) {
    *fallbackGraphPtr_ = *globalGraphPtr_;
    std::cout << YELLOW_START << "GraphManager" << GREEN_START << " Reset fallback graph to global graph." << COLOR_END << std::endl;
    activeGraphPtr_ = fallbackGraphPtr_;
    std::cout << YELLOW_START << "GraphManager" << GREEN_START << " Activated fallback graph pointer." << COLOR_END << std::endl;
    // Reset counter
    numOptimizationsSinceGraphSwitching_ = 0;
  }
}

gtsam::NavState GraphManager::updateGraphAndState(double& currentTime) {
  // Define variables for timing
  std::chrono::time_point<std::chrono::high_resolution_clock> startLoopTime;
  std::chrono::time_point<std::chrono::high_resolution_clock> endLoopTime;
  // Timing
  startLoopTime = std::chrono::high_resolution_clock::now();
  // Method variables
  gtsam::NonlinearFactorGraph newGlobalGraphFactors, newFallbackGraphFactors;
  gtsam::Values newGraphValues;
  std::map<gtsam::Key, double> newGraphKeysTimestampsMap;
  gtsam::Key currentKey;
  /// Timing 1
  endLoopTime = std::chrono::high_resolution_clock::now();
  if (graphConfigPtr_->verboseLevel > 2) {
    std::cout << YELLOW_START << "FG-GraphManager" << COLOR_END << " Initialization took "
              << std::chrono::duration_cast<std::chrono::milliseconds>(endLoopTime - startLoopTime).count() << " ms." << std::endl;
  }

  // Mutex Block 1 -----------------
  startLoopTime = std::chrono::high_resolution_clock::now();
  {
    // Lock
    const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
    // Get copy of factors and values
    newGlobalGraphFactors = globalFactorsBuffer_;
    newFallbackGraphFactors = fallbackFactorsBuffer_;
    newGraphValues = graphValuesBuffer_;
    newGraphKeysTimestampsMap = graphKeysTimestampsMapBuffer_;
    // Empty graph buffers
    globalFactorsBuffer_.resize(0);
    fallbackFactorsBuffer_.resize(0);
    graphValuesBuffer_.clear();
    graphKeysTimestampsMapBuffer_.clear();
    // Empty Buffer Preintegrator --> everything missed during the update will be in here
    imuBufferPreintegratorPtr_->resetIntegrationAndSetBias(graphState_.imuBias());
    // Get current key and time
    currentKey = stateKey_;
    currentTime = stateTime_;
  }
  /// Timing 2
  endLoopTime = std::chrono::high_resolution_clock::now();
  if (graphConfigPtr_->verboseLevel > 2) {
    std::cout << YELLOW_START << "FG-GraphManager" << COLOR_END << " First mutex block took "
              << std::chrono::duration_cast<std::chrono::milliseconds>(endLoopTime - startLoopTime).count() << " ms." << std::endl;
  }

  // Graph Update (time consuming) -------------------
  gtsam::NavState resultNavState;
  gtsam::imuBias::ConstantBias resultBias;
  startLoopTime = std::chrono::high_resolution_clock::now();
  {
    // Mutex, s.t. the used graph is consistent
    const std::lock_guard<std::mutex> consistentActiveGraphLock(consistentActiveGraphMutex_);

    // Perform update
    // If currently using the global graph --> only add factors to global graph
    if (activeGraphPtr_ == globalGraphPtr_) {
      activeGraphPtr_->update(newGlobalGraphFactors, newGraphValues, newGraphKeysTimestampsMap);
    }
    // Otherwise optimize fallback graph and add factors also to global graph
    else if (activeGraphPtr_ == fallbackGraphPtr_) {
      static bool calledException__ = false;
      activeGraphPtr_->update(newFallbackGraphFactors, newGraphValues, newGraphKeysTimestampsMap);
      if (!calledException__) {
        try {
          globalGraphPtr_->update(newGlobalGraphFactors, newGraphValues, newGraphKeysTimestampsMap);
        } catch (const std::exception& e) {
          std::cout << YELLOW_START << "FG-GraphManager" << RED_START
                    << " Exception was thrown while optimizing the global graph. Continuing, as fallback graph is active." << COLOR_END
                    << std::endl;
          calledException__ = true;
        }
      }

    } else {
      std::runtime_error("Active graph pointer is neither pointing to the global-, nor to the fallback graph.");
    }

    // Additional iterations
    for (size_t itr = 0; itr < graphConfigPtr_->additionalIterations; ++itr) {
      activeGraphPtr_->update();
    }
    /// Timing 3
    endLoopTime = std::chrono::high_resolution_clock::now();
    if (graphConfigPtr_->verboseLevel > 2) {
      std::cout << YELLOW_START << "FG-GraphManager" << COLOR_END << " Optimization of the graph took "
                << std::chrono::duration_cast<std::chrono::milliseconds>(endLoopTime - startLoopTime).count() << " ms." << std::endl;
    }

    // Compute result
    startLoopTime = std::chrono::high_resolution_clock::now();
    resultNavState = calculateStateAtKey(currentKey);
    resultBias = activeGraphPtr_->calculateEstimate<gtsam::imuBias::ConstantBias>(gtsam::symbol_shorthand::B(currentKey));

    // Increase counter
    ++numOptimizationsSinceGraphSwitching_;
  }
  /// Timing 4
  endLoopTime = std::chrono::high_resolution_clock::now();
  if (graphConfigPtr_->verboseLevel > 2) {
    std::cout << YELLOW_START << "FG-GraphManager" << COLOR_END << " Calculation of the estimate took "
              << std::chrono::duration_cast<std::chrono::milliseconds>(endLoopTime - startLoopTime).count() << " ms." << std::endl;
  }

  // Mutex block 2 ------------------
  startLoopTime = std::chrono::high_resolution_clock::now();
  {
    // Lock
    const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
    // Update Graph State
    graphState_.updateNavStateAndBias(currentKey, currentTime, resultNavState, resultBias);
    // Predict from solution to obtain refined propagated state
    imuPropagatedState_ = imuBufferPreintegratorPtr_->predict(graphState_.navState(), graphState_.imuBias());
  }
  /// Timing 5
  endLoopTime = std::chrono::high_resolution_clock::now();
  if (graphConfigPtr_->verboseLevel > 2) {
    std::cout << YELLOW_START << "FG-GraphManager" << COLOR_END << " Second mutex block took "
              << std::chrono::duration_cast<std::chrono::milliseconds>(endLoopTime - startLoopTime).count() << " ms." << std::endl;
  }

  return resultNavState;
}

gtsam::NavState GraphManager::calculateStateAtKey(const gtsam::Key& key) {
  auto resultPose =
      activeGraphPtr_->calculateEstimate<gtsam::Pose3>(gtsam::symbol_shorthand::X(key));  // auto result = mainGraphPtr_->estimate();
  auto resultVelocity = activeGraphPtr_->calculateEstimate<gtsam::Vector3>(gtsam::symbol_shorthand::V(key));
  return gtsam::NavState(resultPose, resultVelocity);
}

// Private --------------------------------------------------------------------

bool GraphManager::findGraphKeys_(double maxTimestampDistance, double timeKm1, double timeK, gtsam::Key& closestKeyKm1,
                                  gtsam::Key& closestKeyK, const std::string& name) {
  // Find closest lidar keys in existing graph
  double closestGraphTimeKm1, closestGraphTimeK;
  double maxSearchDeviation = 1 / (2 * imuBuffer_.getImuRate());
  maxSearchDeviation += 0.1 * maxSearchDeviation;
  {
    // Looking up from IMU buffer --> acquire mutex (otherwise values for key might not be set)
    const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
    bool successKm1 = imuBuffer_.getClosestKeyAndTimestamp(name + " km1", maxSearchDeviation, timeKm1, closestGraphTimeKm1, closestKeyKm1);
    bool successK = imuBuffer_.getClosestKeyAndTimestamp(name + " k", maxSearchDeviation, timeK, closestGraphTimeK, closestKeyK);
    if (!successKm1 || !successK) {
      return false;
    }
  }

  // Check
  if (closestGraphTimeKm1 > closestGraphTimeK) {
    ROS_ERROR("Time at time step k-1 must be smaller than time at time step k.");
    return false;
  }

  double keyTimestampDistance = std::abs(closestGraphTimeK - closestGraphTimeKm1);
  if (keyTimestampDistance > maxTimestampDistance) {
    std::cerr << YELLOW_START << "FG-GraphManager" << RED_START << " Distance of " << name
              << " timestamps is too big. Found timestamp difference is  " << closestGraphTimeK - closestGraphTimeKm1
              << " which is larger than the maximum admissible distance of " << maxTimestampDistance << COLOR_END << std::endl;
  }
  return true;
}

void GraphManager::updateImuIntegrators_(const TimeToImuMap& imuMeas) {
  if (imuMeas.size() < 2) {
    std::cerr << YELLOW_START << "FG-GraphManager" << COLOR_END << " Received less than 2 IMU messages --- No Preintegration done."
              << std::endl;
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

}  // namespace compslam_se