#include "fg_filtering/GraphManager.hpp"

namespace fg_filtering {

// Public --------------------------------------------------------------------

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
  // Use previously defined prior for gyro
  imuBiasPriorPtr_ = std::make_shared<gtsam::imuBias::ConstantBias>(accBiasPrior, gyrBiasPrior_);

  // Init preintegrators
  imuBufferPreintegratorPtr_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(imuParamsPtr_, *imuBiasPriorPtr_);
  imuStepPreintegratorPtr_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(imuParamsPtr_, *imuBiasPriorPtr_);
  imuParamsPtr_->print("IMU Preintegration Parameters:");
  return true;
}

bool GraphManager::initPoseVelocityBiasGraph(const double timeStep, const gtsam::Pose3& initialPose) {
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
  /// Prior factor noise
  auto poseNoise =
      gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-6, 1e-6, 1e-6).finished());  // rad,rad,rad,m, m, m
  auto velocityNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e-3);                                             // m/s
  auto biasNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3).finished());
  /// Factor
  newGraphFactors_.resize(0);
  newGraphFactors_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      gtsam::symbol_shorthand::X(stateKey_), initialPose,
      poseNoise);  // POSE - PriorFactor format is (key,value,matrix) value is same type as type of PriorFactor
  newGraphFactors_.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(gtsam::symbol_shorthand::V(stateKey_), gtsam::Vector3(0, 0, 0),
                                                                      velocityNoise);  // VELOCITY
  newGraphFactors_.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(gtsam::symbol_shorthand::B(stateKey_),
                                                                                    *imuBiasPriorPtr_,
                                                                                    biasNoise);  // BIAS
  // Initial estimate
  gtsam::Values valuesEstimate;
  valuesEstimate.insert(gtsam::symbol_shorthand::X(stateKey_), initialPose);
  valuesEstimate.insert(gtsam::symbol_shorthand::V(stateKey_), gtsam::Vector3(0, 0, 0));
  valuesEstimate.insert(gtsam::symbol_shorthand::B(stateKey_), *imuBiasPriorPtr_);

  // Initialize factor graph
  ROS_INFO_STREAM("Lag of the IncrementalFixedLagSmoother: " << smootherLag_);
  mainGraphPtr_ =
      std::make_shared<gtsam::IncrementalFixedLagSmoother>(smootherLag_, isamParams_);  // std::make_shared<gtsam::ISAM2>(isamParams_);
  mainGraphPtr_->params().print("Factor Graph Parameters:");
  std::cout << "Pose Between Factor Noise - RPY(rad): " << poseBetweenNoise_[0] << "," << poseBetweenNoise_[1] << ","
            << poseBetweenNoise_[2] << ", XYZ(m): " << poseBetweenNoise_[3] << "," << poseBetweenNoise_[4] << "," << poseBetweenNoise_[5]
            << std::endl;

  // Add prior factor to graph and update
  std::map<gtsam::Key, double> priorKeyTimestampMap;
  writeValueKeysToKeyTimeStampMap_(valuesEstimate, timeStep, priorKeyTimestampMap);
  mainGraphPtr_->update(newGraphFactors_, valuesEstimate, priorKeyTimestampMap);

  // Reset
  newGraphFactors_.resize(0);

  // Update Current State
  graphState_.updateNavStateAndBias(stateKey_, timeStep, gtsam::NavState(initialPose, gtsam::Vector3(0, 0, 0)), *imuBiasPriorPtr_);
  imuPropagatedState_ = gtsam::NavState(initialPose, gtsam::Vector3(0, 0, 0));
  return true;
}

gtsam::NavState GraphManager::addImuFactorAndGetState(const double imuTime_k) {
  // Write current time
  stateTime_ = imuTime_k;

  {
    // Operating on graph data --> acquire mutex
    const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);

    // Get new key
    gtsam::Key oldKey = stateKey_;
    gtsam::Key newKey = newStateKey_();

    // Add to key buffer
    imuBuffer_.addToKeyBuffer(imuTime_k, newKey);
    // Get last two measurements from buffer
    IMUMap imuMeas;
    /// Two measurements to determine dt
    imuBuffer_.getLastTwoMeasurements(imuMeas);

    // Update IMU preintegrator
    updateImuIntegrators_(imuMeas);
    // Predict propagated state
    imuPropagatedState_ = imuStepPreintegratorPtr_->predict(imuPropagatedState_, graphState_.imuBias());

    // Add IMU Factor to graph
    gtsam::CombinedImuFactor imuFactor(gtsam::symbol_shorthand::X(oldKey), gtsam::symbol_shorthand::V(oldKey),
                                       gtsam::symbol_shorthand::X(newKey), gtsam::symbol_shorthand::V(newKey),
                                       gtsam::symbol_shorthand::B(oldKey), gtsam::symbol_shorthand::B(newKey), *imuStepPreintegratorPtr_);
    newGraphFactors_.add(imuFactor);

    // Add IMU values
    gtsam::Values valuesEstimate;
    valuesEstimate.insert(gtsam::symbol_shorthand::X(stateKey_), imuPropagatedState_.pose());
    valuesEstimate.insert(gtsam::symbol_shorthand::V(stateKey_), imuPropagatedState_.velocity());
    valuesEstimate.insert(gtsam::symbol_shorthand::B(stateKey_), graphState_.imuBias());
    newGraphValues_.insert(valuesEstimate);

    // Add timestamp for fixed lag smoother
    writeValueKeysToKeyTimeStampMap_(valuesEstimate, imuTime_k, newGraphKeysTimestampsMap_);
  }

  // Add to IMU pose buffer
  imuBuffer_.addImuPoseToBuffer(imuTime_k, imuPropagatedState_.pose());

  // Return copy of propagated state (for publishing)
  return imuPropagatedState_;
}

gtsam::Key GraphManager::addPoseBetweenFactor(const double lidarTimeKm1, const double lidarTimeK, const gtsam::Pose3& pose) {
  // Check
  if (lidarTimeKm1 > lidarTimeK) {
    throw std::runtime_error("Time at time step k-1 must be smaller than time at time step k.");
  }

  // Find corresponding keys in graph
  double maxSearchDeviation = 1 / (2 * imuBuffer_.getImuRate());
  double maxLidarTimestampDistance = 1.0 / lidarRate_ + 2.0 * maxSearchDeviation;
  gtsam::Key closestLidarKeyKm1, closestLidarKeyK;

  if (!findGraphKeys_(maxLidarTimestampDistance, lidarTimeKm1, lidarTimeK, closestLidarKeyKm1, closestLidarKeyK, "lidar")) {
    std::cerr << YELLOW_START << "GRAPH MANAGER" << COLOR_END << " PoseBetween factor not added to graph." << std::endl;
    return closestLidarKeyK;
  }

  // Create noise model
  auto poseBetweenNoise =
      gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << poseBetweenNoise_[0], poseBetweenNoise_[1], poseBetweenNoise_[2],
                                           poseBetweenNoise_[3], poseBetweenNoise_[4], poseBetweenNoise_[5])
                                              .finished());  // rad,rad,rad,m,m,m
  auto tukeyErrorFunction = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Tukey::Create(0.3), poseBetweenNoise);

  // Create pose between factor and add it
  gtsam::BetweenFactor<gtsam::Pose3> poseBetweenFactor(gtsam::symbol_shorthand::X(closestLidarKeyKm1),
                                                       gtsam::symbol_shorthand::X(closestLidarKeyK), pose, tukeyErrorFunction);

  // Write to graph
  {
    // Operating on graph data --> acquire mutex
    const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
    newGraphFactors_.add(poseBetweenFactor);
  }

  // Print summary
  if (verboseLevel_ > 0) {
    std::cout << YELLOW_START << "GraphManager" << COLOR_END << " Current key: " << stateKey_ << "," << GREEN_START
              << " LiDAR PoseBetween factor added between key " << closestLidarKeyKm1 << " and key " << closestLidarKeyK << COLOR_END
              << std::endl;
  }

  return closestLidarKeyK;
}

void GraphManager::addPoseUnaryFactor(const double lidarTimeK, const gtsam::Pose3& unaryPose) {
  // Find closest key in existing graph
  double closestGraphTime;
  gtsam::Key closestKey;
  double maxSearchDeviation = 1 / (2 * lidarRate_);
  if (!imuBuffer_.getClosestKeyAndTimestamp("lidar unary", maxSearchDeviation, lidarTimeK, closestGraphTime, closestKey)) {
    std::cerr << YELLOW_START << "GraphManager" << RED_START << " Not adding lidar unary constraint to graph." << std::endl;
    return;
  }

  auto poseUnaryNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << poseUnaryNoise_[0], poseUnaryNoise_[1], poseUnaryNoise_[2],
                                                             poseUnaryNoise_[3], poseUnaryNoise_[4], poseUnaryNoise_[5])
                                                                .finished());  // rad,rad,rad,x,y,z
  auto tukeyErrorFunction = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(0.3), poseUnaryNoise);

  // Unary factor
  gtsam::PriorFactor<gtsam::Pose3> poseUnaryFactor(gtsam::symbol_shorthand::X(stateKey_), unaryPose, tukeyErrorFunction);

  // Write to graph
  {
    // Operating on graph data --> acquire mutex
    const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
    newGraphFactors_.add(poseUnaryFactor);
  }

  // Print summary
  if (verboseLevel_ > 0) {
    std::cout << YELLOW_START << "GraphManager" << COLOR_END << " Current key " << stateKey_ << GREEN_START
              << " LiDAR unary factor added to key " << closestKey << COLOR_END << std::endl;
  }
}

void GraphManager::addGnssPositionUnaryFactor(double gnssTimeK, const gtsam::Vector3& position) {
  // Find closest key in existing graph
  double closestGraphTime;
  gtsam::Key closestKey;
  double maxSearchDeviation = 1 / (2 * gnssRate_);
  if (!imuBuffer_.getClosestKeyAndTimestamp("Gnss Unary", maxSearchDeviation, gnssTimeK, closestGraphTime, closestKey)) {
    std::cerr << YELLOW_START << "GraphManager" << RED_START << " Not adding gnss unary constraint to graph." << std::endl;
    return;
  }

  // Create noise model
  auto gnssPositionUnaryNoise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(3) << gnssPositionUnaryNoise_, gnssPositionUnaryNoise_, gnssPositionUnaryNoise_).finished());  // rad,rad,rad,m,m,m
  auto tukeyErrorFunction = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(0.5), gnssPositionUnaryNoise);

  // Create unary factor and add it
  gtsam::GPSFactor gnssPositionUnaryFactor(gtsam::symbol_shorthand::X(closestKey), position, tukeyErrorFunction);

  // Write to graph
  {
    // Operating on graph data --> acquire mutex
    const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
    newGraphFactors_.add(gnssPositionUnaryFactor);
  }

  // Print summary
  if (verboseLevel_ > 0) {
    std::cout << YELLOW_START << "GraphManager" << COLOR_END << " Current key " << stateKey_ << GREEN_START << ", GNSS factor added to key "
              << closestKey << COLOR_END << std::endl;
  }
}

void GraphManager::addGnssHeadingUnaryFactor(double gnssTime, const gtsam::Vector3& heading, double measuredYaw) {
  // Print information
  if (verboseLevel_ > 2) {
    std::cout << YELLOW_START << "GraphManager" << COLOR_END << " Current key " << stateKey_ << std::setprecision(14)
              << ", GNSS heading measurement at time stamp " << gnssTime << " is: " << heading.x() << "," << heading.y() << ","
              << heading.z() << std::endl;
  }

  // Find closest key in existing graph
  double closestGraphTime;
  gtsam::Key closestKey;
  double maxSearchDeviation = 1 / (2 * gnssRate_);
  imuBuffer_.getClosestKeyAndTimestamp("gnss heading", maxSearchDeviation, gnssTime, closestGraphTime, closestKey);

  // Create noise model
  //  auto gnssHeadingUnaryNoise =
  //      gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(1) << gnssHeadingUnaryNoise_).finished());  // rad,rad,rad,m,m,m
  auto gnssHeadingUnaryNoise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(3) << gnssHeadingUnaryNoise_, gnssHeadingUnaryNoise_, gnssHeadingUnaryNoise_).finished());  // rad,rad,rad,m,m,m
  auto tukeyErrorFunction = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(0.5), gnssHeadingUnaryNoise);

  // Create unary factor and add it
  //  fg_filtering::HeadingFactorYaw gnssHeadingUnaryFactor(gtsam::symbol_shorthand::X(closestKey), heading, measuredYaw,
  //  tukeyErrorFunction); fg_filtering::HeadingFactorMatrix gnssHeadingUnaryFactor(gtsam::symbol_shorthand::X(closestKey), heading,
  //  measuredYaw,
  //                                                           tukeyErrorFunction);
  fg_filtering::HeadingFactorHeadingVector gnssHeadingUnaryFactor(gtsam::symbol_shorthand::X(closestKey), heading, measuredYaw,
                                                                  tukeyErrorFunction);

  // Write to graph
  {
    // Operating on graph data --> acquire mutex
    const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
    newGraphFactors_.add(gnssHeadingUnaryFactor);
  }

  // Print summary
  if (verboseLevel_ > 0) {
    std::cout << YELLOW_START << "GraphManager" << COLOR_END << " Current key " << stateKey_ << GREEN_START
              << " Key where GNSS factor is added to key " << closestKey << COLOR_END << std::endl;
  }
}

bool GraphManager::addZeroMotionFactor(double maxTimestampDistance, double timeKm1, double timeK, const gtsam::Pose3 pose) {
  // Operating on graph data --> acquire mutex during whole method
  const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);

  // Check external motion
  if (pose.translation().norm() > zeroMotionTh_) {
    std::cout << YELLOW_START << "GraphManager" << COLOR_END << " Current key " << stateKey_
              << ", Not adding zero motion factor due to too big motion." << std::endl;
    return false;
  }

  // Find corresponding keys in graph
  gtsam::Key closestKeyKm1, closestKeyK;
  if (!findGraphKeys_(maxTimestampDistance, timeKm1, timeK, closestKeyKm1, closestKeyK, "zero motion factor")) {
    return false;
  }

  // Add Zero Pose Factor
  newGraphFactors_.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::symbol_shorthand::X(closestKeyKm1),
                                                          gtsam::symbol_shorthand::X(closestKeyK), gtsam::Pose3::identity(),
                                                          gtsam::noiseModel::Isotropic::Sigma(6, 1e-3)));
  // Add Zero Velocity Factor
  newGraphFactors_.add(gtsam::PriorFactor<gtsam::Vector3>(gtsam::symbol_shorthand::V(closestKeyKm1), gtsam::Vector3::Zero(),
                                                          gtsam::noiseModel::Isotropic::Sigma(3, 1e-3)));

  if (verboseLevel_ > 0) {
    std::cout << YELLOW_START << "GraphManager" << COLOR_END << " Current key " << stateKey_
              << GREEN_START " Zero Motion Factor added between keys " << closestKeyKm1 << " and " << closestKeyK << COLOR_END << std::endl;
  }
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
  // Define variables for timing
  std::chrono::time_point<std::chrono::high_resolution_clock> startLoopTime;
  std::chrono::time_point<std::chrono::high_resolution_clock> endLoopTime;
  // Timing
  startLoopTime = std::chrono::high_resolution_clock::now();
  // Method variables
  gtsam::NonlinearFactorGraph newGraphFactors;
  gtsam::Values newGraphValues;
  std::map<gtsam::Key, double> newGraphKeysTimestampsMap;
  gtsam::Key currentKey;
  /// Timing 1
  double currentTime;
  endLoopTime = std::chrono::high_resolution_clock::now();
  if (verboseLevel_ > 2) {
    std::cout << YELLOW_START << "GraphManager" << COLOR_END << " Initialization took "
              << std::chrono::duration_cast<std::chrono::milliseconds>(endLoopTime - startLoopTime).count() << " ms." << std::endl;
  }

  // Mutex Block 1 -----------------
  startLoopTime = std::chrono::high_resolution_clock::now();
  {
    // Lock
    const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
    // Get copy of factors and values
    newGraphFactors = newGraphFactors_;
    newGraphValues = newGraphValues_;
    newGraphKeysTimestampsMap = newGraphKeysTimestampsMap_;
    // Empty graph buffers
    newGraphFactors_.resize(0);
    newGraphValues_.clear();
    newGraphKeysTimestampsMap_.clear();
    // Empty Buffer Preintegrator --> everything missed during the update will be in here
    imuBufferPreintegratorPtr_->resetIntegrationAndSetBias(graphState_.imuBias());
    // Get current key and time
    currentKey = stateKey_;
    currentTime = stateTime_;
  }
  /// Timing 2
  endLoopTime = std::chrono::high_resolution_clock::now();
  if (verboseLevel_ > 2) {
    std::cout << YELLOW_START << "GraphManager" << COLOR_END << " First mutex block took "
              << std::chrono::duration_cast<std::chrono::milliseconds>(endLoopTime - startLoopTime).count() << " ms." << std::endl;
  }

  // Graph Update (time consuming) -------------------
  startLoopTime = std::chrono::high_resolution_clock::now();
  mainGraphPtr_->update(newGraphFactors, newGraphValues, newGraphKeysTimestampsMap);

  // Additional iterations
  for (size_t itr = 0; itr < additonalIterations_; ++itr) {
    mainGraphPtr_->update();
  }
  /// Timing 3
  endLoopTime = std::chrono::high_resolution_clock::now();
  if (verboseLevel_ > 2) {
    std::cout << YELLOW_START << "GraphManager" << COLOR_END << " Optimization of the graph took "
              << std::chrono::duration_cast<std::chrono::milliseconds>(endLoopTime - startLoopTime).count() << " ms." << std::endl;
  }

  // Compute result
  startLoopTime = std::chrono::high_resolution_clock::now();
  // auto result = mainGraphPtr_->calculateEstimate();
  gtsam::NavState resultNavState = calculateStateAtKey(currentKey);
  gtsam::imuBias::ConstantBias resultBias =
      mainGraphPtr_->calculateEstimate<gtsam::imuBias::ConstantBias>(gtsam::symbol_shorthand::B(currentKey));
  /// Timing 4
  endLoopTime = std::chrono::high_resolution_clock::now();
  if (verboseLevel_ > 2) {
    std::cout << YELLOW_START << "GraphManager" << COLOR_END << " Calculation of the estimate took "
              << std::chrono::duration_cast<std::chrono::milliseconds>(endLoopTime - startLoopTime).count() << " ms." << std::endl;
  }

  // Mutex block 2 ------------------
  startLoopTime = std::chrono::high_resolution_clock::now();
  {
    // Lock
    const std::lock_guard<std::mutex> operateOnGraphDataLock(operateOnGraphDataMutex_);
    // Update Graph State
    graphState_.updateNavStateAndBias(currentKey, currentTime, resultNavState, resultBias);
    // result.at<gtsam::imuBias::ConstantBias>(gtsam::symbol_shorthand::B(currentKey)));
    // Predict from solution to obtain refined propagated state
    imuPropagatedState_ = imuBufferPreintegratorPtr_->predict(graphState_.navState(), graphState_.imuBias());
  }
  /// Timing 5
  endLoopTime = std::chrono::high_resolution_clock::now();
  if (verboseLevel_ > 2) {
    std::cout << YELLOW_START << "GraphManager" << COLOR_END << " Second mutex block took "
              << std::chrono::duration_cast<std::chrono::milliseconds>(endLoopTime - startLoopTime).count() << " ms." << std::endl;
  }

  return resultNavState;
}

gtsam::NavState GraphManager::calculateStateAtKey(const gtsam::Key& key) {
  auto resultPose =
      mainGraphPtr_->calculateEstimate<gtsam::Pose3>(gtsam::symbol_shorthand::X(key));  // auto result = mainGraphPtr_->estimate();
  auto resultVelocity = mainGraphPtr_->calculateEstimate<gtsam::Vector3>(gtsam::symbol_shorthand::V(key));
  return gtsam::NavState(resultPose, resultVelocity);
}

// Private --------------------------------------------------------------------

bool GraphManager::findGraphKeys_(double maxTimestampDistance, double timeKm1, double timeK, gtsam::Key& closestKeyKm1,
                                  gtsam::Key& closestKeyK, const std::string& name) {
  // Find closest lidar keys in existing graph
  double closestGraphTimeKm1, closestGraphTimeK;
  double maxSearchDeviation = 1 / (2 * imuBuffer_.getImuRate());
  if (!imuBuffer_.getClosestKeyAndTimestamp(name + " km1", maxSearchDeviation, timeKm1, closestGraphTimeKm1, closestKeyKm1)) {
    return false;
  }
  if (!imuBuffer_.getClosestKeyAndTimestamp(name + " k", maxSearchDeviation, timeK, closestGraphTimeK, closestKeyK)) {
    return false;
  }

  double keyTimestampDistance = std::abs(closestGraphTimeK - closestGraphTimeKm1);
  if (keyTimestampDistance > maxTimestampDistance) {
    std::cerr << YELLOW_START << "GraphManager" << RED_START << " Distance of " << name
              << " timestamps is too big. Found timestamp difference is  " << closestGraphTimeK - closestGraphTimeKm1
              << " which is larger than the maximum admissible distance of " << maxTimestampDistance << std::endl;
    return false;
  }
  return true;
}

void GraphManager::updateImuIntegrators_(const IMUMap& imuMeas) {
  if (imuMeas.size() < 2) {
    std::cerr << YELLOW_START << "GraphManager" << COLOR_END << " Received less than 2 IMU messages --- No Preintegration done."
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

}  // namespace fg_filtering