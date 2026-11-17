/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "b2w_estimator_graph_ros2/B2WEstimator.h"

#include <array>
#include <initializer_list>
#include <vector>
#include <stdexcept>
#include <string>
#include <utility>

// Project
#include "b2w_estimator_graph_ros2/B2WStaticTransforms.h"
#include "b2w_estimator_graph_ros2/constants.h"

// GraphMSF ROS2
#include "graph_msf_ros2/ros/read_ros_params.h"

namespace b2w_se {

void B2WEstimator::readParams() {
  // Check
  if (!graphConfigPtr_) {
    throw std::runtime_error("B2WEstimator: graphConfigPtr must be initialized.");
  }

  // Flags
  useGnssFlag_ = graph_msf::tryGetParam<bool>(this, "sensor_params.useGnss");
  dynamic_cast<B2WStaticTransforms*>(staticTransformsPtr_.get())->setUseGnssFlag(useGnssFlag_);

  useLioOdometryFlag_ = graph_msf::tryGetParam<bool>(this, "sensor_params.useLioOdometry");
  dynamic_cast<B2WStaticTransforms*>(staticTransformsPtr_.get())->setUseLioOdometryFlag(useLioOdometryFlag_);

  useLioBetweenOdometryFlag_ = graph_msf::tryGetParam<bool>(this, "sensor_params.useLioBetweenOdometry");
  dynamic_cast<B2WStaticTransforms*>(staticTransformsPtr_.get())->setUseLioBetweenOdometryFlag(useLioBetweenOdometryFlag_);

  useVioOdometryFlag_ = graph_msf::tryGetParam<bool>(this, "sensor_params.useVioOdometry");
  dynamic_cast<B2WStaticTransforms*>(staticTransformsPtr_.get())->setUseVioOdometryFlag(useVioOdometryFlag_);

  useVioOdometryBetweenFlag_ = graph_msf::tryGetParam<bool>(this, "sensor_params.useVioOdometryBetween");
  lioAvailabilityTimeoutSec_ = graph_msf::tryGetParam<double>(this, "sensor_params.lioAvailabilityTimeoutSec");
  readWheelOdometryParams_();
  dynamic_cast<B2WStaticTransforms*>(staticTransformsPtr_.get())->setUseVioOdometryBetweenFlag(useVioOdometryBetweenFlag_);


    // Print sensor flags in bright green
    RCLCPP_INFO(this->get_logger(), "\033[92m=== B2W Estimator Configuration ===\033[0m");
    RCLCPP_INFO(this->get_logger(), "\033[92m- Use GNSS: %s\033[0m", useGnssFlag_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "\033[92m- Use LIO Odometry: %s\033[0m", useLioOdometryFlag_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "\033[92m- Use LIO Between Odometry: %s\033[0m", useLioBetweenOdometryFlag_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "\033[92m- Use VIO Odometry: %s\033[0m", useVioOdometryFlag_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "\033[92m- Use VIO Odometry Between: %s\033[0m", useVioOdometryBetweenFlag_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "\033[92m===============================\033[0m");

  // Sensor Param as
  lioOdometryRate_ = graph_msf::tryGetParam<int>(this, "sensor_params.lioOdometryRate");
  gnssRate_ = graph_msf::tryGetParam<int>(this, "sensor_params.gnssRate");
  lioBetweenOdometryRate_ = graph_msf::tryGetParam<int>(this, "sensor_params.lioBetweenRate");
  vioOdometryRate_ = graph_msf::tryGetParam<int>(this, "sensor_params.vioOdometryRate");
  vioOdometryBetweenRate_ = graph_msf::tryGetParam<int>(this, "sensor_params.vioOdometryBetweenRate");

  // Alignment Parameters
  const auto initialSe3AlignmentNoiseDensity =
      graph_msf::tryGetParam<std::vector<double>>(this, "alignment_params.initialSe3AlignmentNoiseDensity");
  initialSe3AlignmentNoise_ << initialSe3AlignmentNoiseDensity[0], initialSe3AlignmentNoiseDensity[1], initialSe3AlignmentNoiseDensity[2],
      initialSe3AlignmentNoiseDensity[3], initialSe3AlignmentNoiseDensity[4], initialSe3AlignmentNoiseDensity[5];
  const auto lioSe3AlignmentRandomWalk =
      graph_msf::tryGetParam<std::vector<double>>(this, "alignment_params.lioSe3AlignmentRandomWalk");
  lioSe3AlignmentRandomWalk_ << lioSe3AlignmentRandomWalk[0], lioSe3AlignmentRandomWalk[1], lioSe3AlignmentRandomWalk[2],
      lioSe3AlignmentRandomWalk[3], lioSe3AlignmentRandomWalk[4], lioSe3AlignmentRandomWalk[5];

  const auto vioSe3AlignmentRandomWalk =
      graph_msf::tryGetParam<std::vector<double>>(this, "alignment_params.vioSe3AlignmentRandomWalk");
  vioSe3AlignmentRandomWalk_ << vioSe3AlignmentRandomWalk[0], vioSe3AlignmentRandomWalk[1], vioSe3AlignmentRandomWalk[2],
      vioSe3AlignmentRandomWalk[3], vioSe3AlignmentRandomWalk[4], vioSe3AlignmentRandomWalk[5];
  alignmentRandomWalkScaleWithoutGnss_ = graph_msf::tryGetParam<double>(this, "alignment_params.randomWalkScaleWithoutGnss");
  alignmentRandomWalkGnssAbsenceSec_ = graph_msf::tryGetParam<double>(this, "alignment_params.gnssAbsenceSecondsForScale");
  alignmentRandomWalkReleaseSeconds_ = graph_msf::tryGetParam<double>(this, "alignment_params.randomWalkReleaseSecondsAfterGnssReturn");
  alignmentRandomWalkReleaseMaxFactor_ = graph_msf::tryGetParam<double>(this, "alignment_params.randomWalkReleaseMaxFactor");
  readPseudoGnssParams_();  // after the alignment parameters: overrides the LIO/VIO alignment random walks when enabled
  manualInitEnabled_ = graph_msf::tryGetParam<bool>(this, "manual_init.enabled");
  manualInitTopic_ = graph_msf::tryGetParam<std::string>(this, "manual_init.topic");

  // Noise Parameters
  /// LiDAR Odometry
  const auto lidarUnaryNoise =
      graph_msf::tryGetParam<std::vector<double>>(this, "noise_params.lioPoseUnaryNoiseDensity");  // roll,pitch,yaw,x,y,z
  lioPoseUnaryNoise_ << lidarUnaryNoise[0], lidarUnaryNoise[1], lidarUnaryNoise[2], lidarUnaryNoise[3], lidarUnaryNoise[4], lidarUnaryNoise[5];
  lioVioRobustNormType_ = graph_msf::tryGetParam<std::string>(this, "noise_params.lioVioRobustNorm");
  lioVioRobustNormConstant_ = graph_msf::tryGetParam<double>(this, "noise_params.lioVioRobustNormConstant");
  lioVioRobustNorm();  // validates the type string early
  gnssRobustNormType_ = graph_msf::tryGetParam<std::string>(this, "noise_params.gnssRobustNorm");
  gnssRobustNormConstant_ = graph_msf::tryGetParam<double>(this, "noise_params.gnssRobustNormConstant");
  gnssRobustNorm();

  const auto lidarBetweenNoise =
      graph_msf::tryGetParam<std::vector<double>>(this, "noise_params.lioPoseBetweenNoiseDensity");  // roll,pitch,yaw,x,y,z
  lioBetweenNoise_ << lidarBetweenNoise[0], lidarBetweenNoise[1], lidarBetweenNoise[2], lidarBetweenNoise[3], lidarBetweenNoise[4], lidarBetweenNoise[5];

  /// VIO Odometry Between
  const auto vioPoseBetweenNoise =
      graph_msf::tryGetParam<std::vector<double>>(this, "noise_params.vioPoseBetweenNoiseDensity");  // roll,pitch,yaw,x,y,z
  vioPoseBetweenNoise_ << vioPoseBetweenNoise[0], vioPoseBetweenNoise[1], vioPoseBetweenNoise[2], vioPoseBetweenNoise[3],
      vioPoseBetweenNoise[4], vioPoseBetweenNoise[5];

    /// VIO Odometry Unary
    const auto vioPoseUnaryNoise =
            graph_msf::tryGetParam<std::vector<double>>(this, "noise_params.vioPoseUnaryNoiseDensity");  // roll,pitch,yaw,x,y,z
    vioPoseUnaryNoise_ << vioPoseUnaryNoise[0], vioPoseUnaryNoise[1], vioPoseUnaryNoise[2], vioPoseUnaryNoise[3],
            vioPoseUnaryNoise[4], vioPoseUnaryNoise[5];

  // A zero (or negative) standard deviation becomes a hard constraint in GTSAM (noiseModel::Diagonal::Sigmas -> Constrained).
  for (const auto& [name, sigmas] : std::initializer_list<std::pair<const char*, Eigen::Matrix<double, 6, 1>>>{
           {"lioPoseUnaryNoiseDensity", lioPoseUnaryNoise_},
           {"lioPoseBetweenNoiseDensity", lioBetweenNoise_},
           {"vioPoseBetweenNoiseDensity", vioPoseBetweenNoise_},
           {"vioPoseUnaryNoiseDensity", vioPoseUnaryNoise_}}) {
    if (!(sigmas.array() > 0.0).all() || !sigmas.allFinite()) {
      throw std::runtime_error(std::string("B2WEstimator: noise_params.") + name + " must be finite and > 0 in every entry.");
    }
  }

// GNSS parameters
  // Source re-origin detection (LIO + VIO), source-only kinematic plausibility
  for (SourceEpochState* e : {&lioEpoch_, &vioEpoch_}) {
    e->enabled = graph_msf::tryGetParam<bool>(this, "source_reorigin.enabled");
    e->vMax = graph_msf::tryGetParam<double>(this, "source_reorigin.maxLinearSpeed");
    e->wMax = graph_msf::tryGetParam<double>(this, "source_reorigin.maxAngularSpeed");
    e->posMargin = graph_msf::tryGetParam<double>(this, "source_reorigin.positionMargin");
    e->rotMargin = graph_msf::tryGetParam<double>(this, "source_reorigin.rotationMarginDeg") * M_PI / 180.0;
  }

if (useGnssFlag_) {
    // GNSS Handler
    gnssHandlerPtr_ = std::make_shared<graph_msf::GnssHandler>();

    // Read Yaw initial guess options
    gnssHandlerPtr_->setUseYawInitialGuessFromFile(graph_msf::tryGetParam<bool>(this, "gnss_params.useYawInitialGuessFromFile"));
    gnssHandlerPtr_->setUseYawInitialGuessFromAlignment(graph_msf::tryGetParam<bool>(this, "gnss_params.yawInitialGuessFromAlignment"));

    // Alignment options
    if (gnssHandlerPtr_->getUseYawInitialGuessFromAlignment()) {
    // Make sure no dual true
    gnssHandlerPtr_->setUseYawInitialGuessFromFile(false);
    trajAlignParams_.se3Rate = graph_msf::tryGetParam<double>(this, "trajectoryAlignment.lidarRate");
    trajAlignParams_.r3Rate = graph_msf::tryGetParam<double>(this, "trajectoryAlignment.gnssRate");
    trajAlignParams_.minDistanceHeadingInit = graph_msf::tryGetParam<double>(this, "trajectoryAlignment.minimumDistanceHeadingInit");
    trajAlignParams_.minimumSpatialSpread = graph_msf::tryGetParam<double>(this, "trajectoryAlignment.minimumSpatialSpread");
    trajAlignParams_.noMovementDistance = graph_msf::tryGetParam<double>(this, "trajectoryAlignment.noMovementDistance");
    trajAlignParams_.noMovementTime = graph_msf::tryGetParam<double>(this, "trajectoryAlignment.noMovementTime");
    trajectoryAlignmentHandler_ = makeTrajectoryAligner_();

    // LIO re-alignment after an outage: needs the aligner (this block) and GNSS.
    lioRealignEnabled_ = useLioOdometryFlag_ && graph_msf::tryGetParam<bool>(this, "lio_realignment.enabled");
    lioRealignAbsenceTimeoutSec_ = graph_msf::tryGetParam<double>(this, "lio_realignment.absenceTimeoutSec");
    const int attemptEveryN = graph_msf::tryGetParam<int>(this, "lio_realignment.attemptEveryNGnssMsgs");
    lioRealignAttemptEveryNGnssMsgs_ = attemptEveryN < 1 ? 1 : attemptEveryN;
    lioRealignNoGnssFallbackSec_ = graph_msf::tryGetParam<double>(this, "lio_realignment.noGnssFallbackSec");
    RCLCPP_INFO(this->get_logger(), "\033[92m- LIO re-alignment after outage: %s (absence > %.1f s)\033[0m",
                lioRealignEnabled_ ? "enabled" : "disabled", lioRealignAbsenceTimeoutSec_);
    } else if (!gnssHandlerPtr_->getUseYawInitialGuessFromAlignment() && gnssHandlerPtr_->getUseYawInitialGuessFromFile()) {
    gnssHandlerPtr_->setGlobalYawDegFromFile(graph_msf::tryGetParam<double>(this, "gnss_params.initYaw"));
    }

    // GNSS Reference
    gnssHandlerPtr_->setUseGnssReferenceFlag(graph_msf::tryGetParam<bool>(this, "gnss_params.useGnssReference"));

    if (gnssHandlerPtr_->getUseGnssReferenceFlag()) {
    RCLCPP_INFO(this->get_logger(), "Using GNSS reference from parameters.");
    gnssHandlerPtr_->setGnssReferenceLatitude(graph_msf::tryGetParam<double>(this, "gnss_params.referenceLatitude"));
    gnssHandlerPtr_->setGnssReferenceLongitude(graph_msf::tryGetParam<double>(this, "gnss_params.referenceLongitude"));
    gnssHandlerPtr_->setGnssReferenceAltitude(graph_msf::tryGetParam<double>(this, "gnss_params.referenceAltitude"));
    gnssHandlerPtr_->setGnssReferenceHeading(graph_msf::tryGetParam<double>(this, "gnss_params.referenceHeading"));
    } else {
    RCLCPP_INFO(this->get_logger(), "Will wait for GNSS measurements to initialize reference coordinates.");
    }

    // GNSS Outlier Threshold
    gnssPositionOutlierThreshold_ = graph_msf::tryGetParam<double>(this, "noise_params.gnssPositionOutlierThreshold");
}

  // Set frames

  dynamic_cast<B2WStaticTransforms*>(staticTransformsPtr_.get())
      ->setGnssFrame(graph_msf::tryGetParam<std::string>(this, "extrinsics.gnssFrame"));

  /// LiDAR odometry frame
  dynamic_cast<B2WStaticTransforms*>(staticTransformsPtr_.get())
      ->setLioOdometryFrame(graph_msf::tryGetParam<std::string>(this, "extrinsics.lidarOdometryFrame"));

  /// LiDAR between-odometry frame. It was declared but never read, so with useLioBetweenOdometry the start-up TF lookup got an
  /// empty frame id and the node terminated (tf2::InvalidArgumentException).
  const std::string lidarBetweenFrameParam = graph_msf::tryGetParam<std::string>(this, "extrinsics.lidarBetweenFrame");
  if (useLioBetweenOdometryFlag_ && lidarBetweenFrameParam.empty()) {
    throw std::runtime_error("B2WEstimator: extrinsics.lidarBetweenFrame must be set when sensor_params.useLioBetweenOdometry is true.");
  }
  dynamic_cast<B2WStaticTransforms*>(staticTransformsPtr_.get())->setLidarBetweenFrame(lidarBetweenFrameParam);

  /// VIO Odometry frame
  dynamic_cast<B2WStaticTransforms*>(staticTransformsPtr_.get())
      ->setVioOdometryFrame(graph_msf::tryGetParam<std::string>(this, "extrinsics.vioOdometryFrame"));

 // VIO Odometry frame Between    
  dynamic_cast<B2WStaticTransforms*>(staticTransformsPtr_.get())
  ->setVioOdometryBetweenFrame(graph_msf::tryGetParam<std::string>(this, "extrinsics.vioOdometryBetweenFrame"));

}

void B2WEstimator::readWheelOdometryParams_() {
  useWheelOdometryFlag_ = graph_msf::tryGetParam<bool>(this, "sensor_params.useWheelOdometry");
  wheelOdometryRate_ = graph_msf::tryGetParam<double>(this, "sensor_params.wheelOdometryRate");
  if (!useWheelOdometryFlag_) {
    return;
  }
  auto fail = [](const std::string& what) { throw std::runtime_error("B2WEstimator wheel odometry: " + what); };
  auto vec3 = [&](const std::string& name) {
    const auto v = graph_msf::tryGetParam<std::vector<double>>(this, name);
    if (v.size() != 3) fail(name + " must have 3 entries.");
    return Eigen::Vector3d(v[0], v[1], v[2]);
  };

  // hf_b2w_description
  HfB2wDescription description;
  wheelOdometryBaseFrame_ = graph_msf::tryGetParam<std::string>(this, "hf_b2w_description.baseFrame");
  description.wheelRadius = graph_msf::tryGetParam<double>(this, "hf_b2w_description.wheelRadius");
  description.abductionAxis = vec3("hf_b2w_description.abductionAxis");
  description.pitchAxis = vec3("hf_b2w_description.pitchAxis");
  if (wheelOdometryBaseFrame_.empty()) fail("hf_b2w_description.baseFrame is empty (is hf_b2w_description.yaml loaded?).");
  if (!(description.wheelRadius > 0.0)) fail("hf_b2w_description.wheelRadius must be > 0.");
  const std::array<const char*, 4> legNames{"LF", "LH", "RF", "RH"};
  for (std::size_t l = 0; l < 4; ++l) {
    const std::string prefix = std::string("hf_b2w_description.") + legNames[l];
    const std::vector<std::string> names = this->get_parameter(prefix + ".jointNames").as_string_array();  // tryGetParam cannot print string arrays
    const auto origins = graph_msf::tryGetParam<std::vector<double>>(this, prefix + ".origins");
    if (names.size() != 4) fail(prefix + ".jointNames must list HAA, HFE, KFE, WHEEL.");
    if (origins.size() != 12) fail(prefix + ".origins must have 12 entries (HAA, HFE, KFE, WHEEL origins).");
    auto& leg = description.legs[l];
    leg.name = legNames[l];
    for (std::size_t j = 0; j < 4; ++j) {
      leg.jointNames[j] = names[j];
      leg.origins[j] = Eigen::Vector3d(origins[3 * j], origins[3 * j + 1], origins[3 * j + 2]);
    }
  }

  // Slip handling and noise
  HfB2wWheelOdometryParams params;
  wheelOdometryRobustNormType_ = graph_msf::tryGetParam<std::string>(this, "wheel_odometry.robustNorm");
  wheelOdometryRobustNormConstant_ = graph_msf::tryGetParam<double>(this, "wheel_odometry.robustNormConstant");
  makeRobustNorm_(wheelOdometryRobustNormType_, wheelOdometryRobustNormConstant_, "wheel_odometry.robustNorm");  // validates
  params.wheelConsistencyGate = graph_msf::tryGetParam<double>(this, "wheel_odometry.wheelConsistencyGate");
  params.minConsistentWheels = graph_msf::tryGetParam<int>(this, "wheel_odometry.minConsistentWheels");
  params.maxYawRate = graph_msf::tryGetParam<double>(this, "wheel_odometry.maxYawRate");
  params.maxDeviationFromState = graph_msf::tryGetParam<double>(this, "wheel_odometry.maxDeviationFromState");
  params.sigma = vec3("wheel_odometry.sigma");
  params.sigmaPerSpeed = graph_msf::tryGetParam<double>(this, "wheel_odometry.sigmaPerSpeed");
  params.sigmaPerLegRate = graph_msf::tryGetParam<double>(this, "wheel_odometry.sigmaPerLegRate");
  params.standstillWheelRate = graph_msf::tryGetParam<double>(this, "wheel_odometry.standstillWheelRate");
  params.standstillLegRate = graph_msf::tryGetParam<double>(this, "wheel_odometry.standstillLegRate");
  params.standstillAngularRate = graph_msf::tryGetParam<double>(this, "wheel_odometry.standstillAngularRate");
  params.standstillMinDuration = graph_msf::tryGetParam<double>(this, "wheel_odometry.standstillMinDuration");
  params.standstillMaxStateSpeed = graph_msf::tryGetParam<double>(this, "wheel_odometry.standstillMaxStateSpeed");
  params.standstillSigma = graph_msf::tryGetParam<double>(this, "wheel_odometry.standstillSigma");
  if (!(params.sigma.array() > 0.0).all() || !(params.standstillSigma > 0.0)) fail("wheel_odometry sigmas must be > 0.");
  if (params.minConsistentWheels < 1 || params.minConsistentWheels > 4) fail("wheel_odometry.minConsistentWheels must be 1..4.");
  if (!(wheelOdometryRate_ > 0.0)) fail("sensor_params.wheelOdometryRate must be > 0.");

  wheelOdometryPtr_ = std::make_unique<HfB2wWheelOdometry>(description, params);
  REGULAR_COUT << GREEN_START << " [Wheel odometry] enabled: hf_b2w_description base '" << wheelOdometryBaseFrame_ << "', wheel radius "
               << description.wheelRadius << " m, rate " << wheelOdometryRate_ << " Hz, sigma " << params.sigma.transpose() << COLOR_END
               << std::endl;
}

void B2WEstimator::readPseudoGnssParams_() {
  pseudoGnssEnabled_ = graph_msf::tryGetParam<bool>(this, "pseudo_gnss.enabled");
  if (!pseudoGnssEnabled_) {
    return;
  }
  PseudoGnssAlignmentParams& p = pseudoGnssParams_;
  p.windowSec = graph_msf::tryGetParam<double>(this, "pseudo_gnss.initWindowSec");
  p.minPathLength = graph_msf::tryGetParam<double>(this, "pseudo_gnss.initMinPathLength");
  p.minFixes = graph_msf::tryGetParam<int>(this, "pseudo_gnss.initMinFixes");
  p.inlierThreshold = graph_msf::tryGetParam<double>(this, "pseudo_gnss.initInlierThreshold");
  p.maxYawStdDeg = graph_msf::tryGetParam<double>(this, "pseudo_gnss.initMaxYawStdDeg");
  if (!(p.windowSec > 0.0) || !(p.inlierThreshold > 0.0) || !(p.maxYawStdDeg > 0.0) || p.minFixes < 3) {
    throw std::runtime_error("B2WEstimator pseudo_gnss: initWindowSec, initInlierThreshold and initMaxYawStdDeg must be > 0, initMinFixes >= 3.");
  }
  const auto walk = [this](const std::string& name) {
    const auto v = graph_msf::tryGetParam<std::vector<double>>(this, name);
    if (v.size() != 6) throw std::runtime_error("B2WEstimator: " + name + " must have 6 entries.");
    return Eigen::Matrix<double, 6, 1>(v.data());
  };
  lioSe3AlignmentRandomWalk_ = walk("pseudo_gnss.lioSe3AlignmentRandomWalk");
  vioSe3AlignmentRandomWalk_ = walk("pseudo_gnss.vioSe3AlignmentRandomWalk");
  pseudoGnssAlignment_ = std::make_unique<PseudoGnssAlignment>(p);
  REGULAR_COUT << GREEN_START << " [Pseudo-GNSS] enabled: no reported-sigma gate on GNSS fixes; initialisation from a yaw fit over "
               << p.windowSec << " s, >= " << p.minPathLength << " m path, >= " << p.minFixes << " inlier fixes (inlier < "
               << p.inlierThreshold << " m), yaw std <= " << p.maxYawStdDeg << " deg; LIO/VIO alignment random walk "
               << lioSe3AlignmentRandomWalk_.transpose() << " / " << vioSe3AlignmentRandomWalk_.transpose() << "." << COLOR_END
               << std::endl;
}

}  // namespace b2w_se
