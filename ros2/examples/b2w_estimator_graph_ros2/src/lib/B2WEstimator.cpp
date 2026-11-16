/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include <cstdio>
#include <algorithm>
#include <cctype>
#include <stdexcept>
#include "b2w_estimator_graph_ros2/B2WEstimator.h"

// Project
#include "b2w_estimator_graph_ros2/B2WStaticTransforms.h"

// Workspace
#include "graph_msf/measurements/UnaryMeasurementXD.h"
#include "graph_msf/measurements/UnaryMeasurementXDAbsolute.h"
#include "graph_msf/measurements/BinaryMeasurementXD.h"
#include "graph_msf/measurements/UnaryMeasurementXD.h"
#include "graph_msf_ros2/util/conversions.h"
#include "b2w_estimator_graph_ros2/constants.h"

// Timing (minimal additions)
#include <atomic>
#include <chrono>
#include <limits>

// Set to 0 to compile out callback timing.
#define B2W_ENABLE_CALLBACK_TIMING 0

namespace b2w_se {

graph_msf::RobustNorm B2WEstimator::gnssRobustNorm() const { return makeRobustNorm_(gnssRobustNormType_, gnssRobustNormConstant_, "noise_params.gnssRobustNorm"); }

graph_msf::RobustNorm B2WEstimator::lioVioRobustNorm() const { return makeRobustNorm_(lioVioRobustNormType_, lioVioRobustNormConstant_, "noise_params.lioVioRobustNorm"); }

graph_msf::RobustNorm B2WEstimator::makeRobustNorm_(const std::string& type, const double c, const char* paramName) {
  std::string t = type;
  std::transform(t.begin(), t.end(), t.begin(), [](unsigned char ch) { return std::tolower(ch); });
  if (t == "none") return graph_msf::RobustNorm::None();
  if (t == "huber") return graph_msf::RobustNorm::Huber(c);
  if (t == "cauchy") return graph_msf::RobustNorm::Cauchy(c);
  if (t == "tukey") return graph_msf::RobustNorm::Tukey(c);
  if (t == "gemanmcclure" || t == "gm") return graph_msf::RobustNorm::GemanMcClure(c);
  if (t == "dcs") return graph_msf::RobustNorm::DCS(c);
  throw std::runtime_error(std::string(paramName) + " must be one of none|huber|cauchy|tukey|gemanmcclure|dcs, got '" + type + "'");
}

#if B2W_ENABLE_CALLBACK_TIMING
namespace detail {

struct CallbackTimingStats {
  std::atomic<std::uint64_t> n{0};
  std::atomic<std::int64_t> total_ns{0};
  std::atomic<std::int64_t> max_ns{0};
  std::atomic<std::int64_t> last_report_ns{0};
};

inline void atomicMax(std::atomic<std::int64_t>& a, std::int64_t v) {
  std::int64_t cur = a.load(std::memory_order_relaxed);
  while (v > cur && !a.compare_exchange_weak(cur, v, std::memory_order_relaxed)) {}
}

class ScopedCallbackTimer {
public:
  ScopedCallbackTimer(const char* name, const rclcpp::Logger& logger, CallbackTimingStats& stats)
  : name_(name), logger_(logger), stats_(stats), start_(std::chrono::steady_clock::now()) {}

  ~ScopedCallbackTimer() {
    using namespace std::chrono;
    const auto end = steady_clock::now();
    const std::int64_t dur_ns = duration_cast<nanoseconds>(end - start_).count();

    const std::uint64_t n = stats_.n.fetch_add(1, std::memory_order_relaxed) + 1;
    const std::int64_t total_ns = stats_.total_ns.fetch_add(dur_ns, std::memory_order_relaxed) + dur_ns;
    atomicMax(stats_.max_ns, dur_ns);

    constexpr std::int64_t kReportPeriodNs =
        static_cast<std::int64_t>(5) * static_cast<std::int64_t>(1000000000LL);

    const std::int64_t now_ns = duration_cast<nanoseconds>(end.time_since_epoch()).count();
    std::int64_t last = stats_.last_report_ns.load(std::memory_order_relaxed);
    if ((now_ns - last) >= kReportPeriodNs &&
        stats_.last_report_ns.compare_exchange_strong(last, now_ns, std::memory_order_relaxed)) {
      const double avg_ms = (n > 0) ? (static_cast<double>(total_ns) / 1e6) / static_cast<double>(n) : 0.0;
      const double max_ms = static_cast<double>(stats_.max_ns.load(std::memory_order_relaxed)) / 1e6;
      RCLCPP_INFO(logger_, "[CB] %s: n=%llu avg=%.3f ms max=%.3f ms",
                  name_, static_cast<unsigned long long>(n), avg_ms, max_ms);
    }
  }

private:
  const char* name_;
  rclcpp::Logger logger_;
  CallbackTimingStats& stats_;
  std::chrono::steady_clock::time_point start_;
};

}  // namespace detail

#define B2W_SCOPED_CB_TIMER(NAME_LITERAL) \
  static ::b2w_se::detail::CallbackTimingStats __cb_stats; \
  ::b2w_se::detail::ScopedCallbackTimer __cb_timer((NAME_LITERAL), this->get_logger(), __cb_stats)

#else
#define B2W_SCOPED_CB_TIMER(NAME_LITERAL) (void)0
#endif

B2WEstimator::B2WEstimator(const std::string& node_name,
                           const rclcpp::NodeOptions& options)
: graph_msf::GraphMsfRos2(node_name, options)
{
  RCLCPP_INFO(this->get_logger(), "B2WEstimator-Constructor called.");
  // setup();
}

void B2WEstimator::setup(const rclcpp::Node::SharedPtr& self) {
  REGULAR_COUT << GREEN_START << " B2WEstimator-Setup called." << COLOR_END << std::endl;

  // Boolean flags
  this->declare_parameter("sensor_params.useGnss", false);
  this->declare_parameter("sensor_params.useLioOdometry", false);
  this->declare_parameter("sensor_params.useLioBetweenOdometry", false);
  this->declare_parameter("sensor_params.useVioOdometry", false);
  this->declare_parameter("sensor_params.useVioOdometryBetween", false);
  this->declare_parameter("sensor_params.lioAvailabilityTimeoutSec", 3.0);
  this->declare_parameter("sensor_params.useWheelOdometry", false);
  this->declare_parameter("sensor_params.wheelOdometryRate", 20.0);

  // hf_b2w_description (minimal leg kinematics owned by this estimator) and wheel-odometry parameters; values live in
  // config/b2w_specific/hf_b2w_description.yaml and b2w_graph_params_gnss.yaml
  this->declare_parameter("hf_b2w_description.baseFrame", std::string(""));
  this->declare_parameter("hf_b2w_description.wheelRadius", 0.0);
  this->declare_parameter("hf_b2w_description.abductionAxis", std::vector<double>{});
  this->declare_parameter("hf_b2w_description.pitchAxis", std::vector<double>{});
  for (const char* leg : {"LF", "LH", "RF", "RH"}) {
    this->declare_parameter(std::string("hf_b2w_description.") + leg + ".jointNames", std::vector<std::string>{});
    this->declare_parameter(std::string("hf_b2w_description.") + leg + ".origins", std::vector<double>{});
  }
  {
    const HfB2wWheelOdometryParams d;
    this->declare_parameter("wheel_odometry.robustNorm", std::string("huber"));
    this->declare_parameter("wheel_odometry.robustNormConstant", 2.0);
    this->declare_parameter("wheel_odometry.wheelConsistencyGate", d.wheelConsistencyGate);
    this->declare_parameter("wheel_odometry.minConsistentWheels", d.minConsistentWheels);
    this->declare_parameter("wheel_odometry.maxYawRate", d.maxYawRate);
    this->declare_parameter("wheel_odometry.maxDeviationFromState", d.maxDeviationFromState);
    this->declare_parameter("wheel_odometry.sigma", std::vector<double>{d.sigma.x(), d.sigma.y(), d.sigma.z()});
    this->declare_parameter("wheel_odometry.sigmaPerSpeed", d.sigmaPerSpeed);
    this->declare_parameter("wheel_odometry.sigmaPerLegRate", d.sigmaPerLegRate);
    this->declare_parameter("wheel_odometry.standstillWheelRate", d.standstillWheelRate);
    this->declare_parameter("wheel_odometry.standstillLegRate", d.standstillLegRate);
    this->declare_parameter("wheel_odometry.standstillAngularRate", d.standstillAngularRate);
    this->declare_parameter("wheel_odometry.standstillMinDuration", d.standstillMinDuration);
    this->declare_parameter("wheel_odometry.standstillMaxStateSpeed", d.standstillMaxStateSpeed);
    this->declare_parameter("wheel_odometry.standstillSigma", d.standstillSigma);
  }

  // GNSS parameters
  this->declare_parameter("gnss_params.initYaw", 90.0);
  this->declare_parameter("gnss_params.useYawInitialGuessFromFile", false);
  this->declare_parameter("gnss_params.yawInitialGuessFromAlignment", true);
  this->declare_parameter("lio_realignment.enabled", false);
  this->declare_parameter("lio_realignment.absenceTimeoutSec", 10.0);
  this->declare_parameter("lio_realignment.attemptEveryNGnssMsgs", 5);
  this->declare_parameter("lio_realignment.noGnssFallbackSec", 0.0);
  this->declare_parameter("source_reorigin.enabled", false);
  this->declare_parameter("source_reorigin.maxLinearSpeed", 4.0);
  this->declare_parameter("source_reorigin.maxAngularSpeed", 3.0);
  this->declare_parameter("source_reorigin.positionMargin", 0.5);
  this->declare_parameter("source_reorigin.rotationMarginDeg", 11.5);
  this->declare_parameter("gnss_params.useGnssReference", false);
  this->declare_parameter("gnss_params.referenceLatitude", 47.4084860363);
  this->declare_parameter("gnss_params.referenceLongitude", 8.50435818058);
  this->declare_parameter("gnss_params.referenceAltitude", 565.0);
  this->declare_parameter("gnss_params.referenceHeading", 0.0);

  // Trajectory Alignment parameters
  this->declare_parameter("trajectoryAlignment.gnssRate", 10.0); // [Hz], rate of gnss measurements
  this->declare_parameter("trajectoryAlignment.lidarRate", 10.0); // [Hz], rate of lidar odometry
  this->declare_parameter("trajectoryAlignment.minimumDistanceHeadingInit", 3.0); // [m], minimal length of trajectory to get yaw between GNSS and Lidar trajectory
  this->declare_parameter("trajectoryAlignment.minimumSpatialSpread", 0.01); // [m], minimum spatial spread required for trajectory alignment
  this->declare_parameter("trajectoryAlignment.noMovementDistance", 0.1); // [m], if measurements are below this distance and in time range, robot is considered standing
  this->declare_parameter("trajectoryAlignment.noMovementTime", 1.0); // [s], if measurements is time range and below distance, robot is considered standing

  // Sensor parameters (int)
  this->declare_parameter("sensor_params.lioOdometryRate", 0);
  this->declare_parameter("sensor_params.lioBetweenRate", 0);
  this->declare_parameter("sensor_params.gnssRate", 0);

  this->declare_parameter("sensor_params.vioOdometryRate", 0);
  this->declare_parameter("sensor_params.vioOdometryBetweenRate", 0);

  // Alignment parameters (vector of double)
  this->declare_parameter("alignment_params.initialSe3AlignmentNoiseDensity", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  this->declare_parameter("alignment_params.lioSe3AlignmentRandomWalk", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  this->declare_parameter("alignment_params.vioSe3AlignmentRandomWalk", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  this->declare_parameter("alignment_params.randomWalkScaleWithoutGnss", 1.0);
  this->declare_parameter("alignment_params.gnssAbsenceSecondsForScale", 5.0);
  this->declare_parameter("alignment_params.randomWalkReleaseSecondsAfterGnssReturn", 0.0);
  this->declare_parameter("alignment_params.randomWalkReleaseMaxFactor", 20.0);

  // Noise parameters (vectors of double)
  this->declare_parameter("noise_params.lioPoseUnaryNoiseDensity", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  this->declare_parameter("noise_params.lioVioRobustNorm", std::string("dcs"));
  this->declare_parameter("noise_params.lioVioRobustNormConstant", 1.0);
  this->declare_parameter("noise_params.gnssRobustNorm", std::string("huber"));
  this->declare_parameter("noise_params.gnssRobustNormConstant", 0.25);
  this->declare_parameter("noise_params.lioPoseBetweenNoiseDensity", std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter("noise_params.vioPoseBetweenNoiseDensity", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  this->declare_parameter("noise_params.vioPoseUnaryNoiseDensity", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  this->declare_parameter("noise_params.gnssPositionOutlierThreshold", 1.0);

  // Extrinsic frames (string)
  this->declare_parameter("extrinsics.lidarOdometryFrame", std::string(""));
  this->declare_parameter("extrinsics.betweenLidarOdometryFrame", std::string(""));
  this->declare_parameter("extrinsics.gnssFrame", std::string(""));
  this->declare_parameter("extrinsics.lidarBetweenFrame", std::string(""));
  this->declare_parameter("extrinsics.vioOdometryFrame", std::string(""));
  this->declare_parameter("extrinsics.vioOdometryBetweenFrame", std::string(""));

  // Create B2WStaticTransforms
  staticTransformsPtr_ = std::make_shared<B2WStaticTransforms>(this->shared_from_this());

  REGULAR_COUT << RED_START << " PARAMETER READING NOT STARTED" << COLOR_END << std::endl;

  B2WEstimator::readParams();

  // Initialize ROS 2 publishers and subscribers
  B2WEstimator::initializePublishers();
  B2WEstimator::initializeSubscribers();
  B2WEstimator::initializeMessages();
  B2WEstimator::initializeServices();

  GraphMsfRos2::setup(staticTransformsPtr_);

  // Transforms --> query until returns true
  bool foundTransforms = false;
  double accumulatedSleepTime = 0.0;
  while (!foundTransforms) {
    foundTransforms = staticTransformsPtr_->findTransformations();
    // Sleep for 0.1 seconds to avoid busy waiting
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    accumulatedSleepTime += 0.1;

    // Print message every second
    if (accumulatedSleepTime >= 1.0) {
      REGULAR_COUT << "Waiting for transforms... " << static_cast<int>(accumulatedSleepTime) << " seconds elapsed" << COLOR_END << std::endl;
      accumulatedSleepTime = 0.0; // Reset the counter
    }
  }

  GraphMsfRos2::validateHeadingUncertaintyTransformsOrThrow();

  // All extrinsics are looked up once above; nothing reads TF afterwards.
  if (auto* tfStatic = dynamic_cast<graph_msf::StaticTransformsTf*>(staticTransformsPtr_.get())) {
    tfStatic->releaseTfListener();
  }

  // Cache frequently used frame names + constant lever arm once (step 9.1)
  cacheFrames_();

  REGULAR_COUT << GREEN_START << " Set up successfully." << COLOR_END << std::endl;
}

void B2WEstimator::cacheFrames_() {
  auto* st = dynamic_cast<B2WStaticTransforms*>(staticTransformsPtr_.get());
  if (!st) return;

  worldFrame_ = staticTransformsPtr_->getWorldFrame();

  baseLinkFrame_ = st->getBaseLinkFrame();
  gnssFrame_ = st->getGnssFrame();
  lioOdometryFrame_ = st->getLioOdometryFrame();
  lidarBetweenFrame_ = st->getLidarBetweenFrame();
  vioOdometryFrame_ = st->getVioOdometryFrame();

  if (useGnssFlag_) {
    const Eigen::Isometry3d T_B_I =
        staticTransformsPtr_->rv_T_frame1_frame2(baseLinkFrame_, staticTransformsPtr_->getImuFrame());
    const Eigen::Isometry3d T_I_G =
        staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(), gnssFrame_);
    t_B_G_cached_ = (T_B_I * T_I_G).translation();
  }

  framesCached_ = true;
}

void B2WEstimator::addAlignmentPose_(double time, const Eigen::Isometry3d& T_M_B) {
  std::lock_guard<std::mutex> lk(alignmentPoseBufMutex_);
  alignmentPoseBuf_.emplace_back(time, T_M_B);

  const double t_min = time - kAlignmentBufKeepSec;
  while (!alignmentPoseBuf_.empty() && alignmentPoseBuf_.front().first < t_min) {
    alignmentPoseBuf_.pop_front();
  }
}

bool B2WEstimator::getClosestAlignmentPose_(double t, Eigen::Isometry3d& T_M_B_out, double* best_dt_out) const {
  std::lock_guard<std::mutex> lk(alignmentPoseBufMutex_);
  if (alignmentPoseBuf_.empty()) {
    if (best_dt_out) *best_dt_out = std::numeric_limits<double>::infinity();
    return false;
  }

  auto best_it = alignmentPoseBuf_.begin();
  double best_dt = std::abs(best_it->first - t);

  for (auto it = alignmentPoseBuf_.begin(); it != alignmentPoseBuf_.end(); ++it) {
    const double dt = std::abs(it->first - t);
    if (dt < best_dt) { best_dt = dt; best_it = it; }
  }

  if (best_dt_out) *best_dt_out = best_dt;
  if (best_dt > kInitSyncMaxDt) return false;  // too far apart in time
  T_M_B_out = best_it->second;
  return true;
}

void B2WEstimator::publishState(const std::shared_ptr<graph_msf::SafeIntegratedNavState>& integratedNavStatePtr,
                                const std::shared_ptr<graph_msf::SafeNavStateWithCovarianceAndBias>& optimizedStateWithCovarianceAndBiasPtr) {
  GraphMsfRos2::publishState(integratedNavStatePtr, optimizedStateWithCovarianceAndBiasPtr);
  if (integratedNavStatePtr && this->areYawAndPositionInited()) {
    std::lock_guard<std::mutex> lk(stateBufferMutex_);
    stateBuffer_.push_back({integratedNavStatePtr->getTimeK(), integratedNavStatePtr->getT_W_Ik(), integratedNavStatePtr->getI_v_W_I(),
                            integratedNavStatePtr->getI_w_W_I()});
    while (stateBuffer_.size() > 2500) stateBuffer_.pop_front();
  }
}

bool B2WEstimator::lookupBufferedState_(const double t, const double maxDt, BufferedState& out) {
  std::lock_guard<std::mutex> lk(stateBufferMutex_);
  double bestDt = std::numeric_limits<double>::infinity();
  for (auto it = stateBuffer_.rbegin(); it != stateBuffer_.rend(); ++it) {
    const double dt = std::abs(it->t - t);
    if (dt < bestDt) { bestDt = dt; out = *it; }
    if (it->t < t - maxDt) break;
  }
  return bestDt <= maxDt;
}

bool B2WEstimator::lookupWorldImuPose_(const double t, const double maxDt, Eigen::Isometry3d& T_W_I) {
  BufferedState s;
  if (!lookupBufferedState_(t, maxDt, s)) return false;
  T_W_I = s.T_W_I;
  return true;
}

Eigen::Matrix<double, 6, 1> B2WEstimator::currentAlignmentRandomWalk_(const Eigen::Matrix<double, 6, 1>& randomWalk, const double t) {
  if (alignmentRandomWalkScaleWithoutGnss_ >= 1.0 || !useGnssFlag_) return randomWalk;
  const double lastGnss = lastValidGnssHeaderTime_.load(std::memory_order_relaxed);
  const bool absent = lastGnss > 0.0 && (t - lastGnss) > alignmentRandomWalkGnssAbsenceSec_;
  if (absent != alignmentRandomWalkScaledActive_.load(std::memory_order_relaxed)) {
    alignmentRandomWalkScaledActive_.store(absent, std::memory_order_relaxed);
    if (alignmentRandomWalkReleaseSeconds_ > 0.0) {
      std::lock_guard<std::mutex> lk(walkReleaseMutex_);
      if (absent) {
        walkAbsenceStart_ = lastGnss;
      } else if (walkAbsenceStart_ > 0.0) {
        const double period = std::max(graphConfigPtr_->createReferenceAlignmentKeyframeEveryNSeconds_, 1e-3);
        const double missed = std::max(1.0, (lastGnss - walkAbsenceStart_) / period);
        walkReleaseFactor_ = std::min(std::sqrt(missed), alignmentRandomWalkReleaseMaxFactor_);
        walkReleaseUntil_ = lastGnss + alignmentRandomWalkReleaseSeconds_;
        REGULAR_COUT << YELLOW_START << " GNSS back after " << std::fixed << std::setprecision(1) << (lastGnss - walkAbsenceStart_)
                     << " s: alignment random walk released x" << std::setprecision(2) << walkReleaseFactor_ << " for "
                     << alignmentRandomWalkReleaseSeconds_ << " s so the alignments can re-converge." << COLOR_END << "\n";
      }
    }
    REGULAR_COUT << YELLOW_START << (absent ? " GNSS absent for more than " : " GNSS back: alignment random walk restored (was scaled by ")
                 << (absent ? alignmentRandomWalkGnssAbsenceSec_ : alignmentRandomWalkScaleWithoutGnss_)
                 << (absent ? " s: alignment random walk scaled by " : ")") << (absent ? std::to_string(alignmentRandomWalkScaleWithoutGnss_) : "")
                 << COLOR_END << "\n";
  }
  if (absent) {
    return (randomWalk * alignmentRandomWalkScaleWithoutGnss_).eval();
  }
  if (alignmentRandomWalkReleaseSeconds_ > 0.0) {
    std::lock_guard<std::mutex> lk(walkReleaseMutex_);
    if (t < walkReleaseUntil_) return (randomWalk * walkReleaseFactor_).eval();
  }
  return randomWalk;
}

std::string B2WEstimator::epochFrameName_(SourceEpochState& s, const std::string& baseFrame, const double t, const Eigen::Isometry3d& T_M_S,
                                          const char* name) {
  if (s.enabled && s.havePrev && t > s.prevT) {
    const double dt = t - s.prevT;
    const Eigen::Isometry3d dT = s.prevPose.inverse() * T_M_S;
    const double dp = dT.translation().norm();
    const double dr = std::abs(Eigen::AngleAxisd(dT.rotation()).angle());
    if (dp > s.vMax * dt + s.posMargin || dr > s.wMax * dt + s.rotMargin) {
      ++s.epoch;
      REGULAR_COUT << YELLOW_START << " [" << name << " re-origin] pose step of " << std::fixed << std::setprecision(2) << dp << " m / "
                   << std::setprecision(1) << (dr * 180.0 / M_PI) << " deg in " << std::setprecision(2) << dt << " s at t = " << std::setprecision(1) << t
                   << " is not physical: new fixed-frame epoch " << baseFrame << "_e" << s.epoch << " (fresh alignment)." << COLOR_END << "\n";
    }
  }
  s.havePrev = true;
  s.prevT = t;
  s.prevPose = T_M_S;
  return s.epoch == 0 ? baseFrame : baseFrame + "_e" + std::to_string(s.epoch);
}

std::shared_ptr<graph_msf::TrajectoryAlignmentHandler> B2WEstimator::makeTrajectoryAligner_() const {
  auto aligner = std::make_shared<graph_msf::TrajectoryAlignmentHandler>();
  aligner->setSe3Rate(trajAlignParams_.se3Rate);
  aligner->setR3Rate(trajAlignParams_.r3Rate);
  aligner->setMinDistanceHeadingInit(trajAlignParams_.minDistanceHeadingInit);
  aligner->setMinimumSpatialSpread(trajAlignParams_.minimumSpatialSpread);
  aligner->setNoMovementDistance(trajAlignParams_.noMovementDistance);
  aligner->setNoMovementTime(trajAlignParams_.noMovementTime);
  return aligner;
}

bool B2WEstimator::lioAvailable_(const double t) const {
  if (!useLioOdometryFlag_) {
    return false;
  }
  if (lioAvailabilityTimeoutSec_ <= 0.0) {
    return true;  // legacy: trust the configuration flag
  }
  const double lastLio = lastAcceptedLioHeaderTime_.load(std::memory_order_relaxed);
  if (lastLio > 0.0) {
    return (t - lastLio) <= lioAvailabilityTimeoutSec_;  // stays false while LIO is degenerate (its callback returns early)
  }
  const double firstVio = firstVioHeaderTime_.load(std::memory_order_relaxed);
  return firstVio <= 0.0 || (t - firstVio) <= lioAvailabilityTimeoutSec_;  // no LIO yet: give it the timeout from the first VIO pose
}

std::shared_ptr<graph_msf::TrajectoryAlignmentHandler> B2WEstimator::aligner_() {
  std::lock_guard<std::mutex> lk(realignMutex_);
  return trajectoryAlignmentHandler_;
}

void B2WEstimator::switchInitAlignmentSource_(const InitAlignmentSource source, const double t, const std::string& reason) {
  const InitAlignmentSource previous = initAlignmentSource_.exchange(source);
  if (previous == source) {
    return;
  }
  if (previous != InitAlignmentSource::None) {
    // The aligner and the pose buffer hold the other source's map frame.
    {
      std::lock_guard<std::mutex> lk(realignMutex_);
      trajectoryAlignmentHandler_ = makeTrajectoryAligner_();
    }
    std::lock_guard<std::mutex> lk(alignmentPoseBufMutex_);
    alignmentPoseBuf_.clear();
  }
  REGULAR_COUT << YELLOW_START << " [World alignment] " << (source == InitAlignmentSource::Vio ? "VIO" : "LIO")
               << " map is now the source of the world alignment (t = " << std::fixed << std::setprecision(1) << t << "): " << reason
               << "." << COLOR_END << "\n";
}

// Runs on the GNSS callback thread while LIO is being re-aligned: feeds the GNSS path and
// periodically tries the alignment. A converged result is parked for the LIO thread to inject.
void B2WEstimator::realignFromGnss_(const Eigen::Vector3d& W_t_W_Gnss, const double timeK) {
  std::shared_ptr<graph_msf::TrajectoryAlignmentHandler> aligner;
  int n = 0;
  {
    std::lock_guard<std::mutex> lk(realignMutex_);
    if (realignInjectPending_) {
      return;  // a result is already waiting for the LIO thread
    }
    aligner = trajectoryAlignmentHandler_;
    n = ++realignGnssMsgCounter_;
  }
  if (!aligner) {
    return;
  }
  aligner->addR3Position(W_t_W_Gnss, timeK);
  if (n % lioRealignAttemptEveryNGnssMsgs_ != 0) {
    return;
  }
  double yaw_W_M = 0.0;
  Eigen::Isometry3d T_W_M = Eigen::Isometry3d::Identity();
  if (!aligner->alignTrajectories(yaw_W_M, T_W_M)) {
    return;  // needs ~minimumDistanceHeadingInit of motion on both paths
  }
  {
    std::lock_guard<std::mutex> lk(realignMutex_);
    realignT_W_M_ = T_W_M;
    realignInjectPending_ = true;
  }
  REGULAR_COUT << GREEN_START << " [LIO realign] Trajectory alignment converged: yaw(W<-M) = " << std::fixed
               << std::setprecision(1) << (180.0 * yaw_W_M / M_PI) << " deg, t = " << T_W_M.translation().transpose()
               << ". Handing to the LIO thread for injection." << COLOR_END << "\n";
}

void B2WEstimator::initializePublishers() {
  // Non-time-critical publishers with best-effort and larger queue
  auto best_effort_qos = rclcpp::QoS(100)
                             .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                             .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE)
                             .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

  pubMeasMapLioLidarPath_ = this->create_publisher<nav_msgs::msg::Path>("/graph_msf/measLiDAR_path_map_lidar", best_effort_qos);           // RViz: green
  pubMeasMapVioPath_ = this->create_publisher<nav_msgs::msg::Path>("/graph_msf/measVIO_path_map_zed_base_link", best_effort_qos);          // RViz: orange
  pubMeasMapVioBetweenPath_ = this->create_publisher<nav_msgs::msg::Path>("/graph_msf/measVIO_Between_path_map_zed_base_link", best_effort_qos);  // RViz: dark red
  pubMeasMapLioPath_ = this->create_publisher<nav_msgs::msg::Path>("/graph_msf/measLiDAR_between_path_world", best_effort_qos);

  if (useGnssFlag_) {
    pubGnssPoseWithCov = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/graph_msf/gnss_pose_with_covariance", best_effort_qos);
    pubMeasGNSSPath_ = this->create_publisher<nav_msgs::msg::Path>("/graph_msf/measGNSS_path", best_effort_qos);  // RViz: yellow
    REGULAR_COUT << COLOR_END << " Initialized GNSS Path publisher\n";

    auto qos_reliable_latched = rclcpp::QoS(10)
      .transient_local()
      .reliable()
      .keep_last(1);

    pubStatus_ = this->create_publisher<std_msgs::msg::Bool>("/graph_msf/alignment_status", qos_reliable_latched);
    alignmentStatus_.store(false, std::memory_order_relaxed);
    alignmentStatusTimer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() {
      publishAlignmentStatus_();
    });
    publishAlignmentStatus_();

    pubReferenceNavSatFixCoordinates_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
      "/graph_msf/reference_gnss_position", qos_reliable_latched);

    pubReferenceNavSatFixCoordinatesENU_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
      "/graph_msf/reference_gnss_position_enu", qos_reliable_latched);
  }
}

void B2WEstimator::publishAlignmentStatus_() {
  if (!pubStatus_) {
    return;
  }

  std_msgs::msg::Bool statusMsg;
  statusMsg.data = alignmentStatus_.load(std::memory_order_relaxed);
  pubStatus_->publish(statusMsg);
}

void B2WEstimator::initializeSubscribers() {
  // Per-stream groups so different sensors run concurrently on a MultiThreadedExecutor.
  auto cb_gnss  = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto cb_lidar = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto cb_vio   = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions so_gnss;  so_gnss.callback_group  = cb_gnss;
  rclcpp::SubscriptionOptions so_lidar; so_lidar.callback_group = cb_lidar;
  rclcpp::SubscriptionOptions so_vio;   so_vio.callback_group   = cb_vio;

  // Low-latency QoS: drop instead of backlog.
  const auto qosReliable = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
  const auto qos1 = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

  if (useGnssFlag_) {
    subGnssNavSatFix_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gnss_topic", qosReliable,
        std::bind(&B2WEstimator::gnssNavSatFixCallback_, this, std::placeholders::_1),
        so_gnss);
    REGULAR_COUT << COLOR_END << " Initialized GNSS NavSatFix subscriber with topic: /gnss_topic\n";
  }

  if (useLioOdometryFlag_) {
    subLioOdometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/lidar_odometry_topic", qosReliable,
        std::bind(&B2WEstimator::lidarOdometryCallback_, this, std::placeholders::_1),
        so_lidar);
    REGULAR_COUT << COLOR_END << " Initialized LiDAR Odometry subscriber with topic: /lidar_odometry_topic\n";
  }

  if (useLioOdometryFlag_ || useLioBetweenOdometryFlag_) {
    subLioDegeneracy_ = this->create_subscription<std_msgs::msg::Bool>(
        "/lio_degeneracy_topic", qosReliable,
        std::bind(&B2WEstimator::lidarDegeneracyCallback_, this, std::placeholders::_1),
        so_lidar);
    REGULAR_COUT << COLOR_END << " Initialized LiDAR degeneracy subscriber with topic: /lio_degeneracy_topic\n";
  }

  if (useLioBetweenOdometryFlag_) {
    subLioBetweenOdometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/between_lidar_odometry_topic", qos1,
        std::bind(&B2WEstimator::lidarBetweenOdometryCallback_, this, std::placeholders::_1),
        so_lidar);
    REGULAR_COUT << COLOR_END << " Initialized LiDAR Between Odometry subscriber with topic: /between_lidar_odometry_topic\n";
  }

  if (useVioOdometryFlag_) {
    subVioOdometry_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/vio_odometry_topic", qosReliable,
        std::bind(&B2WEstimator::vioOdometryCallback_, this, std::placeholders::_1),
        so_vio);
    REGULAR_COUT << COLOR_END << " Initialized VIO Odometry subscriber with topic: /vio_odometry_topic\n";
  }

  if (useWheelOdometryFlag_) {
    auto cb_joints = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions so_joints;
    so_joints.callback_group = cb_joints;
    // Sensor-data QoS (best effort) matches both reliable and best-effort joint-state publishers
    subJointStates_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states_topic", rclcpp::SensorDataQoS().keep_last(10),
        std::bind(&B2WEstimator::jointStatesCallback_, this, std::placeholders::_1), so_joints);
    REGULAR_COUT << COLOR_END << " Initialized joint-state subscriber (wheel odometry) with topic: " << subJointStates_->get_topic_name()
                 << "\n";
  }

  if (useVioOdometryBetweenFlag_) {
    subVioOdometryBetween_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/vio_odometry_between_topic", qos1,
        std::bind(&B2WEstimator::vioOdometryBetweenCallback_, this, std::placeholders::_1),
        so_vio);
    REGULAR_COUT << COLOR_END << " Initialized VIO Odometry between subscriber with topic: /vio_odometry_between_topic\n";
  }
}

void B2WEstimator::lidarDegeneracyCallback_(
    const std_msgs::msg::Bool::ConstSharedPtr& degeneracyPtr) {
  lioDegenerate_.store(degeneracyPtr->data, std::memory_order_relaxed);
}

void B2WEstimator::initializeMessages() {
  measLio_mapLidarPathPtr_ = std::make_shared<nav_msgs::msg::Path>();
  measVio_mapCameraPathPtr_ = std::make_shared<nav_msgs::msg::Path>();
  measVioBetween_mapCameraPathPtr_ = std::make_shared<nav_msgs::msg::Path>();
  if (useGnssFlag_) {
    measGnssPathPtr_ = std::make_shared<nav_msgs::msg::Path>();
  }
}

void B2WEstimator::initializeServices() {
  // Nothing for now
}

bool B2WEstimator::isPlausibleSourceStep_(const SourceEpochState& s, const Eigen::Isometry3d& T_km1_k, const double dt) {
  const double stepTranslation = T_km1_k.translation().norm();
  const double stepRotation = Eigen::AngleAxisd(T_km1_k.rotation()).angle();
  return stepTranslation <= s.vMax * dt + s.posMargin && stepRotation <= s.wMax * dt + s.rotMargin;
}

void B2WEstimator::lidarBetweenOdometryCallback_(const nav_msgs::msg::Odometry::ConstSharedPtr& lidarBetweenOdomPtr) {
  B2W_SCOPED_CB_TIMER("lidarBetweenOdometryCallback_");

  // Degenerate scans add no factor, and the next good scan restarts the delta chain instead of spanning the degenerate stretch.
  if (lioDegenerate_.load(std::memory_order_relaxed)) {
    lidarBetweenChainValid_ = false;
    return;
  }

  if (!areRollAndPitchInited()) {
    return;
  }

  // Timestamp [s] (avoid rclcpp::Time construction)
  const double lidarBetweenTimeK =
      lidarBetweenOdomPtr->header.stamp.sec + lidarBetweenOdomPtr->header.stamp.nanosec * 1e-9;

  // Rate gate: between factors at most at lioBetweenRate
  if (lidarBetweenChainValid_ && lioBetweenOdometryRate_ > 0.0) {
    const double dt = lidarBetweenTimeK - lidarBetweenTimeKm1_;
    if (dt > 0.0 && dt < (1.0 / lioBetweenOdometryRate_)) {
      return;
    }
  }

  ++lidarBetweenCallbackCounter_;

  Eigen::Isometry3d lio_T_M_Lk = Eigen::Isometry3d::Identity();
  graph_msf::odomMsgToEigen(*lidarBetweenOdomPtr, lio_T_M_Lk.matrix());

  // GNSS trajectory alignment: only fed from here when the absolute LIO unary is off (the unary path already feeds it, with the
  // lever arm; feeding both gave the aligner two inconsistent trajectories).
  if (!useLioOdometryFlag_ && useGnssFlag_ && gnssHandlerPtr_->getUseYawInitialGuessFromAlignment()) {
    addAlignmentPose_(lidarBetweenTimeK, lio_T_M_Lk);
    const std::string& baseFrame =
        lidarBetweenOdomPtr->child_frame_id.empty() ? baseLinkFrame_ : lidarBetweenOdomPtr->child_frame_id;
    const Eigen::Vector3d t_B_G = (baseFrame == baseLinkFrame_)
                                      ? t_B_G_cached_
                                      : staticTransformsPtr_->rv_T_frame1_frame2(baseFrame, gnssFrame_).translation();
    if (const auto aligner = aligner_()) {
      aligner->addSe3Position(lio_T_M_Lk.translation() + lio_T_M_Lk.rotation() * t_B_G, lidarBetweenTimeK);
    }
  }

  // Frame names (cached)
  const std::string& lioBetweenOdomFrameName = lidarBetweenFrame_;
  const std::string& lioOdomFrameName = lioOdometryFrame_;

  if (lidarBetweenCallbackCounter_ <= 2 || !lidarBetweenChainValid_) {
    // (Re)start of the delta chain: remember this pose, no factor yet.
  } else if (!areYawAndPositionInited()) {
    if (!useGnssFlag_ && !useLioOdometryFlag_) {
      graph_msf::UnaryMeasurementXD<Eigen::Isometry3d, 6> unary6DMeasurement(
          "Lidar_unary_6D", int(lioBetweenOdometryRate_), lioBetweenOdomFrameName, lioBetweenOdomFrameName + sensorFrameCorrectedNameId,
          graph_msf::RobustNorm::None(), lidarBetweenTimeK, 1.0, lio_T_M_Lk, lioPoseUnaryNoise_);
      REGULAR_COUT << GREEN_START << " LiDAR odometry callback is setting global yaw, as it was not set so far." << COLOR_END << "\n";
      this->initYawAndPosition(unary6DMeasurement);
    }
  } else {
    const double dt = lidarBetweenTimeK - lidarBetweenTimeKm1_;
    const Eigen::Isometry3d T_Lkm1_Lk = lio_T_M_Lkm1_.inverse() * lio_T_M_Lk;
    if (dt <= 0.0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[LIO Between] non-increasing timestamps (dt=%.4f s). Skipping.", dt);
    } else if (!isPlausibleSourceStep_(lioEpoch_, T_Lkm1_Lk, dt)) {
      // A step the robot cannot make (e.g. a DLIO re-origin) must not become a between factor.
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "[LIO Between] non-physical step of %.2f m in %.3f s (source re-origin?). Skipping this delta.",
                           T_Lkm1_Lk.translation().norm(), dt);
    } else {
      graph_msf::BinaryMeasurementXD<Eigen::Isometry3d, 6> delta6DMeasurement(
          "Lidar_between_6D", int(lioBetweenOdometryRate_), lioOdomFrameName, lioOdomFrameName + sensorFrameCorrectedNameId,
          lioVioRobustNorm(), lidarBetweenTimeKm1_, lidarBetweenTimeK, T_Lkm1_Lk, lioBetweenNoise_);
      this->addBinaryPose3Measurement(delta6DMeasurement);
    }
  }

  lio_T_M_Lkm1_ = lio_T_M_Lk;
  lidarBetweenTimeKm1_ = lidarBetweenTimeK;
  lidarBetweenChainValid_ = true;

  // Visualization: do work only if needed (step 9.3)
  if (pubMeasMapLioPath_ && pubMeasMapLioPath_->get_subscription_count() > 0) {
    addToPathMsg(measLio_mapLidarPathPtr_, worldFrame_, lidarBetweenOdomPtr->header.stamp, lio_T_M_Lk.translation(),
                 static_cast<int>(graphConfigPtr_->imuBufferLength_ / 2.0));
    pubMeasMapLioPath_->publish(*measLio_mapLidarPathPtr_);
  }
}

void B2WEstimator::gnssNavSatFixCallback_(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& navSatFixPtr) {
  B2W_SCOPED_CB_TIMER("gnssNavSatFixCallback_");

  // 0) Fast validity checks
  const auto status = navSatFixPtr->status.status;
  if (status != sensor_msgs::msg::NavSatStatus::STATUS_FIX &&
      status != sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX &&
      status != sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                         "GNSS message has invalid fix status (%d). Expected FIX/SBAS_FIX/GBAS_FIX. Skipping.",
                         status);
    return;
  }

  if (navSatFixPtr->position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                         "GNSS message has unknown covariance type. Skipping.");
    return;
  }
  if (navSatFixPtr->position_covariance[0] < 0.0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                         "GNSS message has invalid covariance. Skipping.");
    return;
  }
  if (!std::isfinite(navSatFixPtr->latitude) || !std::isfinite(navSatFixPtr->longitude) || !std::isfinite(navSatFixPtr->altitude)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                         "GNSS message contains non-finite values. Skipping.");
    return;
  }

  ++gnssCallbackCounter_;

  // Timestamp [s] (avoid rclcpp::Time construction)
  const double timeK =
      navSatFixPtr->header.stamp.sec + navSatFixPtr->header.stamp.nanosec * 1e-9;

  static FrequencyChecker gnss_freq_checker(40, 5.0);
  if (gnss_freq_checker.tick(timeK)) {
    REGULAR_COUT << GOLD_START
                 << "[GNSS] callback frequency (avg last "
                 << gnss_freq_checker.last_window_n() << "): "
                 << std::fixed << std::setprecision(2)
                 << gnss_freq_checker.last_hz() << " Hz"
                 << COLOR_END << "\n";
  }

  Eigen::Vector3d gnssCoord(navSatFixPtr->latitude, navSatFixPtr->longitude, navSatFixPtr->altitude);

  Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>> Penu_map(navSatFixPtr->position_covariance.data());
  Eigen::Matrix3d P_enu = 0.5 * (Penu_map + Penu_map.transpose());

  Eigen::Matrix3d P_lv03 = graph_msf::gnss_cov::rotateCov_ENU_to_LV03(
      *gnssHandlerPtr_, navSatFixPtr->latitude, navSatFixPtr->longitude, navSatFixPtr->altitude, P_enu);

  Eigen::Vector3d estStdDevXYZ(std::sqrt(P_lv03(0,0)),
                               std::sqrt(P_lv03(1,1)),
                               std::sqrt(P_lv03(2,2)));

  if (gnssCallbackCounter_ < NUM_GNSS_CALLBACKS_UNTIL_START) {
    accumulatedGnssCoordinates_ += gnssCoord;
    if ((gnssCallbackCounter_ % 10) == 0) {
      REGULAR_COUT << " NOT ENOUGH GNSS MESSAGES ARRIVED!\n";
    }
    return;
  } else if (gnssCallbackCounter_ == NUM_GNSS_CALLBACKS_UNTIL_START) {

    Eigen::Vector3d avgGnssCoord = accumulatedGnssCoordinates_ / NUM_GNSS_CALLBACKS_UNTIL_START;
    gnssHandlerPtr_->initHandler(avgGnssCoord);

    auto referenceGNSSmsg = std::make_shared<sensor_msgs::msg::NavSatFix>();
    referenceGNSSmsg->header = navSatFixPtr->header;
    referenceGNSSmsg->latitude  = gnssHandlerPtr_->getGnssReferenceLatitude();
    referenceGNSSmsg->longitude = gnssHandlerPtr_->getGnssReferenceLongitude();
    referenceGNSSmsg->altitude  = gnssHandlerPtr_->getGnssReferenceAltitude();
    pubReferenceNavSatFixCoordinates_->publish(*referenceGNSSmsg);

    REGULAR_COUT << "\033[1;36m"
                 << "==================== GNSS REFERENCE ====================\n"
                 << " LATITUDE : " << referenceGNSSmsg->latitude << "\n"
                 << " LONGITUDE: " << referenceGNSSmsg->longitude << "\n"
                 << " ALTITUDE : " << referenceGNSSmsg->altitude << "\n"
                 << "=======================================================\n"
                 << "\033[0m";

    auto referenceGNSSmsgENU = std::make_shared<sensor_msgs::msg::NavSatFix>();
    Eigen::Vector3d originAsENU = Eigen::Vector3d::Zero();
    gnssHandlerPtr_->convertNavSatToPositionLV03(gnssCoord, originAsENU);

    referenceGNSSmsgENU->header   = navSatFixPtr->header;
    referenceGNSSmsgENU->latitude = originAsENU(0);
    referenceGNSSmsgENU->longitude= originAsENU(1);
    referenceGNSSmsgENU->altitude = originAsENU(2);
    pubReferenceNavSatFixCoordinatesENU_->publish(*referenceGNSSmsgENU);

    REGULAR_COUT << "\033[1;36m"
                 << "==================== GNSS REFERENCE ENU ====================\n"
                 << " X : " << referenceGNSSmsgENU->latitude << "\n"
                 << " Y : " << referenceGNSSmsgENU->longitude << "\n"
                 << " Z : " << referenceGNSSmsgENU->altitude << "\n"
                 << "===========================================================\n"
                 << "\033[0m";

    REGULAR_COUT << " GNSS Handler initialized." << COLOR_END << "\n";
    alignmentStatus_.store(false, std::memory_order_relaxed);
    publishAlignmentStatus_();
  }

  Eigen::Vector3d W_t_W_Gnss = Eigen::Vector3d::Zero();
  gnssHandlerPtr_->convertNavSatToPositionLV03(gnssCoord, W_t_W_Gnss);

  std::string fixedFrame = worldFrame_;

  if (!areYawAndPositionInited()) {

    // Frame names
    const std::string& baseFrame = baseLinkFrame_;
    const std::string& gnssFrame = gnssFrame_;

    // Static lever arm
    const Eigen::Vector3d t_B_G = t_B_G_cached_;

    double initYaw_W_Base = 0.0;

    bool have_R_W_B_full = false;
    Eigen::Matrix3d R_W_B_full = Eigen::Matrix3d::Identity();
    bool trajectoryAlignmentSucceeded = false;

    if (gnssHandlerPtr_->getUseYawInitialGuessFromFile()) {
      initYaw_W_Base = gnssHandlerPtr_->getGlobalYawDegFromFile() / 180.0 * M_PI;
    } else if (gnssHandlerPtr_->getUseYawInitialGuessFromAlignment()) {

      if (gnssCallbackCounter_ % 20 == 0) {
        REGULAR_COUT << YELLOW_START << " Adding GNSS measurement to trajectory alignment." << "\n";
      }

      const auto aligner = aligner_();
      aligner->addR3Position(W_t_W_Gnss, timeK);

      double yaw_W_M = 0.0;
      Eigen::Isometry3d T_W_M = Eigen::Isometry3d::Identity();
      if (!aligner->alignTrajectories(yaw_W_M, T_W_M)) {
        if (gnssCallbackCounter_ % 10 == 0) {
          REGULAR_COUT << YELLOW_START
                       << "Trajectory alignment not ready. Waiting for more motion."
                       << COLOR_END << "\n";
        }
        return;
      }

      // Use the helper (step 9.2)
      Eigen::Isometry3d T_M_B_t0 = Eigen::Isometry3d::Identity();
      double best_dt = 0.0;
      if (!getClosestAlignmentPose_(timeK, T_M_B_t0, &best_dt)) {
        if (!std::isfinite(best_dt)) {
          REGULAR_COUT << YELLOW_START
                       << "Trajectory alignment ready, but alignment pose buffer is empty (cannot compute yaw(W<-B))."
                       << COLOR_END << "\n";
        } else {
          REGULAR_COUT << YELLOW_START
                       << "Trajectory alignment ready, but no sufficiently time-synced alignment pose for yaw(W<-B). "
                       << "best_dt=" << std::fixed << std::setprecision(3) << best_dt << " s"
                       << COLOR_END << "\n";
        }
        return;
      }

      R_W_B_full = T_W_M.rotation() * T_M_B_t0.rotation();
      have_R_W_B_full = true;

      const Eigen::Vector3d x_W = R_W_B_full.col(0);
      initYaw_W_Base = std::atan2(x_W.y(), x_W.x());

      trajectoryAlignmentSucceeded = true;
      REGULAR_COUT << GREEN_START
                   << "Trajectory Alignment Successful. "
                   << "yaw(W<-M) [deg]=" << (180.0 * yaw_W_M / M_PI)
                   << "  yaw(W<-B) [deg]=" << (180.0 * initYaw_W_Base / M_PI)
                   << COLOR_END << "\n";
    }

    Eigen::Matrix3d R_W_B_forLever = Eigen::AngleAxisd(initYaw_W_Base, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    if (have_R_W_B_full) {
      R_W_B_forLever = R_W_B_full;
    }
    const Eigen::Vector3d W_t_W_Base = W_t_W_Gnss - R_W_B_forLever * t_B_G;

    const bool yawPositionInitSucceeded = this->initYawAndPositionInWorld(initYaw_W_Base, W_t_W_Base,
                                                                          /*frame1=*/baseFrame,
                                                                          /*frame2=*/baseFrame);
    if (yawPositionInitSucceeded) {
      if (trajectoryAlignmentSucceeded) {
        alignmentStatus_.store(true, std::memory_order_relaxed);
        publishAlignmentStatus_();
      }
      REGULAR_COUT << GREEN_START << " GNSS initialization of yaw and position successful." << COLOR_END << "\n";
    } else {
      alignmentStatus_.store(false, std::memory_order_relaxed);
      publishAlignmentStatus_();
      REGULAR_COUT << RED_START << " GNSS initialization of yaw and position failed." << COLOR_END << "\n";
    }

  } else {

    lastValidGnssHeaderTime_.store(timeK, std::memory_order_relaxed);
    if (lioRealignEnabled_ && lioAlignState_.load(std::memory_order_acquire) == LioAlignState::Realigning) {
      realignFromGnss_(W_t_W_Gnss, timeK);
    }

    const std::string& gnssFrameName = gnssFrame_;
    const double timestampSec =
        navSatFixPtr->header.stamp.sec + navSatFixPtr->header.stamp.nanosec * 1e-9;

    // graph_msf::UnaryMeasurementXDAbsolute<Eigen::Vector3d, 3> meas_W_t_W_Gnss(
    //     "GnssPosition", int(gnssRate_), gnssFrameName, gnssFrameName + sensorFrameCorrectedNameId,
    //     graph_msf::RobustNorm::None(),
    //     timestampSec, gnssPositionOutlierThreshold_,
    //     W_t_W_Gnss, estStdDevXYZ,
    //     fixedFrame, worldFrame_);

    graph_msf::UnaryMeasurementXDAbsolute<Eigen::Vector3d, 3> meas_W_t_W_Gnss(
        "GnssPosition", int(gnssRate_), gnssFrameName, gnssFrameName + sensorFrameCorrectedNameId,
        gnssRobustNorm(),
        timestampSec, gnssPositionOutlierThreshold_,
        W_t_W_Gnss, estStdDevXYZ,
        fixedFrame, worldFrame_);

    this->addUnaryPosition3AbsoluteMeasurement(meas_W_t_W_Gnss);
  }

  if (fixedFrame != worldFrame_) {
    fixedFrame += referenceFrameAlignedNameId;
  }

  // Publish only if needed (step 9.3)
  if (pubGnssPoseWithCov->get_subscription_count() > 0) {
    geometry_msgs::msg::PoseWithCovarianceStamped pwc;
    pwc.header.frame_id = fixedFrame;
    pwc.header.stamp = navSatFixPtr->header.stamp;
    pwc.pose.pose.position.x = W_t_W_Gnss.x();
    pwc.pose.pose.position.y = W_t_W_Gnss.y();
    pwc.pose.pose.position.z = W_t_W_Gnss.z();
    pwc.pose.pose.orientation.w = 1.0;
    pwc.pose.pose.orientation.x = 0.0;
    pwc.pose.pose.orientation.y = 0.0;
    pwc.pose.pose.orientation.z = 0.0;

    for (double &v : pwc.pose.covariance) v = 0.0;
    pwc.pose.covariance[0]  = P_lv03(0,0); pwc.pose.covariance[1]  = P_lv03(0,1); pwc.pose.covariance[2]  = P_lv03(0,2);
    pwc.pose.covariance[6]  = P_lv03(1,0); pwc.pose.covariance[7]  = P_lv03(1,1); pwc.pose.covariance[8]  = P_lv03(1,2);
    pwc.pose.covariance[12] = P_lv03(2,0); pwc.pose.covariance[13] = P_lv03(2,1); pwc.pose.covariance[14] = P_lv03(2,2);

    const double big_var = 1e6;
    pwc.pose.covariance[21] = big_var;
    pwc.pose.covariance[28] = big_var;
    pwc.pose.covariance[35] = big_var;

    pubGnssPoseWithCov->publish(pwc);
  }

  // GNSS path: do work only if needed (step 9.3)
  if (pubMeasGNSSPath_->get_subscription_count() > 0) {
    addToPathMsg(measGnssPathPtr_, fixedFrame, navSatFixPtr->header.stamp, W_t_W_Gnss, static_cast<int>(graphConfigPtr_->imuBufferLength_ / 2.0));
    pubMeasGNSSPath_->publish(*measGnssPathPtr_);
  }
}

void B2WEstimator::lidarOdometryCallback_(const nav_msgs::msg::Odometry::ConstSharedPtr& odomLidarPtr) {
  B2W_SCOPED_CB_TIMER("lidarOdometryCallback_");

  if (lioDegenerate_.load(std::memory_order_relaxed)) {
    return;
  }

  const double prevAcceptedLioHeaderTime = lastAcceptedLioHeaderTime_.load(std::memory_order_relaxed);
  static std::uint64_t lidarRawCallbackCounter_ = 0;
  static std::uint64_t lidarAcceptedCallbackCounter_ = 0;
  static std::uint64_t lidarRateGateRejectedCounter_ = 0;
  static FrequencyChecker lio_raw_wall_freq_checker(40, 5.0);
  static FrequencyChecker lio_accepted_wall_freq_checker(40, 5.0);
  static FrequencyChecker lio_accepted_stamp_freq_checker(40, 5.0);

  const double lidarOdometryTimeK =
      odomLidarPtr->header.stamp.sec + odomLidarPtr->header.stamp.nanosec * 1e-9;
  const double lidarCallbackWallTimeK = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();

  ++lidarRawCallbackCounter_;
  const bool lioReportNow = lio_raw_wall_freq_checker.tick(lidarCallbackWallTimeK);

  if (prevAcceptedLioHeaderTime > 0.0 &&
      (lidarOdometryTimeK - prevAcceptedLioHeaderTime) < (1.0 / lioOdometryRate_)) {
    ++lidarRateGateRejectedCounter_;
    if (lioReportNow) {
      REGULAR_COUT << BLUE_START
                   << "[LIO] raw arrival rate (wall, avg last "
                   << lio_raw_wall_freq_checker.last_window_n() << "): "
                   << std::fixed << std::setprecision(2)
                   << lio_raw_wall_freq_checker.last_hz() << " Hz"
                   << " | accepted arrival rate (wall): "
                   << lio_accepted_wall_freq_checker.last_hz() << " Hz"
                   << " | accepted rate (header stamp): "
                   << lio_accepted_stamp_freq_checker.last_hz() << " Hz"
                   << " | accepted/raw: " << lidarAcceptedCallbackCounter_
                   << "/" << lidarRawCallbackCounter_
                   << " | rate-gated: " << lidarRateGateRejectedCounter_
                   << COLOR_END << "\n";
    }
    return;
  }
  lastAcceptedLioHeaderTime_.store(lidarOdometryTimeK, std::memory_order_relaxed);
  ++lidarAcceptedCallbackCounter_;
  lio_accepted_wall_freq_checker.tick(lidarCallbackWallTimeK);
  lio_accepted_stamp_freq_checker.tick(lidarOdometryTimeK);

  ++lidarUnaryCallbackCounter_;
  if (lioReportNow) {
    REGULAR_COUT << BLUE_START
                 << "[LIO] raw arrival rate (wall, avg last "
                 << lio_raw_wall_freq_checker.last_window_n() << "): "
                 << std::fixed << std::setprecision(2)
                 << lio_raw_wall_freq_checker.last_hz() << " Hz"
                 << " | accepted arrival rate (wall): "
                 << lio_accepted_wall_freq_checker.last_hz() << " Hz"
                 << " | accepted rate (header stamp): "
                 << lio_accepted_stamp_freq_checker.last_hz() << " Hz"
                 << " | accepted/raw: " << lidarAcceptedCallbackCounter_
                 << "/" << lidarRawCallbackCounter_
                 << " | rate-gated: " << lidarRateGateRejectedCounter_
                 << COLOR_END << "\n";
  }

  Eigen::Isometry3d lio_T_M_Lk = Eigen::Isometry3d::Identity();
  graph_msf::odomMsgToEigen(*odomLidarPtr, lio_T_M_Lk.matrix());

  if (!areYawAndPositionInited() && initAlignmentSource_.load() != InitAlignmentSource::Lio) {
    switchInitAlignmentSource_(InitAlignmentSource::Lio, lidarOdometryTimeK, "LiDAR odometry is arriving");
  }
  addAlignmentPose_(lidarOdometryTimeK, lio_T_M_Lk);

  // Frame Name
  const std::string& lioOdometryFrame = lioOdometryFrame_;
  const std::string lioFixedFrame = epochFrameName_(lioEpoch_, odomLidarPtr->header.frame_id, lidarOdometryTimeK, lio_T_M_Lk, "LIO");

  // The heading-uncertainty disk is defined for the active world-to-fixed-frame alignment, where the fixed
  // frame is the drifting LiDAR odometry/map frame carried in header.frame_id, not the sensor/body child frame.
  setHeadingUncertaintyFixedFrame("lio", lioFixedFrame);

  if (useLioOdometryFlag_ && useGnssFlag_ && gnssHandlerPtr_->getUseYawInitialGuessFromAlignment()) {

    const std::string& baseFrame =
        odomLidarPtr->child_frame_id.empty() ? baseLinkFrame_ : odomLidarPtr->child_frame_id;

    // Prefer cached lever arm if the message base frame matches cached base link.
    const Eigen::Vector3d t_B_G =
        (baseFrame == baseLinkFrame_) ? t_B_G_cached_
                                      : staticTransformsPtr_->rv_T_frame1_frame2(baseFrame, gnssFrame_).translation();

    const Eigen::Vector3d p_G_in_M =
        lio_T_M_Lk.translation() + lio_T_M_Lk.rotation() * t_B_G;

    if (const auto aligner = aligner_()) {
      aligner->addSe3Position(p_G_in_M, lidarOdometryTimeK);
    }
  }

  if (lioOdometryFrame != odomLidarPtr->child_frame_id) {
    REGULAR_COUT << RED_START << "====================================================================\n"
                 << "ERROR: LIDAR ODOMETRY FRAME MISMATCH!\n"
                 << "Expected frame: " << lioOdometryFrame << "\n"
                 << "Odometry message child_frame_id: " << odomLidarPtr->child_frame_id << "\n"
                 << "====================================================================\n"
                 << COLOR_END << "\n";
  }
  if (lioFixedFrame.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "LiDAR odometry header.frame_id is empty. Absolute pose unary will be ill-defined.");
  } else if (lioFixedFrame == lioOdometryFrame) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "LiDAR absolute pose uses the sensor/body frame as fixed frame (%s). "
                         "Expected a map/odom-like reference frame in header.frame_id.",
                         lioFixedFrame.c_str());
  }

  graph_msf::UnaryMeasurementXDAbsolute<Eigen::Isometry3d, 6> unary6DMeasurement(
      "Lidar_unary_6D", int(lioOdometryRate_),
      lioOdometryFrame, lioOdometryFrame + sensorFrameCorrectedNameId,
      lioVioRobustNorm(),
      // graph_msf::RobustNorm::None(),
      lidarOdometryTimeK, 1.0,
      lio_T_M_Lk, lioPoseUnaryNoise_,
      lioFixedFrame, worldFrame_,
      initialSe3AlignmentNoise_, currentAlignmentRandomWalk_(lioSe3AlignmentRandomWalk_, lidarOdometryTimeK),
      graph_msf::AbsoluteUnaryAlignmentRecoveryPolicy::RestartFromPrior);

  if (lidarUnaryCallbackCounter_ <= 2) {
    return;
  } else if (!areYawAndPositionInited()) {
    if (!useGnssFlag_) {
      REGULAR_COUT << GREEN_START << " LiDAR odometry callback is setting global yaw, as it was not set so far."
                   << COLOR_END << "\n";
      this->initYawAndPosition(unary6DMeasurement);
    }
  } else {
    // LIO re-alignment after an outage (see B2WEstimator.h).
    if (lioRealignEnabled_) {
      const double lioGapSec = lidarOdometryTimeK - prevAcceptedLioHeaderTime;
      if (lioAlignState_.load(std::memory_order_acquire) == LioAlignState::Active && prevAcceptedLioHeaderTime > 0.0 &&
          lioGapSec > lioRealignAbsenceTimeoutSec_) {
        {
          std::lock_guard<std::mutex> lk(realignMutex_);
          trajectoryAlignmentHandler_ = makeTrajectoryAligner_();  // drop the old-frame trajectory
          realignInjectPending_ = false;
          realignGnssMsgCounter_ = 0;
          realignStartHeaderTime_ = lidarOdometryTimeK;
        }
        lioAlignState_.store(LioAlignState::Realigning, std::memory_order_release);
        measLio_mapLidarPathPtr_->poses.clear();
        measLio_mapLidarPathPtr_->header.frame_id = lioFixedFrame + referenceFrameAlignedNameId;
        measLio_mapLidarPathPtr_->header.stamp = odomLidarPtr->header.stamp;
        if (pubMeasMapLioLidarPath_->get_subscription_count() > 0) {
          pubMeasMapLioLidarPath_->publish(*measLio_mapLidarPathPtr_);
        }
        REGULAR_COUT << YELLOW_START << " [LIO realign] LIO was absent for " << std::fixed << std::setprecision(1) << lioGapSec
                     << " s; its map frame has most likely been re-origined. Withholding LIO unaries until a fresh "
                     << worldFrame_ << " -> " << lioFixedFrame << " alignment converges." << COLOR_END << "\n";
      }
      if (lioAlignState_.load(std::memory_order_acquire) == LioAlignState::Realigning) {
        bool inject = false;
        Eigen::Isometry3d T_W_M = Eigen::Isometry3d::Identity();
        {
          std::lock_guard<std::mutex> lk(realignMutex_);
          if (realignInjectPending_) {
            inject = true;
            T_W_M = realignT_W_M_;
            realignInjectPending_ = false;
          }
        }
        if (!inject && lioRealignNoGnssFallbackSec_ > 0.0) {
          // No GNSS to align against: anchor the (possibly re-origined) LIO map to the current world estimate so that
          // LIO keeps constraining the graph instead of being withheld for the rest of the outage.
          const double sinceGnss = lidarOdometryTimeK - lastValidGnssHeaderTime_.load(std::memory_order_relaxed);
          double sinceStart = 0.0;
          {
            std::lock_guard<std::mutex> lk(realignMutex_);
            sinceStart = lidarOdometryTimeK - realignStartHeaderTime_;
          }
          Eigen::Isometry3d T_W_I = Eigen::Isometry3d::Identity();
          if (sinceGnss > lioRealignNoGnssFallbackSec_ && sinceStart > lioRealignNoGnssFallbackSec_ &&
              lookupWorldImuPose_(lidarOdometryTimeK, 0.1, T_W_I)) {
            const Eigen::Isometry3d T_I_L = staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(), lioOdometryFrame);
            T_W_M = T_W_I * T_I_L * lio_T_M_Lk.inverse();
            inject = true;
            REGULAR_COUT << YELLOW_START << " [LIO realign] No GNSS for " << std::fixed << std::setprecision(1) << sinceGnss
                         << " s: anchoring " << lioFixedFrame << " to the current " << worldFrame_ << " estimate instead (fallback)."
                         << COLOR_END << "\n";
          }
        }
        if (!inject) {
          return;  // still aligning: this scan stays out of the graph
        }
        // Same thread as the unary below, so GMSF's initial-guess map is never accessed concurrently.
        this->initWorldFrameToFixedFrameTransform(T_W_M, lioFixedFrame);
        lioAlignState_.store(LioAlignState::Active, std::memory_order_release);
        REGULAR_COUT << GREEN_START << " [LIO realign] Injected T_" << worldFrame_ << "_" << lioFixedFrame << " (t = "
                     << T_W_M.translation().transpose() << "). Resuming LIO unaries." << COLOR_END << "\n";
      }
    }
    this->addUnaryPose3AbsoluteMeasurement(unary6DMeasurement);
  }

  // Visualization: do work only if needed (step 9.3)
  if (pubMeasMapLioLidarPath_->get_subscription_count() > 0 && areYawAndPositionInited()) {
    addToPathMsg(measLio_mapLidarPathPtr_, lioFixedFrame + referenceFrameAlignedNameId,
           odomLidarPtr->header.stamp, lio_T_M_Lk.translation(),
           static_cast<int>(graphConfigPtr_->imuBufferLength_ / 2.0));
    pubMeasMapLioLidarPath_->publish(*measLio_mapLidarPathPtr_);
  }
}

void B2WEstimator::vioOdometryCallback_(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& vioOdomPtr) {
  B2W_SCOPED_CB_TIMER("vioOdometryCallback_");

  if (!areRollAndPitchInited()) {
    return;
  }

  static std::uint64_t vioRawCallbackCounter = 0;
  static std::uint64_t vioAcceptedCallbackCounter = 0;
  static std::uint64_t vioRateGateRejectedCounter = 0;
  static FrequencyChecker vio_raw_wall_freq_checker(50, 5.0);
  static FrequencyChecker vio_accepted_wall_freq_checker(50, 5.0);
  static FrequencyChecker vio_accepted_stamp_freq_checker(50, 5.0);

  const double timeK =
      vioOdomPtr->header.stamp.sec + vioOdomPtr->header.stamp.nanosec * 1e-9;
  const double vioCallbackWallTimeK = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();

  ++vioRawCallbackCounter;
  const bool vioReportNow = vio_raw_wall_freq_checker.tick(vioCallbackWallTimeK);

  static double lastVioTimeK = 0.0;
  if (vioOdometryRate_ > 0.0 && lastVioTimeK > 0.0) {
    const double dt = timeK - lastVioTimeK;
    if (dt > 0.0 && dt < (1.0 / static_cast<double>(vioOdometryRate_))) {
      ++vioRateGateRejectedCounter;
      if (vioReportNow) {
        REGULAR_COUT << MAGENTA_START
                     << "[VIO] raw arrival rate (wall, avg last "
                     << vio_raw_wall_freq_checker.last_window_n() << "): "
                     << std::fixed << std::setprecision(2)
                     << vio_raw_wall_freq_checker.last_hz() << " Hz"
                     << " | accepted arrival rate (wall): "
                     << vio_accepted_wall_freq_checker.last_hz() << " Hz"
                     << " | accepted rate (header stamp): "
                     << vio_accepted_stamp_freq_checker.last_hz() << " Hz"
                     << " | accepted/raw: " << vioAcceptedCallbackCounter
                     << "/" << vioRawCallbackCounter
                     << " | rate-gated: " << vioRateGateRejectedCounter
                     << COLOR_END << "\n";
      }
      return;
    }
  }
  lastVioTimeK = timeK;
  double expectedFirstVio = 0.0;
  firstVioHeaderTime_.compare_exchange_strong(expectedFirstVio, timeK);
  ++vioAcceptedCallbackCounter;
  vio_accepted_wall_freq_checker.tick(vioCallbackWallTimeK);
  vio_accepted_stamp_freq_checker.tick(lastVioTimeK);

  static std::uint64_t vioUnaryCallbackCounter = 0;
  ++vioUnaryCallbackCounter;
  if (vioReportNow) {
    REGULAR_COUT << MAGENTA_START
                 << "[VIO] raw arrival rate (wall, avg last "
                 << vio_raw_wall_freq_checker.last_window_n() << "): "
                 << std::fixed << std::setprecision(2)
                 << vio_raw_wall_freq_checker.last_hz() << " Hz"
                 << " | accepted arrival rate (wall): "
                 << vio_accepted_wall_freq_checker.last_hz() << " Hz"
                 << " | accepted rate (header stamp): "
                 << vio_accepted_stamp_freq_checker.last_hz() << " Hz"
                 << " | accepted/raw: " << vioAcceptedCallbackCounter
                 << "/" << vioRawCallbackCounter
                 << " | rate-gated: " << vioRateGateRejectedCounter
                 << COLOR_END << "\n";
  }

  Eigen::Matrix<double, 6, 1> vio_covariance = vioPoseUnaryNoise_;

  Eigen::Isometry3d vio_T_M_Ck = Eigen::Isometry3d::Identity();
  graph_msf::geometryPoseToEigen(*vioOdomPtr, vio_T_M_Ck.matrix());

  // Frame name (cached)
  const std::string& vioOdometryFrame = vioOdometryFrame_;
  const std::string vioFixedFrame = epochFrameName_(vioEpoch_, vioOdomPtr->header.frame_id, timeK, vio_T_M_Ck, "VIO");

  // Publish a second alignment uncertainty disk for the active VIO odom/map frame.
  setHeadingUncertaintyFixedFrame("vio", vioFixedFrame);

  if (vioFixedFrame.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "VIO header.frame_id is empty. Absolute pose unary will be ill-defined.");
  } else if (vioFixedFrame == vioOdometryFrame) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "VIO absolute pose uses the sensor/body frame as fixed frame (%s). "
                         "Expected a map/odom-like reference frame in header.frame_id.",
                         vioFixedFrame.c_str());
  }

  // When LIO is not available (disabled, or enabled but not arriving / degenerate for lioAvailabilityTimeoutSec), the VIO map is
  // aligned with the world: the ZED trajectory becomes the SE3 input of the GNSS trajectory alignment. Convert the camera pose to the
  // base frame before buffering it, since the graph is initialized in baseFrame_. Only needed until the world is initialised.
  const bool lioAvailable = lioAvailable_(timeK);
  const auto noLioReason = [this]() {
    if (!useLioOdometryFlag_) return std::string("LiDAR odometry disabled");
    char reason[96];
    std::snprintf(reason, sizeof(reason), "LiDAR odometry enabled but not arriving for %g s", lioAvailabilityTimeoutSec_);
    return std::string(reason);
  };
  if (useGnssFlag_ && !lioAvailable && !useLioBetweenOdometryFlag_ && !areYawAndPositionInited() &&
      gnssHandlerPtr_->getUseYawInitialGuessFromAlignment()) {
    if (initAlignmentSource_.load() != InitAlignmentSource::Vio) {
      switchInitAlignmentSource_(InitAlignmentSource::Vio, timeK, noLioReason());
    }
    const Eigen::Isometry3d T_V_I =
        staticTransformsPtr_->rv_T_frame1_frame2(vioOdometryFrame_, staticTransformsPtr_->getImuFrame());
    const Eigen::Isometry3d T_I_B =
        staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getImuFrame(), baseLinkFrame_);
    const Eigen::Isometry3d vio_T_M_B = vio_T_M_Ck * T_V_I * T_I_B;

    addAlignmentPose_(timeK, vio_T_M_B);

    const Eigen::Vector3d p_G_in_M =
        vio_T_M_B.translation() + vio_T_M_B.rotation() * t_B_G_cached_;
    if (const auto aligner = aligner_()) {
      aligner->addSe3Position(p_G_in_M, timeK);
    }
  }

  graph_msf::UnaryMeasurementXDAbsolute<Eigen::Isometry3d, 6> unary6DMeasurement(
      "Vio_unary_6D",
      int(vioOdometryRate_),
      vioOdometryFrame,
      vioOdometryFrame + sensorFrameCorrectedNameId,
      lioVioRobustNorm(),
      // graph_msf::RobustNorm::None(),
      timeK,
      1.0,
      vio_T_M_Ck,
      vio_covariance,
      vioFixedFrame,
      worldFrame_,
      initialSe3AlignmentNoise_,
      currentAlignmentRandomWalk_(vioSe3AlignmentRandomWalk_, timeK),
      graph_msf::AbsoluteUnaryAlignmentRecoveryPolicy::RestartFromPrior);

  if (vioUnaryCallbackCounter <= 2) {
    // skip first few samples
  } else if (!areYawAndPositionInited()) {
    if (!useGnssFlag_ && !lioAvailable) {
      // No GNSS and no LiDAR odometry: the world frame is the VIO map.
      if (initAlignmentSource_.load() != InitAlignmentSource::Vio) {
        switchInitAlignmentSource_(InitAlignmentSource::Vio, timeK, noLioReason());
      }
      REGULAR_COUT << GREEN_START
                   << " VIO odometry callback is setting global yaw, as it was not set so far."
                   << COLOR_END << "\n";
      this->initYawAndPosition(unary6DMeasurement);
    }
  } else {
    this->addUnaryPose3AbsoluteMeasurement(unary6DMeasurement);
  }

  // Visualization: do work only if needed (step 9.3)
  if ((pubMeasMapVioPath_->get_subscription_count() > 0) && areYawAndPositionInited()) {
    addToPathMsg(measVio_mapCameraPathPtr_, vioFixedFrame + referenceFrameAlignedNameId, vioOdomPtr->header.stamp,
                 vio_T_M_Ck.translation(), static_cast<int>(graphConfigPtr_->imuBufferLength_ / 2.0));
    pubMeasMapVioPath_->publish(*measVio_mapCameraPathPtr_);
  }
}

void B2WEstimator::vioOdometryBetweenCallback_(const nav_msgs::msg::Odometry::ConstSharedPtr& vioOdomPtr) {
  B2W_SCOPED_CB_TIMER("vioOdometryBetweenCallback_");

  if (!areRollAndPitchInited()) {
    return;
  }

  const double timeK =
      vioOdomPtr->header.stamp.sec + vioOdomPtr->header.stamp.nanosec * 1e-9;

  static double lastVioTimeK = 0.0;
  if (vioOdometryBetweenRate_ > 0.0 && lastVioTimeK > 0.0) {
    const double dt = timeK - lastVioTimeK;
    if (dt > 0.0 && dt < (1.0 / static_cast<double>(vioOdometryBetweenRate_))) {
      return;
    }
  }
  lastVioTimeK = timeK;

  static std::uint64_t vioBetweenCallbackCounter = 0;
  ++vioBetweenCallbackCounter;

  static FrequencyChecker vio_between_freq_checker(50, 5.0);
  if (vio_between_freq_checker.tick(lastVioTimeK)) {
    REGULAR_COUT << MAGENTA_START
                 << "[VIO Between] callback frequency (avg last "
                 << vio_between_freq_checker.last_window_n() << "): "
                 << std::fixed << std::setprecision(2)
                 << vio_between_freq_checker.last_hz() << " Hz"
                 << COLOR_END << "\n";
  }

  // Noise: the configured standard deviations [rad, rad, rad, m, m, m] (noise_params.vioPoseBetweenNoiseDensity). The message
  // covariance is not used: it is the covariance of the absolute pose in the ZED odom frame, not of the relative motion. It used
  // to be read as variances into a standard-deviation vector, the configured value was never used (the size check was always
  // true for the fixed 36-element array), and a zero variance would have become a hard constraint in GTSAM.
  const Eigen::Matrix<double, 6, 1>& vio_covariance = vioPoseBetweenNoise_;

  Eigen::Isometry3d vio_T_M_Ck = Eigen::Isometry3d::Identity();
  graph_msf::odomMsgToEigen(*vioOdomPtr, vio_T_M_Ck.matrix());

  // Frame name (cached)
  const std::string& vioOdometryFrame = vioOdometryFrame_;

  if (vioOdometryFrame != vioOdomPtr->child_frame_id) {
    REGULAR_COUT << RED_START << "====================================================================\n"
                 << "ERROR: VIO ODOMETRY FRAME MISMATCH!\n"
                 << "Expected frame: " << vioOdometryFrame << "\n"
                 << "Odometry message child_frame_id: " << vioOdomPtr->child_frame_id << "\n"
                 << "====================================================================\n"
                 << COLOR_END << "\n";
  }

  static Eigen::Isometry3d vio_T_M_Ckm1 = Eigen::Isometry3d::Identity();
  static double vioTimeKm1 = 0.0;

  if (vioBetweenCallbackCounter <= 2) {
    vio_T_M_Ckm1 = vio_T_M_Ck;
    vioTimeKm1 = timeK;
    return;
  }

  if (!areYawAndPositionInited()) {
    vio_T_M_Ckm1 = vio_T_M_Ck;
    vioTimeKm1 = timeK;
    return;
  }

  if (vioTimeKm1 > 0.0 && timeK > vioTimeKm1 &&
      !isPlausibleSourceStep_(vioEpoch_, vio_T_M_Ckm1.inverse() * vio_T_M_Ck, timeK - vioTimeKm1)) {
    // A step the robot cannot make (e.g. a ZED relocalization) must not become a between factor.
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "[VIO Between] non-physical step of %.2f m in %.3f s (relocalization?). Skipping this delta.",
                         (vio_T_M_Ckm1.inverse() * vio_T_M_Ck).translation().norm(), timeK - vioTimeKm1);
  } else if (vioTimeKm1 > 0.0 && timeK > vioTimeKm1) {
    const Eigen::Isometry3d T_Ckm1_Ck = vio_T_M_Ckm1.inverse() * vio_T_M_Ck;

    graph_msf::BinaryMeasurementXD<Eigen::Isometry3d, 6> delta6DMeasurement(
        "Vio_between_6D",
        int(vioOdometryBetweenRate_),
        vioOdometryFrame,
        vioOdometryFrame + sensorFrameCorrectedNameId,
        lioVioRobustNorm(),
        vioTimeKm1,
        timeK,
        T_Ckm1_Ck,
        vio_covariance);

    this->addBinaryPose3Measurement(delta6DMeasurement);
  } else {
    REGULAR_COUT << RED_START
                 << "[VIO Between] ERROR: invalid time ordering for between factor. "
                 << "vioTimeKm1=" << std::fixed << std::setprecision(9) << vioTimeKm1
                 << ", timeK=" << timeK
                 << ", dt=" << (timeK - vioTimeKm1)
                 << COLOR_END << "\n";
  }

  vio_T_M_Ckm1 = vio_T_M_Ck;
  vioTimeKm1 = timeK;

  // Visualization: do work only if needed (step 9.3)
  if (pubMeasMapVioBetweenPath_->get_subscription_count() > 0) {
    addToPathMsg(measVioBetween_mapCameraPathPtr_,
                 vioOdometryFrame + referenceFrameAlignedNameId,
                 vioOdomPtr->header.stamp,
                 vio_T_M_Ck.translation(),
                 static_cast<int>(graphConfigPtr_->imuBufferLength_ / 2.0));
    pubMeasMapVioBetweenPath_->publish(*measVioBetween_mapCameraPathPtr_);
  }
}

// Wheel-leg odometry: base velocity from /joint_states + gyro through the hf_b2w_description rolling-contact model, fused as a
// body-frame velocity of the base. Slip handling: per-wheel voting, skid-turn gate, all-wheel-slip gate vs the current estimate,
// speed/leg-motion dependent noise and a robust kernel.
void B2WEstimator::jointStatesCallback_(const sensor_msgs::msg::JointState::ConstSharedPtr& jointStatesPtr) {
  B2W_SCOPED_CB_TIMER("jointStatesCallback_");
  if (!wheelOdometryPtr_ || !areYawAndPositionInited()) {
    return;
  }
  const double timeK = jointStatesPtr->header.stamp.sec + jointStatesPtr->header.stamp.nanosec * 1e-9;
  if (lastWheelOdometryTime_ > 0.0 && timeK - lastWheelOdometryTime_ < 1.0 / wheelOdometryRate_ - 1e-4) {
    return;  // rate gate (also drops non-increasing stamps)
  }

  // One-time setup: frames and the joint-name map
  if (!wheelOdometryFramesCached_) {
    if (wheelOdometryBaseFrame_ != baseLinkFrame_) {
      RCLCPP_ERROR(this->get_logger(), "[Wheel odometry] hf_b2w_description.baseFrame '%s' != base link frame '%s'. Disabled.",
                   wheelOdometryBaseFrame_.c_str(), baseLinkFrame_.c_str());
      wheelOdometryPtr_.reset();
      return;
    }
    const std::string& imuFrame = staticTransformsPtr_->getImuFrame();
    wheelOdometryR_B_I_ = staticTransformsPtr_->rv_T_frame1_frame2(baseLinkFrame_, imuFrame).rotation();
    wheelOdometryI_t_I_B_ = staticTransformsPtr_->rv_T_frame1_frame2(imuFrame, baseLinkFrame_).translation();
    wheelOdometryFramesCached_ = true;
  }
  if (!wheelOdometryPtr_->hasJointMap() && !wheelOdometryPtr_->setJointNames(jointStatesPtr->name)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "[Wheel odometry] /joint_states does not contain all joints of hf_b2w_description. Skipping.");
    ++wheelOdometryStats_.missing;
    return;
  }

  // Body rates and current velocity estimate at the joint-state time (base axes, base origin)
  BufferedState s;
  if (!lookupBufferedState_(timeK, 0.05, s)) {
    ++wheelOdometryStats_.noState;
    return;
  }
  const Eigen::Vector3d B_w_W_B = wheelOdometryR_B_I_ * s.I_w_W_I;
  const Eigen::Vector3d B_v_W_B_state = wheelOdometryR_B_I_ * (s.I_v_W_I + s.I_w_W_I.cross(wheelOdometryI_t_I_B_));

  const HfB2wWheelOdometryResult r =
      wheelOdometryPtr_->estimate(jointStatesPtr->position, jointStatesPtr->velocity, B_w_W_B, &B_v_W_B_state, timeK);
  lastWheelOdometryTime_ = timeK;

  using Status = HfB2wWheelOdometryResult::Status;
  switch (r.status) {
    case Status::Ok: r.standstill ? ++wheelOdometryStats_.standstill : ++wheelOdometryStats_.accepted; break;
    case Status::Turning: ++wheelOdometryStats_.turning; break;
    case Status::TooFewConsistentWheels: ++wheelOdometryStats_.inconsistent; break;
    case Status::DeviatesFromState: ++wheelOdometryStats_.deviates; break;
    case Status::MissingJoints: ++wheelOdometryStats_.missing; break;
  }
  if (timeK - wheelOdometryStats_.lastReportTime > 20.0) {
    const auto& st = wheelOdometryStats_;
    REGULAR_COUT << GREEN_START << " [Wheel odometry] accepted " << st.accepted << ", standstill " << st.standstill << ", rejected: turning "
                 << st.turning << ", wheels inconsistent (slip/lift) " << st.inconsistent << ", deviates from state (slip) "
                 << st.deviates << ", no state " << st.noState << ", missing joints " << st.missing << COLOR_END << "\n";
    wheelOdometryStats_.lastReportTime = timeK;
  }
  if (r.status != Status::Ok) {
    return;
  }

  graph_msf::UnaryMeasurementXD<Eigen::Vector3d, 3> B_v_W_B_measurement(
      "WheelOdometry", static_cast<int>(wheelOdometryRate_), baseLinkFrame_, baseLinkFrame_ + sensorFrameCorrectedNameId,
      makeRobustNorm_(wheelOdometryRobustNormType_, wheelOdometryRobustNormConstant_, "wheel_odometry.robustNorm"), timeK,
      /*covarianceViolationThreshold=*/0.0, r.B_v_W_B, r.sigma);
  this->addUnaryVelocity3LocalMeasurement(B_v_W_B_measurement);
}

}  // namespace b2w_se
