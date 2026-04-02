/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
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

  // GNSS parameters
  this->declare_parameter("gnss_params.initYaw", 90.0);
  this->declare_parameter("gnss_params.useYawInitialGuessFromFile", false);
  this->declare_parameter("gnss_params.yawInitialGuessFromAlignment", true);
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

  this->declare_parameter("sensor_params.lioBetweenOdometryRate", 0);
  this->declare_parameter("sensor_params.vioOdometryRate", 0);
  this->declare_parameter("sensor_params.vioOdometryBetweenRate", 0);

  // Alignment parameters (vector of double)
  this->declare_parameter("alignment_params.initialSe3AlignmentNoiseDensity", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  this->declare_parameter("alignment_params.lioSe3AlignmentRandomWalk", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  this->declare_parameter("alignment_params.vioSe3AlignmentRandomWalk", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

  // Noise parameters (vectors of double)
  this->declare_parameter("noise_params.lioPoseUnaryNoiseDensity", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
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
    t_B_G_cached_ = staticTransformsPtr_->rv_T_frame1_frame2(baseLinkFrame_, gnssFrame_).translation();
  }

  framesCached_ = true;
}

bool B2WEstimator::getClosestLioPose_(double t, Eigen::Isometry3d& T_M_B_out, double* best_dt_out) const {
  std::lock_guard<std::mutex> lk(lioPoseBufMutex_);
  if (lioPoseBuf_.empty()) {
    if (best_dt_out) *best_dt_out = std::numeric_limits<double>::infinity();
    return false;
  }

  auto best_it = lioPoseBuf_.begin();
  double best_dt = std::abs(best_it->first - t);

  for (auto it = lioPoseBuf_.begin(); it != lioPoseBuf_.end(); ++it) {
    const double dt = std::abs(it->first - t);
    if (dt < best_dt) { best_dt = dt; best_it = it; }
  }

  if (best_dt_out) *best_dt_out = best_dt;
  if (best_dt > kInitSyncMaxDt) return false;  // too far apart in time
  T_M_B_out = best_it->second;
  return true;
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

  if (useGnssFlag_) {
    pubGnssPoseWithCov = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/graph_msf/gnss_pose_with_covariance", best_effort_qos);
    pubMeasGNSSPath_ = this->create_publisher<nav_msgs::msg::Path>("/graph_msf/measGNSS_path", best_effort_qos);  // RViz: yellow
    REGULAR_COUT << COLOR_END << " Initialized GNSS Path publisher\n";

    auto qos_reliable_latched = rclcpp::QoS(10)
      .transient_local()
      .reliable()
      .keep_last(1);

    pubStatus_ = this->create_publisher<std_msgs::msg::Bool>("/graph_msf/alignment_status", qos_reliable_latched);

    pubReferenceNavSatFixCoordinates_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
      "/graph_msf/reference_gnss_position", qos_reliable_latched);

    pubReferenceNavSatFixCoordinatesENU_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
      "/graph_msf/reference_gnss_position_enu", qos_reliable_latched);
  }
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

  if (useVioOdometryBetweenFlag_) {
    subVioOdometryBetween_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/vio_odometry_between_topic", qos1,
        std::bind(&B2WEstimator::vioOdometryBetweenCallback_, this, std::placeholders::_1),
        so_vio);
    REGULAR_COUT << COLOR_END << " Initialized VIO Odometry between subscriber with topic: /vio_odometry_between_topic\n";
  }
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

void B2WEstimator::lidarBetweenOdometryCallback_(const nav_msgs::msg::Odometry::ConstSharedPtr& lidarBetweenOdomPtr) {
  B2W_SCOPED_CB_TIMER("lidarBetweenOdometryCallback_");

  if (!areRollAndPitchInited()) {
    return;
  }

  ++lidarBetweenCallbackCounter_;

  Eigen::Isometry3d lio_T_M_Lk = Eigen::Isometry3d::Identity();
  graph_msf::odomMsgToEigen(*lidarBetweenOdomPtr, lio_T_M_Lk.matrix());

  // Timestamp [s] (avoid rclcpp::Time construction)
  const double lidarBetweenTimeK =
      lidarBetweenOdomPtr->header.stamp.sec + lidarBetweenOdomPtr->header.stamp.nanosec * 1e-9;

  if (lidarBetweenCallbackCounter_ == 0) {
    lio_T_M_Lkm1_ = lio_T_M_Lk;
    lidarBetweenTimeKm1_ = lidarBetweenTimeK;
  }

  if (useGnssFlag_ && gnssHandlerPtr_->getUseYawInitialGuessFromAlignment()) {
    trajectoryAlignmentHandler_->addSe3Position(lio_T_M_Lk.translation(), lidarBetweenTimeK);
  }

  // Frame names (cached)
  const std::string& lioBetweenOdomFrameName = lidarBetweenFrame_;
  const std::string& lioOdomFrameName = lioOdometryFrame_;

  if (lidarBetweenCallbackCounter_ <= 2) {
    return;
  } else if (!areYawAndPositionInited()) {
    if (!useGnssFlag_ && !useLioOdometryFlag_) {
      graph_msf::UnaryMeasurementXD<Eigen::Isometry3d, 6> unary6DMeasurement(
          "Lidar_unary_6D", int(lioBetweenOdometryRate_), lioBetweenOdomFrameName, lioBetweenOdomFrameName + sensorFrameCorrectedNameId,
          graph_msf::RobustNorm::None(), lidarBetweenTimeK, 1.0, lio_T_M_Lk, lioPoseUnaryNoise_);
      REGULAR_COUT << GREEN_START << " LiDAR odometry callback is setting global yaw, as it was not set so far." << COLOR_END << "\n";
      this->initYawAndPosition(unary6DMeasurement);
    }
  } else {
    const Eigen::Isometry3d T_Lkm1_Lk = lio_T_M_Lkm1_.inverse() * lio_T_M_Lk;
    graph_msf::BinaryMeasurementXD<Eigen::Isometry3d, 6> delta6DMeasurement(
        "Lidar_between_6D", int(lioBetweenOdometryRate_), lioOdomFrameName, lioOdomFrameName + sensorFrameCorrectedNameId,
        graph_msf::RobustNorm::None(), lidarBetweenTimeKm1_, lidarBetweenTimeK, T_Lkm1_Lk, lioBetweenNoise_);
    this->addBinaryPose3Measurement(delta6DMeasurement);
  }

  lio_T_M_Lkm1_ = lio_T_M_Lk;
  lidarBetweenTimeKm1_ = lidarBetweenTimeK;

  // Visualization: do work only if needed (step 9.3)
  if (pubMeasMapLioPath_->get_subscription_count() > 0) {
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
    pubStatus_->publish(std_msgs::msg::Bool().set__data(false));
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

    if (gnssHandlerPtr_->getUseYawInitialGuessFromFile()) {
      initYaw_W_Base = gnssHandlerPtr_->getGlobalYawDegFromFile() / 180.0 * M_PI;
    } else if (gnssHandlerPtr_->getUseYawInitialGuessFromAlignment()) {

      if (gnssCallbackCounter_ % 20 == 0) {
        REGULAR_COUT << YELLOW_START << " Adding GNSS measurement to trajectory alignment." << "\n";
      }

      trajectoryAlignmentHandler_->addR3Position(W_t_W_Gnss, timeK);

      double yaw_W_M = 0.0;
      Eigen::Isometry3d T_W_M = Eigen::Isometry3d::Identity();
      if (!trajectoryAlignmentHandler_->alignTrajectories(yaw_W_M, T_W_M)) {
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
      if (!getClosestLioPose_(timeK, T_M_B_t0, &best_dt)) {
        if (!std::isfinite(best_dt)) {
          REGULAR_COUT << YELLOW_START
                       << "Trajectory alignment ready, but LIO pose buffer is empty (cannot compute yaw(W<-B))."
                       << COLOR_END << "\n";
        } else {
          REGULAR_COUT << YELLOW_START
                       << "Trajectory alignment ready, but no sufficiently time-synced LIO pose for yaw(W<-B). "
                       << "best_dt=" << std::fixed << std::setprecision(3) << best_dt << " s"
                       << COLOR_END << "\n";
        }
        return;
      }

      R_W_B_full = T_W_M.rotation() * T_M_B_t0.rotation();
      have_R_W_B_full = true;

      const Eigen::Vector3d x_W = R_W_B_full.col(0);
      initYaw_W_Base = std::atan2(x_W.y(), x_W.x());

      pubStatus_->publish(std_msgs::msg::Bool().set__data(true));
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

    if (this->initYawAndPositionInWorld(initYaw_W_Base, W_t_W_Base,
                                        /*frame1=*/baseFrame,
                                        /*frame2=*/baseFrame)) {
      REGULAR_COUT << GREEN_START << " GNSS initialization of yaw and position successful." << COLOR_END << "\n";
    } else {
      REGULAR_COUT << RED_START << " GNSS initialization of yaw and position failed." << COLOR_END << "\n";
    }

  } else {

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
        graph_msf::RobustNorm::Huber(0.25),
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

  static double lastLidarOdometryTimeK_ = 0.0;
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

  if (lastLidarOdometryTimeK_ > 0.0 &&
      (lidarOdometryTimeK - lastLidarOdometryTimeK_) < (1.0 / lioOdometryRate_)) {
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
  lastLidarOdometryTimeK_ = lidarOdometryTimeK;
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

  {
    std::lock_guard<std::mutex> lk(lioPoseBufMutex_);
    lioPoseBuf_.emplace_back(lidarOdometryTimeK, lio_T_M_Lk);

    const double t_min = lidarOdometryTimeK - kLioBufKeepSec;
    while (!lioPoseBuf_.empty() && lioPoseBuf_.front().first < t_min) {
      lioPoseBuf_.pop_front();
    }
  }

  // Frame Name
  const std::string& lioOdometryFrame = lioOdometryFrame_;
  const std::string& lioFixedFrame = odomLidarPtr->header.frame_id;

  // The heading-uncertainty disk is defined for the active world-to-fixed-frame alignment, where the fixed
  // frame is the drifting LiDAR odometry/map frame carried in header.frame_id, not the sensor/body child frame.
  setHeadingUncertaintyFixedFrame("lio", lioFixedFrame);

  if (useGnssFlag_ && gnssHandlerPtr_->getUseYawInitialGuessFromAlignment()) {

    const std::string& baseFrame =
        odomLidarPtr->child_frame_id.empty() ? baseLinkFrame_ : odomLidarPtr->child_frame_id;

    // Prefer cached lever arm if the message base frame matches cached base link.
    const Eigen::Vector3d t_B_G =
        (baseFrame == baseLinkFrame_) ? t_B_G_cached_
                                      : staticTransformsPtr_->rv_T_frame1_frame2(baseFrame, gnssFrame_).translation();

    const Eigen::Vector3d p_G_in_M =
        lio_T_M_Lk.translation() + lio_T_M_Lk.rotation() * t_B_G;

    trajectoryAlignmentHandler_->addSe3Position(p_G_in_M, lidarOdometryTimeK);
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
      graph_msf::RobustNorm::DCS(1.0),
      // graph_msf::RobustNorm::None(),
      lidarOdometryTimeK, 1.0,
      lio_T_M_Lk, lioPoseUnaryNoise_,
      lioFixedFrame, worldFrame_,
      initialSe3AlignmentNoise_, lioSe3AlignmentRandomWalk_,
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
  const std::string& vioFixedFrame = vioOdomPtr->header.frame_id;

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

  graph_msf::UnaryMeasurementXDAbsolute<Eigen::Isometry3d, 6> unary6DMeasurement(
      "Vio_unary_6D",
      int(vioOdometryRate_),
      vioOdometryFrame,
      vioOdometryFrame + sensorFrameCorrectedNameId,
      graph_msf::RobustNorm::DCS(1.0),
      // graph_msf::RobustNorm::None(),
      timeK,
      1.0,
      vio_T_M_Ck,
      vio_covariance,
      vioFixedFrame,
      worldFrame_,
      initialSe3AlignmentNoise_,
      vioSe3AlignmentRandomWalk_,
      graph_msf::AbsoluteUnaryAlignmentRecoveryPolicy::RestartFromPrior);

  if (vioUnaryCallbackCounter <= 2) {
    // skip first few samples
  } else if (!areYawAndPositionInited()) {
    if (!useGnssFlag_ && !useLioOdometryFlag_) {
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

  Eigen::Matrix<double, 6, 1> vio_covariance = vioPoseBetweenNoise_;

  if ((vioOdomPtr->pose.covariance.size() == 36) && (vioOdomPtr->pose.covariance.data() != nullptr)) {
    Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> cov_map(vioOdomPtr->pose.covariance.data());
    vio_covariance << cov_map(3, 3), cov_map(4, 4), cov_map(5, 5),
                      cov_map(0, 0), cov_map(1, 1), cov_map(2, 2);
  } else {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                         "VIO odometry message does not contain a 6x6 covariance. Using default between covariance.");
  }

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

  if (vioTimeKm1 > 0.0 && timeK > vioTimeKm1) {
    const Eigen::Isometry3d T_Ckm1_Ck = vio_T_M_Ckm1.inverse() * vio_T_M_Ck;

    graph_msf::BinaryMeasurementXD<Eigen::Isometry3d, 6> delta6DMeasurement(
        "Vio_between_6D",
        int(vioOdometryBetweenRate_),
        vioOdometryFrame,
        vioOdometryFrame + sensorFrameCorrectedNameId,
        graph_msf::RobustNorm::DCS(1.0),
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

}  // namespace b2w_se
