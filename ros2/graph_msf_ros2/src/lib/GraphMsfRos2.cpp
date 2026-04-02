// Implementation
#include "graph_msf_ros2/GraphMsfRos2.h"

// ROS2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <shared_mutex>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_srvs/srv/trigger.hpp>

// std
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>

// GTSAM
#include <gtsam/geometry/Pose3.h>

// Timing (minimal additions)
#include <atomic>
#include <chrono>
#include <cstdint>

// Workspace
#include "graph_msf/core/GraphManager.h"
#include "graph_msf_ros2/util/conversions.h"

// Set to 0 to compile out function timing.
#define GRAPH_MSF_ENABLE_FUNCTION_TIMING 0

namespace graph_msf {

namespace {
inline rclcpp::Time timeFromSec(double t_sec) {
  // Rounded to nearest nanosecond to avoid systematic truncation jitter.
  return rclcpp::Time(static_cast<int64_t>(std::llround(t_sec * 1e9)));
}

enum class PoseTangentFrame {
  kRightLocal,
  kLeftWorld,
};

struct HeadingSigmaEstimate {
  bool valid = false;
  double yawRad = 0.0;
  double sigmaRad = 0.0;
};

struct AttitudeSigmaEstimate {
  bool valid = false;
  double rollRad = 0.0;
  double pitchRad = 0.0;
  double yawRad = 0.0;
  double rollSigmaRad = 0.0;
  double pitchSigmaRad = 0.0;
  double yawSigmaRad = 0.0;
};

struct HeadingMarkerStyle {
  std::string sourceLabel;
  std::string markerSuffix;
  std_msgs::msg::ColorRGBA fillColor;
  std_msgs::msg::ColorRGBA boundaryColor;
  std_msgs::msg::ColorRGBA textColor;
};

inline double wrapAnglePi(double angle) {
  while (angle <= -M_PI) angle += 2.0 * M_PI;
  while (angle > M_PI) angle -= 2.0 * M_PI;
  return angle;
}

inline bool extractWorldHeading(const gtsam::Pose3& pose, double& headingRad) {
  const Eigen::Vector3d worldXAxis = pose.rotation().matrix().col(0);
  const double horizontalNorm = worldXAxis.head<2>().norm();
  if (horizontalNorm < 1e-6) {
    return false;
  }

  headingRad = std::atan2(worldXAxis.y(), worldXAxis.x());
  return true;
}

inline bool extractWorldRoll(const gtsam::Pose3& pose, double& rollRad) {
  rollRad = pose.rotation().roll();
  return std::isfinite(rollRad);
}

inline bool extractWorldPitch(const gtsam::Pose3& pose, double& pitchRad) {
  pitchRad = pose.rotation().pitch();
  return std::isfinite(pitchRad);
}

gtsam::Pose3 perturbPose(const gtsam::Pose3& pose, const gtsam::Vector6& delta, const PoseTangentFrame tangentFrame) {
  const gtsam::Pose3 deltaPose = gtsam::Pose3::Expmap(delta);
  if (tangentFrame == PoseTangentFrame::kLeftWorld) {
    return deltaPose.compose(pose);
  }
  return pose.compose(deltaPose);
}

AttitudeSigmaEstimate computeAttitudeMeanSigmasNumeric(const gtsam::Pose3& pose,
                                                       const gtsam::Matrix66& covariance,
                                                       const PoseTangentFrame tangentFrame) {
  AttitudeSigmaEstimate result;
  if (!extractWorldRoll(pose, result.rollRad) || !extractWorldPitch(pose, result.pitchRad) ||
      !extractWorldHeading(pose, result.yawRad)) {
    return result;
  }

  // The covariance query and the angle Jacobians live in the tangent space of the pose representation.
  // Estimate all three scalar Jacobians numerically in the same tangent convention as the covariance.
  constexpr double kEps = 1e-6;
  Eigen::Matrix<double, 3, 6> J = Eigen::Matrix<double, 3, 6>::Zero();
  for (int i = 0; i < 6; ++i) {
    gtsam::Vector6 delta = gtsam::Vector6::Zero();
    delta(i) = kEps;

    const gtsam::Pose3 posePlus = perturbPose(pose, delta, tangentFrame);
    const gtsam::Pose3 poseMinus = perturbPose(pose, -delta, tangentFrame);

    double rollPlus = 0.0;
    double pitchPlus = 0.0;
    double yawPlus = 0.0;
    double rollMinus = 0.0;
    double pitchMinus = 0.0;
    double yawMinus = 0.0;
    if (!extractWorldRoll(posePlus, rollPlus) || !extractWorldPitch(posePlus, pitchPlus) ||
        !extractWorldHeading(posePlus, yawPlus) || !extractWorldRoll(poseMinus, rollMinus) ||
        !extractWorldPitch(poseMinus, pitchMinus) || !extractWorldHeading(poseMinus, yawMinus)) {
      return AttitudeSigmaEstimate{};
    }

    J(0, i) = wrapAnglePi(rollPlus - rollMinus) / (2.0 * kEps);
    J(1, i) = (pitchPlus - pitchMinus) / (2.0 * kEps);
    J(2, i) = wrapAnglePi(yawPlus - yawMinus) / (2.0 * kEps);
  }

  const gtsam::Matrix66 symmetricCovariance = 0.5 * (covariance + covariance.transpose());
  const Eigen::Matrix3d angleCovariance = J * symmetricCovariance * J.transpose();
  if (!angleCovariance.allFinite()) {
    return AttitudeSigmaEstimate{};
  }

  result.valid = true;
  result.rollSigmaRad = std::sqrt(std::max(0.0, angleCovariance(0, 0)));
  result.pitchSigmaRad = std::sqrt(std::max(0.0, angleCovariance(1, 1)));
  result.yawSigmaRad = std::sqrt(std::max(0.0, angleCovariance(2, 2)));
  return result;
}

HeadingSigmaEstimate computeYawMeanSigmaNumeric(const gtsam::Pose3& pose,
                                               const gtsam::Matrix66& covariance,
                                               const PoseTangentFrame tangentFrame) {
  HeadingSigmaEstimate result;
  const AttitudeSigmaEstimate attitudeEstimate = computeAttitudeMeanSigmasNumeric(pose, covariance, tangentFrame);
  if (!attitudeEstimate.valid) {
    return result;
  }
  result.valid = true;
  result.yawRad = attitudeEstimate.yawRad;
  result.sigmaRad = attitudeEstimate.yawSigmaRad;
  return result;
}

geometry_msgs::msg::Point makeMarkerPoint(const Eigen::Vector3d& positionWorld) {
  geometry_msgs::msg::Point point;
  point.x = positionWorld.x();
  point.y = positionWorld.y();
  point.z = positionWorld.z();
  return point;
}

std::string formatHeadingSigmaLabel(const std::string& label, const double sigmaRad) {
  std::ostringstream ss;
  const double sigmaDeg = sigmaRad * 180.0 / M_PI;
  const int precision = sigmaDeg < 0.1 ? 3 : 2;
  ss << label << " sigma=" << std::fixed << std::setprecision(precision) << sigmaDeg << " deg";
  return ss.str();
}

HeadingMarkerStyle makeHeadingMarkerStyle(const std::string& sourceId) {
  HeadingMarkerStyle style;
  style.sourceLabel = sourceId;
  style.markerSuffix = sourceId;

  auto setColor = [](std_msgs::msg::ColorRGBA& color, const float r, const float g, const float b, const float a) {
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
  };

  if (sourceId == "lio") {
    style.sourceLabel = "LIO";
    style.markerSuffix = "lio";
    setColor(style.fillColor, 0.95F, 0.55F, 0.15F, 0.28F);
    setColor(style.boundaryColor, 0.98F, 0.70F, 0.25F, 0.95F);
    setColor(style.textColor, 0.98F, 0.85F, 0.70F, 0.98F);
  } else if (sourceId == "fused") {
    style.sourceLabel = "FUSED";
    style.markerSuffix = "fused";
    setColor(style.fillColor, 0.90F, 0.25F, 0.85F, 0.24F);
    setColor(style.boundaryColor, 0.98F, 0.45F, 0.92F, 0.96F);
    setColor(style.textColor, 1.00F, 0.82F, 0.98F, 0.98F);
  } else if (sourceId == "vio") {
    style.sourceLabel = "VIO";
    style.markerSuffix = "vio";
    setColor(style.fillColor, 0.20F, 0.65F, 0.95F, 0.24F);
    setColor(style.boundaryColor, 0.35F, 0.85F, 1.00F, 0.95F);
    setColor(style.textColor, 0.75F, 0.92F, 1.00F, 0.98F);
  } else {
    style.markerSuffix = sourceId.empty() ? "alignment" : sourceId;
    setColor(style.fillColor, 0.80F, 0.80F, 0.80F, 0.20F);
    setColor(style.boundaryColor, 0.92F, 0.92F, 0.92F, 0.90F);
    setColor(style.textColor, 0.95F, 0.95F, 0.95F, 0.98F);
  }

  return style;
}

void fillHeadingUncertaintyDiskMarker(visualization_msgs::msg::Marker& marker,
                                      const Eigen::Vector3d& robotPositionWorld,
                                      const double meanHeadingRad,
                                      const double sigmaHeadingRad,
                                      const double diskRadius,
                                      const double nSigmas,
                                      const double zOffset) {
  constexpr int kNumCircleSegments = 72;
  marker.points.clear();

  // A heading uncertainty is angular, not planar.
  // We therefore draw a robot-centered disk sector that spans +/- N*sigma around the current world heading.
  const double halfAngle = std::min(M_PI, std::max(0.0, nSigmas * sigmaHeadingRad));
  const double startAngle = meanHeadingRad - halfAngle;
  const double endAngle = meanHeadingRad + halfAngle;
  const bool drawFullDisk = halfAngle >= M_PI - 1e-6;

  const int numSegments = drawFullDisk ? kNumCircleSegments
                                       : std::max(6, static_cast<int>(std::ceil((2.0 * halfAngle / (2.0 * M_PI)) *
                                                                                static_cast<double>(kNumCircleSegments))));
  const Eigen::Vector3d centerWorld = robotPositionWorld + Eigen::Vector3d(0.0, 0.0, zOffset);
  const geometry_msgs::msg::Point centerPoint = makeMarkerPoint(centerWorld);

  for (int i = 0; i < numSegments; ++i) {
    const double angle0 = drawFullDisk
                              ? (2.0 * M_PI * static_cast<double>(i)) / static_cast<double>(numSegments)
                              : startAngle + (endAngle - startAngle) * static_cast<double>(i) / static_cast<double>(numSegments);
    const double angle1 = drawFullDisk
                              ? (2.0 * M_PI * static_cast<double>(i + 1)) / static_cast<double>(numSegments)
                              : startAngle + (endAngle - startAngle) * static_cast<double>(i + 1) / static_cast<double>(numSegments);

    const Eigen::Vector3d edge0 = centerWorld + Eigen::Vector3d(diskRadius * std::cos(angle0), diskRadius * std::sin(angle0), 0.0);
    const Eigen::Vector3d edge1 = centerWorld + Eigen::Vector3d(diskRadius * std::cos(angle1), diskRadius * std::sin(angle1), 0.0);

    marker.points.push_back(centerPoint);
    marker.points.push_back(makeMarkerPoint(edge0));
    marker.points.push_back(makeMarkerPoint(edge1));
  }
}

void fillHeadingUncertaintyDiskOutlineMarker(visualization_msgs::msg::Marker& marker,
                                             const Eigen::Vector3d& robotPositionWorld,
                                             const double diskRadius,
                                             const double zOffset) {
  constexpr int kNumCircleSegments = 72;
  marker.points.clear();

  const Eigen::Vector3d centerWorld = robotPositionWorld + Eigen::Vector3d(0.0, 0.0, zOffset);
  for (int i = 0; i <= kNumCircleSegments; ++i) {
    const double angle = (2.0 * M_PI * static_cast<double>(i)) / static_cast<double>(kNumCircleSegments);
    const Eigen::Vector3d pointWorld =
        centerWorld + Eigen::Vector3d(diskRadius * std::cos(angle), diskRadius * std::sin(angle), 0.0);
    marker.points.push_back(makeMarkerPoint(pointWorld));
  }
}

void fillHeadingUncertaintySectorBoundaryMarker(visualization_msgs::msg::Marker& marker,
                                                const Eigen::Vector3d& robotPositionWorld,
                                                const double meanHeadingRad,
                                                const double sigmaHeadingRad,
                                                const double diskRadius,
                                                const double nSigmas,
                                                const double zOffset) {
  constexpr int kNumCircleSegments = 72;
  marker.points.clear();

  const double halfAngle = std::min(M_PI, std::max(0.0, nSigmas * sigmaHeadingRad));
  const double startAngle = meanHeadingRad - halfAngle;
  const double endAngle = meanHeadingRad + halfAngle;
  const bool drawFullDisk = halfAngle >= M_PI - 1e-6;

  const Eigen::Vector3d centerWorld = robotPositionWorld + Eigen::Vector3d(0.0, 0.0, zOffset);
  marker.points.push_back(makeMarkerPoint(centerWorld));

  if (drawFullDisk) {
    for (int i = 0; i <= kNumCircleSegments; ++i) {
      const double angle = (2.0 * M_PI * static_cast<double>(i)) / static_cast<double>(kNumCircleSegments);
      const Eigen::Vector3d pointWorld =
          centerWorld + Eigen::Vector3d(diskRadius * std::cos(angle), diskRadius * std::sin(angle), 0.0);
      marker.points.push_back(makeMarkerPoint(pointWorld));
    }
    marker.points.push_back(makeMarkerPoint(centerWorld));
    return;
  }

  const int numSegments =
      std::max(6, static_cast<int>(std::ceil((2.0 * halfAngle / (2.0 * M_PI)) * static_cast<double>(kNumCircleSegments))));
  for (int i = 0; i <= numSegments; ++i) {
    const double angle = startAngle + (endAngle - startAngle) * static_cast<double>(i) / static_cast<double>(numSegments);
    const Eigen::Vector3d pointWorld =
        centerWorld + Eigen::Vector3d(diskRadius * std::cos(angle), diskRadius * std::sin(angle), 0.0);
    marker.points.push_back(makeMarkerPoint(pointWorld));
  }
  marker.points.push_back(makeMarkerPoint(centerWorld));
}

void fillHeadingArrowMarkerPoints(visualization_msgs::msg::Marker& marker,
                                  const Eigen::Vector3d& robotPositionWorld,
                                  const double meanHeadingRad,
                                  const double diskRadius,
                                  const double zOffset) {
  marker.points.clear();

  geometry_msgs::msg::Point startPoint;
  startPoint.x = robotPositionWorld.x();
  startPoint.y = robotPositionWorld.y();
  startPoint.z = robotPositionWorld.z() + zOffset;

  geometry_msgs::msg::Point endPoint;
  endPoint.x = robotPositionWorld.x() + diskRadius * std::cos(meanHeadingRad);
  endPoint.y = robotPositionWorld.y() + diskRadius * std::sin(meanHeadingRad);
  endPoint.z = robotPositionWorld.z() + zOffset;

  marker.points.push_back(startPoint);
  marker.points.push_back(endPoint);
}
}  // namespace

#if GRAPH_MSF_ENABLE_FUNCTION_TIMING
namespace detail {

struct FunctionTimingStats {
  std::atomic<std::uint64_t> n{0};
  std::atomic<std::int64_t> total_ns{0};
  std::atomic<std::int64_t> max_ns{0};
  std::atomic<std::int64_t> last_report_ns{0};
};

inline void atomicMax(std::atomic<std::int64_t>& a, std::int64_t v) {
  std::int64_t cur = a.load(std::memory_order_relaxed);
  while (v > cur && !a.compare_exchange_weak(cur, v, std::memory_order_relaxed)) {}
}

class ScopedFunctionTimer {
public:
  ScopedFunctionTimer(const char* name, const rclcpp::Logger& logger, FunctionTimingStats& stats)
  : name_(name), logger_(logger), stats_(stats), start_(std::chrono::steady_clock::now()) {}

  ~ScopedFunctionTimer() {
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
      RCLCPP_INFO(logger_, "[T] %s: n=%llu avg=%.3f ms max=%.3f ms",
                  name_, static_cast<unsigned long long>(n), avg_ms, max_ms);
    }
  }

private:
  const char* name_;
  rclcpp::Logger logger_;
  FunctionTimingStats& stats_;
  std::chrono::steady_clock::time_point start_;
};

}  // namespace detail

#define GRAPH_MSF_SCOPED_TIMER(NAME_LITERAL) \
  static ::graph_msf::detail::FunctionTimingStats __fn_stats; \
  ::graph_msf::detail::ScopedFunctionTimer __fn_timer((NAME_LITERAL), this->get_logger(), __fn_stats)

#else
#define GRAPH_MSF_SCOPED_TIMER(NAME_LITERAL) (void)0
#endif

GraphMsfRos2::GraphMsfRos2(const std::string& nodeName, const rclcpp::NodeOptions& options) : Node(nodeName, options) {
  GRAPH_MSF_SCOPED_TIMER("GraphMsfRos2::GraphMsfRos2");

  RCLCPP_INFO(this->get_logger(), "GraphMsfRos2-Constructor called.");

  // Configurations ----------------------------
  // Graph Config
  graphConfigPtr_ = std::make_shared<GraphConfig>();

  // Start non-time-critical thread for paths, variances, markers
  nonTimeCriticalThread_ = std::thread(&GraphMsfRos2::nonTimeCriticalThreadFunction, this);

  // Start IMU odometry thread for time-critical publishing
  imuOdomThread_ = std::thread(&GraphMsfRos2::imuOdomThreadFunction, this);

  // Start TF transforms thread for time-critical publishing
  tfTransformThread_ = std::thread(&GraphMsfRos2::tfTransformThreadFunction, this);
}

GraphMsfRos2::~GraphMsfRos2() {
  GRAPH_MSF_SCOPED_TIMER("GraphMsfRos2::~GraphMsfRos2");

  RCLCPP_INFO(this->get_logger(), "GraphMsfRos2-Destructor called.");

  // Signal all threads to shutdown
  shutdownRequested_ = true;
  nonTimeCriticalQueueCondition_.notify_all();
  imuOdomQueueCondition_.notify_all();
  tfTransformQueueCondition_.notify_all();

  // Wait for non-time-critical thread to finish
  if (nonTimeCriticalThread_.joinable()) {
    nonTimeCriticalThread_.join();
  }

  // Wait for IMU odometry thread to finish
  if (imuOdomThread_.joinable()) {
    imuOdomThread_.join();
  }

  // Wait for TF transform thread to finish
  if (tfTransformThread_.joinable()) {
    tfTransformThread_.join();
  }
}

void GraphMsfRos2::setup(std::shared_ptr<StaticTransforms> staticTransformsPtr) {
  GRAPH_MSF_SCOPED_TIMER("GraphMsfRos2::setup");

  RCLCPP_INFO(this->get_logger(), "GraphMsfRos2-Setup called.");

  // Check
  if (staticTransformsPtr == nullptr) {
    throw std::runtime_error("Static transforms not set. Has to be set.");
  }

  // Clock
  clock_ = this->get_clock();

  // Declare ROS parameters
  GraphMsfRos2::declareRosParams();

  // Read parameters ----------------------------
  GraphMsfRos2::readParams();

  // Call super class Setup ----------------------------
  GraphMsf::setup(graphConfigPtr_, staticTransformsPtr);

  // Publishers ----------------------------
  GraphMsfRos2::initializePublishers();

  // Subscribers ----------------------------
  GraphMsfRos2::initializeSubscribers();

  // Messages ----------------------------
  GraphMsfRos2::initializeMessages();

  tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // Services ----------------------------
  GraphMsfRos2::initializeServices(*this);

  // Time
  startTime = std::chrono::high_resolution_clock::now();

  // Wrap up
  RCLCPP_INFO(this->get_logger(), "Set up successfully.");
}

void GraphMsfRos2::initializePublishers() {
  RCLCPP_INFO(this->get_logger(), "Initializing publishers.");

  // Odometry
  pubEstOdomImu_ = this->create_publisher<nav_msgs::msg::Odometry>("/graph_msf/est_odometry_odom_imu", ROS_QUEUE_SIZE);
  pubEstWorldImu_ = this->create_publisher<nav_msgs::msg::Odometry>("/graph_msf/est_odometry_world_imu", ROS_QUEUE_SIZE);
  pubOptWorldImu_ = this->create_publisher<nav_msgs::msg::Odometry>("/graph_msf/opt_odometry_world_imu", ROS_QUEUE_SIZE);

  // Non-time-critical publishers with best-effort and larger queue
  auto best_effort_qos = rclcpp::QoS(100)
                             .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                             .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE)
                             .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

  // Vector3 Variances
  pubEstWorldPosVariance_ =
      this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/graph_msf/est_world_pos_variance", best_effort_qos);
  pubEstWorldRotVariance_ =
      this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/graph_msf/est_world_rot_variance", best_effort_qos);
  pubAttitudeSigmas_ =
      this->create_publisher<geometry_msgs::msg::TwistStamped>("/graph_msf/opt_attitude_sigmas", best_effort_qos);

  // Velocity Marker
  pubLinVelocityMarker_ = this->create_publisher<visualization_msgs::msg::Marker>("/graph_msf/lin_velocity_marker", best_effort_qos);
  pubAngularVelocityMarker_ = this->create_publisher<visualization_msgs::msg::Marker>("/graph_msf/angular_velocity_marker", best_effort_qos);
  if (publishHeadingUncertaintyMarkersFlag_) {
    pubHeadingUncertaintyMarkers_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("/graph_msf/heading_uncertainty_markers", best_effort_qos);
  }

  // QoS profile for paths to be visualized in RViz (latching)
  // auto rviz_path_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

  // Paths
  pubEstOdomImuPath_ = this->create_publisher<nav_msgs::msg::Path>("/graph_msf/est_path_odom_imu", best_effort_qos);    // RViz: white
  pubEstWorldImuPath_ = this->create_publisher<nav_msgs::msg::Path>("/graph_msf/est_path_world_imu", best_effort_qos);  // RViz: light blue
  pubOptWorldImuPath_ = this->create_publisher<nav_msgs::msg::Path>("/graph_msf/opt_path_world_imu", best_effort_qos);  // RViz: magenta

  // Imu Bias
  pubAccelBias_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/graph_msf/accel_bias", best_effort_qos);
  pubGyroBias_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/graph_msf/gyro_bias", best_effort_qos);
}

void GraphMsfRos2::initializeSubscribers() {
  RCLCPP_INFO(this->get_logger(), "Initializing subscribers.");

  // Imu
  subImu_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu_topic", rclcpp::QoS(ROS_QUEUE_SIZE).best_effort(),
                                                             std::bind(&GraphMsfRos2::imuCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "GraphMsfRos2 Initialized main IMU subscriber with topic: %s", subImu_->get_topic_name());
}

void GraphMsfRos2::initializeMessages() {
  RCLCPP_INFO(this->get_logger(), "Initializing messages.");

  // Odometry
  estOdomImuMsgPtr_ = std::make_shared<nav_msgs::msg::Odometry>();
  estWorldImuMsgPtr_ = std::make_shared<nav_msgs::msg::Odometry>();
  optWorldImuMsgPtr_ = std::make_shared<nav_msgs::msg::Odometry>();

  // Vector3 Variances
  estWorldPosVarianceMsgPtr_ = std::make_shared<geometry_msgs::msg::Vector3Stamped>();
  estWorldRotVarianceMsgPtr_ = std::make_shared<geometry_msgs::msg::Vector3Stamped>();
  attitudeSigmasMsgPtr_ = std::make_shared<geometry_msgs::msg::TwistStamped>();

  // Path
  estOdomImuPathPtr_ = std::make_shared<nav_msgs::msg::Path>();
  estWorldImuPathPtr_ = std::make_shared<nav_msgs::msg::Path>();
  optWorldImuPathPtr_ = std::make_shared<nav_msgs::msg::Path>();

  // Imu Bias
  accelBiasMsgPtr_ = std::make_shared<geometry_msgs::msg::Vector3Stamped>();
  gyroBiasMsgPtr_ = std::make_shared<geometry_msgs::msg::Vector3Stamped>();
}

void GraphMsfRos2::initializeServices(rclcpp::Node& node) {
  RCLCPP_INFO(node.get_logger(), "Initializing services.");

  // Trigger offline smoother optimization
  srvSmootherOptimize_ = node.create_service<std_srvs::srv::Trigger>(
      "/graph_msf/trigger_offline_optimization",
      std::bind(&GraphMsfRos2::srvOfflineSmootherOptimizeCallback, this, std::placeholders::_1, std::placeholders::_2));
}

bool GraphMsfRos2::srvOfflineSmootherOptimizeCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                                      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
  GRAPH_MSF_SCOPED_TIMER("GraphMsfRos2::srvOfflineSmootherOptimizeCallback");

  // Hardcode max iterations (you can make this configurable via parameter if needed)
  int maxIterations = 100;

  // Trigger offline smoother optimization and create response
  if (optimizeSlowBatchSmoother(maxIterations, optimizationResultLoggingPath, false)) {
    res->success = true;
    res->message = "Optimization successful.";
  } else {
    res->success = false;
    res->message = "Optimization failed.";
  }
  return true;
}

void GraphMsfRos2::addToPathMsg(const nav_msgs::msg::Path::SharedPtr& pathPtr, const std::string& fixedFrameName, const rclcpp::Time& stamp,
                                const Eigen::Vector3d& t, const int maxBufferLength) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = fixedFrameName;
  pose.header.stamp = stamp;
  pose.pose.position.x = t(0);
  pose.pose.position.y = t(1);
  pose.pose.position.z = t(2);
  pose.pose.orientation.w = 1.0;

  pathPtr->header.frame_id = fixedFrameName;
  pathPtr->header.stamp = stamp;
  pathPtr->poses.push_back(pose);

  constexpr std::size_t kTrimMargin = 50;
  const std::size_t maxLen = static_cast<std::size_t>(std::max(0, maxBufferLength));

  if (maxLen > 0 && pathPtr->poses.size() > (maxLen + kTrimMargin)) {
    const std::size_t excess = pathPtr->poses.size() - maxLen;
    pathPtr->poses.erase(pathPtr->poses.begin(), pathPtr->poses.begin() + static_cast<long>(excess));
  }
}

void GraphMsfRos2::addToOdometryMsg(const nav_msgs::msg::Odometry::SharedPtr& msgPtr, const std::string& fixedFrame,
                                    const std::string& movingFrame, const rclcpp::Time& stamp, const Eigen::Isometry3d& T,
                                    const Eigen::Vector3d& F_v_W_F, const Eigen::Vector3d& F_w_W_F,
                                    const Eigen::Matrix<double, 6, 6>& poseCovariance, const Eigen::Matrix<double, 6, 6>& twistCovariance) {
  msgPtr->header.frame_id = fixedFrame;
  msgPtr->child_frame_id = movingFrame;
  msgPtr->header.stamp = stamp;

  // Convert Eigen::Isometry3d to geometry_msgs::msg::Pose using tf2
  geometry_msgs::msg::Pose pose_msg;
  tf2::convert(T, pose_msg);
  msgPtr->pose.pose = pose_msg;

  msgPtr->twist.twist.linear.x = F_v_W_F(0);
  msgPtr->twist.twist.linear.y = F_v_W_F(1);
  msgPtr->twist.twist.linear.z = F_v_W_F(2);
  msgPtr->twist.twist.angular.x = F_w_W_F(0);
  msgPtr->twist.twist.angular.y = F_w_W_F(1);
  msgPtr->twist.twist.angular.z = F_w_W_F(2);

  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      msgPtr->pose.covariance[6 * i + j] = poseCovariance(i, j);
      msgPtr->twist.covariance[6 * i + j] = twistCovariance(i, j);
    }
  }
}

void GraphMsfRos2::extractCovariancesFromOptimizedState(
    Eigen::Matrix<double, 6, 6>& poseCovarianceRos, Eigen::Matrix<double, 6, 6>& twistCovarianceRos,
    const std::shared_ptr<graph_msf::SafeNavStateWithCovarianceAndBias>& optimizedStateWithCovarianceAndBiasPtr) {
    // Extract covariances from optimized state
    poseCovarianceRos.setZero();
    twistCovarianceRos.setZero();

    if (optimizedStateWithCovarianceAndBiasPtr != nullptr) {
      poseCovarianceRos =
          graph_msf::convertCovarianceGtsamConventionToRosConvention(optimizedStateWithCovarianceAndBiasPtr->getPoseCovariance());
      twistCovarianceRos.block<3, 3>(0, 0) = optimizedStateWithCovarianceAndBiasPtr->getVelocityCovariance();
    }
}

// Markers
void GraphMsfRos2::createLinVelocityMarker(const std::string& referenceFrameName, const rclcpp::Time& stamp, const Eigen::Vector3d& velocity,
                                           visualization_msgs::msg::Marker& marker) {
  // Arrow
  marker.header.frame_id = referenceFrameName;
  marker.header.stamp = stamp;
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // Scale and Color
  marker.scale.x = 0.1;  // shaft diameter
  marker.scale.y = 0.2;  // head diameter
  marker.scale.z = 0.2;  // head length
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;

  // Define Arrow through start and end point
  geometry_msgs::msg::Point startPoint, endPoint;
  startPoint.x = 0.0;  // origin
  startPoint.y = 0.0;  // origin
  startPoint.z = 0.0;  // 0 meter above origin
  endPoint.x = startPoint.x + velocity(0);
  endPoint.y = startPoint.y + velocity(1);
  endPoint.z = startPoint.z + velocity(2);
  marker.points.push_back(startPoint);
  marker.points.push_back(endPoint);

  // Quaternion for orientation
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  marker.pose.orientation.x = q.x();
  marker.pose.orientation.y = q.y();
  marker.pose.orientation.z = q.z();
  marker.pose.orientation.w = q.w();
}

void GraphMsfRos2::createAngularVelocityMarker(const std::string& referenceFrameName, const rclcpp::Time& stamp,
                                               const Eigen::Vector3d& angularVelocity, const Eigen::Isometry3d& currentPose,
                                               visualization_msgs::msg::Marker& marker) {
  // Cylinder to visualize angular velocity as a disc/ring oriented along rotation axis
  marker.header.frame_id = referenceFrameName;
  marker.header.stamp = stamp;
  marker.ns = "angular_velocity";
  marker.id = 1;
  marker.type = visualization_msgs::msg::Marker::CYLINDER;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // Angular velocity magnitude
  double angularMagnitude = angularVelocity.norm();

  if (angularMagnitude > 1e-6) {  // Only create marker if there's significant angular velocity
    // Scale based on angular velocity magnitude
    double baseRadius = std::min(std::max(angularMagnitude * 0.2, 0.1), 0.5);
    marker.scale.x = baseRadius * 2.0;  // diameter in x
    marker.scale.y = baseRadius * 2.0;  // diameter in y
    marker.scale.z = 0.02;              // thin disc height

    // Color: blue for angular velocity with alpha based on magnitude
    marker.color.a = std::min(angularMagnitude * 0.5 + 0.3, 1.0);
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
  } else {
    // No significant angular velocity - make marker invisible
    marker.scale.x = 0.0;
    marker.scale.y = 0.0;
    marker.scale.z = 0.0;
    marker.color.a = 0.0;
  }

  // Set lifetime
  marker.lifetime = rclcpp::Duration::from_seconds(0.1);

  // Position at current pose position
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;

  // Orient the disc perpendicular to the angular velocity vector (rotation axis)
  if (angularMagnitude > 1e-6) {
    Eigen::Vector3d rotationAxis = angularVelocity.normalized();

    // Create a rotation that aligns the cylinder's z-axis with the rotation axis
    // Default cylinder orientation is along z-axis
    Eigen::Vector3d zAxis(0, 0, 1);

    // Calculate rotation to align z-axis with rotation axis
    Eigen::Quaterniond orientation;
    if (rotationAxis.dot(zAxis) > 0.9999) {
      // Already aligned
      orientation = Eigen::Quaterniond::Identity();
    } else if (rotationAxis.dot(zAxis) < -0.9999) {
      // Opposite direction - rotate 180 degrees around x-axis
      orientation = Eigen::Quaterniond(0, 1, 0, 0);
    } else {
      // General case - use cross product to find rotation axis
      Eigen::Vector3d rotAxis = zAxis.cross(rotationAxis).normalized();
      double angle = std::acos(std::max(-1.0, std::min(1.0, zAxis.dot(rotationAxis))));
      orientation = Eigen::Quaterniond(Eigen::AngleAxisd(angle, rotAxis));
    }

    marker.pose.orientation.x = orientation.x();
    marker.pose.orientation.y = orientation.y();
    marker.pose.orientation.z = orientation.z();
    marker.pose.orientation.w = orientation.w();
  } else {
    // Default orientation
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
  }
}

void GraphMsfRos2::createContactMarker(const std::string& referenceFrameName, const rclcpp::Time& stamp, const Eigen::Vector3d& position,
                                       const std::string& nameSpace, const int id, visualization_msgs::msg::Marker& marker) {
  // Sphere
  marker.header.frame_id = referenceFrameName;
  marker.header.stamp = stamp;
  marker.ns = nameSpace;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // Scale and Color
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  // Position
  marker.pose.position.x = position(0);
  marker.pose.position.y = position(1);
  marker.pose.position.z = position(2);

  // Orientation
  marker.pose.orientation.w = 1.0;
}

long GraphMsfRos2::secondsSinceStart() {
  currentTime = std::chrono::high_resolution_clock::now();
  return std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime).count();
}

// Main IMU Callback
void GraphMsfRos2::imuCallback(const sensor_msgs::msg::Imu::SharedPtr imuMsgPtr) {
  GRAPH_MSF_SCOPED_TIMER("GraphMsfRos2::imuCallback");

  // Convert to Eigen
  Eigen::Vector3d linearAcc(imuMsgPtr->linear_acceleration.x, imuMsgPtr->linear_acceleration.y, imuMsgPtr->linear_acceleration.z);
  Eigen::Vector3d angularVel(imuMsgPtr->angular_velocity.x, imuMsgPtr->angular_velocity.y, imuMsgPtr->angular_velocity.z);
  Eigen::Matrix<double, 6, 1> addedImuMeasurements = Eigen::Matrix<double, 6, 1>::Zero();  // accel, gyro

  // Create pointer for carrying state
  std::shared_ptr<SafeIntegratedNavState> preIntegratedNavStatePtr = nullptr;
  std::shared_ptr<SafeNavStateWithCovarianceAndBias> optimizedStateWithCovarianceAndBiasPtr = nullptr;

  // Add measurement and get state
  if (GraphMsf::addCoreImuMeasurementAndGetState(linearAcc, angularVel,
                                                 imuMsgPtr->header.stamp.sec + 1e-9 * imuMsgPtr->header.stamp.nanosec,
                                                 preIntegratedNavStatePtr, optimizedStateWithCovarianceAndBiasPtr, addedImuMeasurements)) {
    // Publish Odometry
    this->publishState(preIntegratedNavStatePtr, optimizedStateWithCovarianceAndBiasPtr);
  } else if (GraphMsf::isGraphInited()) {
    RCLCPP_WARN(this->get_logger(), "Could not add IMU measurement.");
  }
}

// Publish state ---------------------------------------------------------------
// Higher Level Functions
void GraphMsfRos2::publishState(
    const std::shared_ptr<graph_msf::SafeIntegratedNavState>& integratedNavStatePtr,
    const std::shared_ptr<graph_msf::SafeNavStateWithCovarianceAndBias>& optimizedStateWithCovarianceAndBiasPtr) {
  GRAPH_MSF_SCOPED_TIMER("GraphMsfRos2::publishState");

  // Time
  const double timeK = integratedNavStatePtr->getTimeK();

  // Queue TF transforms data for separate thread processing
  {
    std::lock_guard<std::mutex> lock(tfTransformQueueMutex_);
    tfTransformQueue_.emplace(integratedNavStatePtr, optimizedStateWithCovarianceAndBiasPtr, timeK);
  }
  tfTransformQueueCondition_.notify_one();

  const bool needImuOdom =
      (pubEstOdomImu_->get_subscription_count() > 0) || (pubEstWorldImu_->get_subscription_count() > 0);

  // Add optimized path data at full rate if available (existing behavior)
  bool newOptimizedState = false;
  if (optimizedStateWithCovarianceAndBiasPtr != nullptr &&
      optimizedStateWithCovarianceAndBiasPtr->getTimeK() - lastOptimizedStateTimestamp_ > 1e-03) {
    newOptimizedState = true;
  }

  const bool needNonTimeCritical =
      newOptimizedState && optimizedStateWithCovarianceAndBiasPtr != nullptr &&
      ((pubEstWorldPosVariance_->get_subscription_count() > 0) ||
       (pubEstWorldRotVariance_->get_subscription_count() > 0) ||
       (pubAttitudeSigmas_->get_subscription_count() > 0) ||
       (pubEstOdomImuPath_->get_subscription_count() > 0) ||
       (pubEstWorldImuPath_->get_subscription_count() > 0) ||
       (pubOptWorldImuPath_->get_subscription_count() > 0) ||
       (pubOptWorldImu_->get_subscription_count() > 0) ||
       (pubAccelBias_->get_subscription_count() > 0) ||
       (pubGyroBias_->get_subscription_count() > 0) ||
       (pubHeadingUncertaintyMarkers_ != nullptr && pubHeadingUncertaintyMarkers_->get_subscription_count() > 0));

  // Covariances (only if needed)
  Eigen::Matrix<double, 6, 6> poseCovarianceRos, twistCovarianceRos;
  if (needImuOdom || needNonTimeCritical) {
    graph_msf::GraphMsfRos2::extractCovariancesFromOptimizedState(poseCovarianceRos, twistCovarianceRos,
                                                                  optimizedStateWithCovarianceAndBiasPtr);
  } else {
    poseCovarianceRos.setZero();
    twistCovarianceRos.setZero();
  }

  // Queue IMU odometry data for separate thread processing
  if (needImuOdom) {
    {
      std::lock_guard<std::mutex> lock(imuOdomQueueMutex_);
      imuOdomQueue_.emplace(integratedNavStatePtr, poseCovarianceRos, twistCovarianceRos);
    }
    imuOdomQueueCondition_.notify_one();
  }

  // Variances (only diagonal elements)
  Eigen::Vector3d positionVarianceRos = poseCovarianceRos.block<3, 3>(0, 0).diagonal();
  Eigen::Vector3d orientationVarianceRos = poseCovarianceRos.block<3, 3>(3, 3).diagonal();

  // Add path data at full rate (but don't publish yet)
  {
    std::lock_guard<std::mutex> lock(pathMsgMutex_);
    addToPathMsg(estOdomImuPathPtr_, staticTransformsPtr_->getOdomFrame(), rclcpp::Time(timeK * 1e9),
                 integratedNavStatePtr->getT_O_Ik_gravityAligned().translation(), graphConfigPtr_->imuBufferLength_ * 4);
    addToPathMsg(estWorldImuPathPtr_, staticTransformsPtr_->getWorldFrame(), rclcpp::Time(timeK * 1e9),
                 integratedNavStatePtr->getT_W_Ik().translation(), graphConfigPtr_->imuBufferLength_ * 4);
  }

  // Add optimized path data at full rate if available
  if (newOptimizedState) {
    {
      std::lock_guard<std::mutex> lock(pathMsgMutex_);
      addToPathMsg(optWorldImuPathPtr_, staticTransformsPtr_->getWorldFrame(),
                   rclcpp::Time(optimizedStateWithCovarianceAndBiasPtr->getTimeK() * 1e9),
                   optimizedStateWithCovarianceAndBiasPtr->getT_W_Ik().translation(), graphConfigPtr_->imuBufferLength_);
    }

    // Queue non-time critical data for non-time-critical thread processing
    if (needNonTimeCritical) {
      {
        std::lock_guard<std::mutex> lock(nonTimeCriticalQueueMutex_);
        nonTimeCriticalQueue_.emplace(poseCovarianceRos, twistCovarianceRos, positionVarianceRos, orientationVarianceRos,
                                      optimizedStateWithCovarianceAndBiasPtr);
      }
      nonTimeCriticalQueueCondition_.notify_one();
    }
    lastOptimizedStateTimestamp_ = optimizedStateWithCovarianceAndBiasPtr->getTimeK();
  }
}

// Publish to TF
void GraphMsfRos2::publishTfTransforms(const std::shared_ptr<graph_msf::SafeIntegratedNavState>& integratedNavStatePtr) {
  GRAPH_MSF_SCOPED_TIMER("GraphMsfRos2::publishTfTransforms");

  // Time
  const double& timeK = integratedNavStatePtr->getTimeK();  // Alias

  std::shared_lock<std::shared_mutex> lock(staticTransformsPtr_->mutex());

  // B_O
  Eigen::Isometry3d T_B_Ok =
      staticTransformsPtr_->rv_T_frame1_frame2(staticTransformsPtr_->getBaseLinkFrame(), staticTransformsPtr_->getImuFrame()) *
      integratedNavStatePtr->getT_O_Ik_gravityAligned().inverse();
  publishTfTreeTransform(staticTransformsPtr_->getBaseLinkFrame(), staticTransformsPtr_->getOdomFrame(), timeK, T_B_Ok);
  // O_W
  Eigen::Isometry3d T_O_W = integratedNavStatePtr->getT_W_O().inverse();
  publishTfTreeTransform(staticTransformsPtr_->getOdomFrame(), staticTransformsPtr_->getWorldFrame(), timeK, T_O_W);

  // Publish Velocity Markers
  publishVelocityMarkers(integratedNavStatePtr);
}

// Copy the arguments in order to be thread safe
void GraphMsfRos2::publishNonTimeCriticalData(
    const Eigen::Matrix<double, 6, 6> poseCovarianceRos, const Eigen::Matrix<double, 6, 6> twistCovarianceRos,
    const Eigen::Vector3d positionVarianceRos, const Eigen::Vector3d orientationVarianceRos,
    const std::shared_ptr<const graph_msf::SafeNavStateWithCovarianceAndBias> optimizedStateWithCovarianceAndBiasPtr) {
  GRAPH_MSF_SCOPED_TIMER("GraphMsfRos2::publishNonTimeCriticalData");

  // Mutex for not overloading ROS
  std::lock_guard<std::mutex> lock(rosPublisherMutex_);

  // Publish Variances
  publishDiagVarianceVectors(positionVarianceRos, orientationVarianceRos, optimizedStateWithCovarianceAndBiasPtr->getTimeK());

  // Publish exact roll/pitch/yaw sigmas from the fused pose marginal.
  publishAttitudeSigmas(optimizedStateWithCovarianceAndBiasPtr);

  // Publish paths (data was already added at full rate in publishState)
  publishImuPaths();

  // Publish optimized path (data was already added at full rate in publishState)
  if (pubOptWorldImuPath_->get_subscription_count() > 0) {
    std::lock_guard<std::mutex> lockPath(pathMsgMutex_);
    pubOptWorldImuPath_->publish(*optWorldImuPathPtr_);
  }

  // Publish heading uncertainty visualization in the world frame.
  publishHeadingUncertaintyMarkers(optimizedStateWithCovarianceAndBiasPtr);

  // Optimized estimate ----------------------
  publishOptimizedStateAndBias(optimizedStateWithCovarianceAndBiasPtr, poseCovarianceRos, twistCovarianceRos);
}

void GraphMsfRos2::publishOptimizedStateAndBias(
    const std::shared_ptr<const graph_msf::SafeNavStateWithCovarianceAndBias> optimizedStateWithCovarianceAndBiasPtr,
    const Eigen::Matrix<double, 6, 6>& poseCovarianceRos, const Eigen::Matrix<double, 6, 6>& twistCovarianceRos) {
  GRAPH_MSF_SCOPED_TIMER("GraphMsfRos2::publishOptimizedStateAndBias");

  // A: Publish Odometry and IMU Biases at update rate
  if (optimizedStateWithCovarianceAndBiasPtr != nullptr) {
    // Odometry messages
    // world->imu
    if (pubOptWorldImu_->get_subscription_count() > 0) {
      addToOdometryMsg(optWorldImuMsgPtr_, staticTransformsPtr_->getWorldFrame(), staticTransformsPtr_->getImuFrame(),
                       rclcpp::Time(optimizedStateWithCovarianceAndBiasPtr->getTimeK() * 1e9),
                       optimizedStateWithCovarianceAndBiasPtr->getT_W_Ik(), optimizedStateWithCovarianceAndBiasPtr->getI_v_W_I(),
                       optimizedStateWithCovarianceAndBiasPtr->getI_w_W_I(), poseCovarianceRos, twistCovarianceRos);
      pubOptWorldImu_->publish(*optWorldImuMsgPtr_);
    }

    // Biases
    // Publish accel bias
    if (pubAccelBias_->get_subscription_count() > 0) {
      Eigen::Vector3d accelBias = optimizedStateWithCovarianceAndBiasPtr->getAccelerometerBias();
      accelBiasMsgPtr_->header.stamp = rclcpp::Time(optimizedStateWithCovarianceAndBiasPtr->getTimeK() * 1e9);
      accelBiasMsgPtr_->header.frame_id = staticTransformsPtr_->getImuFrame();
      accelBiasMsgPtr_->vector.x = accelBias(0);
      accelBiasMsgPtr_->vector.y = accelBias(1);
      accelBiasMsgPtr_->vector.z = accelBias(2);
      pubAccelBias_->publish(*accelBiasMsgPtr_);
    }
    // Publish gyro bias
    if (pubGyroBias_->get_subscription_count() > 0) {
      Eigen::Vector3d gyroBias = optimizedStateWithCovarianceAndBiasPtr->getGyroscopeBias();
      gyroBiasMsgPtr_->header.stamp = rclcpp::Time(optimizedStateWithCovarianceAndBiasPtr->getTimeK() * 1e9);
      gyroBiasMsgPtr_->header.frame_id = staticTransformsPtr_->getImuFrame();
      gyroBiasMsgPtr_->vector.x = gyroBias(0);
      gyroBiasMsgPtr_->vector.y = gyroBias(1);
      gyroBiasMsgPtr_->vector.z = gyroBias(2);
      pubGyroBias_->publish(*gyroBiasMsgPtr_);
    }
  }  // Publishing odometry and biases
}

// Publish TF transforms for optimized state at full rate
void GraphMsfRos2::publishOptimizedStateTfTransforms(
    const std::shared_ptr<const graph_msf::SafeNavStateWithCovarianceAndBias> optimizedStateWithCovarianceAndBiasPtr,
    const double timeStamp) {
  GRAPH_MSF_SCOPED_TIMER("GraphMsfRos2::publishOptimizedStateTfTransforms");

  // Publish Transforms at imu rate
  if (optimizedStateWithCovarianceAndBiasPtr != nullptr && timeStamp - lastIntegratedStateTimestamp_ > 1e-03) {
    lastIntegratedStateTimestamp_ = timeStamp;
    // TFs in Optimized State
    for (const auto& framePairTransformMapIterator :
         optimizedStateWithCovarianceAndBiasPtr->getReferenceFrameTransforms().getTransformsMap()) {
      // Case 1: If includes world frame --> everything child of world
      if (framePairTransformMapIterator.first.first == staticTransformsPtr_->getWorldFrame() ||
          framePairTransformMapIterator.first.second == staticTransformsPtr_->getWorldFrame()) {
        // Get transform
        Eigen::Isometry3d T_W_M;
        std::string mapFrameName;
        const std::string& worldFrameName = staticTransformsPtr_->getWorldFrame();

        // If world is second, then map is first
        if (framePairTransformMapIterator.first.second == worldFrameName) {
          T_W_M = framePairTransformMapIterator.second.inverse();
          mapFrameName = framePairTransformMapIterator.first.first;
        }
        // If world is second, then map is first, this is the case for holistic alignment
        else {
          T_W_M = framePairTransformMapIterator.second;
          mapFrameName = framePairTransformMapIterator.first.second;
        }

        // Publish TF Tree only (removed PoseWithCovarianceStamped)
        publishTfTreeTransform(worldFrameName, mapFrameName + referenceFrameAlignedNameId, timeStamp, T_W_M);
      }
      // Case 2: Other transformations
      else {
        const std::string& sensorFrameName = framePairTransformMapIterator.first.first;
        const std::string& sensorFrameNameCorrected = framePairTransformMapIterator.first.second;
        const Eigen::Isometry3d& T_sensor_sensorCorrected = framePairTransformMapIterator.second;

        // Publish TF Tree only (removed PoseWithCovarianceStamped)
        publishTfTreeTransform(sensorFrameName, sensorFrameNameCorrected, timeStamp, T_sensor_sensorCorrected);
      }
    }  // for each frame pair transform
  }    // Publishing of Transforms
}

// Lower Level Functions
void GraphMsfRos2::publishTfTreeTransform(const std::string& parentFrameName, const std::string& childFrameName, const double timeStamp,
                                          const Eigen::Isometry3d& T_frame_childFrame) const {
  GRAPH_MSF_SCOPED_TIMER("GraphMsfRos2::publishTfTreeTransform");

  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.stamp = rclcpp::Time(timeStamp * 1e9);
  transformStamped.header.frame_id = parentFrameName;
  transformStamped.child_frame_id = childFrameName;
  transformStamped.transform.translation.x = T_frame_childFrame.translation().x();
  transformStamped.transform.translation.y = T_frame_childFrame.translation().y();
  transformStamped.transform.translation.z = T_frame_childFrame.translation().z();
  Eigen::Quaterniond q(T_frame_childFrame.rotation());
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  tfBroadcaster_->sendTransform(transformStamped);
}

void GraphMsfRos2::publishImuOdoms(const std::shared_ptr<const graph_msf::SafeIntegratedNavState>& preIntegratedNavStatePtr,
                                   const Eigen::Matrix<double, 6, 6>& poseCovarianceRos,
                                   const Eigen::Matrix<double, 6, 6>& twistCovarianceRos) const {
  GRAPH_MSF_SCOPED_TIMER("GraphMsfRos2::publishImuOdoms");

  // Odom->imu
  if (pubEstOdomImu_->get_subscription_count() > 0) {
    addToOdometryMsg(estOdomImuMsgPtr_, staticTransformsPtr_->getOdomFrame(), staticTransformsPtr_->getImuFrame(),
                     rclcpp::Time(preIntegratedNavStatePtr->getTimeK() * 1e9), preIntegratedNavStatePtr->getT_O_Ik_gravityAligned(),
                     preIntegratedNavStatePtr->getI_v_W_I(), preIntegratedNavStatePtr->getI_w_W_I(), poseCovarianceRos, twistCovarianceRos);
    pubEstOdomImu_->publish(*estOdomImuMsgPtr_);
  }
  // World->imu
  if (pubEstWorldImu_->get_subscription_count() > 0) {
    addToOdometryMsg(estWorldImuMsgPtr_, staticTransformsPtr_->getWorldFrame(), staticTransformsPtr_->getImuFrame(),
                     rclcpp::Time(preIntegratedNavStatePtr->getTimeK() * 1e9), preIntegratedNavStatePtr->getT_W_Ik(),
                     preIntegratedNavStatePtr->getI_v_W_I(), preIntegratedNavStatePtr->getI_w_W_I(), poseCovarianceRos, twistCovarianceRos);
    pubEstWorldImu_->publish(*estWorldImuMsgPtr_);
  }
}

void GraphMsfRos2::publishDiagVarianceVectors(const Eigen::Vector3d& posVarianceRos, const Eigen::Vector3d& rotVarianceRos,
                                              const double timeStamp) const {
  GRAPH_MSF_SCOPED_TIMER("GraphMsfRos2::publishDiagVarianceVectors");

  // World Position Variance
  if (pubEstWorldPosVariance_->get_subscription_count() > 0) {
    estWorldPosVarianceMsgPtr_->header.stamp = rclcpp::Time(timeStamp * 1e9);
    estWorldPosVarianceMsgPtr_->header.frame_id = staticTransformsPtr_->getWorldFrame();
    estWorldPosVarianceMsgPtr_->vector.x = posVarianceRos(0);
    estWorldPosVarianceMsgPtr_->vector.y = posVarianceRos(1);
    estWorldPosVarianceMsgPtr_->vector.z = posVarianceRos(2);
    pubEstWorldPosVariance_->publish(*estWorldPosVarianceMsgPtr_);
  }
  // World Rotation Variance
  if (pubEstWorldRotVariance_->get_subscription_count() > 0) {
    estWorldRotVarianceMsgPtr_->header.stamp = rclcpp::Time(timeStamp * 1e9);
    estWorldRotVarianceMsgPtr_->header.frame_id = staticTransformsPtr_->getWorldFrame();
    estWorldRotVarianceMsgPtr_->vector.x = rotVarianceRos(0);
    estWorldRotVarianceMsgPtr_->vector.y = rotVarianceRos(1);
    estWorldRotVarianceMsgPtr_->vector.z = rotVarianceRos(2);
    pubEstWorldRotVariance_->publish(*estWorldRotVarianceMsgPtr_);
  }
}

void GraphMsfRos2::publishAttitudeSigmas(
    const std::shared_ptr<const graph_msf::SafeNavStateWithCovarianceAndBias>& optimizedStateWithCovarianceAndBiasPtr) {
  GRAPH_MSF_SCOPED_TIMER("GraphMsfRos2::publishAttitudeSigmas");

  if (pubAttitudeSigmas_->get_subscription_count() == 0 || optimizedStateWithCovarianceAndBiasPtr == nullptr ||
      graphMgrPtr_ == nullptr) {
    return;
  }

  gtsam::Pose3 T_W_I_fusedMarginal = gtsam::Pose3::Identity();
  gtsam::Matrix66 fusedPoseCovarianceLeftWorld = gtsam::Z_6x6;
  if (!graphMgrPtr_->calculateCurrentPoseMarginalInWorld(T_W_I_fusedMarginal, fusedPoseCovarianceLeftWorld)) {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                          "Attitude sigma publish skipped because the current pose marginal was unavailable this cycle.");
    return;
  }

  std::shared_lock<std::shared_mutex> transformsLock(staticTransformsPtr_->mutex());
  const std::string imuFrameName = staticTransformsPtr_->getImuFrame();
  const std::string baseLinkFrameName = staticTransformsPtr_->getBaseLinkFrame();
  const gtsam::Pose3 T_I_B(staticTransformsPtr_->rv_T_frame1_frame2(imuFrameName, baseLinkFrameName).matrix());
  transformsLock.unlock();

  // Right-composing the fixed IMU->base extrinsic preserves the left/world tangent covariance convention.
  const gtsam::Pose3 T_W_B_fusedMarginal = T_W_I_fusedMarginal.compose(T_I_B);
  const AttitudeSigmaEstimate attitudeEstimate =
      computeAttitudeMeanSigmasNumeric(T_W_B_fusedMarginal, fusedPoseCovarianceLeftWorld, PoseTangentFrame::kLeftWorld);
  if (!attitudeEstimate.valid) {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                          "Attitude sigma publish skipped because roll/pitch/yaw was ill-defined this cycle.");
    return;
  }

  constexpr double kRadToDeg = 180.0 / M_PI;

  // Overload TwistStamped.angular with exact attitude sigmas [deg] of T_W_base_link.
  attitudeSigmasMsgPtr_->header.stamp = timeFromSec(optimizedStateWithCovarianceAndBiasPtr->getTimeK());
  attitudeSigmasMsgPtr_->header.frame_id = baseLinkFrameName;
  attitudeSigmasMsgPtr_->twist.linear.x = 0.0;
  attitudeSigmasMsgPtr_->twist.linear.y = 0.0;
  attitudeSigmasMsgPtr_->twist.linear.z = 0.0;
  attitudeSigmasMsgPtr_->twist.angular.x = attitudeEstimate.rollSigmaRad * kRadToDeg;
  attitudeSigmasMsgPtr_->twist.angular.y = attitudeEstimate.pitchSigmaRad * kRadToDeg;
  attitudeSigmasMsgPtr_->twist.angular.z = attitudeEstimate.yawSigmaRad * kRadToDeg;
  pubAttitudeSigmas_->publish(*attitudeSigmasMsgPtr_);
}

void GraphMsfRos2::publishVelocityMarkers(const std::shared_ptr<const graph_msf::SafeIntegratedNavState>& navStatePtr) const {
  GRAPH_MSF_SCOPED_TIMER("GraphMsfRos2::publishVelocityMarkers");

  const bool pubLin = (pubLinVelocityMarker_->get_subscription_count() > 0);
  const bool pubAng = (pubAngularVelocityMarker_->get_subscription_count() > 0);
  if (!pubLin && !pubAng) {
    return;
  }

  // Linear Velocity Marker
  if (pubLin) {
    visualization_msgs::msg::Marker velocityMarker;
    createLinVelocityMarker(staticTransformsPtr_->getImuFrame(), rclcpp::Time(navStatePtr->getTimeK() * 1e9), navStatePtr->getI_v_W_I(),
                            velocityMarker);
    pubLinVelocityMarker_->publish(velocityMarker);
  }

  // Angular Velocity Marker
  if (pubAng) {
    visualization_msgs::msg::Marker angularVelocityMarker;
    createAngularVelocityMarker(staticTransformsPtr_->getImuFrame(), rclcpp::Time(navStatePtr->getTimeK() * 1e9), navStatePtr->getI_w_W_I(),
                                navStatePtr->getT_W_Ik(), angularVelocityMarker);
    pubAngularVelocityMarker_->publish(angularVelocityMarker);
  }
}

void GraphMsfRos2::validateHeadingUncertaintyTransformsOrThrow() const {
  if (!publishHeadingUncertaintyMarkersFlag_) {
    return;
  }

  std::shared_lock<std::shared_mutex> transformsLock(staticTransformsPtr_->mutex());
  const std::string& imuFrameName = staticTransformsPtr_->getImuFrame();
  const std::string& baseLinkFrameName = staticTransformsPtr_->getBaseLinkFrame();

  if (baseLinkFrameName.empty()) {
    throw std::runtime_error(
        "Heading uncertainty markers require a non-empty base_link frame name because the visualization is robot-centered.");
  }

  if (!staticTransformsPtr_->isFramePairInDictionary(imuFrameName, baseLinkFrameName)) {
    throw std::runtime_error("Heading uncertainty markers require the static transform " + imuFrameName + " -> " +
                             baseLinkFrameName + " to exist. The visualization is defined in the robot/base frame and "
                             "must not fall back to the IMU frame.");
  }
}

void GraphMsfRos2::setHeadingUncertaintyFixedFrame(const std::string& sourceId, const std::string& fixedFrameName) {
  std::lock_guard<std::mutex> lock(headingUncertaintyFixedFrameMutex_);
  if (fixedFrameName.empty()) {
    headingUncertaintyFixedFrames_.erase(sourceId);
  } else {
    headingUncertaintyFixedFrames_[sourceId] = fixedFrameName;
  }
}

void GraphMsfRos2::publishHeadingUncertaintyMarkers(
    const std::shared_ptr<const graph_msf::SafeNavStateWithCovarianceAndBias>& optimizedStateWithCovarianceAndBiasPtr) {
  GRAPH_MSF_SCOPED_TIMER("GraphMsfRos2::publishHeadingUncertaintyMarkers");

  if (!publishHeadingUncertaintyMarkersFlag_ || pubHeadingUncertaintyMarkers_ == nullptr ||
      pubHeadingUncertaintyMarkers_->get_subscription_count() == 0) {
    return;
  }

  validateHeadingUncertaintyTransformsOrThrow();

  std::shared_lock<std::shared_mutex> transformsLock(staticTransformsPtr_->mutex());
  visualization_msgs::msg::MarkerArray markerArray;
  const double timeStamp = optimizedStateWithCovarianceAndBiasPtr != nullptr ? optimizedStateWithCovarianceAndBiasPtr->getTimeK() : 0.0;
  const rclcpp::Time stamp = timeFromSec(timeStamp);
  const std::string& worldFrameName = staticTransformsPtr_->getWorldFrame();
  const std::string& imuFrameName = staticTransformsPtr_->getImuFrame();
  const std::string& baseLinkFrameName = staticTransformsPtr_->getBaseLinkFrame();
  std::map<std::string, std::string> selectedFixedFrames;
  {
    std::lock_guard<std::mutex> lock(headingUncertaintyFixedFrameMutex_);
    selectedFixedFrames = headingUncertaintyFixedFrames_;
  }

  // Dedicated topic: clearing first keeps the visualization consistent even when alignment frame sets change over time.
  visualization_msgs::msg::Marker clearMarker;
  clearMarker.header.frame_id = worldFrameName;
  clearMarker.header.stamp = stamp;
  clearMarker.action = visualization_msgs::msg::Marker::DELETEALL;
  markerArray.markers.push_back(clearMarker);

  if (optimizedStateWithCovarianceAndBiasPtr == nullptr) {
    pubHeadingUncertaintyMarkers_->publish(markerArray);
    return;
  }

  const gtsam::Pose3 T_W_I_snapshot(optimizedStateWithCovarianceAndBiasPtr->getT_W_Ik().matrix());
  const gtsam::Pose3 T_I_B(staticTransformsPtr_->rv_T_frame1_frame2(imuFrameName, baseLinkFrameName).matrix());
  gtsam::Pose3 T_W_robot = T_W_I_snapshot.compose(T_I_B);
  double robotHeadingYawRad = 0.0;

  // Use the same projected-heading definition as the uncertainty Jacobian. This avoids spurious
  // rejection when roll/pitch are large but the robot still has a well-defined horizontal heading.
  if (!extractWorldHeading(T_W_robot, robotHeadingYawRad)) {
    pubHeadingUncertaintyMarkers_->publish(markerArray);
    return;
  }

  // Query the exact current fused pose marginal in world coordinates. This is the correct posterior quantity for
  // the robot's current world-yaw uncertainty because the current pose marginal already contains the effect of the
  // alignment states and all cross-correlations in the fixed-lag posterior.
  HeadingSigmaEstimate fusedRobotHeadingEstimate;
  gtsam::Pose3 T_W_I_fusedMarginal = gtsam::Pose3::Identity();
  gtsam::Matrix66 fusedPoseCovarianceLeftWorld = gtsam::Z_6x6;
  if (graphMgrPtr_ != nullptr &&
      graphMgrPtr_->calculateCurrentPoseMarginalInWorld(T_W_I_fusedMarginal, fusedPoseCovarianceLeftWorld)) {
    const gtsam::Pose3 T_W_B_fusedMarginal = T_W_I_fusedMarginal.compose(T_I_B);
    fusedRobotHeadingEstimate =
        computeYawMeanSigmaNumeric(T_W_B_fusedMarginal, fusedPoseCovarianceLeftWorld, PoseTangentFrame::kLeftWorld);
    if (fusedRobotHeadingEstimate.valid) {
      T_W_robot = T_W_B_fusedMarginal;
      robotHeadingYawRad = fusedRobotHeadingEstimate.yawRad;
    }
  }

  const Eigen::Vector3d robotPositionWorld = T_W_robot.translation();
  const std::string robotHeadingLabel = "base";

  const auto makeMarker = [&](const std::string& nameSpace, const int id, const int type) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = worldFrameName;
    marker.header.stamp = stamp;
    marker.ns = nameSpace;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(0.0);
    marker.frame_locked = false;
    return marker;
  };

  const std::string markerNs = "heading_uncertainty_disk";

  bool robotContextMarkersAdded = false;
  auto ensureRobotContextMarkers = [&]() {
    if (robotContextMarkersAdded) {
      return;
    }

    visualization_msgs::msg::Marker diskOutlineMarker =
        makeMarker(markerNs, 0, visualization_msgs::msg::Marker::LINE_STRIP);
    diskOutlineMarker.scale.x = 0.04;
    diskOutlineMarker.color.r = 0.85F;
    diskOutlineMarker.color.g = 0.85F;
    diskOutlineMarker.color.b = 0.85F;
    diskOutlineMarker.color.a = 0.70F;
    fillHeadingUncertaintyDiskOutlineMarker(diskOutlineMarker, robotPositionWorld, headingUncertaintyDiskRadius_,
                                            headingUncertaintyZOffset_);
    markerArray.markers.push_back(diskOutlineMarker);

    visualization_msgs::msg::Marker arrowMarker =
        makeMarker(markerNs, 3, visualization_msgs::msg::Marker::ARROW);
    arrowMarker.scale.x = 0.08;
    arrowMarker.scale.y = 0.14;
    arrowMarker.scale.z = 0.16;
    arrowMarker.color.r = 0.15F;
    arrowMarker.color.g = 0.90F;
    arrowMarker.color.b = 0.35F;
    arrowMarker.color.a = 0.98F;
    fillHeadingArrowMarkerPoints(arrowMarker, robotPositionWorld, robotHeadingYawRad,
                                 headingUncertaintyDiskRadius_, headingUncertaintyZOffset_);
    markerArray.markers.push_back(arrowMarker);

    robotContextMarkersAdded = true;
  };

  auto appendHeadingUncertaintySector = [&](const std::string& sourceId, const std::string& labelSuffix, const double sigmaRad,
                                            int& sourceIndex) {
    ensureRobotContextMarkers();

    const HeadingMarkerStyle style = makeHeadingMarkerStyle(sourceId);
    const std::string sourceMarkerNs = markerNs + "_" + style.markerSuffix;

    visualization_msgs::msg::Marker sectorFillMarker =
        makeMarker(sourceMarkerNs, 10 * sourceIndex + 1, visualization_msgs::msg::Marker::TRIANGLE_LIST);
    sectorFillMarker.scale.x = 1.0;
    sectorFillMarker.scale.y = 1.0;
    sectorFillMarker.scale.z = 1.0;
    sectorFillMarker.color = style.fillColor;
    fillHeadingUncertaintyDiskMarker(sectorFillMarker, robotPositionWorld, robotHeadingYawRad,
                                     sigmaRad, headingUncertaintyDiskRadius_,
                                     headingUncertaintyNSigmas_, headingUncertaintyZOffset_);
    markerArray.markers.push_back(sectorFillMarker);

    visualization_msgs::msg::Marker sectorBoundaryMarker =
        makeMarker(sourceMarkerNs, 10 * sourceIndex + 2, visualization_msgs::msg::Marker::LINE_STRIP);
    sectorBoundaryMarker.scale.x = 0.06;
    sectorBoundaryMarker.color = style.boundaryColor;
    fillHeadingUncertaintySectorBoundaryMarker(sectorBoundaryMarker, robotPositionWorld, robotHeadingYawRad,
                                               sigmaRad, headingUncertaintyDiskRadius_,
                                               headingUncertaintyNSigmas_, headingUncertaintyZOffset_);
    markerArray.markers.push_back(sectorBoundaryMarker);

    if (publishHeadingUncertaintyTextFlag_) {
      visualization_msgs::msg::Marker textMarker =
          makeMarker(sourceMarkerNs, 10 * sourceIndex + 3, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
      textMarker.pose.position.x = robotPositionWorld.x();
      textMarker.pose.position.y = robotPositionWorld.y();
      textMarker.pose.position.z = robotPositionWorld.z() + headingUncertaintyZOffset_ + 0.22 + 0.18 * sourceIndex;
      textMarker.pose.orientation.w = 1.0;
      textMarker.scale.z = 0.24;
      textMarker.color = style.textColor;
      textMarker.text = formatHeadingSigmaLabel(robotHeadingLabel + " " + style.sourceLabel + " " + labelSuffix, sigmaRad);
      markerArray.markers.push_back(textMarker);
    }

    ++sourceIndex;
  };

  // 2) Exact fused robot world-yaw uncertainty from the current pose marginal.
  int sourceIndex = 0;
  if (fusedRobotHeadingEstimate.valid) {
    appendHeadingUncertaintySector("fused", "world_yaw", fusedRobotHeadingEstimate.sigmaRad, sourceIndex);
  }

  // 3) One colored sector per active alignment source. Each source uses its own targeted one-key marginal query.
  for (const auto& selectedFrameEntry : selectedFixedFrames) {
    const std::string& sourceId = selectedFrameEntry.first;
    const std::string& selectedFixedFrame = selectedFrameEntry.second;
    if (selectedFixedFrame.empty()) {
      continue;
    }

    // The reference-frame publication path later adds a translation-only keyframe correction, but yaw depends only on
    // the rotation of T_W_F, so the raw optimizer pose estimate is the correct input for this angular diagnostic.
    gtsam::Pose3 T_W_selectedFixedFrame = gtsam::Pose3::Identity();
    gtsam::Matrix66 selectedAlignmentCovariance = gtsam::Z_6x6;
    if (graphMgrPtr_ == nullptr ||
        !graphMgrPtr_->calculateActiveReferenceFramePoseMarginalInWorld(selectedFixedFrame, T_W_selectedFixedFrame,
                                                                        selectedAlignmentCovariance)) {
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "Heading uncertainty marker skipped source '%s' frame '%s' because the marginal was unavailable this cycle.",
                            sourceId.c_str(), selectedFixedFrame.c_str());
      continue;
    }

    const HeadingSigmaEstimate alignmentDiskEstimate =
        computeYawMeanSigmaNumeric(T_W_selectedFixedFrame, selectedAlignmentCovariance, PoseTangentFrame::kRightLocal);
    if (!alignmentDiskEstimate.valid) {
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "Heading uncertainty marker skipped source '%s' frame '%s' because heading was ill-defined this cycle.",
                            sourceId.c_str(), selectedFixedFrame.c_str());
      continue;
    }

    appendHeadingUncertaintySector(sourceId, "align[" + selectedFixedFrame + "]", alignmentDiskEstimate.sigmaRad, sourceIndex);
  }

  pubHeadingUncertaintyMarkers_->publish(markerArray);
}

void GraphMsfRos2::publishImuPaths() const {
  GRAPH_MSF_SCOPED_TIMER("GraphMsfRos2::publishImuPaths");

  // Only publish the accumulated path data (data was already added in publishState at full rate)
  std::lock_guard<std::mutex> lock(pathMsgMutex_);

  // odom->imu
  if (pubEstOdomImuPath_->get_subscription_count() > 0) {
    pubEstOdomImuPath_->publish(*estOdomImuPathPtr_);
  }
  // world->imu
  if (pubEstWorldImuPath_->get_subscription_count() > 0) {
    pubEstWorldImuPath_->publish(*estWorldImuPathPtr_);
  }
}

void GraphMsfRos2::nonTimeCriticalThreadFunction() {
  // Rate limiting variables
  while (!shutdownRequested_) {
    std::unique_lock<std::mutex> lock(nonTimeCriticalQueueMutex_);

    // Wait for work or shutdown signal
    nonTimeCriticalQueueCondition_.wait(lock, [this] { return !nonTimeCriticalQueue_.empty() || shutdownRequested_; });

    // Process all queued work with rate limiting
    while (!nonTimeCriticalQueue_.empty() && !shutdownRequested_) {
      // Get the data from the queue
      NonTimeCriticalData data = std::move(nonTimeCriticalQueue_.front());
      nonTimeCriticalQueue_.pop();

      // Unlock while processing to avoid blocking the main thread
      lock.unlock();

      {
        GRAPH_MSF_SCOPED_TIMER("GraphMsfRos2::nonTimeCriticalThreadFunction.process");
        // Process the non-time-critical data
        publishNonTimeCriticalData(data.poseCovarianceRos, data.twistCovarianceRos, data.positionVarianceRos, data.orientationVarianceRos,
                                   data.optimizedStateWithCovarianceAndBiasPtr);
      }

      // Re-lock for the next iteration
      lock.lock();
    }
  }
}

void GraphMsfRos2::imuOdomThreadFunction() {
  // Process IMU odometry publishing at high rate
  while (!shutdownRequested_) {
    std::unique_lock<std::mutex> lock(imuOdomQueueMutex_);

    // Wait for work or shutdown signal
    imuOdomQueueCondition_.wait(lock, [this] { return !imuOdomQueue_.empty() || shutdownRequested_; });

    // Process all queued IMU odometry data
    while (!imuOdomQueue_.empty() && !shutdownRequested_) {
      // Get the data from the queue
      ImuOdomData data = std::move(imuOdomQueue_.front());
      imuOdomQueue_.pop();

      // Unlock while processing to avoid blocking the main thread
      lock.unlock();

      {
        GRAPH_MSF_SCOPED_TIMER("GraphMsfRos2::imuOdomThreadFunction.process");
        // Publish IMU odometry messages
        publishImuOdoms(data.integratedNavStatePtr, data.poseCovarianceRos, data.twistCovarianceRos);
      }

      // Re-lock for the next iteration
      lock.lock();
    }
  }
}

void GraphMsfRos2::tfTransformThreadFunction() {
  // Process TF transforms publishing at high rate
  while (!shutdownRequested_) {
    std::unique_lock<std::mutex> lock(tfTransformQueueMutex_);

    // Wait for work or shutdown signal
    tfTransformQueueCondition_.wait(lock, [this] { return !tfTransformQueue_.empty() || shutdownRequested_; });

    // Process all queued TF transform data
    while (!tfTransformQueue_.empty() && !shutdownRequested_) {
      // Get the data from the queue
      TfTransformData data = std::move(tfTransformQueue_.front());
      tfTransformQueue_.pop();

      // Unlock while processing to avoid blocking the main thread
      lock.unlock();

      {
        GRAPH_MSF_SCOPED_TIMER("GraphMsfRos2::tfTransformThreadFunction.process");
        // Publish TF transforms
        publishTfTransforms(data.integratedNavStatePtr);
        publishOptimizedStateTfTransforms(data.optimizedStateWithCovarianceAndBiasPtr, data.timeK);
      }

      // Re-lock for the next iteration
      lock.lock();
    }
  }
}

}  // namespace graph_msf
