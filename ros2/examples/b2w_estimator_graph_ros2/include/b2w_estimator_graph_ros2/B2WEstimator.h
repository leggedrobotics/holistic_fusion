/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#pragma once

// std
#include <atomic>
#include <chrono>
#include <memory>
#include <deque>
#include <cstddef>
#include <cmath>
#include <limits>
#include <mutex>
#include <utility>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// ROS 2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

// Workspace
#include "graph_msf/gnss/GnssHandler.h"
#include "graph_msf/gnss/GnssCovariance.h"
#include "graph_msf/measurements/UnaryMeasurementXD.h"
#include "graph_msf/trajectory_alignment/TrajectoryAlignmentHandler.h"
#include "graph_msf_ros2/GraphMsfRos2.h"
#include "b2w_estimator_graph_ros2/HfB2wWheelOdometry.h"

// Defined Macros
#define ROS_QUEUE_SIZE 1
#define NUM_GNSS_CALLBACKS_UNTIL_START 20  // 0

namespace b2w_se {

class B2WEstimator : public graph_msf::GraphMsfRos2 {
 public:
  explicit B2WEstimator(const std::string& node_name,
                        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  ~B2WEstimator() override = default;

  void setup(const rclcpp::Node::SharedPtr& self);

 protected:
  // Virtual Functions
  void readParams();
  void initializePublishers();
  void initializeSubscribers();
  void initializeMessages();
  void initializeServices();

 private:
  // Time
  std::chrono::time_point<std::chrono::high_resolution_clock> startTime_;
  std::chrono::time_point<std::chrono::high_resolution_clock> currentTime_;

  // Config -------------------------------------

  // Rates
  double gnssRate_ = 1.0;
  double lioBetweenOdometryRate_ = 10.0;
  double lioOdometryRate_ = 10.0;
  double vioOdometryRate_ = 10.0;
  double vioOdometryBetweenRate_ = 25.0;

  // Alignment Parameters
  Eigen::Matrix<double, 6, 1> initialSe3AlignmentNoise_ = 1.0 * Eigen::Matrix<double, 6, 1>::Ones();
  Eigen::Matrix<double, 6, 1> lioSe3AlignmentRandomWalk_ = 0.0 * Eigen::Matrix<double, 6, 1>::Ones();
  Eigen::Matrix<double, 6, 1> vioSe3AlignmentRandomWalk_ = 0.0 * Eigen::Matrix<double, 6, 1>::Ones();

  // Noise
  double gnssPositionOutlierThreshold_ = 1.0;
  // LiDAR Odometry
  Eigen::Matrix<double, 6, 1> lioPoseUnaryNoise_ = 100.0 * Eigen::Matrix<double, 6, 1>::Ones();
  // Robust kernel applied to the LIO/VIO unary (and VIO between) factors: none | huber | cauchy | tukey | gemanmcclure | dcs
  std::string lioVioRobustNormType_ = "dcs";
  double lioVioRobustNormConstant_ = 1.0;
  graph_msf::RobustNorm lioVioRobustNorm() const;
  // Robust kernel on the GNSS position unary: legacy Huber(0.25)
  std::string gnssRobustNormType_ = "huber";
  double gnssRobustNormConstant_ = 0.25;
  graph_msf::RobustNorm gnssRobustNorm() const;
  static graph_msf::RobustNorm makeRobustNorm_(const std::string& type, double c, const char* paramName);
  Eigen::Matrix<double, 6, 1> lioBetweenNoise_ = 100.0 * Eigen::Matrix<double, 6, 1>::Ones();
  // VIO Odometry
  Eigen::Matrix<double, 6, 1> vioPoseBetweenNoise_ = 100.0 * Eigen::Matrix<double, 6, 1>::Ones();
  Eigen::Matrix<double, 6, 1> vioPoseUnaryNoise_ = 100.0 * Eigen::Matrix<double, 6, 1>::Ones();

  // ROS Related stuff ----------------------------

  // Callbacks
  void lidarOdometryCallback_(const nav_msgs::msg::Odometry::ConstSharedPtr& lidarOdomPtr);
  void lidarDegeneracyCallback_(const std_msgs::msg::Bool::ConstSharedPtr& degeneracyPtr);
  void lidarBetweenOdometryCallback_(const nav_msgs::msg::Odometry::ConstSharedPtr& lidarBetweenOdomPtr);
  void gnssNavSatFixCallback_(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& navSatFixPtr);
  void vioOdometryCallback_(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& vioOdomPtr);
  void vioOdometryBetweenCallback_(const nav_msgs::msg::Odometry::ConstSharedPtr& vioOdomPtr);
  void publishAlignmentStatus_();

  class FrequencyChecker {
   public:
    FrequencyChecker(std::size_t window_size = 40, double report_period_s = 1.0)
        : window_size_(window_size), report_period_s_(report_period_s) {}

    bool tick(double t_s) {
      if (!std::isfinite(t_s)) return false;

      time_window_.push_back(t_s);
      if (time_window_.size() > window_size_) time_window_.pop_front();

      last_window_n_ = time_window_.size();
      last_hz_ = 0.0;
      if (last_window_n_ >= 2) {
        const double dt = time_window_.back() - time_window_.front();
        if (dt > 1e-9) last_hz_ = static_cast<double>(last_window_n_ - 1) / dt;
      }

      if (last_report_t_ == 0.0) last_report_t_ = t_s;
      if ((t_s - last_report_t_) >= report_period_s_) {
        last_report_t_ = t_s;
        return true;
      }
      return false;
    }

    double last_hz() const { return last_hz_; }
    std::size_t last_window_n() const { return last_window_n_; }

    void reset() {
      time_window_.clear();
      last_report_t_ = 0.0;
      last_hz_ = 0.0;
      last_window_n_ = 0;
    }

   private:
    std::size_t window_size_;
    double report_period_s_;

    std::deque<double> time_window_;
    double last_report_t_ = 0.0;

    double last_hz_ = 0.0;
    std::size_t last_window_n_ = 0;
  };

  Eigen::Vector3d accumulatedGnssCoordinates_{0.0, 0.0, 0.0};

  // Callback Members
  int lidarBetweenCallbackCounter_ = -1;
  double lidarBetweenTimeKm1_{0.0};
  Eigen::Isometry3d lio_T_M_Lkm1_ = Eigen::Isometry3d::Identity();
  bool lidarBetweenChainValid_ = false;  // false after a degenerate scan: the next good scan restarts the delta chain

  int gnssCallbackCounter_ = -1;
  int lidarUnaryCallbackCounter_ = -1;

  int num_imu_errors_ = 0;
  rclcpp::Time last_imu_timestamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subGnssNavSatFix_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLioBetweenOdometry_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLioOdometry_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subLioDegeneracy_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subVioOdometry_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subVioOdometryBetween_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pubGnssPoseWithCov;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pubStatus_;
  rclcpp::TimerBase::SharedPtr alignmentStatusTimer_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubMeasGNSSPath_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubMeasMapLioPath_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubMeasMapLioLidarPath_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubMeasMapVioPath_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubMeasMapVioBetweenPath_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pubReferenceNavSatFixCoordinates_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pubReferenceNavSatFixCoordinatesENU_;

  // Messages
  std::shared_ptr<nav_msgs::msg::Path> measLio_mapLidarPathPtr_;
  std::shared_ptr<nav_msgs::msg::Path> measVio_mapCameraPathPtr_;
  std::shared_ptr<nav_msgs::msg::Path> measVioBetween_mapCameraPathPtr_;
  std::shared_ptr<nav_msgs::msg::Path> measGnssPathPtr_;

  // --------------------------------------------------------------------------
  // Alignment pose buffer for GNSS initialization. Poses are always ^M T_B,
  // regardless of whether LIO or VIO supplies the alignment trajectory.
  mutable std::mutex alignmentPoseBufMutex_;
  std::deque<std::pair<double, Eigen::Isometry3d>> alignmentPoseBuf_;  // (time, ^M T_B)
  static constexpr double kAlignmentBufKeepSec = 5.0;                   // [s]
  static constexpr double kInitSyncMaxDt = 0.20;                 // [s]
  void addAlignmentPose_(double time, const Eigen::Isometry3d& T_M_B);
  bool getClosestAlignmentPose_(double t, Eigen::Isometry3d& T_M_B_out, double* best_dt_out = nullptr) const;
  // --------------------------------------------------------------------------

  // GNSS Handler
  std::shared_ptr<graph_msf::GnssHandler> gnssHandlerPtr_;

  // TrajectoryAlignment Handler
  std::shared_ptr<graph_msf::TrajectoryAlignmentHandler> trajectoryAlignmentHandler_;
  struct TrajectoryAlignmentParams {
    double se3Rate = 10.0, r3Rate = 10.0, minDistanceHeadingInit = 3.0, minimumSpatialSpread = 0.01;
    double noMovementDistance = 1.0, noMovementTime = 3.0;
  } trajAlignParams_;
  std::shared_ptr<graph_msf::TrajectoryAlignmentHandler> makeTrajectoryAligner_() const;

  // --------------------------------------------------------------------------
  enum class LioAlignState { Active, Realigning };
  std::atomic<LioAlignState> lioAlignState_{LioAlignState::Active};
  std::atomic<double> lastAcceptedLioHeaderTime_{0.0};
  // Source of the world alignment before initialisation. LIO when its odometry actually arrives. When LIO is disabled, never
  // arrives or stays degenerate for lioAvailabilityTimeoutSec_ and the VIO unary is enabled, the VIO map is aligned with the world
  // instead: with GNSS the VIO path feeds the GNSS trajectory alignment, without GNSS the world is initialised from VIO. If LIO
  // shows up before initialisation, it takes over again (the aligner and pose buffer are reset at every switch).
  enum class InitAlignmentSource { None, Lio, Vio };
  std::atomic<InitAlignmentSource> initAlignmentSource_{InitAlignmentSource::None};
  double lioAvailabilityTimeoutSec_ = 3.0;  // <= 0: legacy (LIO assumed available whenever useLioOdometry is set)
  std::atomic<double> firstVioHeaderTime_{0.0};
  bool lioAvailable_(double t) const;
  std::shared_ptr<graph_msf::TrajectoryAlignmentHandler> aligner_();  // current aligner, read under realignMutex_
  void switchInitAlignmentSource_(InitAlignmentSource source, double t, const std::string& reason);
  std::mutex realignMutex_;  // guards the members below and the aligner swap
  bool realignInjectPending_ = false;
  Eigen::Isometry3d realignT_W_M_ = Eigen::Isometry3d::Identity();
  int realignGnssMsgCounter_ = 0;
  bool lioRealignEnabled_ = false;
  double lioRealignAbsenceTimeoutSec_ = 10.0;
  int lioRealignAttemptEveryNGnssMsgs_ = 5;
  void realignFromGnss_(const Eigen::Vector3d& W_t_W_Gnss, double timeK);
  // Fallback when GNSS is not available during a re-alignment: anchor the LIO map to the current world estimate.
  double lioRealignNoGnssFallbackSec_ = 0.0;  // <= 0: disabled (legacy: LIO withheld until a GNSS alignment converges)
  // Alignment random walk scaling while GNSS is absent: without GNSS the alignment drift is unobservable, so any walk
  // only lets the optimiser split source disagreement arbitrarily between state and alignment. Scale < 1 freezes it.
  double alignmentRandomWalkScaleWithoutGnss_ = 1.0;  // 1.0 = legacy
  double alignmentRandomWalkGnssAbsenceSec_ = 5.0;
  std::atomic<bool> alignmentRandomWalkScaledActive_{false};
  // Release at GNSS return: for releaseSeconds after the first valid fix following an absence, the walk is inflated by
  // sqrt(missed keyframes) (capped) so that the alignments can jump to the drift that became observable again.
  double alignmentRandomWalkReleaseSeconds_ = 0.0;  // <= 0: disabled
  double alignmentRandomWalkReleaseMaxFactor_ = 20.0;
  std::mutex walkReleaseMutex_;
  double walkReleaseUntil_ = -1.0;
  double walkReleaseFactor_ = 1.0;
  double walkAbsenceStart_ = -1.0;
  Eigen::Matrix<double, 6, 1> currentAlignmentRandomWalk_(const Eigen::Matrix<double, 6, 1>& randomWalk, double t);
  std::atomic<double> lastValidGnssHeaderTime_{0.0};
  double realignStartHeaderTime_ = 0.0;  // guarded by realignMutex_
  // Ring buffer of the published state, ~6 s at IMU rate. T_W_I anchors LIO to the world estimate (re-alignment fallback);
  // the velocities feed the wheel odometry.
  struct BufferedState {
    double t = 0.0;
    Eigen::Isometry3d T_W_I = Eigen::Isometry3d::Identity();
    Eigen::Vector3d I_v_W_I = Eigen::Vector3d::Zero();  // IMU velocity in IMU axes
    Eigen::Vector3d I_w_W_I = Eigen::Vector3d::Zero();  // bias-corrected angular velocity in IMU axes
  };
  std::mutex stateBufferMutex_;
  std::deque<BufferedState> stateBuffer_;
  bool lookupBufferedState_(double t, double maxDt, BufferedState& out);
  bool lookupWorldImuPose_(double t, double maxDt, Eigen::Isometry3d& T_W_I);
  // Source re-origin detection (source-only, no state reference): a pose step the robot cannot physically make
  // (|dp| > vMax*dt + posMargin or |dR| > wMax*dt + rotMargin) means the source restarted its map frame (e.g. a DLIO
  // reset re-origins dlio_map under the same name). The source's fixed frame then gets a new epoch suffix, so GMSF
  // creates a fresh alignment variable initialised from the current state instead of fighting the stale one.
  struct SourceEpochState {
    bool enabled = false;
    double vMax = 4.0, wMax = 3.0, posMargin = 0.5, rotMargin = 0.2;
    bool havePrev = false;
    double prevT = 0.0;
    Eigen::Isometry3d prevPose = Eigen::Isometry3d::Identity();
    int epoch = 0;
  };
  SourceEpochState lioEpoch_, vioEpoch_;
  // Kinematic plausibility of a relative source step over dt, with the re-origin thresholds of s (used by the between factors)
  static bool isPlausibleSourceStep_(const SourceEpochState& s, const Eigen::Isometry3d& T_km1_k, double dt);
  std::string epochFrameName_(SourceEpochState& s, const std::string& baseFrame, double t, const Eigen::Isometry3d& T_M_S, const char* name);
  void publishState(const std::shared_ptr<graph_msf::SafeIntegratedNavState>& integratedNavStatePtr,
                    const std::shared_ptr<graph_msf::SafeNavStateWithCovarianceAndBias>& optimizedStateWithCovarianceAndBiasPtr) override;
  // --------------------------------------------------------------------------

  // Wheel-leg odometry from /joint_states with the self-contained hf_b2w_description (HfB2wWheelOdometry.h) -----------------
  bool useWheelOdometryFlag_ = false;
  double wheelOdometryRate_ = 20.0;  // [Hz] max rate of wheel-odometry factors
  std::string wheelOdometryRobustNormType_ = "huber";
  double wheelOdometryRobustNormConstant_ = 2.0;
  std::string wheelOdometryBaseFrame_;  // frame of hf_b2w_description (must equal the base link frame)
  std::unique_ptr<HfB2wWheelOdometry> wheelOdometryPtr_;
  double lastWheelOdometryTime_ = 0.0;
  bool wheelOdometryFramesCached_ = false;
  Eigen::Matrix3d wheelOdometryR_B_I_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d wheelOdometryI_t_I_B_ = Eigen::Vector3d::Zero();
  struct WheelOdometryStats {
    long accepted = 0, standstill = 0, turning = 0, inconsistent = 0, deviates = 0, noState = 0, missing = 0;
    double lastReportTime = 0.0;
  } wheelOdometryStats_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subJointStates_;
  void jointStatesCallback_(const sensor_msgs::msg::JointState::ConstSharedPtr& jointStatesPtr);
  void readWheelOdometryParams_();

  // Flags
  bool useGnssFlag_ = false;
  bool useLioBetweenOdometryFlag_ = false;
  bool useLioOdometryFlag_ = true;
  bool useVioOdometryFlag_ = false;
  bool useVioOdometryBetweenFlag_ = false;
  std::atomic_bool alignmentStatus_{false};
  std::atomic_bool lioDegenerate_{false};

  // --------------------------------------------------------------------------
  // Cached frame names + constant lever arm (step 9.1)
  void cacheFrames_();
  bool framesCached_ = false;

  std::string worldFrame_;
  std::string baseLinkFrame_;
  std::string gnssFrame_;
  std::string lioOdometryFrame_;
  std::string lidarBetweenFrame_;
  std::string vioOdometryFrame_;

  Eigen::Vector3d t_B_G_cached_ = Eigen::Vector3d::Zero();
  // --------------------------------------------------------------------------
};

}  // namespace b2w_se
