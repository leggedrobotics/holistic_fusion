/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#pragma once

// std
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
#include <std_msgs/msg/bool.hpp>

// Workspace
#include "graph_msf/gnss/GnssHandler.h"
#include "graph_msf/gnss/GnssCovariance.h"
#include "graph_msf/measurements/UnaryMeasurementXD.h"
#include "graph_msf/trajectory_alignment/TrajectoryAlignmentHandler.h"
#include "graph_msf_ros2/GraphMsfRos2.h"

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
  Eigen::Matrix<double, 6, 1> lioBetweenNoise_ = 100.0 * Eigen::Matrix<double, 6, 1>::Ones();
  // VIO Odometry
  Eigen::Matrix<double, 6, 1> vioPoseBetweenNoise_ = 100.0 * Eigen::Matrix<double, 6, 1>::Ones();
  Eigen::Matrix<double, 6, 1> vioPoseUnaryNoise_ = 100.0 * Eigen::Matrix<double, 6, 1>::Ones();

  // ROS Related stuff ----------------------------

  // Callbacks
  void lidarOdometryCallback_(const nav_msgs::msg::Odometry::ConstSharedPtr& lidarOdomPtr);
  void lidarBetweenOdometryCallback_(const nav_msgs::msg::Odometry::ConstSharedPtr& lidarBetweenOdomPtr);
  void gnssNavSatFixCallback_(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& navSatFixPtr);
  void vioOdometryCallback_(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& vioOdomPtr);
  void vioOdometryBetweenCallback_(const nav_msgs::msg::Odometry::ConstSharedPtr& vioOdomPtr);

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

  int gnssCallbackCounter_ = -1;
  int lidarUnaryCallbackCounter_ = -1;

  int num_imu_errors_ = 0;
  rclcpp::Time last_imu_timestamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subGnssNavSatFix_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLioBetweenOdometry_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLioOdometry_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subVioOdometry_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subVioOdometryBetween_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pubGnssPoseWithCov;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pubStatus_;
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
  // LIO pose buffer for GNSS initialization
  mutable std::mutex lioPoseBufMutex_;
  std::deque<std::pair<double, Eigen::Isometry3d>> lioPoseBuf_;  // (time, ^M T_B)
  static constexpr double kLioBufKeepSec = 5.0;                  // [s]
  static constexpr double kInitSyncMaxDt = 0.20;                 // [s]
  bool getClosestLioPose_(double t, Eigen::Isometry3d& T_M_B_out, double* best_dt_out = nullptr) const;
  // --------------------------------------------------------------------------

  // GNSS Handler
  std::shared_ptr<graph_msf::GnssHandler> gnssHandlerPtr_;

  // TrajectoryAlignment Handler
  std::shared_ptr<graph_msf::TrajectoryAlignmentHandler> trajectoryAlignmentHandler_;

  // Flags
  bool useGnssFlag_ = false;
  bool useLioBetweenOdometryFlag_ = false;
  bool useLioOdometryFlag_ = true;
  bool useVioOdometryFlag_ = false;
  bool useVioOdometryBetweenFlag_ = false;

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
