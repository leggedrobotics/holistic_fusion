#ifndef SIGNAL_LOGGER_HPP_
#define SIGNAL_LOGGER_HPP_

// ROS
#include <ros/node_handle.h>

// GTSAM
#include <gtsam/geometry/Pose3.h>

// Workspace
#include "signal_logger/signal_logger.hpp"

namespace fg_filtering {

class SignalLogger {
 public:
  // Constructor / Destructor
  SignalLogger(){};

  ~SignalLogger() {
    ROS_INFO("Logging nodes ------------------------------------------");
    ROS_INFO("Saving log-files.");
    signal_logger::logger->stopAndSaveLoggerData(
        signal_logger::LogFileTypeSet{signal_logger::LogFileType::BINARY, signal_logger::LogFileType::CSV});
    signal_logger::logger->cleanup();
  };

  // Setup
  void setup(ros::NodeHandle& nh) {
    // configure logger
    double window_time = 6000.0;
    std::string logger_name = "FG_FILTERING_ICRA_LOG";
    // signal logger ros
    signal_logger::setSignalLoggerRos(&nh);
    // Options
    signal_logger::SignalLoggerOptions siloOptions;
    siloOptions.updateFrequency_ = static_cast<int>(1.0 / 0.01);
    siloOptions.maxLoggingTime_ = window_time;
    // siloOptions.collectScriptFileName_ = loggingScriptFilename;
    siloOptions.loggerName_ = logger_name;
    // Init logger
    signal_logger::logger->initLogger(siloOptions);
    ROS_INFO("Initialize logger with window time: %f, sampling frequency: %d", siloOptions.maxLoggingTime_, siloOptions.updateFrequency_);

    // Add poses to logger
    addRosTimeToLogger();
    addWorldPosesToLogger();
    addOdomPosesToLogger();
    addLinearVelocitiesToLogger();
    addLatencyToLogger();
    addBiasesToLogger();
    addOptimizedPoseToLogger();
    signal_logger::logger->updateLogger();
    signal_logger::logger->startLogger();
  };

  void addRosTimeToLogger() {
    signal_logger::add(_rosTimeS, "ros_time_s", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
    signal_logger::add(_rosTimeNs, "ros_time_ns", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
  }

  // Prepare logger
  void addWorldPosesToLogger() {
    // Translation
    signal_logger::add(_worldPositionLogMeas.x(), "world_x", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
    signal_logger::add(_worldPositionLogMeas.y(), "world_y", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
    signal_logger::add(_worldPositionLogMeas.z(), "world_z", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);

    // Rotation
    signal_logger::add(_worldRollPitchYawLogMeas.x(), "world_roll", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
    signal_logger::add(_worldRollPitchYawLogMeas.y(), "world_pitch", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
    signal_logger::add(_worldRollPitchYawLogMeas.z(), "world_yaw", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
  }

  void addOdomPosesToLogger() {
    // Translation
    signal_logger::add(_odomPositionLogMeas.x(), "odom_x", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
    signal_logger::add(_odomPositionLogMeas.y(), "odom_y", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
    signal_logger::add(_odomPositionLogMeas.z(), "odom_z", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);

    // Rotation
    signal_logger::add(_odomRollPitchYawLogMeas.x(), "odom_roll", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
    signal_logger::add(_odomRollPitchYawLogMeas.y(), "odom_pitch", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
    signal_logger::add(_odomRollPitchYawLogMeas.z(), "odom_yaw", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
  }

  void addLinearVelocitiesToLogger() {
    // Translation
    signal_logger::add(_imuLinearVelocityLogMeas.x(), "v_x", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
    signal_logger::add(_imuLinearVelocityLogMeas.y(), "v_y", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
    signal_logger::add(_imuLinearVelocityLogMeas.z(), "v_z", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
  }

  void addLatencyToLogger() {
    // Translation
    signal_logger::add(_latencyInMicroseconds, "imu_latency_us", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
  }

  void addBiasesToLogger() {
    // Linear acceleration
    signal_logger::add(_imuBiasLogMeas.accelerometer()(0), "bias_lin_x", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
    signal_logger::add(_imuBiasLogMeas.accelerometer()(1), "bias_lin_y", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
    signal_logger::add(_imuBiasLogMeas.accelerometer()(2), "bias_lin_z", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);

    // Angular velocity
    signal_logger::add(_imuBiasLogMeas.gyroscope()(0), "bias_ang_x", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
    signal_logger::add(_imuBiasLogMeas.gyroscope()(1), "bias_ang_y", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
    signal_logger::add(_imuBiasLogMeas.gyroscope()(2), "bias_ang_z", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
  }

  // Prepare logger
  void addOptimizedPoseToLogger() {
    // Translation
    signal_logger::add(_optimizedPositionLogMeas.x(), "optimized_x", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
    signal_logger::add(_optimizedPositionLogMeas.y(), "optimized_y", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
    signal_logger::add(_optimizedPositionLogMeas.z(), "optimized_z", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);

    // Rotation
    signal_logger::add(_optimizedRollPitchYawLogMeas.x(), "optimized_roll", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
    signal_logger::add(_optimizedRollPitchYawLogMeas.y(), "optimized_pitch", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
    signal_logger::add(_optimizedRollPitchYawLogMeas.z(), "optimized_yaw", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);

    signal_logger::add(_optimizedTimeS, "optimized_time_s", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
    signal_logger::add(_optimizedTimeNs, "optimized_time_ns", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
  }

  // Publish logger data
  void publishLogger(long rosTimeS, long rosTimeNs, const gtsam::Pose3 worldPose, const gtsam::Pose3 odomPose,
                     const gtsam::Point3 linearVelocity, double latency, const gtsam::imuBias::ConstantBias imuBias) {
    // Set correct ros time
    _rosTimeS = rosTimeS;
    _rosTimeNs = rosTimeNs;

    // World Pose
    _worldPositionLogMeas.x() = worldPose.x();
    _worldPositionLogMeas.y() = worldPose.y();
    _worldPositionLogMeas.z() = worldPose.z();
    _worldRollPitchYawLogMeas.x() = worldPose.rotation().roll() * 180 / M_PI;
    _worldRollPitchYawLogMeas.y() = worldPose.rotation().pitch() * 180 / M_PI;
    _worldRollPitchYawLogMeas.z() = worldPose.rotation().yaw() * 180 / M_PI;

    // Odom Pose
    _odomPositionLogMeas.x() = odomPose.x();
    _odomPositionLogMeas.y() = odomPose.y();
    _odomPositionLogMeas.z() = odomPose.z();
    _odomRollPitchYawLogMeas.x() = odomPose.rotation().roll() * 180 / M_PI;
    _odomRollPitchYawLogMeas.y() = odomPose.rotation().pitch() * 180 / M_PI;
    _odomRollPitchYawLogMeas.z() = odomPose.rotation().yaw() * 180 / M_PI;

    // Linear Velocity
    _imuLinearVelocityLogMeas.x() = linearVelocity.x();
    _imuLinearVelocityLogMeas.y() = linearVelocity.y();
    _imuLinearVelocityLogMeas.z() = linearVelocity.z();

    // Latency
    _latencyInMicroseconds = latency;

    // Bias
    _imuBiasLogMeas = imuBias;

    signal_logger::logger->collectLoggerData();
    signal_logger::logger->publishData();
  }

  void publishOptimizedPosition(long optimizedTimeS, long optimizedTimeNs, const gtsam::Pose3 optimizedPose) {
    // Set correct ros time
    _optimizedTimeS = optimizedTimeS;
    _optimizedTimeNs = optimizedTimeNs;

    // World Pose
    _optimizedPositionLogMeas.x() = optimizedPose.x();
    _optimizedPositionLogMeas.y() = optimizedPose.y();
    _optimizedPositionLogMeas.z() = optimizedPose.z();
    _optimizedRollPitchYawLogMeas.x() = optimizedPose.rotation().roll() * 180 / M_PI;
    _optimizedRollPitchYawLogMeas.y() = optimizedPose.rotation().pitch() * 180 / M_PI;
    _optimizedRollPitchYawLogMeas.z() = optimizedPose.rotation().yaw() * 180 / M_PI;
  }

 private:
  long _rosTimeS;
  long _rosTimeNs;
  Eigen::Vector3d _worldPositionLogMeas;
  Eigen::Vector3d _worldRollPitchYawLogMeas;
  Eigen::Vector3d _odomPositionLogMeas;
  Eigen::Vector3d _odomRollPitchYawLogMeas;
  Eigen::Vector3d _imuLinearVelocityLogMeas;
  double _latencyInMicroseconds;
  gtsam::imuBias::ConstantBias _imuBiasLogMeas;

  Eigen::Vector3d _optimizedPositionLogMeas;
  Eigen::Vector3d _optimizedRollPitchYawLogMeas;
  long _optimizedTimeS;
  long _optimizedTimeNs;
};

/////////////////////////////////////////////////

// Signal logger for GNSS
class SignalLoggerGnss {
 public:
  // Constructor / Destructor
  SignalLoggerGnss(){};

  ~SignalLoggerGnss() {
    ROS_INFO("Logging nodes ------------------------------------------");
    ROS_INFO("Saving log-files.");
    signal_logger::logger->stopAndSaveLoggerData(
        signal_logger::LogFileTypeSet{signal_logger::LogFileType::BINARY, signal_logger::LogFileType::CSV});
    signal_logger::logger->cleanup();
  };

  // Setup
  void setup(ros::NodeHandle& nh) {
    // configure logger
    double window_time = 6000.0;
    std::string logger_name = "GNSS_ICRA_LOG";
    // signal logger ros
    signal_logger::setSignalLoggerRos(&nh);
    // Options
    signal_logger::SignalLoggerOptions siloOptions;
    siloOptions.updateFrequency_ = static_cast<int>(100);
    siloOptions.maxLoggingTime_ = window_time;
    // siloOptions.collectScriptFileName_ = loggingScriptFilename;
    siloOptions.loggerName_ = logger_name;
    // Init logger
    signal_logger::logger->initLogger(siloOptions);
    ROS_INFO("Initialize logger with window time: %f, sampling frequency: %d", siloOptions.maxLoggingTime_, siloOptions.updateFrequency_);

    // Add poses to logger
    addRosTimeToLogger();
    addWorldPositionsToLogger();
    signal_logger::logger->updateLogger();
    signal_logger::logger->startLogger();
  };

  void addRosTimeToLogger() {
    // Translation
    signal_logger::add(_rosTimeS, "ros_time_s", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
    signal_logger::add(_rosTimeNs, "ros_time_ns", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
  }

  // Prepare logger
  void addWorldPositionsToLogger() {
    // Translation
    signal_logger::add(_worldPositionLogMeas.x(), "world_x", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
    signal_logger::add(_worldPositionLogMeas.y(), "world_y", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
    signal_logger::add(_worldPositionLogMeas.z(), "world_z", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
  }

  // Publish logger data
  void publishLogger(long rosTimeS, long rosTimeNs, const gtsam::Point3 worldPose) {
    // Set correct ros time
    _rosTimeS = rosTimeS;
    _rosTimeNs = rosTimeNs;

    // World Pose
    _worldPositionLogMeas.x() = worldPose.x();
    _worldPositionLogMeas.y() = worldPose.y();
    _worldPositionLogMeas.z() = worldPose.z();

    signal_logger::logger->collectLoggerData();
    signal_logger::logger->publishData();
  }

 private:
  long _rosTimeS;
  long _rosTimeNs;
  Eigen::Vector3d _worldPositionLogMeas;
};

}  // namespace fg_filtering

#endif