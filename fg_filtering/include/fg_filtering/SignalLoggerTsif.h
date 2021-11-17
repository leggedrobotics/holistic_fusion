#ifndef SIGNAL_LOGGER_TSIF_HPP_
#define SIGNAL_LOGGER_TSIF_HPP_

// ROS
#include <ros/node_handle.h>

// GTSAM
#include <gtsam/geometry/Pose3.h>

// Workspace
#include "signal_logger/signal_logger.hpp"

namespace compslam_se {

class SignalLoggerTsif {
 public:
  // Constructor / Destructor
  SignalLoggerTsif(){};

  ~SignalLoggerTsif() {
    std::cout << "Logging nodes ------------------------------------------\n";
    std::cout << "Saving log-files.\n";
    signal_logger::logger->stopAndSaveLoggerData(
        signal_logger::LogFileTypeSet{signal_logger::LogFileType::BINARY, signal_logger::LogFileType::CSV});
    signal_logger::logger->cleanup();
  };

  // Setup
  void setup(ros::NodeHandle& nh) {
    // configure logger
    double window_time = 6000.0;
    std::string logger_name = "/home/nubertj/.ros/ODOM_ICRA_LOG";
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
    addLinearVelocitiesToLogger();
    addLatencyToLogger();
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
    signal_logger::add(_latencyInMs, "imu_latency", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
  }

  // Publish logger data
  void publishLogger(long rosTimeS, long rosTimeNs, const gtsam::Pose3 worldPose,
                     const gtsam::Point3 linearVelocity) {  //, double latency) {
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

    // Linear Velocity
    _imuLinearVelocityLogMeas.x() = linearVelocity.x();
    _imuLinearVelocityLogMeas.y() = linearVelocity.y();
    _imuLinearVelocityLogMeas.z() = linearVelocity.z();

    // Latency
    //_latencyInMs = latency;

    signal_logger::logger->collectLoggerData();
    signal_logger::logger->publishData();
  }

 private:
  long _rosTimeS;
  long _rosTimeNs;
  Eigen::Vector3d _worldPositionLogMeas;
  Eigen::Vector3d _worldRollPitchYawLogMeas;
  Eigen::Vector3d _imuLinearVelocityLogMeas;
  double _latencyInMs;
};

}  // namespace compslam_se

#endif