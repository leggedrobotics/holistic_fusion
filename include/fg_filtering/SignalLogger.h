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
  void setup(ros::NodeHandle &nh) {
    // configure logger
    double window_time = 60.0;
    std::string logger_name = "fg_filtering_log";
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
    ROS_INFO("Initialize logger with window time: %f, sampling frequency: %d", siloOptions.maxLoggingTime_,
             siloOptions.updateFrequency_);

    // Add poses to logger
    addPosesToLogger();
  };

  // Prepare logger
  void addPosesToLogger() {
    // Translation
    signal_logger::add(_logMeas.x(), "translation_x", "cabin", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
    signal_logger::add(_logMeas.y(), "translation_y", "cabin", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
    signal_logger::add(_logMeas.z(), "translation_z", "cabin", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);

    // Rotation
    signal_logger::add(_logMeas.rotation().yaw(), "rotation_yaw", "cabin", "", 1, signal_logger::LogElementAction::SAVE,
                       10000, signal_logger::BufferType::EXPONENTIALLY_GROWING);
    signal_logger::add(_logMeas.rotation().pitch(), "rotation_pitch", "cabin", "", 1,
                       signal_logger::LogElementAction::SAVE, 10000, signal_logger::BufferType::EXPONENTIALLY_GROWING);
    signal_logger::add(_logMeas.rotation().roll(), "rotation_roll", "cabin", "", 1,
                       signal_logger::LogElementAction::SAVE, 10000, signal_logger::BufferType::EXPONENTIALLY_GROWING);

    signal_logger::logger->updateLogger();
    signal_logger::logger->startLogger();
  }

  // Publish logger data
  void publishLogger(const gtsam::Pose3 &inputMeasurement) {
    _logMeas = inputMeasurement;

    signal_logger::logger->collectLoggerData();
    signal_logger::logger->publishData();
  }

 private:
  gtsam::Pose3 _logMeas;
};

}  // namespace fg_filtering

#endif