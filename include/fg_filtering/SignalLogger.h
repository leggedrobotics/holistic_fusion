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
    ROS_INFO("Initialize logger with window time: %f, sampling frequency: %d", siloOptions.maxLoggingTime_, siloOptions.updateFrequency_);

    // Add poses to logger
    addPosesToLogger();
    addBiasesToLogger();
    signal_logger::logger->updateLogger();
    signal_logger::logger->startLogger();
  };

  // Prepare logger
  void addPosesToLogger() {
    // Translation
    signal_logger::add(_positionLogMeas.x(), "translation_x", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
    signal_logger::add(_positionLogMeas.y(), "translation_y", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
    signal_logger::add(_positionLogMeas.z(), "translation_z", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);

    // Rotation
    signal_logger::add(_orientationLogMeas.x(), "rotation_roll", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
    signal_logger::add(_orientationLogMeas.y(), "rotation_pitch", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
    signal_logger::add(_orientationLogMeas.z(), "rotation_yaw", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
  }

  void addBiasesToLogger() {
    // Linear acceleration
    signal_logger::add(_biasLogMeas.accelerometer()(0), "bias_linAcc_x", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
    signal_logger::add(_biasLogMeas.accelerometer()(1), "bias_linAcc_y", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
    signal_logger::add(_biasLogMeas.accelerometer()(2), "bias_linAcc_z", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);

    // Angular velocity
    signal_logger::add(_biasLogMeas.gyroscope()(0), "bias_angVel_x", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
    signal_logger::add(_biasLogMeas.gyroscope()(1), "bias_angVel_y", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
    signal_logger::add(_biasLogMeas.gyroscope()(2), "bias_angVel_z", "", "", 1, signal_logger::LogElementAction::SAVE, 10000,
                       signal_logger::BufferType::EXPONENTIALLY_GROWING);
  }

  // Publish logger data
  void publishLogger(const gtsam::Pose3 inputMeasurement, const gtsam::imuBias::ConstantBias inputBias) {
    //    ROS_ERROR_STREAM("Value of x position in logger: " << inputMeasurement.translation()(0));
    //    ROS_ERROR_STREAM("Value of roll old: " << inputMeasurement.rotation().roll());
    //    ROS_ERROR_STREAM("Value of roll new: " << inputMeasurement.rotation());
    _positionLogMeas.x() = inputMeasurement.x();
    _positionLogMeas.y() = inputMeasurement.y();
    _positionLogMeas.z() = inputMeasurement.z();
    _orientationLogMeas.x() = inputMeasurement.rotation().roll() * 180 / M_PI;
    _orientationLogMeas.y() = inputMeasurement.rotation().pitch() * 180 / M_PI;
    _orientationLogMeas.z() = inputMeasurement.rotation().yaw() * 180 / M_PI;
    _biasLogMeas = inputBias;

    signal_logger::logger->collectLoggerData();
    signal_logger::logger->publishData();
  }

 private:
  Eigen::Vector3d _positionLogMeas;
  Eigen::Vector3d _orientationLogMeas;
  gtsam::imuBias::ConstantBias _biasLogMeas;
};

}  // namespace fg_filtering

#endif