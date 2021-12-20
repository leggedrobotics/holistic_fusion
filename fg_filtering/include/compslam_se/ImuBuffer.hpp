#ifndef IMU_MANAGER_HPP_
#define IMU_MANAGER_HPP_
#define DEFAULT_IMU_RATE 100

// C++
#include <map>

// Eigen
#include <Eigen/Dense>

// ROS
#include <ros/node_handle.h>

// GTSAM
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>

// Package
#include "compslam_se/Datatypes.hpp"

namespace compslam_se {

#define GREEN_START "\033[92m"
#define YELLOW_START "\033[33m"
#define RED_START "\033[31m"
#define COLOR_END "\033[0m"

class ImuBuffer {
 public:
  // Constructor
  ImuBuffer() : imuRate_(DEFAULT_IMU_RATE) {
    // Reset IMU Buffer
    timeToImuBuffer_.clear();
  }

  // Destructor
  ~ImuBuffer() = default;

  // Setters
  inline void setImuRate(double d) { imuRate_ = d; }
  inline void setImuBufferLength(int i) { imuBufferLength_ = i; }
  inline void setVerboseLevel(int i) { verboseLevel_ = i; }

  // Add to buffers
  void addToIMUBuffer(double ts, double accX, double accY, double accZ, double gyrX, double gyrY, double gyrZ);
  void addToKeyBuffer(double ts, gtsam::Key key);

  // Getters
  inline double getImuRate() { return imuRate_; }
  void getLastTwoMeasurements(TimeToImuMap& imuMap);
  bool getClosestKeyAndTimestamp(const std::string& callingName, double maxSearchDeviation, double tLidar, double& tInGraph,
                                 gtsam::Key& key);
  bool getIMUBufferIteratorsInInterval(const double& ts_start, const double& ts_end, TimeToImuMap::iterator& s_itr,
                                       TimeToImuMap::iterator& e_itr);

  // Public member functions
  /// Determine initial IMU pose w.r.t to gravity vector pointing up
  bool estimateAttitudeFromImu(const std::string& imuGravityDirection, gtsam::Rot3& init_attitude, double& gravity_magnitude,
                               Eigen::Vector3d& gyrBias);

 private:
  // Member variables
  TimeToImuMap timeToImuBuffer_;  // IMU buffer
  TimeToKeyMap timeToKeyBuffer_;
  double imuRate_ = -1;  // Rate of IMU input (Hz) - Used to calculate minimum measurements needed to calculate gravity and init attitude
  int imuBufferLength_ = -1;
  const double imuPoseInitWaitSecs_ = 1.0;  // Multiplied with _imuRate
  int verboseLevel_ = 0;
};

}  // namespace compslam_se

#endif