#ifndef IMU_MANAGER_HPP_
#define IMU_MANAGER_HPP_
#define DEFAULT_IMU_RATE 100

// C++
#include <map>
#include <mutex>

// Eigen
#include <Eigen/Dense>

// ROS
#include <ros/node_handle.h>

// GTSAM
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>

// Catkin workspace
#include "fg_filtering/Datatypes.hpp"

namespace fg_filtering {

class ImuManager {
 public:
  // Constructor
  ImuManager() : imuRate_(DEFAULT_IMU_RATE) {
    // Reset IMU Buffer
    imuBuffer_.clear();
  }

  // Destructor
  ~ImuManager() = default;

  // Setters
  inline void setImuRate(double d) { imuRate_ = d; }

  // Add to buffers
  void addToIMUBuffer(double ts, double accX, double accY, double accZ, double gyrX, double gyrY, double gyrZ);
  inline void addToKeyBuffer(double ts, gtsam::Key key) { timeToKeyBuffer_[ts] = key; }
  inline void addImuPoseToBuffer(double ts, const gtsam::Pose3& pose) { imuPosesInGraphBuffer_[ts] = pose; }

  // Getters
  inline double getImuRate() { return imuRate_; }

  void getLastTwoMeasurements(IMUMap& imuMap);

  void getClosestIMUBufferIteratorToTime(const double& tLidar, IMUMapItr& s_itr);

  void getClosestKeyAndTimestamp(double tLidar, double& tInGraph, gtsam::Key& key);

  inline void getCorrespondingImuGraphPose(const double& tLidar, gtsam::Pose3& pose) {
    pose = imuPosesInGraphBuffer_.lower_bound(tLidar)->second;
  }

  bool getIMUBufferIteratorsInInterval(const double& ts_start, const double& ts_end, IMUMapItr& s_itr, IMUMapItr& e_itr);

  // Determine initial IMU pose w.r.t to gravity vector pointing up
  bool estimateAttitudeFromImu(const double imu_pose_init_ts, const std::string& imuGravityDirection, gtsam::Rot3& init_attitude,
                               double& gravity_magnitude, Eigen::Vector3d& gyrBias);

 private:
  // Methods
  bool getInterpolatedImuMeasurements_(const double& ts_start, const double& ts_end, IMUMap& interpolatedIMUMap);

  static gtsam::Vector6 interpolateIMUMeasurement_(const double& ts1, const gtsam::Vector6& meas1, const double& ts2, const double& ts3,
                                                   const gtsam::Vector6& meas3);

  static gtsam::Vector6 extrapolateIMUMeasurement_(const double& ts1, const gtsam::Vector6& meas1, const double& ts2,
                                                   const gtsam::Vector6& meas2, const double& ts3);

  // Member variables
  std::mutex imuBufferMutex_;  // Mutex for reading writing IMU buffer
  IMUMap imuBuffer_;           // IMU buffer
  TimeToKeyMap timeToKeyBuffer_;
  GraphIMUPoseMap imuPosesInGraphBuffer_;
  double imuRate_;  // Rate of IMU input (Hz) - Used to calculate minimum measurements needed to calculate gravity and init attitude
  const double imuPoseInitWaitSecs_ = 1.0;  // Multiplied with _imuRate
};

}  // namespace fg_filtering
#endif