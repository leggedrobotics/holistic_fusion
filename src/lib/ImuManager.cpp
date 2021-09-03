#include "fg_filtering/ImuManager.hpp"

namespace fg_filtering {

// Public --------------------------------------------------------
void ImuManager::addToIMUBuffer(double ts, double accX, double accY, double accZ, double gyrX, double gyrY, double gyrZ) {
  // Convert to gtsam type
  gtsam::Vector6 imuMeas;
  imuMeas << accX, accY, accZ, gyrX, gyrY, gyrZ;

  // Add to buffer
  std::lock_guard<std::mutex> lock(imuBufferMutex_);
  imuBuffer_[ts] = imuMeas;
}

void ImuManager::getLastTwoMeasurements(IMUMap& imuMap) {
  IMUMapItr endItr = --(imuBuffer_.end());
  IMUMapItr previousItr = --(--(imuBuffer_.end()));

  // Write into IMU Map
  imuMap[previousItr->first] = previousItr->second;
  imuMap[endItr->first] = endItr->second;
}

void ImuManager::getClosestIMUBufferIteratorToTime(const double& tLidar, IMUMapItr& s_itr) {
  std::cout << "Buffer Start/End: " << std::fixed << imuBuffer_.begin()->first << "/" << imuBuffer_.rbegin()->first
            << "Searched time stamp: " << tLidar << std::endl;
  s_itr = imuBuffer_.lower_bound(tLidar);
}

bool ImuManager::getClosestKeyAndTimestamp(const std::string& callingName, double maxSearchDeviation, double tK, double& tInGraph,
                                           gtsam::Key& key) {
  auto upperIterator = timeToKeyBuffer_.upper_bound(tK);
  auto lowerIterator = upperIterator;
  --lowerIterator;

  // Keep key which is closer to tLidar
  tInGraph = std::abs(tK - lowerIterator->first) < std::abs(upperIterator->first - tK) ? lowerIterator->first : upperIterator->first;
  key = std::abs(tK - lowerIterator->first) < std::abs(upperIterator->first - tK) ? lowerIterator->second : upperIterator->second;

  if (verboseLevel_ >= 2) {
    std::cout << YELLOW_START << "ImuManager" << COLOR_END << " " << callingName << std::setprecision(14) << " searched time step: " << tK
              << std::endl;
    std::cout << YELLOW_START << "ImuManager" << COLOR_END << " " << callingName << std::setprecision(14)
              << " Found time step: " << tInGraph << std::endl;
    std::cout << YELLOW_START << "ImuManager" << COLOR_END << " Delay: " << tInGraph - tK << " s" << std::endl;
  }

  // Check for error and warn user
  double timeDeviation = std::abs(tInGraph - tK);
  if (timeDeviation > maxSearchDeviation) {
    std::cerr << YELLOW_START << "ImuManager " << RED_START << callingName << " Time deviation at key " << key << " is " << timeDeviation
              << " s, being larger than admissible deviation of " << maxSearchDeviation << " s" << std::endl;
    return false;
  }

  return true;
}

bool ImuManager::getIMUBufferIteratorsInInterval(const double& ts_start, const double& ts_end, IMUMapItr& s_itr, IMUMapItr& e_itr) {
  // Check if timestamps are in correct order
  if (ts_start >= ts_end) {
    std::cout << "\033[33mIMU-Manager\033[0m IMU Lookup Timestamps are not correct ts_start(" << std::fixed << ts_start << ") >= ts_end("
              << ts_end << ")\n";
    return false;
  }

  // Get Iterator Belonging to ts_start
  s_itr = imuBuffer_.lower_bound(ts_start);
  // Get Iterator Belonging to ts_end
  e_itr = imuBuffer_.lower_bound(ts_end);

  // Check if it is first value in the buffer which means there is no value before to interpolate with
  if (s_itr == imuBuffer_.begin()) {
    std::cout << "\033[33mIMU-Manager\033[0m Lookup requires first message of IMU buffer, cannot Interpolate back, "
                 "Lookup Start/End: "
              << std::fixed << ts_start << "/" << ts_end << ", Buffer Start/End: " << imuBuffer_.begin()->first << "/"
              << imuBuffer_.rbegin()->first << std::endl;
    return false;
  }

  // Check if lookup start time is ahead of buffer start time
  if (s_itr == imuBuffer_.end()) {
    std::cout << "\033[33mIMU-Manager\033[0m IMU Lookup start time ahead latest IMU message in the buffer, lookup: " << ts_start
              << ", latest IMU: " << imuBuffer_.rbegin()->first << std::endl;
    return false;
  }

  // Check if last value is valid
  if (e_itr == imuBuffer_.end()) {
    std::cout << "\033[33mIMU-Manager\033[0m Lookup is past IMU buffer, with lookup Start/End: " << std::fixed << ts_start << "/" << ts_end
              << " and latest IMU: " << imuBuffer_.rbegin()->first << std::endl;
    e_itr = imuBuffer_.end();
    --e_itr;
  }

  // Check if two IMU messages are different
  if (s_itr == e_itr) {
    std::cout << "\033[33mIMU-Manager\033[0m Not Enough IMU values between timestamps , with Start/End: " << std::fixed << ts_start << "/"
              << ts_end << ", with diff: " << ts_end - ts_start << std::endl;
    return false;
  }

  // If everything is good
  return true;
}

bool ImuManager::estimateAttitudeFromImu(const double initTs, const std::string& imuGravityDirection, gtsam::Rot3& initAttitude,
                                         double& gravityMagnitude, Eigen::Vector3d& gyrBias) {
  // Get timestamp of first message for lookup
  if (imuBuffer_.size() < (imuRate_ * imuPoseInitWaitSecs_)) {
    return false;
  } else {
    // Get IMU Message iterators in the interval
    IMUMap initImuMap;
    double prev_ts = initTs - imuPoseInitWaitSecs_;
    bool success = getInterpolatedImuMeasurements_(prev_ts, initTs, initImuMap);
    if (success) {
      // Accumulate Acceleration part of IMU Messages
      Eigen::Vector3d initAccMean(0.0, 0.0, 0.0), initGyrMean(0.0, 0.0, 0.0);
      for (auto& itr : initImuMap) {
        initAccMean += itr.second.head<3>();
        initGyrMean += itr.second.tail<3>();
      }

      // Average IMU measurements and set assumed gravity direction
      initAccMean /= initImuMap.size();
      gravityMagnitude = initAccMean.norm();
      Eigen::Vector3d gUnitVec;
      if (imuGravityDirection == "up") {
        gUnitVec = Eigen::Vector3d(0.0, 0.0, 1.0);  // ROS convention
      } else if (imuGravityDirection == "down") {
        gUnitVec = Eigen::Vector3d(0.0, 0.0, -1.0);
      } else {
        throw std::runtime_error("Gravity direction must be either 'up' or 'down'.");
      }
      // Normalize gravity vectors to remove the affect of gravity magnitude from place-to-place
      initAccMean.normalize();
      initAttitude = gtsam::Rot3(Eigen::Quaterniond().setFromTwoVectors(initAccMean, gUnitVec));

      // Gyro
      initGyrMean /= initImuMap.size();
      gyrBias = initGyrMean;

      // Calculate robot initial orientation using gravity vector.
      std::cout << "\033[33mFGF-IMU-Manager\033[0m Gravity Magnitude: " << gravityMagnitude << std::endl;
      std::cout << "\033[33mFGF-IMU-Manager\033[0m Mean IMU Acceleration Vector(x,y,z): " << initAccMean.transpose()
                << " - Gravity Unit Vector(x,y,z): " << gUnitVec.transpose() << std::endl;
      std::cout << "\033[33mFGF-IMU-Manager\033[0m Yaw/Pitch/Roll(deg): " << initAttitude.ypr().transpose() * (180.0 / M_PI) << std::endl;
      std::cout << "\033[33mFGF-IMU-Manager\033[0m Gyro bias(x,y,z): " << initGyrMean.transpose() << std::endl;
    } else {
      return false;
    }
  }
  return true;
}

// Private -----------------------------------------------------------
bool ImuManager::getInterpolatedImuMeasurements_(const double& ts_start, const double& ts_end, IMUMap& interpolatedIMUMap) {
  // clear
  interpolatedIMUMap.clear();

  // Get nearest IMUMap iterators corresponding to timestamps
  IMUMapItr s_itr, e_itr;
  std::lock_guard<std::mutex> lock(imuBufferMutex_);
  bool success = getIMUBufferIteratorsInInterval(ts_start, ts_end, s_itr, e_itr);
  if (success) {
    // Copy in between IMU measurements
    interpolatedIMUMap.insert(s_itr, e_itr);  // Note: Value at e_itr is not inserted

    // Interpolate first element
    if (s_itr->first > ts_start) {
      auto prev_s_itr = s_itr;
      --prev_s_itr;
      gtsam::Vector6 ts_start_meas =
          interpolateIMUMeasurement_(prev_s_itr->first, prev_s_itr->second, ts_start, s_itr->first, s_itr->second);

      // Add Interpolated Value to return IMUMap
      interpolatedIMUMap[ts_start] = ts_start_meas;
    }

    // Add last element
    if (e_itr->first > ts_end) {
      // Interpolate IMU message at timestamp
      auto prev_e_itr = e_itr;
      --prev_e_itr;
      gtsam::Vector6 ts_end_meas = interpolateIMUMeasurement_(prev_e_itr->first, prev_e_itr->second, ts_end, e_itr->first, e_itr->second);

      interpolatedIMUMap[ts_end] = ts_end_meas;
    } else {
      // Add last actual IMU message
      interpolatedIMUMap[e_itr->first] =
          e_itr->second;  ////std::map insert doesn't insert last iterator so if e_itr->first > ts_end then it means
      /// we are at the end of IMU buffer and we need to insert the last IMU message to the return
      /// buffer
      // Extrapolate IMU message at timestamp(ts_end>k), e_itr(k), prev_e_itr(k-1)
      auto prev_e_itr = e_itr;
      --prev_e_itr;
      gtsam::Vector6 ts_end_meas = extrapolateIMUMeasurement_(prev_e_itr->first, prev_e_itr->second, e_itr->first, e_itr->second, ts_end);
      // Add Extrapolated Value to return IMUMap
      interpolatedIMUMap[ts_end] = ts_end_meas;
    }
    return true;
  } else {
    return false;
  }
}

gtsam::Vector6 ImuManager::interpolateIMUMeasurement_(const double& ts1, const gtsam::Vector6& meas1, const double& ts2, const double& ts3,
                                                      const gtsam::Vector6& meas3) {
  double tsDiffRatio = (ts2 - ts1) / (ts3 - ts1);                //(x2-x1)/(x3-x1)
  gtsam::Vector6 meas2 = (meas3 - meas1) * tsDiffRatio + meas1;  // y2 = (y3-y1) * ((x2-x1)/(x3-x1)) + y1

  return meas2;
}

gtsam::Vector6 ImuManager::extrapolateIMUMeasurement_(const double& ts1, const gtsam::Vector6& meas1, const double& ts2,
                                                      const gtsam::Vector6& meas2, const double& ts3) {
  double tsDiffRatio = (ts3 - ts1) / (ts2 - ts1);                //(x2-x1)/(x3-x1)
  gtsam::Vector6 meas3 = (meas2 - meas1) * tsDiffRatio + meas1;  // y3 = (y2-y1) * ((x2-x1)/(x3-x1)) + y1

  return meas3;
}

}  // namespace fg_filtering