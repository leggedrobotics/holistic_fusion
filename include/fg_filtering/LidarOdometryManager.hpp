#ifndef LIDAR_ODOMETRY_MANAGER_HPP_
#define LIDAR_ODOMETRY_MANAGER_HPP_

//C++
#include <map>
#include <mutex>

//Eigen
#include <Eigen/Dense>

//GTSAM
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>

namespace fg_filtering {
typedef std::map<double, gtsam::Vector6, std::less<double>, Eigen::aligned_allocator<std::pair<const double, gtsam::Vector6>>> LidarOdometryMap;
typedef LidarOdometryMap::iterator LidarOdometryMapItr;

class LidarOdometryManager {
 public:
  //Constructor
  LidarOdometryManager()
      : _lidarOdometryRate(10) {
    //Reset IMU Buffer
    _lidarOdometryBuffer.clear();
  }

  //Destructor
  ~LidarOdometryManager() {}

  //Setters
  void setLidarOdometryRate(double d) { _lidarOdometryRate = d; }

  // Add IMU measurement to buffer
  void addToLidarOdometryBuffer() {
    //Convert to gtsam type
    gtsam::Vector6 lidarOdometryMeas;
    //imuMeas << accX, accY, accZ, gyrX, gyrY, gyrZ;

    //Add to buffer
    std::lock_guard<std::mutex> lock(_lidarOdometryBufferMutex);
    //_IMUBuffer[ts] = imuMeas;
  }

  //Get iterators to IMU messages in a given time interval
  bool getLidarOdometryBufferIteratorsInInterval(const double& ts_start, const double& ts_end, LidarOdometryMapItr& s_itr, LidarOdometryMapItr& e_itr) {
    //Check if timestamps are in correct order
    if (ts_start >= ts_end) {
      std::cout << "\033[33mlidarOdometry-Manager\033[0m LiDAR Odometry Lookup Timestamps are not correct ts_start(" << std::fixed << ts_start << ") >= ts_end(" << ts_end << ")\n";
      return false;
    }

    //Get Iterator Belonging to ts_start
    s_itr = _lidarOdometryBuffer.lower_bound(ts_start);
    //Get Iterator Belonging to ts_end
    e_itr = _lidarOdometryBuffer.lower_bound(ts_end);

    //Check if it is first value in the buffer which means there is no value before to interpolate with
    if (s_itr == _lidarOdometryBuffer.begin()) {
      std::cout << "\033[33mIMU-Manager\033[0m Lookup requires first message of IMU buffer, cannot Interpolate back, Lookup Start/End: "
                << std::fixed << ts_start << "/" << ts_end << ", Buffer Start/End: "
                << _lidarOdometryBuffer.begin()->first << "/" << _lidarOdometryBuffer.rbegin()->first << std::endl;
      return false;
    }

    //Check if lookup start time is ahead of buffer start time
    if (s_itr == _lidarOdometryBuffer.end()) {
      std::cout << "\033[33mIMU-Manager\033[0m IMU Lookup start time ahead latest IMU message in the buffer, lookup: " << ts_start
                << ", latest IMU: " << _lidarOdometryBuffer.rbegin()->first << std::endl;
      return false;
    }

    //Check if last value is valid
    if (e_itr == _lidarOdometryBuffer.end()) {
      std::cout << "\033[33mIMU-Manager\033[0m Lookup is past IMU buffer, with lookup Start/End: " << std::fixed << ts_start
                << "/" << ts_end << " and latest IMU: " << _lidarOdometryBuffer.rbegin()->first << std::endl;
      e_itr = _lidarOdometryBuffer.end();
      --e_itr;
    }

    //Check if two IMU messages are different
    if (s_itr == e_itr) {
      std::cout << "\033[33mIMU-Manager\033[0m Not Enough IMU values between timestamps , with Start/End: " << std::fixed << ts_start << "/" << ts_end
                << ", with diff: " << ts_end - ts_start << std::endl;
      return false;
    }

    //If everything is good
    return true;
  }

 private:
  std::mutex _lidarOdometryBufferMutex;               //Mutex for reading writing IMU buffer
  LidarOdometryMap _lidarOdometryBuffer;                        //IMU buffer
  double _lidarOdometryRate;                          //Rate of IMU input (Hz) - Used to calculate minimum measurements needed to calculate gravity and init attitude
  const double _imuPoseInitWaitSecs = 1.0;  //Multiplied with _imuRate
};
}  // namespace fg_filtering
#endif