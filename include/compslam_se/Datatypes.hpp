#ifndef FG_FILTERING_DATATYPES_HPP
#define FG_FILTERING_DATATYPES_HPP

namespace compslam_se {

// Map from time to 6D IMU measurements
typedef std::map<double, gtsam::Vector6, std::less<double>, Eigen::aligned_allocator<std::pair<const double, gtsam::Vector6>>> TimeToImuMap;

// Map from time to gtsam key
typedef std::map<double, gtsam::Key, std::less<double>, Eigen::aligned_allocator<std::pair<const double, gtsam::Vector6>>> TimeToKeyMap;

}  // namespace compslam_se

#endif  // FG_FILTERING_DATATYPES_HPP
