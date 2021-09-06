//
// Created by nubertj on 02.08.21.
//

#ifndef FG_FILTERING_DATATYPES_HPP
#define FG_FILTERING_DATATYPES_HPP

// Map from time to 6D IMU measurements
typedef std::map<double, gtsam::Vector6, std::less<double>, Eigen::aligned_allocator<std::pair<const double, gtsam::Vector6>>> IMUMap;
/// IMU map iterator
typedef IMUMap::iterator IMUMapItr;
// Map from time to pose
typedef std::map<double, gtsam::Pose3, std::less<double>, Eigen::aligned_allocator<std::pair<const double, gtsam::Vector6>>>
    GraphIMUPoseMap;
// Map from time to gtsam key
typedef std::map<double, gtsam::Key, std::less<double>, Eigen::aligned_allocator<std::pair<const double, gtsam::Vector6>>> TimeToKeyMap;

#endif  // FG_FILTERING_DATATYPES_HPP
