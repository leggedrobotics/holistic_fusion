#ifndef STATICTRANSFORMS_H
#define STATICTRANSFORMS_H

#include <Eigen/Eigen>
#include <iostream>

namespace compslam_se {

#define GREEN_START "\033[92m"
#define YELLOW_START "\033[33m"
#define COLOR_END "\033[0m"

class StaticTransforms {
 public:
  // Constructor
  StaticTransforms() { std::cout << YELLOW_START << "StaticTransforms" << COLOR_END << " StaticTransforms instance created." << std::endl; }

  // Setters
  void setMapFrame(const std::string& s) { mapFrame_ = s; }

  void setOdomFrame(const std::string& s) { odomFrame_ = s; }

  void setImuFrame(const std::string& s) { imuFrame_ = s; }

  void setLidarFrame(const std::string& s) { lidarFrame_ = s; }

  void setLeftGnssFrame(const std::string& s) { leftGnssFrame_ = s; }

  void setRightGnssFrame(const std::string& s) { rightGnssFrame_ = s; }

  void setCabinFrame(const std::string& s) { cabinFrame_ = s; }

  // Getters
  /// Frames
  const std::string& getMapFrame() { return mapFrame_; }

  const std::string& getOdomFrame() { return odomFrame_; }

  const std::string& getImuFrame() { return imuFrame_; }

  const std::string& getLidarFrame() { return lidarFrame_; }

  const std::string& getLeftGnssFrame() { return leftGnssFrame_; }

  const std::string& getRightGnssFrame() { return rightGnssFrame_; }

  const std::string& getCabinFrame() { return cabinFrame_; }

  /// Transformations
  const Eigen::Matrix4d& T_L_I() { return T_L_I_; }
  const Eigen::Matrix4d& T_GnssL_I() { return T_GnssL_I_; }
  const Eigen::Matrix4d& T_GnssR_I() { return T_GnssR_I_; }
  Eigen::Matrix4d T_C_I_;
  const Eigen::Matrix4d& T_C_I() { return T_C_I_; }

  // Functionality
  virtual void findTransformations() = 0;

 protected:
  // Names
  /// Frames
  std::string mapFrame_;
  std::string odomFrame_;
  std::string imuFrame_;  // If used --> copied to imuCabinFrame_
  std::string lidarFrame_;
  std::string leftGnssFrame_;
  std::string rightGnssFrame_;
  std::string cabinFrame_;

  // Transformations
  Eigen::Matrix4d T_L_I_;
  Eigen::Matrix4d T_GnssL_I_;
  Eigen::Matrix4d T_GnssR_I_;
};

}  // namespace compslam_se

#endif  // STATICTRANSFORMS_H
