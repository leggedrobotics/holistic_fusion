#ifndef MENZI_SIM_STATICTRANSFORMS_H
#define MENZI_SIM_STATICTRANSFORMS_H

// ROS
#include "tf/tf.h"
// Catkin ws
#include "excavator_model/ExcavatorModel.hpp"

namespace fg_filtering {

class StaticTransforms {
 public:
  StaticTransforms(ros::NodeHandle& privateNode);

  // Setters
  void setOdomFrame(const std::string& s) { odomFrame_ = s; }

  void setBaseLinkFrame(const std::string& s) { baseLinkFrame_ = s; }

  void setImuCabinFrame(const std::string& s) { imuCabinFrame_ = s; }

  void setImuRooftopFrame(const std::string& s) { imuRooftopFrame_ = s; }

  void setLidarFrame(const std::string& s) { lidarFrame_ = s; }

  void setCabinFrame(const std::string& s) { cabinFrame_ = s; }

  void setLeftGnssFrame(const std::string& s) { leftGnssFrame_ = s; }

  void setRightGnssFrame(const std::string& s) { rightGnssFrame_ = s; }

  // Getters
  /// Frames
  std::string getOdomFrame() { return odomFrame_; }

  std::string getBaseLinkFrame() { return baseLinkFrame_; }

  std::string getImuFrame() { return imuFrame_; }

  std::string getLidarFrame() { return lidarFrame_; }

  std::string getCabinFrame() { return cabinFrame_; }

  std::string getLeftGnssFrame() { return leftGnssFrame_; }

  std::string getRightGnssFrame() { return rightGnssFrame_; }

  /// Transformations
  const tf::Transform& T_L_C() { return tf_T_L_C_; }

  const tf::Transform& T_C_L() { return tf_T_C_L_; }

  const tf::Transform& T_L_I() { return tf_T_L_I_; }

  const tf::Transform& T_I_C() { return tf_T_I_C_; }

  const tf::Transform& T_C_I() { return tf_T_C_I_; }

  const tf::Transform& T_Cabin_GnssL() { return tf_T_Cabin_GnssL_; }

  const tf::Transform& T_Cabin_GnssR() { return tf_T_Cabin_GnssR_; }

  const tf::Transform& T_C_B() { return tf_T_C_B_; }

  double BC_z_offset() { return BC_Z_offset_; }

  // Functionality
  void findTransformations();

 private:
  // Names
  /// Description
  std::string urdfDescription_;
  /// Frames
  std::string odomFrame_;
  std::string baseLinkFrame_;
  std::string imuCabinFrame_;
  std::string imuRooftopFrame_;
  std::string imuFrame_;
  std::string lidarFrame_;
  std::string cabinFrame_;
  std::string leftGnssFrame_;
  std::string rightGnssFrame_;

  // Robot Models
  urdf::Model urdfModel_;
  std::unique_ptr<excavator_model::ExcavatorModel> excavatorModelPtr_;

  // Transformations
  tf::Transform tf_T_L_C_;
  tf::Transform tf_T_C_L_;
  tf::Transform tf_T_L_I_;
  tf::Transform tf_T_I_C_;
  tf::Transform tf_T_C_I_;
  tf::Transform tf_T_GnssL_Cabin_;
  tf::Transform tf_T_Cabin_GnssL_;
  tf::Transform tf_T_GnssR_Cabin_;
  tf::Transform tf_T_Cabin_GnssR_;
  tf::Transform tf_T_B_C_;
  tf::Transform tf_T_C_B_;
  double BC_Z_offset_;

  // Methods
  tf::Transform getTransformFromID(const unsigned int bodyId);
};

}  // namespace fg_filtering

#endif  // MENZI_SIM_STATICTRANSFORMS_H
