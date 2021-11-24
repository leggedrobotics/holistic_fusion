#ifndef MENZI_SIM_STATICTRANSFORMS_H
#define MENZI_SIM_STATICTRANSFORMS_H

// ROS
#include <urdf/model.h>
#include <Eigen/Eigen>
#include <kdl/tree.hpp>
#include "tf/tf.h"

namespace compslam_se {

class ElementToRoot final {
 public:
  /// Constructor
  explicit ElementToRoot(const tf::Transform& T, const std::string& rootName_, const std::string& elementName_)
      : T_root_element(T), rootName(rootName_), elementName(elementName_) {}

  tf::Transform T_root_element;  ///< The KDL segment
  std::string rootName;          ///< The name of the root element to which this link is attached
  std::string elementName;       ///< The name of the element
};

class StaticTransforms {
 public:
  StaticTransforms(ros::NodeHandle& privateNode);

  // Setters
  void setMapFrame(const std::string& s) { mapFrame_ = s; }

  void setOdomFrame(const std::string& s) { odomFrame_ = s; }

  void setBaseLinkFrame(const std::string& s) { baseLinkFrame_ = s; }

  void setImuCabinFrame(const std::string& s) { imuCabinFrame_ = s; }

  void setImuRooftopFrame(const std::string& s) { imuRooftopFrame_ = s; }

  void setImuBaseFrame(const std::string& s) { imuBaseFrame_ = s; }

  void setLidarFrame(const std::string& s) { lidarFrame_ = s; }

  void setCabinFrame(const std::string& s) { cabinFrame_ = s; }

  void setLeftGnssFrame(const std::string& s) { leftGnssFrame_ = s; }

  void setRightGnssFrame(const std::string& s) { rightGnssFrame_ = s; }

  // Getters
  /// Frames
  std::string getMapFrame() { return mapFrame_; }

  std::string getOdomFrame() { return odomFrame_; }

  std::string getBaseLinkFrame() { return baseLinkFrame_; }

  std::string getImuCabinFrame() { return imuCabinFrame_; }

  std::string getImuRooftopFrame() { return imuRooftopFrame_; }

  std::string getImuBaseFrame() { return imuBaseFrame_; }

  std::string getLidarFrame() { return lidarFrame_; }

  std::string getCabinFrame() { return cabinFrame_; }

  std::string getLeftGnssFrame() { return leftGnssFrame_; }

  std::string getRightGnssFrame() { return rightGnssFrame_; }

  /// Transformations
  const tf::Transform& T_L_C() { return tf_T_L_C_; }

  const tf::Transform& T_C_L() { return tf_T_C_L_; }

  const tf::Transform& T_L_Ic() { return tf_T_L_Ic_; }

  const tf::Transform& T_Ic_C() { return tf_T_Ic_C_; }

  const tf::Transform& T_C_Ic() { return tf_T_C_Ic_; }

  const tf::Transform& T_C_GnssL() { return tf_T_C_GnssL_; }

  const tf::Transform& T_C_GnssR() { return tf_T_C_GnssR_; }

  const tf::Transform& T_GnssL_C() { return tf_T_GnssL_C_; }

  const tf::Transform& T_GnssR_C() { return tf_T_GnssR_C_; }

  const tf::Transform& T_B_Ib() { return tf_T_B_Ib_; }

  double BC_z_offset() { return BC_Z_offset_; }

  // Functionality
  void findTransformations();

 private:
  // Names
  /// Description
  std::string urdfDescription_;
  /// Frames
  std::string mapFrame_;
  std::string odomFrame_;
  std::string baseLinkFrame_;
  std::string imuCabinFrame_;
  std::string imuRooftopFrame_;  // If used --> copied to imuCabinFrame_
  std::string imuBaseFrame_;
  std::string lidarFrame_;
  std::string cabinFrame_;
  std::string leftGnssFrame_;
  std::string rightGnssFrame_;

  // Robot Models
  urdf::Model urdfModel_;
  KDL::Tree tree_;

  // Transformations
  tf::Transform tf_T_L_C_;
  tf::Transform tf_T_C_L_;
  tf::Transform tf_T_L_Ic_;
  tf::Transform tf_T_Ic_C_;
  tf::Transform tf_T_C_Ic_;
  tf::Transform tf_T_GnssL_C_;
  tf::Transform tf_T_C_GnssL_;
  tf::Transform tf_T_GnssR_C_;
  tf::Transform tf_T_C_GnssR_;
  tf::Transform tf_T_Ib_B_;
  tf::Transform tf_T_B_Ib_;
  double BC_Z_offset_;

  /// A map of dynamic segment names to SegmentPair structures
  std::map<std::string, ElementToRoot> segments_;

  /// A pointer to the parsed URDF model
  std::unique_ptr<urdf::Model> model_;

  // Methods
  tf::Transform kdlToTransform(const KDL::Frame& k);
  void getRootTransformations(const KDL::SegmentMap::const_iterator element, std::string rootName = "");
};

}  // namespace compslam_se

#endif  // MENZI_SIM_STATICTRANSFORMS_H
