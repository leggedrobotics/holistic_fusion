#ifndef MENZI_SIM_STATICTRANSFORMS_H
#define MENZI_SIM_STATICTRANSFORMS_H

// ROS
#include <urdf/model.h>
#include <Eigen/Eigen>
#include "tf/tf.h"

namespace compslam_se {

#define GREEN_START "\033[92m"
#define YELLOW_START "\033[33m"
#define COLOR_END "\033[0m"

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
  // Constructor
  StaticTransforms() { std::cout << YELLOW_START << "StaticTransforms" << COLOR_END << " StaticTransforms instance created." << std::endl; }

  // Setters
  void setMapFrame(const std::string& s) { mapFrame_ = s; }

  void setOdomFrame(const std::string& s) { odomFrame_ = s; }

  void setImuFrame(const std::string& s) { imuFrame_ = s; }

  void setLidarFrame(const std::string& s) { lidarFrame_ = s; }

  void setLeftGnssFrame(const std::string& s) { leftGnssFrame_ = s; }

  void setRightGnssFrame(const std::string& s) { rightGnssFrame_ = s; }

  // Getters
  /// Frames
  std::string getMapFrame() { return mapFrame_; }

  std::string getOdomFrame() { return odomFrame_; }

  std::string getImuFrame() { return imuFrame_; }

  std::string getLidarFrame() { return lidarFrame_; }

  std::string getLeftGnssFrame() { return leftGnssFrame_; }

  std::string getRightGnssFrame() { return rightGnssFrame_; }

  /// Transformations
  const tf::Transform& T_L_I() { return tf_T_L_I_; }
  const tf::Transform& T_C_I() { return tf_T_C_I_; } // center to IMU
  const tf::Transform& T_GnssL_I() { return tf_T_GnssL_I_; }
  const tf::Transform& T_GnssR_I() { return tf_T_GnssR_I_; }

  // Functionality
  virtual void findTransformations() = 0;

 protected:
  // Names
  /// Description
  std::string urdfDescription_;
  /// Frames
  std::string mapFrame_;
  std::string odomFrame_;
  std::string imuFrame_;  // If used --> copied to imuCabinFrame_
  std::string lidarFrame_;
  std::string leftGnssFrame_;
  std::string rightGnssFrame_;

  // Robot Models
  urdf::Model urdfModel_;

  // Transformations
  tf::Transform tf_T_L_I_;
  tf::Transform tf_T_C_I_;
  tf::Transform tf_T_GnssL_I_;
  tf::Transform tf_T_GnssR_I_;

  /// A map of dynamic segment names to SegmentPair structures
  std::map<std::string, ElementToRoot> segments_;

  /// A pointer to the parsed URDF model
  std::unique_ptr<urdf::Model> model_;
};

}  // namespace compslam_se

#endif  // MENZI_SIM_STATICTRANSFORMS_H
