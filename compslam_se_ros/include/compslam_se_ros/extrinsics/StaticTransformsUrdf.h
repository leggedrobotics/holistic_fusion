#ifndef StaticTransformsUrdf_H
#define StaticTransformsUrdf_H

// ROS
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include "urdf/model.h"

// Workspace
#include "compslam_se/StaticTransforms.h"
#include "compslam_se_ros/extrinsics/ElementToRoot.h"

namespace compslam_se {

class StaticTransformsUrdf : public StaticTransforms {
 public:
  StaticTransformsUrdf(ros::NodeHandle& privateNode);
  void setup();

 protected:
  // Names
  /// Description
  std::string urdfDescriptionName_;

  // Robot Models
  urdf::Model urdfModel_;
  std::unique_ptr<urdf::Model> model_;
  KDL::Tree tree_;

  /// A map of dynamic segment names to SegmentPair structures
  std::map<std::string, ElementToRoot> segments_;

  // Methods
  void getRootTransformations(const KDL::SegmentMap::const_iterator element, std::string rootName = "");
  tf::Transform kdlToTransform(const KDL::Frame& k);

 private:
  virtual void findTransformations() = 0;
  ros::NodeHandle privateNode_;
};
}  // namespace compslam_se
#endif  // end StaticTransformsUrdf_H
