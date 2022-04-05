#ifndef M545StaticTransforms_H
#define M545StaticTransforms_H

// ROS
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

// Workspace
#include "compslam_se/StaticTransforms.h"

namespace compslam_ros {

class StaticTransforms : public compslam_se::StaticTransforms {
 public:
  StaticTransforms(ros::NodeHandle& privateNode);

 private:
  void findTransformations() override;

  // Members
  KDL::Tree tree_;
  tf::TransformListener listener_;
};
}  // namespace compslam_ros
#endif  // end M545ESTIMATORGRAPH_H
