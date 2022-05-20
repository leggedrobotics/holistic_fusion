#ifndef StaticTransformsUrdf_H
#define StaticTransformsUrdf_H

// ROS
#include <tf/transform_listener.h>

// Workspace
#include "compslam_se/StaticTransforms.h"

namespace compslam_se {

class StaticTransformsTf : public StaticTransforms {
 public:
  StaticTransformsTf(ros::NodeHandle& privateNode);

 private:
  void findTransformations() override;

  // Members
  tf::TransformListener listener_;
};
}  // namespace compslam_se
#endif  // end StaticTransformsUrdf_H
