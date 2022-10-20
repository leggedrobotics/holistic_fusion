#ifndef StaticTransformsUrdf_H
#define StaticTransformsUrdf_H

// ROS
#include <tf/transform_listener.h>

// Workspace
#include "graph_msf/StaticTransforms.h"

namespace graph_msf {

class StaticTransformsTf : public StaticTransforms {
 public:
  StaticTransformsTf(ros::NodeHandle& privateNode);

 protected:
  virtual void findTransformations() = 0;

  // Members
  tf::TransformListener listener_;
};
}  // namespace graph_msf
#endif  // end StaticTransformsUrdf_H
