/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef StaticTransformsUrdf_H
#define StaticTransformsUrdf_H

// ROS
#include <tf/transform_listener.h>

// Workspace
#include "graph_msf/config/StaticTransforms.h"

namespace graph_msf {

class StaticTransformsTf : public StaticTransforms {
 public:
  StaticTransformsTf() = default;

 protected:
  void findTransformations() override;

  // Members
  tf::TransformListener listener_;
};
}  // namespace graph_msf
#endif  // end StaticTransformsUrdf_H
