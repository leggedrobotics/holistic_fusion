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
#include "holistic_fusion/config/StaticTransforms.h"

namespace holistic_fusion {

class StaticTransformsTf : public StaticTransforms {
 public:
  StaticTransformsTf() = default;

 protected:
  bool findTransformations() override;

  // Members
  tf::TransformListener listener_;
};
}  // namespace holistic_fusion
#endif  // end StaticTransformsUrdf_H
