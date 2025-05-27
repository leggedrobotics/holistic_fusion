/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#pragma once
// ROS 2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>

// Workspace
#include "graph_msf/config/StaticTransforms.h"

namespace graph_msf {

class StaticTransformsTf : public StaticTransforms {
 public:
  explicit StaticTransformsTf(const rclcpp::Node::SharedPtr& node,
                              const graph_msf::StaticTransforms& staticTransforms);

  virtual ~StaticTransformsTf() = default;

 protected:
  void findTransformations() override;

  // Members
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace graph_msf
