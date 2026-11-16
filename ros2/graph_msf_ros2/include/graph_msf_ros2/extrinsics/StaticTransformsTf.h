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
  explicit StaticTransformsTf(const rclcpp::Node::SharedPtr& node);

  virtual ~StaticTransformsTf() = default;

  // Stop listening to /tf and /tf_static once all static transforms are stored. The listener (own spin thread) otherwise
  // deserialises every /tf message for the node's lifetime, including the node's own 400 Hz output.
  void releaseTfListener();

 protected:
  bool findTransformations() override;

  // Members
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace graph_msf
