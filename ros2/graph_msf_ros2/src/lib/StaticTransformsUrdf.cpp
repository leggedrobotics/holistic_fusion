/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#include "graph_msf_ros2/extrinsics/StaticTransformsUrdf.h"
#include <tf2_ros/transform_listener.h>

namespace graph_msf {

StaticTransformsUrdf::StaticTransformsUrdf(const std::shared_ptr<rclcpp::Node> privateNodePtr)
    : privateNode_(privateNodePtr) {
  RCLCPP_INFO(privateNode_->get_logger(), "StaticTransformsUrdf Initializing...");
}

void StaticTransformsUrdf::setup() {
  RCLCPP_INFO(privateNode_->get_logger(), "Setting up for description name '%s'", urdfDescriptionName_.c_str());

  // Declare and retrieve the URDF parameter
  privateNode_->declare_parameter<std::string>(urdfDescriptionName_, "default_description");
  std::string urdfDescriptionContent;
  privateNode_->get_parameter(urdfDescriptionName_, urdfDescriptionContent);

  if (urdfDescriptionContent.empty()) {
    throw std::runtime_error("StaticTransformsUrdf - Could not load description, description empty.");
  }

  // Initialize the urdf::Model from the URDF string content
  if (!urdfModel_.initString(urdfDescriptionContent)) {
    throw std::runtime_error("StaticTransformsUrdf - Failed to parse URDF description content.");
  }

  // Initialize the KDL tree
  if (!kdl_parser::treeFromUrdfModel(urdfModel_, tree_)) {
    throw std::runtime_error("Failed to extract KDL tree from robot description");
  }

  segments_.clear();
  getRootTransformations(tree_.getRootSegment());

  RCLCPP_INFO(privateNode_->get_logger(), "StaticTransformsUrdf Initialized.");
}

void StaticTransformsUrdf::getRootTransformations(const KDL::SegmentMap::const_iterator element, std::string rootName) {
  const std::string& elementName = GetTreeElementSegment(element->second).getName();
  if (rootName.empty()) {
    rootName = elementName;
  }

  auto children = GetTreeElementChildren(element->second);
  for (const auto& child : children) {
    KDL::Chain chain;
    tree_.getChain(rootName, child->second.segment.getName(), chain);

    tf2::Transform T_root_element;
    for (int i = 0; i < chain.getNrOfSegments(); ++i) {
      KDL::Frame frameToRoot = chain.getSegment(i).getFrameToTip();
      double x, y, z, w;
      frameToRoot.M.GetQuaternion(x, y, z, w);
      tf2::Quaternion q(x, y, z, w);

      tf2::Vector3 t(frameToRoot.p[0], frameToRoot.p[1], frameToRoot.p[2]);
      T_root_element = T_root_element * tf2::Transform(q, t);
    }

    ElementToRoot elementToRoot(T_root_element, rootName, child->second.segment.getName());
    segments_.emplace(child->second.segment.getName(), elementToRoot);
    getRootTransformations(child, rootName);
  }
}

tf2::Transform StaticTransformsUrdf::kdlToTransform(const KDL::Frame& k) {
  tf2::Transform tf_T;
  tf_T.setOrigin(tf2::Vector3(k.p.x(), k.p.y(), k.p.z()));
  tf2::Quaternion q;
  k.M.GetQuaternion(q[0], q[1], q[2], q[3]);
  tf_T.setRotation(q);

  return tf_T;
}

}  // namespace graph_msf
