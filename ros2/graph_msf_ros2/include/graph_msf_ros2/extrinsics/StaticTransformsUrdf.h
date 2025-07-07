/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef StaticTransformsUrdf_H
#define StaticTransformsUrdf_H

#include <tf2_ros/transform_listener.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include "graph_msf/config/StaticTransforms.h"
#include "graph_msf_ros2/extrinsics/ElementToRoot.h"
#include "rclcpp/rclcpp.hpp"
#include "urdf/model.h"

namespace graph_msf {

class StaticTransformsUrdf : public StaticTransforms {
 public:
  StaticTransformsUrdf(const std::shared_ptr<rclcpp::Node> privateNodePtr);
  void setup();

 protected:
  std::string urdfDescriptionName_;
  urdf::Model urdfModel_;
  std::unique_ptr<urdf::Model> model_;
  KDL::Tree tree_;
  std::map<std::string, ElementToRoot> segments_;
  void getRootTransformations(const KDL::SegmentMap::const_iterator element, std::string rootName = "");
  tf2::Transform kdlToTransform(const KDL::Frame& k);
  std::shared_ptr<rclcpp::Node> privateNode_;

 private:
  virtual void findTransformations() = 0;
};

}  // namespace graph_msf
#endif  // end StaticTransformsUrdf_H
