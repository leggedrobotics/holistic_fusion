/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef AnymalStaticTransforms_H
#define AnymalStaticTransforms_H
// Workspace
#include "graph_msf_ros/extrinsics/StaticTransformsTf.h"

namespace anymal_se {

class AnymalStaticTransforms : public graph_msf::StaticTransformsTf {
 public:
  AnymalStaticTransforms(const std::shared_ptr<ros::NodeHandle> privateNodePtr,
                         const graph_msf::StaticTransforms& staticTransforms = graph_msf::StaticTransforms());

  // Setters
  void setLioOdometryFrame(const std::string& s) { lioOdometryFrame_ = s; }
  void setGnssFrame(const std::string& s) { gnssFrame_ = s; }

  // Getters
  const std::string& getLioOdometryFrame() { return lioOdometryFrame_; }
  const std::string& getGnssFrame() { return gnssFrame_; }

 private:
  void findTransformations() override;

  // Members
  std::string lioOdometryFrame_;
  std::string gnssFrame_;
};
}  // namespace anymal_se
#endif  // end AsopStaticTransforms_H
