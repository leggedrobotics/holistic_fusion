/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef Smb_Static_Transforms_H
#define Smb_Static_Transforms_H
// Workspace
#include "graph_msf_ros/extrinsics/StaticTransformsTf.h"

namespace smb_se {

class SmbStaticTransforms : public graph_msf::StaticTransformsTf {
 public:
  SmbStaticTransforms(const std::shared_ptr<ros::NodeHandle> privateNodePtr,
                      const graph_msf::StaticTransforms& staticTransforms = graph_msf::StaticTransforms());

  // Setters
  void setLioOdometryFrame(const std::string& s) { lidarOdometryFrame_ = s; }
  void setWheelOdometryFrame(const std::string& s) { wheelOdometryFrame_ = s; }
  void setLegOdometryFrame(const std::string& s) { legOdometryFrame_ = s; }
  void setVioOdometryFrame(const std::string& s) { vioOdometryFrame_ = s; }

  // Getters
  const std::string& getLioOdometryFrame() { return lidarOdometryFrame_; }
  const std::string& getWheelOdometryFrame() { return wheelOdometryFrame_; }
  const std::string& getLegOdometryFrame() { return legOdometryFrame_; }
  const std::string& getVioOdometryFrame() { return vioOdometryFrame_; }

 private:
  void findTransformations() override;

  // Members
  std::string lidarOdometryFrame_;
  std::string wheelOdometryFrame_;
  std::string legOdometryFrame_;
  std::string vioOdometryFrame_;
};
}  // namespace smb_se
#endif  // end Smb_Static_Transforms_H
