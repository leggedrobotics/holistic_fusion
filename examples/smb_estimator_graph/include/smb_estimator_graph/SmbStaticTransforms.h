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
  void setLidarOdometryFrame(const std::string& s) { lidarOdometryFrame_ = s; }
  void setWheelOdometryFrame(const std::string& s) { wheelOdometryFrame_ = s; }

  // Getters
  const std::string& getLidarOdometryFrame() { return lidarOdometryFrame_; }
  const std::string& getWheelOdometryFrame() { return wheelOdometryFrame_; }

 private:
  void findTransformations() override;

  // Members
  std::string lidarOdometryFrame_;
  std::string wheelOdometryFrame_;
};
}  // namespace smb_se
#endif  // end Smb_Static_Transforms_H
