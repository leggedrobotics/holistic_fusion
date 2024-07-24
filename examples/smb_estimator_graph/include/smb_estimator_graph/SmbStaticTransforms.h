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
  void setWheelOdometryBetweenFrame(const std::string& s) { wheelOdometryBetweenFrame_ = s; }
  void setWheelLinearVelocityLeftFrame(const std::string& s) { wheelLinearVelocityLeftFrame_ = s; }
  void setWheelLinearVelocityRightFrame(const std::string& s) { wheelLinearVelocityRightFrame_ = s; }
  void setVioOdometryFrame(const std::string& s) { vioOdometryFrame_ = s; }

  // Getters
  const std::string& getLioOdometryFrame() { return lidarOdometryFrame_; }
  const std::string& getWheelOdometryBetweenFrame() { return wheelOdometryBetweenFrame_; }
  const std::string& getWheelLinearVelocityLeftFrame() { return wheelLinearVelocityLeftFrame_; }
  const std::string& getWheelLinearVelocityRightFrame() { return wheelLinearVelocityRightFrame_; }
  const std::string& getVioOdometryFrame() { return vioOdometryFrame_; }

 private:
  void findTransformations() override;

  // Frames
  // LiDAR
  std::string lidarOdometryFrame_;
  // Wheel
  // Between
  std::string wheelOdometryBetweenFrame_;
  // Linear Velocities
  std::string wheelLinearVelocityLeftFrame_;
  std::string wheelLinearVelocityRightFrame_;
  // VIO
  std::string vioOdometryFrame_;
};
}  // namespace smb_se
#endif  // end Smb_Static_Transforms_H
