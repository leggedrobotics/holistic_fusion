/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef ExcavatorStaticTransforms_H
#define ExcavatorStaticTransforms_H
// Workspace
#include "graph_msf_ros/extrinsics/StaticTransformsTf.h"

namespace excavator_se {

class ExcavatorStaticTransforms : public graph_msf::StaticTransformsTf {
 public:
  ExcavatorStaticTransforms(std::shared_ptr<ros::NodeHandle> privateNodePtr);

  // Setters ---------------------------------------------------------------
  void setLidarFrame(const std::string& s) { lidarFrame_ = s; }
  void setLeftGnssFrame(const std::string& s) { leftGnssFrame_ = s; }
  void setRightGnssFrame(const std::string& s) { rightGnssFrame_ = s; }
  void setCabinFrame(const std::string& s) { cabinFrame_ = s; }
  void setBaseLinkFrame(const std::string& s) { baseLinkFrame_ = s; }

  // Getters ---------------------------------------------------------------
  const std::string& getLidarFrame() { return lidarFrame_; }
  const std::string& getLeftGnssFrame() { return leftGnssFrame_; }
  const std::string& getRightGnssFrame() { return rightGnssFrame_; }
  const std::string& getCabinFrame() { return cabinFrame_; }
  const std::string& getBaseLinkFrame() { return baseLinkFrame_; }

 protected:  // Methods
  void findTransformations() override;

 private:  // Members
  // Robot frame names
  std::string lidarFrame_;
  std::string leftGnssFrame_;
  std::string rightGnssFrame_;
  std::string cabinFrame_;
  std::string baseLinkFrame_;
};
}  // namespace excavator_se
#endif  // end AsopStaticTransforms_H
