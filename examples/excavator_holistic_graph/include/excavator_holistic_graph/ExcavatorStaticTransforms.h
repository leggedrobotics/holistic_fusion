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
  ExcavatorStaticTransforms(const std::shared_ptr<ros::NodeHandle> privateNodePtr,
                            const graph_msf::StaticTransforms& staticTransforms = graph_msf::StaticTransforms());

  // Setters ---------------------------------------------------------------
  void setLioOdometryFrame(const std::string& s) { lidarFrame_ = s; }
  void setLeftGnssFrame(const std::string& s) { leftGnssFrame_ = s; }
  void setRightGnssFrame(const std::string& s) { rightGnssFrame_ = s; }
  void setCabinFrame(const std::string& s) { cabinFrame_ = s; }

  // Getters ---------------------------------------------------------------
  const std::string& getLioOdometryFrame() { return lidarFrame_; }
  const std::string& getLeftGnssFrame() { return leftGnssFrame_; }
  const std::string& getRightGnssFrame() { return rightGnssFrame_; }
  const std::string& getCabinFrame() { return cabinFrame_; }

 protected:  // Methods
  void findTransformations() override;

 private:  // Members
  // Robot frame names
  std::string lidarFrame_;
  std::string leftGnssFrame_;
  std::string rightGnssFrame_;
  std::string cabinFrame_;
};
}  // namespace excavator_se
#endif  // end AsopStaticTransforms_H
