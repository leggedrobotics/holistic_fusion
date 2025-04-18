/*
Copyright 2024 by Julian Nubert & Timon Mathis, Robotic Systems Lab & Autonomous Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef POSITION3_STATIC_TRANSFORMS_H
#define POSITION3_STATIC_TRANSFORMS_H

// Workspace
#include "graph_msf_ros/extrinsics/StaticTransformsTf.h"

namespace position3_se {

class Position3StaticTransforms : public graph_msf::StaticTransformsTf {
 public:
  Position3StaticTransforms(const std::shared_ptr<ros::NodeHandle> privateNodePtr, bool usePrism, bool useGnss);

  // Setters ---------------------------------------------------------------
  void setPrismPositionMeasFrame(const std::string& s) { prismPositionMeasFrame_ = s; }
  void setGnssPositionMeasFrame(const std::string& s) { gnssPositionMeasFrame_ = s; }
  void setGnssOfflinePoseMeasFrame(const std::string& s) { gnssOfflinePoseMeasFrame_ = s; }

  // Getters ---------------------------------------------------------------
  const std::string& getPrismPositionMeasFrame() { return prismPositionMeasFrame_; }
  const std::string& getGnssPositionMeasFrame() { return gnssPositionMeasFrame_; }
  const std::string& getGnssOfflinePoseMeasFrame() { return gnssOfflinePoseMeasFrame_; }

 protected:  // Methods
  void findTransformations() override;

 private:  // Members
  // Frame names
  std::string prismPositionMeasFrame_;
  std::string gnssPositionMeasFrame_;
  std::string gnssOfflinePoseMeasFrame_;

  // Bools
  bool usePrism_;
  bool useGnss_;
};

}  // namespace position3_se

#endif  // end POSITION3_STATIC_TRANSFORMS_H
