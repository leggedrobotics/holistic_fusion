/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef STATIC_TRANSFORMS_H
#define STATIC_TRANSFORMS_H

// Eigen
#include <Eigen/Eigen>

// Workspace
#include "graph_msf/core/TransformsDictionary.h"

namespace graph_msf {

class StaticTransforms : public TransformsDictionary<Eigen::Isometry3d> {
 public:
  // Constructor
  StaticTransforms() : TransformsDictionary<Eigen::Isometry3d>(Eigen::Isometry3d::Identity()) {
    std::cout << YELLOW_START << "StaticTransforms" << COLOR_END << " StaticTransforms instance created." << std::endl;
  }

  void setImuFrame(const std::string& s) { imuFrame_ = s; }
  void setWorldFrame(const std::string& s) { worldFrame_ = s; }
  void setOdomFrame(const std::string& s) { odomFrame_ = s; }
  void setBaseLinkFrame(const std::string& s) { baseLinkFrame_ = s; }
  void setInitializationFrame(const std::string& s) { initializationFrame_ = s; }

  // Getters ------------------------------------------------------------
  // Frames
  const std::string& getImuFrame() { return imuFrame_; }
  const std::string& getWorldFrame() { return worldFrame_; }
  const std::string& getOdomFrame() { return odomFrame_; }
  const std::string& getBaseLinkFrame() { return baseLinkFrame_; }
  const std::string& getInitializationFrame() { return initializationFrame_; }

  // Functionality ------------------------------------------------------------
  virtual void findTransformations() {
    std::cout << YELLOW_START << "StaticTransforms" << COLOR_END << " findTransformations() function not implemented." << std::endl;
    std::runtime_error("findTransformations() function not implemented.");
  }

 protected:
  // Required frames
  std::string worldFrame_;
  std::string odomFrame_;
  std::string imuFrame_;
  std::string baseLinkFrame_;
  std::string initializationFrame_;
};

}  // namespace graph_msf

#endif  // STATIC_TRANSFORMS_H
