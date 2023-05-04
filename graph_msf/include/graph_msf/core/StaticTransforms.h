/*
Copyright 2022 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef STATIC_TRANSFORMS_H
#define STATIC_TRANSFORMS_H

#include <Eigen/Eigen>
#include <iostream>

namespace graph_msf {

#define GREEN_START "\033[92m"
#define YELLOW_START "\033[33m"
#define RED_START "\033[31m"
#define COLOR_END "\033[0m"

class StaticTransforms {
 public:
  // Constructor
  StaticTransforms() { std::cout << YELLOW_START << "StaticTransforms" << COLOR_END << " StaticTransforms instance created." << std::endl; }

  // Setters ------------------------------------------------------------
  void set_T_frame1_frame2_andInverse(const std::string& frame1, const std::string& frame2, const Eigen::Isometry3d& T_frame1_frame2) {
    lv_T_frame1_frame2(frame1, frame2) = T_frame1_frame2;
    lv_T_frame1_frame2(frame2, frame1) = rv_T_frame1_frame2(frame1, frame2).inverse();
  }

  void setImuFrame(const std::string& s) { imuFrame_ = s; }
  void setWorldFrame(const std::string& s) { worldFrame_ = s; }
  void setMapFrame(const std::string& s) { mapFrame_ = s; }
  void setOdomFrame(const std::string& s) { odomFrame_ = s; }
  void setBaseLinkFrame(const std::string& s) { baseLinkFrame_ = s; }
  void setInitializationFrame(const std::string& s) { initializationFrame_ = s; }

  // Getters ------------------------------------------------------------
  // Returns a left value of the requested transformation
  Eigen::Isometry3d& lv_T_frame1_frame2(const std::string& frame1, const std::string& frame2) {
    if (frame1 == frame2) {
      return identityTransform_;
    } else {
      std::pair<std::string, std::string> framePair(frame1, frame2);
      return T_frame1_frame2_map_[framePair];
    }
  }

  // Returns a right value to the requested transformation
  const Eigen::Isometry3d& rv_T_frame1_frame2(const std::string& frame1, const std::string& frame2) {
    if (frame1 == frame2) {
      return identityTransform_;
    } else {
      std::pair<std::string, std::string> framePair(frame1, frame2);
      auto keyIterator = T_frame1_frame2_map_.find(framePair);
      if (keyIterator == T_frame1_frame2_map_.end()) {
        std::cout << YELLOW_START << "StaticTransforms" << COLOR_END << " No transform found for " << frame1 << " and " << frame2
                  << std::endl;
        throw std::runtime_error("No transform found for " + frame1 + " and " + frame2);
      }
      return keyIterator->second;
    }
  }

  // Frames
  const std::string& getImuFrame() { return imuFrame_; }
  const std::string& getWorldFrame() { return worldFrame_; }
  const std::string& getMapFrame() { return mapFrame_; }
  const std::string& getOdomFrame() { return odomFrame_; }
  const std::string& getBaseLinkFrame() { return baseLinkFrame_; }
  const std::string& getInitializationFrame() { return initializationFrame_; }

  // Functionality ------------------------------------------------------------
  virtual void findTransformations() = 0;

 protected:  // Members
  // General container class
  std::map<std::pair<std::string, std::string>, Eigen::Isometry3d> T_frame1_frame2_map_;

  // Return reference object
  Eigen::Isometry3d identityTransform_ = Eigen::Isometry3d::Identity();

  // Required frames
  std::string worldFrame_;
  std::string mapFrame_;
  std::string odomFrame_;
  std::string imuFrame_;
  std::string baseLinkFrame_;
  std::string initializationFrame_;
};

}  // namespace graph_msf

#endif  // STATIC_TRANSFORMS_H
