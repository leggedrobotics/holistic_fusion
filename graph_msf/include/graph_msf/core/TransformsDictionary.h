/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef TRANSFORMS_DICTIONARY_H
#define TRANSFORMS_DICTIONARY_H

// C++
#include <iostream>

// Workspace
#include "graph_msf/interface/Terminal.h"

namespace graph_msf {

template <class TRANSFORM_TYPE>
class TransformsDictionary {
 public:
  // Constructor
  TransformsDictionary(const TRANSFORM_TYPE& identityTransform) : identityTransform_(identityTransform) {
    std::cout << YELLOW_START << "GMsf" << COLOR_END << " TransformsDictionary instance created." << std::endl;
  }

  // Setters ------------------------------------------------------------
  void set_T_frame1_frame2_andInverse(const std::string& frame1, const std::string& frame2, const TRANSFORM_TYPE& T_frame1_frame2) {
    lv_T_frame1_frame2(frame1, frame2) = T_frame1_frame2;
    lv_T_frame1_frame2(frame2, frame1) = rv_T_frame1_frame2(frame1, frame2).inverse();
  }

  // Getters ------------------------------------------------------------
  // Returns a left value of the requested transformation
  TRANSFORM_TYPE& lv_T_frame1_frame2(const std::string& frame1, const std::string& frame2) {
    if (frame1 == frame2) {
      identityTransformCopy_ = identityTransform_;
      return identityTransformCopy_;
    } else if (frame1 == "") {
      std::cout << YELLOW_START << "StaticTransforms" << COLOR_END << " No frame1 given." << std::endl;
      throw std::runtime_error("No frame1 given.");
    } else if (frame2 == "") {
      std::cout << YELLOW_START << "StaticTransforms" << COLOR_END << " No frame2 given." << std::endl;
      throw std::runtime_error("No frame2 given.");
    } else {
      std::pair<std::string, std::string> framePair(frame1, frame2);
      return T_frame1_frame2_map_[framePair];
    }
  }

  // Returns a right value to the requested transformation
  const TRANSFORM_TYPE& rv_T_frame1_frame2(const std::string& frame1, const std::string& frame2) {
    if (frame1 == frame2) {
      return identityTransform_;
    } else {
      std::pair<std::string, std::string> framePair(frame1, frame2);
      auto keyIterator = T_frame1_frame2_map_.find(framePair);
      if (keyIterator == T_frame1_frame2_map_.end()) {
        std::cout << YELLOW_START << "StaticTransforms" << COLOR_END << " No transform found for " << frame1 << " and " << frame2 << "."
                  << std::endl;
        throw std::runtime_error("No transform found for " + frame1 + " and " + frame2 + ".");
      }
      return keyIterator->second;
    }
  }

 protected:  // Members
  // General container class
  std::map<std::pair<std::string, std::string>, TRANSFORM_TYPE> T_frame1_frame2_map_;

  // Return reference object
  const TRANSFORM_TYPE identityTransform_;
  TRANSFORM_TYPE identityTransformCopy_;
};

}  // namespace graph_msf

#endif  // TRANSFORMS_DICTIONARY_H
