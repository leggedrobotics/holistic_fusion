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
#include <map>

// Workspace
#include "graph_msf/interface/Terminal.h"

namespace graph_msf {

template <class TRANSFORM_TYPE>
class TransformsDictionary {
 public:
  // Constructor
  TransformsDictionary(TRANSFORM_TYPE identityObject) : identity_(identityObject) {}

  // Getters ------------------------------------------------------------
  // Check for specific transformation pair
  bool isFramePairInDictionary(const std::string& frame1, const std::string& frame2) {
    std::pair<std::string, std::string> framePair(frame1, frame2);
    auto keyIterator = T_frame1_frame2_map_.find(framePair);
    if (keyIterator == T_frame1_frame2_map_.end()) {
      return false;
    } else {
      return true;
    }
  }

  bool removeTransform(const std::string& frame1, const std::string& frame2) {
    std::pair<std::string, std::string> framePair(frame1, frame2);
    auto keyIterator = T_frame1_frame2_map_.find(framePair);
    if (keyIterator == T_frame1_frame2_map_.end()) {
      return false;
    } else {
      T_frame1_frame2_map_.erase(keyIterator);
      return true;
    }
  }

  // Number of transformations
  size_t getNumberStoredTransformationPairs() { return numStoredTransforms_; }

  // Returns a left value of the requested transformation
  TRANSFORM_TYPE& lv_T_frame1_frame2(const std::string& frame1, const std::string& frame2) {
    if (frame1 == "") {
      std::cout << YELLOW_START << "GMsf-TransformsDict" << COLOR_END << " No frame1 given." << std::endl;
      throw std::runtime_error("No frame1 given.");
    } else if (frame2 == "") {
      std::cout << YELLOW_START << "GMsf-TransformsDict" << COLOR_END << " No frame2 given." << std::endl;
      throw std::runtime_error("No frame2 given.");
    } else {
      std::pair<std::string, std::string> framePair(frame1, frame2);
      return T_frame1_frame2_map_[framePair];
    }
  }

  // Returns a right value to the requested transformation
  const TRANSFORM_TYPE& rv_T_frame1_frame2(const std::string& frame1, const std::string& frame2) const {
    // Check whether it is identity
    if (frame1 == frame2) {
      return identity_;
    }
    // Normal Operation
    std::pair<std::string, std::string> framePair(frame1, frame2);
    auto keyIterator = T_frame1_frame2_map_.find(framePair);
    if (keyIterator == T_frame1_frame2_map_.end()) {
      std::cout << YELLOW_START << "GMsf-TransformsDict" << COLOR_END << " No transform found between " << frame1 << " and " << frame2
                << "." << std::endl;
      throw std::runtime_error("No transform found for " + frame1 + " and " + frame2 + ".");
    }
    return keyIterator->second;
  }

  // Return reference to the map
  std::map<std::pair<std::string, std::string>, TRANSFORM_TYPE>& getTransformsMap() { return T_frame1_frame2_map_; }
  const std::map<std::pair<std::string, std::string>, TRANSFORM_TYPE>& getTransformsMap() const { return T_frame1_frame2_map_; }

  // Setters ------------------------------------------------------------
  // With inverse
  void set_T_frame1_frame2_andInverse(const std::string& frame1, const std::string& frame2, const TRANSFORM_TYPE& T_frame1_frame2) {
    // Check whether transformation pair is already there
    if (!isFramePairInDictionary(frame1, frame2)) {
      lv_T_frame1_frame2(frame1, frame2) = T_frame1_frame2;
      lv_T_frame1_frame2(frame2, frame1) = rv_T_frame1_frame2(frame1, frame2).inverse();
      ++numStoredTransforms_;
    } else {
      std::cout << YELLOW_START << "GMsf-TransformsDict" << COLOR_END << " Transformation pair " << frame1 << " and " << frame2
                << " already exists. Not adding it to the transforms." << std::endl;
    }
  }

  // Without inverse
  void set_T_frame1_frame2(const std::string& frame1, const std::string& frame2, const TRANSFORM_TYPE T_frame1_frame2) {
    // Check whether transformation pair is already there
    if (!isFramePairInDictionary(frame1, frame2)) {
      ++numStoredTransforms_;
    }
    // Set transformation
    lv_T_frame1_frame2(frame1, frame2) = T_frame1_frame2;
  }

 private:
  // General container class
  std::map<std::pair<std::string, std::string>, TRANSFORM_TYPE> T_frame1_frame2_map_;

  // Identity transformation
  TRANSFORM_TYPE identity_;

  // Number of stored transformations
  size_t numStoredTransforms_ = 0;
};

}  // namespace graph_msf

#endif  // TRANSFORMS_DICTIONARY_H
