/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GTSAMEXPRESSIONTRANSFORMS_H
#define GTSAMEXPRESSIONTRANSFORMS_H

// GTSAM
#include <gtsam/slam/expressions.h>

// Workspace
#include "graph_msf/core/TransformsDictionary.h"

namespace graph_msf {

class TransformsExpressionKeys : public TransformsDictionary<gtsam::Key> {
 public:
  // Constructor
  TransformsExpressionKeys() : TransformsDictionary<gtsam::Key>(-1) {
    std::cout << YELLOW_START << "StaticTransforms" << COLOR_END << " StaticTransforms instance created." << std::endl;
  }

  // Safe Modifiers
  // Returns
  bool newFramePairSafelyAddedToDictionary(const std::string& frame1, const std::string& frame2) {
    // Check and modify content --> acquire lock
    std::lock_guard<std::mutex> lock(dictionaryModifierMutex_);
    // Logic
    if (isFramePairInDictionary(frame1, frame2)) {
      return false;
    } else {
      // Create
      addNewExpressionTransformation(frame1, frame2);
      return true;
    }
  }

  // Functionality ------------------------------------------------------------
  void addNewExpressionTransformation(const std::string& frame1, const std::string& frame2) {
    set_T_frame1_frame2(frame1, frame2, gtsam::symbol_shorthand::T(getNumberStoredTransformationPairs()));
  }

 private:
  // Mutex
  std::mutex dictionaryModifierMutex_;
};

}  // namespace graph_msf

#endif  // STATIC_TRANSFORMS_H
