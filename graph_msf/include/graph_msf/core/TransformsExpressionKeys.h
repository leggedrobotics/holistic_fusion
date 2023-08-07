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

struct FactorGraphStateKey {
  // Constructor
  FactorGraphStateKey(const gtsam::Key& key, const double time, const bool atLeastOnceOptimized)
      : key_(key), time_(time), atLeastOnceOptimized_(atLeastOnceOptimized) {}
  FactorGraphStateKey() {}
  // Members
  gtsam::Key key_ = -1;
  double time_ = 0.0;
  bool atLeastOnceOptimized_ = false;
};

class TransformsExpressionKeys : public TransformsDictionary<FactorGraphStateKey> {
 public:
  // Constructor
  TransformsExpressionKeys() : TransformsDictionary<FactorGraphStateKey>(FactorGraphStateKey()) {
    std::cout << YELLOW_START << "StaticTransforms" << COLOR_END << " StaticTransforms instance created." << std::endl;
  }

  // Safe Modifiers
  // Returns
  bool newFramePairSafelyAddedToDictionary(gtsam::Key& returnKey, const std::string& frame1, const std::string& frame2,
                                           const double timeK) {
    // Check and modify content --> acquire lock
    FactorGraphStateKey factorGraphStateKey;
    std::lock_guard<std::mutex> lock(internalDictionaryModifierMutex_);
    // Logic
    if (isFramePairInDictionary(frame1, frame2)) {
      factorGraphStateKey = rv_T_frame1_frame2(frame1, frame2);
      returnKey = factorGraphStateKey.key_;
      return false;
    } else {
      // Create
      addNewFactorGraphStateKey(returnKey, frame1, frame2, timeK);
      return true;
    }
  }

  // Functionality ------------------------------------------------------------
  void addNewFactorGraphStateKey(gtsam::Key& returnKey, const std::string& frame1, const std::string& frame2, const double timeK) {
    returnKey = gtsam::symbol_shorthand::T(getNumberStoredTransformationPairs());
    FactorGraphStateKey factorGraphStateKey(returnKey, timeK, false);
    set_T_frame1_frame2(frame1, frame2, factorGraphStateKey);
  }

  std::mutex& mutex() { return externalModifierMutex_; }

 private:
  // Mutex
  std::mutex internalDictionaryModifierMutex_;
  std::mutex externalModifierMutex_;
};

}  // namespace graph_msf

#endif  // STATIC_TRANSFORMS_H
