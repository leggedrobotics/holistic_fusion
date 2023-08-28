/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GTSAMEXPRESSIONTRANSFORMS_H
#define GTSAMEXPRESSIONTRANSFORMS_H

// Output
#define REGULAR_COUT std::cout << YELLOW_START << "GMSF-TransformExpressionKeys" << COLOR_END

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
    REGULAR_COUT << " Instance created." << std::endl;
  }

  // Safe Modifiers
  // Returns
  template <class MEASUREMENT_TYPE, int DIM>
  gtsam::Expression<MEASUREMENT_TYPE> getTransformationExpression(gtsam::Values& potentialNewGraphValues,
                                                                  std::vector<gtsam::PriorFactor<MEASUREMENT_TYPE>>& priorFactors,
                                                                  const MEASUREMENT_TYPE& T_initial, const std::string& frame1,
                                                                  const std::string& frame2, const double timeK) {
    gtsam::Key T_key;
    std::lock_guard<std::mutex> modifyGraphKeysLock(this->mutex());
    // Case: The dynamically allocated key is not yet in the graph
    if (this->newFramePairSafelyAddedToDictionary<DIM>(T_key, frame1, frame2, timeK)) {  // Newly added to dictionary
      // Initial values for T_M_W
      potentialNewGraphValues.insert(T_key, T_initial);
      // Print to terminal
      REGULAR_COUT << GREEN_START << " Created new transform between frames " << frame1 << " and " << frame2 << " with key "
                   << gtsam::Symbol(T_key) << "." << COLOR_END << std::endl;
      // New factors to constrain new variable or regularize it
      priorFactors.emplace_back(T_key, T_initial, gtsam::noiseModel::Diagonal::Sigmas(1.0e-03 * gtsam::Vector::Ones(DIM)));
    }

    // Return
    return gtsam::Expression<MEASUREMENT_TYPE>(T_key);
  }

  template <int DIM>
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
      addNewFactorGraphStateKey<DIM>(returnKey, frame1, frame2, timeK);
      return true;
    }
  }

  // Functionality ------------------------------------------------------------
  template <int DIM>
  void addNewFactorGraphStateKey(gtsam::Key& returnKey, const std::string& frame1, const std::string& frame2, const double timeK) {
    // Assert
    static_assert(DIM == 3 || DIM == 6, "DIM must be either 3 or 6.");
    // Logic
    if constexpr (DIM == 6) {
      returnKey = gtsam::symbol_shorthand::T(getNumberStoredTransformationPairs());
    } else if constexpr (DIM == 3) {
      returnKey = gtsam::symbol_shorthand::D(getNumberStoredTransformationPairs());
    }
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
