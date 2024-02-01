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
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/expressions.h>

// Workspace
#include "graph_msf/core/TransformsDictionary.h"

namespace graph_msf {

class FactorGraphStateKey {
 public:
  // Constructor
  FactorGraphStateKey(const gtsam::Key& key, const double time, const int numberStepsOptimized,
                      const gtsam::Pose3& approximateTransformationBeforeOptimization)
      : key_(key),
        time_(time),
        numberStepsOptimized_(numberStepsOptimized),
        approximateTransformationBeforeOptimization_(approximateTransformationBeforeOptimization) {}
  FactorGraphStateKey() {}

  // Accessors
  const gtsam::Key& key() const { return key_; }
  int getNumberStepsOptimized() const { return numberStepsOptimized_; }
  double getTime() const { return time_; }
  void incrementNumberStepsOptimized() { ++numberStepsOptimized_; }
  const gtsam::Pose3& getApproximateTransformationBeforeOptimization() const { return approximateTransformationBeforeOptimization_; }

  // Setters
  void setTimeStamp(const double time) { time_ = time; }
  void setApproximateTransformationBeforeOptimization(const gtsam::Pose3& approximateTransformationBeforeOptimization) {
    approximateTransformationBeforeOptimization_ = approximateTransformationBeforeOptimization;
  }

 private:
  // Members
  gtsam::Key key_ = -1;
  double time_ = 0.0;
  int numberStepsOptimized_ = 0;
  gtsam::Pose3 approximateTransformationBeforeOptimization_ = gtsam::Pose3::Identity();
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
    // Make sure to handle both position and translation
    gtsam::Pose3 T_approximate;
    if constexpr (DIM == 3) {
      T_approximate = gtsam::Pose3::Identity();
      T_approximate.matrix().block<3, 1>(0, 3) = T_initial;
    } else if constexpr (DIM == 6) {
      T_approximate = T_initial;
    }
    // Retrieve key and insert information to map
    gtsam::Key T_key;
    std::lock_guard<std::mutex> modifyGraphKeysLock(this->mutex());
    // Case: The dynamically allocated key is not yet in the graph
    if (this->newFramePairSafelyAddedToDictionary<DIM>(T_key, frame1, frame2, timeK, T_approximate)) {  // Newly added to dictionary
      // Initial values for T_M_W
      potentialNewGraphValues.insert(T_key, T_initial);
      // Print to terminal
      REGULAR_COUT << GREEN_START << " Created new transform between frames " << frame1 << " and " << frame2 << " with key "
                   << gtsam::Symbol(T_key) << "." << COLOR_END << std::endl;
      // New factors to constrain new variable or regularize it
      priorFactors.emplace_back(T_key, T_initial, gtsam::noiseModel::Diagonal::Sigmas(1.0 * gtsam::Vector::Ones(DIM)));
    }

    // Return
    return gtsam::Expression<MEASUREMENT_TYPE>(T_key);
  }

  template <int DIM>
  bool newFramePairSafelyAddedToDictionary(gtsam::Key& returnKey, const std::string& frame1, const std::string& frame2, const double timeK,
                                           const gtsam::Pose3& approximateTransformation) {
    // Check and modify content --> acquire lock
    FactorGraphStateKey factorGraphStateKey;
    std::lock_guard<std::mutex> lock(internalDictionaryModifierMutex_);
    // Logic
    if (isFramePairInDictionary(frame1, frame2)) {
      factorGraphStateKey = rv_T_frame1_frame2(frame1, frame2);
      lv_T_frame1_frame2(frame1, frame2).setTimeStamp(timeK);
      lv_T_frame1_frame2(frame1, frame2).setApproximateTransformationBeforeOptimization(approximateTransformation);
      returnKey = factorGraphStateKey.key();
      return false;
    } else {
      // Create
      addNewFactorGraphStateKey<DIM>(returnKey, frame1, frame2, timeK, approximateTransformation);
      return true;
    }
  }

  // Functionality ------------------------------------------------------------
  template <int DIM>
  void addNewFactorGraphStateKey(gtsam::Key& returnKey, const std::string& frame1, const std::string& frame2, const double timeK,
                                 const gtsam::Pose3& approximateTransformation) {
    // Assert
    static_assert(DIM == 3 || DIM == 6, "DIM must be either 3 or 6.");
    // Logic
    if constexpr (DIM == 6) {
      returnKey = gtsam::symbol_shorthand::T(getNumberStoredTransformationPairs());
    } else if constexpr (DIM == 3) {
      returnKey = gtsam::symbol_shorthand::D(getNumberStoredTransformationPairs());
    }
    FactorGraphStateKey factorGraphStateKey(returnKey, timeK, 0, approximateTransformation);
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
