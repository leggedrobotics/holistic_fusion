/*
Copyright 2024 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#ifndef GTSAMEXPRESSIONTRANSFORMS_H
#define GTSAMEXPRESSIONTRANSFORMS_H

// Output
#define REGULAR_COUT std::cout << YELLOW_START << "GMSF-TransformExpressionKeys" << COLOR_END

// Workspace
#include "graph_msf/config/AdmissibleGtsamSymbols.h"
#include "graph_msf/core/FactorGraphStateKey.h"
#include "graph_msf/core/TransformsDictionary.h"

namespace graph_msf {

// Explanation:
// 1) global variables are constant along the entire operation, hence they are not removed and will get the same gtsam key upon return
// 2) reference frames can change, but are modelled according to random walk, hence they are only removed once a successor is initialized
// 3) landmarks are initialized and removed as needed, hence they are removed once they are not needed anymore
// Frames that are not needed but also not removed are set to be "inactive"

class TransformsExpressionKeys : public TransformsDictionary<FactorGraphStateKey> {
 public:
  // Constructor
  TransformsExpressionKeys() : TransformsDictionary<FactorGraphStateKey>(FactorGraphStateKey()) {
    REGULAR_COUT << " Instance created." << std::endl;
  }

  // Destructor
  ~TransformsExpressionKeys() = default;

  // Cleanup
  bool removeTransform(const std::string& frame1, const std::string& frame2, FactorGraphStateKey& returnRemovedKey) override {
    // Removed main element
    bool removedTransformFlag = TransformsDictionary<FactorGraphStateKey>::removeTransform(frame1, frame2, returnRemovedKey);
    // If found, we also remove the key from the key-to-frame pair map
    if (removedTransformFlag) {
      auto keyFramePairMapIterator = transformGtsamKeyToFramePairMap_.find(returnRemovedKey.key());
      if (keyFramePairMapIterator != transformGtsamKeyToFramePairMap_.end()) {
        transformGtsamKeyToFramePairMap_.erase(keyFramePairMapIterator);
      } else {
        REGULAR_COUT << RED_START << " Key " << gtsam::Symbol(returnRemovedKey.key()) << " not found in map." << COLOR_END << std::endl;
        throw std::runtime_error("Key not found in map.");
      }
    }
    return removedTransformFlag;
  }

  // Same but with only two arguments
  bool removeTransform(const std::string& frame1, const std::string& frame2) {
    FactorGraphStateKey removedKeyPlaceholder;
    return removeTransform(frame1, frame2, removedKeyPlaceholder);
  }

  // Getters ------------------------------------------------------------
  // Get transform to frame pair map
  bool getFramePairFromGtsamKey(std::pair<std::string, std::string>& framePairRef, const gtsam::Key& gtsamKey) const {
    // Check if key is in map
    auto keyFramePairMapIterator = transformGtsamKeyToFramePairMap_.find(gtsamKey);
    if (keyFramePairMapIterator == transformGtsamKeyToFramePairMap_.end()) {
      return false;
    }
    // Retrieve frame pair
    framePairRef = keyFramePairMapIterator->second;
    return true;
  }

  // Get transform to frame pair map
  [[nodiscard]] const std::map<gtsam::Key, std::pair<std::string, std::string>>& getTransformGtsamKeyToFramePairMap() const {
    return transformGtsamKeyToFramePairMap_;
  }

  // Returns of key
  template <char SYMBOL_CHAR>
  FactorGraphStateKey getTransformationKey(bool& newGraphKeyAdded, const std::string& frame1, const std::string& frame2, const double timeK,
                                           const gtsam::Pose3& approximateTransformationBeforeOptimization,
                                           const VariableType& variableType) {
    // Retrieve key and insert information to map
    FactorGraphStateKey stateKey;
    std::lock_guard<std::mutex> modifyGraphKeysLock(this->mutex());
    // Case: The dynamically allocated key is not yet in the graph
    newGraphKeyAdded = this->addNewFramePairSafelyToDictionary<SYMBOL_CHAR>(stateKey, frame1, frame2, timeK,
                                                                            approximateTransformationBeforeOptimization, variableType);
    return stateKey;
  }

  // Safe addition of new frame pair to dictionary --> checks whether already present
  template <char SYMBOL_CHAR>
  bool addNewFramePairSafelyToDictionary(FactorGraphStateKey& returnStateKey, const std::string& frame1, const std::string& frame2,
                                         const double timeK, const gtsam::Pose3& approximateTransformationBeforeOptimization,
                                         const VariableType& variableType) {
    // Check and modify content --> acquire lock
    std::lock_guard<std::mutex> lock(internalDictionaryModifierMutex_);

    // Logic
    // CASE 1: Frame pair is already in dictionary, hence keyframe is not new
    if (isFramePairInDictionary(frame1, frame2)) {
      FactorGraphStateKey& factorGraphStateKey = lv_T_frame1_frame2(frame1, frame2);
      // Update Timestamp and approximate transformation if newer
      if (timeK > factorGraphStateKey.getTime()) {
        factorGraphStateKey.setTimeStamp(timeK);
        factorGraphStateKey.setApproximateTransformationBeforeOptimization(approximateTransformationBeforeOptimization);
      }
      returnStateKey = factorGraphStateKey;
      return false;
    }
    // CASE 2: Frame pair is not in dictionary --> Newly added
    else {
      returnStateKey =
          addNewFactorGraphStateKey<SYMBOL_CHAR>(frame1, frame2, timeK, approximateTransformationBeforeOptimization, variableType);
      return true;
    }
  }

  // Functionality ------------------------------------------------------------
  template <char SYMBOL_CHAR>
  FactorGraphStateKey addNewFactorGraphStateKey(const std::string& frame1, const std::string& frame2, const double timeK,
                                                const gtsam::Pose3& approximateTransformationBeforeOptimization,
                                                const VariableType& variableType) {
    // Compile Time Checks and Variables
    checkSymbol<SYMBOL_CHAR>();
    constexpr int symbolIndex = getSymbolIndex<SYMBOL_CHAR>();
    static_assert(symbolIndex >= 0, "Symbol not found in admissible symbols.");

    // Create new key
    gtsam::Key gtsamKey = gtsam::Symbol(SYMBOL_CHAR, numStoredTransformsPerLetter_[symbolIndex]);
    REGULAR_COUT << GREEN_START << " New key " << gtsam::Symbol(gtsamKey) << " created for frame pair " << frame1 << " and " << frame2
                 << COLOR_END << std::endl;
    // Add to main dictionary
    FactorGraphStateKey factorGraphStateKey(gtsamKey, timeK, 0, approximateTransformationBeforeOptimization, variableType);
    set_T_frame1_frame2(frame1, frame2, factorGraphStateKey);

    // Increase the counter of specific symbol
    ++numStoredTransformsPerLetter_[symbolIndex];

    // Add to key-to-frame pair map
    transformGtsamKeyToFramePairMap_[gtsamKey] = std::make_pair(frame1, frame2);

    // Return
    return factorGraphStateKey;
  }

  std::mutex& mutex() { return externalModifierMutex_; }

 private:
  // Mapping in other direction --> from key to frame pair
  std::map<gtsam::Key, std::pair<std::string, std::string>> transformGtsamKeyToFramePairMap_;

  // Mutex
  std::mutex internalDictionaryModifierMutex_;
  std::mutex externalModifierMutex_;

  // Num Stored Transforms Per Letter --> compile time initialization, set all to zero
  std::array<unsigned long, sizeof(ADMISSIBLE_DYNAMIC_SYMBOL_CHARS)> numStoredTransformsPerLetter_ =
      makeValuedArray<unsigned long, sizeof(ADMISSIBLE_DYNAMIC_SYMBOL_CHARS)>(0);
};

}  // namespace graph_msf

#endif  // STATIC_TRANSFORMS_H
