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
#include "graph_msf/core/TransformsDictionary.h"

namespace graph_msf {

class FactorGraphStateKey {
 public:
  // Constructor
  FactorGraphStateKey(const gtsam::Key& key, const double time, const int numberStepsOptimized,
                      const gtsam::Pose3& approximateTransformationBeforeOptimization, const Eigen::Vector3d& measurementKeyframePosition)
      : key_(key),
        time_(time),
        numberStepsOptimized_(numberStepsOptimized),
        approximateTransformationBeforeOptimization_(approximateTransformationBeforeOptimization),
        measurementKeyframePosition_(measurementKeyframePosition) {}
  FactorGraphStateKey() {}

  // Accessors
  const gtsam::Key key() const { return key_; }
  int getNumberStepsOptimized() const { return numberStepsOptimized_; }
  double getTime() const { return time_; }
  [[nodiscard]] gtsam::Pose3 getApproximateTransformationBeforeOptimization() const { return approximateTransformationBeforeOptimization_; }
  [[nodiscard]] Eigen::Vector3d getMeasurementKeyframePosition() const { return measurementKeyframePosition_; }

  // Setters
  void setTimeStamp(const double time) { time_ = time; }
  void incrementNumberStepsOptimized() { ++numberStepsOptimized_; }
  void setApproximateTransformationBeforeOptimization(const gtsam::Pose3& approximateTransformationBeforeOptimization) {
    approximateTransformationBeforeOptimization_ = approximateTransformationBeforeOptimization;
  }
  void setMeasurementKeyframePosition(const Eigen::Vector3d& measurementKeyframePosition) {
    measurementKeyframePosition_ = measurementKeyframePosition;
  }

 private:
  // Members
  gtsam::Key key_ = -1;
  double time_ = 0.0;
  int numberStepsOptimized_ = 0;
  gtsam::Pose3 approximateTransformationBeforeOptimization_ = gtsam::Pose3();
  Eigen::Vector3d measurementKeyframePosition_ = Eigen::Vector3d::Zero();
};

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
      auto keyFramePairMapIterator = transformKeyToFramePairMap_.find(returnRemovedKey.key());
      if (keyFramePairMapIterator != transformKeyToFramePairMap_.end()) {
        transformKeyToFramePairMap_.erase(keyFramePairMapIterator);
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
  bool getFramePairFromGtsamKey(std::pair<std::string, std::string>& framePairRef, const gtsam::Key& key) const {
    // Check if key is in map
    auto keyFramePairMapIterator = transformKeyToFramePairMap_.find(key);
    if (keyFramePairMapIterator == transformKeyToFramePairMap_.end()) {
      return false;
    }
    // Retrieve frame pair
    framePairRef = keyFramePairMapIterator->second;
    return true;
  }

  // Get transform to frame pair map
  [[nodiscard]] const std::map<gtsam::Key, std::pair<std::string, std::string>>& getTransformKeyToFramePairMap() const {
    return transformKeyToFramePairMap_;
  }

  // Returns of key
  typedef gtsam::Key (*F)(std::uint64_t);
  template <char SYMBOL_CHAR>
  gtsam::Key getTransformationKey(bool& newGraphKeyAdded, Eigen::Vector3d& modifiedMeasurementKeyframePosition, const std::string& frame1,
                                  const std::string& frame2, const double timeK,
                                  const gtsam::Pose3& approximateTransformationBeforeOptimization,
                                  const bool centerMeasurementsAtRobotPositionBeforeAlignment) {
    // Retrieve key and insert information to map
    gtsam::Key T_key;
    std::lock_guard<std::mutex> modifyGraphKeysLock(this->mutex());
    // Case: The dynamically allocated key is not yet in the graph
    newGraphKeyAdded = this->addNewFramePairSafelyToDictionary<SYMBOL_CHAR>(T_key, modifiedMeasurementKeyframePosition, frame1, frame2,
                                                                            timeK, approximateTransformationBeforeOptimization,
                                                                            centerMeasurementsAtRobotPositionBeforeAlignment);

    // Return
    return T_key;
  }

  // Overloaded function (without keyframe centering), e.g. for extrinsic calibration
  template <char SYMBOL_CHAR>
  gtsam::Key getTransformationKey(bool& newGraphKeyAdded, const std::string& frame1, const std::string& frame2, const double timeK,
                                  const gtsam::Pose3& approximateTransformationBeforeOptimization) {
    Eigen::Vector3d keyframePositionPlaceholder = Eigen::Vector3d::Zero();  // Placeholder
    return getTransformationKey<SYMBOL_CHAR>(newGraphKeyAdded, keyframePositionPlaceholder, frame1, frame2, timeK,
                                             approximateTransformationBeforeOptimization, false);
  }

  // Safe addition of new frame pair to dictionary --> checks whether already present
  template <char SYMBOL_CHAR>
  bool addNewFramePairSafelyToDictionary(gtsam::Key& returnKey, Eigen::Vector3d& modifiedMeasurementKeyframePosition,
                                         const std::string& frame1, const std::string& frame2, const double timeK,
                                         const gtsam::Pose3& approximateTransformationBeforeOptimization,
                                         const bool centerMeasurementsAtRobotPositionBeforeAlignment) {
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
      returnKey = factorGraphStateKey.key();
      // Get Keyframe position from before (not changed here)
      modifiedMeasurementKeyframePosition = factorGraphStateKey.getMeasurementKeyframePosition();
      return false;
    }
    // CASE 2: Frame pair is not in dictionary --> Newly added
    else {
      // If we do not want to move origin --> set to zero
      if (!centerMeasurementsAtRobotPositionBeforeAlignment) {
        modifiedMeasurementKeyframePosition = Eigen::Vector3d::Zero();
      }  // else: updatedMeasurementOrigin is kept from the function that calls this function
      returnKey = addNewFactorGraphStateKey<SYMBOL_CHAR>(frame1, frame2, timeK, approximateTransformationBeforeOptimization,
                                                         modifiedMeasurementKeyframePosition);

      // Return
      return true;
    }
  }

  // Functionality ------------------------------------------------------------
  template <char SYMBOL_CHAR>
  gtsam::Key addNewFactorGraphStateKey(const std::string& frame1, const std::string& frame2, const double timeK,
                                       const gtsam::Pose3& approximateTransformationBeforeOptimization,
                                       const Eigen::Vector3d& measurementKeyframePosition) {
    // Compile Time Checks and Variables
    checkSymbol<SYMBOL_CHAR>();
    constexpr int symbolIndex = getSymbolIndex<SYMBOL_CHAR>();
    static_assert(symbolIndex >= 0, "Symbol not found in admissible symbols.");

    // Create new key
    gtsam::Key returnKey = gtsam::Symbol(SYMBOL_CHAR, numStoredTransformsPerLetter_[symbolIndex]);
    REGULAR_COUT << GREEN_START << " New key " << gtsam::Symbol(returnKey) << " created for frame pair " << frame1 << " and " << frame2
                 << COLOR_END << std::endl;
    // Add to main dictionary
    FactorGraphStateKey factorGraphStateKey(returnKey, timeK, 0, approximateTransformationBeforeOptimization, measurementKeyframePosition);
    set_T_frame1_frame2(frame1, frame2, factorGraphStateKey);

    // Increase the counter of specific symbol
    ++numStoredTransformsPerLetter_[symbolIndex];

    // Add to key-to-frame pair map
    transformKeyToFramePairMap_[returnKey] = std::make_pair(frame1, frame2);

    // Return
    return returnKey;
  }

  std::mutex& mutex() { return externalModifierMutex_; }

 private:
  // Mapping in other direction --> from key to frame pair
  std::map<gtsam::Key, std::pair<std::string, std::string>> transformKeyToFramePairMap_;

  // Mutex
  std::mutex internalDictionaryModifierMutex_;
  std::mutex externalModifierMutex_;

  // Num Stored Transforms Per Letter --> compile time initialization, set all to zero
  std::array<unsigned long, sizeof(ADMISSIBLE_DYNAMIC_SYMBOL_CHARS)> numStoredTransformsPerLetter_ =
      makeValuedArray<unsigned long, sizeof(ADMISSIBLE_DYNAMIC_SYMBOL_CHARS)>(0);
};

}  // namespace graph_msf

#endif  // STATIC_TRANSFORMS_H
